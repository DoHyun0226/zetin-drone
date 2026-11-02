"""Offline decoder tests for the batched 1 kHz dual-IMU binary stream."""

import contextlib
import csv
import importlib.util
import io
import pathlib
import struct
import sys
import tempfile
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
DECODER_PATH = REPO_ROOT / "scripts" / "ground" / "imu_raw.py"
ZIMU_HEADER = struct.Struct("<4sBBHIII")
ZIMU_SAMPLE = struct.Struct("<H12h")
ZCAL_PACKET = struct.Struct("<4sB3x13f")


def _zimu(seq, dropped, samples, base_t_us=100_000, version=1):
    return (
        ZIMU_HEADER.pack(
            b"ZIMU", version, len(samples), 0, seq, base_t_us, dropped
        )
        + b"".join(ZIMU_SAMPLE.pack(*sample) for sample in samples)
    )


def _zcal(
    *,
    gyro_bias1=(1.0, 2.0, 3.0),
    gyro_bias2=(4.0, 5.0, 6.0),
    accel_scale1=0.5,
    accel_scale2=0.25,
    gyro_scale=0.1,
    accel_scale=0.01,
    imu2_sign=(-1.0, 1.0, -1.0),
):
    return ZCAL_PACKET.pack(
        b"ZCAL",
        1,
        *gyro_bias1,
        *gyro_bias2,
        accel_scale1,
        accel_scale2,
        gyro_scale,
        accel_scale,
        *imu2_sign,
    )


class DecodeImuRawTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            "_decode_imu_raw_under_test", DECODER_PATH
        )
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        cls.decoder = module

    @classmethod
    def tearDownClass(cls):
        sys.modules.pop(cls.decoder.__name__, None)

    def _decode(self, payload):
        temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(temp_dir.cleanup)
        input_path = pathlib.Path(temp_dir.name) / "capture.bin"
        output_path = pathlib.Path(temp_dir.name) / "capture.csv"
        input_path.write_bytes(payload)
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            summary = self.decoder.decode_file(input_path, output_path)
        with output_path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            rows = list(reader)
            fieldnames = reader.fieldnames
        return summary, rows, fieldnames, stderr.getvalue()

    def test_batch_round_trip_preserves_raw_values_and_reconstructs_timestamps(self):
        samples = [
            (1000, 1, -2, 3, -4, 5, -6, 7, -8, 9, -10, 11, -12),
            (1250, 101, -102, 103, -104, 105, -106,
             107, -108, 109, -110, 111, -112),
        ]

        summary, rows, _, _ = self._decode(
            _zimu(8, 0, samples, base_t_us=200_000)
        )

        self.assertEqual(summary.total_batches, 1)
        self.assertEqual(summary.total_samples, 2)
        self.assertEqual(rows[0]["sample_idx"], "0")
        self.assertEqual(rows[0]["t_us"], "200000")
        self.assertEqual(rows[0]["imu1_gyro_x_raw"], "1")
        self.assertEqual(rows[0]["imu2_accel_z_raw"], "-12")
        self.assertEqual(rows[1]["sample_idx"], "1")
        self.assertEqual(rows[1]["t_us"], "201250")
        self.assertEqual(rows[1]["imu1_accel_x_raw"], "-104")
        self.assertEqual(rows[1]["imu2_gyro_y_raw"], "-108")

    def test_batch_sequence_gap_reports_exact_wireless_loss(self):
        sample = (1000,) + tuple(range(12))

        summary, _, _, stderr = self._decode(
            _zimu(10, 0, [sample]) + _zimu(13, 0, [sample])
        )

        self.assertEqual(summary.lost_batches, 2)
        self.assertIn("wireless_lost_batches=2", stderr)

    def test_last_producer_drop_count_is_reported(self):
        sample = (1000,) + tuple(range(12))

        summary, _, _, stderr = self._decode(
            _zimu(1, 3, [sample]) + _zimu(2, 9, [sample])
        )

        self.assertEqual(summary.producer_dropped, 9)
        self.assertIn("producer_dropped=9", stderr)

    def test_missing_zcal_outputs_only_raw_columns_and_warns(self):
        sample = (1000,) + tuple(range(12))

        _, _, fieldnames, stderr = self._decode(_zimu(1, 0, [sample]))

        self.assertIn("imu1_gyro_x_raw", fieldnames)
        self.assertNotIn("imu1_gyro_x_dps", fieldnames)
        self.assertIn("WARNING", stderr)
        self.assertIn("ZCAL", stderr)

    def test_zcal_applies_bias_scale_and_imu2_sign_in_imu1_sensor_frame(self):
        sample = (
            1000,
            20, 40, 60,
            100, 200, 300,
            80, 100, 120,
            400, 500, 600,
        )

        # ZCAL can arrive after early batches if the first calibration
        # datagram is lost; per-boot constants still recover those rows.
        _, rows, fieldnames, stderr = self._decode(
            _zimu(1, 0, [sample]) + _zcal()
        )

        self.assertIn("imu1_gyro_x_dps", fieldnames)
        self.assertNotIn("WARNING", stderr)
        row = rows[0]
        self.assertAlmostEqual(float(row["imu1_gyro_x_dps"]), 1.0, places=6)
        self.assertAlmostEqual(float(row["imu1_gyro_y_dps"]), 2.0, places=6)
        self.assertAlmostEqual(float(row["imu1_accel_x_g"]), 0.5, places=6)
        self.assertAlmostEqual(float(row["imu2_gyro_x_dps"]), -12.0, places=6)
        self.assertAlmostEqual(float(row["imu2_gyro_y_dps"]), 5.0, places=6)
        self.assertAlmostEqual(float(row["imu2_accel_x_g"]), -1.0, places=6)
        self.assertAlmostEqual(float(row["imu2_accel_z_g"]), -1.5, places=6)

    def test_truncated_unknown_and_unsupported_records_are_skipped_and_counted(self):
        sample = (1000,) + tuple(range(12))
        truncated = ZIMU_HEADER.pack(
            b"ZIMU", 1, 2, 0, 20, 10_000, 0
        ) + ZIMU_SAMPLE.pack(*sample)
        # v2 는 이제 지원된다. 미지원 케이스는 알 수 없는 버전으로 만든다.
        unsupported = _zimu(21, 0, [sample], version=3)
        valid = _zimu(22, 4, [sample])

        summary, rows, _, stderr = self._decode(
            b"NOPEbad-datagram" + truncated + unsupported + valid
        )

        self.assertEqual(len(rows), 1)
        self.assertEqual(summary.total_batches, 1)
        self.assertEqual(summary.truncated_records, 1)
        self.assertEqual(summary.unknown_magic_records, 1)
        self.assertEqual(summary.unsupported_version_records, 1)
        self.assertIn("truncated=1", stderr)
        self.assertIn("unknown_magic=1", stderr)
        self.assertIn("unsupported_version=1", stderr)


if __name__ == "__main__":
    unittest.main()
