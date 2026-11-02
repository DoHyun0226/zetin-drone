"""Tests for the scripts/drone_logs.py log-analysis entry point."""

import csv
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
DRONE_LOGS = SCRIPTS_DIR / "drone_logs.py"
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ground import imu_raw, probe_response  # noqa: E402
from test_decode_imu_raw import _zcal, _zimu  # noqa: E402


def run(args, **kwargs):
    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"
    return subprocess.run(
        [str(a) for a in args],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
        check=kwargs.pop("check", False),
        **kwargs,
    )


class DroneLogsTest(unittest.TestCase):
    def test_help_lists_four_subcommands(self):
        out = run([sys.executable, DRONE_LOGS, "--help"], check=True)
        for sub in ("flight", "imuraw", "probe", "magcal"):
            self.assertIn(sub, out.stdout)

    def test_unknown_subcommand_fails(self):
        self.assertNotEqual(
            run([sys.executable, DRONE_LOGS, "nope"]).returncode, 0
        )

    def test_imuraw_matches_decoder(self):
        samples = [(1000,) + tuple(range(1, 13)), (1250,) + tuple(range(13, 25))]
        with tempfile.TemporaryDirectory() as temp:
            tmp = Path(temp)
            bin_path = tmp / "cap.bin"
            bin_path.write_bytes(_zcal() + _zimu(0, 0, samples))
            run(
                [sys.executable, DRONE_LOGS, "imuraw", bin_path, tmp / "a.csv"],
                check=True,
            )
            imu_raw.decode_file(bin_path, tmp / "b.csv")
            self.assertEqual(
                (tmp / "a.csv").read_bytes(), (tmp / "b.csv").read_bytes()
            )

    def test_flight_no_plot_runs_on_fixture(self):
        fields = ("Timestamp", "Mixer_RP_Scale", "Mixer_Yaw_Scale")
        with tempfile.TemporaryDirectory() as temp:
            csv_path = Path(temp) / "flight.csv"
            with csv_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow(
                    dict(zip(fields, ("00:00:00.000", 1.0, 1.0)))
                )
            out = run(
                [sys.executable, DRONE_LOGS, "flight", csv_path, "--no-plot"]
            )
        self.assertEqual(0, out.returncode, out.stderr)

    def test_flight_default_picks_latest_flight_log_not_camera_csv(self):
        from ground import flight_log

        with tempfile.TemporaryDirectory() as temp:
            tmp = Path(temp)
            flight = tmp / "flight_log_A.csv"
            camera = tmp / "camera_B.csv"
            flight.write_text("x\n")
            camera.write_text("x\n")
            now = time.time()
            os.utime(flight, (now - 100, now - 100))
            os.utime(camera, (now, now))
            original = flight_log.LOG_DIR
            flight_log.LOG_DIR = tmp
            try:
                self.assertEqual(flight.resolve(), flight_log.select_log_file().resolve())
            finally:
                flight_log.LOG_DIR = original

    def test_probe_analyzer_finds_firmware_constants_after_move(self):
        self.assertTrue(probe_response.FIRMWARE_PATH.exists())


if __name__ == "__main__":
    unittest.main()
