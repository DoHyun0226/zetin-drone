import csv
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from scripts.ground import failsafe_telemetry


REPO_ROOT = Path(__file__).resolve().parents[1]
ANALYZE_SCRIPT = REPO_ROOT / "scripts" / "drone_logs.py"

TELEMETRY_FIELDS = (
    "Roll",
    "Pitch",
    "Yaw",
    "Gyro_X",
    "Gyro_Y",
    "Gyro_Z",
    "Accel_X",
    "Accel_Y",
    "Accel_Z",
    "Throttle",
    "Fault_RC",
    "Fault_Critical",
    "RC_Total_Pkts",
    "RC_Dropped_Pkts",
    "Fault_IMU1",
    "Fault_IMU2",
    "Fault_Disagree",
    "Active_IMUs",
    "Mixer_Scaled",
    "Fault_Attitude",
    "Calibration_OK",
    "Armed",
    "Motor_M1",
    "Motor_M2",
    "Motor_M3",
    "Motor_M4",
    "PID_Loop_Hz",
    "TgtRate_Roll",
    "TgtRate_Pitch",
    "TgtRate_Yaw",
    "MagHeading",
    "Mag_X",
    "Mag_Y",
    "Mag_Z",
    "Yaw_Hold",
    "Failsafe_Phase",
    "Trim_Roll",
    "Trim_Pitch",
)


class AnalyzeFailsafeTelemetryTests(unittest.TestCase):
    def run_analyzer(self, telemetry_field_count, samples):
        fields = TELEMETRY_FIELDS[:telemetry_field_count]
        with tempfile.TemporaryDirectory() as tmp_dir:
            log_path = Path(tmp_dir) / "flight.csv"
            with log_path.open("w", newline="", encoding="utf-8") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=("Timestamp",) + fields)
                writer.writeheader()
                for sample in samples:
                    row = dict.fromkeys(fields, "")
                    row.update(sample)
                    writer.writerow({"Timestamp": sample["Timestamp"], **row})

            env = os.environ.copy()
            env["MPLBACKEND"] = "Agg"
            return subprocess.run(
                [sys.executable, str(ANALYZE_SCRIPT), "flight", str(log_path)],
                cwd=REPO_ROOT,
                env=env,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )

    def test_reports_phase_transitions_duration_terminal_phase_and_trim(self):
        result = self.run_analyzer(
            38,
            [
                {
                    "Timestamp": "12:00:00.000",
                    "Failsafe_Phase": 0,
                    "Trim_Roll": 1.25,
                    "Trim_Pitch": -0.75,
                },
                {
                    "Timestamp": "12:00:00.100",
                    "Failsafe_Phase": 1,
                    "Trim_Roll": 1.25,
                    "Trim_Pitch": -0.75,
                },
                {
                    "Timestamp": "12:00:00.250",
                    "Failsafe_Phase": 1,
                    "Trim_Roll": 1.25,
                    "Trim_Pitch": -0.75,
                },
                {
                    "Timestamp": "12:00:00.400",
                    "Failsafe_Phase": 2,
                    "Trim_Roll": 1.25,
                    "Trim_Pitch": -0.75,
                },
            ],
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("NONE (0) → DESCENDING (1)", result.stdout)
        self.assertIn("DESCENDING (1) → CUT_LANDED (2)", result.stdout)
        self.assertIn("자동착륙 소요 시간: 0.300초", result.stdout)
        self.assertIn("종료 phase: CUT_LANDED (2)", result.stdout)
        self.assertIn("Trim: Roll +1.25°, Pitch -0.75°", result.stdout)

    def test_legacy_34_and_35_field_logs_report_unknown_without_crashing(self):
        samples = [{"Timestamp": "12:00:00.000"}]

        for field_count in (34, 35):
            with self.subTest(field_count=field_count):
                result = self.run_analyzer(field_count, samples)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(
                    "Failsafe_Phase: 전이 없음 (legacy/unknown)",
                    result.stdout,
                )


class MonitorFailsafeTelemetryTests(unittest.TestCase):
    def formatter(self):
        formatter = getattr(failsafe_telemetry, "format_monitor_status", None)
        self.assertIsNotNone(
            formatter,
            "monitor status formatter must expose failsafe and trim telemetry",
        )
        return formatter

    def renderer(self):
        renderer = getattr(failsafe_telemetry, "render_monitor_title", None)
        self.assertIsNotNone(
            renderer,
            "monitor title renderer must apply the visible failsafe alert",
        )
        return renderer

    def test_descending_phase_is_named_and_visibly_flagged(self):
        text, alert = self.formatter()(
            {
                "Failsafe_Phase": 1,
                "Trim_Roll": 1.25,
                "Trim_Pitch": -0.75,
            }
        )

        self.assertTrue(alert)
        self.assertIn("AUTO-LAND ACTIVE", text)
        self.assertIn("DESCENDING (1)", text)
        self.assertIn("Trim R +1.25° / P -0.75°", text)

    def test_none_phase_is_named_without_alert(self):
        text, alert = self.formatter()(
            {
                "Failsafe_Phase": 0,
                "Trim_Roll": 0.0,
                "Trim_Pitch": 0.0,
            }
        )

        self.assertFalse(alert)
        self.assertNotIn("AUTO-LAND ACTIVE", text)
        self.assertIn("NONE (0)", text)

    def test_terminal_phases_use_firmware_enum_names_and_remain_flagged(self):
        for phase, expected_name in (
            (2, "CUT_LANDED (2)"),
            (3, "CUT_TIMEOUT (3)"),
            (4, "CUT_ABORT (4)"),
        ):
            with self.subTest(phase=phase):
                text, alert = self.formatter()(
                    {
                        "Failsafe_Phase": phase,
                        "Trim_Roll": 0.0,
                        "Trim_Pitch": 0.0,
                    }
                )
                self.assertTrue(alert)
                self.assertIn(expected_name, text)

    def test_probe_unavailable_and_no_response_count_are_visible(self):
        text, alert = self.formatter()(
            {
                "Failsafe_Phase": 1,
                "Trim_Roll": 0.0,
                "Trim_Pitch": 0.0,
                "Hover_Est": 1340.25,
                "Hover_Valid": 1,
                "Failsafe_Probe_State": 3,
                "Failsafe_Probe_NoResponse": 0,
                "Failsafe_Probe_Response_G": 0.01234,
            }
        )

        self.assertTrue(alert)
        self.assertIn("Hover 1340.2µs, valid 1", text)
        self.assertIn("Probe UNAVAILABLE (3)", text)
        self.assertIn("no-response 0", text)
        self.assertIn("response 0.0123g", text)

    def test_probe_blocked_is_named_and_marked_as_an_alert(self):
        text, alert = self.formatter()(
            {
                "Failsafe_Phase": 0,
                "Trim_Roll": 0.0,
                "Trim_Pitch": 0.0,
                "Failsafe_Probe_State": 4,
                "Failsafe_Probe_NoResponse": 1,
                "Failsafe_Probe_Response_G": 0.02,
            }
        )

        self.assertTrue(alert)
        self.assertIn("Probe BLOCKED (4)", text)
        self.assertIn("⚠", text)
        self.assertIn("no-response 1", text)

    def test_probe_state_formatter_names_blocked_and_handles_none(self):
        formatter = getattr(
            failsafe_telemetry,
            "format_failsafe_probe_state",
            None,
        )
        self.assertIsNotNone(formatter)
        self.assertEqual("BLOCKED (4)", formatter(4))
        self.assertEqual("legacy/unknown", formatter(None))

    def test_renderer_applies_alert_box_to_nonzero_phase(self):
        figure, axis = plt.subplots()
        try:
            title = self.renderer()(
                axis,
                "System status — active IMUs: 2, faults: RC",
                {
                    "Failsafe_Phase": 1,
                    "Trim_Roll": 1.25,
                    "Trim_Pitch": -0.75,
                },
            )
            self.assertIn("AUTO-LAND ACTIVE", title.get_text())
            self.assertEqual(title.get_color(), "white")
            self.assertEqual(title.get_fontweight(), "bold")
            self.assertIsNotNone(title.get_bbox_patch())
        finally:
            plt.close(figure)

    def test_legacy_none_values_render_as_unknown_without_alert(self):
        text, alert = self.formatter()(
            {
                "Failsafe_Phase": None,
                "Trim_Roll": None,
                "Trim_Pitch": None,
            }
        )

        self.assertFalse(alert)
        self.assertIn("Failsafe legacy/unknown", text)
        self.assertIn("Trim R unknown / P unknown", text)
        self.assertNotIn("Hover", text)

    def test_legacy_none_probe_values_do_not_crash_or_add_empty_status(self):
        text, alert = self.formatter()(
            {
                "Failsafe_Phase": None,
                "Trim_Roll": None,
                "Trim_Pitch": None,
                "Hover_Est": None,
                "Hover_Valid": None,
                "Failsafe_Probe_State": None,
                "Failsafe_Probe_NoResponse": None,
                "Failsafe_Probe_Response_G": None,
            }
        )

        self.assertFalse(alert)
        self.assertNotIn("Hover", text)
        self.assertNotIn("Probe", text)


if __name__ == "__main__":
    unittest.main()
