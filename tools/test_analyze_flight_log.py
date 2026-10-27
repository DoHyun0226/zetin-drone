import csv
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ANALYZER = REPO_ROOT / "scripts" / "analyze_flight_log.py"


class AnalyzeFlightLogAllocationTest(unittest.TestCase):
    def _run_analyzer(self, fieldnames, rows):
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "flight.csv"
            with csv_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            env = os.environ.copy()
            env["MPLBACKEND"] = "Agg"
            return subprocess.run(
                [sys.executable, str(ANALYZER), str(csv_path), "--no-plot"],
                capture_output=True,
                text=True,
                env=env,
                check=False,
            )

    def test_reports_allocator_authority_pid_and_integrator_summary(self):
        fieldnames = (
            "Timestamp",
            "Mixer_RP_Scale", "Mixer_Yaw_Scale",
            "Yaw_Authority_State",
            "PID_Roll_US", "PID_Pitch_US", "PID_Yaw_US",
            "I_Roll_US", "I_Pitch_US", "I_Yaw_US",
        )
        rows = []
        values = (
            ("00:00:00.000", 1.0, 1.0, 0, 1, -2, 3, 0.1, -0.2, 0.3),
            ("00:00:00.100", 0.8, 0.75, 1, -5, 4, -7, -0.5, 0.6, -0.7),
            ("00:00:00.200", 0.6, 0.5, 1, 3, -6, 2, 1.0, -1.2, 1.4),
            ("00:00:00.300", 0.4, 0.25, 0, 2, 1, -4, -0.8, 0.9, -1.1),
            ("00:00:00.400", 0.2, 0.0, 1, -4, 5, 6, 0.7, -1.0, 1.3),
            ("00:00:00.500", 0.3, 0.1, 1, 2, -3, 5, -0.6, 0.8, -0.9),
        )
        for value_row in values:
            rows.append(dict(zip(fieldnames, value_row)))

        result = self._run_analyzer(fieldnames, rows)

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "Allocator scale: RP min 0.200, p05 0.225; "
            "Yaw min 0.000, p05 0.025",
            result.stdout,
        )
        self.assertIn(
            "Yaw authority LIMITED: 2 entries, 0.300 s cumulative",
            result.stdout,
        )
        self.assertIn(
            "PID |max| (us): Roll 5.000, Pitch 6.000, Yaw 7.000",
            result.stdout,
        )
        self.assertIn(
            "I-term |max| (us): Roll 1.000, Pitch 1.200, Yaw 1.400",
            result.stdout,
        )

    def test_legacy_csv_reports_unknown_extended_allocation_summary(self):
        result = self._run_analyzer(
            ("Timestamp", "Roll", "Pitch", "Yaw", "Throttle"),
            [{
                "Timestamp": "00:00:00.000",
                "Roll": 0.0,
                "Pitch": 0.0,
                "Yaw": 0.0,
                "Throttle": 1100,
            }],
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("Allocator scale: legacy/unknown", result.stdout)
        self.assertIn("Yaw authority LIMITED: legacy/unknown", result.stdout)
        self.assertIn("PID |max| (us): legacy/unknown", result.stdout)
        self.assertIn("I-term |max| (us): legacy/unknown", result.stdout)

    def test_authority_duration_is_unavailable_without_timestamp_column(self):
        result = self._run_analyzer(
            ("Yaw_Authority_State",),
            [{"Yaw_Authority_State": 0}, {"Yaw_Authority_State": 1}],
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "Yaw authority LIMITED: 1 entries, duration unavailable",
            result.stdout,
        )
        self.assertNotIn("0.000 s cumulative", result.stdout)

    def test_authority_duration_is_unavailable_when_all_timestamps_are_invalid(self):
        result = self._run_analyzer(
            ("Timestamp", "Yaw_Authority_State"),
            [
                {"Timestamp": "invalid-a", "Yaw_Authority_State": 0},
                {"Timestamp": "invalid-b", "Yaw_Authority_State": 1},
            ],
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "Yaw authority LIMITED: 1 entries, duration unavailable",
            result.stdout,
        )

    def test_unknown_authority_gap_breaks_duration_continuity(self):
        result = self._run_analyzer(
            ("Timestamp", "Yaw_Authority_State"),
            [
                {"Timestamp": "00:00:00.000", "Yaw_Authority_State": 0},
                {"Timestamp": "00:00:00.100", "Yaw_Authority_State": 1},
                {"Timestamp": "00:00:00.200", "Yaw_Authority_State": 1},
                {"Timestamp": "00:00:00.300", "Yaw_Authority_State": ""},
                {"Timestamp": "00:00:00.400", "Yaw_Authority_State": 0},
            ],
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "Yaw authority LIMITED: 1 entries, "
            "0.100 s cumulative (incomplete: unknown state/time gap)",
            result.stdout,
        )
        self.assertNotIn("0.300 s cumulative", result.stdout)

    def test_authority_duration_handles_midnight_rollover(self):
        result = self._run_analyzer(
            ("Timestamp", "Yaw_Authority_State"),
            [
                {"Timestamp": "23:59:59.900", "Yaw_Authority_State": 1},
                {"Timestamp": "00:00:00.100", "Yaw_Authority_State": 1},
                {"Timestamp": "00:00:00.300", "Yaw_Authority_State": 0},
            ],
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "Yaw authority LIMITED: 1 entries, 0.400 s cumulative",
            result.stdout,
        )

    def test_single_timed_authority_sample_reports_measured_zero_duration(self):
        result = self._run_analyzer(
            ("Timestamp", "Yaw_Authority_State"),
            [{"Timestamp": "00:00:00.000", "Yaw_Authority_State": 1}],
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "Yaw authority LIMITED: 1 entries, 0.000 s cumulative",
            result.stdout,
        )
        self.assertNotIn("duration unavailable", result.stdout)

    def test_non_finite_scale_pid_and_integrator_values_are_excluded(self):
        fieldnames = (
            "Timestamp",
            "Mixer_RP_Scale", "Mixer_Yaw_Scale",
            "Yaw_Authority_State",
            "PID_Roll_US", "PID_Pitch_US", "PID_Yaw_US",
            "I_Roll_US", "I_Pitch_US", "I_Yaw_US",
        )
        result = self._run_analyzer(
            fieldnames,
            [
                dict(zip(
                    fieldnames,
                    ("00:00:00.000", 0.5, 0.25, 0, 5, -6, 7, 1, -2, 3),
                )),
                dict(zip(
                    fieldnames,
                    (
                        "00:00:00.100", "-inf", "-inf", 0,
                        "inf", "-inf", "inf", "inf", "-inf", "inf",
                    ),
                )),
            ],
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "Allocator scale: RP min 0.500, p05 0.500; "
            "Yaw min 0.250, p05 0.250",
            result.stdout,
        )
        self.assertIn(
            "PID |max| (us): Roll 5.000, Pitch 6.000, Yaw 7.000",
            result.stdout,
        )
        self.assertIn(
            "I-term |max| (us): Roll 1.000, Pitch 2.000, Yaw 3.000",
            result.stdout,
        )

    def test_invalid_authority_states_are_unknown_duration_gaps(self):
        result = self._run_analyzer(
            ("Timestamp", "Yaw_Authority_State"),
            [
                {"Timestamp": "00:00:00.000", "Yaw_Authority_State": 1},
                {"Timestamp": "00:00:00.100", "Yaw_Authority_State": 1},
                {"Timestamp": "00:00:00.200", "Yaw_Authority_State": "inf"},
                {"Timestamp": "00:00:00.300", "Yaw_Authority_State": 1},
                {"Timestamp": "00:00:00.400", "Yaw_Authority_State": 3},
                {"Timestamp": "00:00:00.500", "Yaw_Authority_State": 1},
                {"Timestamp": "00:00:00.600", "Yaw_Authority_State": 0},
            ],
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "Yaw authority LIMITED: 3 entries, "
            "0.200 s cumulative (incomplete: unknown state/time gap)",
            result.stdout,
        )
        self.assertNotIn("0.300 s cumulative", result.stdout)


if __name__ == "__main__":
    unittest.main()
