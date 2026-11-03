import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(REPO_ROOT / "tools"))

import check_repo_layout  # noqa: E402


class GroundLayoutTest(unittest.TestCase):
    def test_shared_modules_live_in_ground_package(self):
        for name in ("telemetry_schema", "failsafe_telemetry", "camera_stream", "paths"):
            self.assertTrue((SCRIPTS / "ground" / f"{name}.py").exists(), name)
        for name in ("telemetry_schema", "failsafe_telemetry", "camera_stream"):
            self.assertFalse((SCRIPTS / f"{name}.py").exists(), name)

    def test_paths_point_at_repo_logs(self):
        from ground import paths

        self.assertEqual(paths.REPO_ROOT, REPO_ROOT)
        self.assertEqual(paths.LOG_DIR, REPO_ROOT / "logs")

    def test_only_three_entry_points_at_top_level(self):
        self.assertEqual(sorted(p.name for p in SCRIPTS.glob("*.py")),
                         ["drone_control.py", "drone_logs.py", "drone_video.py"])

    def test_old_script_names_are_legacy_tokens(self):
        for old in ("control_dualsense.py", "tune_pid.py", "receive_telemetry.py", "monitor_telemetry.py",
                    "gamepad_probe.py", "test_dualsense_input.py", "record_camera.py", "camera_to_mp4.py",
                    "analyze_flight_log.py", "decode_imu_raw.py", "analyze_probe_response.py", "magcal_fit.py"):
            self.assertIn(old, check_repo_layout.LEGACY_TOKENS)

    def test_archived_scripts_resolve_ground_imports(self):
        archived = sorted((SCRIPTS / "archive").glob("*.py"))
        names = {p.name for p in archived}
        for name in ("bench_sign_test.py", "bench_yaw_test.py", "bench_thrust_ramp.py",
                     "motor_serial.py", "receive_dual_imu_debug.py"):
            self.assertIn(name, names)
        for p in archived:
            src = p.read_text(encoding="utf-8")
            self.assertNotRegex(src, r"(?m)^from (telemetry_schema|failsafe_telemetry) import")
            if "from ground." in src:
                self.assertIn("os.path.dirname(os.path.dirname(os.path.abspath(__file__)))", src, p.name)


if __name__ == "__main__":
    unittest.main()
