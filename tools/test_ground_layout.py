import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))


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


if __name__ == "__main__":
    unittest.main()
