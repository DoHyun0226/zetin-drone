import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "tools"))

import drone_video  # noqa: E402
from ground import camera_record  # noqa: E402
from test_camera_to_mp4 import make_recording  # noqa: E402


class DroneVideoTest(unittest.TestCase):
    def test_default_subcommand_is_record(self):
        with mock.patch.object(camera_record, "main", return_value=0) as m:
            self.assertEqual(drone_video.main(["--no-preview"]), 0)
        m.assert_called_once_with(["--no-preview"])

    def test_no_args_runs_record(self):
        with mock.patch.object(camera_record, "main", return_value=0) as m:
            self.assertEqual(drone_video.main([]), 0)
        m.assert_called_once_with([])

    def test_explicit_record_forwards_rest(self):
        with mock.patch.object(camera_record, "main", return_value=3) as m:
            self.assertEqual(drone_video.main(["record", "--timeout", "2"]), 3)
        m.assert_called_once_with(["--timeout", "2"])

    def test_mp4_no_mp4_writes_aligned_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, csv_path, _ = make_recording(Path(tmp))
            self.assertEqual(drone_video.main(["mp4", str(csv_path), "--no-mp4"]), 0)
            self.assertTrue(csv_path.with_name(csv_path.stem + "_aligned.csv").exists())

    def test_unknown_subcommand_fails(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            self.assertEqual(drone_video.main(["bogus"]), 2)
        self.assertIn("unknown subcommand: bogus", err.getvalue())
        self.assertIn("usage:", err.getvalue())

    def test_record_default_out_dir_is_repo_logs(self):
        self.assertEqual(camera_record.LOG_DIR, REPO_ROOT / "logs")


if __name__ == "__main__":
    unittest.main()
