import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_repo_layout import check_migration_map  # noqa: E402


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

    def test_migration_map_allows_merged_destinations(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            (repo / "docs").mkdir()
            (repo / "new").mkdir()
            (repo / "new" / "merged.py").write_text("")
            mapping = repo / "docs" / "migration_map.md"
            header = "| Old path | New path |\n|---|---|\n"
            mapping.write_text(
                header + "| `old/a.py` | `new/merged.py` |\n"
                "| `old/b.py` | `new/merged.py` |\n",
                encoding="utf-8",
            )
            self.assertEqual([], check_migration_map(repo, mapping, 2))
            mapping.write_text(
                header + "| `old/a.py` | `new/merged.py` |\n"
                "| `old/a.py` | `new/merged.py` |\n",
                encoding="utf-8",
            )
            errors = check_migration_map(repo, mapping, 2)
            self.assertTrue(any("duplicate old path" in e for e in errors))
            mapping.write_text(
                header + "| `old/a.py` | `new/gone.py` |\n", encoding="utf-8"
            )
            errors = check_migration_map(repo, mapping, 1)
            self.assertTrue(any("new path missing" in e for e in errors))


if __name__ == "__main__":
    unittest.main()
