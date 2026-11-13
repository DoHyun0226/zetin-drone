"""scripts/ground/firmware_source.py(여러 파일로 나뉜 스케치 읽기) 시험."""

import shutil
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from ground import firmware_source as fs  # noqa: E402


class FirmwareSourceTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name) / "zetin_fc"
        (self.dir / "fc").mkdir(parents=True)
        (self.dir / "logic").mkdir()
        (self.dir / "zetin_fc.ino").write_text('#include "fc/config.h"\nvoid setup() {}\n')
        (self.dir / "fc" / "config.h").write_text(
            "constexpr float FS_PROBE_RESPONSE_G = 0.03f;\nconst int FS_DESCENT_DELTA_US = 60;\n")
        (self.dir / "logic" / "a.h").write_text("const float DUP = 1.0f;\n")
        (self.dir / "logic" / "b.h").write_text("const float DUP = 2.0f;\n")

    def tearDown(self):
        self.tmp.cleanup()

    def test_order_is_ino_then_fc_then_logic(self):
        names = [p.relative_to(self.dir).as_posix() for p in fs.sketch_files(self.dir)]
        self.assertEqual(["zetin_fc.ino", "fc/config.h", "logic/a.h", "logic/b.h"], names)

    def test_constant_is_found_in_a_module_header(self):
        self.assertEqual("0.03f", fs.read_float_constant("FS_PROBE_RESPONSE_G", self.dir))

    def test_missing_and_duplicate_definitions_are_errors(self):
        with self.assertRaises(LookupError):
            fs.read_float_constant("NOT_THERE", self.dir)
        with self.assertRaises(LookupError):
            fs.read_float_constant("DUP", self.dir)

    def test_mutation_changes_exactly_the_file_that_holds_it(self):
        path = fs.mutate_unique(self.dir, "FS_DESCENT_DELTA_US = 60;", "FS_DESCENT_DELTA_US = 150;")
        self.assertEqual(self.dir / "fc" / "config.h", path)
        self.assertIn("= 150;", path.read_text())
        with self.assertRaises(LookupError):
            fs.mutate_unique(self.dir, "FS_DESCENT_DELTA_US = 60;", "x")

    def test_real_sketch_has_its_probe_constants(self):
        for name in ("FS_PROBE_RESPONSE_G", "FS_PROBE_DIP_FRAC"):
            self.assertRegex(fs.read_float_constant(name), r"^[0-9.]+f$")


if __name__ == "__main__":
    unittest.main()
