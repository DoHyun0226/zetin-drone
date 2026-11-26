"""tools/compare_firmware_elf.py 를 호스트에서 만든 작은 ELF32 로 시험한다."""

import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import compare_firmware_elf as cmp  # noqa: E402

SOURCE = r"""
#include <string.h>
struct Desc { unsigned magic; unsigned secure; unsigned r[2]; char version[32];
  char project[32]; char time[16]; char date[16]; char idf[32];
  unsigned char sha[32]; unsigned short lo, hi; unsigned char mmu; unsigned char r3[3];
  unsigned r2[18]; };
__attribute__((used)) const struct Desc esp_app_desc = {
  0xABCD5432u, SECURE, {0, 0}, "v1", "zetin_fc", TIME, "Nov 13 2026", "v5",
  {SHA}, 0, 99, 16, {0}, {0} };
const int gain = GAIN;
int counter;
int main(void) { counter += gain; return counter + (int)strlen(esp_app_desc.time); }
"""


class CompareFirmwareElfTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cc = shutil.which("gcc")
        if cls.cc is None:
            raise unittest.SkipTest("gcc is unavailable")
        cls.tmp = tempfile.TemporaryDirectory()
        probe = subprocess.run([cls.cc, "-m32", "-x", "c", "-", "-o",
                                str(Path(cls.tmp.name) / "probe")],
                               input="int main(void){return 0;}\n", capture_output=True,
                               text=True, check=False)
        if probe.returncode != 0:
            cls.tmp.cleanup()
            raise unittest.SkipTest("32-bit gcc multilib is unavailable")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def build(self, name, gain="3", time='"12:00:00"', sha="1", secure="0"):
        src = Path(self.tmp.name) / f"{name}.c"
        out = Path(self.tmp.name) / f"{name}.elf"
        src.write_text(SOURCE)
        subprocess.run([self.cc, "-m32", "-O1", "-Wl,--build-id=none", f"-DGAIN={gain}", f"-DTIME={time}",
                        f"-DSHA={sha}", f"-DSECURE={secure}", str(src), "-o", str(out)],
                       check=True, capture_output=True)
        return out

    def test_identical_builds_compare_equal(self):
        diffs, _ = cmp.compare(self.build("a"), self.build("b"))
        self.assertEqual([], diffs)

    def test_a_changed_constant_is_reported(self):
        diffs, _ = cmp.compare(self.build("a"), self.build("c", gain="4"))
        self.assertTrue(diffs)

    def test_build_time_and_sha_are_masked(self):
        diffs, _ = cmp.compare(self.build("a"),
                               self.build("d", time='"13:14:15"', sha="7"))
        self.assertEqual([], diffs)

    def test_secure_version_is_not_masked(self):
        diffs, _ = cmp.compare(self.build("a"), self.build("e", secure="1"))
        self.assertTrue(diffs)


if __name__ == "__main__":
    unittest.main()
