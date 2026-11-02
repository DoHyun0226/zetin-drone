import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
NATIVE_TEST_DIR = REPO_ROOT / "tools" / "native_tests"
SKETCH_DIR = REPO_ROOT / "firmware" / "flight" / "dual_imu_cascade_flow_hold"
EXECUTION_TIMEOUT_S = 600

# 헤더 단위 시험 3개와, 새 스케치를 통째로 넣은 폐루프 SIL 하나.
NATIVE_TESTS = (
    "test_flow_calibration",
    "test_flow_odometry",
    "test_flow_hold",
    "test_sil_flow_hold",
)


class FlowHoldNativeTest(unittest.TestCase):
    """Compile and run the flow-hold native tests at -m32."""

    @classmethod
    def setUpClass(cls):
        cls.compiler = shutil.which("g++")
        if cls.compiler is None:
            raise unittest.SkipTest("g++ is unavailable; skipping flow-hold native tests")
        cls.tmp = tempfile.TemporaryDirectory()
        probe = subprocess.run(
            [cls.compiler, "-std=c++17", "-m32", "-x", "c++", "-",
             "-o", str(Path(cls.tmp.name) / "m32_probe")],
            input="static_assert(sizeof(long) == 4); int main() { return 0; }\n",
            capture_output=True, text=True, check=False, timeout=60,
        )
        if probe.returncode != 0:
            cls.tmp.cleanup()
            raise unittest.SkipTest(
                "32-bit g++ multilib is unavailable; flow-hold native tests require -m32\n"
                f"stderr:\n{probe.stderr}"
            )

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def _compile_and_run(self, name):
        executable = Path(self.tmp.name) / name
        compile_command = [
            self.compiler, "-std=c++17", "-O1", "-g", "-Wall", "-m32",
            "-I", str(NATIVE_TEST_DIR / "shims"),
            "-I", str(SKETCH_DIR),
            "-x", "c++", str(NATIVE_TEST_DIR / f"{name}.cpp"),
            "-o", str(executable),
        ]
        compiled = subprocess.run(
            compile_command, capture_output=True, text=True, check=False, timeout=300,
        )
        if compiled.returncode != 0:
            self.fail(
                f"{name} compilation failed\ncommand: {' '.join(compile_command)}\n"
                f"stdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
            )
        run = subprocess.run(
            [str(executable)], capture_output=True, text=True, check=False,
            timeout=EXECUTION_TIMEOUT_S,
        )
        if run.returncode != 0:
            self.fail(f"{name} failed\nstdout:\n{run.stdout}\nstderr:\n{run.stderr}")
        self.assertNotIn("[FAIL]", run.stdout + run.stderr)

    def test_flow_calibration(self):
        self._compile_and_run("test_flow_calibration")

    def test_flow_odometry(self):
        self._compile_and_run("test_flow_odometry")

    def test_flow_hold(self):
        self._compile_and_run("test_flow_hold")

    def test_sil_flow_hold(self):
        self._compile_and_run("test_sil_flow_hold")


if __name__ == "__main__":
    unittest.main()
