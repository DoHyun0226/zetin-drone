import re
import shutil
import signal
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
NATIVE_TEST_DIR = REPO_ROOT / "tools" / "native_tests"
SKETCH_DIR = (
    REPO_ROOT / "firmware" / "flight" / "zetin_fc"
)
sys.path.insert(0, str(REPO_ROOT / "scripts"))
from ground import firmware_source  # noqa: E402


class NativeControlMathTest(unittest.TestCase):
    """실제 비행 펌웨어(`zetin_fc`)를 host shim 과 함께 컴파일해 C++ control-math
    assertion 을 실행한다. 저장소의 `unittest discover` CI 가 수집하도록 unittest 형식으로
    둔다. g++/-m32 툴체인이 없으면 실패하지 않고 skip 한다."""

    def test_native_control_math(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is unavailable; skipping native firmware logic tests")

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)

            m32_probe = tmp_path / "m32_probe"
            probe_command = [
                compiler, "-std=c++17", "-m32", "-x", "c++", "-", "-o", str(m32_probe),
            ]
            try:
                probe = subprocess.run(
                    probe_command,
                    input="static_assert(sizeof(long) == 4); int main() { return 0; }\n",
                    capture_output=True, text=True, check=False, timeout=60,
                )
            except subprocess.TimeoutExpired as error:
                self.fail(
                    "32-bit g++ multilib toolchain probe timed out after 60 seconds\n"
                    f"command: {' '.join(probe_command)}\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            if probe.returncode != 0:
                self.skipTest(
                    "32-bit g++ multilib toolchain is unavailable; "
                    "native ESP32-width tests require -m32\n"
                    f"command: {' '.join(probe_command)}\n"
                    f"stdout:\n{probe.stdout}\nstderr:\n{probe.stderr}"
                )

            executable = tmp_path / "test_control_math"
            compile_command = [
                compiler, "-std=c++17", "-O0", "-g", "-m32",
                "-I", str(NATIVE_TEST_DIR / "shims"),
                "-I", str(SKETCH_DIR),
                "-I", str(SKETCH_DIR / "logic"),
                "-x", "c++", str(NATIVE_TEST_DIR / "test_control_math.cpp"),
                "-o", str(executable),
            ]
            try:
                compiled = subprocess.run(
                    compile_command, capture_output=True, text=True, check=False, timeout=60,
                )
            except subprocess.TimeoutExpired as error:
                self.fail(
                    "native firmware test compilation timed out after 60 seconds\n"
                    f"command: {' '.join(compile_command)}\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            if compiled.returncode != 0:
                self.fail(
                    "native firmware test compilation failed\n"
                    f"command: {' '.join(compile_command)}\n"
                    f"stdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
                )

            try:
                completed = subprocess.run(
                    [str(executable)], capture_output=True, text=True, check=False, timeout=10,
                )
            except subprocess.TimeoutExpired as error:
                self.fail(
                    "native firmware logic tests timed out after 10 seconds\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            # 일부 샌드박스는 32비트 바이너리의 syscall 을 SIGSYS 로 막는다. qemu-i386 이 있으면
            # 그 위에서 다시 돌리고, 없으면 거짓 실패 대신 skip 한다.
            if completed.returncode == -signal.SIGSYS:
                qemu_i386 = shutil.which("qemu-i386-static")
                if qemu_i386 is None:
                    self.skipTest(
                        "32-bit native test execution was blocked by SIGSYS and "
                        "qemu-i386-static is unavailable"
                    )
                try:
                    completed = subprocess.run(
                        [qemu_i386, str(executable)],
                        capture_output=True, text=True, check=False, timeout=10,
                    )
                except subprocess.TimeoutExpired as error:
                    self.fail(
                        "native firmware logic tests under qemu-i386-static timed out "
                        "after 10 seconds\n"
                        f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                    )
            if completed.returncode != 0:
                self.fail(
                    "native firmware logic tests failed\n"
                    f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
                )

    def test_failsafe_descent_delta_must_fit_control_margin(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is unavailable; skipping compile-time guard test")

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            m32_probe = tmp_path / "m32_probe"
            probe_command = [
                compiler, "-std=c++17", "-m32", "-x", "c++", "-", "-o", str(m32_probe),
            ]
            probe = subprocess.run(
                probe_command,
                input="static_assert(sizeof(long) == 4); int main() { return 0; }\n",
                capture_output=True, text=True, check=False, timeout=60,
            )
            if probe.returncode != 0:
                self.skipTest(
                    "32-bit g++ multilib toolchain is unavailable; "
                    "compile-time guard test requires -m32\n"
                    f"command: {' '.join(probe_command)}\n"
                    f"stdout:\n{probe.stdout}\nstderr:\n{probe.stderr}"
                )

            mutated_sketch = tmp_path / "zetin_fc"
            shutil.copytree(SKETCH_DIR, mutated_sketch)
            sketch_path = mutated_sketch / "zetin_fc.ino"
            # 상수는 스케치의 어느 파일에 있어도 된다. 정확히 한 번 나와야 한다.
            firmware_source.subn_unique(
                mutated_sketch,
                r"(const int\s+FS_DESCENT_DELTA_US\s*=\s*)60;",
                r"\g<1>150;",
            )

            object_path = tmp_path / "invalid_delta.o"
            compile_command = [
                compiler, "-std=c++17", "-O0", "-g", "-m32",
                "-I", str(NATIVE_TEST_DIR / "shims"),
                "-I", str(mutated_sketch),
                "-I", str(mutated_sketch / "logic"),
                "-x", "c++", "-c", str(sketch_path), "-o", str(object_path),
            ]
            compiled = subprocess.run(
                compile_command, capture_output=True, text=True, check=False, timeout=60,
            )
            self.assertNotEqual(
                compiled.returncode,
                0,
                "FS_DESCENT_DELTA_US == CTRL_MARGIN unexpectedly compiled",
            )
            self.assertIn(
                "FS_DESCENT_DELTA_US must be less than CTRL_MARGIN",
                compiled.stderr,
            )

if __name__ == "__main__":
    unittest.main()
