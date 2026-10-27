import shutil
import signal
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
NATIVE_TEST_DIR = REPO_ROOT / "tools" / "native_tests"
SKETCH_DIR = (
    REPO_ROOT / "firmware" / "flight" / "dual_imu_cascade_pwm"
)
EXECUTION_TIMEOUT_S = 180


class SilAttitudeTest(unittest.TestCase):
    """실제 캐스케이드 스케치를 호스트 SIL 플랜트와 폐루프로 실행한다.

    저장소의 unittest discover가 수집할 수 있는 형식을 유지하며, g++ 또는
    32비트 multilib가 없을 때만 명시적으로 건너뛴다.
    """

    def test_sil_attitude(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is unavailable; skipping native attitude SIL tests")

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
                    "native attitude SIL tests require -m32\n"
                    f"command: {' '.join(probe_command)}\n"
                    f"stdout:\n{probe.stdout}\nstderr:\n{probe.stderr}"
                )

            executable = tmp_path / "test_sil_attitude"
            compile_command = [
                compiler, "-std=c++17", "-O0", "-g", "-Wall", "-m32",
                "-I", str(NATIVE_TEST_DIR / "shims"),
                "-I", str(SKETCH_DIR),
                "-x", "c++", str(NATIVE_TEST_DIR / "test_sil_attitude.cpp"),
                "-o", str(executable),
            ]
            try:
                compiled = subprocess.run(
                    compile_command, capture_output=True, text=True, check=False, timeout=60,
                )
            except subprocess.TimeoutExpired as error:
                self.fail(
                    "native attitude SIL compilation timed out after 60 seconds\n"
                    f"command: {' '.join(compile_command)}\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            if compiled.returncode != 0:
                self.fail(
                    "native attitude SIL compilation failed\n"
                    f"command: {' '.join(compile_command)}\n"
                    f"stdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
                )

            try:
                completed = subprocess.run(
                    [str(executable)], capture_output=True, text=True, check=False,
                    timeout=EXECUTION_TIMEOUT_S,
                )
            except subprocess.TimeoutExpired as error:
                self.fail(
                    f"native attitude SIL timed out after {EXECUTION_TIMEOUT_S} seconds\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            # 일부 샌드박스는 32비트 ELF syscall을 SIGSYS로 막는다. 이때만
            # qemu-i386-static으로 같은 바이너리를 다시 실행한다.
            if completed.returncode == -signal.SIGSYS:
                qemu_i386 = shutil.which("qemu-i386-static")
                if qemu_i386 is None:
                    self.skipTest(
                        "32-bit native SIL execution was blocked by SIGSYS and "
                        "qemu-i386-static is unavailable"
                    )
                try:
                    completed = subprocess.run(
                        [qemu_i386, str(executable)],
                        capture_output=True, text=True, check=False,
                        timeout=EXECUTION_TIMEOUT_S,
                    )
                except subprocess.TimeoutExpired as error:
                    self.fail(
                        "native attitude SIL under qemu-i386-static timed out "
                        f"after {EXECUTION_TIMEOUT_S} seconds\n"
                        f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                    )
            if completed.returncode != 0:
                self.fail(
                    "native attitude SIL failed\n"
                    f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
                )
            for line in completed.stdout.splitlines():
                if line.startswith("[R1]") or line.startswith("[SIL] S2b"):
                    print(line)


if __name__ == "__main__":
    unittest.main()
