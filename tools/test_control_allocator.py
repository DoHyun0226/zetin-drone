import shutil
import signal
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
NATIVE_TEST_DIR = REPO_ROOT / "tools" / "native_tests"
SKETCH_DIR = REPO_ROOT / "firmware" / "flight" / "dual_imu_cascade_pwm"


class ControlAllocatorTest(unittest.TestCase):
    """Compile and run the pure priority-control allocator at ESP32-width."""

    def test_control_allocator(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is unavailable; skipping control allocator test")

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            probe_command = [
                compiler, "-std=c++17", "-m32", "-x", "c++", "-", "-o",
                str(tmp_path / "m32_probe"),
            ]
            probe = subprocess.run(
                probe_command,
                input="static_assert(sizeof(long) == 4); int main() { return 0; }\n",
                capture_output=True,
                text=True,
                check=False,
                timeout=60,
            )
            if probe.returncode != 0:
                self.skipTest(
                    "32-bit g++ multilib is unavailable; allocator test requires -m32\n"
                    f"command: {' '.join(probe_command)}\n"
                    f"stdout:\n{probe.stdout}\nstderr:\n{probe.stderr}"
                )

            executable = tmp_path / "test_control_allocator"
            compile_command = [
                compiler, "-std=c++17", "-O0", "-g", "-Wall", "-m32",
                "-I", str(SKETCH_DIR),
                str(NATIVE_TEST_DIR / "test_control_allocator.cpp"),
                "-o", str(executable),
            ]
            compiled = subprocess.run(
                compile_command,
                capture_output=True,
                text=True,
                check=False,
                timeout=60,
            )
            if compiled.returncode != 0:
                self.fail(
                    "control allocator test compilation failed\n"
                    f"command: {' '.join(compile_command)}\n"
                    f"stdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
                )

            command = [str(executable)]
            completed = subprocess.run(
                command, capture_output=True, text=True, check=False, timeout=10
            )
            if completed.returncode == -signal.SIGSYS:
                qemu_i386 = shutil.which("qemu-i386-static")
                if qemu_i386 is None:
                    self.skipTest(
                        "allocator test execution was SIGSYS-blocked and "
                        "qemu-i386-static is unavailable"
                    )
                command = [qemu_i386, str(executable)]
                completed = subprocess.run(
                    command, capture_output=True, text=True, check=False, timeout=10
                )
            if completed.returncode != 0:
                self.fail(
                    "control allocator test failed\n"
                    f"command: {' '.join(command)}\n"
                    f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
                )


if __name__ == "__main__":
    unittest.main()
