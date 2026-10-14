import shutil
import signal
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
NATIVE_TEST_DIR = REPO_ROOT / "tools" / "native_tests"
SKETCH_DIR = REPO_ROOT / "firmware" / "flight" / "dual_imu_cascade_pwm"


class YawAuthorityTest(unittest.TestCase):
    """Compile and run the pure yaw-authority contract at ESP32 width."""

    def test_yaw_authority(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is unavailable; skipping yaw-authority test")

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
                    "32-bit g++ multilib is unavailable; yaw-authority test requires -m32\n"
                    f"command: {' '.join(probe_command)}\n"
                    f"stdout:\n{probe.stdout}\nstderr:\n{probe.stderr}"
                )

            executable = tmp_path / "test_yaw_authority"
            compile_command = [
                compiler, "-std=c++17", "-O0", "-g", "-Wall", "-Wextra", "-m32",
                "-I", str(SKETCH_DIR),
                str(NATIVE_TEST_DIR / "test_yaw_authority.cpp"),
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
                    "yaw-authority test compilation failed\n"
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
                        "yaw-authority test execution was SIGSYS-blocked and "
                        "qemu-i386-static is unavailable"
                    )
                command = [qemu_i386, str(executable)]
                completed = subprocess.run(
                    command, capture_output=True, text=True, check=False, timeout=10
                )

            print(f"[YAW-AUTH] runner={' '.join(command)}")
            print(completed.stdout, end="")
            if completed.stderr:
                print(completed.stderr, end="")
            if completed.returncode != 0:
                self.fail(
                    "yaw-authority test failed\n"
                    f"command: {' '.join(command)}\n"
                    f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
                )


if __name__ == "__main__":
    unittest.main()
