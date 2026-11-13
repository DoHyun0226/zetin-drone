import re
import shutil
import signal
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
NATIVE_TEST_DIR = REPO_ROOT / "tools" / "native_tests"
SKETCH_DIR = REPO_ROOT / "firmware" / "archive" / "legacy_flight" / "dual_imu_cascade_pwm"
SKETCH_PATH = SKETCH_DIR / "dual_imu_cascade_pwm.ino"


def _firmware_float_constant(name):
    source = SKETCH_PATH.read_text(encoding="utf-8")
    match = re.search(
        rf"\b(?:const|constexpr)\s+float\s+{re.escape(name)}\s*=\s*"
        r"([0-9]+(?:\.[0-9]+)?f)\s*;",
        source,
    )
    if match is None:
        raise AssertionError(f"firmware float constant not found: {name}")
    return match.group(1)


class FailsafeLandTest(unittest.TestCase):
    """Compile and run the failsafe-land unit test at -m32."""

    def test_failsafe_land(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is unavailable; skipping failsafe-land unit test")

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            m32_probe = tmp_path / "m32_probe"
            probe_command = [
                compiler, "-std=c++17", "-m32", "-x", "c++", "-",
                "-o", str(m32_probe),
            ]
            try:
                probe = subprocess.run(
                    probe_command,
                    input="static_assert(sizeof(long) == 4); int main() { return 0; }\n",
                    capture_output=True,
                    text=True,
                    check=False,
                    timeout=60,
                )
            except subprocess.TimeoutExpired as error:
                self.fail(
                    "failsafe-land unit test 32-bit g++ probe timed out "
                    "after 60 seconds\n"
                    f"command: {' '.join(probe_command)}\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            if probe.returncode != 0:
                self.skipTest(
                    "32-bit g++ multilib is unavailable; "
                    "failsafe-land unit test requires -m32\n"
                    f"stdout:\n{probe.stdout}\nstderr:\n{probe.stderr}"
                )

            executable = tmp_path / "test_failsafe_land"
            compile_command = [
                compiler, "-std=c++17", "-O0", "-g", "-Wall", "-m32",
                "-I", str(NATIVE_TEST_DIR / "shims"),
                "-I", str(SKETCH_DIR),
                (
                    "-DFIRMWARE_FS_LAND_LPF_ALPHA="
                    f"{_firmware_float_constant('FS_LAND_LPF_ALPHA')}"
                ),
                (
                    "-DFIRMWARE_FS_PROBE_DIP_FRAC="
                    f"{_firmware_float_constant('FS_PROBE_DIP_FRAC')}"
                ),
                (
                    "-DFIRMWARE_FS_PROBE_RESPONSE_G="
                    f"{_firmware_float_constant('FS_PROBE_RESPONSE_G')}"
                ),
                "-x", "c++",
                str(NATIVE_TEST_DIR / "test_failsafe_land.cpp"),
                "-o", str(executable),
            ]
            try:
                compiled = subprocess.run(
                    compile_command,
                    capture_output=True,
                    text=True,
                    check=False,
                    timeout=60,
                )
            except subprocess.TimeoutExpired as error:
                self.fail(
                    "failsafe-land unit test compilation timed out "
                    "after 60 seconds\n"
                    f"command: {' '.join(compile_command)}\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            if compiled.returncode != 0:
                self.fail(
                    "failsafe-land unit test compilation failed\n"
                    f"command: {' '.join(compile_command)}\n"
                    f"stdout:\n{compiled.stdout}\nstderr:\n{compiled.stderr}"
                )

            command = [str(executable)]
            try:
                completed = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    check=False,
                    timeout=20,
                )
            except subprocess.TimeoutExpired as error:
                self.fail(
                    "failsafe-land unit test timed out after 20 seconds\n"
                    f"command: {' '.join(command)}\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            if completed.returncode == -signal.SIGSYS:
                qemu_i386 = shutil.which("qemu-i386-static")
                if qemu_i386 is None:
                    self.skipTest(
                        "failsafe-land unit test execution was SIGSYS-blocked and "
                        "qemu-i386-static is unavailable"
                    )
                command = [qemu_i386, str(executable)]
                try:
                    completed = subprocess.run(
                        command,
                        capture_output=True,
                        text=True,
                        check=False,
                        timeout=20,
                    )
                except subprocess.TimeoutExpired as error:
                    self.fail(
                        "failsafe-land unit test timed out after 20 seconds\n"
                        f"command: {' '.join(command)}\n"
                        f"stdout:\n{error.stdout or ''}\n"
                        f"stderr:\n{error.stderr or ''}"
                    )

            print(f"[FS-LAND] runner={' '.join(command)}")
            print(completed.stdout, end="")
            if completed.stderr:
                print(completed.stderr, end="")
            if completed.returncode != 0:
                self.fail(
                    "failsafe-land unit test failed\n"
                    f"command: {' '.join(command)}\n"
                    f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
                )


if __name__ == "__main__":
    unittest.main()
