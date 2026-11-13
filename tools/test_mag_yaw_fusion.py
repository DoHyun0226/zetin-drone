import os
import shutil
import signal
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
NATIVE_TEST_DIR = REPO_ROOT / "tools" / "native_tests"
SKETCH_DIR = REPO_ROOT / "firmware" / "archive" / "legacy_flight" / "dual_imu_cascade_pwm"
SUPPORTED_MUTATIONS = {"none", "disabled", "inverted", "tilt-inverted"}


class MagYawFusionTest(unittest.TestCase):
    """Compile the real flight sketch and exercise BMM350 yaw fusion at -m32."""

    def test_mag_yaw_fusion(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is unavailable; skipping BMM350 yaw-fusion SIL")

        mutation = os.environ.get("BMM350_SIL_MUTATION", "none")
        if mutation not in SUPPORTED_MUTATIONS:
            self.fail(
                "BMM350_SIL_MUTATION must be one of "
                + ", ".join(sorted(SUPPORTED_MUTATIONS))
            )

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
                    "32-bit g++ probe timed out after 60 seconds\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            if probe.returncode != 0:
                self.skipTest(
                    "32-bit g++ multilib is unavailable; BMM350 SIL requires -m32\n"
                    f"stdout:\n{probe.stdout}\nstderr:\n{probe.stderr}"
                )

            sketch_dir = SKETCH_DIR
            if mutation != "none":
                sketch_dir = tmp_path / "mutated_sketch"
                shutil.copytree(SKETCH_DIR, sketch_dir)
                fusion_header = sketch_dir / "mag_yaw_fusion.h"
                source = fusion_header.read_text()
                if mutation == "disabled":
                    old = "return gain * wrapDeg(mag_heading_deg - yaw_deg);"
                    new = "return 0.0f;"
                elif mutation == "inverted":
                    old = "return gain * wrapDeg(mag_heading_deg - yaw_deg);"
                    new = "return -gain * wrapDeg(mag_heading_deg - yaw_deg);"
                else:
                    old = (
                        "const float roll = roll_deg * PI / 180.0f;\n"
                        "  const float pitch = pitch_deg * PI / 180.0f;"
                    )
                    new = (
                        "const float roll = -roll_deg * PI / 180.0f;\n"
                        "  const float pitch = -pitch_deg * PI / 180.0f;"
                    )
                if source.count(old) != 1:
                    self.fail(
                        f"{mutation} mutation target was not found exactly once"
                    )
                fusion_header.write_text(source.replace(old, new, 1))

            executable = tmp_path / "test_mag_yaw_fusion"
            compile_command = [
                compiler, "-std=c++17", "-O0", "-g", "-Wall", "-m32",
                "-I", str(NATIVE_TEST_DIR / "shims"),
                "-I", str(sketch_dir),
                "-x", "c++",
                str(NATIVE_TEST_DIR / "test_mag_yaw_fusion.cpp"),
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
                    "BMM350 SIL compilation timed out after 60 seconds\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            if compiled.returncode != 0:
                self.fail(
                    "BMM350 SIL compilation failed\n"
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
                    "BMM350 SIL timed out after 20 seconds\n"
                    f"stdout:\n{error.stdout or ''}\nstderr:\n{error.stderr or ''}"
                )
            if completed.returncode == -signal.SIGSYS:
                qemu_i386 = shutil.which("qemu-i386-static")
                if qemu_i386 is None:
                    self.skipTest(
                        "32-bit SIL execution was SIGSYS-blocked and "
                        "qemu-i386-static is unavailable"
                    )
                command = [qemu_i386, str(executable)]
                completed = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    check=False,
                    timeout=20,
                )

            print(f"[BMM350-SIL] mutation={mutation} runner={' '.join(command)}")
            print(completed.stdout, end="")
            if completed.stderr:
                print(completed.stderr, end="")
            if completed.returncode != 0:
                self.fail(
                    f"BMM350 yaw-fusion SIL failed with mutation={mutation}"
                )


if __name__ == "__main__":
    unittest.main()
