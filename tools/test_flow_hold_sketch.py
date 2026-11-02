"""The flow-hold sketch is a copy of the main cascade sketch.

Headers both sketches share must stay byte-identical. A fix made in only one
copy is exactly how the flix firmware reintroduced bugs that were already
fixed, so this test fails until the same change lands in both folders.
"""

import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
BASE = REPO_ROOT / "firmware" / "flight" / "dual_imu_cascade_pwm"
FLOW = REPO_ROOT / "firmware" / "flight" / "dual_imu_cascade_flow_hold"

SHARED_HEADERS = (
    "control_allocator.h",
    "failsafe_land.h",
    "mag_yaw_fusion.h",
    "msp_sensor.h",
    "yaw_authority.h",
    "yaw_command.h",
)


class FlowHoldSketchTest(unittest.TestCase):
    def test_shared_headers_match_base_sketch(self):
        for name in SHARED_HEADERS:
            with self.subTest(header=name):
                self.assertEqual(
                    (FLOW / name).read_bytes(),
                    (BASE / name).read_bytes(),
                    f"{name} differs between dual_imu_cascade_pwm and "
                    "dual_imu_cascade_flow_hold; apply the change to both",
                )

    def test_base_headers_are_all_listed(self):
        base_headers = {path.name for path in BASE.glob("*.h")}
        self.assertEqual(base_headers, set(SHARED_HEADERS))


if __name__ == "__main__":
    unittest.main()
