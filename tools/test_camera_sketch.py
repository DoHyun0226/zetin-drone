"""Static guards for the ESP32-CAM stream sketch and the laptop camera tools.

These do not build anything. They pin the spec's anti-goals that a compile
cannot see: the camera path must never touch the flight UDP port, the capture
timestamp must not be printed with a 32-bit format, and the static IP must
stay outside the flight SoftAP's DHCP pool.
"""

import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SKETCH_DIR = REPO_ROOT / "firmware" / "camera" / "esp32cam_stream"
SKETCH = SKETCH_DIR / "esp32cam_stream.ino"
CAMERA_SCRIPTS = (
    REPO_ROOT / "scripts" / "ground" / "camera_stream.py",
    REPO_ROOT / "scripts" / "record_camera.py",
    REPO_ROOT / "scripts" / "camera_to_mp4.py",
)
CI = REPO_ROOT / ".github" / "workflows" / "ci.yml"

# ESP-IDF dhcpserver 의 기본 풀: SoftAP 주소 + 1 부터 DHCPS_MAX_LEASE(0x64) 개.
DHCP_POOL = range(2, 102)


class CameraSketchStaticTest(unittest.TestCase):
    def test_never_touches_flight_udp_port(self):
        self.assertTrue(SKETCH.exists(), f"missing {SKETCH}")
        sources = [p for p in sorted(SKETCH_DIR.iterdir()) if p.is_file()]
        sources += [p for p in CAMERA_SCRIPTS if p.exists()]
        for path in sources:
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path.relative_to(REPO_ROOT)):
                self.assertNotIn("4210", text)
                self.assertNotIn("WiFiUDP", text)

    def test_timestamp_not_printed_with_int_format(self):
        self.assertNotIn('"%d.%06d"', SKETCH.read_text(encoding="utf-8"))

    def test_static_ip_outside_dhcp_pool(self):
        text = SKETCH.read_text(encoding="utf-8")
        hosts = [int(n) for n in re.findall(r"IPAddress\(\s*192\s*,\s*168\s*,\s*4\s*,\s*(\d+)\s*\)", text)]
        self.assertIn(200, hosts, "camera static IP 192.168.4.200 not found")
        self.assertNotIn(200, DHCP_POOL)

    def test_full_tx_power_by_default(self):
        # 11 dBm measured 0.7-1.2 fps with 17-640 ms ping on the bench
        # (2026-11-02); 19.5 dBm gave 7.9 fps and 18-91 ms.
        self.assertIn("CAM_WIFI_TX_POWER = WIFI_POWER_19_5dBm;", SKETCH.read_text(encoding="utf-8"))

    def test_vga_quality_10_by_default(self):
        # user-chosen default (2026-11-02); bench: VGA q10 = 15-22 kB, 2.8-7.7 fps; SVGA q10 fell to 1.3 fps
        # because a 28 kB frame needs ~5 round trips through the 5744 B TCP send buffer
        text = SKETCH.read_text(encoding="utf-8")
        self.assertIn("const framesize_t FRAME_SIZE = FRAMESIZE_VGA;", text)
        self.assertIn("const int JPEG_QUALITY = 10;", text)

    def test_xclk_12mhz_by_default(self):
        # bench 2026-11-02: XCLK 20 MHz alone (sensor powered down) cut the camera link to
        # ~70-110 kB/s and the stream to 5.0/0.65 fps; 12 MHz gave 9.5-9.7 fps. 24 MHz corrupts frames.
        text = SKETCH.read_text(encoding="utf-8")
        self.assertIn("const int CAM_XCLK_HZ = 12000000;", text)
        self.assertRegex(text, r"config\.xclk_freq_hz\s*=\s*CAM_XCLK_HZ;")

    def test_mirror_and_flip_are_applied_from_constants(self):
        text = SKETCH.read_text(encoding="utf-8")
        self.assertIn("const bool CAM_HMIRROR = true;", text)  # OV3660 board: image came out mirrored
        self.assertIn("const bool CAM_VFLIP = false;", text)
        self.assertRegex(text, r"set_hmirror\(\s*s\s*,\s*CAM_HMIRROR")
        self.assertRegex(text, r"set_vflip\(\s*s\s*,\s*CAM_VFLIP")

    def test_ci_compiles_camera_with_esp32cam_fqbn(self):
        text = CI.read_text(encoding="utf-8")
        self.assertIn("esp32:esp32:esp32cam", text)
        self.assertIn("firmware/camera/esp32cam_stream", text)


if __name__ == "__main__":
    unittest.main()
