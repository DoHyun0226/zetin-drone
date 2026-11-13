"""ESP32-CAM 스트림 스케치와 노트북 카메라 도구의 정적 검사.

이 시험은 아무것도 빌드하지 않는다. 컴파일로는 드러나지 않는 스펙의 금지 사항을 고정한다.
카메라 경로는 비행 UDP 포트를 절대 건드리지 않는다. 캡처 타임스탬프는 32비트 형식으로
출력하지 않는다. 고정 IP 는 비행 SoftAP 의 DHCP 풀 밖에 둔다.
"""

import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SKETCH_DIR = REPO_ROOT / "firmware" / "camera" / "esp32cam_stream"
SKETCH = SKETCH_DIR / "esp32cam_stream.ino"
CAMERA_SCRIPTS = (
    REPO_ROOT / "scripts" / "ground" / "camera_stream.py",
    REPO_ROOT / "scripts" / "ground" / "camera_record.py",
    REPO_ROOT / "scripts" / "ground" / "camera_mp4.py",
    REPO_ROOT / "scripts" / "drone_video.py",
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
        # 벤치(2026-11-02)에서 11 dBm 은 0.7-1.2 fps, ping 17-640 ms 로 측정됐다.
        # 19.5 dBm 은 7.9 fps, 18-91 ms.
        self.assertIn("CAM_WIFI_TX_POWER = WIFI_POWER_19_5dBm;", SKETCH.read_text(encoding="utf-8"))

    def test_vga_quality_10_by_default(self):
        # 사용자가 고른 기본값(2026-11-02). 벤치: VGA q10 = 15-22 kB, 2.8-7.7 fps. SVGA q10 은 1.3 fps 로 떨어졌다.
        # 28 kB 프레임이 5744 B TCP 송신 버퍼를 거치려면 약 5번 왕복해야 하기 때문이다.
        text = SKETCH.read_text(encoding="utf-8")
        self.assertIn("const framesize_t FRAME_SIZE = FRAMESIZE_VGA;", text)
        self.assertIn("const int JPEG_QUALITY = 10;", text)

    def test_xclk_12mhz_by_default(self):
        # 벤치 2026-11-02: XCLK 20 MHz 만으로(센서 전원 꺼짐) 카메라 링크가 약 70-110 kB/s,
        # 스트림이 5.0/0.65 fps 로 떨어졌다. 12 MHz 는 9.5-9.7 fps. 24 MHz 는 프레임을 깨뜨린다.
        text = SKETCH.read_text(encoding="utf-8")
        self.assertIn("const int CAM_XCLK_HZ = 12000000;", text)
        self.assertRegex(text, r"config\.xclk_freq_hz\s*=\s*CAM_XCLK_HZ;")

    def test_mirror_and_flip_are_applied_from_constants(self):
        text = SKETCH.read_text(encoding="utf-8")
        self.assertIn("const bool CAM_HMIRROR = true;", text)  # OV3660 보드: 영상이 좌우 반전되어 나왔다
        self.assertIn("const bool CAM_VFLIP = false;", text)
        self.assertRegex(text, r"set_hmirror\(\s*s\s*,\s*CAM_HMIRROR")
        self.assertRegex(text, r"set_vflip\(\s*s\s*,\s*CAM_VFLIP")

    def test_ci_compiles_camera_with_esp32cam_fqbn(self):
        text = CI.read_text(encoding="utf-8")
        self.assertIn("esp32:esp32:esp32cam", text)
        self.assertIn("firmware/camera/esp32cam_stream", text)


if __name__ == "__main__":
    unittest.main()
