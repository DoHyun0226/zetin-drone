# 카메라 보드 펌웨어

[`esp32cam_stream/`](esp32cam_stream/)은 "ESP32-CAM 호환보드 + CH340 업로드 하판"용
영상 스트림 스케치다. 비행 ESP32의 SoftAP `Drone_Tuning`에 스테이션으로 붙어 노트북으로
MJPEG를 보낸다. 노트북에서는 [`scripts/record_camera.py`](../../scripts/record_camera.py)로
받아 녹화한다. 설계는
[카메라 스트리밍 설계](../../docs/superpowers/specs/2026-11-02-camera-streaming-design.md)에 있다.

**실험 단계다.** 2026-11-02 벤치에서 OV3660 보드(PSRAM 있음)가 USB 전원으로 부팅·접속·스트림까지
동작했다. XCLK 20 MHz 당시에는 VGA·품질 10에서 장면에 따라 2.8–7.7 fps·최장 공백 3 s였고, 기본값을 12 MHz로 바꾼 뒤 재검증에서는 9.3·8.4 fps(30 s 두 번, 일련번호 틈·재접속 0)였다. BEC 전원과 비행 링크 영향(벤치 2)은 아직 시험하지 않았다.

## 보드 설정

| 항목 | 값 |
|---|---|
| 보드(FQBN) | AI Thinker ESP32-CAM (`esp32:esp32:esp32cam`) |
| 파티션 | 기본값(Huge APP) |
| PSRAM | 기본값(사용). 없는 보드면 스케치가 QVGA·버퍼 1개로 내려간다 |
| 카메라 핀맵 | AI-Thinker. 스케치 안에 직접 정의돼 있다 |

```bash
arduino-cli compile --fqbn esp32:esp32:esp32cam firmware/camera/esp32cam_stream/
arduino-cli upload  --fqbn esp32:esp32:esp32cam -p /dev/ttyUSB0 firmware/camera/esp32cam_stream/
```

업로드는 CH340 하판(micro-B)에 꽂아서 한다. 업로드가 시작되지 않으면 하판의 IO0 버튼을
누른 채 RST를 눌렀다 뗀다. 업로드가 끝나면 RST를 한 번 눌러 실행한다.

## 네트워크와 엔드포인트

- 접속: `Drone_Tuning`, 고정 IP `192.168.4.200`. SoftAP DHCP 풀(.2–.101) 밖이다.
- `http://192.168.4.200:81/stream`: MJPEG 스트림이다. 클라이언트는 하나만 받는다. 두 번째
  요청은 첫 스트림이 끝날 때까지 기다린다.
- `http://192.168.4.200/status`: 설정과 카운터를 담은 JSON이다.
- 비행 링크의 UDP 포트에는 아무것도 보내지 않는다. 비행 펌웨어는 마지막 UDP 발신자에게
  회신하므로, 거기에 보내면 텔레메트리 목적지가 바뀐다.

`/status` 키:

| 키 | 뜻 |
|---|---|
| `frame_size`, `jpeg_quality`, `fps_cap`, `byte_rate_cap` | 현재 설정 |
| `psram`, `sensor_pid` | 보드·센서 확인용(OV2640 = `0x0026`, OV3660 = `0x3660`) |
| `reset_reason` | 마지막 리셋 원인. `brownout`이면 BEC 전원 문제를 의심한다 |
| `frames_sent`, `effective_fps`, `avg_frame_bytes` | 송신량(최근 5 s 기준 fps·평균 크기) |
| `frames_capture_failed`, `camera_reinits` | 캡처 실패. 연속 10회 실패하면 카메라를 재초기화한다 |
| `sends_failed` | 스트림 송신 실패(연결 끊김) 횟수 |
| `rssi`, `wifi_reconnects` | 링크 상태 |
| `stream_active`, `uptime_ms` | 스트림 진행 여부, 부팅 후 시간 |

## 설정 상수

스케치 위쪽에 있다.

| 상수 | 기본값 | 바꾸는 때 |
|---|---|---|
| `FRAME_SIZE` | VGA 640×480 | SVGA는 XCLK 20 MHz 시절 1.3 fps까지 떨어졌다. 12 MHz에서는 다시 재지 않았다 |
| `JPEG_QUALITY` | 10 | 값이 작을수록 고화질이고 프레임이 커진다(VGA·10 ≈ 15–22 kB). 끊김이 잦으면 12–14로 올린다 |
| `FPS_CAP` | 10 | |
| `BYTE_RATE_CAP` | 187,500 B/s (1.5 Mbps) | 조종 링크 보호용 평균 상한 |
| `CAM_WIFI_TX_POWER` | 19.5 dBm | 11 dBm에서는 지연이 커져 1 fps 안팎으로 떨어졌다(스케치 주석) |
| `CAM_XCLK_HZ` | 12 MHz | 20 MHz에서는 클럭만으로 카메라 링크가 느려져 5 fps 안팎이었다. 10 MHz도 나빴고 24 MHz는 프레임이 깨진다. 바꾸면 벤치에서 fps·ping을 다시 잰다 |
| `CAM_HMIRROR`, `CAM_VFLIP` | `true`, `false` | 기체에 단 방향에 맞춰 바꾼다 |

## 시리얼 로그 (115200)

- `[BOOT] reset reason …` / `psram …` / `sensor PID …`: 부팅 정보
- `[WIFI] connected` / `disconnected` / `restart`: 접속 상태. 10 s 넘게 끊기면 접속을 처음부터 다시 한다
- `[SUM] fps … avg … rssi …`: 5 s마다 출력하는 요약

## 비행 전

카메라를 켠 채로 비행하기 전에 설계 문서 7절의 벤치 1–3을 통과해야 한다. 그중 벤치 2는
프롭을 떼고 무장한 상태에서 영상 OFF/ON의 RC 링크를 비교하는 시험이다.
