# 카메라 영상 스트리밍·노트북 녹화 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** ESP32-CAM 보드가 비행 ESP32 SoftAP로 MJPEG를 내보내면, 노트북 도구가 그것을 녹화하고 비행 로그 시각에 맞출 수 있게 만든다.

**Architecture:**
- **카메라 스케치:** HTTP 서버 두 개를 띄운다. 80번은 `/status`, 81번은 `/stream`이다.
  파트 헤더 포맷과 토큰 버킷은 순수 헤더로 빼서 호스트 `-m32`로 시험한다.
- **노트북 쪽:** stdlib만 쓰는 `camera_stream.py`(파서·CSV 스키마·시각 맞춤 수학) 위에 CLI
  두 개를 얹는다. `record_camera.py`(녹화)와 `camera_to_mp4.py`(후처리)다. OpenCV는
  미리보기에서만 lazy import한다.

**Tech Stack:**
- 펌웨어: Arduino-ESP32 3.3.x, `esp_camera`, `esp_http_server`
- 노트북: Python 3.12 stdlib(`http.client`, `csv`), OpenCV(미리보기), ffmpeg(mp4)
- 시험: `unittest`, g++ `-m32` 네이티브 시험

**Spec:** [`docs/superpowers/specs/2026-11-02-camera-streaming-design.md`](../specs/2026-11-02-camera-streaming-design.md)

## Global Constraints

**네트워크·영상 설정**
- SoftAP `Drone_Tuning`, 비밀번호는 비행 펌웨어와 같은 `12345678`, 게이트웨이 `192.168.4.1`
- 카메라 고정 IP `192.168.4.200/24`. DHCP 풀 .2–.101(`DHCPS_MAX_LEASE 0x64`) 밖이다.
- 기본값: `FRAMESIZE_HVGA`(480×320), JPEG 품질 12, fps 상한 10(프레임 간 최소 100,000 µs)
- 바이트율 상한 1.5 Mbps = **187,500 B/s**, 조각 4096 B, 버킷 용량 8192 B
- PSRAM이 없으면 `FRAMESIZE_QVGA`, `fb_count=1`, `CAMERA_FB_IN_DRAM`으로 떨어진다.
- 녹화기 기본 URL `http://192.168.4.200:81/stream`

**금지 사항**
- 카메라 스케치·녹화기·후처리 어디에서도 UDP 4210을 쓰지 않는다. `WiFiUDP` 금지.
- 비행 펌웨어(`firmware/flight/**`)는 수정하지 않는다.
- `docs/hardware_configuration.md`는 수정하지 않는다(사용자 미커밋 편집).

**시각·파일 형식**
- 파트 헤더 `X-Timestamp`는 부팅 후 µs를 `<sec>.<usec 6자리>`로 쓴다. `%d`에 64비트를
  넘기지 않는다.
- 노트북 시각 문자열은 비행 로그와 같은 `datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]`
- 출력 파일: `logs/camera_<%Y-%m-%d_%H%M%S>.mjpeg` / `.csv`. 후처리 결과는
  `..._aligned.csv` / `.mp4`.

**빌드·시험**
- 카메라 FQBN은 `esp32:esp32:esp32cam`이다. 비행 스케치 루프(`esp32s3`)와 분리한다.
- 시험은 `tools/test_*.py`(unittest)에 둔다. CI는 skip을 실패로 보므로 ffmpeg·OpenCV
  유무에 따라 skip하는 시험을 만들지 않는다.

**커밋**
- 형식은 `feat(camera): …` / `ci(camera): …` / `docs(camera): …`, 소문자 동사로 시작하는
  영어 한 문장이다. 끝에 Co-Authored-By 줄을 붙인다.
- **각 커밋 전 사용자 확인**을 받는다. 사용자가 이 계획 범위의 커밋을 미리 허락했으면
  그대로 진행한다.
- 파일 단위로만 `git add`한다.

## Review Focus

1. **경계 문자열이 청크 경계에 걸려 쪼개져 오는 경우**: 파서는 같은 프레임을 정확히
   한 번 내야 한다. 시험은 Task 3의 `test_split_at_every_byte`.
2. **JPEG 바이트 안에 경계 문자열과 같은 바이트열이 들어 있는 경우**: Content-Length로
   읽으므로 잘리면 안 된다. 시험은 Task 3의 `test_payload_containing_boundary`.
3. **녹화 중 카메라 재부팅(seq·캡처 시각이 되감김)**: 새 구간으로 분리되고, 틈으로 세지 않으며,
   frame 번호는 이어져야 한다. 시험은 Task 5의 `test_reconnect_starts_new_segment`.
4. **녹화 중 노트북 벽시계 점프(NTP)**: 후처리가 경고해야 한다. 시험은 Task 4의
   `test_clock_jump_detected`.
5. **조각이 버킷 용량보다 큰 경우(헤더+조각)**: 영원히 기다리지 않아야 한다. 시험은
   Task 1의 `bucket_oversized_request_waits_for_capacity`.

---

## Task 0: 브랜치

- [ ] **Step 1:** `git switch -c feat/camera-stream main`을 실행한다.
  - 미커밋 `docs/hardware_configuration.md`가 충돌하면 멈추고 사용자에게 묻는다.
  - 설계 문서와 이 계획(untracked)은 그대로 따라온다.
- [ ] **Step 2:** 설계 문서와 이 계획만 `git add`하고 커밋한다(사용자 확인).
  메시지: `docs(camera): add the camera streaming design and plan`

## Task 1: 펌웨어 순수 헤더 — 파트 헤더 포맷·토큰 버킷

**Files:**
- Create: `firmware/camera/esp32cam_stream/stream_format.h`, `firmware/camera/esp32cam_stream/stream_pacing.h`
- Test: `tools/native_tests/test_camera_stream.cpp`, `tools/test_camera_stream_native.py`

**Interfaces:**
- Produces (둘 다 `<cstdint>`·`<cstdio>`만 쓴다. Arduino 헤더 금지):
  - `stream_format.h`:
    - `constexpr const char* kStreamBoundary = "zetincamframe";`
    - `size_t formatBoundary(char* buf, size_t cap)`: `"\r\n--zetincamframe\r\n"`을 쓰고 길이를 돌려준다.
    - `size_t formatPartHeader(char* buf, size_t cap, uint32_t jpegLen, int64_t captureUs, uint32_t seq)`
      : `"Content-Type: image/jpeg\r\nContent-Length: <len>\r\nX-Timestamp: <sec>.<usec06>\r\nX-Frame-Seq: <seq>\r\n\r\n"`
      을 쓴다. 넘치면 0을 돌려준다. `long long`·`unsigned long`으로 명시 캐스트한다.
  - `stream_pacing.h`: `class TokenBucket`
    - `TokenBucket(uint32_t rateBytesPerS, uint32_t capacityBytes)`
    - `void reset(int64_t nowUs)`: 가득 찬 상태로 시작한다.
    - `uint32_t waitUs(int64_t nowUs, uint32_t bytes)`: 지금 보내려면 기다려야 할 µs다. 올림한다.
      요청이 용량보다 크면 용량으로 클램프한다. 그래서 대기는 항상 유한하다.
    - `void consume(int64_t nowUs, uint32_t bytes)`: 토큰을 쓴다. 토큰이 음수까지 내려갈 수 있다.
      클램프하지 않은 실제 바이트를 쓴다.

- [ ] **Step 1: 실패하는 네이티브 시험 작성.** `tools/native_tests/test_camera_stream.cpp`는
  `test_msp_sensor.cpp`와 같은 `runCase`/`CHECK` 형식을 쓴다. 케이스는 다음과 같다.
  - `part_header_exact_bytes`: `formatPartHeader(buf,256,12345,3000000123LL,7)`의 결과가
    `"Content-Type: image/jpeg\r\nContent-Length: 12345\r\nX-Timestamp: 3000.000123\r\nX-Frame-Seq: 7\r\n\r\n"`이다.
  - `part_header_large_uptime`: `captureUs = 5000000000123LL`(약 58일)이면 `X-Timestamp: 5000000.000123`이다.
  - `part_header_too_small_returns_zero`: `cap = 20`이면 0을 돌려준다.
  - `boundary_bytes`: `"\r\n--zetincamframe\r\n"`, 길이 19
  - `bucket_starts_full`: `TokenBucket b(187500, 8192); b.reset(0);`에서 `waitUs(0,8192) == 0`이다.
  - `bucket_wait_after_drain`: `consume(0,8192)` 뒤 `waitUs(0,4096) == 21846`이다.
  - `bucket_refills_to_capacity_only`: drain한 뒤 10 s가 지나도 `waitUs(10'000'000,8192) == 0`이고,
    `consume`을 두 번 하면 다시 대기가 생긴다(용량을 넘어 쌓이지 않는다).
  - `bucket_oversized_request_waits_for_capacity`: drain 뒤 `waitUs(0, 20000) == waitUs(0, 8192)`이고, 그 값은 유한하다(43,691 µs).
- [ ] **Step 2:** `tools/test_camera_stream_native.py`를 만든다. `tools/test_msp_sensor.py`를 복사하고
  `SKETCH_DIR = REPO_ROOT/"firmware"/"camera"/"esp32cam_stream"`, 소스 파일 이름, 메시지 접두사만
  바꾼다. 실행 명령은 `python -m unittest tools.test_camera_stream_native -v`이다. 헤더가
  없으므로 컴파일 실패로 FAIL해야 한다.
- [ ] **Step 3:** 두 헤더를 구현한다. 토큰은 int64 '바이트×10⁶'으로 둔다. 리필은
  `elapsedUs × rate`로 계산해 부동소수를 쓰지 않는다.
- [ ] **Step 4:** 같은 명령으로 실행해 PASS를 확인한다. 8케이스 모두 `[PASS]`여야 한다.
- [ ] **Step 5:** 커밋한다(사용자 확인). 메시지: `feat(camera): add the stream part header and token bucket for the camera board`

## Task 2: 카메라 스케치 + CI + 펌웨어 문서

**Files:**
- Create: `firmware/camera/esp32cam_stream/esp32cam_stream.ino`, `firmware/camera/README.md`
- Modify: `.github/workflows/ci.yml`(firmware 잡), `docs/firmware_catalog.md`, `firmware/README.md`
- Test: `tools/test_camera_sketch.py`

**Interfaces:**
- Consumes: Task 1의 `formatBoundary`, `formatPartHeader`, `TokenBucket`
- Produces:
  - HTTP `GET :81/stream`: `Content-Type: multipart/x-mixed-replace;boundary=zetincamframe`
  - HTTP `GET :80/status`: JSON 키는 `frame_size, jpeg_quality, fps_cap, byte_rate_cap, psram, sensor_pid, reset_reason, frames_sent, frames_capture_failed, camera_reinits, sends_failed, avg_frame_bytes, effective_fps, rssi, wifi_reconnects, stream_active, uptime_ms`

**스케치가 정해야 하는 값.** 본문은 구현자가 쓴다. CameraWebServer 예제
(`~/.arduino15/packages/esp32/hardware/esp32/3.3.7/libraries/ESP32/examples/Camera/CameraWebServer/`)를
참고하되 웹 UI·얼굴인식·`%d.%06d` 포맷은 가져오지 않는다.
- **AI-Thinker 핀:** PWDN 32, RESET −1, XCLK 0, SIOD 26, SIOC 27, Y9..Y2 = 35,34,39,36,21,19,18,5,
  VSYNC 25, HREF 23, PCLK 22. 스케치 안에 직접 `#define`한다.
- **카메라 설정:** `xclk_freq_hz = 20000000`, `PIXFORMAT_JPEG`. PSRAM이면 HVGA, `fb_count=2`,
  `CAMERA_GRAB_LATEST`, `CAMERA_FB_IN_PSRAM`이다. 아니면 Global Constraints의 폴백을 쓴다.
  부팅 로그에 `sensor PID 0x%04x`, `psram yes|no`, `reset reason <esp_reset_reason()>`를 찍는다.
  브라운아웃 리셋(`ESP_RST_BROWNOUT`)을 구분하려는 것이다.
- **캡처 복구:** `esp_camera_fb_get()`이 연속 10회 실패하면 `esp_camera_deinit()` → `esp_camera_init()`을
  다시 하고 `camera_reinits`를 1 올린다.
- **WiFi:** `WiFi.mode(WIFI_STA)`, `WiFi.config(192.168.4.200, 192.168.4.1, 255.255.255.0)`,
  `setSleep(false)`, `setAutoReconnect(true)`, `WiFi.setTxPower(CAM_WIFI_TX_POWER)`.
  - `CAM_WIFI_TX_POWER`의 기본값은 `WIFI_POWER_11dBm`다. AP와 수 cm 거리라 최대 출력이 필요 없고,
    낮추면 송신 피크 전류와 근접 간섭이 준다. 벤치 1에서 RSSI가 나쁘면 올린다.
  - 자동 재접속에 맡긴다. 10 s 이상 끊겨 있으면 `disconnect()` → `config()` → `begin()` 전체를 다시 한다.
    이 수동 재시작은 10 s에 한 번만 한다. 카메라가 비행 보드보다 먼저 켜진 경우도 이 경로로 복구한다.
  - `wifi_reconnects`는 끊김 → 연결 전이마다 1 올린다.
- **HTTP:** `HTTPD_DEFAULT_CONFIG()`로 80번을 띄운다. 81번은 `server_port += 1; ctrl_port += 1`로 띄운다.
  둘 다 `send_wait_timeout = 2`(s)다.
- **대기 함수:** 1000 µs 이상은 `vTaskDelay(pdMS_TO_TICKS(ms))`로 CPU를 양보하고, 깨어난 뒤 시각을
  다시 확인한다. 1000 µs 미만만 `delayMicroseconds`를 쓴다.
- **스트림 루프.** 반복마다 다음 순서로 진행한다.
  1. 고정 주기 `next_frame_us += 100000`까지 대기한다. 이미 뒤처졌으면 `next_frame_us = now`로 리셋한다.
  2. `esp_camera_fb_get()`을 부른다. 실패하면 `frames_capture_failed`를 올리고 10 ms 쉰 뒤 처음으로 돌아간다.
  3. 경계와 파트 헤더를 보낸다. seq는 `frames_sent`를 쓴다.
  4. JPEG를 4096 B 조각으로 나눠 보낸다. 경계·헤더·각 조각마다 그 직전에
     `waitUs` → `delayMicroseconds`/`vTaskDelay` → `consume`을 거친다.
  5. `httpd_resp_send_chunk`가 실패하면 `sends_failed`를 올리고 fb를 반환한 뒤 루프를 끝낸다.
  6. 성공하면 fb를 반환하고 `frames_sent`를 올린다.
- **루프 상태:** 스트림 진입 때 `TokenBucket.reset`을 하고 `stream_active=true`로 둔다. 나갈 때
  `false`로 되돌린다.
- **시리얼(115200):** 5 s마다 `fps / avg_bytes / rssi / reconnects / capture_fail / send_fail`을 한 줄로 찍는다.

- [ ] **Step 1: 실패하는 정적 시험 작성.** `tools/test_camera_sketch.py`에 다음을 둔다.
  - `test_never_touches_flight_udp_port`: 스케치 디렉터리의 모든 파일, `scripts/record_camera.py`,
    `scripts/camera_to_mp4.py`, `scripts/camera_stream.py`에 `4210`과 `WiFiUDP`가 없다.
    아직 없는 파일은 건너뛰고, 스케치 `.ino`는 반드시 존재해야 한다.
  - `test_timestamp_not_printed_with_int_format`: `.ino`에 `"%d.%06d"`가 없다.
  - `test_static_ip_outside_dhcp_pool`: `.ino`에서 `IPAddress(192, 168, 4, N)` 꼴의 N을
    찾는다. 그중 카메라 주소가 200이고 2..101 밖이다.
  - `test_ci_compiles_camera_with_esp32cam_fqbn`: `ci.yml`에
    `esp32:esp32:esp32cam`과 `firmware/camera/esp32cam_stream`이 있다.
- [ ] **Step 2:** `python -m unittest tools.test_camera_sketch -v`로 실행해 FAIL을 확인한다.
- [ ] **Step 3:** 스케치를 구현한다. `ci.yml` firmware 잡의 비행 루프 뒤에 단계를 추가한다.
  - 단계 이름: `Compile camera sketch`
  - 명령: `arduino-cli compile --warnings all --fqbn esp32:esp32:esp32cam --build-path /tmp/zetin-esp32cam_stream firmware/camera/esp32cam_stream/`
- [ ] **Step 4: 문서.**
  - `docs/firmware_catalog.md`
    - 머리말의 스케치 수를 27개로 고치고, '실험' 분류를 문장에 반영한다.
    - 표에 행을 추가한다: 수명주기 `실험`, 대상 MCU `ESP32 (ESP32-CAM, AI-Thinker 핀맵)`, 센서 `카메라(OV2640 가정)`,
      모터 `없음`, 메모 "HTTP MJPEG :81, 상태 :80, 고정 IP 192.168.4.200. 4210 미사용. CI는 esp32cam FQBN으로 별도 컴파일. 실기 미검증", 링크
      `../firmware/camera/esp32cam_stream/`
  - `firmware/README.md`: `## 카메라 보드` 절을 둔다. 두세 줄 요약과 `camera/README.md` 링크를 넣는다.
  - `firmware/camera/README.md`: 보드 선택(AI Thinker ESP32-CAM), 기본 파티션(Huge APP), PSRAM,
    CH340 하판 업로드(IO0 버튼), 시리얼 로그 해석, `/status` 키, 설정 상수 위치를 적는다.
    마지막에 "비행 전 링크 비교는 설계 문서 7절" 링크를 단다.
- [ ] **Step 5: 확인.**
  - `python -m unittest tools.test_camera_sketch -v`가 PASS여야 한다.
  - `python tools/check_repo_layout.py`가 `repository layout checks passed`여야 한다.
  - 로컬에서 Step 3의 arduino-cli 명령을 실행한다(로컬 코어는 3.3.7). 경고 없이 성공해야 한다.
    경고가 나면 고친다.
- [ ] **Step 6:** 커밋한다(사용자 확인). 메시지: `feat(camera): add the ESP32-CAM MJPEG stream sketch and compile it in CI`

## Task 3: `camera_stream.py` — multipart 파서·CSV 스키마·요약

**Files:**
- Create: `scripts/camera_stream.py`
- Test: `tools/test_camera_stream.py`

**Interfaces:**
- Produces:
  - `class StreamFormatError(ValueError)`
  - `@dataclass(frozen=True) class StreamPart: jpeg: bytes; capture_s: float | None; seq: int | None; tag: object = None`
    (시험의 3인자 생성은 tag=None과 비교한다)
    - `tag`은 그 파트의 **헤더가 완성된** feed 호출에 넘긴 값이다. 녹화기는 여기에 수신 시각을 넣는다.
      카메라 송신 속도 제한 때문에 마지막 바이트 기준 시각에는 프레임 크기/187.5 kB/s(최대 약 100 ms)가
      더해진다. 헤더 도착 시각을 쓰면 이 지연이 시각 맞춤에서 빠진다.
  - `def boundary_from_content_type(value: str) -> bytes`: `multipart/x-mixed-replace;boundary=zetincamframe` → `b"zetincamframe"`. 공백·따옴표를 허용하고, 없으면 `StreamFormatError`
  - `class MjpegParser: __init__(self, boundary: bytes)`, `feed(self, data: bytes, tag: object = None) -> list[StreamPart]`,
    `finish(self) -> bool`: 끊길 때 부른다. 완성되지 않은 파트가 남아 있었으면 True를 돌려주고 버린다.
  - `INDEX_FIELDS = ("frame", "segment", "camera_seq", "offset", "size", "capture_s", "recv_epoch_s", "recv_monotonic_s", "recv_time")`
  - `def format_recv_time(epoch_s: float) -> str`
  - `@dataclass(frozen=True) class RecordingSummary: frames: int; segments: int; seq_gaps: int; effective_fps: float; longest_gap_s: float; reconnect_gaps_s: tuple[float, ...]`
    - `longest_gap_s`는 구간 **안**의 최장 공백이다.
    - `reconnect_gaps_s`는 구간 k의 마지막 수신부터 구간 k+1의 첫 수신까지다. 둘 다 `recv_monotonic_s`로 계산한다.
  - `def summarize(rows: list[dict]) -> RecordingSummary`: rows의 값은 문자열이어도 된다(CSV로 읽은 값).
    fps와 공백은 벽시계 점프에 영향받지 않도록 `recv_monotonic_s`로 계산한다.

**파서 규칙**
- 경계 줄(`--<boundary>`, 앞에 `\r\n`이 있어도 되고 없어도 된다)을 찾는다.
- 헤더는 `\r\n\r\n`까지이고 이름은 대소문자를 무시한다.
- `Content-Length` 바이트만큼 정확히 읽는다.
- `Content-Length`가 없거나 숫자가 아니면 `StreamFormatError`다.
- `X-Timestamp`·`X-Frame-Seq`가 없으면 `None`이다.
- 본문 직후의 바이트는 반드시 `\r\n--<boundary>`로 시작해야 한다. 아니면 길이 불일치로 보고
  `StreamFormatError`를 낸다. 단, 바이트가 아직 덜 왔으면 기다린다.
- 경계 앞의 쓰레기 바이트는 버린다.

- [ ] **Step 1: 실패하는 시험 작성.** 공통 픽스처 `make_stream(parts)`는 Task 2의 파트 형식대로 바이트를 만든다.
  - `test_single_part`: JPEG `b"\xff\xd8abc\xff\xd9"`, ts `"3000.000123"`, seq 7을 넣으면 `StreamPart(jpeg, 3000.000123, 7)`이 나온다.
  - `test_split_at_every_byte`: 3파트 스트림을 1바이트씩 feed해도 3파트가 순서대로 정확히 나온다.
  - `test_payload_containing_boundary`: JPEG 안에 `b"\r\n--zetincamframe\r\n"`이 들어 있어도 원본과 같다.
  - `test_missing_content_length_raises` / `test_non_numeric_content_length_raises`
  - `test_missing_optional_headers_are_none`
  - `test_header_names_case_insensitive`: `content-length`, `x-timestamp`
  - `test_boundary_from_content_type`: 정상, 따옴표가 있는 경우, 경계가 없는 경우(raise)
  - `test_tag_is_header_arrival_chunk`: 헤더를 담은 청크에 tag "A", 본문 나머지 청크에 "B"를 주면 결과 tag가 "A"다.
  - `test_length_mismatch_raises`: Content-Length가 실제 본문보다 2 작으면 `StreamFormatError`다.
  - `test_finish_reports_truncated_part`: 본문 중간에서 `finish()`를 부르면 True이고, 완결된 직후에 부르면 False다.
  - `test_format_recv_time_matches_flight_log`: 결과가
    `datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]`와 같다.
  - `test_summarize_counts_gaps_within_segment_only`: 구간 0의 seq가 0,1,3이고 구간 1의 seq가
    0,1이면 `seq_gaps == 1`, `segments == 2`이다.
  - `test_summarize_fps_and_longest_gap`: 구간 하나에 `recv_monotonic_s` 0, 0.1, 0.2, 0.9를 넣으면
    `longest_gap_s == 0.7`(±1e-9)이고 `effective_fps == 3/0.9`이다. 여러 구간이면 각 구간
    `(n-1)/span`의 프레임 가중 평균이 아니라 `Σ(n-1)/Σspan`이다.
  - `test_summarize_reconnect_gaps`: 구간 0이 0–1.0 s, 구간 1이 3.5–4.0 s이면 `reconnect_gaps_s == (2.5,)`이고
    `longest_gap_s`에는 2.5가 들어가지 않는다.
- [ ] **Step 2:** `python -m unittest tools.test_camera_stream -v`로 실행해 FAIL(ImportError)을 확인한다.
- [ ] **Step 3:** 구현한다. stdlib만 쓴다. 파서는 내부 `bytearray` 버퍼와 상태
  (`seek_boundary` / `headers` / `body`)로 동작한다.
- [ ] **Step 4:** 같은 명령으로 실행해 PASS를 확인한다.
- [ ] **Step 5:** 커밋한다(사용자 확인). 메시지: `feat(camera): parse the MJPEG stream and summarize a recording index`

## Task 4: `camera_stream.py` — 시각 맞춤 수학

**Files:**
- Modify: `scripts/camera_stream.py`
- Test: `tools/test_camera_stream.py`(케이스 추가)

**Interfaces:**
- Produces:
  - `def estimate_offsets(rows) -> dict[int, float]`: 구간별 `min(recv_epoch_s − capture_s)`다. `capture_s`가 빈 행은 제외한다.
  - `def drift_report(rows, window_s: float = 60.0) -> dict[int, float | None]`: 구간마다
    (마지막 window의 최소 오프셋 − 첫 window의 최소 오프셋)이다. 구간 길이가 `2×window_s`보다 짧으면 `None`이다.
  - `def clock_jumps(rows, threshold_s: float = 0.5) -> list[int]`: 연속 행에서
    `(recv_epoch_s − recv_monotonic_s)`가 threshold보다 크게 변한 행의 `frame` 번호들
  - `def aligned_rows(rows) -> list[dict]`: 입력 열에 `capture_epoch_s`와 `capture_time`(format_recv_time 형식)을 더한다.
  - `def frame_durations(aligned: list[dict], last_s: float = 0.1, min_s: float = 0.001) -> list[float]`:
    다음 프레임의 `capture_epoch_s`와의 차이다. 구간 경계에서도 같은 식을 쓴다(둘 다 노트북 시계 기준이다).
    `min_s` 미만은 `min_s`로 하고, 마지막 프레임은 `last_s`다. `capture_s`가 없는 행은 `recv_epoch_s`로 대체한다.

- [ ] **Step 1: 실패하는 시험 추가.**
  - `test_offset_is_min_delay`: capture가 0,0.1,0.2이고 recv가 100.05,100.30,100.21이면 오프셋은 100.01(±1e-9)이다.
  - `test_offsets_per_segment`: 두 구간의 오프셋이 각각 따로 나온다.
  - `test_drift_report_short_segment_none` / `test_drift_report_linear_drift`: 오프셋이
    300 s 동안 선형으로 0.03 s 증가(`1e-4·t`, 10 Hz 행)하면 결과는 0.024다(±0.001).
    첫 창의 최소는 t=0일 때 0이고, 마지막 창(240–300 s)의 최소는 t≈240일 때 0.024이기 때문이다.
  - `test_clock_jump_detected`: 행 50부터 `recv_epoch_s`만 +2 s 점프하면 `[50]`이다.
  - `test_frame_durations`: 시각 0, 0.1, 0.1000001, 0.3이면 `[0.1, 0.001, 0.2, 0.1]`(±1e-6)이다.
  - `test_aligned_rows_adds_columns`
- [ ] **Step 2:** FAIL을 확인한다. **Step 3:** 구현한다. **Step 4:** PASS를 확인한다(Task 3 케이스 포함 전체).
- [ ] **Step 5:** 커밋한다(사용자 확인). 메시지: `feat(camera): estimate the camera-to-laptop clock offset and frame durations`

## Task 5: `record_camera.py` — 녹화 CLI

**Files:**
- Create: `scripts/record_camera.py`
- Modify: `requirements.txt`(`opencv-python>=4.8` 추가), `scripts/README.md`(카메라 절), `.gitignore`(`logs/*.mjpeg`, `logs/*.mp4`)
- Test: `tools/test_record_camera.py`

**Interfaces:**
- Consumes: Task 3의 `MjpegParser`, `boundary_from_content_type`, `INDEX_FIELDS`, `format_recv_time`, `summarize`, `StreamFormatError`
- Produces:
  - `class LatestFrame`: `set(jpeg: bytes, recv_monotonic_s: float)`, `get() -> tuple[int, bytes, float] | None`.
    첫 값은 세대 번호이고 set마다 1 오른다. lock으로 보호한다.
  - `@dataclass class RecordState: next_frame: int = 0`: 프레임마다 갱신한다. 예외로 끊겨도 번호가 이어진다.
  - `def record_chunks(chunks: Iterable[bytes], parser: MjpegParser, mjpeg_file: BinaryIO, writer: csv.DictWriter, segment: int, state: RecordState, latest: LatestFrame | None, stop: threading.Event, now_epoch=time.time, now_mono=time.monotonic) -> None`
    - 청크마다 `(now_epoch(), now_mono())`를 tag로 feed한다. 행의 `recv_*`에는 파트 tag(헤더 도착 시각)를 쓴다.
    - `offset`은 쓰기 전의 `mjpeg_file.tell()`이다. JPEG를 쓰고 행을 쓴 뒤 두 파일을 모두 `flush`한다.
    - 청크마다 `stop`을 확인하고, 걸렸으면 돌아간다.
  - `def iter_response(resp: http.client.HTTPResponse) -> Iterator[bytes]`
    - `resp.read1(4096)`로 읽는다. `read(4096)`은 쓰지 않는다. ESP httpd가 chunked 전송이라, 끊길 때
      `read`는 완성된 데이터까지 `IncompleteRead.partial`에 묻는다.
    - `IncompleteRead`가 나면 `e.partial`을 내보내고 끝낸다. 빈 바이트도 끝으로 본다.
  - `@dataclass class OpenedStream: boundary: bytes; chunks: Iterator[bytes]; sock: socket.socket; close: Callable[[], None]`
  - `def open_stream(url: str, timeout_s: float) -> OpenedStream`
    - `http.client.HTTPConnection`으로 연다. 상태가 200이 아니면 `OSError`다.
    - `sock`은 `conn.sock`이다. `close`는 `resp.close()` 다음 `conn.close()`를 부른다.
    - 접속 직후 `conn.sock.getsockname()[0]`이 `192.168.4.`로 시작하지 않으면 `[WARN]`을 한 줄 찍는다.
      노트북이 다른 인터페이스로 나가는 경우를 잡으려는 것이다.
  - `class ActiveConnection`: lock으로 보호하는 소켓 슬롯이다. `set(sock)`, `clear()`, `interrupt()`가 있다.
    - `interrupt()`는 lock 안에서 소켓을 꺼낸다. 그다음 **lock 밖에서** `sock.shutdown(socket.SHUT_RDWR)`만 부르고,
      `OSError`는 무시한다.
    - 다른 스레드에서 `close()`를 부르면 막힌 `read1()`을 깨우지 못하고 버퍼 잠금에서 같이 막힌다(Codex 재현).
      그래서 메인 스레드는 shutdown만 하고, `OpenedStream.close()`는 녹화 스레드의 `finally`에서만 부른다.
  - `def record_loop(url, mjpeg_path: Path, csv_path: Path, latest, stop, active: ActiveConnection, timeout_s, retry_s=1.0, opener=open_stream) -> None`
    - 녹화 스레드의 본체다. **두 파일을 이 스레드가 열고, `finally`에서 이 스레드가 닫는다.** 메인 스레드는 파일을 만지지 않는다.
    - 성공적으로 접속할 때마다 segment를 +1 한다. 0부터 시작한다. 접속하면 `active.set(opened.sock)`을 한다.
      `finally`에서 `active.clear()`를 하고 `opened.close()`를 부른다.
    - `OSError`, `StreamFormatError`, `http.client.HTTPException`이 나면 parser를 `finish()`하고(잘린 파트는 경고), close 함수를 `finally`에서
      부른 뒤 `stop.wait(retry_s)`로 쉬고 재시도한다. `stop`이면 끝낸다.
  - `def run(url, out_dir: Path, preview: bool, timeout_s: float, retry_s: float = 1.0, opener=open_stream, stop: threading.Event | None = None) -> RecordingSummary`
    - `record_loop`를 **백그라운드 스레드**로 띄운다.
    - 미리보기는 **메인 스레드**에서 돈다. Linux OpenCV GUI는 메인 스레드가 안전하다.
      `--no-preview`면 메인 스레드는 `stop`이나 Ctrl+C까지 기다린다.
    - 끝내는 순서는 이렇다.
      1. `stop.set()` → `active.interrupt()`
      2. `join(timeout_s + 2)`
      3. 스레드가 죽었으면 CSV를 다시 읽어 `summarize` 결과를 돌려준다.
      4. 아직 살아 있으면 `[ERROR] recorder thread did not exit`를 찍고 `RuntimeError`를 낸다. 파일은 그 스레드 몫이라 메인은 닫지 않는다.
  - CLI: `--url`(기본 `http://192.168.4.200:81/stream`), `--out-dir`(기본 저장소 `logs/`), `--no-preview`, `--timeout`(기본 5.0)
    - 종료 시 요약을 출력한다: 프레임 수, 구간 수(=재접속+1), seq 틈, 실효 fps, 최장 공백, 두 파일 경로.

**미리보기 규칙**
- 메인 스레드에서 돈다. `cv2`는 미리보기 함수 안에서 import한다.
- 약 30 Hz로 `LatestFrame.get()`을 부른다. **세대 번호가 바뀌었을 때만** `cv2.imdecode` → `imshow`를 한다.
  매 반복 `waitKey(1)`을 부른다.
- 화면에 `fps`와 `age <ms>`를 겹쳐 쓴다. `q`를 누르면 `stop.set()`이다.
- import에 실패하면 경고 한 줄을 찍고 미리보기 없이 계속한다. 녹화 스레드는 미리보기를 기다리지 않는다.

- [ ] **Step 1: 실패하는 시험 작성.** 실제 네트워크와 cv2 없이 돈다.
  - `test_record_chunks_writes_mjpeg_and_index`: 3파트를 넣으면 `.mjpeg` 내용이 세 JPEG를 이은 것과
    같고, CSV 3행의 offset/size가 맞으며, `recv_time` 형식이 맞다(가짜 시계 주입).
  - `test_record_chunks_updates_latest`
  - `test_reconnect_starts_new_segment`: 가짜 `opener`가 첫 호출에는 seq 0,1,2를 주고 `OSError`로
    끊기며, 두 번째에는 seq 0,1(재부팅)을 준 뒤 `stop`을 건다. 결과는 `segments == 2`,
    `seq_gaps == 0`, `frames == 5`이고, CSV의 segment 열은 0,0,0,1,1, **frame 열은 0,1,2,3,4**다. `retry_s = 0`이다.
  - `test_stop_ends_endless_stream`: 끝없이 프레임을 내는 가짜 스트림에 `preview=False`로 `run`을 돌린다.
    다른 스레드에서 0.2 s 뒤 `stop.set()`을 하면 2 s 안에 반환하고, close 함수가 불렸다.
  - `test_stop_while_read_blocked`: **실제 소켓**으로 시험한다(가짜 Event로는 close 잠금 문제를 놓친다).
    - `socket.socketpair()`의 서버 쪽에 HTTP 200 응답 헤더(chunked)와 완결 파트 하나를 쓴 뒤, 닫지 않고 침묵한다.
    - 가짜 opener는 클라이언트 쪽 소켓을 `http.client.HTTPResponse`로 감싸 `OpenedStream`을 만든다.
    - `run(timeout_s=30)`에서 0.2 s 뒤 stop을 걸면 2 s 안에 반환하고, 스레드는 죽었고, CSV에는 1행이 있다.
  - `test_iter_response_keeps_partial_on_incomplete_read`: 가짜 소켓(`makefile`이 `BytesIO`를 돌려준다)에 chunked HTTP
    응답을 담는다. 응답은 완결된 multipart 파트 하나 뒤에서 끊긴다. 이것을 `HTTPResponse`로 감싸
    `iter_response`로 읽어 parser에 넣으면 그 파트가 나온다.
  - `test_latest_frame_generation`: set 두 번이면 세대 번호가 1, 2다.
  - `test_opener_failure_retries_without_crash`: opener가 두 번 `OSError`를 낸 뒤 정상 스트림을 주면 녹화된다.
  - `test_default_url_uses_stream_port`: argparse 기본값이 `http://192.168.4.200:81/stream`이다.
- [ ] **Step 2:** `python -m unittest tools.test_record_camera -v`로 FAIL을 확인한다.
- [ ] **Step 3:** 구현한다. 파일명 시각은 `datetime.now():%Y-%m-%d_%H%M%S`다.
- [ ] **Step 4:** PASS를 확인한다. 이어서 `python -m unittest tools.test_camera_sketch -v`도 다시 돌린다(4210 검사가 새 파일을 본다).
- [ ] **Step 5: 문서.** `scripts/README.md`에 `## 카메라 녹화` 절을 둔다. 내용은 다음과 같다.
  - 사전 점검: 노트북이 `Drone_Tuning`에 붙어 있고, 다른 인터페이스(유선·테더링)가 192.168.4.0/24를
    가로채지 않으며, WiFi 절전이 꺼져 있다([지상국 링크 문서](../docs/ground_station_link.md)).
  - 실행 예
  - 출력 파일
  - `control_dualsense.py`와 동시에 별도 터미널에서 실행한다는 점
  - 후처리 명령(Task 6)
  - 시각 맞춤 한계 한 줄(설계 문서 5.3 링크)
- [ ] **Step 6:** 커밋한다(사용자 확인). 메시지: `feat(camera): record the camera stream on the laptop with a live preview`

## Task 6: `camera_to_mp4.py` — 후처리

**Files:**
- Create: `scripts/camera_to_mp4.py`
- Test: `tools/test_camera_to_mp4.py`

**Interfaces:**
- Consumes: Task 4의 `estimate_offsets`, `drift_report`, `clock_jumps`, `aligned_rows`, `frame_durations`
- Produces:
  - `def load_index(csv_path: Path) -> list[dict]`
  - `def write_aligned_csv(rows: list[dict], path: Path) -> None`: 열은 `INDEX_FIELDS + ("capture_epoch_s", "capture_time")`
  - `def extract_frames(mjpeg_path: Path, rows, out_dir: Path) -> tuple[list[Path], list[dict]]`: offset/size로 `frame_000000.jpg`…를 쓴다.
    - 둘째 값은 실제로 뽑힌 행들이다.
    - **마지막 행**만 잘렸으면(강제 종료) 그 행을 빼고 `[WARN]`을 찍는다.
    - 중간 행이 잘렸으면 `ValueError`다.
  - `def build_video_inputs(mjpeg_path: Path, aligned: list[dict], out_dir: Path) -> tuple[list[Path], list[float]]`
    - `extract_frames`로 남은 행을 먼저 확정한다.
    - **그 행들로** `frame_durations`를 다시 계산한다.
    - `len(frames) == len(durations)`를 assert한다. CLI는 이 함수만 쓴다.
  - `def write_concat_list(frames: list[Path], durations: list[float], list_path: Path) -> None`: ffmpeg concat demuxer 형식이다.
    - 파일마다 `file '<abs>'`, `option framerate 1000`, `duration <s>`를 쓴다. 마지막 파일은 `file`과
      `option framerate 1000`으로 한 번 더 반복한다.
    - `option framerate 1000`이 없으면 기본 25 Hz 시간축으로 PTS가 양자화된다. 실측에서
      [0.1, 0.001, 0.2, 0.1]이 0, 0.12, 0.16, 0.32, 0.40이 됐다(ffmpeg n9.0.2).
  - `def build_ffmpeg_command(list_path: Path, out_path: Path) -> list[str]`:
    `["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", str(list_path), "-fps_mode", "vfr", "-c:v", "libx264", "-enc_time_base", "1:1000", "-pix_fmt", "yuv420p", str(out_path)]`
  - CLI: `camera_to_mp4.py logs/camera_X.csv [--no-mp4]`
    - `.mjpeg`는 같은 이름에서 찾는다.
    - 출력은 `_aligned.csv`이고, `--no-mp4`가 아니면 `.mp4`도 만든다.
    - 임시 프레임은 `tempfile.TemporaryDirectory(dir=<출력 폴더>)`에 둔다. tmpfs인 `/tmp`를 피하려는 것이다.
    - 출력: 구간별 오프셋, 드리프트(None이면 "구간 짧음"), 시계 점프 행 목록(있으면 `[WARN]`), 결과 경로

- [ ] **Step 1: 실패하는 시험 작성.**
  - `test_extract_frames_roundtrip`: Task 5의 `record_chunks`로 만든 `.mjpeg`·`.csv`에서 뽑은 JPEG가 원본과 같다.
  - `test_extract_frames_truncated_middle_raises` / `test_extract_frames_truncated_last_skipped`
  - `test_concat_list_format`: 2프레임, durations [0.1, 0.1]이면 정확한 텍스트가 나온다(`option framerate 1000`과 마지막 파일 반복 포함).
  - `test_build_ffmpeg_command`: 위 리스트와 같다.
  - `test_build_video_inputs_recomputes_after_truncation`: capture 시각이 0, 0.1, 10이고 마지막 프레임이 잘린
    녹화본을 넣는다. 결과는 프레임 2개, durations `[0.1, 0.1]`(마지막은 `last_s`)이다. 9.9가 나오면 안 된다.
  - `test_aligned_csv_columns`

  ffmpeg는 실행하지 않는다(CI skip 금지).
- [ ] **Step 2:** FAIL을 확인한다. **Step 3:** 구현한다. **Step 4:** PASS를 확인한다.
- [ ] **Step 5: 로컬 실측(1회).** 합성 JPEG 몇 장으로 만든 녹화본에 CLI를 돌려 `.mp4`가 생기는지 본다.
  `ffprobe -v error -select_streams v -show_entries frame=pts_time -of csv=p=0 <mp4>`로 본 프레임별
  PTS가 durations 누적합과 ±1 ms 안으로 맞아야 한다. 결과는 커밋 메시지가 아니라 작업 보고에 적는다.
- [ ] **Step 6:** 커밋한다(사용자 확인). 메시지: `feat(camera): align a camera recording to laptop time and export mp4`

## Task 7: 전체 확인과 벤치 인계

- [ ] **Step 1: 전체 시험.**
  - 명령: `python -m unittest discover tools -p "test_*.py" -v`, `python tools/check_repo_layout.py`, `python -m py_compile scripts/*.py`
  - 기대 결과: 실패 0, skip 0
  - CLAUDE.md 규칙에 따라 pytest 묶음은 ws1 대상이지만 이 저장소는 unittest 묶음이다. 로컬 실행
    시간이 1분을 넘으면 중단하고 ws1에서 돌린다.
- [ ] **Step 2:** 로컬에서 카메라 스케치와 비행 스케치 하나(`dual_imu_cascade_pwm`, `esp32s3`)를
  `arduino-cli compile`한다. 비행 스케치는 회귀가 없음을 보는 용도다.
- [ ] **Step 3: 커밋 전 Codex 리뷰.** `adversarial-review`로 브랜치 diff를 리뷰받는다. 수렴할 때까지 반복한다.
- [ ] **Step 4: 실기 벤치 인계.** 사용자에게 설계 문서 7절의 벤치 1–3 체크리스트를 전달한다.
  - 실기 결과가 나오기 전에는 "검증 완료"라고 쓰지 않는다.
  - 결과가 나오면 노션 `💻 CODE > 펌웨어` 토글 아래에 기록한다(`docs/notion_doc_convention.md`).

## Self-review

- **Spec 대응**
  - 5.1 → Task 1, 2
  - 5.2 → Task 3, 5
  - 5.3 → Task 4, 6
  - 6절(문서·CI) → Task 2, 5
  - 7절 호스트 시험 → Task 1, 3–6
  - 7절 벤치 → Task 7 인계
  - 반목표 4210 → Task 2의 정적 시험
  - 비행 펌웨어 미수정 → Global Constraints
- **수치 재계산(원자료 기준)**
  - 187,500 B/s = 1.5×10⁶/8
  - 4096 B 대기 = ⌈4096/187500×10⁶⌉ = 21,846 µs
  - 8192 B = 43,690.7 → 43,691 µs
  - 10 fps = 100,000 µs
  - HVGA = 480×320(`sensor.h`)
  - DHCP `DHCPS_MAX_LEASE 0x64` → .2–.101
  - AI-Thinker 핀은 3.3.7 `camera_pins.h`에서 옮겼다.
- **독립 검토 기록**은 아래 절에 남긴다.

## 독립 검토 기록

### ① Claude 재계산(원자료)

원자료: `sensor.h`, `camera_pins.h`, `dhcpserver.h`, `sdkconfig.h`(3.3.7), ffmpeg 실측

| 지적 | 처리 |
|---|---|
| 드리프트 시험 기댓값 0.03 | 0.024로 정정(마지막 창 최소가 t≈240) |
| duration 허용오차 1e-9 | 0.1999999를 수용하도록 1e-6 |

### ② Codex 적대적 검토

Codex는 수치를 모두 재계산해 일치를 확인했다. P1은 없었다.

| 지적 | 처리 |
|---|---|
| 예외 시 frame 번호 중복 | `RecordState`로 바꾸고 frame 연속 검사 추가 |
| `read(4096)`이 끊길 때 완성 파트를 `IncompleteRead.partial`에 묻음 | `read1`과 partial 처리, `BytesIO` 시험 추가 |
| ffmpeg 25 Hz 시간축 양자화 | Claude가 실측해 재현했다(0.12/0.16). `option framerate 1000`과 `-enc_time_base 1:1000`으로 해결 확인(0.100/0.101/0.301). 프레임별 PTS 검증으로 변경 |
| q·stop이 장수명 스트림에 전달 안 됨 | `record_chunks(stop=…)`, `finally` close, 무한 스트림 종료 시험 추가 |
| 재접속 공백 보고·길이 불일치·EOF 시험 누락 | `reconnect_gaps_s`, `test_length_mismatch_raises`, `finish()` 추가 |
| (재검토 1) `StreamPart` tag 필수화로 시험 생성자 불일치 | `tag: object = None` 기본값 |
| (재검토 1) 읽기가 막히면 종료 시 스레드가 남은 채 파일을 닫음 | 파일은 녹화 스레드가 소유. 막힌 읽기 종료 시험 추가 |
| (재검토 2) 다른 스레드의 `close()`가 막힌 `read1`을 못 깨우고 같이 막힘 | `ActiveConnection.interrupt()`는 lock 밖 `shutdown(SHUT_RDWR)`만. close는 녹화 스레드 `finally`. socketpair 실소켓 시험 |
| (재검토 1) 잘린 마지막 프레임을 빼고도 기존 duration을 씀(9.9 s) | `build_video_inputs`가 남은 행으로 다시 계산, 길이 assert, 통합 시험 |

### ③ agy(Gemini 4 Argon) 독립 검토

①·② 결론을 보여주지 않고 "적히지 않은 전제"를 물었다.

**채택**
- 수신 시각에 송신 속도 제한 지연(프레임 크기/187.5 kB/s)이 섞인다 → 헤더 도착 시각(`tag`)으로 기록
- OpenCV GUI를 워커 스레드에서 띄움 → 미리보기는 메인 스레드, 녹화는 백그라운드. 세대 번호로 새 프레임만 디코드
- `vTaskDelay` / busy-wait → 1 ms 이상은 `vTaskDelay`. 고정 주기 스케줄
- `reconnect()` 반복·부팅 순서 → `setAutoReconnect` 더하기 10 s 단위 전체 재시작. 벤치 1에 "카메라 먼저 전원" 시험
- 브라운아웃 구분 → `reset_reason`
- 카메라 TX 출력 → `WIFI_POWER_11dBm` 기본
- FPC 접촉 불량 → 연속 10회 실패 시 카메라 재초기화
- 노트북 라우팅·절전 → 로컬 주소 경고, README 사전 점검
- tmpfs·강제 종료 → 출력 폴더에 임시 디렉터리. 마지막 잘린 프레임은 건너뜀
- 벤치 2에 스로틀 변화, BMM350 heading, JPEG 크기 변화 확인 추가(설계 7절)
- 실효 fps 기준을 바이트율 상한에 맞춤(설계 1절)

**기각**
- TCP keepalive: 근거는 `CONFIG_LWIP_TCP_SND_BUF_DEFAULT 5744`다. 페이싱(187.5 kB/s)에서 버퍼가 약 31 ms에 차고
  `send_wait_timeout = 2` s가 걸리므로, '10–30 s 정체' 전제가 성립하지 않는다. 대신 벤치 1의 `kill -9` 재접속 실측으로 확인한다.
- `IP_TOS` 우선순위와 동적 backpressure: SoftAP 중계 경로에서 효과가 확인되지 않았다. 벤치 2 실측이 먼저다.
- 470 µF 커패시터: 하드웨어 요건이라 계획 밖이다. 사용자 인계에 권고로만 적는다.
