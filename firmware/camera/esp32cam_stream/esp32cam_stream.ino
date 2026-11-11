// ESP32-CAM(AI-Thinker 핀맵) 영상 스트림 — 비행 ESP32 의 SoftAP 에 붙어 노트북으로
// MJPEG 를 내보낸다. 노트북의 scripts/drone_video.py 가 받아 녹화한다.
//
//   :81 /stream  multipart/x-mixed-replace;boundary=zetincamframe
//                파트마다 X-Timestamp(캡처 시각, 부팅 후 s.µs) · X-Frame-Seq(송신 일련번호)
//   :80 /status  설정·카운터 JSON
//
// 비행 링크의 UDP 포트는 쓰지 않는다. 비행 펌웨어는 마지막 UDP 발신자에게 회신하므로
// 여기서 그 포트로 무엇이든 보내면 텔레메트리 목적지가 바뀐다.
// 설계: docs/superpowers/specs/2026-11-02-camera-streaming-design.md

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "stream_format.h"
#include "stream_pacing.h"

// ===== 링크 =====
const char* WIFI_SSID = "Drone_Tuning";
const char* WIFI_PASS = "12345678";
// SoftAP 기본 DHCP 풀은 192.168.4.2–.101 (DHCPS_MAX_LEASE 0x64) — 그 밖의 고정 주소.
const IPAddress CAM_IP = IPAddress(192, 168, 4, 200);
const IPAddress CAM_GATEWAY = IPAddress(192, 168, 4, 1);
const IPAddress CAM_SUBNET = IPAddress(255, 255, 255, 0);
// 최대 출력. 11 dBm 으로 낮췄더니 벤치(2026-11-02, OV3660 보드)에서 카메라 구간 ping 이
// 17–640 ms 로 튀고 녹화가 0.7–1.2 fps 에 그쳤다 — 미확인 ACK 가 TCP 송신 버퍼(5744 B)를
// 넘지 못해 왕복 지연이 곧 fps 를 누른다. 19.5 dBm 에서는 18–91 ms, 7.9 fps.
// 송신 피크 전류가 커지므로 BEC 로 옮기면 reset_reason 이 brownout 인지 확인한다.
const wifi_power_t CAM_WIFI_TX_POWER = WIFI_POWER_19_5dBm;
const uint32_t WIFI_CHECK_MS = 2000;
const uint32_t WIFI_RESTART_AFTER_MS = 10000;  // 이만큼 끊겨 있으면 begin 부터 다시

// ===== 영상 =====
// 센서 방향. OV3660 보드에서 좌우가 뒤집혀 나와 기본으로 미러링한다. 기체에 단 방향에
// 맞춰 바꾼다.
const bool CAM_HMIRROR = true;
const bool CAM_VFLIP = false;
// 센서 입력 클럭. 20 MHz 에서는 센서를 끄고 클럭만 돌려도 카메라 링크의 TCP 처리량이 69/81 kB/s
// 로 떨어졌고(클럭까지 끄면 219/254 kB/s) 스트림이 5.0·0.65 fps 였다. 12 MHz 에서는 9.5–9.7 fps
// (벤치 2026-11-02, 11번 채널). CPU 는 남아 있었고 우선순위로는 달라지지 않았다 — 클럭에 따른
// 전기·RF 결합이 유력하지만 경로는 미확인이다. 10 MHz 도 나빴고 24 MHz 는 프레임이 깨진다.
const int CAM_XCLK_HZ = 12000000;
// VGA·품질 10: 20 MHz 클럭 시절 벤치에서 15–22 kB/프레임. 12 MHz 에서는 10 fps 상한 근처까지
// 나온다(약 150 kB/s). 처리량은 TCP 송신 버퍼(5744 B) ÷ 왕복 지연 근사를 따른다.
const framesize_t FRAME_SIZE = FRAMESIZE_VGA;           // 640x480
const framesize_t FRAME_SIZE_NO_PSRAM = FRAMESIZE_QVGA;  // 320x240
const int JPEG_QUALITY = 10;
const uint32_t FPS_CAP = 10;
const int64_t FRAME_INTERVAL_US = 1000000 / FPS_CAP;
// 1.5 Mbps. fps 상한만으로는 JPEG 크기 변동을 못 막아서 바이트율도 묶는다.
const uint32_t BYTE_RATE_CAP = 187500;
const uint32_t SEND_CHUNK_BYTES = 4096;
const uint32_t BUCKET_CAPACITY_BYTES = 8192;  // 조각 두 개 — 프레임 하나가 한꺼번에 몰리지 않게
const uint32_t CAPTURE_FAILS_BEFORE_REINIT = 10;
const int SEND_WAIT_TIMEOUT_S = 2;  // 소켓 송신 한 번의 대기 제한 (기본 5 s)
const uint32_t SUMMARY_MS = 5000;

// ===== AI-Thinker ESP32-CAM 핀 =====
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ===== 상태 (스트림 핸들러가 쓰고 loop()·/status 가 읽는다) =====
static bool g_psram = false;
static framesize_t g_frame_size = FRAME_SIZE;
static uint16_t g_sensor_pid = 0;
static esp_reset_reason_t g_reset_reason = ESP_RST_UNKNOWN;
static volatile uint32_t g_frames_sent = 0;
static volatile uint32_t g_frames_capture_failed = 0;
static volatile uint32_t g_camera_reinits = 0;
static volatile uint32_t g_sends_failed = 0;
static volatile uint32_t g_jpeg_bytes_sent = 0;  // 래핑 허용 — 구간 차분으로만 쓴다
static volatile uint32_t g_wifi_reconnects = 0;
static volatile bool g_stream_active = false;
static float g_effective_fps = 0.0f;
static uint32_t g_avg_frame_bytes = 0;

static httpd_handle_t g_status_httpd = nullptr;
static httpd_handle_t g_stream_httpd = nullptr;

static const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
  }
}

static const char* frameSizeName(framesize_t f) {
  switch (f) {
    case FRAMESIZE_QVGA: return "QVGA";
    case FRAMESIZE_HVGA: return "HVGA";
    case FRAMESIZE_VGA: return "VGA";
    default: return "other";
  }
}

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = CAM_XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = JPEG_QUALITY;
  if (g_psram) {
    config.frame_size = FRAME_SIZE;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAME_SIZE_NO_PSRAM;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  g_frame_size = config.frame_size;
  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init failed 0x%x\n", (unsigned)err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s != nullptr) {
    g_sensor_pid = s->id.PID;
    s->set_hmirror(s, CAM_HMIRROR ? 1 : 0);
    s->set_vflip(s, CAM_VFLIP ? 1 : 0);
  }
  return true;
}

// 1 ms 이상은 vTaskDelay 로 CPU 를 양보하고 깨어난 뒤 다시 잰다. 그 아래만 busy-wait.
static void waitUntilUs(int64_t targetUs) {
  for (;;) {
    const int64_t remaining = targetUs - esp_timer_get_time();
    if (remaining <= 0) return;
    if (remaining >= 1000) {
      const TickType_t ticks = pdMS_TO_TICKS((uint32_t)(remaining / 1000));
      vTaskDelay(ticks > 0 ? ticks : 1);
    } else {
      delayMicroseconds((uint32_t)remaining);
    }
  }
}

// 조각마다 토큰을 기다렸다가 보낸다. 실패하면 그 결과를 돌려준다.
static esp_err_t sendPaced(httpd_req_t* req, TokenBucket& bucket, const char* data, size_t len) {
  size_t off = 0;
  while (off < len) {
    const uint32_t n = (uint32_t)((len - off) < SEND_CHUNK_BYTES ? (len - off) : SEND_CHUNK_BYTES);
    const uint32_t wait = bucket.waitUs(esp_timer_get_time(), n);
    if (wait > 0) waitUntilUs(esp_timer_get_time() + wait);
    bucket.consume(esp_timer_get_time(), n);
    const esp_err_t res = httpd_resp_send_chunk(req, data + off, n);
    if (res != ESP_OK) return res;
    off += n;
  }
  return ESP_OK;
}

// 동기 루프라 :81 서버의 작업 태스크를 점유한다. 두 번째 클라이언트는 첫 스트림이
// 끝날 때까지 기다린다 — 녹화기 하나만 클라이언트로 두는 설계라 받아들인다.
static esp_err_t streamHandler(httpd_req_t* req) {
  char content_type[64];
  snprintf(content_type, sizeof(content_type), "multipart/x-mixed-replace;boundary=%s", kStreamBoundary);
  esp_err_t res = httpd_resp_set_type(req, content_type);
  if (res != ESP_OK) return res;

  TokenBucket bucket(BYTE_RATE_CAP, BUCKET_CAPACITY_BYTES);
  bucket.reset(esp_timer_get_time());
  g_stream_active = true;
  uint32_t consecutive_fails = 0;
  const int64_t stream_start_us = esp_timer_get_time();
  int64_t next_frame_us = stream_start_us;
  char boundary[32];
  const size_t boundary_len = formatBoundary(boundary, sizeof(boundary));
  char header[160];

  for (;;) {
    waitUntilUs(next_frame_us);
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb == nullptr) {
      g_frames_capture_failed = g_frames_capture_failed + 1;
      if (++consecutive_fails >= CAPTURE_FAILS_BEFORE_REINIT) {
        // FPC 접촉 불량 등으로 드라이버가 멈춘 경우의 복구 경로
        Serial.println("[CAM] capture stuck -> reinit");
        esp_camera_deinit();
        initCamera();
        g_camera_reinits = g_camera_reinits + 1;
        consecutive_fails = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    consecutive_fails = 0;

    const int64_t capture_us = (int64_t)fb->timestamp.tv_sec * 1000000 + fb->timestamp.tv_usec;
    if (frameIsStale(capture_us, stream_start_us)) {
      esp_camera_fb_return(fb);  // 버퍼를 돌려줘야 드라이버가 새 프레임을 채운다
      continue;
    }
    const size_t header_len = formatPartHeader(header, sizeof(header), (uint32_t)fb->len, capture_us, g_frames_sent);
    res = sendPaced(req, bucket, boundary, boundary_len);
    if (res == ESP_OK) res = sendPaced(req, bucket, header, header_len);
    if (res == ESP_OK) res = sendPaced(req, bucket, (const char*)fb->buf, fb->len);
    const size_t sent_len = fb->len;
    esp_camera_fb_return(fb);
    if (res != ESP_OK) {
      g_sends_failed = g_sends_failed + 1;
      break;
    }
    g_frames_sent = g_frames_sent + 1;
    g_jpeg_bytes_sent = g_jpeg_bytes_sent + (uint32_t)sent_len;

    next_frame_us += FRAME_INTERVAL_US;
    const int64_t now = esp_timer_get_time();
    if (next_frame_us < now) next_frame_us = now;  // 뒤처졌으면 따라잡으려 몰아 보내지 않는다
  }
  g_stream_active = false;
  return res;
}

static esp_err_t statusHandler(httpd_req_t* req) {
  char json[640];
  const int n = snprintf(
      json, sizeof(json),
      "{\"frame_size\":\"%s\",\"jpeg_quality\":%d,\"fps_cap\":%lu,\"byte_rate_cap\":%lu,"
      "\"psram\":%s,\"sensor_pid\":\"0x%04x\",\"reset_reason\":\"%s\","
      "\"frames_sent\":%lu,\"frames_capture_failed\":%lu,\"camera_reinits\":%lu,"
      "\"sends_failed\":%lu,\"avg_frame_bytes\":%lu,\"effective_fps\":%.2f,"
      "\"rssi\":%d,\"wifi_reconnects\":%lu,\"stream_active\":%s,\"uptime_ms\":%lu}",
      frameSizeName(g_frame_size), JPEG_QUALITY, (unsigned long)FPS_CAP, (unsigned long)BYTE_RATE_CAP,
      g_psram ? "true" : "false", (unsigned)g_sensor_pid, resetReasonName(g_reset_reason),
      (unsigned long)g_frames_sent, (unsigned long)g_frames_capture_failed,
      (unsigned long)g_camera_reinits, (unsigned long)g_sends_failed,
      (unsigned long)g_avg_frame_bytes, (double)g_effective_fps, (int)WiFi.RSSI(),
      (unsigned long)g_wifi_reconnects, g_stream_active ? "true" : "false",
      (unsigned long)millis());
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, n > 0 ? n : 0);
}

static void startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.send_wait_timeout = SEND_WAIT_TIMEOUT_S;

  httpd_uri_t status_uri = {};
  status_uri.uri = "/status";
  status_uri.method = HTTP_GET;
  status_uri.handler = statusHandler;
  if (httpd_start(&g_status_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(g_status_httpd, &status_uri);
  } else {
    Serial.println("[HTTP] status server failed");
  }

  // 스트림 핸들러가 무한 루프로 서버를 점유하므로 서버를 나눈다 (예제와 같은 방식).
  config.server_port += 1;
  config.ctrl_port += 1;
  httpd_uri_t stream_uri = {};
  stream_uri.uri = "/stream";
  stream_uri.method = HTTP_GET;
  stream_uri.handler = streamHandler;
  if (httpd_start(&g_stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(g_stream_httpd, &stream_uri);
  } else {
    Serial.println("[HTTP] stream server failed");
  }
}

static void beginWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.config(CAM_IP, CAM_GATEWAY, CAM_SUBNET);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setTxPower(CAM_WIFI_TX_POWER);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  g_reset_reason = esp_reset_reason();
  g_psram = psramFound();
  Serial.printf("\n[BOOT] reset reason %s\n", resetReasonName(g_reset_reason));
  Serial.printf("[BOOT] psram %s\n", g_psram ? "yes" : "no");
  if (!initCamera()) {
    Serial.println("[BOOT] camera init failed - restarting in 5 s");
    delay(5000);
    ESP.restart();
  }
  Serial.printf("[BOOT] sensor PID 0x%04x, frame %s, quality %d, fps cap %lu, byte cap %lu B/s\n",
                (unsigned)g_sensor_pid, frameSizeName(g_frame_size), JPEG_QUALITY,
                (unsigned long)FPS_CAP, (unsigned long)BYTE_RATE_CAP);
  beginWifi();
  startServers();
  Serial.println("[BOOT] stream http://192.168.4.200:81/stream  status http://192.168.4.200/status");
}

void loop() {
  static uint32_t last_check_ms = 0;
  static uint32_t disconnected_since_ms = 0;
  static uint32_t last_restart_ms = 0;
  static bool was_connected = false;
  static bool ever_connected = false;
  static uint32_t last_summary_ms = 0;
  static uint32_t last_frames = 0;
  static uint32_t last_bytes = 0;

  const uint32_t now = millis();
  if (now - last_check_ms >= WIFI_CHECK_MS) {
    last_check_ms = now;
    const bool connected = WiFi.status() == WL_CONNECTED;
    if (connected) {
      if (!was_connected) {
        if (ever_connected) g_wifi_reconnects = g_wifi_reconnects + 1;
        Serial.printf("[WIFI] connected %s rssi %d\n", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
      }
      ever_connected = true;
      disconnected_since_ms = 0;
    } else {
      if (was_connected) Serial.println("[WIFI] disconnected");
      if (disconnected_since_ms == 0) disconnected_since_ms = now;
      // 자동 재접속에 맡기되 오래 끊겨 있으면 begin 부터 다시 — 카메라가 비행 보드보다
      // 먼저 켜져 첫 begin 이 AP 를 못 찾은 경우도 이 경로로 붙는다.
      if (now - disconnected_since_ms >= WIFI_RESTART_AFTER_MS &&
          now - last_restart_ms >= WIFI_RESTART_AFTER_MS) {
        last_restart_ms = now;
        Serial.println("[WIFI] restart");
        WiFi.disconnect();
        beginWifi();
      }
    }
    was_connected = connected;
  }

  if (now - last_summary_ms >= SUMMARY_MS) {
    const uint32_t frames = g_frames_sent;
    const uint32_t bytes = g_jpeg_bytes_sent;
    const uint32_t df = frames - last_frames;
    const float dt_s = (now - last_summary_ms) / 1000.0f;
    g_effective_fps = last_summary_ms == 0 ? 0.0f : df / dt_s;
    if (df > 0) g_avg_frame_bytes = (bytes - last_bytes) / df;
    last_frames = frames;
    last_bytes = bytes;
    last_summary_ms = now;
    Serial.printf("[SUM] fps %.1f avg %lu B rssi %d reconnects %lu capture_fail %lu send_fail %lu reinit %lu %s\n",
                  (double)g_effective_fps, (unsigned long)g_avg_frame_bytes, (int)WiFi.RSSI(),
                  (unsigned long)g_wifi_reconnects, (unsigned long)g_frames_capture_failed,
                  (unsigned long)g_sends_failed, (unsigned long)g_camera_reinits,
                  g_stream_active ? "streaming" : "idle");
  }
  delay(50);
}
