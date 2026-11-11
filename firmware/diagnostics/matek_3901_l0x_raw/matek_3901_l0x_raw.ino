#include <Arduino.h>

// Matek 3901-L0X 수신 확인 진단. 모듈이 UART 로 밀어넣는 바이트를 그대로
// 16진수로 찍고, MSPv2 프레임이 완성되면 거리·광류 값을 같은 줄에 푼다.
// 모터·IMU·WiFi 는 건드리지 않는다.
//
// 배선 (PCB v1.5.2 소나 커넥터): 모듈 TX -> GPIO17(ESP RX). 모듈 RX 는
// 연결하지 않는다(커넥터의 ESP TX=GPIO18 은 비워 둔다).
// 모듈 전원은 5V(4.5~5.5V). 3.3V 로 주면 측정거리가 줄어든다.
//
// 로그는 UART0(GPIO43/44)로 낸다. PCB v1.5.2 의 USB 포트는 이 핀에 붙은
// CH343 브리지(/dev/ttyACM0)라서, "USB CDC On Boot" 설정과 상관없이 거기로
// 나가도록 Serial 대신 Serial0 을 쓴다.
#define LOG Serial0

const int SENSOR_RX_PIN = 17;
const uint32_t SENSOR_BAUD = 115200;
const uint32_t SUMMARY_MS = 1000;

// 펌웨어 msp_sensor.h 와 일부러 따로 푼다 — 같은 실수를 공유하면 서로 맞는
// 것처럼 보인다. 프레임: '$' 'X' dir flags cmd(u16 LE) size(u16 LE) payload crc
// CRC(crc8_dvb_s2, 0xD5) 는 방향 바이트 다음(flags)부터 payload 끝까지다.
const size_t HEADER_LEN = 8;
const size_t MAX_PAYLOAD = 32;
uint8_t frame[HEADER_LEN + MAX_PAYLOAD + 1];
size_t frameLen = 0;

uint32_t bytesThisSecond = 0;
uint32_t bytesTotal = 0;
uint32_t strayBytes = 0;   // 프레임 밖 바이트. 늘면 보레이트·배선을 의심한다
uint32_t framesOk = 0;
uint32_t crcErrors = 0;
uint32_t lastSummary = 0;

static uint8_t crc8DvbS2(const uint8_t *p, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static int32_t readI32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static void printHex(const uint8_t *p, size_t n) {
  for (size_t i = 0; i < n; i++) LOG.printf("%02X ", p[i]);
}

static void printStray(uint8_t c) {
  strayBytes++;
  LOG.printf("[stray] %02X\n", c);
}

static void handleFrame(size_t payloadLen) {
  const uint16_t cmd = frame[4] | (frame[5] << 8);
  const uint8_t *payload = &frame[HEADER_LEN];
  const uint8_t got = frame[HEADER_LEN + payloadLen];
  const uint8_t want = crc8DvbS2(&frame[3], 5 + payloadLen);

  LOG.printf("%6lu ms  ", (unsigned long)millis());
  printHex(frame, HEADER_LEN + payloadLen + 1);
  if (got != want) {
    crcErrors++;
    LOG.printf(" CRC BAD (want %02X)\n", want);
    return;
  }
  framesOk++;
  if (cmd == 0x1F01 && payloadLen == 5) {
    // 80~2000mm 밖이면 음수다. 너무 가까워도 멀어도 같은 음수다.
    LOG.printf(" RANGE q=%u dist=%ld mm\n", payload[0],
               (long)readI32(&payload[1]));
  } else if (cmd == 0x1F02 && payloadLen == 9) {
    LOG.printf(" FLOW q=%u x=%ld y=%ld\n", payload[0],
               (long)readI32(&payload[1]), (long)readI32(&payload[5]));
  } else {
    LOG.printf(" cmd=0x%04X size=%u (모르는 프레임)\n", cmd,
               (unsigned)payloadLen);
  }
}

// 바이트 하나를 프레임 버퍼에 쌓는다. 헤더가 어긋나면 그 바이트까지 stray 로
// 찍고 버린다. 단, 그 바이트가 '$' 면 새 프레임의 시작으로 받는다.
static void feed(uint8_t c) {
  if (frameLen == 0) {
    if (c == '$') frame[frameLen++] = c;
    else printStray(c);
    return;
  }

  frame[frameLen++] = c;
  const bool badX = (frameLen == 2 && c != 'X');
  const bool badDir = (frameLen == 3 && c != '<' && c != '>' && c != '!');
  if (badX || badDir) {
    for (size_t i = 0; i + 1 < frameLen; i++) printStray(frame[i]);
    frameLen = 0;
    if (c == '$') frame[frameLen++] = c;
    else printStray(c);
    return;
  }
  if (frameLen < HEADER_LEN) return;

  const size_t payloadLen = frame[6] | (frame[7] << 8);
  if (payloadLen > MAX_PAYLOAD) {
    LOG.printf("[oversize] size=%u, header ", (unsigned)payloadLen);
    printHex(frame, HEADER_LEN);
    LOG.println();
    frameLen = 0;
    return;
  }
  if (frameLen == HEADER_LEN + payloadLen + 1) {
    handleFrame(payloadLen);
    frameLen = 0;
  }
}

void setup() {
  LOG.begin(115200);
  delay(2000);
  Serial1.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, -1);

  LOG.println();
  LOG.println("=== Matek 3901-L0X raw 수신 진단 ===");
  LOG.printf("RX=GPIO%d, %lu 8N1. 모듈 TX -> GPIO%d, 전원 5V\n", SENSOR_RX_PIN,
             (unsigned long)SENSOR_BAUD, SENSOR_RX_PIN);
}

void loop() {
  while (Serial1.available() > 0) {
    bytesThisSecond++;
    bytesTotal++;
    feed((uint8_t)Serial1.read());
  }

  const uint32_t now = millis();
  if (now - lastSummary >= SUMMARY_MS) {
    lastSummary = now;
    LOG.printf("[summary] %lu B/s, total %lu B, ok %lu, crc_bad %lu, stray %lu\n",
               (unsigned long)bytesThisSecond, (unsigned long)bytesTotal,
               (unsigned long)framesOk, (unsigned long)crcErrors,
               (unsigned long)strayBytes);
    if (bytesThisSecond == 0) {
      LOG.println("[summary] 1초간 0바이트 — 5V 전원·모듈 TX->GPIO17 배선 확인");
    }
    bytesThisSecond = 0;
  }
}
