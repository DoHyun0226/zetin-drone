#pragma once

#include <Arduino.h>

// Matek 3901-L0X (PMW3901 광류 + VL53L0X 라이다) 가 UART 로 밀어넣는 INAV
// MSPv2 센서 프레임 파서. 수신 전용이며 어떤 것도 제어하지 않는다.
// (광류: 바닥 무늬가 움직이는 양으로 수평 이동 감지 / 라이다: 바닥까지 거리)
//
// 프레임 (iNav src/main/msp/msp_serial.c 상태기계 확인):
//   '$' 'X' <dir>  flags(u8) cmd(u16 LE) size(u16 LE) payload  crc8
//                  └──────────── CRC 적용 구간 ────────────┘
// `$X` 와 방향 바이트('<' '>' '!')는 CRC에 들어가지 않는다.
// (LE = little endian: 여러 바이트 숫자를 낮은 바이트부터 보낸다)
//
// 페이로드 (iNav src/main/msp/msp_protocol_v2_sensor_msg.h 원문):
//   MSP2_SENSOR_RANGEFINDER 0x1F01 : u8 quality; i32 distanceMm  (5B)
//       distanceMm 은 **범위 밖이면 음수**다. 이 모듈의 작동 범위가
//       8cm~200cm 이므로 **너무 가까울 때도 음수**이고, 부호만으로는
//       위/아래를 구분할 수 없다. 접지 판정은 직전 유효값의 추세를 봐야 한다.
//   MSP2_SENSOR_OPTIC_FLOW  0x1F02 : u8 quality; i32 motionX; i32 motionY (9B)
//
// [파서 동작 정리] 바이트가 하나 올 때마다 mspSensorParseByte()를 호출한다(상태 기계)
//  상태가 IDLE → '$' → 'X' → 방향 → flags → cmd → size → payload → crc 순으로 넘어가고,
//  crc까지 받으면 검사 후 결과를 돌려준다. 중간에 엉뚱한 바이트가 오면 처음으로 돌아가 다시 '$'를 찾는다

static const uint16_t MSP2_SENSOR_RANGEFINDER = 0x1F01;   // 라이다 거리 메시지 번호
static const uint16_t MSP2_SENSOR_OPTIC_FLOW  = 0x1F02;   // 광류 메시지 번호

// 두 메시지는 5B/9B다. 여유를 두되, 이보다 큰 페이로드는 저장하지 않고
// **소비만 해서 동기를 유지**한다(다른 MSP 트래픽이 섞여도 재동기화된다).
static const uint16_t MSP_SENSOR_MAX_PAYLOAD = 16;

// mspSensorParseByte()의 반환값: 이번 바이트로 무엇이 완성됐는가
enum MspParseResult : uint8_t {
  MSP_PARSE_NONE = 0,       // 프레임 미완성
  MSP_PARSE_RANGEFINDER,    // 라이다 프레임 완성 → p.range 갱신됨
  MSP_PARSE_OPTIC_FLOW,     // 광류 프레임 완성 → p.flow 갱신됨
  MSP_PARSE_UNKNOWN,        // CRC 정상인데 모르는 function
  MSP_PARSE_CRC_ERROR,      // CRC 불일치 → 프레임 버림 (배선 노이즈/보레이트 불일치 의심)
};

// 상태 기계의 현재 위치 = "다음에 올 바이트가 무엇인가"
enum MspParseState : uint8_t {
  MSP_ST_IDLE = 0,     // '$' 기다리는 중
  MSP_ST_HEADER_X,     // 'X' 기다리는 중 (MSPv2 표시)
  MSP_ST_HEADER_DIR,   // 방향 바이트 기다리는 중
  MSP_ST_FLAGS,
  MSP_ST_CMD_LO,       // 메시지 번호 하위 바이트
  MSP_ST_CMD_HI,       // 메시지 번호 상위 바이트
  MSP_ST_SIZE_LO,      // 페이로드 길이 하위 바이트
  MSP_ST_SIZE_HI,      // 페이로드 길이 상위 바이트
  MSP_ST_PAYLOAD,      // 페이로드 수신 중
  MSP_ST_CRC,          // 마지막 CRC 바이트 기다리는 중
};

struct MspRangefinderSample {
  uint8_t quality = 0;       // 측정 품질 (0~255, 클수록 좋음)
  int32_t distance_mm = 0;   // 바닥까지 거리 (mm). 범위 밖이면 음수
};

struct MspOpticFlowSample {
  uint8_t quality = 0;    // 바닥 무늬 품질 (어두움/무늬 없음이면 낮다)
  int32_t motion_x = 0;   // x 방향 이동량 (센서 원값)
  int32_t motion_y = 0;   // y 방향 이동량 (센서 원값)
};

// 파서 상태 + 마지막으로 해석한 값 + 진단 카운터
struct MspSensorParser {
  uint8_t  state = MSP_ST_IDLE;   // MspParseState 값
  uint8_t  flags = 0;
  uint16_t cmd = 0;               // 받고 있는 프레임의 메시지 번호
  uint16_t size = 0;              // 받고 있는 프레임의 페이로드 길이
  uint16_t offset = 0;            // 페이로드 중 몇 바이트를 받았는지
  uint8_t  crc = 0;               // 지금까지 계산한 CRC
  bool     oversize = false;   // 페이로드가 버퍼보다 크면 저장 대신 소비만
  uint8_t  payload[MSP_SENSOR_MAX_PAYLOAD] = {0};

  // 진단 카운터. crc_errors 나 oversize_frames 가 늘면 배선/보레이트 문제,
  // unknown_frames 가 늘면 function 상수가 틀렸다는 뜻이다.
  uint32_t frames_ok = 0;
  uint32_t crc_errors = 0;
  uint32_t unknown_frames = 0;
  uint32_t oversize_frames = 0;
  uint16_t last_unknown_cmd = 0;

  MspRangefinderSample range;   // 마지막 정상 라이다 값
  MspOpticFlowSample flow;      // 마지막 정상 광류 값
};

// CRC-8/DVB-S2 (다항식 0xD5): 바이트 하나를 crc에 섞는다. MSPv2 표준 체크섬
//  비트마다 최상위 비트가 1이면 왼쪽으로 밀고 0xD5와 XOR, 아니면 그냥 밀기 (8번 반복)
static inline uint8_t mspCrc8DvbS2(uint8_t crc, uint8_t value) {
  crc ^= value;
  for (int bit = 0; bit < 8; bit++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

// 바이트 4개(little endian)를 int32로 조립. p[0]이 가장 낮은 바이트
static inline int32_t mspReadI32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

// 예상치 못한 바이트에서는 IDLE 로 돌아간다. 그 바이트 자체가 '$' 이면
// 곧바로 새 프레임의 시작으로 받아들여야 재동기화가 한 바이트도 안 흘린다.
static inline MspParseResult mspSensorRestart(MspSensorParser &p, uint8_t c) {
  p.state = (c == '$') ? MSP_ST_HEADER_X : MSP_ST_IDLE;
  return MSP_PARSE_NONE;
}

// UART에서 읽은 바이트 1개를 넣는다. 프레임이 완성된 바이트에서만 NONE 이외 값을 돌려준다
static inline MspParseResult mspSensorParseByte(MspSensorParser &p, uint8_t c) {
  switch (p.state) {
    case MSP_ST_IDLE:
      if (c == '$') p.state = MSP_ST_HEADER_X;
      return MSP_PARSE_NONE;

    case MSP_ST_HEADER_X:
      if (c != 'X') return mspSensorRestart(p, c);
      p.state = MSP_ST_HEADER_DIR;
      return MSP_PARSE_NONE;

    case MSP_ST_HEADER_DIR:
      // 방향 바이트: '<' FC로 가는 쪽, '>' FC에서 나오는 쪽, '!' 오류. 셋 다 받아들인다
      if (c != '<' && c != '>' && c != '!') return mspSensorRestart(p, c);
      p.crc = 0;                      // CRC 는 방향 바이트 **다음**부터다
      p.state = MSP_ST_FLAGS;
      return MSP_PARSE_NONE;

    case MSP_ST_FLAGS:
      p.flags = c;
      p.crc = mspCrc8DvbS2(p.crc, c);
      p.state = MSP_ST_CMD_LO;
      return MSP_PARSE_NONE;

    case MSP_ST_CMD_LO:
      p.cmd = c;
      p.crc = mspCrc8DvbS2(p.crc, c);
      p.state = MSP_ST_CMD_HI;
      return MSP_PARSE_NONE;

    case MSP_ST_CMD_HI:
      p.cmd |= (uint16_t)c << 8;   // 상위 바이트를 8비트 밀어 합친다
      p.crc = mspCrc8DvbS2(p.crc, c);
      p.state = MSP_ST_SIZE_LO;
      return MSP_PARSE_NONE;

    case MSP_ST_SIZE_LO:
      p.size = c;
      p.crc = mspCrc8DvbS2(p.crc, c);
      p.state = MSP_ST_SIZE_HI;
      return MSP_PARSE_NONE;

    case MSP_ST_SIZE_HI:
      p.size |= (uint16_t)c << 8;
      p.crc = mspCrc8DvbS2(p.crc, c);
      p.offset = 0;
      p.oversize = (p.size > MSP_SENSOR_MAX_PAYLOAD);   // 버퍼보다 크면 저장하지 않고 개수만 센다
      p.state = (p.size == 0) ? MSP_ST_CRC : MSP_ST_PAYLOAD;   // 페이로드가 없으면 바로 CRC
      return MSP_PARSE_NONE;

    case MSP_ST_PAYLOAD:
      p.crc = mspCrc8DvbS2(p.crc, c);
      if (!p.oversize) p.payload[p.offset] = c;   // oversize면 버퍼 넘침을 막으려고 저장하지 않는다
      p.offset++;
      if (p.offset >= p.size) p.state = MSP_ST_CRC;
      return MSP_PARSE_NONE;

    case MSP_ST_CRC: {
      const uint8_t expected = p.crc;
      p.state = MSP_ST_IDLE;   // 결과와 상관없이 다음 프레임을 기다린다
      if (c != expected) {
        p.crc_errors++;
        return MSP_PARSE_CRC_ERROR;
      }
      if (p.oversize) {
        p.oversize_frames++;
        return MSP_PARSE_UNKNOWN;
      }
      // 메시지 번호와 길이가 둘 다 맞을 때만 해석한다
      if (p.cmd == MSP2_SENSOR_RANGEFINDER && p.size == 5) {
        p.range.quality = p.payload[0];
        p.range.distance_mm = mspReadI32(&p.payload[1]);
        p.frames_ok++;
        return MSP_PARSE_RANGEFINDER;
      }
      if (p.cmd == MSP2_SENSOR_OPTIC_FLOW && p.size == 9) {
        p.flow.quality = p.payload[0];
        p.flow.motion_x = mspReadI32(&p.payload[1]);
        p.flow.motion_y = mspReadI32(&p.payload[5]);
        p.frames_ok++;
        return MSP_PARSE_OPTIC_FLOW;
      }
      // 아는 function 인데 길이가 다르면 그것도 unknown 으로 센다 — 조용히
      // 잘못 해석하는 것보다 카운터가 올라가는 편이 낫다.
      p.unknown_frames++;
      p.last_unknown_cmd = p.cmd;
      return MSP_PARSE_UNKNOWN;
    }

    default:
      p.state = MSP_ST_IDLE;
      return MSP_PARSE_NONE;
  }
}
