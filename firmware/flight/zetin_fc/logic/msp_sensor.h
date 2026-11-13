#pragma once

#include <Arduino.h>

// Matek 3901-L0X (PMW3901 광류 + VL53L0X 라이다) 가 UART 로 밀어넣는 INAV
// MSPv2 센서 프레임 파서. 수신 전용이며 어떤 것도 제어하지 않는다.
//
// 프레임 (iNav src/main/msp/msp_serial.c 상태기계 확인):
//   '$' 'X' <dir>  flags(u8) cmd(u16 LE) size(u16 LE) payload  crc8
//                  └──────────── CRC 적용 구간 ────────────┘
// `$X` 와 방향 바이트('<' '>' '!')는 CRC에 들어가지 않는다.
//
// 페이로드 (iNav src/main/msp/msp_protocol_v2_sensor_msg.h 원문):
//   MSP2_SENSOR_RANGEFINDER 0x1F01 : u8 quality; i32 distanceMm  (5B)
//       distanceMm 은 **범위 밖이면 음수**다. 이 모듈의 작동 범위가
//       8cm~200cm 이므로 **너무 가까울 때도 음수**이고, 부호만으로는
//       위/아래를 구분할 수 없다. 접지 판정은 직전 유효값의 추세를 봐야 한다.
//   MSP2_SENSOR_OPTIC_FLOW  0x1F02 : u8 quality; i32 motionX; i32 motionY (9B)

static const uint16_t MSP2_SENSOR_RANGEFINDER = 0x1F01;
static const uint16_t MSP2_SENSOR_OPTIC_FLOW  = 0x1F02;

// 두 메시지는 5B/9B다. 여유를 두되, 이보다 큰 페이로드는 저장하지 않고
// **소비만 해서 동기를 유지**한다(다른 MSP 트래픽이 섞여도 재동기화된다).
static const uint16_t MSP_SENSOR_MAX_PAYLOAD = 16;

enum MspParseResult : uint8_t {
  MSP_PARSE_NONE = 0,       // 프레임 미완성
  MSP_PARSE_RANGEFINDER,
  MSP_PARSE_OPTIC_FLOW,
  MSP_PARSE_UNKNOWN,        // CRC 정상인데 모르는 function
  MSP_PARSE_CRC_ERROR,
};

enum MspParseState : uint8_t {
  MSP_ST_IDLE = 0,
  MSP_ST_HEADER_X,
  MSP_ST_HEADER_DIR,
  MSP_ST_FLAGS,
  MSP_ST_CMD_LO,
  MSP_ST_CMD_HI,
  MSP_ST_SIZE_LO,
  MSP_ST_SIZE_HI,
  MSP_ST_PAYLOAD,
  MSP_ST_CRC,
};

struct MspRangefinderSample {
  uint8_t quality = 0;
  int32_t distance_mm = 0;
};

struct MspOpticFlowSample {
  uint8_t quality = 0;
  int32_t motion_x = 0;
  int32_t motion_y = 0;
};

struct MspSensorParser {
  uint8_t  state = MSP_ST_IDLE;
  uint8_t  flags = 0;
  uint16_t cmd = 0;
  uint16_t size = 0;
  uint16_t offset = 0;
  uint8_t  crc = 0;
  bool     oversize = false;   // 페이로드가 버퍼보다 크면 저장 대신 소비만
  uint8_t  payload[MSP_SENSOR_MAX_PAYLOAD] = {0};

  // 진단 카운터. crc_errors 나 oversize_frames 가 늘면 배선/보레이트 문제,
  // unknown_frames 가 늘면 function 상수가 틀렸다는 뜻이다.
  uint32_t frames_ok = 0;
  uint32_t crc_errors = 0;
  uint32_t unknown_frames = 0;
  uint32_t oversize_frames = 0;
  uint16_t last_unknown_cmd = 0;

  MspRangefinderSample range;
  MspOpticFlowSample flow;
};

static inline uint8_t mspCrc8DvbS2(uint8_t crc, uint8_t value) {
  crc ^= value;
  for (int bit = 0; bit < 8; bit++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

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
      p.cmd |= (uint16_t)c << 8;
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
      p.oversize = (p.size > MSP_SENSOR_MAX_PAYLOAD);
      p.state = (p.size == 0) ? MSP_ST_CRC : MSP_ST_PAYLOAD;
      return MSP_PARSE_NONE;

    case MSP_ST_PAYLOAD:
      p.crc = mspCrc8DvbS2(p.crc, c);
      if (!p.oversize) p.payload[p.offset] = c;
      p.offset++;
      if (p.offset >= p.size) p.state = MSP_ST_CRC;
      return MSP_PARSE_NONE;

    case MSP_ST_CRC: {
      const uint8_t expected = p.crc;
      p.state = MSP_ST_IDLE;
      if (c != expected) {
        p.crc_errors++;
        return MSP_PARSE_CRC_ERROR;
      }
      if (p.oversize) {
        p.oversize_frames++;
        return MSP_PARSE_UNKNOWN;
      }
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
