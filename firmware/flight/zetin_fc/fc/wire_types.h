// zetin_fc/fc/wire_types.h — ZIMU/ZCAL 와이어 형식, 텔레메트리 샘플 구조체, raw IMU 링, 지연 이벤트 링(WIFI_LATENCY_DEBUG).
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
static const uint32_t IMU_RAW_RING_SIZE = 512;
static const uint32_t IMU_RAW_RING_MASK = IMU_RAW_RING_SIZE - 1;
static const uint8_t IMU_RAW_BATCH_MAX = 50;
// v2: ImuRawSample 에 failsafe_phase 추가. 디코더는 v1 도 계속 읽는다.
static const uint8_t IMU_RAW_VERSION = 2;

struct __attribute__((packed)) ImuRawSample {
  uint16_t dt_us;
  int16_t imu1_gyro[3];
  int16_t imu1_accel[3];
  int16_t imu2_gyro[3];
  int16_t imu2_accel[3];
  // v2. 1kHz 로그를 자동착륙 위상으로 자를 수 있어야 오프라인에서 착지 판별식
  // 후보를 평가할 수 있다. .bin 은 펌웨어 micros(), 20Hz CSV 는 PC 벽시계라
  // 공통 키가 없어서 2026-08-01 분석이 여기서 막혔다.
  uint8_t failsafe_phase;
};

struct __attribute__((packed)) ImuRawHeader {
  char magic[4];
  uint8_t version;
  uint8_t n_samples;
  uint16_t reserved;
  uint32_t batch_seq;
  uint32_t base_t_us;
  uint32_t dropped;
};

struct __attribute__((packed)) ImuRawDatagram {
  ImuRawHeader header;
  ImuRawSample samples[IMU_RAW_BATCH_MAX];
};

struct __attribute__((packed)) ImuCalDatagram {
  char magic[4];
  uint8_t version;
  uint8_t reserved[3];
  float gyro_bias1[3];
  float gyro_bias2[3];
  float accel_scale1;
  float accel_scale2;
  float gyro_scale;
  float accel_scale;
  float imu2_sign[3];
};

// Arduino's generated function prototypes are inserted before later global
// declarations, so snapshot return types must be declared in this top block.
struct AllocationTelemetrySample {
  bool scaled;
  float rp_scale;
  float yaw_scale;
  float collective_us;
  float pid_roll_us;
  float pid_pitch_us;
  float pid_yaw_us;
  float i_roll_us;
  float i_pitch_us;
  float i_yaw_us;
  int yaw_authority_state;
  int requested_us;          // allocator 입력 collective(같은 tick)
  // 같은 tick 의 failsafe·착지 상태. Core 0 가 따로 읽으면 resume 초기화와 섞인다.
  uint8_t fs_phase;
  uint8_t land_mode;
  uint8_t land_stage;
  uint8_t land_cut;
  float land_ref_m;
};

static_assert(sizeof(ImuRawSample) == 27, "ZIMU v2 sample must be 27 bytes");
static_assert(sizeof(ImuRawHeader) == 20, "ZIMU header must be 20 bytes");
static_assert(sizeof(ImuRawDatagram) == 1370,
              "maximum ZIMU v2 datagram must be 1370 bytes");
static_assert(sizeof(ImuCalDatagram) == 60, "ZCAL datagram must be 60 bytes");

// 지상국 `imucal`: 정지 상태 IMU 보정. Core 1 이 자기 1 kHz 샘플로 재고(SPI 를 두 코어가
// 같이 쓰지 않게), Core 0 이 결과를 NVS 에 저장하고 보고한다.
enum ImuCalReason : uint8_t {
  IMUCAL_OK = 0, IMUCAL_MOVED, IMUCAL_ACCEL, IMUCAL_READ, IMUCAL_ARMED
};
struct ImuCalResult {
  bool ok;
  uint8_t reason;
  float bias1[3], bias2[3];
  float scale1, scale2;
  float max_sd;
};
struct ImuCalAccum {
  bool active;
  uint32_t n, ticks;
  double sum1[3], sq1[3], sum2[3], sq2[3];
  double amag1, amag2;
};
// NVS 에 저장하는 형식. magic 이 다르면(빈 NVS·다른 형식) 컴파일 값을 쓴다.
struct StoredImuCal {
  uint32_t magic;
  float bias1[3], bias2[3];
  float scale1, scale2;
};

// 광류 위치·고도 유지 (Arduino 프로토타입 생성 때문에 함수보다 앞에 둔다)
// Core 0 → Core 1: 완성된 프레임 스냅샷. seq 는 로컬 카운터다(MSP 에 송신 seq 없음).
struct FlowSensorSnapshot {
  uint32_t flow_seq;
  int32_t flow_x, flow_y;
  int flow_quality;
  uint32_t flow_t_us;
  bool flow_timing_ok;
  uint32_t range_seq;
  int32_t range_mm;
  uint32_t range_t_us;
  bool range_timing_ok;
  uint32_t crc_errors;
};

struct FlowHoldTelemetry {
  uint32_t flow_seq;
  float flow_dt_ms;
  int flow_reject;
  float c_roll, c_pitch;
  float pos_r, pos_p;
  float height_m, vz_ms;
  int poshold_state;
  float candidate_roll_deg, candidate_pitch_deg;
  float applied_roll_deg, applied_pitch_deg;
  int althold_state;
  float candidate_alt_us, applied_alt_us;
  int flowcal_state;
  uint32_t flowcal_samples;
};

struct ImuRawRing {
  ImuRawSample samples[IMU_RAW_RING_SIZE];
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
  volatile uint32_t dropped = 0;
  volatile uint32_t first_t_us = 0;
};

static ImuRawRing imuRawRing;
static ImuRawSample imuRawBatch[IMU_RAW_BATCH_MAX];
static ImuRawDatagram imuRawDatagram;
static ImuCalDatagram imuCalDatagram;
volatile bool raw_stream_enabled = true;
static bool rawProducerTimeValid = false;
static uint32_t rawProducerLastUs = 0;
static bool rawConsumerTimeValid = false;
static uint32_t rawConsumerLastUs = 0;
static uint32_t rawBatchSeq = 0;
static uint32_t rawLastSendMs = 0;
static uint32_t rawLastCalMs = 0;

#ifndef WIFI_LATENCY_DEBUG
#define WIFI_LATENCY_DEBUG 0
#endif
#if WIFI_LATENCY_DEBUG
#ifndef WIFI_LATENCY_UDP_THRESHOLD_US
#define WIFI_LATENCY_UDP_THRESHOLD_US 30000U
#endif
#ifndef WIFI_LATENCY_PID_THRESHOLD_US
#define WIFI_LATENCY_PID_THRESHOLD_US 5000U
#endif

enum LatKind : uint8_t {
  LAT_UDP_LOOP,
  LAT_UDP_PARSE,
  LAT_UDP_SEND,
  LAT_PID
};

struct LatEvt {
  uint32_t t_ms;
  uint32_t dur_us;
  uint8_t kind;
};

static const uint32_t LAT_RING_SIZE = 128;
static const uint32_t LAT_RING_MASK = LAT_RING_SIZE - 1;

struct LatRing {
  LatEvt events[LAT_RING_SIZE];
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
  volatile uint32_t dropped = 0;
};

static LatRing udpLatRing;
static LatRing pidLatRing;

static uint32_t latUdpMaxUs = 0;
static uint32_t latPidMaxUs = 0;
static uint32_t latUdpLoopOver30 = 0;
static uint32_t latUdpLoopOver100 = 0;
static uint32_t latUdpLoopOver300 = 0;
static uint32_t latPidOver5 = 0;
static uint32_t latLastSummaryMs = 0;
#endif
