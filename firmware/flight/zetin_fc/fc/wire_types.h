// zetin_fc/fc/wire_types.h — ZIMU/ZCAL 와이어 형식, 텔레메트리 샘플 구조체, raw IMU 링, 지연 이벤트 링(WIFI_LATENCY_DEBUG).
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// [IMU 원시 데이터 스트림(ZIMU) 정리]
//  pid_task(1kHz)가 매 tick 두 IMU의 레지스터 원값을 링 버퍼에 넣고(생산자),
//  udp_task가 최대 50개씩 묶어 UDP 바이너리 패킷으로 PC에 보낸다(소비자)
//  20Hz CSV 텔레메트리로는 안 보이는 1kHz 진동/지연을 오프라인으로 분석하려는 용도
// 링 버퍼 칸 수 (2의 거듭제곱이어야 아래 MASK가 동작)
static const uint32_t IMU_RAW_RING_SIZE = 512;
// index & MASK = index % 512 (나눗셈 없이 빠르게)
static const uint32_t IMU_RAW_RING_MASK = IMU_RAW_RING_SIZE - 1;
// 한 패킷에 담는 최대 샘플 수 (50ms 분량)
static const uint8_t IMU_RAW_BATCH_MAX = 50;
// v2: ImuRawSample 에 failsafe_phase 추가. 디코더는 v1 도 계속 읽는다.
static const uint8_t IMU_RAW_VERSION = 2;

/*
GCC/Clang 컴파일러 확장 문법-->컴파일러에게 이 함수, 변수, 구조체는 특별하게 다뤄달라고 지시할 때 사용
기본 형태
void foo(void) __attribute__((속성));
int x __attribute__((속성)) = 0;
//
- 괄호 두 겹으로 쓰는 것이 규칙, 속성 여러 개는 쉼표로 이어 씀
- packed: 구조체 멤버 사이 패딩을 없애는 용도-> 통신 패킷, 센서 레지스터 구조체 등에 사용
*/
struct __attribute__((packed)) ImuRawSample {
  // 2B  직전 샘플과의 시간 간격 (µs)
  uint16_t dt_us;
  // 6B  IMU1 자이로 x,y,z (레지스터 원값)
  int16_t imu1_gyro[3];
  // 6B  IMU1 가속도
  int16_t imu1_accel[3];
  // 6B  IMU2 자이로
  int16_t imu2_gyro[3];
  // 6B  IMU2 가속도
  int16_t imu2_accel[3];
  // v2. 1kHz 로그를 자동착륙 위상으로 자를 수 있어야 오프라인에서 착지 판별식
  // 후보를 평가할 수 있다. .bin 은 펌웨어 micros(), 20Hz CSV 는 PC 벽시계라
  // 공통 키가 없어서 2026-08-01 분석이 여기서 막혔다.
  // 1B  그 순간의 자동착륙 단계
  uint8_t failsafe_phase;
// 합계 27B
};

// 패킷 머리(20B)
struct __attribute__((packed)) ImuRawHeader {
  // "ZIMU" - 받는 쪽이 어떤 패킷인지 식별
  char magic[4];
  // 2 - 형식이 바뀌어도 디코더가 구분할 수 있게
  uint8_t version;
  // 이 패킷에 든 샘플 수(1~50)
  uint8_t n_samples;
  // 예약 (0)
  uint16_t reserved;
  // 패킷 일련번호 -> 번호가 비면 UDP 패킷 손실
  uint32_t batch_seq;
  // 첫 샘플의 절대 시각
  uint32_t base_t_us;
  // 링 버퍼가 넘쳐서 버린 샘플 누적 개수
  uint32_t dropped;
};

// 실제 전송 단위
struct __attribute__((packed)) ImuRawDatagram {
  // 20B
  ImuRawHeader header;
  // 27B * 50 = 1350B
  ImuRawSample samples[IMU_RAW_BATCH_MAX];
};

struct __attribute__((packed)) ImuCalDatagram {
  // ZCAL: ZIMU의 원시값을 실제 물리 단위로 바꾸는 데 필요한 보정 상수들을 담은 패킷
  // ZIMU에 담긴 건 센서 레지스터 원값-> 이 숫자만으로 실제 회전 속도, 가속도 알 수 없음-> 펌웨어 안에서 '실제값 = (원값 * 스케일) - 바이어스'로 바꿔서 사용
  // raw 스트림이 켜져 있는 동안 1초에 1번 전송 (UDP 손실, PC 로그 중간 시작 대비)
  // 4B  magic = "ZCAL": 보정값 패킷이라는 표시
  char magic[4];
  // 1B  형식 버전(현재 2)
  uint8_t version;
  // 3B  예비 칸. 뒤의 float들을 4의 배수 위치(오프셋 8)부터 시작하게 맞춤
  uint8_t reserved[3];
  // 12B IMU1 자이로 바이어스 x,y,z (dps). 부팅 시 정지 상태에서 측정한 영점 오차
  float gyro_bias1[3];
  // 12B IMU2 자이로 바이어스 x,y,z (dps). IMU1 센서축 기준으로 표현
  float gyro_bias2[3];
  // 4B  IMU1 가속도 보정 배율. 부팅 시 정지 상태 |a| = 1g가 되도록 측정
  float accel_scale1;
  // 4B  IMU2 가속도 보정 배율
  float accel_scale2;
  // 4B  자이로 원값(LSB) -> dps 변환 상수 (GYRO_SCALE, FSR ±2000dps 기준)
  float gyro_scale;
  // 4B  가속도 원값(LSB) -> g 변환 상수 (ACCEL_SCALE, FSR ±16g 기준)
  float accel_scale;
  // 12B IMU2 축 부호 x,y,z (+1/-1). IMU2 장착 방향을 IMU1 축에 맞추는 값
  float imu2_sign[3];
// 합계 60B
};

// Arduino 는 .ino 함수의 원형을 뒤쪽 전역 선언보다 앞에 끼워 넣으므로, 스냅숏 반환 타입은
// 이 앞 블록에 선언해야 했다(모듈로 나눈 뒤 .ino 에는 함수가 없지만 순서는 그대로 둔다).
// 모터 분배기(allocator) 결과를 pid_task → sendTelemetry로 넘기는 한 tick 스냅샷 (텔레메트리 필드 66~75)
struct AllocationTelemetrySample {
  // 이번 tick 자세 명령이 축소됐는가 (rp_scale<1 또는 yaw_scale<1)
  bool scaled;
  // roll/pitch 배율 (1 = 그대로)
  float rp_scale;
  // yaw 배율 λ (1 = 그대로, 0 = yaw 포기)
  float yaw_scale;
  // 실제 적용된 스로틀 (µs)
  float collective_us;
  // 축별 PID 전체 출력 (µs)
  float pid_roll_us;
  float pid_pitch_us;
  float pid_yaw_us;
  // 그중 I항 기여 (µs)
  float i_roll_us;
  float i_pitch_us;
  float i_yaw_us;
  // YawAuthorityState (0 NORMAL, 1 LIMITED, 2 RECOVERING)
  int yaw_authority_state;
  int requested_us;          // allocator 입력 collective(같은 tick)
  // 같은 tick 의 failsafe·착지 상태. Core 0 가 따로 읽으면 resume 초기화와 섞인다.
  uint8_t fs_phase;
  uint8_t land_mode;
  uint8_t land_stage;
  uint8_t land_cut;
  float land_ref_m;
};

// static_assert(조건, "틀렸을 때 보여줄 메시지") --> 컴파일할 때 조건을 검사해서, 틀리면 컴파일 자체를 실패시키는 C++ 문법
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
  // 원형으로 재사용하는 저장 공간 (512칸)
  ImuRawSample samples[IMU_RAW_RING_SIZE];
  // 쓰기 위치 (Core 1만 증가시킴)
  volatile uint32_t head = 0;
  // 읽기 위치 (Core 0만 증가시킴)
  volatile uint32_t tail = 0;
  // 가득 차서 버린 수
  volatile uint32_t dropped = 0;
  // 버퍼가 비었을 때 들어온 첫 샘플 시각
  volatile uint32_t first_t_us = 0;
};

static ImuRawRing imuRawRing;
// 링에서 꺼낸 샘플을 잠깐 모아 두는 작업대
// IMU_RAW_BATCH_MAX = 50
static ImuRawSample imuRawBatch[IMU_RAW_BATCH_MAX];
// 헤더를 붙인 보낼 패킷--> udp.write()에 그대로 넘김
static ImuRawDatagram imuRawDatagram;
// 보정값 패킷(ZCAL)
static ImuCalDatagram imuCalDatagram;
// 기본 ON
// volatile: 컴파일러가 임의로 값을 레지스터에 캐싱, 코드를 최적화하지 않고, 매번 실제 메모리에 직접 접근하도록 지시하는 키워드
volatile bool raw_stream_enabled = true;
// 생산자 쪽(pid_task, 넣을 때), 소비자 쪽(udp_task, 꺼내서 보낼 때)-->패킷의 첫 샘플 시각이 헤더의 base_t_us가 됨
// 생산자: 기준 시각이 있나?
static bool rawProducerTimeValid = false;
// 생산자: 직전에 넣은 샘플의 시각
static uint32_t rawProducerLastUs = 0;
// 소비자: 기준 시각이 있나?
static bool rawConsumerTimeValid = false;
// 소비자: 직전에 꺼낸 샘플의 시각
static uint32_t rawConsumerLastUs = 0;
// 역할: 패킷 일련번호. 보낼 때마다 +1, 사용처: PC가 번호가 빈 걸 보고 UDP 손실을 감지
static uint32_t rawBatchSeq = 0;
// 역할: 마지막 ZIMU 전송 시각, 사용처: 50개가 안 모여도 50ms가 지나면 보냄
static uint32_t rawLastSendMs = 0;
// 마지막 ZCAL 전송 시각: 1초마다 보정값 재전송
static uint32_t rawLastCalMs = 0;

// [WiFi 지연 디버그] 빌드 옵션으로 WIFI_LATENCY_DEBUG=1을 주면 켜진다 (기본 0 = 아래 코드가 통째로 빠짐)
//  udp_task/pid_task가 기준(30ms / 5ms)보다 오래 걸린 순간을 링 버퍼에 기록하고 loop()가 시리얼로 출력한다
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
