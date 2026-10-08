#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ICM42670P.h>
#include <DFRobot_BMM350.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <errno.h>
#include <float.h>

#include "mag_yaw_fusion.h"
#include "yaw_command.h"
#include "failsafe_land.h"
#include "msp_sensor.h"

static const uint32_t IMU_RAW_RING_SIZE = 512;
static const uint32_t IMU_RAW_RING_MASK = IMU_RAW_RING_SIZE - 1;
static const uint8_t IMU_RAW_BATCH_MAX = 50;
// v2: ImuRawSample 에 failsafe_phase 추가. 디코더는 v1 도 계속 읽는다.
static const uint8_t IMU_RAW_VERSION = 2;


/*
GCC/Clang 컴파일러 확장 문법
기본 형태 
void foo(void) __attribute__((속성));
int x __attribute__((속성)) = 0;
*/

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

static_assert(sizeof(ImuRawSample) == 27, "ZIMU v2 sample must be 27 bytes");
static_assert(sizeof(ImuRawHeader) == 20, "ZIMU header must be 20 bytes");
static_assert(sizeof(ImuRawDatagram) == 1370,
              "maximum ZIMU v2 datagram must be 1370 bytes");
static_assert(sizeof(ImuCalDatagram) == 60, "ZCAL datagram must be 60 bytes");

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
// ==========================================================
// 1. 튜닝 파라미터
// ==========================================================
// [Outer] 각도 P: 각도 오차 -> 목표 각속도(dps)
volatile float Kp_Angle_Roll  = 6.0f;
volatile float Kp_Angle_Pitch = 6.0f;
volatile float Kp_Angle_Yaw   = 3.0f;

// [Inner] 각속도 PID: 각속도 오차 -> 모터 파워
// Ki_Rate_Roll/Pitch: 0.005 -> 0.05 (호스트 SIL 근거, commit 735f77d). 0.005는
// 일정 CG 치우침 외란에서 10초 후 ±50µs 클램프의 ~1.4%만 써 적분기가 사실상
// 비활성이었다(파라미터 무관하게 강건한 결론). 0.05는 적분기를 활성화하되
// 클램프의 <5µs만 쓰는 보수적 시작값이다. SIL의 완전트림 값(0.5)은 실기 지연·
// 노이즈에서 진동 위험이 있어 일부러 피했고, 최종값은 테더 벤치에서 상향 튜닝한다.
volatile float Kp_Rate_Roll  = 0.50f, Ki_Rate_Roll  = 0.05f, Kd_Rate_Roll  = 0.015f;
volatile float Kp_Rate_Pitch = 0.50f, Ki_Rate_Pitch = 0.05f, Kd_Rate_Pitch = 0.015f;
volatile float Kp_Rate_Yaw   = 1.50f, Ki_Rate_Yaw   = 0.05f,  Kd_Rate_Yaw   = 0.0f;

volatile int  base_throttle = 1000;
volatile int  min_throttle  = 1050;
volatile int  max_throttle  = 1300;
volatile bool yaw_hold_override = false;   // "yaw 1": heading 고정 + 재슬레이빙 금지
volatile bool yaw_hold_now = false;   // 텔레메트리용: 현재 heading 잠금 중인가

const int MAG_THROTTLE_REF_US = 1000;   // 보정 기준(모터 off 근처)
// 모터 전류 간섭 보정: body-frame ΔB(µT)는 base_throttle에 비례한다고 근사한다.
// hard/soft-iron 보정과 body 축 변환을 거친 mag에서, 틸트 보정 전에 뺀다.
// mag.x -= mag_comp_x*(base_throttle-REF) 등. µT/µs. 런타임 `magc x y z`. 0=off.
// 벤치 특성화 2026-07-27 (log 023805, 검증 024330): throttle 간섭 +3.64→+0.02°/100µs.
// 2026-08-04: soft-iron 행렬 도입으로 이 계수가 사는 좌표계가 바뀌었다. 간섭은
// raw 축의 물리적 ΔB이고 보정은 선형이므로 새 계수 = W @ 기존계수로 환산했다.
// 같은 날 log 032922의 벤치 4단계 재검증 통과 기록이 있다. 방위·배터리 조건을
// 넓힌 검증이나 현재 하드웨어의 재시험을 뜻하지 않는다. docs/bmm350_yaw_bench_test.md.
volatile float mag_comp_x =  0.007467272f;  // µT/µs
volatile float mag_comp_y = -0.001149868f;
volatile float mag_comp_z = -0.000526263f;

// ==========================================================
// 2. 시스템 상수
// ==========================================================
const char* WIFI_SSID    = "Drone_Tuning";
const char* WIFI_PASS    = "12345678";
const int   UDP_PORT     = 4210;
const int   WIFI_CHANNEL = 11;  // ch6 혼잡(인근 AP 다수) → 스캔상 가장 빈 채널로 이동

const int pinM1   = 4;   // FL
const int pinM2   = 5;   // RR
const int pinM3   = 6;   // FR
const int pinM4   = 7;   // RL
const int SPI_CS1 = 10;  // IMU1
const int SPI_CS2 = 9;   // IMU2
// Matek 3901-L0X (광류+라이다). 모듈 TX -> 이 핀, 3.3V 로직이라 직결한다.
// 모듈 RX 는 연결하지 않는다 — 우리는 보낼 것이 없고 수신 전용이다.
// 19/20 은 USB D-/D+ 라 피했고 스트래핑 핀도 피했다.
const int MSP_SENSOR_RX_PIN = 16;
const uint32_t MSP_SENSOR_BAUD = 115200;
// 이보다 오래 프레임이 없으면 quality 를 -1 로 보고한다. 얼어붙은 센서가
// 마지막 값으로 유효해 보이면 안 된다(MAG_STALE_MS 와 같은 이유).
const uint32_t MSP_SENSOR_STALE_MS = 500;
// udp_task 한 반복에서 처리할 바이트 상한. BMM350 의 블로킹 I2C 가 RC
// 수신큐를 넘치게 했던 전례가 있어 이 경로는 절대 블로킹하지 않는다.
// 실제 유량은 20Hz x ~15B = 300B/s 라 이 상한에 닿을 일이 없다.
const int MSP_SENSOR_MAX_BYTES_PER_TICK = 64;

const int BMM_SDA = 35;
const int BMM_SCL = 36;
const int LDO_SHD = 38;

const int ESC_FREQ   = 400;
const int ESC_RES    = 14;
const int ESC_PERIOD = 2500;                 // us
const int ESC_MAXDUTY = (1 << ESC_RES) - 1;  // magic number 제거

const float GYRO_SCALE   = 1.0f / 16.4f;     // raw -> deg/s
const float ACCEL_SCALE  = 1.0f / 2048.0f;   // raw -> g
const float SAFETY_ANGLE = 60.0f;
const float YAW_DEADZONE = 0.3f;
const float MAX_TARGET_ANGLE_RP = 30.0f;     // UDP 오입력에 대한 자세 명령 제한
const float TRIM_MAX_DEG = 10.0f;   // 트림 절대값 상한
const float MAX_TARGET_RATE_RP  = 300.0f;    // outer-loop 출력 제한 (deg/s)
const float MAX_TARGET_RATE_YAW = 180.0f;
// yaw 모드 판정 임계치. 벤치에서 조정한다(설계 문서 §1 참조).
const float YAW_RATE_DEADZONE   = 3.0f;    // 명령 각속도 절댓값이 이 값 미만이면 스틱 중립
const float YAW_HOLD_SETTLE_DPS = 10.0f;   // bodyGz 절댓값이 이 값 미만이면 새 잠금 진입 가능
// 250Hz outer loop에서 K=0.001 -> tau ≈ 0.004/K = 4s.
const float K_MAG = 0.001f;
const uint32_t MAG_SAMPLE_PERIOD_MS = 20;    // BMM350 read = 50Hz, core 0 only
const uint32_t MAG_STALE_MS = 100;

// scripts/magcal_fit.py 출력값을 아래 두 상수에 붙여 넣는다.
// 2026-08-04 캘리브 (log flight_log_2026-08-04_023840.csv, 1255샘플 중 inlier
// 1119, 커버리지비 2.08). 이전 min/max offset은 여기서 6.79µT 벗어나 있었고
// 그것은 수평성분 32.2µT 대비 최대 ±12.2° heading 오차였다.
// 교차검증: 무작위 절반 분할 8회의 중심 차이 중앙값 0.29µT (시간 분할은
// 후반 커버리지비가 39.6이라 b_z를 구속하지 못하므로 쓰면 안 된다).
// 물리 정합성: 피팅 반경 50.10µT vs 한국 총강도 실측 ~50.4µT (오차 0.6%).
const float MAG_HARD_IRON[3] = {-3.457566904f, 4.794903225f, -17.307261041f};
const float MAG_SOFT_IRON[3][3] = {
  {0.998555071f, 0.005825468f, 0.018437852f},
  {0.005825468f, 0.976739944f, 0.006050833f},
  {0.018437852f, 0.006050833f, 1.026752877f},
};

// BMM350 축이 기체 축과 다르면 벤치 sign test 뒤 여기만 바꾼다.
const float MAG_BODY_SIGN_X = 1.0f;
const float MAG_BODY_SIGN_Y = 1.0f;
const float MAG_BODY_SIGN_Z = 1.0f;

// IMU2는 x,z축이 IMU1 대비 반전 (y는 동일). 첫 파일 기준 그대로.
const float IMU2_SIGN_X = -1.0f;
const float IMU2_SIGN_Y =  1.0f;
const float IMU2_SIGN_Z = -1.0f;
static const float IMU1_SIGN[3] = { 1.0f, 1.0f, 1.0f };
static const float IMU2_SIGN[3] = { IMU2_SIGN_X, IMU2_SIGN_Y, IMU2_SIGN_Z };

// --- 안전/redundancy 임계값 (하드웨어 맞춰 튜닝 필요) ---
const uint32_t RC_TIMEOUT_MS      = 500;
// 아래 failsafe 상수는 전부 벤치 조정 대상이다.
constexpr int CTRL_MARGIN = 150;
const int      FS_DESCENT_DELTA_US = 60;
static_assert(FS_DESCENT_DELTA_US < CTRL_MARGIN,
              "FS_DESCENT_DELTA_US must be less than CTRL_MARGIN");
// 컴파일 타임 가드는 delta < CTRL_MARGIN만 검사한다. 실제 하한은
// delta < min(CTRL_MARGIN, entry_throttle - 1050)이며 entry_throttle은
// 런타임 값이라 static_assert로 잡을 수 없다. 예: entry=1150이면 delta=120도
// 이미 collective 하한에 걸려 실제 하강이 일어나지 않는다.
const uint32_t FS_MIN_DESCEND_MS   = 1000;
constexpr float FS_LAND_LPF_ALPHA = 0.03f;   // 1kHz에서 약 5Hz
// 능동 프로브 상수는 모두 Stage E 벤치 조정 대상이다. 호버 1340us는 아이들
// 1000us 대비 340us이고, 그 11.8%인 40us 딥은 명목 질량의 기존 동작을
// 보존한다. 질량이 달라져도 같은 추력 비율을 유지하도록 hover_est 기준으로
// 매 자동착륙 진입 때 계산한다. 런타임 결과는 측정 가능한 20us 이상,
// CTRL_MARGIN 미만으로 clamp하고 clamp 여부를 진입 로그로 공개한다.
// 2026-08-01: 딥을 2배로 올려 공중 응답만 키우려 했으나 **이 기체에서는
// 불가능**하다. 전달 가능한 딥의 상한은
//     (hover_est - FS_DESCENT_DELTA_US) - CTRL_MARGIN - 1050
// 이고 hover_est=1290 에서 그 값이 30us 인데 현재 딥이 이미 34us 다. 더 키우면
// 믹서가 collective 를 밀어올려 전달률이 FS_PROBE_MIN_DELIVERY_FRAC(80%) 아래로
// 떨어지고 BLOCKED 가 된다(2배면 44%). hover_est 1340 이상이면 여유가 생기지만
// 우리 호버는 1290 이다. R13 회귀 시험이 이 한계를 잡아냈다.
// 딥을 늘리려면 CTRL_MARGIN 이나 1050 하한을 먼저 손봐야 하며 둘 다 안전
// 관련이라 별도 검토가 필요하다 → docs/failsafe_land_research.md
constexpr float FS_PROBE_DIP_FRAC      = 0.118f;
const int      FS_PROBE_DIP_MIN_US      = 20;
const uint32_t FS_PROBE_PERIOD_MS       = 400;
const uint32_t FS_PROBE_DIP_MS          = 120;
const uint32_t FS_PROBE_SAMPLE_DELAY_MS = 30;
// ⚠️ 이 값은 **판정에 쓰이지 않는다.** 프로브는 텔레메트리 41~43 기록 전용이고
// 착지 컷은 시간 기반 백스톱이 한다(아래 FS_MAX_MS, 그리고 하강 블록의
// `const bool landed = false`).
//
// 0.06 → 0.03 변경의 근거는 **철회됐다**(07231eb). 근거였던 "2026-08-01 공중
// 오판 9회 중 4회"는 20Hz 단일 샘플을 자유낙하로 오독한 것이고, 그 하강은
// 실제로는 지면에서 났다 — 즉 공중 오판은 일어나지 않았다. 오히려 0.03으로
// 내린 뒤 015448에서 지면 응답 10회가 전부 임계 위가 되어 확정이 불가능해졌다.
// 되돌릴 값이지만 프로브가 판정에서 빠진 지금은 무의미하므로 그대로 둔다.
// 전말은 docs/failsafe_land_research.md.
constexpr float FS_PROBE_RESPONSE_G    = 0.03f;
// 응답은 전달 딥에 비례한다. 기존 1.5배 명목 응답 보장에서 80%를 요구하면
// 임계 대비 1.2배가 남지만, 50% 전달은 임계 아래라 판정 근거가 사라진다.
constexpr float FS_PROBE_MIN_DELIVERY_FRAC = 0.80f;
// 계단 응답은 response=FS_PROBE_DIP_FRAC*(1-exp(-FS_PROBE_DIP_MS*
// FS_LAND_LPF_ALPHA/1ms))로 LPF와 결합된다. V2 SIL 실측은 alpha=0.03에서
// 0.1146g, 0.01에서 0.0789g, 0.005에서 0.0464g였다. 따라서 alpha를 낮추면
// 노이즈뿐 아니라 프로브 감도도 같이 낮아진다. 딥은 최소 3τ(95% 응답) 동안
// 지속하고, 그때 관측 가능한 응답은 판정 임계의 1.5배보다 커야 한다.
static_assert(FS_PROBE_DIP_MS * FS_LAND_LPF_ALPHA >= 3.0f,
              "probe dip must span at least three LPF time constants");
static_assert(FS_PROBE_DIP_FRAC * 0.95f > 1.5f * FS_PROBE_RESPONSE_G,
              "observable probe response must exceed the decision margin");
static_assert(
    FS_PROBE_MIN_DELIVERY_FRAC * FS_PROBE_DIP_FRAC * 0.95f
        > 1.2f * FS_PROBE_RESPONSE_G,
    "minimum delivered probe must preserve the response margin");
const float    FS_LAND_ACCEL_TOL_G      = 0.10f;
// ⚠️ 이 값도 **판정에 쓰이지 않는다** — 프로브 상태(텔레메트리 41~42)를 계산할
// 때만 들어간다.
//
// 2에서 4로 올린 근거였던 "2026-08-01 공중에서 CUT_LANDED가 났다"는 관측은
// **철회됐다**(07231eb). 그 컷은 지면에서 났고 정상 동작이었다. 다만 함께
// 측정한 사실 하나는 유효하다: 딥이 없어도 호버 중 120ms 창의 |accel| 낙폭이
// p95 0.0904g로 임계 0.06g보다 크다 — 교란만으로 프로브 신호 전체 크기가
// 나온다. 연속 판정 횟수를 늘려도 판별식이 갖지 못한 정보를 만들 수는 없고,
// 그래서 프로브 자체가 판정에서 빠졌다. 근본 대책은
// docs/failsafe_land_research.md (거리계로 착지를 측정하는 쪽).
const uint8_t  FS_PROBE_CONFIRM_N       = 4;
// 프로브가 판정에서 빠진 뒤로 이 값은 **하강 시간 예산**이다. 고도나 접지를
// 확인하지 않으므로 예산이 부족하면 공중에서도 TIMEOUT으로 모터가 꺼진다.
//
// 2026-08-01 실측 (접지 충격 임펄스 적분으로 접지 속도를 재고 등가속 모델
// v=a·t, h=½a·t² 로 역산):
//   023021  t=0.80s v=0.76m/s → a=0.95   h=0.30m
//   023226  t=0.82s v=1.30m/s → a=1.59   h=0.53m
//   023904  t=1.76s v=2.11m/s → a=1.20   h=1.86m   ← 환경상 최고 고도
// 평균 a≈1.24 m/s² (추력 부족 60/290=21%=2.06 m/s² 에서 항력만큼 감소).
// 아래는 이 평균 가속도를 일정하다고 가정한 외삽이며 실측 고도 한계가 아니다:
//   2.0s→2.49m  2.5s→3.89m  3.0s→5.60m  3.5s→7.62m
// 3.0s에서 실제로 5.60m를 내려온다는 보장은 없다. 아래 static_assert는
// 프로브 시간 배치만 검사한다(2720ms 초과 필요). 2.5s는 컴파일되지 않는다.
const uint32_t FS_MAX_MS           = 3000;
const uint32_t FS_RESUME_MAX_MS    = 3U * FS_MAX_MS;
static_assert(FS_PROBE_DIP_MIN_US < CTRL_MARGIN,
              "FS_PROBE_DIP_MIN_US must be less than CTRL_MARGIN");
static_assert(FS_PROBE_SAMPLE_DELAY_MS < FS_PROBE_DIP_MS,
              "probe sample delay must be shorter than the dip");
static_assert(FS_PROBE_DIP_MS < FS_PROBE_PERIOD_MS,
              "probe dip must finish before the next probe");
// 검출기를 제어에 쓰던 당시의 시간 배치 가드다. 현재 반환값은 기록에만 쓴다.
// 당시 요구는 "접지 후 confirm_n회 연속 무반응이 백스톱 전에 끝날 것"인데
// 접지 시각은 런타임 값이라 컴파일 타임에 못 쓴다. 대신 필요조건만 건다:
// 접지 전에 버려지는 공중 프로브를 최소 1회 허용하고도 confirm_n회가 더
// 들어가야 한다 = 총 confirm_n+1회. 이건 필요조건일 뿐 충분조건이 아니다 —
// 접지가 늦으면 여전히 타임아웃이 날 수 있고, 그쪽은 SIL V1/V2/V4가 잡는다.
// (confirm_n-1로 쓰면 period=2000에서 3120<5000으로 통과하지만 SIL은 3건
//  타임아웃이었다. 그 오차를 변조 검사로 발견해 고친 식이다.)
static_assert(FS_MIN_DESCEND_MS
                  + FS_PROBE_PERIOD_MS * FS_PROBE_CONFIRM_N
                  + FS_PROBE_DIP_MS < FS_MAX_MS,
              "landing must be confirmable before the backstop cut");
const LandProbeConfig FS_LAND_PROBE_CONFIG = {
    FS_LAND_LPF_ALPHA,
    FS_MIN_DESCEND_MS,
    FS_PROBE_PERIOD_MS,
    FS_PROBE_DIP_MS,
    FS_PROBE_SAMPLE_DELAY_MS,
    FS_PROBE_RESPONSE_G,
    FS_LAND_ACCEL_TOL_G,
    FS_PROBE_CONFIRM_N,
    FS_PROBE_DIP_MIN_US,
};
// 아래 호버 추정 상수는 모두 Stage E-0 벤치 조정 대상이다. 10°/±0.05g는
// 자세·수직 정상상태만 받는 보수적 시작값이고, 3초 LPF는 요구 범위(2~5초)의
// 중앙값이다. 1.5초 누적은 짧은 스로틀 통과를 호버로 확정하지 않으면서
// 실측 상한 모델인 0.06g 저주파 진동에서도 3초 안에 유효해지는 시작값이다.
const float    HOVER_LEVEL_MAX_DEG    = 10.0f;
const float    HOVER_ACCEL_TOL_G      = 0.05f;
const int      HOVER_MIN_THROTTLE_US  = 1150;
// hover_est 하한이 이 값보다 낮으면 하강값에서 딥을 뺀 스로틀이 1000us
// 아래가 되어 프로브가 UNAVAILABLE에 고정된다. 현재 컷은 프로브 가용성과
// 무관하게 FS_MAX_MS 타임아웃을 사용하지만 진단용 딥의 가용성은 유지한다.
static_assert(
    HOVER_MIN_THROTTLE_US
        >= 1000 + FS_DESCENT_DELTA_US + FS_PROBE_DIP_MIN_US,
    "hover estimate floor must keep the probe dip above idle");
const float    HOVER_LPF_TAU_S        = 3.0f;
const uint32_t HOVER_VALID_MS         = 1500;
// 실측 호버는 약 1340us다. 무장 직후 spool-up 로그(041032)는 1100~1190us
// 구간이 전체 무장 시간의 9.4%였으므로, 지상 즉시컷은 보수적으로 1150us
// 이하에서만 허용한다. 이보다 높으면 hover_est가 오염됐더라도 자동착륙을
// 택한다(유효 호버 추정치가 있어야 한다). 지상에서도 FS_MAX_MS 동안 모터가
// 돌 수 있으며, 이 절대 하한은 고도나 실제 접지를 판별하는 센서가 아니다.
const int      FS_GROUND_CUT_MAX_US  = 1150;
const uint32_t PID_WDT_TIMEOUT_MS = 500;     // pid_task 정지 시 강제 재부팅 (1kHz 루프 대비 큰 여유)
const int32_t  FROZEN_DELTA_RAW   = 1;       // 6축 raw 변화량 합이 이 이하면 정지 의심 (LSB)
const uint32_t IMU_FROZEN_MS      = 300;     // 그 상태가 이만큼 지속되면 freeze 확정
const float    GYRO_DISAGREE_DPS  = 15.0f;   // 두 IMU 각속도 차이 임계 (deg/s)
const uint32_t IMU_DISAGREE_MS    = 150;     // 불일치 지속 시간

const int   BIAS_CALIB_SAMPLES   = 2000;
const float BIAS_MOVEMENT_THRESH = 1.0f;     // 축별 stddev 상한 (deg/s)
const int   BIAS_CALIB_RETRIES   = 3;
const float ACCEL_CAL_MIN_G      = 0.8f;     // 이 범위 밖은 센서 이상: 보정 금지
const float ACCEL_CAL_MAX_G      = 1.2f;

const int OUTER_DIV = 4;                      // outer loop = 1kHz / 4 = 250Hz

// --- 필터/적분 상한 ---
const float GYRO_SW_LPF_HZ = 80.0f;  // 하드웨어 LPF(121Hz)에 겹치는 소프트웨어 LPF
const float I_TERM_MAX_US  = 50.0f;  // 축별 적분 기여 상한 (모터 µs)

// ==========================================================
// 3. 유틸
// ==========================================================
class LowPassFilter {
public:
  float alpha, last = 0.0f;
  LowPassFilter(float cutoff_hz, float dt) {
    float rc = 1.0f / (2.0f * PI * cutoff_hz);
    alpha = dt / (rc + dt);
  }
  float update(float in) { last += alpha * (in - last); return last; }
  void reset(float value = 0.0f) { last = value; }
};

// Arduino sketch preprocessor의 자동 함수 원형보다 먼저 보여야 하는 반환 타입.
struct MotorMix {
  int motor[4];
  bool scaled;
  float collective_us;
};

struct ImuTelemetrySample {
  float imu1GyroX, imu1GyroY, imu1GyroZ;
  float imu1AccelX, imu1AccelY, imu1AccelZ;
  float imu2GyroX, imu2GyroY, imu2GyroZ;
  float imu2AccelX, imu2AccelY, imu2AccelZ;
};

// IMU별 freeze 감시 (raw 레지스터 값이 멈췄는지)
struct FreezeMon {
  int16_t  lastGyro[3] = {0, 0, 0};
  int16_t  lastAccel[3] = {0, 0, 0};
  uint32_t since = 0;
  bool     init  = false;
};

static inline uint32_t imuRawRingCount(const ImuRawRing &ring) {
  const uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE);
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  return head - tail;
}

static inline bool imuRawRingPush(
    ImuRawRing &ring, const ImuRawSample &sample, uint32_t sampleUs) {
  const uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_RELAXED);
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  if (head - tail >= IMU_RAW_RING_SIZE) {
    __atomic_fetch_add(&ring.dropped, 1, __ATOMIC_RELAXED);
    return false;
  }
  if (head == tail) {
    __atomic_store_n(&ring.first_t_us, sampleUs, __ATOMIC_RELAXED);
  }
  ring.samples[head & IMU_RAW_RING_MASK] = sample;
  __atomic_store_n(&ring.head, head + 1, __ATOMIC_RELEASE);
  return true;
}

static inline bool imuRawRingPop(ImuRawRing &ring, ImuRawSample &sample) {
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_RELAXED);
  if (tail == __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE)) return false;
  sample = ring.samples[tail & IMU_RAW_RING_MASK];
  __atomic_store_n(&ring.tail, tail + 1, __ATOMIC_RELEASE);
  return true;
}

static inline ImuRawSample makeImuRawSample(
    uint16_t dtUs, const inv_imu_sensor_event_t &e1,
    const inv_imu_sensor_event_t &e2, uint8_t failsafePhase) {
  ImuRawSample sample = {};
  sample.dt_us = dtUs;
  sample.failsafe_phase = failsafePhase;
  for (int axis = 0; axis < 3; axis++) {
    sample.imu1_gyro[axis] = e1.gyro[axis];
    sample.imu1_accel[axis] = e1.accel[axis];
    sample.imu2_gyro[axis] = e2.gyro[axis];
    sample.imu2_accel[axis] = e2.accel[axis];
  }
  return sample;
}

static inline size_t buildImuRawDatagram(
    ImuRawDatagram &datagram, uint32_t batchSeq, uint32_t baseTUs,
    uint32_t dropped, const ImuRawSample *samples, uint8_t nSamples) {
  if (nSamples == 0 || nSamples > IMU_RAW_BATCH_MAX) return 0;
  memcpy(datagram.header.magic, "ZIMU", 4);
  datagram.header.version = IMU_RAW_VERSION;
  datagram.header.n_samples = nSamples;
  datagram.header.reserved = 0;
  datagram.header.batch_seq = batchSeq;
  datagram.header.base_t_us = baseTUs;
  datagram.header.dropped = dropped;
  memcpy(datagram.samples, samples, nSamples * sizeof(ImuRawSample));
  return sizeof(ImuRawHeader) + nSamples * sizeof(ImuRawSample);
}

static inline ImuTelemetrySample makeImuTelemetrySample(
    const float g1[3], const float a1[3],
    const float g2[3], const float a2[3]) {
  return {
    g1[1], -g1[0], -g1[2],
    a1[1] * ACCEL_SCALE, -a1[0] * ACCEL_SCALE, a1[2] * ACCEL_SCALE,
    g2[1], -g2[0], -g2[2],
    a2[1] * ACCEL_SCALE, -a2[0] * ACCEL_SCALE, a2[2] * ACCEL_SCALE
  };
}
#if WIFI_LATENCY_DEBUG
static inline void recordLatEvent(LatRing &ring, uint32_t tMs,
                                  uint32_t durUs, LatKind kind) {
  uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_RELAXED);
  uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  if (head - tail >= LAT_RING_SIZE) {
    __atomic_fetch_add(&ring.dropped, 1, __ATOMIC_RELAXED);
    return;
  }
  ring.events[head & LAT_RING_MASK] = {tMs, durUs, (uint8_t)kind};
  __atomic_store_n(&ring.head, head + 1, __ATOMIC_RELEASE);
}

static inline bool popLatEvent(LatRing &ring, LatEvt &event) {
  uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_RELAXED);
  if (tail == __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE)) return false;
  event = ring.events[tail & LAT_RING_MASK];
  __atomic_store_n(&ring.tail, tail + 1, __ATOMIC_RELEASE);
  return true;
}

static const char *latKindName(uint8_t kind) {
  switch (kind) {
    case LAT_UDP_LOOP:  return "UDP_LOOP";
    case LAT_UDP_PARSE: return "UDP_PARSE";
    case LAT_UDP_SEND:  return "UDP_SEND";
    case LAT_PID:       return "PID";
    default:            return "UNKNOWN";
  }
}

static void drainLatRing(LatRing &ring) {
  LatEvt event;
  while (popLatEvent(ring, event)) {
    if (event.kind == LAT_PID) {
      if (event.dur_us > latPidMaxUs) latPidMaxUs = event.dur_us;
      if (event.dur_us > 5000U) latPidOver5++;
    } else {
      if (event.dur_us > latUdpMaxUs) latUdpMaxUs = event.dur_us;
      if (event.kind == LAT_UDP_LOOP) {
        if (event.dur_us > 30000U) latUdpLoopOver30++;
        if (event.dur_us > 100000U) latUdpLoopOver100++;
        if (event.dur_us > 300000U) latUdpLoopOver300++;
      }
    }
    Serial.printf("[LAT] t=%lu kind=%s dur_us=%lu\n",
                  (unsigned long)event.t_ms, latKindName(event.kind),
                  (unsigned long)event.dur_us);
  }
}
#endif
static bool checkFreeze(FreezeMon &m, const inv_imu_sensor_event_t &e, uint32_t nowMs) {
  if (!m.init) {
    for (int k = 0; k < 3; k++) {
      m.lastGyro[k] = e.gyro[k];
      m.lastAccel[k] = e.accel[k];
    }
    m.init = true;
    return false;
  }

  // abs(x)+abs(y)+abs(z)의 변화만 보면 축 변화가 서로 상쇄될 수 있다.
  // 각 레지스터의 변화량을 직접 더해 통신 정지/동일 프레임 반복을 감시한다.
  int32_t delta = 0;
  for (int k = 0; k < 3; k++) {
    delta += abs((int32_t)e.gyro[k] - m.lastGyro[k]);
    delta += abs((int32_t)e.accel[k] - m.lastAccel[k]);
  }

  bool frozen = false;
  if (delta <= FROZEN_DELTA_RAW) {
    if (m.since == 0) m.since = nowMs;
    else if (nowMs - m.since >= IMU_FROZEN_MS) frozen = true;
  } else {
    m.since = 0;
  }
  for (int k = 0; k < 3; k++) {
    m.lastGyro[k] = e.gyro[k];
    m.lastAccel[k] = e.accel[k];
  }
  return frozen;
}

// RC 워치독. nowMs는 루프 맨 위에서 뜬 값이라 udp_task(코어0)가 그 뒤에
// lastRcMs를 갱신하면 lastRcMs가 nowMs보다 미래일 수 있다. 부호 있는 차이로
// 비교해야 unsigned 언더플로로 즉시 타임아웃이 터지지 않는다. millis() wrap도
// 모듈러 뺄셈이라 그대로 안전하다.
static inline bool rcTimedOut(uint32_t nowMs, uint32_t lastMs) {
  return (int32_t)(nowMs - lastMs) > (int32_t)RC_TIMEOUT_MS;
}

// ==========================================================
// 4. 시스템 변수
// ==========================================================
WiFiUDP   udp;
char      packetBuffer[256];
IPAddress laptopIP;
int       laptopPort            = 0;
volatile bool connectionEstablished = false;

// startGyro/startAccel은 ODR·FSR만 설정하고 UI 필터 대역폭은 칩 전원
// 기본값(bypass) 그대로 둔다. 그러면 프롭 진동이 필터 없이 PID에 들어오므로
// protected icm_driver에 접근해 하드웨어 LPF를 켠다 (gyro 121Hz, accel 25Hz).
class ICM42670WithLPF : public ICM42670 {
public:
  ICM42670WithLPF(SPIClass &spi_bus, uint8_t cs) : ICM42670(spi_bus, cs) {}
  int setLowPassFilters() {
    int rc = 0;
    rc |= inv_imu_set_gyro_ln_bw(&icm_driver, GYRO_CONFIG1_GYRO_FILT_BW_121);
    rc |= inv_imu_set_accel_ln_bw(&icm_driver, ACCEL_CONFIG1_ACCEL_FILT_BW_25);
    return rc;
  }
};

ICM42670WithLPF IMU1(SPI, SPI_CS1);
ICM42670WithLPF IMU2(SPI, SPI_CS2);
DFRobot_BMM350_I2C bmm(&Wire, 0x14);

volatile bool  safety_lock  = true;
// 런타임 safety_lock 전이는 Core 1(pid_task)만 수행한다. Core 0은 요청만
// 게시하며, 두 요청이 함께 대기하면 disarm이 arm보다 우선한다.
volatile bool  safety_disarm_requested = false;
volatile bool  safety_arm_requested = false;
volatile bool  failsafe_resume_requested = false;
volatile bool  hover_reset_requested = false;
portMUX_TYPE safetyRequestMux = portMUX_INITIALIZER_UNLOCKED;
// 런타임 쓰기는 pid_task만 담당한다. Core 0은 텔레메트리/명령 처리에서 읽기만 한다.
volatile uint8_t fs_phase = FS_NONE;   // 텔레메트리 Failsafe_Phase
volatile uint8_t fs_probe_state = FS_PROBE_WAIT;
volatile uint8_t fs_probe_no_response = 0;
volatile float fs_probe_response_g = 0.0f;
// 한 비행에서 최초 자동착륙에 들어간 시각. resume은 지우지 않으며 새 start가
// 실제로 Core 1에서 arm 전이될 때만 초기화한다.
volatile uint32_t fs_first_enter_ms = 0;
volatile bool fs_first_enter_valid = false;
// 3901-L0X 상태. udp_task 만 쓰고 sendTelemetry 가 읽는다(같은 코어).
MspSensorParser mspSensor;
volatile int32_t msp_range_mm = 0;
volatile int     msp_range_quality = -1;   // -1 = 신선한 프레임 없음
volatile int32_t msp_flow_x = 0, msp_flow_y = 0;
volatile int     msp_flow_quality = -1;
uint32_t msp_last_range_ms = 0;
uint32_t msp_last_flow_ms = 0;
uint16_t msp_reported_unknown = 0;         // 같은 ID 를 반복해 찍지 않기 위해

volatile float hover_est = 0.0f;       // 추정 호버 collective (us)
// hover_valid는 만료시키지 않는다. false는 RC 끊김 시 즉시 컷을 뜻하므로
// 공중에서 freshness 만료로 false가 되면 추락한다. 낡은 추정치가 덜 위험하다.
volatile bool hover_valid = false;
// hoverTracker와 그 공개 mirror는 pid_task만 쓴다. Core 0의 start는
// safetyRequestMux를 통해 reset을 요청하고 직접 구조체를 초기화하지 않는다.
HoverThrottleEstimator hoverTracker = {};
volatile float targetAngleX = 0.0f, targetAngleY = 0.0f, targetAngleZ = 0.0f;
// 기체 트림(도). 추정기 0°와 진짜 수평의 차이를 보정한다. 비행별 상태가 아니라
// 기체 속성이므로 start/stop이 지우지 않는다.
volatile float trim_roll = 0.0f;
volatile float trim_pitch = 0.0f;
volatile float targetYawRate = 0.0f;   // rcr이 준 yaw 각속도 명령 (dps)

volatile float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f; // 추정 각도
volatile float gyroX  = 0.0f, gyroY  = 0.0f, gyroZ  = 0.0f; // 융합 각속도 (body frame)
volatile float accX   = 0.0f, accY   = 0.0f, accZ   = 0.0f; // 융합 가속도 (g)
ImuTelemetrySample imuSampleSnapshot = {};
portMUX_TYPE imuSampleMux = portMUX_INITIALIZER_UNLOCKED;
volatile int   motorOut[4] = {1000,1000,1000,1000};
volatile float tgtRate[3]  = {0,0,0}; // roll,pitch,yaw dps
volatile int   pidLoopHz   = 0;

// Core 0 writer -> Core 1 reader. 짝수 seq만 완성된 벡터다.
volatile bool  mag_enabled = false;
volatile bool  mag_ready = false;
volatile bool  mag_calibrating = false;
volatile float magFieldX = 0.0f, magFieldY = 0.0f, magFieldZ = 0.0f;
volatile float magHeading = 0.0f;
volatile float magTelemX = 0.0f, magTelemY = 0.0f, magTelemZ = 0.0f;
volatile bool magTelemCalActive = false;
volatile uint32_t magSampleMs = 0;
volatile bool magSampleValid = false;
volatile bool mag_reference_pending = true;

portMUX_TYPE magSnapshotMux = portMUX_INITIALIZER_UNLOCKED;
float magYawReferenceOffset = 0.0f;

float magCalMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
float magCalMax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
uint32_t magCalSampleCount = 0;
uint32_t lastMagReadMs = 0;

float iTermRoll = 0.0f, iTermPitch = 0.0f, iTermYaw = 0.0f;  // 적분 기여 (모터 µs)

float gyro_bias1[3] = {0,0,0};
float gyro_bias2[3] = {0,0,0};
// 두 센서의 gain 오차는 독립이고 단일-IMU 폴백도 있으므로 각각 보정한다.
float accel_scale1 = 1.0f;
float accel_scale2 = 1.0f;

volatile uint32_t lastRcMs        = 0;
volatile bool     fault_rc        = false;
volatile bool     fault_imu1      = false;   // IMU1 freeze
volatile bool     fault_imu2      = false;   // IMU2 freeze
volatile bool     fault_disagree  = false;   // 두 IMU 불일치 (중재 불가)
volatile bool     fault_attitude  = false;   // 과도 기울기
volatile int      active_imus     = 2;       // 현재 사용 중인 IMU 수 (telemetry)
volatile bool     mixer_scaled    = false;   // 자세 mixer가 축소됐는지
volatile bool     calibration_ok  = false;

// 재시동 판단용 현재 센서 상태. fault_imu* / fault_disagree는 비행 중 latch된다.
volatile bool     imu1_frozen_now = false;
volatile bool     imu2_frozen_now = false;
volatile bool     imu_disagree_now = false;

// scripts/control_dualsense.py 프로토콜 호환용
volatile uint32_t lastRcSeq       = 0;
volatile bool     rcSeqValid      = false;
volatile uint32_t rcTotalPkts     = 0;
volatile uint32_t rcDroppedPkts   = 0;

static inline void requestSafetyDisarm() {
  portENTER_CRITICAL(&safetyRequestMux);
  safety_disarm_requested = true;
  safety_arm_requested = false;
  failsafe_resume_requested = false;
  hover_reset_requested = false;
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline void requestSafetyArm() {
  portENTER_CRITICAL(&safetyRequestMux);
  if (!safety_disarm_requested) {
    hover_reset_requested = true;
    safety_arm_requested = true;
  }
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline bool configChangeAllowed(const char *setting) {
  if (fs_phase != FS_DESCENDING) return true;
  Serial.printf(">>> %s refused (auto-land descending)\n", setting);
  return false;
}

static inline void applyPendingSafetyRequest() {
  portENTER_CRITICAL(&safetyRequestMux);
  if (safety_disarm_requested) {
    safety_lock = true;
    safety_disarm_requested = false;
    safety_arm_requested = false;
    hover_reset_requested = false;
  } else if (safety_arm_requested) {
    if (hover_reset_requested) {
      // Core 1이 reset을 끝낸 뒤에만 safety_lock=false를 공개한다.
      hover_valid = false;
      hover_est = 0.0f;
      hoverTracker = {};
    }
    hover_reset_requested = false;
    safety_lock = false;
    safety_arm_requested = false;
  }
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline void requestFailsafeResume() {
  portENTER_CRITICAL(&safetyRequestMux);
  failsafe_resume_requested = true;
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline bool takeFailsafeResumeRequest() {
  portENTER_CRITICAL(&safetyRequestMux);
  const bool requested = failsafe_resume_requested;
  failsafe_resume_requested = false;
  portEXIT_CRITICAL(&safetyRequestMux);
  return requested;
}

static inline ResumeRefusalReason resumeRefusalReason(uint32_t nowMs) {
  if (fs_phase != FS_DESCENDING) return RESUME_REFUSED_PHASE;
  if (fs_first_enter_valid &&
      (uint32_t)(nowMs - fs_first_enter_ms) >= FS_RESUME_MAX_MS) {
    return RESUME_REFUSED_CUMULATIVE;
  }
  if (rcTimedOut(nowMs, lastRcMs)) return RESUME_REFUSED_RC;
  if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
    return RESUME_REFUSED_TILT;
  }
  if (active_imus <= 0
      || (imu1_frozen_now && imu2_frozen_now)
      || imu_disagree_now) {
    return RESUME_REFUSED_IMU;
  }
  if (!hover_valid) return RESUME_REFUSED_HOVER;
  return RESUME_ALLOWED;
}

static inline void logResumeRefusal(ResumeRefusalReason reason) {
  Serial.printf(">>> RESUME REFUSED %s\n", resumeRefusalName(reason));
}

static inline void publishMagSample(
    float x, float y, float z, uint32_t sample_ms) {
  // I2C read happens before this lock. The critical section only publishes
  // four words, so pid_task never waits on the blocking transaction.
  portENTER_CRITICAL(&magSnapshotMux);
  magFieldX = x;
  magFieldY = y;
  magFieldZ = z;
  magSampleMs = sample_ms;
  magSampleValid = true;
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline bool readMagSnapshot(MagSnapshot &snapshot) {
  portENTER_CRITICAL(&magSnapshotMux);
  bool valid = magSampleValid;
  snapshot.x = magFieldX;
  snapshot.y = magFieldY;
  snapshot.z = magFieldZ;
  snapshot.sample_ms = magSampleMs;
  portEXIT_CRITICAL(&magSnapshotMux);
  return valid;
}

static inline void publishMagTelemetry(
    float x, float y, float z, bool calibration_raw) {
  portENTER_CRITICAL(&magSnapshotMux);
  // Core 1 may already have entered its normal-domain block when Core 0
  // starts calibration. Do not let that late writer overwrite a raw snapshot.
  if (calibration_raw || !mag_calibrating) {
    magTelemX = x;
    magTelemY = y;
    magTelemZ = z;
    magTelemCalActive = calibration_raw;
  }
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline void readMagTelemetry(
    float &x, float &y, float &z, bool &calibration_raw) {
  portENTER_CRITICAL(&magSnapshotMux);
  x = magTelemX;
  y = magTelemY;
  z = magTelemZ;
  calibration_raw = magTelemCalActive;
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline void requestMagReferenceUpdate() {
  portENTER_CRITICAL(&magSnapshotMux);
  mag_reference_pending = true;
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline bool takeMagReferenceRequest() {
  portENTER_CRITICAL(&magSnapshotMux);
  const bool requested = mag_reference_pending;
  mag_reference_pending = false;
  portEXIT_CRITICAL(&magSnapshotMux);
  return requested;
}

static inline void publishImuSample(const ImuTelemetrySample &sample) {
  portENTER_CRITICAL(&imuSampleMux);
  imuSampleSnapshot = sample;
  portEXIT_CRITICAL(&imuSampleMux);
}

static inline ImuTelemetrySample readImuSampleSnapshot() {
  portENTER_CRITICAL(&imuSampleMux);
  const ImuTelemetrySample sample = imuSampleSnapshot;
  portEXIT_CRITICAL(&imuSampleMux);
  return sample;
}

static inline void publishImuRawRegisters(
    const inv_imu_sensor_event_t &e1, const inv_imu_sensor_event_t &e2,
    uint32_t sampleUs, uint8_t failsafePhase) {
  uint16_t dtUs = 0;
  if (rawProducerTimeValid) {
    const uint32_t elapsedUs = sampleUs - rawProducerLastUs;
    dtUs = elapsedUs > 65535UL
        ? 0xFFFFU
        : static_cast<uint16_t>(elapsedUs);
  }
  const ImuRawSample sample = makeImuRawSample(dtUs, e1, e2, failsafePhase);
  if (imuRawRingPush(imuRawRing, sample, sampleUs)) {
    rawProducerLastUs = sampleUs;
    rawProducerTimeValid = true;
  }
}

static bool initMagnetometer() {
  if (mag_ready) return true;

  pinMode(LDO_SHD, OUTPUT);
  digitalWrite(LDO_SHD, HIGH);
  delay(100);
  Wire.begin(BMM_SDA, BMM_SCL);
  uint8_t status = bmm.begin();
  if (status != 0) {
    Serial.printf("[MAG] BMM350 init failed (%u)\n", (unsigned)status);
    return false;
  }
  bmm.setOperationMode(eBmm350NormalMode);
  bmm.setRate(BMM350_DATA_RATE_50HZ);
  mag_ready = true;
  lastMagReadMs = millis() - MAG_SAMPLE_PERIOD_MS;
  Serial.println("[MAG] BMM350 ready @50Hz");
  return true;
}

static void startMagCalibration() {
  portENTER_CRITICAL(&magSnapshotMux);
  const bool alreadyActive = mag_calibrating;
  portEXIT_CRITICAL(&magSnapshotMux);
  if (alreadyActive) {
    Serial.println("[MAGCAL] already active");
    return;
  }
  mag_enabled = false;
  requestMagReferenceUpdate();
  portENTER_CRITICAL(&magSnapshotMux);
  mag_calibrating = true;
  portEXIT_CRITICAL(&magSnapshotMux);
  magCalMin[0] = FLT_MAX;
  magCalMin[1] = FLT_MAX;
  magCalMin[2] = FLT_MAX;
  magCalMax[0] = -FLT_MAX;
  magCalMax[1] = -FLT_MAX;
  magCalMax[2] = -FLT_MAX;
  magCalSampleCount = 0;
  Serial.println("[MAGCAL] logging raw XYZ; rotate drone, then send 'magcal 0'");
}

static void stopMagCalibration() {
  MagSnapshot calibrated = {};
  const bool have_sample =
      magCalSampleCount > 0U && readMagSnapshot(calibrated);
  if (have_sample) {
    const float magThrDelta =
        (float)(base_throttle - MAG_THROTTLE_REF_US);
    calibrated.x -= mag_comp_x * magThrDelta;
    calibrated.y -= mag_comp_y * magThrDelta;
    calibrated.z -= mag_comp_z * magThrDelta;
  }

  portENTER_CRITICAL(&magSnapshotMux);
  mag_calibrating = false;
  if (have_sample) {
    magTelemX = calibrated.x;
    magTelemY = calibrated.y;
    magTelemZ = calibrated.z;
  }
  magTelemCalActive = false;
  portEXIT_CRITICAL(&magSnapshotMux);
  if (magCalSampleCount == 0U) {
    Serial.println("[MAGCAL] no samples");
    return;
  }
  Serial.printf("[MAGCAL] samples=%lu\n", (unsigned long)magCalSampleCount);
  Serial.printf("[MAGCAL] span X=%.6f uT Y=%.6f uT Z=%.6f uT\n",
                magCalMax[0] - magCalMin[0],
                magCalMax[1] - magCalMin[1],
                magCalMax[2] - magCalMin[2]);
  Serial.println(
      "[MAGCAL] save the telemetry CSV and run scripts/magcal_fit.py <flight_log.csv>");
}

static inline void applyMagCalibration(
    const float raw[3], const float hard_iron[3],
    const float soft_iron[3][3], float out[3]) {
  const float centered[3] = {
    raw[0] - hard_iron[0],
    raw[1] - hard_iron[1],
    raw[2] - hard_iron[2],
  };
  const float body_sign[3] = {
    MAG_BODY_SIGN_X, MAG_BODY_SIGN_Y, MAG_BODY_SIGN_Z,
  };
  for (int axis = 0; axis < 3; axis++) {
    float transformed = 0.0f;
    for (int source = 0; source < 3; source++) {
      transformed += soft_iron[axis][source] * centered[source];
    }
    out[axis] = body_sign[axis] * transformed;
  }
}

static void sampleMagnetometer(uint32_t now_ms) {
  if (!mag_ready || now_ms - lastMagReadMs < MAG_SAMPLE_PERIOD_MS) return;
  lastMagReadMs = now_ms;

  // Blocking I2C read: caller is udp_task on core 0, never pid_task.
  sBmm350MagData_t data = bmm.getGeomagneticData();
  const float raw[3] = {data.float_x, data.float_y, data.float_z};
  if (!isfinite(raw[0]) || !isfinite(raw[1]) || !isfinite(raw[2])) return;

  if (mag_calibrating) {
    publishMagTelemetry(raw[0], raw[1], raw[2], true);
    for (int axis = 0; axis < 3; axis++) {
      if (raw[axis] < magCalMin[axis]) magCalMin[axis] = raw[axis];
      if (raw[axis] > magCalMax[axis]) magCalMax[axis] = raw[axis];
    }
    magCalSampleCount++;
  }

  float calibrated[3];
  applyMagCalibration(raw, MAG_HARD_IRON, MAG_SOFT_IRON, calibrated);
  publishMagSample(calibrated[0], calibrated[1], calibrated[2], now_ms);
}

// ==========================================================
// 5. 모터
// ==========================================================
void writeMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);
  uint32_t duty = ((uint32_t)us * ESC_MAXDUTY) / ESC_PERIOD;
  ledcWrite(pin, duty);
}
void stopMotors() {
  writeMotor(pinM1, 1000); writeMotor(pinM2, 1000);
  writeMotor(pinM3, 1000); writeMotor(pinM4, 1000);
}

// 자세 차동 명령을 먼저 보존하고 collective를 이동한다. 그래도 범위를 넘을 때만
// 모든 자세 명령을 같은 비율로 축소해 토크 비율을 유지한다.
static MotorMix mixAndDesaturate(float roll, float pitch, float yaw,
                                 int throttle, int minMotor, int maxMotor) {
  MotorMix out;
  minMotor = constrain(minMotor, 1000, 2000);
  maxMotor = constrain(maxMotor, minMotor, 2000);

  float diff[4] = {
    -pitch + roll - yaw,  // M1: FL
     pitch - roll - yaw,  // M2: RR
    -pitch - roll + yaw,  // M3: FR
     pitch + roll + yaw   // M4: RL
  };

  float minDiff = diff[0], maxDiff = diff[0];
  for (int i = 1; i < 4; i++) {
    minDiff = min(minDiff, diff[i]);
    maxDiff = max(maxDiff, diff[i]);
  }

  const float available = (float)(maxMotor - minMotor);
  const float span = maxDiff - minDiff;
  float scale = 1.0f;
  if (span > available && span > 0.0f) scale = available / span;
  out.scaled = scale < 0.9999f;

  if (out.scaled) {
    for (int i = 0; i < 4; i++) diff[i] *= scale;
    minDiff *= scale;
    maxDiff *= scale;
  }

  const float collectiveLo = minMotor - minDiff;
  const float collectiveHi = maxMotor - maxDiff;
  const float collective = min(max((float)throttle, collectiveLo), collectiveHi);
  out.collective_us = collective;

  for (int i = 0; i < 4; i++) {
    out.motor[i] = constrain((int)lroundf(collective + diff[i]), minMotor, maxMotor);
  }
  return out;
}

// ==========================================================
// 6. Gyro bias 캘리브레이션
// ==========================================================
// sign[]은 각 IMU를 drone(IMU1) frame으로 맞추는 부호.
static bool measure_imu_bias(ICM42670 &imu, const float sign[3],
                             float bias_out[3], float sd_out[3],
                             float &accel_mean_out) {
  double sum[3] = {0,0,0}, sum_sq[3] = {0,0,0};
  double accel_mag_sum = 0.0;
  inv_imu_sensor_event_t e = {};
  int samples = 0;
  int attempts = 0;
  const int maxAttempts = BIAS_CALIB_SAMPLES + 100;
  while (samples < BIAS_CALIB_SAMPLES && attempts < maxAttempts) {
    attempts++;
    if (imu.getDataFromRegisters(e) != 0) {
      delayMicroseconds(1000);
      continue;
    }
    float g[3] = {
      sign[0] * e.gyro[0] * GYRO_SCALE,
      sign[1] * e.gyro[1] * GYRO_SCALE,
      sign[2] * e.gyro[2] * GYRO_SCALE
    };
    for (int k = 0; k < 3; k++) { sum[k] += g[k]; sum_sq[k] += (double)g[k]*g[k]; }
    const double ax = (double)e.accel[0] * ACCEL_SCALE;
    const double ay = (double)e.accel[1] * ACCEL_SCALE;
    const double az = (double)e.accel[2] * ACCEL_SCALE;
    accel_mag_sum += sqrt(ax*ax + ay*ay + az*az);
    samples++;
    delayMicroseconds(1000);
  }
  if (samples != BIAS_CALIB_SAMPLES) return false;
  accel_mean_out = (float)(accel_mag_sum / samples);
  for (int k = 0; k < 3; k++) {
    double mean = sum[k] / samples;
    double var  = sum_sq[k] / samples - mean*mean;
    if (var < 0) var = 0;
    bias_out[k] = (float)mean;
    sd_out[k]   = (float)sqrt(var);
  }
  return true;
}

static float guarded_accel_scale(float accel_mean_g, const char *imu_name) {
  if (accel_mean_g < ACCEL_CAL_MIN_G || accel_mean_g > ACCEL_CAL_MAX_G) {
    Serial.printf(
        "[CALIB] WARN %s accel |a| %.4fg outside [0.8,1.2], scale=1.0\n",
        imu_name, accel_mean_g);
    return 1.0f;
  }
  return 1.0f / accel_mean_g;
}

static bool calibrate_bias() {
  float sd1[3], sd2[3];
  accel_scale1 = 1.0f;
  accel_scale2 = 1.0f;
  for (int a = 1; a <= BIAS_CALIB_RETRIES; a++) {
    Serial.printf("[CALIB] attempt %d/%d (hold still)...\n", a, BIAS_CALIB_RETRIES);
    float accel_mean1 = 0.0f, accel_mean2 = 0.0f;
    bool read1 = measure_imu_bias(
        IMU1, IMU1_SIGN, gyro_bias1, sd1, accel_mean1);
    bool read2 = measure_imu_bias(
        IMU2, IMU2_SIGN, gyro_bias2, sd2, accel_mean2);
    if (!read1 || !read2) {
      Serial.printf("[CALIB] sensor read failed (imu1=%d imu2=%d)\n", (int)read1, (int)read2);
      continue;
    }
    float max_sd = max(max(max(sd1[0],sd1[1]),sd1[2]),
                       max(max(sd2[0],sd2[1]),sd2[2]));
    Serial.printf("[CALIB] IMU1 %.3f %.3f %.3f | IMU2 %.3f %.3f %.3f (maxSD %.3f)\n",
                  gyro_bias1[0],gyro_bias1[1],gyro_bias1[2],
                  gyro_bias2[0],gyro_bias2[1],gyro_bias2[2], max_sd);
    if (max_sd <= BIAS_MOVEMENT_THRESH) {
      accel_scale1 = guarded_accel_scale(accel_mean1, "IMU1");
      accel_scale2 = guarded_accel_scale(accel_mean2, "IMU2");
      Serial.printf(
          "[CALIB] accel mean IMU1 %.4fg scale %.6f | "
          "IMU2 %.4fg scale %.6f\n",
          accel_mean1, accel_scale1, accel_mean2, accel_scale2);
      Serial.println("[CALIB] OK");
      return true;
    }
    Serial.println("[CALIB] movement detected, retry");
  }
  Serial.println("[CALIB] FAIL: reboot and calibrate on a stationary surface");
  return false;
}

// ==========================================================
// 7. 자세 추정용 적응 alpha (가속도 신뢰도 기반)
// ==========================================================
const float ACC_DEV_SOFT = 0.10f, ACC_DEV_HARD = 0.30f;
// 1kHz 루프 기준 시정수 tau ≈ dt/(1-alpha): 0.999→1s, 0.9995→2s, 0.9998→5s.
// 이전 값(0.99, tau 0.1s)은 250Hz legacy 루프에서 온 것으로, 1kHz에서는
// 비행 중 가속도(추력 방향+진동)에 자세 추정이 끌려가 호버가 불안정해진다.
const float ALPHA_STATIC = 0.999f, ALPHA_NORMAL = 0.9995f, ALPHA_DYN = 0.9998f;
static inline float compute_alpha(float ax, float ay, float az) {
  float dev = fabsf(sqrtf(ax*ax+ay*ay+az*az) - 1.0f);
  if (dev < ACC_DEV_SOFT) return ALPHA_STATIC;
  if (dev < ACC_DEV_HARD) return ALPHA_NORMAL;
  return ALPHA_DYN;
}

// 잠금 진입 시 1회만 실행되는 정리. 세 잠금 경로가 모두 이 함수를 통해서만
// wasLocked를 세운다. 매 tick 지우면 시동 해제 상태에서 `yaw 1`을 켤 수 없다.
static inline void enterLockedState(bool &wasLocked) {
  if (!wasLocked) yaw_hold_override = false;
  wasLocked = true;
}

// ==========================================================
// 8. PID 태스크 (Core 1, 1kHz)
// ==========================================================
void pid_task(void *pv) {
  // 모터 정지 수단(stopMotors, RC timeout 검사)이 모두 이 태스크 안에 있으므로,
  // SPI 행업 등으로 태스크가 블로킹되면 모터가 마지막 PWM으로 고정된다.
  // 태스크 워치독이 panic 재부팅을 강제하고, 재부팅 후 ESC는 PWM 신호
  // 소실로 정지하며 setup()의 stopMotors()가 정지 신호를 복원한다.
  esp_task_wdt_add(NULL);

  const TickType_t period = pdMS_TO_TICKS(1);   // vTaskDelayUntil 기반 (busy-wait 제거)
  TickType_t wake = xTaskGetTickCount();
  const float dt = 0.001f;

  LowPassFilter lpfD_Roll(40, dt), lpfD_Pitch(40, dt), lpfD_Yaw(40, dt);
  LowPassFilter lpfG1[3] = {{GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}};
  LowPassFilter lpfG2[3] = {{GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}};
  float prevGyroX = 0.0f, prevGyroY = 0.0f, prevGyroZ = 0.0f;

  FreezeMon fm1, fm2;
  uint32_t disagreeSince = 0;

  float targetRateRoll = 0, targetRatePitch = 0, targetRateYaw = 0;
  uint8_t outerCnt = 0;
  uint8_t magFusionCnt = 0;
  uint32_t lastMicros = micros();
  uint32_t loopCount = 0;
  uint32_t loopMarkerMs = millis();
  bool wasLocked = true;
  LandDetector landDet = {};
  LandProbeConfig landProbeConfig = FS_LAND_PROBE_CONFIG;
  int fs_probe_dip_us = FS_PROBE_DIP_MIN_US;
  uint32_t fs_enter_ms = 0;
  float fs_hold_yaw = 0.0f;
  bool fs_probe_blocked_logged = false;

  inv_imu_sensor_event_t e1 = {}, e2 = {};

  while (true) {
    vTaskDelayUntil(&wake, period);
    esp_task_wdt_reset();
    TickType_t afterWake = xTaskGetTickCount();
    if ((TickType_t)(afterWake - wake) > period) {
      // 큰 지연 뒤 밀린 tick을 연속 실행하지 않는다(센서 stale read/D항 spike 방지).
      wake = afterWake;
    }
    uint32_t nowMs = millis();
    bool checkProbeDelivery = false;
    int probeReferenceThrottle = 0;
    uint32_t probeElapsedMs = 0;
    applyPendingSafetyRequest();
    loopCount++;
    uint32_t loopElapsedMs = nowMs - loopMarkerMs;
    if (loopElapsedMs >= 1000) {
      pidLoopHz = (int)((loopCount * 1000UL) / loopElapsedMs);
      loopCount = 0;
      loopMarkerMs = nowMs;
    }
    uint32_t nowUs = micros();
    float realDt = (nowUs - lastMicros) / 1e6f;
#if WIFI_LATENCY_DEBUG
    uint32_t rawDtUs = nowUs - lastMicros;
    if (rawDtUs > WIFI_LATENCY_PID_THRESHOLD_US) {
      recordLatEvent(pidLatRing, nowMs, rawDtUs, LAT_PID);
    }
#endif
    lastMicros = nowUs;
    // 지연 후 vTaskDelayUntil이 catch-up할 때 지나치게 작은 dt로 D항이 튀는 것을 방지.
    if (realDt < 0.0002f || realDt > 0.01f) realDt = dt;

    // ---------- 센서 읽기 ----------
    int readStatus1 = IMU1.getDataFromRegisters(e1);
    int readStatus2 = IMU2.getDataFromRegisters(e2);

    // 읽기 오류 시 event가 갱신되지 않으므로 동일 프레임으로 취급되어 freeze로 귀결된다.
    // fault는 비행 중 자동 복구하지 않고 다음 수동 start 때만 재평가/해제한다.
    bool frozen1 = checkFreeze(fm1, e1, nowMs);
    bool frozen2 = checkFreeze(fm2, e2, nowMs);
    imu1_frozen_now = frozen1;
    imu2_frozen_now = frozen2;
    if (frozen1 && !fault_imu1) Serial.printf("[FAULT] IMU1 FROZEN (read=%d)\n", readStatus1);
    if (frozen2 && !fault_imu2) Serial.printf("[FAULT] IMU2 FROZEN (read=%d)\n", readStatus2);
    if (frozen1) fault_imu1 = true;
    if (frozen2) fault_imu2 = true;

    // OFF hot path는 이 bool 검사 하나뿐이다. ON일 때만 micros()와 27B 복사를 한다.
    // freeze 판정 직후, 부호/scale/bias/LPF/body 변환 전의 레지스터 원값을 게시한다.
    if (__atomic_load_n(&raw_stream_enabled, __ATOMIC_ACQUIRE)
        && connectionEstablished) {
      // fs_phase 는 이 함수보다 뒤에서 갱신되므로 직전 tick 값이다(1ms).
      publishImuRawRegisters(e1, e2, micros(), (uint8_t)fs_phase);
    }

    // IMU1 sensor frame 기준 각속도 (IMU2 축 정렬 + 개별 bias 보정)
    float g1[3] = {
      e1.gyro[0]*GYRO_SCALE - gyro_bias1[0],
      e1.gyro[1]*GYRO_SCALE - gyro_bias1[1],
      e1.gyro[2]*GYRO_SCALE - gyro_bias1[2] };
    float g2[3] = {
      IMU2_SIGN_X*e2.gyro[0]*GYRO_SCALE - gyro_bias2[0],
      IMU2_SIGN_Y*e2.gyro[1]*GYRO_SCALE - gyro_bias2[1],
      IMU2_SIGN_Z*e2.gyro[2]*GYRO_SCALE - gyro_bias2[2] };
    // 소프트웨어 LPF. 제어와 IMU 불일치 판정 모두 필터된 각속도를 쓴다.
    // (진동으로 순간 차이가 15dps를 넘어 비행 중 disagree 컷이 나는 것을 방지)
    for (int k = 0; k < 3; k++) {
      g1[k] = lpfG1[k].update(g1[k]);
      g2[k] = lpfG2[k].update(g2[k]);
    }
    float a1[3] = {
      (float)e1.accel[0] * accel_scale1,
      (float)e1.accel[1] * accel_scale1,
      (float)e1.accel[2] * accel_scale1 };
    float a2[3] = {
      IMU2_SIGN_X*e2.accel[0] * accel_scale2,
      IMU2_SIGN_Y*e2.accel[1] * accel_scale2,
      IMU2_SIGN_Z*e2.accel[2] * accel_scale2 };

    // LPF 후 gyro와 LPF 없는 accel을 같은 1kHz 프레임에서 body 좌표로 게시한다.
    // 변환 계산은 락 밖에서 끝내고 임계구역에서는 float 12개만 복사한다.
    const ImuTelemetrySample currentImuSample =
        makeImuTelemetrySample(g1, a1, g2, a2);
    publishImuSample(currentImuSample);

    // 현재 샘플 불일치는 재시동 거부 판단에도 사용한다.
    bool disagreeSample = false;
    if (!frozen1 && !frozen2) {
      for (int k = 0; k < 3; k++) {
        if (fabsf(g1[k] - g2[k]) > GYRO_DISAGREE_DPS) disagreeSample = true;
      }
    }
    imu_disagree_now = disagreeSample;

    // latch된 센서는 비행 중 값이 다시 움직여도 자동 재투입하지 않는다.
    bool h1 = !fault_imu1, h2 = !fault_imu2;

    // ---------- IMU 융합 (진짜 redundancy) ----------
    // 둘 다 정상: 불일치 검사 후 평균 / 하나만 정상: 그것만 / 중재 불가: fault
    if (h1 && h2) {
      if (disagreeSample) {
        if (disagreeSince == 0) disagreeSince = nowMs;
        else if (nowMs - disagreeSince >= IMU_DISAGREE_MS) {
          if (!fault_disagree) Serial.println("[FAULT] DUAL IMU DISAGREEMENT");
          fault_disagree = true;
        }
      } else {
        disagreeSince = 0;
      }
    } else {
      disagreeSince = 0;
    }

    float fg[3], fa[3];
    if (h1 && h2 && !fault_disagree) {
      for (int k = 0; k < 3; k++) { fg[k] = (g1[k]+g2[k])*0.5f; fa[k] = (a1[k]+a2[k])*0.5f; }
      active_imus = 2;
    } else if (h1 && !h2) {
      for (int k = 0; k < 3; k++) { fg[k] = g1[k]; fa[k] = a1[k]; }
      active_imus = 1;
    } else if (h2 && !h1) {
      for (int k = 0; k < 3; k++) { fg[k] = g2[k]; fa[k] = a2[k]; }
      active_imus = 1;
    } else {
      // 둘 다 freeze 또는 서로 안 맞음 -> 신뢰 불가
      active_imus = 0;
      if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;
      safety_lock = true;
      mixer_scaled = false;
      iTermRoll = iTermPitch = iTermYaw = 0.0f;
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      enterLockedState(wasLocked);
      motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
      tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
      stopMotors();
      continue;
    }

    // Roll/pitch gyro now matches the validated accel transform; the inverted
    // signs caused estimator lag and rate-loop anti-damping.
    // bodyGz stays unchanged pending separate bench yaw re-verification.
    const float bodyGx =  fg[1];
    const float bodyGy = -fg[0];
    const float bodyGz = -fg[2];
    const float bodyAx =  fa[1] * ACCEL_SCALE;
    const float bodyAy = -fa[0] * ACCEL_SCALE;
    const float bodyAz =  fa[2] * ACCEL_SCALE;
    gyroX = bodyGx; gyroY = bodyGy; gyroZ = bodyGz;
    accX = bodyAx; accY = bodyAy; accZ = bodyAz;

    // ---------- 자세 추정 (상보필터) ----------
    angleX += bodyGx * realDt;
    angleY += bodyGy * realDt;
    if (fabsf(bodyGz) > YAW_DEADZONE) angleZ += bodyGz * realDt;

    float alpha = compute_alpha(bodyAx, bodyAy, bodyAz);
    float accAngleX = atan2f(bodyAy, sqrtf(bodyAx*bodyAx + bodyAz*bodyAz)) * 180.0f / PI;
    float accAngleY = atan2f(-bodyAx, sqrtf(bodyAy*bodyAy + bodyAz*bodyAz)) * 180.0f / PI;
    angleX = alpha * angleX + (1.0f - alpha) * accAngleX;
    angleY = alpha * angleY + (1.0f - alpha) * accAngleY;

    // outerCnt는 safety-lock 경로에서 매 tick reset되므로 독립 divider를 쓴다.
    // 이로써 armed/disarmed 모두 정확히 250Hz, K_MAG tau≈4s를 유지한다.
    if (magFusionCnt == 0 && mag_enabled) {
      MagSnapshot mag;
      if (readMagSnapshot(mag)
          // 코어0 갱신이 코어1의 nowMs보다 미래일 수 있어 부호 있는 차이로 비교한다.
          && (int32_t)(nowMs - mag.sample_ms) <= (int32_t)MAG_STALE_MS
          && mag.x*mag.x + mag.y*mag.y + mag.z*mag.z > 1e-6f) {
        const float magThrDelta = (float)(base_throttle - MAG_THROTTLE_REF_US);
        mag.x -= mag_comp_x * magThrDelta;
        mag.y -= mag_comp_y * magThrDelta;
        mag.z -= mag_comp_z * magThrDelta;
        publishMagTelemetry(mag.x, mag.y, mag.z, false);
        float heading = tiltCompensatedMagHeadingDeg(
            mag.x, mag.y, mag.z, angleX, angleY);
        if (isfinite(heading)) {
          magHeading = heading;
          const bool update_reference = takeMagReferenceRequest();
          if (update_reference) {
            // Relative heading: enabling mag or resetting angleZ must not
            // command a turn toward magnetic north.
            magYawReferenceOffset = wrapDeg(angleZ - heading);
          }
          float referencedHeading =
              wrapDeg(heading + magYawReferenceOffset);
          angleZ += magYawCorrectionDeg(angleZ, referencedHeading, K_MAG);
        }
      }
    }
    magFusionCnt++;
    if (magFusionCnt >= OUTER_DIV) magFusionCnt = 0;

    // ---------- 안전 검사 ----------
    if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
      if (!fault_attitude) Serial.printf("[FAULT] OVER-TILT R:%.1f P:%.1f\n", angleX, angleY);
      fault_attitude = true;
      safety_lock = true;
    }
    if (takeFailsafeResumeRequest()) {
      const ResumeRefusalReason refusal = resumeRefusalReason(nowMs);
      if (refusal != RESUME_ALLOWED) {
        logResumeRefusal(refusal);
      } else {
        fs_phase = FS_NONE;
        fault_rc = false;
        base_throttle = (int)lroundf(hover_est);
        min_throttle = max(1050, base_throttle - CTRL_MARGIN);
        max_throttle = min(1900, base_throttle + CTRL_MARGIN);
        Serial.printf(">>> RESUME hover=%d\n", base_throttle);
      }
    }
    // 잠금 해제 첫 tick의 D kick과 이전 목표 rate 잔류를 제거한다.
    // 이 블록은 RC timeout 진입 가드보다 먼저 실행해야 한다. 뒤에 두면 같은
    // tick에서 새로 진입한 FS_DESCENDING까지 FS_NONE으로 덮어쓰게 된다.
    if (wasLocked && !safety_lock) {
      // start가 남긴 이전 비행의 terminal phase와 start/abort 경합 결과를
      // Core 1에서 해제해 fs_phase의 런타임 writer를 pid_task 하나로 유지한다.
      fs_phase = FS_NONE;
      prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      outerCnt = 0;
      fs_first_enter_ms = 0;
      fs_first_enter_valid = false;
      wasLocked = false;
    }

    const float accelMag =
        sqrtf(accX*accX + accY*accY + accZ*accZ);
    const bool hoverSampleEligible =
        !safety_lock && fs_phase == FS_NONE &&
        fabsf(angleX) < HOVER_LEVEL_MAX_DEG &&
        fabsf(angleY) < HOVER_LEVEL_MAX_DEG &&
        fabsf(accelMag - 1.0f) <= HOVER_ACCEL_TOL_G &&
        base_throttle > HOVER_MIN_THROTTLE_US;
    updateHoverThrottleEstimator(
        hoverTracker, hoverSampleEligible, base_throttle, nowMs, realDt,
        HOVER_LPF_TAU_S, HOVER_VALID_MS);
    hover_est = hoverTracker.estimate_us;
    hover_valid = hoverTracker.valid;

    // RC 타임아웃은 유효 호버 추정치가 없거나 보수적 절대 지상 스로틀이면
    // 즉시 컷하고, 그 외에는 추정 호버를 기준으로 자동착륙한다. hover_est는
    // 하강 목표에만 쓰며, 오염된 높은 값이 공중 즉시컷 경계를 올리지 못한다.
    // 진입 블록 전체를 fs_phase 가드 안에 둔다 — 가드가 없으면 하강 내내
    // rcTimedOut이 참이라 이 블록이 1kHz로 재실행되고, Serial.println이 TX
    // 버퍼를 포화시켜 pid_task를 블로킹하면 500ms 태스크 워치독이 비행 중
    // 재부팅을 일으킨다.
    if (fs_phase == FS_NONE && !safety_lock && rcTimedOut(nowMs, lastRcMs)) {
      fault_rc = true;
      if (!hover_valid) {
        safety_lock = true;
        Serial.println("[FAULT] RC TIMEOUT -> CUT (NO HOVER EST)");  // 한 번만
      } else if (base_throttle <= FS_GROUND_CUT_MAX_US) {
        safety_lock = true;
        Serial.println("[FAULT] RC TIMEOUT -> GROUND CUT");  // 한 번만
      } else {
        fs_phase = FS_DESCENDING;
        fs_hold_yaw = angleZ;
        fs_enter_ms = nowMs;
        if (!fs_first_enter_valid) {
          fs_first_enter_ms = nowMs;
          fs_first_enter_valid = true;
        }
        landDet = {};
        landProbeConfig = FS_LAND_PROBE_CONFIG;
        bool dipClamped = false;
        fs_probe_dip_us = failsafeProbeDipUs(
            hover_est, FS_PROBE_DIP_FRAC, FS_PROBE_DIP_MIN_US,
            CTRL_MARGIN, dipClamped);
        landProbeConfig.dip_us = fs_probe_dip_us;
        if (dipClamped) {
          Serial.printf(
              "[WARN] AUTO-LAND PROBE DIP CLAMPED hover=%.1f applied=%d\n",
              hover_est, fs_probe_dip_us);
        }
        fs_probe_state = FS_PROBE_WAIT;
        fs_probe_no_response = 0;
        fs_probe_response_g = 0.0f;
        fs_probe_blocked_logged = false;
        base_throttle = failsafeDescentThrottle(
            (int)lroundf(hover_est), FS_DESCENT_DELTA_US);
        Serial.println("[FAULT] RC TIMEOUT -> AUTO-LAND");   // 한 번만
      }
    }

    // 하강 중에는 목표를 매 tick 덮어쓴다. rc/rcr 파서도 하강 중 목표 변경을
    // 막으며, 링크 복구만으로는 하강을 끝내지 않는다(명시적 resume 필요).
    if (fs_phase == FS_DESCENDING) {
      targetAngleX = trim_roll;
      targetAngleY = trim_pitch;
      targetAngleZ = fs_hold_yaw;
      targetYawRate = 0.0f;
      const int descentThrottle = failsafeDescentThrottle(
          (int)lroundf(hover_est), FS_DESCENT_DELTA_US);

      const uint32_t elapsed = nowMs - fs_enter_ms;
      // 프로브는 계속 돌리되 **착지 판정에는 쓰지 않는다.**
      //
      // 2026-08-01의 "010251 공중 CUT_LANDED" 해석은 철회됐다. 20Hz 단일
      // 샘플을 자유낙하로 오독했으며 1kHz 기록과 조종자 확인은 접지를 지지한다.
      // 따라서 "지면 응답이 공중보다 큼 / 판별 방향 반대 / 오분류 9/18"도
      // 근거가 없다. 유효한 실패 관측은 015448의 접지 응답 10회가 임계 0.03
      // 위여서 착지를 확정하지 못했다는 것이다. 023904의 접지 전 응답 2회만으로
      // 공중 오검출률을 검증할 수 없어 검출기는 제어에 복귀시키지 않았다.
      //
      // 따라서 자동착륙은 **시간 기반 하강**이다: FS_MAX_MS 동안 내려간 뒤
      // 백스톱이 자른다. 이 경로는 CUT_LANDED를 선택하지 않지만, 접지 전에
      // 예산이 끝나면 TIMEOUT 공중 컷은 가능하다. 접지 후에는 남은 시간만큼
      // 프롭이 돈다. 고도·수직속도 피드백이나 접지 확인은 하지 않는다.
      //
      // 프로브 딥은 실제 스로틀에 계속 적용된다 — 텔레메트리(41~43)로 기록해 다음 판별식을
      // 오프라인 데이터로 설계할 수 있다. 분석은 docs/failsafe_land_research.md.
      const bool probe_says_landed = updateLandDetector(
          landDet, accelMag, elapsed, descentThrottle,
          landProbeConfig);
      (void)probe_says_landed;
      const bool landed = false;
      fs_probe_state = landDet.probe_state;
      fs_probe_no_response = landDet.no_response_count;
      fs_probe_response_g = landDet.last_response_g;
      if (landDet.probe_state == FS_PROBE_BLOCKED &&
          !fs_probe_blocked_logged) {
        fs_probe_blocked_logged = true;
        Serial.println("[WARN] AUTO-LAND PROBE BLOCKED");  // 하강당 최대 1회
      }

      const bool probeActive = landDetectorProbeActive(landDet);
      checkProbeDelivery = probeActive;
      probeReferenceThrottle = descentThrottle;
      probeElapsedMs = elapsed;
      base_throttle =
          descentThrottle -
          (probeActive ? fs_probe_dip_us : 0);
      // 딥 tick에만 collective 하한을 요청 딥만큼 넓힌다. 자세 차동이 기존
      // 하한을 밀어 올려 딥을 자르면 검출기가 공중을 지면으로 오판할 수 있다.
      // 상한은 그대로 두고 1050us 절대 하한도 기존 clamp로 보존한다.
      min_throttle = max(
          1050,
          descentThrottle - CTRL_MARGIN - (probeActive ? fs_probe_dip_us : 0));
      max_throttle = min(1900, descentThrottle + CTRL_MARGIN);
      const FailsafePhase next = failsafeStep(landed, elapsed, FS_MAX_MS);
      if (next != FS_DESCENDING) {
        fs_phase = next;
        safety_lock = true;      // 아래 safety_lock 블록이 이번 tick에 정리한다
        Serial.printf(">>> AUTO-LAND END phase=%d\n", (int)next);
      }
    }
    if (safety_lock) {
      if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;
      mixer_scaled = false;
      iTermRoll = iTermPitch = iTermYaw = 0.0f;
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      targetYawRate = 0.0f;
      targetAngleZ = angleZ;
      yaw_hold_now = false;
      prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      outerCnt = 0;
      enterLockedState(wasLocked);
      motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
      tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
      stopMotors();
      continue;
    }

    // ---------- Outer loop (250Hz): 각도 -> 목표 각속도 ----------
    // yaw는 각속도 명령 + 자동 heading 잠금. hold가 아닌 동안 setpoint를
    // 매 tick 현재 heading으로 슬레이빙해 stale setpoint를 원천 차단한다.
    const YawOuter yawOuter = updateYawOuter(
        targetYawRate, bodyGz, angleZ, targetAngleZ,
        yaw_hold_now, yaw_hold_override,
        YAW_RATE_DEADZONE, YAW_HOLD_SETTLE_DPS);
    targetAngleZ = yawOuter.target_angle_deg;
    yaw_hold_now = yawOuter.hold;

    if (outerCnt == 0) {
      targetRateRoll = constrain((targetAngleX - angleX) * Kp_Angle_Roll,
                                 -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
      targetRatePitch = constrain((targetAngleY - angleY) * Kp_Angle_Pitch,
                                  -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
      targetRateYaw = yawTargetRateDps(yawOuter, targetYawRate, angleZ,
                                       Kp_Angle_Yaw, MAX_TARGET_RATE_YAW);
    }
    outerCnt++;
    if (outerCnt >= OUTER_DIV) outerCnt = 0;

    // ---------- Inner loop (1kHz): 각속도 PID ----------
    float eRoll  = targetRateRoll  - bodyGx;
    float ePitch = targetRatePitch - bodyGy;
    float eYaw   = targetRateYaw   - bodyGz;

    // D는 gyro 미분에만 건다 (setpoint/outer loop 오염 제거)
    float dRoll  = lpfD_Roll.update((bodyGx - prevGyroX) / realDt);
    float dPitch = lpfD_Pitch.update((bodyGy - prevGyroY) / realDt);
    float dYaw   = lpfD_Yaw.update((bodyGz - prevGyroZ) / realDt);
    prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;

    float pidRoll  = Kp_Rate_Roll *eRoll  + iTermRoll  - Kd_Rate_Roll *dRoll;
    float pidPitch = Kp_Rate_Pitch*ePitch + iTermPitch - Kd_Rate_Pitch*dPitch;
    // yaw 각도 유지가 꺼져 있어도 rate 감쇠(target 0)는 항상 동작한다.
    // 모터 토크 불균형과 롤·피치 보정의 반작용으로 생기는 자유 회전을 억제한다.
    float pidYaw   = Kp_Rate_Yaw*eYaw + iTermYaw - Kd_Rate_Yaw*dYaw;

    // ---------- 모터 desaturation + 포화 기반 anti-windup ----------
    const int throttle = base_throttle;
    MotorMix mix = mixAndDesaturate(pidRoll, pidPitch, pidYaw,
                                    throttle, min_throttle, max_throttle);
    mixer_scaled = mix.scaled;
    if (checkProbeDelivery) {
      const float deliveredUs =
          (float)probeReferenceThrottle - mix.collective_us;
      const bool deliveryValid =
          deliveredUs >= FS_PROBE_MIN_DELIVERY_FRAC * fs_probe_dip_us;
      recordLandDetectorProbeDelivery(
          landDet, probeElapsedMs, deliveryValid, landProbeConfig);
    }

    // scale은 자세 명령이 실제로 잘린 경우다. collective 이동만 일어난 경우에는
    // 자세 authority가 보존되므로 적분을 계속한다.
    // 적분은 모터 출력 기여(µs) 단위로 누적하고 I_TERM_MAX_US로 제한한다.
    // (이전 errorSum ±200deg 클램프는 기본 Ki에서 최대 기여 1µs라 적분이
    //  사실상 없는 것과 같아 CG 치우침 같은 정상상태 오차를 못 잡았다.)
    if (throttle > 1100 && !mix.scaled) {
      iTermRoll  = constrain(iTermRoll  + Ki_Rate_Roll  * eRoll  * realDt,
                             -I_TERM_MAX_US, I_TERM_MAX_US);
      iTermPitch = constrain(iTermPitch + Ki_Rate_Pitch * ePitch * realDt,
                             -I_TERM_MAX_US, I_TERM_MAX_US);
      // yaw 적분은 rate/hold 두 모드 모두에서 돈다. 안쪽 루프의 임무가
      // "목표 각속도 추종"이고, P 단독으로는 모터 토크 불균형 같은 일정
      // 외란에서 정상상태 각속도 오차가 남아 정착 임계치 아래로 내려가지
      // 않는다(2026-07-27 실측 최대 +16.6dps). 그러면 자동 잠금이 영영
      // 걸리지 않는다.
      iTermYaw = constrain(iTermYaw + Ki_Rate_Yaw * eYaw * realDt,
                           -I_TERM_MAX_US, I_TERM_MAX_US);
    } else if (throttle <= 1100) {
      iTermRoll = iTermPitch = iTermYaw = 0.0f;
    }

    // 모터 PWM의 단일 writer는 PID task로 유지한다.
    if (safety_lock) {
      motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
      tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
      stopMotors();
      enterLockedState(wasLocked);
      continue;
    }
    writeMotor(pinM1, mix.motor[0]);
    writeMotor(pinM2, mix.motor[1]);
    writeMotor(pinM3, mix.motor[2]);
    writeMotor(pinM4, mix.motor[3]);
    motorOut[0] = mix.motor[0]; motorOut[1] = mix.motor[1];
    motorOut[2] = mix.motor[2]; motorOut[3] = mix.motor[3];
    tgtRate[0] = targetRateRoll; tgtRate[1] = targetRatePitch; tgtRate[2] = targetRateYaw;
  }
}

// ==========================================================
// 9. UDP 태스크 (Core 0) — char* 파싱 (String heap 회피)
// ==========================================================
static char *trimCommand(char *s) {
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  char *end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) end--;
  *end = '\0';
  return s;
}

static bool parseFloatStrict(const char *text, float &out) {
  char *end;
  float value = strtof(text, &end);
  if (end == text || !isfinite(value)) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  out = value;
  return true;
}

static bool parseIntStrict(const char *text, long &out) {
  char *end;
  long value = strtol(text, &end, 10);
  if (end == text) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  out = value;
  return true;
}

static bool setRcTargets(float x, float y, float z, bool hasYaw) {
  if (!isfinite(x) || !isfinite(y) || (hasYaw && !isfinite(z))) return false;
  if (fs_phase == FS_DESCENDING) {
    lastRcMs = millis();
    return true;
  }
  // 트림을 더한 뒤 클램프해야 총합이 ±30°로 제한된다.
  targetAngleX = constrain(x + trim_roll,  -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  targetAngleY = constrain(y + trim_pitch, -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  if (hasYaw) targetAngleZ = z;
  lastRcMs = millis();
  return true;
}

// 표준 형식: rc <seq> <roll> <pitch> <yaw>
// 벤치 수동 테스트용으로 rc <roll> <pitch> [yaw]도 허용한다.
static void handleRcCommand(char *buf) {
  char *save = nullptr;
  char *token = strtok_r(buf, " \t", &save); // "rc"
  (void)token;
  char *arg[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 4 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (strtok_r(nullptr, " \t", &save) != nullptr) return; // 여분 필드 거부

  if (count == 4) {
    if (arg[0][0] == '-' || arg[0][0] == '+') return;
    char *seqEnd;
    errno = 0;
    unsigned long seqLong = strtoul(arg[0], &seqEnd, 10);
    if (seqEnd == arg[0] || *seqEnd != '\0' || errno == ERANGE ||
        seqLong > 0xFFFFFFFFUL) return;

    float x, y, z;
    if (!parseFloatStrict(arg[1], x) || !parseFloatStrict(arg[2], y) ||
        !parseFloatStrict(arg[3], z)) return;

    uint32_t seq = (uint32_t)seqLong;
    rcTotalPkts = rcTotalPkts + 1;
    if (rcSeqValid) {
      int32_t advance = (int32_t)(seq - lastRcSeq); // uint32 wrap도 정상 처리
      if (advance <= 0) {
        rcDroppedPkts = rcDroppedPkts + 1;
        return; // 지연 도착/중복 패킷 폐기
      }
      if (advance > 1) rcDroppedPkts += (uint32_t)(advance - 1);
    }
    lastRcSeq = seq;
    rcSeqValid = true;
    setRcTargets(x, y, z, true);
    return;
  }

  if (count == 2 || count == 3) {
    float x, y, z = 0.0f;
    if (!parseFloatStrict(arg[0], x) || !parseFloatStrict(arg[1], y)) return;
    if (count == 3 && !parseFloatStrict(arg[2], z)) return;
    setRcTargets(x, y, z, count == 3);
  }
}

// rcr <seq> <roll> <pitch> <yaw_rate>
// yaw는 각속도(dps) 명령이다. seq 처리와 워치독 급이는 rc와 상태를 공유한다.
static void handleRcrCommand(char *buf) {
  char *save = nullptr;
  (void)strtok_r(buf, " \t", &save); // "rcr"
  char *arg[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 4 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (count != 4) return;
  if (strtok_r(nullptr, " \t", &save) != nullptr) return; // 여분 필드 거부

  if (arg[0][0] == '-' || arg[0][0] == '+') return;
  char *seqEnd;
  errno = 0;
  unsigned long seqLong = strtoul(arg[0], &seqEnd, 10);
  if (seqEnd == arg[0] || *seqEnd != '\0' || errno == ERANGE ||
      seqLong > 0xFFFFFFFFUL) return;

  float x, y, rate;
  if (!parseFloatStrict(arg[1], x) || !parseFloatStrict(arg[2], y) ||
      !parseFloatStrict(arg[3], rate)) return;

  uint32_t seq = (uint32_t)seqLong;
  rcTotalPkts = rcTotalPkts + 1;
  if (rcSeqValid) {
    int32_t advance = (int32_t)(seq - lastRcSeq);
    if (advance <= 0) {
      rcDroppedPkts = rcDroppedPkts + 1;
      return;
    }
    if (advance > 1) rcDroppedPkts += (uint32_t)(advance - 1);
  }
  lastRcSeq = seq;
  rcSeqValid = true;

  if (!setRcTargets(x, y, 0.0f, false)) return;   // roll/pitch만, yaw 각도는 건드리지 않는다
  if (fs_phase == FS_DESCENDING) return;
  targetYawRate = constrain(rate, -MAX_TARGET_RATE_YAW, MAX_TARGET_RATE_YAW);
}

static void handleGainCommand(const char *buf) {
  if (strlen(buf) < 3) return;
  float value;
  if (!parseFloatStrict(buf + 2, value) || value < 0.0f || value > 100.0f) return;
  if (!configChangeAllowed("PID gain")) return;

  // Cascade 전용 공통 명령
  if      (strncmp(buf, "rp", 2) == 0) { Kp_Rate_Roll = value; Kp_Rate_Pitch = value; }
  else if (strncmp(buf, "ri", 2) == 0) { Ki_Rate_Roll = value; Ki_Rate_Pitch = value; }
  else if (strncmp(buf, "rd", 2) == 0) { Kd_Rate_Roll = value; Kd_Rate_Pitch = value; }
  else if (strncmp(buf, "ap", 2) == 0) { Kp_Angle_Roll = value; Kp_Angle_Pitch = value; }
  else if (strncmp(buf, "ar", 2) == 0) Kp_Angle_Roll = value;
  else if (strncmp(buf, "at", 2) == 0) Kp_Angle_Pitch = value;
  else if (strncmp(buf, "ay", 2) == 0) Kp_Angle_Yaw = value;
  else if (strncmp(buf, "yp", 2) == 0) Kp_Rate_Yaw = value;
  else if (strncmp(buf, "yi", 2) == 0) Ki_Rate_Yaw = value;
  else if (strncmp(buf, "yd", 2) == 0) Kd_Rate_Yaw = value;

  // scripts/tune_pid.py 명령 호환: P/I/D는 inner rate PID에 대응
  else if (strncmp(buf, "pa", 2) == 0) { Kp_Rate_Roll = value; Kp_Rate_Pitch = value; }
  else if (strncmp(buf, "ia", 2) == 0) { Ki_Rate_Roll = value; Ki_Rate_Pitch = value; }
  else if (strncmp(buf, "da", 2) == 0) { Kd_Rate_Roll = value; Kd_Rate_Pitch = value; }
  else if (strncmp(buf, "pr", 2) == 0) Kp_Rate_Roll = value;
  else if (strncmp(buf, "ir", 2) == 0) Ki_Rate_Roll = value;
  else if (strncmp(buf, "dr", 2) == 0) Kd_Rate_Roll = value;
  else if (strncmp(buf, "pp", 2) == 0) Kp_Rate_Pitch = value;
  else if (strncmp(buf, "ip", 2) == 0) Ki_Rate_Pitch = value;
  else if (strncmp(buf, "dp", 2) == 0) Kd_Rate_Pitch = value;
  else if (strncmp(buf, "py", 2) == 0) Kp_Rate_Yaw = value;
  else if (strncmp(buf, "iy", 2) == 0) Ki_Rate_Yaw = value;
  else if (strncmp(buf, "dy", 2) == 0) Kd_Rate_Yaw = value;
}

// 첫 14개 필드는 기존 PC 스크립트와 호환된다. 뒤 필드는 cascade 진단 확장:
// fault_imu1,fault_imu2,fault_disagree,active_imus,scaled,fault_attitude,
// calibration_ok,armed. 그 뒤 Tier1 8개와 MagHeading, Mag_X/Y/Z, Yaw_Hold,
// Failsafe_Phase, Trim_Roll/Pitch, Hover_Est/Valid와 프로브 상태/연속
// 무반응/최근 차분 반응, IMU1 gyro/accel XYZ, IMU2 gyro/accel XYZ를
// 보낸 뒤 목표 roll/pitch/yaw 각도와 필드 59 Mag_Enabled, 3901-L0X 5개
// 필드, 필드 65 Mag_Cal_Active를 append-only로 보낸다. IMU별 값은 융합값과
// 같은 body frame이다.
static void sendTelemetry() {
  if (!connectionEstablished) return;
  bool criticalFault = (active_imus == 0) || fault_attitude || !calibration_ok;
  const float hoverEstSnapshot = hover_est;
  const bool hoverValidSnapshot = hover_valid;
  const uint8_t probeStateSnapshot = fs_probe_state;
  const uint8_t probeNoResponseSnapshot = fs_probe_no_response;
  const float probeResponseSnapshot = fs_probe_response_g;
  const ImuTelemetrySample imuSample = readImuSampleSnapshot();
  float magTelemXSnapshot = 0.0f;
  float magTelemYSnapshot = 0.0f;
  float magTelemZSnapshot = 0.0f;
  bool magTelemCalActiveSnapshot = false;
  readMagTelemetry(
      magTelemXSnapshot, magTelemYSnapshot, magTelemZSnapshot,
      magTelemCalActiveSnapshot);
  udp.beginPacket(laptopIP, laptopPort);
  udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%d,%d,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d,%ld,%d,%ld,%ld,%d,%d",
             angleX, angleY, angleZ,
             gyroX, gyroY, gyroZ,
             accX, accY, accZ,
             base_throttle,
             (int)fault_rc, (int)criticalFault,
             (unsigned long)rcTotalPkts, (unsigned long)rcDroppedPkts,
             (int)fault_imu1, (int)fault_imu2, (int)fault_disagree,
             active_imus, (int)mixer_scaled, (int)fault_attitude,
             (int)calibration_ok, (int)!safety_lock,
             motorOut[0], motorOut[1], motorOut[2], motorOut[3], pidLoopHz,
             tgtRate[0], tgtRate[1], tgtRate[2], magHeading,
             magTelemXSnapshot, magTelemYSnapshot, magTelemZSnapshot,
             (int)yaw_hold_now,
             (int)fs_phase, trim_roll, trim_pitch,
             hoverEstSnapshot, (int)hoverValidSnapshot,
             (int)probeStateSnapshot, (int)probeNoResponseSnapshot,
             probeResponseSnapshot,
             imuSample.imu1GyroX, imuSample.imu1GyroY, imuSample.imu1GyroZ,
             imuSample.imu1AccelX, imuSample.imu1AccelY, imuSample.imu1AccelZ,
             imuSample.imu2GyroX, imuSample.imu2GyroY, imuSample.imu2GyroZ,
             imuSample.imu2AccelX, imuSample.imu2AccelY, imuSample.imu2AccelZ,
             targetAngleX, targetAngleY, targetAngleZ, (int)mag_enabled,
             (long)msp_range_mm, msp_range_quality,
             (long)msp_flow_x, (long)msp_flow_y, msp_flow_quality,
             (int)magTelemCalActiveSnapshot);
  udp.endPacket();
}

static bool sendImuRawBatch() {
  if (!connectionEstablished) return false;
  uint32_t queued = imuRawRingCount(imuRawRing);
  if (queued == 0) return false;
  const uint8_t nSamples = static_cast<uint8_t>(
      min(queued, static_cast<uint32_t>(IMU_RAW_BATCH_MAX)));
  uint32_t baseTUs = 0;
  uint8_t copied = 0;
  for (; copied < nSamples; copied++) {
    ImuRawSample sample = {};
    if (!imuRawRingPop(imuRawRing, sample)) break;
    uint32_t sampleUs;
    if (!rawConsumerTimeValid) {
      sampleUs = __atomic_load_n(&imuRawRing.first_t_us, __ATOMIC_ACQUIRE);
    } else {
      sampleUs = rawConsumerLastUs + sample.dt_us;
    }
    if (copied == 0) baseTUs = sampleUs;
    rawConsumerLastUs = sampleUs;
    rawConsumerTimeValid = true;
    imuRawBatch[copied] = sample;
  }
  if (copied == 0) return false;

  const uint32_t dropped =
      __atomic_load_n(&imuRawRing.dropped, __ATOMIC_RELAXED);
  const size_t packetSize = buildImuRawDatagram(
      imuRawDatagram, rawBatchSeq, baseTUs, dropped, imuRawBatch, copied);
  udp.beginPacket(laptopIP, laptopPort);
  udp.write(reinterpret_cast<const uint8_t *>(&imuRawDatagram), packetSize);
  udp.endPacket();
  rawBatchSeq++;
  return true;
}

static bool sendImuCalibration() {
  if (!connectionEstablished) return false;
  memcpy(imuCalDatagram.magic, "ZCAL", 4);
  imuCalDatagram.version = IMU_RAW_VERSION;
  memset(imuCalDatagram.reserved, 0, sizeof(imuCalDatagram.reserved));
  for (int axis = 0; axis < 3; axis++) {
    imuCalDatagram.gyro_bias1[axis] = gyro_bias1[axis];
    imuCalDatagram.gyro_bias2[axis] = gyro_bias2[axis];
    imuCalDatagram.imu2_sign[axis] = IMU2_SIGN[axis];
  }
  imuCalDatagram.accel_scale1 = accel_scale1;
  imuCalDatagram.accel_scale2 = accel_scale2;
  imuCalDatagram.gyro_scale = GYRO_SCALE;
  imuCalDatagram.accel_scale = ACCEL_SCALE;
  udp.beginPacket(laptopIP, laptopPort);
  udp.write(reinterpret_cast<const uint8_t *>(&imuCalDatagram),
            sizeof(imuCalDatagram));
  udp.endPacket();
  return true;
}

// 3901-L0X UART 폴링. **논블로킹**: available() 만큼만, 반복당 상한까지.
// 어떤 것도 제어하지 않는다 — 텔레메트리 기록 전용이다.
static void pollMspSensor(uint32_t nowMs) {
  int budget = MSP_SENSOR_MAX_BYTES_PER_TICK;
  while (budget-- > 0 && Serial1.available() > 0) {
    const MspParseResult result =
        mspSensorParseByte(mspSensor, (uint8_t)Serial1.read());
    if (result == MSP_PARSE_RANGEFINDER) {
      msp_range_mm = mspSensor.range.distance_mm;
      msp_range_quality = (int)mspSensor.range.quality;
      msp_last_range_ms = nowMs;
    } else if (result == MSP_PARSE_OPTIC_FLOW) {
      msp_flow_x = mspSensor.flow.motion_x;
      msp_flow_y = mspSensor.flow.motion_y;
      msp_flow_quality = (int)mspSensor.flow.quality;
      msp_last_flow_ms = nowMs;
    } else if (result == MSP_PARSE_UNKNOWN &&
               mspSensor.last_unknown_cmd != msp_reported_unknown) {
      // function 상수가 틀렸다면 "값이 안 나온다"가 아니라 이 줄로 드러난다.
      msp_reported_unknown = mspSensor.last_unknown_cmd;
      Serial.printf("[MSP] unknown function 0x%04X\n",
                    mspSensor.last_unknown_cmd);
    }
  }
  // 신선하지 않으면 quality 를 -1 로. 값 자체는 건드리지 않는다 —
  // distance_mm 는 모듈이 범위 밖에서 음수를 쓰므로 센티넬로 못 쓴다.
  if ((int32_t)(nowMs - msp_last_range_ms) > (int32_t)MSP_SENSOR_STALE_MS) {
    msp_range_quality = -1;
  }
  if ((int32_t)(nowMs - msp_last_flow_ms) > (int32_t)MSP_SENSOR_STALE_MS) {
    msp_flow_quality = -1;
  }
}

void udp_task(void *pv) {
  uint32_t lastSend = 0;
  uint32_t lastRawCheck = 0;
#if WIFI_LATENCY_DEBUG
  uint32_t prevTopUs = micros();
#endif
  while (true) {
#if WIFI_LATENCY_DEBUG
    uint32_t topUs = micros();
    uint32_t loopGapUs = topUs - prevTopUs;
    prevTopUs = topUs;
    if (loopGapUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
      recordLatEvent(udpLatRing, millis(), loopGapUs, LAT_UDP_LOOP);
    }
    uint32_t parseStartUs = micros();
#endif
    int packetSize = udp.parsePacket();
#if WIFI_LATENCY_DEBUG
    uint32_t parseDurUs = micros() - parseStartUs;
    if (parseDurUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
      recordLatEvent(udpLatRing, millis(), parseDurUs, LAT_UDP_PARSE);
    }
#endif
    if (packetSize) {
      laptopIP = udp.remoteIP();
      laptopPort = udp.remotePort();
      connectionEstablished = true;

      int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1); // off-by-one 방지
      if (len > 0) {
        packetBuffer[len] = '\0';
        char *buf = trimCommand(packetBuffer);

        if (strncmp(buf, "rcr", 3) == 0 && (buf[3] == ' ' || buf[3] == '\t')) {
          handleRcrCommand(buf);
        }
        else if (strncmp(buf, "rc", 2) == 0 && (buf[2] == ' ' || buf[2] == '\t')) {
          handleRcCommand(buf);
        }
        else if (strcmp(buf, "start") == 0) {
          bool overTilt = fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE;
          bool noUsableImu = imu1_frozen_now && imu2_frozen_now;
          if (fs_phase == FS_DESCENDING) {
            Serial.println(">>> START REFUSED (auto-land descending)");
          }
          else if (!safety_lock || safety_arm_requested) {
            // 이미 시동 상태. 지상국은 start를 여러 번 재전송하므로, 지연
            // 도착한 중복 start가 비행 중 fault latch를 지우고 스로틀 창을
            // 리셋하는 것을 막는다.
            Serial.println(">>> START ignored (already armed)");
          }
          else if (!calibration_ok || overTilt || noUsableImu
                   || imu_disagree_now || mag_calibrating) {
            Serial.printf(">>> START REFUSED calib=%d tilt=%d imu=%d disagree=%d magcal=%d\n",
                          (int)calibration_ok, (int)overTilt,
                          (int)noUsableImu, (int)imu_disagree_now,
                          (int)mag_calibrating);
          } else {
            // safety_lock=true인 동안 Core 0 소유 비행 상태를 초기화한다.
            // hover tracker는 requestSafetyArm()이 Core 1 reset을 함께 요청한다.
            fault_rc = false;
            fault_imu1 = imu1_frozen_now;
            fault_imu2 = imu2_frozen_now;
            fault_disagree = false;
            fault_attitude = false;
            lastRcSeq = 0;
            rcSeqValid = false;
            lastRcMs = millis();
            base_throttle = 1100; min_throttle = 1050; max_throttle = 1250;
            targetAngleX = 0.0f;
            targetAngleY = 0.0f;
            targetAngleZ = 0.0f;
            angleZ = 0.0f;
            requestMagReferenceUpdate();
            fs_probe_state = FS_PROBE_WAIT;
            fs_probe_no_response = 0;
            fs_probe_response_g = 0.0f;
            requestSafetyArm();
            Serial.println(">>> START");
          }
        }
        else if (strcmp(buf, "stop") == 0) {
          requestSafetyDisarm();
          base_throttle = 1000;
          Serial.println(">>> STOP");
        }
        else if (strcmp(buf, "resume") == 0) {
          const ResumeRefusalReason refusal = resumeRefusalReason(millis());
          if (refusal != RESUME_ALLOWED) {
            logResumeRefusal(refusal);
          } else {
            requestFailsafeResume();
          }
        }
        else if (strncmp(buf, "th", 2) == 0) {
          long parsed;
          if (parseIntStrict(buf + 2, parsed)) {
            int nb = constrain((int)parsed, 1000, 1900);
            base_throttle = nb;
            min_throttle = max(1050, nb - CTRL_MARGIN);
            max_throttle = min(1900, nb + CTRL_MARGIN);
          }
        }
        else if (strncmp(buf, "yaw", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            if (enabled == 0) {
              yaw_hold_override = false;
              Serial.println(">>> Yaw hold OFF (auto)");
            } else if (!safety_lock) {
              // 비행 중 임의로 켜면 지상국 setpoint와 어긋난 채 최대 권한
              // 슬램이 날 수 있다. 시동 해제 상태에서만 켤 수 있게 한다.
              Serial.println(">>> Yaw hold refused (armed)");
            } else {
              targetAngleZ = angleZ;
              yaw_hold_override = true;
              Serial.println(">>> Yaw hold ON (override)");
            }
          }
        }
        else if (strncmp(buf, "magcal", 6) == 0) {
          long enabled;
          if (parseIntStrict(buf + 6, enabled) && (enabled == 0 || enabled == 1)) {
            if (enabled == 0) {
              stopMagCalibration();
            } else if (!safety_lock) {
              Serial.println(">>> Magcal refused (armed)");
            } else if (initMagnetometer()) {
              startMagCalibration();
            }
          }
        }
        else if (strncmp(buf, "magc", 4) == 0) {
          char *save = nullptr;
          char *token = strtok_r(buf, " \t", &save); // "magc"
          (void)token;
          char *arg[3] = {nullptr, nullptr, nullptr};
          int count = 0;
          while (count < 3 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
          if (count == 3 && strtok_r(nullptr, " \t", &save) == nullptr) {
            float x, y, z;
            if (parseFloatStrict(arg[0], x) && parseFloatStrict(arg[1], y) &&
                parseFloatStrict(arg[2], z)) {
              if (configChangeAllowed("Mag comp")) {
                mag_comp_x = x;
                mag_comp_y = y;
                mag_comp_z = z;
                Serial.printf(">>> mag comp = %.4f %.4f %.4f uT/us\n",
                              (double)mag_comp_x, (double)mag_comp_y, (double)mag_comp_z);
              }
            }
          }
        }
        else if (strncmp(buf, "mag", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            if (enabled == 0) {
              mag_enabled = false;
              requestMagReferenceUpdate();
              Serial.println(">>> Mag OFF");
            } else if (mag_calibrating) {
              Serial.println(">>> Mag refused (magcal active)");
            } else if (!mag_ready && !safety_lock) {
              // First-time init does blocking I2C/delay(100); only when disarmed.
              Serial.println(">>> Mag refused (armed, not initialized)");
            } else if (initMagnetometer()) {
              requestMagReferenceUpdate();
              mag_enabled = true;
              Serial.println(">>> Mag ON");
            }
          }
        }
        else if (strncmp(buf, "raw", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            const bool wasEnabled =
                __atomic_load_n(&raw_stream_enabled, __ATOMIC_ACQUIRE);
            if (enabled == 0) {
              __atomic_store_n(&raw_stream_enabled, false, __ATOMIC_RELEASE);
              Serial.println(">>> Raw IMU OFF");
            } else {
              if (!wasEnabled) {
                const uint32_t head =
                    __atomic_load_n(&imuRawRing.head, __ATOMIC_ACQUIRE);
                __atomic_store_n(&imuRawRing.tail, head, __ATOMIC_RELEASE);
                rawProducerTimeValid = false;
                rawConsumerTimeValid = false;
                rawLastSendMs = millis();
                rawLastCalMs = rawLastSendMs - 1000U;
                __atomic_store_n(
                    &raw_stream_enabled, true, __ATOMIC_RELEASE);
              }
              Serial.println(">>> Raw IMU ON");
            }
          }
        }
        else if (strncmp(buf, "trim", 4) == 0 &&
                 (buf[4] == ' ' || buf[4] == '\t')) {
          char *save = nullptr;
          (void)strtok_r(buf, " \t", &save);          // "trim"
          char *arg0 = strtok_r(nullptr, " \t", &save);
          char *arg1 = strtok_r(nullptr, " \t", &save);
          if (arg0 != nullptr && arg1 != nullptr &&
              strtok_r(nullptr, " \t", &save) == nullptr) {
            float r, p;
            if (parseFloatStrict(arg0, r) && parseFloatStrict(arg1, p)) {
              if (configChangeAllowed("Trim")) {
                trim_roll  = constrain(r, -TRIM_MAX_DEG, TRIM_MAX_DEG);
                trim_pitch = constrain(p, -TRIM_MAX_DEG, TRIM_MAX_DEG);
                Serial.printf(">>> Trim R:%.2f P:%.2f\n", trim_roll, trim_pitch);
              }
            }
          }
        }
        else if (strcmp(buf, "gains") == 0) {
          if (connectionEstablished) {
            udp.beginPacket(laptopIP, laptopPort);
            udp.printf("GAINS,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                       Kp_Angle_Roll, Kp_Angle_Pitch, Kp_Angle_Yaw,
                       Kp_Rate_Roll, Kp_Rate_Pitch, Kp_Rate_Yaw,
                       Ki_Rate_Roll, Ki_Rate_Pitch, Ki_Rate_Yaw,
                       Kd_Rate_Roll, Kd_Rate_Pitch, Kd_Rate_Yaw);
            udp.endPacket();
          }
        }
        else {
          handleGainCommand(buf);
        }
      }
    }

    uint32_t now = millis();
    if (mag_enabled || mag_calibrating) sampleMagnetometer(now);
    pollMspSensor(now);
    bool sentImuPacket = false;
    if (now - lastRawCheck >= 10U) {
      lastRawCheck = now;
      bool sentRawPacket = false;
      if (__atomic_load_n(&raw_stream_enabled, __ATOMIC_ACQUIRE)
          && connectionEstablished) {
        const uint32_t queued = imuRawRingCount(imuRawRing);
        if (queued >= IMU_RAW_BATCH_MAX
            || (queued > 0 && now - rawLastSendMs >= 50U)) {
          sentImuPacket = sendImuRawBatch();
          sentRawPacket = sentImuPacket;
          if (sentImuPacket) rawLastSendMs = now;
        }
        // 한 udp_task 반복에서는 ZIMU 또는 ZCAL 중 최대 하나만 추가 전송한다.
        if (!sentRawPacket && now - rawLastCalMs >= 1000U) {
          sentImuPacket = sendImuCalibration();
          if (sentImuPacket) rawLastCalMs = now;
        }
      }
    }
    // 새 바이너리 경로는 한 반복에 최대 한 데이터그램만 보낸다. 같은 시각의
    // 기존 ASCII 텔레메트리는 다음 반복으로 미뤄 20Hz 주기를 유지한다.
    if (!sentImuPacket && now - lastSend >= 50) {
      lastSend = now;
#if WIFI_LATENCY_DEBUG
      uint32_t sendStartUs = micros();
#endif
      sendTelemetry();
#if WIFI_LATENCY_DEBUG
      uint32_t sendDurUs = micros() - sendStartUs;
      if (sendDurUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
        recordLatEvent(udpLatRing, millis(), sendDurUs, LAT_UDP_SEND);
      }
#endif
    }
    vTaskDelay(1);
  }
}

// ==========================================================
// 10. setup / loop
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(SPI_CS1, OUTPUT); pinMode(SPI_CS2, OUTPUT);
  digitalWrite(SPI_CS1, HIGH); digitalWrite(SPI_CS2, HIGH);   // float CS 로 인한 버스 경합 방지
  delay(100);

  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  udp.begin(UDP_PORT);

  // 3901-L0X 수신 전용 UART. TX 핀 -1 로 두어 핀 하나만 점유한다.
  Serial1.begin(MSP_SENSOR_BAUD, SERIAL_8N1, MSP_SENSOR_RX_PIN, -1);

  SPI.begin(12, 13, 11, -1);   // SCK, MISO, MOSI; CS는 각 IMU 객체가 관리

  bool esc_ok = ledcAttach(pinM1, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM2, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM3, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM4, ESC_FREQ, ESC_RES);
  if (!esc_ok) while (1) { Serial.println("[FAULT] ESC attach FAIL"); delay(1000); }
  stopMotors();

  if (IMU1.begin() < 0) while (1) { Serial.println("[FAULT] IMU1 FAIL"); delay(1000); }
  if (IMU2.begin() < 0) while (1) { Serial.println("[FAULT] IMU2 FAIL"); delay(1000); }
  int sensorStartStatus = 0;
  sensorStartStatus |= IMU1.startAccel(1600, 16);
  sensorStartStatus |= IMU1.startGyro(1600, 2000);
  sensorStartStatus |= IMU2.startAccel(1600, 16);
  sensorStartStatus |= IMU2.startGyro(1600, 2000);
  sensorStartStatus |= IMU1.setLowPassFilters();
  sensorStartStatus |= IMU2.setLowPassFilters();
  if (sensorStartStatus != 0) {
    while (1) { Serial.printf("[FAULT] IMU START FAIL (%d)\n", sensorStartStatus); delay(1000); }
  }
  delay(500);

  calibration_ok = calibrate_bias();

  // pid_task 전용 태스크 워치독. idle task는 감시하지 않고, timeout 시
  // panic 재부팅으로 마지막 PWM 고정 상태에서 벗어난다.
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = PID_WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK &&
      esp_task_wdt_init(&wdt_config) != ESP_OK) {
    while (1) { Serial.println("[FAULT] TASK WDT INIT FAIL"); delay(1000); }
  }

  BaseType_t pidTaskOk = xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 2, NULL, 1);
  BaseType_t udpTaskOk = xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 1, NULL, 0);
  if (pidTaskOk != pdPASS || udpTaskOk != pdPASS) {
    safety_lock = true;
    while (1) { Serial.println("[FAULT] TASK CREATE FAIL"); delay(1000); }
  }

  Serial.println("DUAL_IMU_CASCADE READY");
}

void loop() {
#if WIFI_LATENCY_DEBUG
  drainLatRing(udpLatRing);
  drainLatRing(pidLatRing);

  uint32_t nowMs = millis();
  if (nowMs - latLastSummaryMs >= 2000U) {
    latLastSummaryMs = nowMs;
    uint32_t dropped = __atomic_load_n(&udpLatRing.dropped, __ATOMIC_RELAXED)
                     + __atomic_load_n(&pidLatRing.dropped, __ATOMIC_RELAXED);
    Serial.printf("[LATSUM] up=%lu udpMaxUs=%lu pidMaxUs=%lu "
                  "udpLoopOver30=%lu udpLoopOver100=%lu udpLoopOver300=%lu "
                  "pidOver5=%lu dropped=%lu\n",
                  (unsigned long)nowMs,
                  (unsigned long)latUdpMaxUs, (unsigned long)latPidMaxUs,
                  (unsigned long)latUdpLoopOver30,
                  (unsigned long)latUdpLoopOver100,
                  (unsigned long)latUdpLoopOver300,
                  (unsigned long)latPidOver5, (unsigned long)dropped);
  }
  delay(100);
#else
  // UDP 객체는 udp_task 한 곳에서만 접근해 cross-core race를 피한다.
  delay(1000);
#endif
}
