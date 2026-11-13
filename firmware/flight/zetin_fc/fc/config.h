// zetin_fc/fc/config.h — 1. 튜닝 파라미터, 2. 시스템 상수.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
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
// 위치 유지 스케치는 0.2: 0.05 에서는 1.5° 자세 차이를 메우는 데 약 10 s 가 걸려 위치
// 유지 보정이 자세로 옮겨지지 않았다. 2026-10-30 테더 비행에서 0.2 는 손 뗀 위치 흔들림을
// 0.14 → 0.07 m 로 줄였고, 0.3 은 pitch 각속도 흔들림이 4.5 → 6.3 dps 로 늘며 0.11 m 로
// 나빠졌다.
volatile float Kp_Rate_Roll  = 0.50f, Ki_Rate_Roll  = 0.20f, Kd_Rate_Roll  = 0.015f;
volatile float Kp_Rate_Pitch = 0.50f, Ki_Rate_Pitch = 0.20f, Kd_Rate_Pitch = 0.015f;
volatile float Kp_Rate_Yaw   = 1.50f, Ki_Rate_Yaw   = 0.05f,  Kd_Rate_Yaw   = 0.0f;

volatile int  base_throttle = 1000;
volatile int  min_throttle  = 1050;
volatile int  max_throttle  = 1300;
volatile bool yaw_hold_override = false;   // "yaw 1": heading 고정 + 재슬레이빙 금지
volatile bool yaw_hold_now = false;   // 텔레메트리용: 현재 heading 잠금 중인가
volatile int yaw_authority_state = YAW_AUTH_NORMAL;

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
// PCB v1.5.2 의 소나 커넥터 UART(ESP RX=17, TX=18) 중 RX 만 쓴다.
const int MSP_SENSOR_RX_PIN = 17;
const uint32_t MSP_SENSOR_BAUD = 115200;
// 이보다 오래 프레임이 없으면 quality 를 -1 로 보고한다. 얼어붙은 센서가
// 마지막 값으로 유효해 보이면 안 된다(MAG_STALE_MS 와 같은 이유).
const uint32_t MSP_SENSOR_STALE_MS = 500;
// udp_task 한 반복에서 처리할 바이트 상한. BMM350 의 블로킹 I2C 가 RC
// 수신큐를 넘치게 했던 전례가 있어 이 경로는 절대 블로킹하지 않는다.
// 실제 유량은 20Hz x ~15B = 300B/s 라 이 상한에 닿을 일이 없다.
const int MSP_SENSOR_MAX_BYTES_PER_TICK = 64;
// 직전 pollMspSensor 호출과 이보다 멀면 Core 0 가 막혀 바이트가 FIFO 에 쌓인
// 것이다. 그때 완성된 프레임의 도착 시각은 측정 시각으로 믿지 않는다.
const uint32_t FLOW_POLL_GAP_MAX_US = 15000;
// 높이 프레임이 이보다 오래되면 광류 환산·고도 유지에 쓰지 않는다(28 Hz).
const uint32_t FLOW_HEIGHT_MAX_AGE_US = 150000;
// 보정은 이 거리 안에서만 누적한다(손 보정 권장 40~70 cm).
const float FLOW_CAL_MIN_D_M = 0.30f;
const float FLOW_CAL_MAX_D_M = 1.50f;
// 컴파일 보정값. `flowcal` 이 출력한 줄을 붙여 넣고 VALID 를 true 로 바꾼다.
// 미보정이면 poshold 를 거부한다.
// 2026-10-30 손 보정(센서를 기체 축에 맞춰 장착, 바닥 약 0.8 m): residual 0.038,
// cond 1.15, corr -0.05, samples 365. 센서를 다시 달면 다시 보정한다.
const bool     FLOW_CAL_VALID = true;
const float    FLOW_CAL_M[2][2] = {{16.14f, 545.93f}, {525.30f, -11.23f}};   // counts/rad
const uint32_t FLOW_CAL_DELAY_US = 30000;

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

// scripts/drone_logs.py magcal 출력값을 아래 두 상수에 붙여 넣는다.
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
// 부팅 때는 보정하지 않고 저장값을 쓴다. 지상국 `imucal` 이 정지 상태에서 다시 재서
// NVS 에 저장하면 그 값이 우선이고, NVS 가 비었을 때 아래 컴파일 값을 쓴다. 둘 다 없으면
// (VALID=false) 예전처럼 부팅 때 정지 보정을 한다. 이 값은 2026-10-30 이 기체의 부팅
// 보정이며 두 부팅이 축마다 0.03 dps 안에서 일치했다(ZCAL 로그). IMU·PCB 를 바꾸면
// `imucal` 로 다시 잰다.
const bool  STORED_IMU_CAL_VALID = true;
const float STORED_GYRO_BIAS1[3] = {-0.7474f, 0.1842f, -0.3703f};   // dps
const float STORED_GYRO_BIAS2[3] = {1.6685f, -0.2698f, 0.9102f};
const float STORED_ACCEL_SCALE1  = 0.9914f;
const float STORED_ACCEL_SCALE2  = 0.9992f;
const int      IMU_CAL_SAMPLES     = BIAS_CALIB_SAMPLES;   // 2 s @ 1 kHz
const uint32_t IMU_CAL_MAX_TICKS   = 2 * BIAS_CALIB_SAMPLES; // 읽기 실패가 이어지면 포기
const float    IMU_CAL_MAX_BIAS_DPS = 10.0f;   // NVS 값 타당성. 넘으면 버린다
const uint32_t IMU_CAL_MAGIC       = 0x5A494331u;   // "ZIC1"

const int OUTER_DIV = 4;                      // outer loop = 1kHz / 4 = 250Hz

// --- 필터/적분 상한 ---
const float GYRO_SW_LPF_HZ = 80.0f;  // 하드웨어 LPF(121Hz)에 겹치는 소프트웨어 LPF
const float I_TERM_MAX_US  = 50.0f;  // 축별 적분 기여 상한 (모터 µs)
