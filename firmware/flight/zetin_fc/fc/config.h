// zetin_fc/fc/config.h — 1. 튜닝 파라미터, 2. 시스템 상수.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// ==========================================================
// 1. 튜닝 파라미터
// ==========================================================
// [Cascade 제어 정리] 두 겹으로 겹친 제어기
//  목표 각도 ─▶ [Outer: Kp_Angle_*] ─▶ 목표 각속도 ─▶ [Inner: K*_Rate_*] ─▶ 모터 출력 차이
//     (°)          250Hz                  (dps)            1kHz               (µs)
//  Kp_Angle_* : 각도 오차를 보고 "목표 각속도"를 정할 때 사용 (예: 5° 틀어짐 → 30dps로 돌아가라)
//  K*_Rate_*  : 목표 각속도와 실제 각속도의 차이를 보고, 그걸 따라가도록 "모터 출력(µs)"을 정할 때 사용
//               (예: 30dps로 돌아야 하는데 지금 10dps → 모터 출력 차이를 이만큼 더 벌려라)
//  바깥 루프가 목표 각속도를 만들고, 안쪽 루프가 그 각속도를 실제로 만들어 낸다
// [Outer] 각도 P: 각도 오차 -> 목표 각속도(dps)
// 비행 중 노트북에서 같은 UDP 명령으로 실시간으로 바꿀 수 있고, pid_task가 매 tick 읽기 때문에 volatile로 선언
// 단위 dps/° : 각도 오차 1°당 목표 각속도 6dps
volatile float Kp_Angle_Roll  = 6.0f;
volatile float Kp_Angle_Pitch = 6.0f;
// heading은 천천히 맞춰도 되고, 급하게 돌리면 모터 여유를 많이 먹어 절반
volatile float Kp_Angle_Yaw   = 3.0f;

// [Inner] 각속도 PID: 각속도 오차 -> 모터 파워
//  Kp_Rate (µs/dps)     : 지금 각속도 오차에 비례해 즉시 반응
//  Ki_Rate (µs/(dps·s)) : 오래 남는 오차를 쌓아서 제거 (CG 치우침, 모터 편차)
//  Kd_Rate (µs/(dps/s)) : 각속도가 급하게 변하면 제동 → 오버슈트 감소
//  yaw는 반토크라는 약한 힘으로 돌아서 Kp가 3배(1.5), 원래 둔하게 움직여 D항은 노이즈만 키우므로 0
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

// 모터 4개의 기본 출력
volatile int  base_throttle = 1000;
// 각 모터 출력의 하한
volatile int  min_throttle  = 1050;
// 각 모터 출력의 상한
volatile int  max_throttle  = 1300;
// true면 updateYawOuter()의 hold가 항상 true:
//  - 요 스틱(rcr yaw_rate)은 무시되고, targetAngleZ를 현재 heading으로 따라가게(슬레이빙) 하지 않는다
//  - 요 목표 각속도 = wrap(targetAngleZ - angleZ) * Kp_Angle_Yaw (±MAX_TARGET_RATE_YAW)
//    → 외란으로 밀려도 원래 방위로 복원, rc 명령의 yaw 필드가 "절대 heading 지정"으로 동작
//  - rc로 따로 안 주면 시동 순간의 heading이 목표가 된다 (잠금 중엔 매 tick targetAngleZ = angleZ)
// 켜기: "yaw 1"은 시동 해제(safety_lock) 상태에서만 수락. 끄기: "yaw 0" 또는 잠금 진입 시 자동 해제
// 주의: 목표-현재 차이가 크면 최대 180dps로 급회전(슬램), angleZ 드리프트도 그대로 따라 돈다 → 벤치 시험용
volatile bool yaw_hold_override = false;   // "yaw 1": heading 고정 + 재슬레이빙 금지
volatile bool yaw_hold_now = false;   // 텔레메트리용: 현재 heading 잠금 중인가
// yaw 권한 상태 (yaw_authority.h). pid_task가 쓰고 텔레메트리가 읽는다
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
// 자기 보정 계수
volatile float mag_comp_x =  0.007467272f;  // µT/µs
volatile float mag_comp_y = -0.001149868f;
volatile float mag_comp_z = -0.000526263f;

// [모터 전류 자기 간섭 보정 정리]
//  모터 전류가 만든 자기장이 섞이면 스로틀을 올릴수록 heading이 틀어진다 (보정 전 약 3.6°/100µs)
//  MAG_THROTTLE_REF_US(1000µs, ESC 정지 신호) = 전류 없음 → 간섭 0으로 보는 기준점
//  보정: magThrDelta = base_throttle - 1000, mag.축 -= mag_comp_축 × magThrDelta (축별 µT/µs 계수)
//  예) base_throttle 1300 → 300µs × 0.00747 ≈ x축 2.24µT 빼기 (지구 수평 자기장 수십 µT라 몇 도 차이)
//  한계: 개별 모터 출력이 아니라 공통 스로틀 기준이라, PID가 모터별로 다르게 줄 때의 간섭 차이는 못 잡는다

// ==========================================================
// 2. 시스템 상수
// ==========================================================
// 런타임에 바뀌지 않는 하드웨어/통신/안전 설정값. const라 비행 중 변경 불가(재빌드 필요)
//
// --- WiFi / UDP: 드론이 AP(공유기)가 되고 노트북이 접속한다 ---
// 드론이 만드는 WiFi 이름
const char* WIFI_SSID    = "Drone_Tuning";
// WiFi 비밀번호 (8자 이상이어야 WPA2 AP가 열림)
const char* WIFI_PASS    = "12345678";
// 명령 수신/텔레메트리 송신 포트 (지상국과 같아야 함)
const int   UDP_PORT     = 4210;
const int   WIFI_CHANNEL = 11;  // ch6 혼잡(인근 AP 다수) → 스캔상 가장 빈 채널로 이동

// --- 핀 배치 (ESP32-S3 GPIO 번호) ---
// 모터 PWM 출력 핀. F/R = 앞/뒤, L/R = 왼쪽/오른쪽. 믹서의 motor[0..3] 순서와 같다
const int pinM1   = 4;   // FL
const int pinM2   = 5;   // RR
const int pinM3   = 6;   // FR
const int pinM4   = 7;   // RL
// SPI 칩 선택(CS) 핀: SCK/MISO/MOSI는 두 IMU가 공유하고 CS로 누구와 통신할지 고른다
const int SPI_CS1 = 10;  // IMU1
const int SPI_CS2 = 9;   // IMU2
// Matek 3901-L0X (광류+라이다). 모듈 TX -> 이 핀, 3.3V 로직이라 직결한다.
// 모듈 RX 는 연결하지 않는다 — 우리는 보낼 것이 없고 수신 전용이다.
// PCB v1.5.2 의 소나 커넥터 UART(ESP RX=17, TX=18) 중 RX 만 쓴다.
// 3901-L0X 수신용 UART RX 핀 (Serial1)
const int MSP_SENSOR_RX_PIN = 17;
// UART 통신 속도 (모듈 설정과 같아야 함)
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

// BMM350 자기계: I2C 핀과 센서 전원 스위치
// I2C 데이터
const int BMM_SDA = 35;
// I2C 클럭
const int BMM_SCL = 36;
// 자기계 전원 레귤레이터(LDO) 켜기 핀. initMagnetometer()에서 HIGH
const int LDO_SHD = 38;

// --- ESC PWM 신호 (LEDC 하드웨어 PWM) ---
// 400Hz = 2500µs 주기마다 1000~2000µs 펄스를 보낸다 (1000 정지, 2000 최대)
// PWM 주파수 (Hz)
const int ESC_FREQ   = 400;
// 듀티 분해능 14비트 = 0~16383 단계
const int ESC_RES    = 14;
// us = 1/400Hz. 400Hz PWM은 2500µs 주기
const int ESC_PERIOD = 2500;                 // us
// writeMotor()에서 duty = µs × ESC_MAXDUTY / ESC_PERIOD 로 펄스 폭을 듀티 값으로 바꾼다
const int ESC_MAXDUTY = (1 << ESC_RES) - 1;  // magic number 제거

// --- 센서 단위 변환 (setup의 FSR 설정과 짝) ---
// 자이로 FSR ±2000dps → 16.4 LSB/(deg/s), 가속도 FSR ±16g → 2048 LSB/g
const float GYRO_SCALE   = 1.0f / 16.4f;     // raw -> deg/s
const float ACCEL_SCALE  = 1.0f / 2048.0f;   // raw -> g
// --- 자세/명령 제한 ---
// 롤/피치가 이 각도(°)를 넘으면 OVER-TILT로 모터 정지
const float SAFETY_ANGLE = 60.0f;
// |yaw 각속도| < 0.3dps면 angleZ 적분 안 함 (정지 중 드리프트 억제)
const float YAW_DEADZONE = 0.3f;
const float MAX_TARGET_ANGLE_RP = 30.0f;     // UDP 오입력에 대한 자세 명령 제한
const float TRIM_MAX_DEG = 10.0f;   // 트림 절대값 상한
const float MAX_TARGET_RATE_RP  = 300.0f;    // outer-loop 출력 제한 (deg/s)
// yaw 목표 각속도 상한 (deg/s)
const float MAX_TARGET_RATE_YAW = 180.0f;
// yaw 모드 판정 임계치. 벤치에서 조정한다(설계 문서 §1 참조).
const float YAW_RATE_DEADZONE   = 3.0f;    // 명령 각속도 절댓값이 이 값 미만이면 스틱 중립
const float YAW_HOLD_SETTLE_DPS = 10.0f;   // bodyGz 절댓값이 이 값 미만이면 새 잠금 진입 가능
// --- 자기계(나침반) 융합 ---

// 250Hz outer loop에서 K=0.001 -> tau ≈ 0.004/K = 4s.
// angleZ를 나침반 방향으로 끌어당기는 비율 (작을수록 천천히)
const float K_MAG = 0.001f;
const uint32_t MAG_SAMPLE_PERIOD_MS = 20;    // BMM350 읽기 50 Hz, Core 0 에서만
// 이보다 오래된 자기계 값은 융합에 쓰지 않음(ms)
const uint32_t MAG_STALE_MS = 100;

// scripts/drone_logs.py magcal 출력값을 아래 두 상수에 붙여 넣는다.
// 2026-08-04 캘리브 (log flight_log_2026-08-04_023840.csv, 1255샘플 중 inlier
// 1119, 커버리지비 2.08). 이전 min/max offset은 여기서 6.79µT 벗어나 있었고
// 그것은 수평성분 32.2µT 대비 최대 ±12.2° heading 오차였다.
// 교차검증: 무작위 절반 분할 8회의 중심 차이 중앙값 0.29µT (시간 분할은
// 후반 커버리지비가 39.6이라 b_z를 구속하지 못하므로 쓰면 안 된다).
// 물리 정합성: 피팅 반경 50.10µT vs 한국 총강도 실측 ~50.4µT (오차 0.6%).
// hard-iron: 기체에 붙은 고정 자석 성분 (µT). 측정값에서 뺀다
const float MAG_HARD_IRON[3] = {-3.457566904f, 4.794903225f, -17.307261041f};
// soft-iron: 금속 때문에 타원으로 찌그러진 측정을 원으로 펴는 3x3 행렬
const float MAG_SOFT_IRON[3][3] = {
  {0.998555071f, 0.005825468f, 0.018437852f},
  {0.005825468f, 0.976739944f, 0.006050833f},
  {0.018437852f, 0.006050833f, 1.026752877f},
};

// BMM350 축이 기체 축과 다르면 벤치 sign test 뒤 여기만 바꾼다.
const float MAG_BODY_SIGN_X = 1.0f;
const float MAG_BODY_SIGN_Y = 1.0f;
const float MAG_BODY_SIGN_Z = 1.0f;

// --- IMU 장착 방향 부호 ---

// IMU2는 x,z축이 IMU1 대비 반전 (y는 동일). 첫 파일 기준 그대로.
// 원값에 곱해서 IMU2를 IMU1 축 기준으로 맞춘 뒤 두 값을 평균/비교한다
const float IMU2_SIGN_X = -1.0f;
const float IMU2_SIGN_Y =  1.0f;
const float IMU2_SIGN_Z = -1.0f;
static const float IMU1_SIGN[3] = { 1.0f, 1.0f, 1.0f };
static const float IMU2_SIGN[3] = { IMU2_SIGN_X, IMU2_SIGN_Y, IMU2_SIGN_Z };

// --- 안전/redundancy 임계값 (하드웨어 맞춰 튜닝 필요) ---
// 조종 명령(rc/rcr)이 이 시간 동안 없으면 페일세이프 (20Hz면 10개 연속 손실)
const uint32_t RC_TIMEOUT_MS      = 500;
// 아래 failsafe 상수는 전부 벤치 조정 대상이다.
// [페일세이프 자동착륙 요약] RC가 끊기면 hover_est - FS_DESCENT_DELTA_US 로 스로틀을
// 낮춰 천천히 내려가고, FS_MAX_MS가 지나면 모터를 끈다. 아래 FS_PROBE_* 는 착지 감지
// 실험용 값으로 현재 판정에는 쓰이지 않고 텔레메트리 기록에만 쓰인다.
// th 명령 시 min/max = base ∓ 150µs. PID가 쓸 수 있는 여유
constexpr int CTRL_MARGIN = 150;
// 자동착륙: 호버 스로틀보다 이만큼 낮춰 하강
const int      FS_DESCENT_DELTA_US = 60;
static_assert(FS_DESCENT_DELTA_US < CTRL_MARGIN,
              "FS_DESCENT_DELTA_US must be less than CTRL_MARGIN");
// 컴파일 타임 가드는 delta < CTRL_MARGIN만 검사한다. 실제 하한은
// delta < min(CTRL_MARGIN, entry_throttle - 1050)이며 entry_throttle은
// 런타임 값이라 static_assert로 잡을 수 없다. 예: entry=1150이면 delta=120도
// 이미 collective 하한에 걸려 실제 하강이 일어나지 않는다.
// 하강 시작 후 이 시간이 지나야 착지 프로브 시작
const uint32_t FS_MIN_DESCEND_MS   = 1000;
// (착지 감지용 |accel| 필터)
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
// [프로브] 스로틀을 잠깐 낮춰(딥) 기체가 반응하면 공중, 반응 없으면 지면으로 본다
// 딥 크기 = (hover_est - 1000) × 11.8%
constexpr float FS_PROBE_DIP_FRAC      = 0.118f;
// 딥 최소 크기 (µs)
const int      FS_PROBE_DIP_MIN_US      = 20;
// 프로브 반복 주기
const uint32_t FS_PROBE_PERIOD_MS       = 400;
// 딥 유지 시간
const uint32_t FS_PROBE_DIP_MS          = 120;
// 딥 시작 후 이만큼 기다렸다가 반응 측정 시작
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
// 이보다 작은 가속도 변화(g)면 "반응 없음"
constexpr float FS_PROBE_RESPONSE_G    = 0.03f;
// 응답은 전달 딥에 비례한다. 기존 1.5배 명목 응답 보장에서 80%를 요구하면
// 임계 대비 1.2배가 남지만, 50% 전달은 임계 아래라 판정 근거가 사라진다.
// 요청한 딥의 80% 이상이 실제 모터에 반영돼야 유효한 프로브
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
// 착지 후보 조건: |accel|이 1g ± 0.1g 안 (정지 상태)
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
// "반응 없음"이 이 횟수 연속이면 착지 후보
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
// 자동착륙 최대 시간. 지나면 모터 정지 (현재 실제 종료 조건)
const uint32_t FS_MAX_MS           = 3000;
// 첫 자동착륙 진입 후 이 시간이 지나면 resume 거부
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
// 위 프로브 상수들을 한 구조체로 묶어 failsafe_land.h의 updateLandDetector()에 넘긴다
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
// [호버 추정] 수평·정지 상태의 스로틀을 천천히 평균내 "떠 있는 데 필요한 스로틀"(hover_est)을 학습
// 롤/피치가 이 안일 때만 호버 샘플로 인정
const float    HOVER_LEVEL_MAX_DEG    = 10.0f;
// |accel|이 1g ± 0.05g 안일 때만 (상승/하강 중 제외)
const float    HOVER_ACCEL_TOL_G      = 0.05f;
// 이 스로틀 이하는 지상으로 보고 제외
const int      HOVER_MIN_THROTTLE_US  = 1150;
// hover_est 하한이 이 값보다 낮으면 하강값에서 딥을 뺀 스로틀이 1000us
// 아래가 되어 프로브가 UNAVAILABLE에 고정된다. 현재 컷은 프로브 가용성과
// 무관하게 FS_MAX_MS 타임아웃을 사용하지만 진단용 딥의 가용성은 유지한다.
static_assert(
    HOVER_MIN_THROTTLE_US
        >= 1000 + FS_DESCENT_DELTA_US + FS_PROBE_DIP_MIN_US,
    "hover estimate floor must keep the probe dip above idle");
// 호버 평균 필터 시정수 (초)
const float    HOVER_LPF_TAU_S        = 3.0f;
// 조건을 만족한 시간이 누적 1.5초 넘으면 hover_valid = true
const uint32_t HOVER_VALID_MS         = 1500;
// 실측 호버는 약 1340us다. 무장 직후 spool-up 로그(041032)는 1100~1190us
// 구간이 전체 무장 시간의 9.4%였으므로, 지상 즉시컷은 보수적으로 1150us
// 이하에서만 허용한다. 이보다 높으면 hover_est가 오염됐더라도 자동착륙을
// 택한다(유효 호버 추정치가 있어야 한다). 지상에서도 FS_MAX_MS 동안 모터가
// 돌 수 있으며, 이 절대 하한은 고도나 실제 접지를 판별하는 센서가 아니다.
// RC 끊김 시 스로틀이 이 이하면 지상으로 보고 즉시 모터 정지
const int      FS_GROUND_CUT_MAX_US  = 1150;
// --- 센서 고장 감시 ---
const uint32_t PID_WDT_TIMEOUT_MS = 500;     // pid_task 정지 시 강제 재부팅 (1kHz 루프 대비 큰 여유)
const int32_t  FROZEN_DELTA_RAW   = 1;       // 6축 raw 변화량 합이 이 이하면 정지 의심 (LSB)
const uint32_t IMU_FROZEN_MS      = 300;     // 그 상태가 이만큼 지속되면 freeze 확정
// [IMU 불일치 판정] x, y, z 중 한 축이라도 각속도 차이가 15dps를 넘는 상태가 150ms 계속되면 fault_disagree를 켠다
//  단위는 각도(°)가 아니라 각속도(°/s). 중간에 한 tick이라도 15dps 이하로 돌아오면 시간 측정을 처음부터 다시 한다 (순간 튐 무시)
//  fault가 켜지면 어느 IMU가 틀렸는지 알 수 없으므로 두 값을 모두 쓰지 않고 시동을 끈다
const float    GYRO_DISAGREE_DPS  = 15.0f;   // 두 IMU 각속도 차이 임계 (deg/s)
const uint32_t IMU_DISAGREE_MS    = 150;     // 불일치 지속 시간

// --- 부팅 시 자이로 bias 캘리브레이션 (calibrate_bias) ---
// 1ms 간격 2000개 = 약 2초 동안 평균
const int   BIAS_CALIB_SAMPLES   = 2000;
const float BIAS_MOVEMENT_THRESH = 1.0f;     // 축별 stddev 상한 (deg/s)
// 움직임이 감지되면 최대 3번 재시도
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

// --- 루프 주기 ---
const int OUTER_DIV = 4;                      // 바깥 루프 = 1 kHz / 4 = 250 Hz

// --- 필터/적분 상한 ---
const float GYRO_SW_LPF_HZ = 80.0f;  // 하드웨어 LPF(121Hz)에 겹치는 소프트웨어 LPF
const float I_TERM_MAX_US  = 50.0f;  // 축별 적분 기여 상한 (모터 µs)
