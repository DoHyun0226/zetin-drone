// zetin_fc/fc/flight_state.h — 호버 추정, 목표·추정 자세, 모터 출력, 적분, IMU 보정값, fault, RC 수신 카운터, allocation 텔레메트리.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
volatile float hover_est = 0.0f;       // 추정 호버 collective (us)
// hover_valid는 만료시키지 않는다. false는 RC 끊김 시 즉시 컷을 뜻하므로
// 공중에서 freshness 만료로 false가 되면 추락한다. 낡은 추정치가 덜 위험하다.
volatile bool hover_valid = false;
// hoverTracker와 그 공개 mirror는 pid_task만 쓴다. Core 0의 start는
// safetyRequestMux를 통해 reset을 요청하고 직접 구조체를 초기화하지 않는다.
HoverThrottleEstimator hoverTracker = {};
volatile float targetAngleX = 0.0f, targetAngleY = 0.0f, targetAngleZ = 0.0f;
// 기체 트림(도). 추정기 0°와 진짜 수평의 차이를 보정한다. 비행별 상태가 아니라
// 기체 속성이므로 start/stop이 지우지 않는다. 기본값은 2026-10-30 테더 비행에서
// 조종자가 맞춘 값(재부팅해도 남도록). 지상국은 접속 때 이 값을 받아 쓴다.
volatile float trim_roll = -2.2f;
volatile float trim_pitch = -1.8f;
volatile float targetYawRate = 0.0f;   // rcr이 준 yaw 각속도 명령 (dps)

volatile float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f; // 추정 각도
volatile float gyroX  = 0.0f, gyroY  = 0.0f, gyroZ  = 0.0f; // 융합 각속도 (body frame)
volatile float accX   = 0.0f, accY   = 0.0f, accZ   = 0.0f; // 융합 가속도 (g)
ImuTelemetrySample imuSampleSnapshot = {};
portMUX_TYPE imuSampleMux = portMUX_INITIALIZER_UNLOCKED;
volatile int   motorOut[4] = {1000,1000,1000,1000};
volatile float tgtRate[3]  = {0,0,0}; // roll,pitch,yaw dps
volatile int   pidLoopHz   = 0;

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
volatile float    mixer_yaw_scale = 1.0f;    // 다음 tick yaw authority 입력
AllocationTelemetrySample allocationTelemetry = {
    false, 1.0f, 1.0f, 1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    YAW_AUTH_NORMAL};
portMUX_TYPE allocationTelemetryMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool     calibration_ok  = false;

// 재시동 판단용 현재 센서 상태. fault_imu* / fault_disagree는 비행 중 latch된다.
volatile bool     imu1_frozen_now = false;
volatile bool     imu2_frozen_now = false;
volatile bool     imu_disagree_now = false;

// scripts/drone_control.py 프로토콜 호환용
volatile uint32_t lastRcSeq       = 0;
volatile bool     rcSeqValid      = false;
volatile uint32_t rcTotalPkts     = 0;
volatile uint32_t rcDroppedPkts   = 0;

