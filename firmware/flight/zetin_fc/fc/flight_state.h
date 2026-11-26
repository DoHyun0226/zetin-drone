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
// 목표 자세 (°). X = roll, Y = pitch, Z = yaw(heading). rc/rcr 명령이 쓰고 pid_task 바깥 루프가 읽는다
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
// IMU별 텔레메트리 값 (pid_task가 매 tick 갱신)
ImuTelemetrySample imuSampleSnapshot = {};
// 12개 float를 한 번에 복사하도록 보호 (중간에 섞인 값 방지)
portMUX_TYPE imuSampleMux = portMUX_INITIALIZER_UNLOCKED;
// 마지막으로 내보낸 모터 출력 (텔레메트리용)
volatile int   motorOut[4] = {1000,1000,1000,1000};
volatile float tgtRate[3]  = {0,0,0}; // roll,pitch,yaw dps
// 실측 pid_task 주기 (1초마다 계산, 정상이면 약 1000)
volatile int   pidLoopHz   = 0;

float iTermRoll = 0.0f, iTermPitch = 0.0f, iTermYaw = 0.0f;  // 적분 기여 (모터 µs)

// 부팅 캘리브레이션으로 구한 자이로 영점 오차 (dps). 매 샘플에서 뺀다
float gyro_bias1[3] = {0,0,0};
float gyro_bias2[3] = {0,0,0};
// 두 센서의 gain 오차는 독립이고 단일-IMU 폴백도 있으므로 각각 보정한다.
// [accel_scale] 정지 시 |a|가 1g가 되도록 맞추는 부팅 보정 배율 (단위 없음, 약 1.0). 예) 측정 평균 0.98g → 1/0.98 ≈ 1.020
//  - 1.0은 "오차 없음"이 아니라 "보정 안 함": 부팅 전 초기값이고, 캘리브레이션 실패나 평균이 0.8~1.2g 밖일 때도 1.0으로 남는다
//  - ACCEL_SCALE(1/2048, LSB → g 단위 변환)과 다른 값이다. accel_scale을 곱해도 값은 아직 LSB 단위
//  - 필요한 이유: compute_alpha()와 착지 감지가 |accel|과 1g의 차이를 기준으로 판단하고, 두 IMU 평균도 스케일이 맞아야 한다
//  - 한계: 3축 공통 배율 하나라 전체 크기만 맞춘다. 축별 오프셋/스케일 오차는 보정하지 못한다
//  - 구조체 ImuCalDatagram에 같은 이름의 멤버가 있지만 별개의 변수다. sendImuCalibration()이 이 값을 패킷에 복사한다
float accel_scale1 = 1.0f;
float accel_scale2 = 1.0f;

// 마지막 rc/rcr 명령 수신 시각 (RC 워치독 기준)
volatile uint32_t lastRcMs        = 0;
// RC 끊김 발생
volatile bool     fault_rc        = false;
volatile bool     fault_imu1      = false;   // IMU1 값 멈춤(freeze)
volatile bool     fault_imu2      = false;   // IMU2 값 멈춤(freeze)
volatile bool     fault_disagree  = false;   // 두 IMU 불일치 (중재 불가)
volatile bool     fault_attitude  = false;   // 과도 기울기
volatile int      active_imus     = 2;       // 현재 사용 중인 IMU 수 (telemetry)
volatile bool     mixer_scaled    = false;   // 자세 mixer가 축소됐는지
volatile float    mixer_yaw_scale = 1.0f;    // 다음 tick yaw authority 입력
AllocationTelemetrySample allocationTelemetry = {
    false, 1.0f, 1.0f, 1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    YAW_AUTH_NORMAL};
portMUX_TYPE allocationTelemetryMux = portMUX_INITIALIZER_UNLOCKED;
// 부팅 자이로 캘리브레이션 성공. false면 start 거부
volatile bool     calibration_ok  = false;

// 재시동 판단용 현재 센서 상태. fault_imu* / fault_disagree는 비행 중 latch된다.
volatile bool     imu1_frozen_now = false;
volatile bool     imu2_frozen_now = false;
volatile bool     imu_disagree_now = false;

// scripts/drone_control.py 프로토콜 호환용
// 마지막으로 받아들인 rc 패킷 번호
volatile uint32_t lastRcSeq       = 0;
// lastRcSeq가 유효한가 (start 후 첫 패킷 전엔 false)
volatile bool     rcSeqValid      = false;
// 받은 rc 패킷 수
volatile uint32_t rcTotalPkts     = 0;
// 번호가 건너뛰거나 늦게 온 패킷 수 (UDP 손실 추정)
volatile uint32_t rcDroppedPkts   = 0;
