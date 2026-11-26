// zetin_fc/fc/flow_state.h — 3901-L0X(MSP) 수신 상태, 광류 스냅숏, 위치·고도 유지·광류 보정·IMU 보정 요청, 광류 추정기 객체.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// 3901-L0X 상태. udp_task 만 쓰고 sendTelemetry 가 읽는다(같은 코어).
// UART 바이트 파서 상태
MspSensorParser mspSensor;
// 라이다 거리 (mm)
volatile int32_t msp_range_mm = 0;
volatile int     msp_range_quality = -1;   // -1 = 신선한 프레임 없음
// 광류 이동량
volatile int32_t msp_flow_x = 0, msp_flow_y = 0;
volatile int     msp_flow_quality = -1;
// 마지막 라이다 프레임 수신 시각 (신선도 판단)
uint32_t msp_last_range_ms = 0;
// 마지막 광류 프레임 수신 시각
uint32_t msp_last_flow_ms = 0;
uint16_t msp_reported_unknown = 0;         // 같은 ID 를 반복해 찍지 않기 위해

// ---- 광류 위치·고도 유지 ----
FlowSensorSnapshot flowSensorSnapshot = {};
portMUX_TYPE flowSensorMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t mspLastPollUs = 0;
bool mspPollSeen = false;
// 직전 폴링이 바이트 예산을 다 쓰고도 FIFO 에 바이트를 남겼는가. 남았다면 이번에
// 꺼내는 프레임도 늦게 꺼낸 것이다(정체 직후 첫 폴링만 걸러서는 부족하다).
bool mspBacklog = false;

// 지상국 요청(Core 0 가 쓰고 Core 1 이 읽는다). failsafe 진입 시 Core 1 이 끈다.
volatile bool poshold_requested = false;
volatile bool althold_requested = false;
enum FlowCalCommand : uint8_t { FLOWCAL_CMD_NONE = 0, FLOWCAL_CMD_START, FLOWCAL_CMD_STOP };
enum FlowCalStateId : uint8_t { FLOWCAL_IDLE = 0, FLOWCAL_RUNNING, FLOWCAL_DONE_OK, FLOWCAL_DONE_FAILED };
volatile uint8_t flowcal_command = FLOWCAL_CMD_NONE;
volatile bool flowcal_running = false;     // Core 0 기준 요청 상태(start 거부용)
volatile bool flowcal_result_pending = false;
volatile bool imucal_request = false;      // Core 0 → Core 1 (flowRequestMux)
volatile bool imucal_running = false;      // Core 0 기준(start·flowcal 거부용)
volatile bool imucal_result_pending = false;
ImuCalResult imuCalResult = {};
ImuCalAccum imuCalAcc = {};                // Core 1 소유
portMUX_TYPE flowRequestMux = portMUX_INITIALIZER_UNLOCKED;

// 순수 조종 명령(트림 제외). RC 파서가 targetAngleX/Y 와 함께 쓴다.
volatile float pilotRollCmd = 0.0f, pilotPitchCmd = 0.0f;

// Core 1 소유. 큰 구조체는 pid_task 스택(4096 B) 밖에 둔다.
FlowOdomConfig flowOdomCfg = flowOdomDefaultConfig();
FlowOdomState flowOdomState;
FlowGyroHistory flowGyro;
HeightEstimator heightEst;
FlowCalibrator flowCalibrator;
FlowCalResult flowCalResult;
FlowAccelEstimator flowAccel;
// 직전 tick 에 위치 유지가 ACTIVE/COAST 였고 가속도 추정이 신선했는가.
// 참일 때만 상보필터의 가속도계 기울기에서 광류 가속도를 뺀다(flow_odometry.h).
bool flowAccelUsable = false;
const uint32_t FLOW_ACCEL_MAX_AGE_US = 350000;   // 프레임 하나 유실까지
PosHold posHold;
AltHold altHold;
PosHoldGains posHoldGains = posHoldDefaultGains();
AltHoldGains altHoldGains = altHoldDefaultGains();
uint32_t flowSeqSeen = 0, rangeSeqSeen = 0;
uint8_t flowcal_state = FLOWCAL_IDLE;
