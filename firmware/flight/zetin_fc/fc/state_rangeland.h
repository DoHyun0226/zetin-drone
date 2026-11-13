// zetin_fc/fc/state_rangeland.h — 거리 기반 자동착륙 상태, 스위치 우편함, 광류·IMU 보정 요청, 하네스 리셋.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// ---- 거리 기반 자동착륙(range_land.h). pid_task 가 쓰고 텔레메트리는 읽기만 한다. ----
// RC 가 끊길 때 거리가 신선하면 RL_MODE_RANGE, 아니면 지금의 시간 기반 하강
// (RL_MODE_TIMED). 거리 기반 하강 중 거리를 못 믿게 되면 RL_MODE_TIMED_FALLBACK.
// 단계 번호(fs_phase)는 둘 다 FS_DESCENDING 이다(공유 헤더 failsafe_land.h 불변).
static const int RL_NO_CAP = 100000;
static const float RL_RESUME_SLEW_US_S = ALTHOLD_SLEW_US_S;
RangeLandState rangeLand;
AltHold rlAltHold;
AltHoldGains rlAltHoldGains = altHoldDescentGains();
volatile uint8_t rl_mode = RL_MODE_NONE;
volatile uint8_t rl_stage = RL_STAGE_IDLE;
volatile uint8_t rl_cut_reason = RL_CUT_NONE;
volatile float rl_h_ref = 0.0f;
// 마지막 신뢰 거리 프레임(range_timing_ok). Core 0 지연 뒤 꺼낸 프레임은 증거로도
// 무효로도 쓰지 않는다.
int32_t rlRangeMm = -1;
uint32_t rlRangeTUs = 0;
bool rlRangeSeen = false;
RangeTrack rlPreTrack = {};   // 진입 전 추적(무장 중 매 신선한 신뢰 프레임)
uint32_t rlRangeMs = 0;       // 마지막 신뢰 프레임 시각(millis 축)
// 요청 = allocator 입력(base + 고도 보정), 전달 = allocator 출력 collective.
volatile int rl_requested_collective_us = 1000;
volatile float rl_delivered_collective_us = 1000.0f;
// 바닥 근접 뒤 요청 collective 래칫. 감쇠 중인 음수 보정이 0 쪽으로 돌아와도 오르지 않는다.
int rl_collective_cap = RL_NO_CAP;
// 이 에피소드의 기준 스로틀 상한. 폴백 때, 그리고 resume 인계 중 재진입 때 그 순간
// 요청 collective 로 고정한다(위로 튀지 않음).
int rl_base_ceiling = RL_NO_CAP;
// resume 뒤 기준 스로틀을 그 순간 collective 에서 호버까지 천천히 옮기는 중.
volatile bool rl_resume_handover = false;
float rlHandoverBase = 0.0f;
int rlHandoverSet = 0;
// 거리 기반 자동착륙 스위치. 실기 검증 전이라 부팅 기본은 OFF(시간 기반 하강만)이고
// 저장하지 않는다. Core 0 은 요청만 넣고, Core 1 이 시동 해제 상태에서만 적용한다 —
// 허용 판단과 적용을 RC 상실 진입과 같은 코어·같은 tick 순서에 두어, 비행 중(하강·
// resume 인계 포함)에는 값이 바뀌지 않는다(Codex 설계 검토 P2).
volatile bool rangeland_enabled = false;            // Core 1 만 쓴다
volatile int8_t rangeland_request = -1;             // Core 0 → Core 1 (flowRequestMux)

static inline void rangeLandResetFirmware() {
  rangeLand = RangeLandState{};
  altHoldReset(rlAltHold);
  rl_mode = RL_MODE_NONE;
  rl_stage = RL_STAGE_IDLE;
  rl_cut_reason = RL_CUT_NONE;
  rl_h_ref = 0.0f;
  rl_collective_cap = RL_NO_CAP;
  rl_base_ceiling = RL_NO_CAP;
  rl_resume_handover = false;
}

FlowHoldTelemetry flowHoldTelemetry = {};
portMUX_TYPE flowTelemetryMux = portMUX_INITIALIZER_UNLOCKED;

static inline void publishFlowHoldTelemetry(const FlowHoldTelemetry &t) {
  portENTER_CRITICAL(&flowTelemetryMux);
  flowHoldTelemetry = t;
  portEXIT_CRITICAL(&flowTelemetryMux);
}

static inline FlowHoldTelemetry readFlowHoldTelemetry() {
  portENTER_CRITICAL(&flowTelemetryMux);
  const FlowHoldTelemetry t = flowHoldTelemetry;
  portEXIT_CRITICAL(&flowTelemetryMux);
  return t;
}

// 부팅과 호스트 시험이 함께 쓰는 초기화. 컴파일 보정값이 유효하면 적용한다.
static inline void resetFlowHoldForHarness() {
  portENTER_CRITICAL(&flowSensorMux);
  flowSensorSnapshot = {};
  portEXIT_CRITICAL(&flowSensorMux);
  mspLastPollUs = 0;
  mspPollSeen = false;
  mspBacklog = false;
  poshold_requested = false;
  althold_requested = false;
  flowcal_command = FLOWCAL_CMD_NONE;
  flowcal_running = false;
  flowcal_result_pending = false;
  imucal_request = false;
  imucal_running = false;
  imucal_result_pending = false;
  imuCalResult = ImuCalResult();
  imuCalAcc = ImuCalAccum();
  pilotRollCmd = 0.0f;
  pilotPitchCmd = 0.0f;
  flowOdomCfg = flowOdomDefaultConfig();
  if (FLOW_CAL_VALID) {
    flowOdomSetCalibration(flowOdomCfg, FLOW_CAL_M, FLOW_CAL_DELAY_US);
  }
  flowOdomReset(flowOdomState);
  flowGyroReset(flowGyro);
  heightReset(heightEst);
  flowCalReset(flowCalibrator);
  flowCalResult = FlowCalResult();
  flowAccelReset(flowAccel);
  flowAccelUsable = false;
  posHoldReset(posHold);
  altHoldReset(altHold);
  posHoldGains = posHoldDefaultGains();
  altHoldGains = altHoldDefaultGains();
  flowSeqSeen = rangeSeqSeen = 0;
  flowcal_state = FLOWCAL_IDLE;
  rangeLandResetFirmware();
  rlRangeMm = -1;
  rlRangeTUs = 0;
  rlRangeSeen = false;
  rlPreTrack = RangeTrack{};
  rlRangeMs = 0;
  rl_requested_collective_us = 1000;
  rl_delivered_collective_us = 1000.0f;
  rangeland_enabled = false;
  rangeland_request = -1;
  publishFlowHoldTelemetry(FlowHoldTelemetry());
}

static inline FlowCalResult lastFlowCalResultForHarness() { return flowCalResult; }
static inline bool flowCalRunningForHarness() { return flowcal_running; }
static inline ImuCalResult lastImuCalResultForHarness() { return imuCalResult; }
static inline bool imuCalRunningForHarness() { return imucal_running; }

static inline bool takeImuCalRequest() {
  portENTER_CRITICAL(&flowRequestMux);
  const bool req = imucal_request;
  imucal_request = false;
  portEXIT_CRITICAL(&flowRequestMux);
  return req;
}

static inline uint8_t takeFlowCalCommand() {
  portENTER_CRITICAL(&flowRequestMux);
  const uint8_t cmd = flowcal_command;
  flowcal_command = FLOWCAL_CMD_NONE;
  portEXIT_CRITICAL(&flowRequestMux);
  return cmd;
}

static inline void requestRangeLand(int8_t enabled) {
  portENTER_CRITICAL(&flowRequestMux);
  rangeland_request = enabled;
  portEXIT_CRITICAL(&flowRequestMux);
}

static inline int8_t takeRangeLandRequest() {
  portENTER_CRITICAL(&flowRequestMux);
  const int8_t req = rangeland_request;
  rangeland_request = -1;
  portEXIT_CRITICAL(&flowRequestMux);
  return req;
}

// Core 1, tick 맨 앞(무장 요청 적용 전). 시동 해제 상태에서만 바꾼다. 하강·resume
// 인계는 무장 중에만 있지만 stop 직후 정리 전 tick 을 위해 같이 본다.
static inline void applyRangeLandRequest() {
  const int8_t req = takeRangeLandRequest();
  if (req < 0) return;
  if (!safety_lock || fs_phase == FS_DESCENDING || rl_resume_handover) {
    Serial.println(">>> RangeLand refused (armed)");
    return;
  }
  rangeland_enabled = (req == 1);
  Serial.println(rangeland_enabled ? ">>> RangeLand ON" : ">>> RangeLand OFF");
}

static inline void requestFlowCalCommand(uint8_t cmd) {
  portENTER_CRITICAL(&flowRequestMux);
  flowcal_command = cmd;
  portEXIT_CRITICAL(&flowRequestMux);
}

