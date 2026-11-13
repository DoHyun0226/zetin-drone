// zetin_fc/fc/flow_requests.h — 광류 텔레메트리 게시, 부팅·하네스 공용 리셋, 광류·IMU 보정·자동착륙 스위치 우편함.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
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

