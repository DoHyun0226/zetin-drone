// zetin_fc/fc/control_support.h — 자세 추정 적응 alpha, 잠금 진입, allocation 텔레메트리 게시·읽기.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
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

static inline void resetAllocationTelemetry() {
  portENTER_CRITICAL(&allocationTelemetryMux);
  mixer_scaled = false;
  mixer_yaw_scale = 1.0f;
  allocationTelemetry = {
      false, 1.0f, 1.0f, 1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      YAW_AUTH_NORMAL, 1000, fs_phase, rl_mode, rl_stage, rl_cut_reason, rl_h_ref};
  portEXIT_CRITICAL(&allocationTelemetryMux);
}

static inline void publishAllocationTelemetry(
    const MotorMix &mix, float pidRoll, float pidPitch, float pidYaw,
    float iRoll, float iPitch, float iYaw, int yawAuthorityState,
    int requestedUs) {
  const AllocationTelemetrySample sample = {
      mix.scaled, mix.rp_scale, mix.yaw_scale, mix.collective_us,
      pidRoll, pidPitch, pidYaw, iRoll, iPitch, iYaw, yawAuthorityState,
      requestedUs, fs_phase, rl_mode, rl_stage, rl_cut_reason, rl_h_ref};
  portENTER_CRITICAL(&allocationTelemetryMux);
  mixer_scaled = sample.scaled;
  allocationTelemetry = sample;
  portEXIT_CRITICAL(&allocationTelemetryMux);
}

static inline AllocationTelemetrySample readAllocationTelemetrySnapshot() {
  portENTER_CRITICAL(&allocationTelemetryMux);
  const AllocationTelemetrySample sample = allocationTelemetry;
  portEXIT_CRITICAL(&allocationTelemetryMux);
  return sample;
}

