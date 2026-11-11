#pragma once

#include <Arduino.h>

#include "mag_yaw_fusion.h"   // wrapDeg

// 바깥 yaw 루프의 모드와 heading setpoint.
// hold=false 인 동안 setpoint를 현재 heading으로 계속 슬레이빙하므로
// setpoint가 stale해질 수 없고, 어떤 전환도 정의상 bumpless다.
struct YawOuter {
  float target_angle_deg;
  bool  hold;
};

static inline YawOuter updateYawOuter(
    float target_yaw_rate_dps, float body_gz_dps, float angle_z_deg,
    float prev_target_angle_deg, bool prev_hold, bool override_hold,
    float rate_deadzone_dps, float settle_dps) {
  const bool stick_centered = fabsf(target_yaw_rate_dps) < rate_deadzone_dps;
  const bool settled = fabsf(body_gz_dps) < settle_dps;
  YawOuter out;
  // 스틱을 밀면 항상 rate 모드로 풀고, 중립에서는 기존 잠금을 유지하거나
  // 정착한 뒤 새로 잠근다. settled는 진입 게이트일 뿐 해제 조건이 아니다.
  out.hold =
      override_hold || (stick_centered && (prev_hold || settled));
  out.target_angle_deg = out.hold ? prev_target_angle_deg : angle_z_deg;
  return out;
}

static inline float yawTargetRateDps(
    const YawOuter &state, float target_yaw_rate_dps, float angle_z_deg,
    float kp_angle_yaw, float max_rate_dps) {
  const float raw = state.hold
      ? wrapDeg(state.target_angle_deg - angle_z_deg) * kp_angle_yaw
      : target_yaw_rate_dps;
  if (raw > max_rate_dps) return max_rate_dps;
  if (raw < -max_rate_dps) return -max_rate_dps;
  return raw;
}
