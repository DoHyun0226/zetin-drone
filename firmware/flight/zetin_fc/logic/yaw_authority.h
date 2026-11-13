#pragma once

#include <math.h>
#include <stdint.h>

enum YawAuthorityState : int {
  YAW_AUTH_NORMAL = 0,
  YAW_AUTH_LIMITED = 1,
  YAW_AUTH_RECOVERING = 2,
};

static_assert(YAW_AUTH_NORMAL == 0, "yaw authority telemetry contract changed");
static_assert(YAW_AUTH_LIMITED == 1, "yaw authority telemetry contract changed");
static_assert(YAW_AUTH_RECOVERING == 2,
              "yaw authority telemetry contract changed");

static constexpr float YAW_AUTH_ENTRY_SCALE = 0.5f;
static constexpr uint32_t YAW_AUTH_ENTRY_MS = 150U;
static constexpr float YAW_AUTH_RECOVERY_SCALE = 0.9f;
static constexpr uint32_t YAW_AUTH_RECOVERY_MS = 500U;

struct YawAuthorityTracker {
  YawAuthorityState state;
  uint32_t entry_since_ms;
  uint32_t recovery_since_ms;
  bool entry_timing;
  bool recovery_timing;
};

struct YawAuthorityCommand {
  float target_angle_deg;
  float target_rate_dps;
  bool hold;
};

static inline void resetYawAuthority(YawAuthorityTracker &tracker) {
  tracker.state = YAW_AUTH_NORMAL;
  tracker.entry_since_ms = 0U;
  tracker.recovery_since_ms = 0U;
  tracker.entry_timing = false;
  tracker.recovery_timing = false;
}

static inline YawAuthorityState updateYawAuthority(
    YawAuthorityTracker &tracker, float yaw_scale,
    float pilot_yaw_rate_dps, float body_gz_dps, bool armed,
    bool failsafe_active, bool safety_locked, uint32_t now_ms,
    float rate_deadzone_dps = 3.0f, float settle_dps = 10.0f) {
  const bool stick_centered =
      fabsf(pilot_yaw_rate_dps) < rate_deadzone_dps;
  if (!armed || failsafe_active || safety_locked || !stick_centered) {
    resetYawAuthority(tracker);
    return tracker.state;
  }

  if (tracker.state == YAW_AUTH_NORMAL) {
    tracker.recovery_timing = false;
    if (yaw_scale <= YAW_AUTH_ENTRY_SCALE) {
      if (!tracker.entry_timing) {
        tracker.entry_timing = true;
        tracker.entry_since_ms = now_ms;
      } else if ((uint32_t)(now_ms - tracker.entry_since_ms) >=
                 YAW_AUTH_ENTRY_MS) {
        tracker.state = YAW_AUTH_LIMITED;
        tracker.entry_timing = false;
      }
    } else {
      tracker.entry_timing = false;
    }
    return tracker.state;
  }

  const bool recovered =
      yaw_scale >= YAW_AUTH_RECOVERY_SCALE && fabsf(body_gz_dps) < settle_dps;
  if (tracker.state == YAW_AUTH_LIMITED) {
    tracker.entry_timing = false;
    if (recovered) {
      tracker.state = YAW_AUTH_RECOVERING;
      tracker.recovery_timing = true;
      tracker.recovery_since_ms = now_ms;
    } else {
      tracker.recovery_timing = false;
    }
    return tracker.state;
  }

  if (tracker.state == YAW_AUTH_RECOVERING) {
    if (!recovered) {
      tracker.state = YAW_AUTH_LIMITED;
      tracker.recovery_timing = false;
    } else if (tracker.recovery_timing &&
               (uint32_t)(now_ms - tracker.recovery_since_ms) >=
                   YAW_AUTH_RECOVERY_MS) {
      resetYawAuthority(tracker);
    }
    return tracker.state;
  }

  resetYawAuthority(tracker);
  return tracker.state;
}

static inline YawAuthorityCommand applyYawAuthority(
    YawAuthorityState state, float angle_z_deg,
    float normal_target_angle_deg, float normal_target_rate_dps,
    bool normal_hold) {
  if (state == YAW_AUTH_NORMAL) {
    return {normal_target_angle_deg, normal_target_rate_dps, normal_hold};
  }
  return {angle_z_deg, 0.0f, false};
}

static inline float conditionalAxisIntegral(
    float current_integral_us, float delta_us, float requested_output_us,
    float axis_scale, int throttle_us, float integral_limit_us) {
  if (throttle_us <= 1100) return 0.0f;

  float next = current_integral_us;
  if (!(axis_scale < 1.0f && requested_output_us * delta_us > 0.0f)) {
    next += delta_us;
  }
  if (next > integral_limit_us) return integral_limit_us;
  if (next < -integral_limit_us) return -integral_limit_us;
  return next;
}
