#pragma once

#include <math.h>
#include <stdint.h>

// [yaw 권한(authority) 감시 정리]
//  저스로틀에서는 모터 범위가 좁아 control_allocator가 yaw를 크게 줄인다 (yaw_scale ↓)
//  이때 heading 잠금이 계속 "원래 방향으로 돌아가라"고 명령하면 실제로는 못 돌면서 목표와의 오차만 커지고,
//  나중에 여유가 생기는 순간 쌓인 오차를 한꺼번에 따라가며 급회전(슬램)할 수 있다
//  그래서 yaw_scale이 오래 낮으면 heading 잠금을 내려놓고(LIMITED) "회전만 멈추기"(목표 각속도 0)로 낮춘다
//  상태 전이: NORMAL --(scale ≤ 0.5가 150ms 지속)--> LIMITED --(scale ≥ 0.9이고 회전 멈춤)--> RECOVERING
//             RECOVERING --(그 상태 500ms 유지)--> NORMAL,  RECOVERING --(조건 깨짐)--> LIMITED
//  시동 꺼짐 / 자동착륙 중 / 스틱을 민 상태면 즉시 NORMAL로 초기화한다

// 텔레메트리 필드(yaw_authority_state)로 숫자 그대로 나간다. 지상국 해석과 맞춰야 하므로 값을 바꾸면 안 된다
enum YawAuthorityState : int {
  YAW_AUTH_NORMAL = 0,       // 정상: heading 잠금/스틱 명령을 그대로 사용
  YAW_AUTH_LIMITED = 1,      // 권한 부족: 잠금 해제, 목표 = 현재 heading, 목표 각속도 0
  YAW_AUTH_RECOVERING = 2,   // 회복 확인 중: 동작은 LIMITED와 같고, 500ms 버티면 NORMAL
};

// 위 숫자가 바뀌면 컴파일을 실패시켜 텔레메트리 계약이 깨진 걸 알린다
static_assert(YAW_AUTH_NORMAL == 0, "yaw authority telemetry contract changed");
static_assert(YAW_AUTH_LIMITED == 1, "yaw authority telemetry contract changed");
static_assert(YAW_AUTH_RECOVERING == 2,
              "yaw authority telemetry contract changed");

static constexpr float YAW_AUTH_ENTRY_SCALE = 0.5f;      // yaw_scale이 이 이하면 "권한 부족" 후보
static constexpr uint32_t YAW_AUTH_ENTRY_MS = 150U;      // 후보 상태가 이만큼 계속돼야 LIMITED 진입 (순간 포화는 무시)
static constexpr float YAW_AUTH_RECOVERY_SCALE = 0.9f;   // yaw_scale이 이 이상이어야 회복 후보 (진입 0.5와 간격을 둬 깜빡임 방지)
static constexpr uint32_t YAW_AUTH_RECOVERY_MS = 500U;   // 회복 조건이 이만큼 유지돼야 NORMAL 복귀

// 상태 머신이 tick 사이에 기억해야 하는 값
struct YawAuthorityTracker {
  YawAuthorityState state;      // 현재 상태
  uint32_t entry_since_ms;      // 진입 후보(scale ≤ 0.5)가 시작된 시각
  uint32_t recovery_since_ms;   // RECOVERING에 들어간 시각
  bool entry_timing;            // 진입 후보 시간을 재는 중인가
  bool recovery_timing;         // 회복 시간을 재는 중인가
};

// applyYawAuthority()의 결과 = 이번 tick 실제로 쓸 yaw 바깥 루프 명령
struct YawAuthorityCommand {
  float target_angle_deg;   // 목표 heading (°) → targetAngleZ
  float target_rate_dps;    // 목표 yaw 각속도 (°/s) → 안쪽 각속도 PID로
  bool hold;                // heading 잠금 여부 → yaw_hold_now
};

// 모든 값을 처음 상태(NORMAL, 타이머 정지)로 되돌린다
static inline void resetYawAuthority(YawAuthorityTracker &tracker) {
  tracker.state = YAW_AUTH_NORMAL;
  tracker.entry_since_ms = 0U;
  tracker.recovery_since_ms = 0U;
  tracker.entry_timing = false;
  tracker.recovery_timing = false;
}

// 매 tick(1kHz) 호출해 상태를 한 단계 진행시키고 새 상태를 돌려준다
static inline YawAuthorityState updateYawAuthority(
    YawAuthorityTracker &tracker, float yaw_scale,
    float pilot_yaw_rate_dps, float body_gz_dps, bool armed,
    bool failsafe_active, bool safety_locked, uint32_t now_ms,
    float rate_deadzone_dps = 3.0f, float settle_dps = 10.0f) {
  // yaw_scale          : 직전 tick allocator가 yaw에 준 배율 (mixer_yaw_scale)
  // pilot_yaw_rate_dps : 스틱(rcr) yaw 각속도 명령. 중립인지 판단용
  // body_gz_dps        : 실제 yaw 각속도. 회전이 멈췄는지(회복 조건) 판단용
  // armed / failsafe_active / safety_locked : 하나라도 비행 정상 상태가 아니면 감시를 끄고 초기화
  // now_ms             : 현재 시각 (millis). 지속 시간 측정용
  // rate_deadzone_dps, settle_dps : yaw_command.h의 스틱 중립/정착 판정과 같은 기준 (YAW_RATE_DEADZONE, YAW_HOLD_SETTLE_DPS)
  const bool stick_centered =
      fabsf(pilot_yaw_rate_dps) < rate_deadzone_dps;
  // 조종자가 직접 yaw를 돌리는 중이면 잠금 자체가 없으므로 감시할 이유가 없다
  if (!armed || failsafe_active || safety_locked || !stick_centered) {
    resetYawAuthority(tracker);
    return tracker.state;
  }

  // [NORMAL] yaw_scale ≤ 0.5가 150ms 연속이면 LIMITED로. 중간에 한 번이라도 회복하면 타이머 리셋
  if (tracker.state == YAW_AUTH_NORMAL) {
    tracker.recovery_timing = false;
    if (yaw_scale <= YAW_AUTH_ENTRY_SCALE) {
      if (!tracker.entry_timing) {
        tracker.entry_timing = true;
        tracker.entry_since_ms = now_ms;
      } else if ((uint32_t)(now_ms - tracker.entry_since_ms) >=
                 YAW_AUTH_ENTRY_MS) {
        // (uint32_t) 뺄셈이라 millis()가 한 바퀴 돌아도(약 49일) 경과 시간이 맞게 나온다
        tracker.state = YAW_AUTH_LIMITED;
        tracker.entry_timing = false;
      }
    } else {
      tracker.entry_timing = false;
    }
    return tracker.state;
  }

  // 회복 조건: yaw 여유가 충분히 돌아왔고(≥0.9) 기체 회전도 거의 멈췄다(<10dps)
  const bool recovered =
      yaw_scale >= YAW_AUTH_RECOVERY_SCALE && fabsf(body_gz_dps) < settle_dps;
  // [LIMITED] 회복 조건을 만족하는 순간 RECOVERING으로 넘어가며 타이머 시작
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

  // [RECOVERING] 조건이 한 번이라도 깨지면 LIMITED로 되돌아가고, 500ms 버티면 NORMAL로 완전 복귀
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

  // 알 수 없는 상태값(메모리 오염 등)이면 안전하게 NORMAL로 초기화
  resetYawAuthority(tracker);
  return tracker.state;
}

// 상태에 따라 yaw 바깥 루프 명령을 고른다
//  NORMAL     : updateYawOuter()/yawTargetRateDps()가 만든 정상 명령을 그대로 통과
//  그 외 상태 : 목표 heading을 현재 heading으로 슬레이빙, 목표 각속도 0, 잠금 해제
//               → 못 따라갈 목표를 붙잡지 않고 "회전만 멈추기"로 낮춘다. NORMAL 복귀 시 그 자리에서 새로 잠긴다
static inline YawAuthorityCommand applyYawAuthority(
    YawAuthorityState state, float angle_z_deg,
    float normal_target_angle_deg, float normal_target_rate_dps,
    bool normal_hold) {
  if (state == YAW_AUTH_NORMAL) {
    return {normal_target_angle_deg, normal_target_rate_dps, normal_hold};
  }
  return {angle_z_deg, 0.0f, false};
}

// [조건부 적분(anti-windup)] 각 축 I항을 한 tick 갱신한 값을 돌려준다
//  current_integral_us : 지금까지 쌓인 I항 (µs)
//  delta_us            : 이번 tick 더할 양 = Ki × 오차 × dt (µs)
//  requested_output_us : 이번 tick PID 전체 출력 (µs). 포화 방향 판단용
//  axis_scale          : allocator가 이 축에 준 배율 (roll/pitch는 rp_scale, yaw는 yaw_scale)
//  throttle_us         : 현재 스로틀. 지상 여부 판단용
//  integral_limit_us   : I항 절댓값 상한 (I_TERM_MAX_US = 50µs)
static inline float conditionalAxisIntegral(
    float current_integral_us, float delta_us, float requested_output_us,
    float axis_scale, int throttle_us, float integral_limit_us) {
  // 1100µs 이하 = 땅에 있거나 이륙 전. 기체가 못 움직이는데 오차가 계속 쌓이면 이륙 순간 튀므로 0으로 비운다
  if (throttle_us <= 1100) return 0.0f;

  float next = current_integral_us;
  // 축이 포화(scale<1)인데 이번 적분이 출력을 같은 방향으로 더 키우는 경우만 적분을 멈춘다
  //  (requested × delta > 0 이면 둘의 부호가 같다 = 이미 잘리는 방향으로 더 미는 것)
  //  반대 방향(포화를 푸는 쪽)이나 포화가 없을 때는 정상적으로 쌓는다
  if (!(axis_scale < 1.0f && requested_output_us * delta_us > 0.0f)) {
    next += delta_us;
  }
  // 최종적으로 ±integral_limit_us로 클램프
  if (next > integral_limit_us) return integral_limit_us;
  if (next < -integral_limit_us) return -integral_limit_us;
  return next;
}
