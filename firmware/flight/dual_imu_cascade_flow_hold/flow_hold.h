#pragma once

#include <math.h>
#include <stdint.h>

// 광류 위치 유지(poshold)와 거리계 고도 유지(althold). 둘 다 1 kHz 로 매 tick
// 호출되고, 측정은 들어온 tick 에만 반영한다. 출력은 "후보"와 "적용"으로
// 나눈다 — 시동 해제 중에도 후보는 계산해 텔레메트리로 보여 주고(프롭 없이 손으로
// 움직여 부호를 확인하는 dry run), 적용은 항상 0 이다.
//
// 부호: 입력 c 는 flow_odometry.h 의 보정 공간 속도다. c 와 같은 부호의 자세
// 변화가 이동을 멈춘다(실기체 규약 +pitch = 기수 숙임에서 전방 이동은 c_pitch < 0
// 이고 pitch 감소 = 기수 들림이 멈춘다. 우측 이동은 c_roll < 0, 좌 기울임이 멈춘다).
// 그래서 오차는 e = c − c_t 이고 출력은 +K_v·e 다. 반대로 하면 드리프트를
// 가속한다(계획 검토 ②-1). 설계: docs/superpowers/plans/2026-10-29-flow-position-hold.md §4.3–4.4
//
// 센서가 끊기거나 조종자가 개입해도 출력은 계단으로 바뀌지 않는다. 위치 유지는
// 그때 **적분항까지 포함한 전체 보정을** 지수 감쇠시킨다 — 적분을 남겨 두면
// 외란이 사라진 뒤 조종 방향을 뒤집거나 센서 없이 계속 기울여 밀고 간다
// (계획 재검토 ②-P1). 고도 유지는 무효 중 적분을 천천히만 줄여, 배터리 처짐
// 보상이 한순간에 빠져 떨어지지 않게 한다.

static const float HOLD_DECAY_TAU_S = 0.1f;   // 0.3 s 에 95 %
static const float ALTHOLD_I_DECAY_TAU_S = 1.0f;
// 후보 출력 변화율 상한. 재진입(SUSPENDED→ACTIVE, ADJUST→ACTIVE)에서 제어항이 새로
// 계산될 때 계단이 생기지 않게 한다(재검토 ②-P2-5: 0.02°→1.83°, 한 tick 47.7 us).
// 위치 루프 시정수(≈1 s)·고도 루프(≈0.8 s)보다 충분히 빠르다.
static const float POSHOLD_SLEW_DEG_S = 10.0f;
static const float ALTHOLD_SLEW_US_S = 300.0f;

static inline float holdSlew(float prev, float target, float max_step) {
  const float d = target - prev;
  return prev + (d > max_step ? max_step : (d < -max_step ? -max_step : d));
}

// 감쇠 상태(조종 개입·센서 상실) 전용. 감쇠 목표가 출력보다 작으면 바로 따르고,
// 크면 출력을 키우지 않는다. 변화율 제한으로 감쇠를 늦추면 조종자 명령과 반대인
// 보정이 남는다(코드 재검토 P1). 감쇠가 아닌 상태(복구·측정 갱신)에는 쓰지 않는다 —
// 거기서 목표가 줄어드는 것은 계단이므로 양방향 제한을 건다(재검토 3회차).
static inline float holdFade(float prev, float target, float max_step) {
  if (prev * target > 0.0f) return fabsf(target) < fabsf(prev) ? target : prev;
  return holdSlew(prev, target, max_step);
}
static const float POSHOLD_COAST_S = 0.15f;   // 프레임 하나 빠짐까지는 ACTIVE
// 프레임 하나가 빠지면 유효 측정 간격이 300 ms 가 된다(빠진 구간 + 그 뒤 dt 로
// 거부되는 구간). 그것까지는 COAST 로 넘기고, 두 개 이상 빠지면 SUSPENDED.
static const float POSHOLD_SUSPEND_S = 0.35f;
static const float POSHOLD_OVERRIDE_HOLD_S = 0.05f;
static const float POSHOLD_PILOT_DEADBAND_DEG = 2.0f;
static const float POSHOLD_YAW_RATE_CMD_DPS = 3.0f;    // YAW_RATE_DEADZONE 와 같다
static const float POSHOLD_YAW_GYRO_DPS = 10.0f;       // YAW_HOLD_SETTLE_DPS 와 같다
static const float POSHOLD_YAW_CHANGE_DEG = 15.0f;

enum PosHoldStateId : uint8_t {
  POSHOLD_OFF = 0,
  POSHOLD_ACTIVE,
  POSHOLD_COAST,
  POSHOLD_SUSPENDED,
  POSHOLD_OVERRIDE,
};

struct PosHoldGains {
  float k_pos;        // 1/s       위치 오차 → 목표 속도
  float k_v;          // deg/(m/s) 속도 오차 → 자세
  float k_vi;         // deg/(m/s·s)
  float v_max_ms;
  float i_max_deg;
  float out_max_deg;
  float leash_m;
  float vel_tau_s;
};

static inline PosHoldGains posHoldDefaultGains() {
  // 1° 기울기 ≈ 0.171 m/s² → 속도 루프 시정수 1/(0.171·6) ≈ 1.0 s,
  // 위치 루프 2 s. 값은 SIL 격자 탐색(K_v 3~8, K_vi 0.3~2, K_pos 0.3~0.8,
  // LPF 0.1/0.2 s)에서 저항 0.05/0.3 N/(m/s)·1.5 m·30 ms 지연 세 조건의 최악값이
  // 좋은 영역 가운데를 골랐다(늦은 구간 속도 RMS ≤ 0.027 m/s). k_pos 는 2026-10-30 테더
  // 비행에서 0.5 → 0.8 로 올렸다(SIL: 60 ms 추가 지연까지 안정, 100 ms 에서 실패).
  PosHoldGains g;
  g.k_pos = 0.8f;
  g.k_v = 6.0f;
  g.k_vi = 1.0f;
  g.v_max_ms = 0.3f;
  g.i_max_deg = 5.0f;
  g.out_max_deg = 8.0f;
  g.leash_m = 1.0f;
  g.vel_tau_s = 0.1f;
  return g;
}

struct PosHold {
  uint8_t state;
  bool started;
  bool prev_enabled;
  bool have_filter;
  float p[2];        // 보정 공간 변위, m (기체 좌표, yaw 회전 미적용)
  float p_ref[2];
  float c_f[2];
  float i[2];        // deg
  float prop[2];     // deg
  float since_meas_s;
  float since_override_s;
  float yaw_ref_deg;
  float out[2];      // 변화율 제한을 거친 후보 출력, deg
};

struct PosHoldInput {
  bool requested = false;
  bool enabled = false;      // 시동 중이고 failsafe 가 아님
  bool calibrated = false;
  bool has_meas = false;     // 이번 tick 에 새 광류 구간이 있다
  bool meas_ok = false;      // 그 구간이 위치 유지에 쓸 수 있다
  float c_roll = 0.0f, c_pitch = 0.0f, meas_dt_s = 0.0f;
  float meas_span_s = 0.0f;  // 위치 적분 구간(프레임 유실 시 dt 보다 길다). 0 이면 dt
  float pilot_roll_deg = 0.0f, pilot_pitch_deg = 0.0f;  // 트림 제외 조종 명령
  float yaw_rate_cmd_dps = 0.0f;
  float gyro_z_dps = 0.0f;
  float yaw_deg = 0.0f;
};

struct PosHoldOutput {
  uint8_t state = POSHOLD_OFF;
  float candidate_roll_deg = 0.0f, candidate_pitch_deg = 0.0f;
  float applied_roll_deg = 0.0f, applied_pitch_deg = 0.0f;
};

static inline float holdWrapDeg(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

static inline float holdClamp(float v, float lim) {
  return v > lim ? lim : (v < -lim ? -lim : v);
}

static inline void posHoldReset(PosHold &ph) {
  ph.state = POSHOLD_OFF;
  ph.started = false;
  ph.prev_enabled = false;
  ph.have_filter = false;
  for (int k = 0; k < 2; k++) {
    ph.p[k] = ph.p_ref[k] = ph.c_f[k] = ph.i[k] = ph.prop[k] = ph.out[k] = 0.0f;
  }
  ph.since_meas_s = 1e9f;
  ph.since_override_s = 1e9f;
  ph.yaw_ref_deg = 0.0f;
}

static inline PosHoldOutput posHoldStep(PosHold &ph, const PosHoldGains &g,
                                        const PosHoldInput &in, float dt_s) {
  PosHoldOutput out;
  if (!in.requested || !in.calibrated) {
    posHoldReset(ph);
    ph.prev_enabled = in.enabled;
    return out;
  }
  if (!ph.started || in.enabled != ph.prev_enabled) {
    // 요청 직후, 그리고 시동·해제·failsafe 경계마다 처음부터 다시 잡는다.
    posHoldReset(ph);
    ph.started = true;
    ph.prev_enabled = in.enabled;
    ph.state = POSHOLD_SUSPENDED;     // 첫 유효 측정에서 ACTIVE 가 된다
    ph.yaw_ref_deg = in.yaw_deg;
  }

  const bool meas = in.has_meas && in.meas_ok && in.meas_dt_s > 0.0f;
  if (meas) {
    const float span = in.meas_span_s > 0.0f ? in.meas_span_s : in.meas_dt_s;
    ph.since_meas_s = 0.0f;
    ph.p[0] += in.c_roll * span;
    ph.p[1] += in.c_pitch * span;
  } else {
    ph.since_meas_s += dt_s;
  }

  const bool overrideNow =
      fabsf(in.pilot_roll_deg) > POSHOLD_PILOT_DEADBAND_DEG ||
      fabsf(in.pilot_pitch_deg) > POSHOLD_PILOT_DEADBAND_DEG ||
      fabsf(in.yaw_rate_cmd_dps) > POSHOLD_YAW_RATE_CMD_DPS ||
      fabsf(in.gyro_z_dps) > POSHOLD_YAW_GYRO_DPS ||
      fabsf(holdWrapDeg(in.yaw_deg - ph.yaw_ref_deg)) > POSHOLD_YAW_CHANGE_DEG;
  ph.since_override_s = overrideNow ? 0.0f : ph.since_override_s + dt_s;

  const float decay = expf(-dt_s / HOLD_DECAY_TAU_S);
  if (ph.since_override_s <= POSHOLD_OVERRIDE_HOLD_S) {
    // 조종자·yaw 회전 중: 위치·heading 기준을 현재로 계속 옮기고 보정은 0 으로.
    // yaw 부호가 미검증이라 크기 비교만 쓰고 위치에 yaw 회전을 적용하지 않는다.
    ph.state = POSHOLD_OVERRIDE;
    ph.p_ref[0] = ph.p[0];
    ph.p_ref[1] = ph.p[1];
    ph.yaw_ref_deg = in.yaw_deg;
    for (int k = 0; k < 2; k++) {
      // 감쇠는 지금 실제 출력에서 시작한다. 내부 합계가 출력과 다르면(포화, 또는
      // 복구 중 변화율 제한으로 출력이 뒤처진 상태) 그 차이가 한 번에 빠지거나 상한에
      // 머문다(코드 재검토 4회차). 그래서 비례항을 출력에 맞춘 뒤 줄인다.
      ph.prop[k] = ph.out[k] - ph.i[k];
      ph.prop[k] *= decay;
      ph.i[k] *= decay;
    }
  } else if (meas) {
    if (ph.state != POSHOLD_ACTIVE && ph.state != POSHOLD_COAST) {
      // 무효·override 에서 돌아올 때 현재 위치를 새 기준으로 잡는다.
      ph.p_ref[0] = ph.p[0];
      ph.p_ref[1] = ph.p[1];
      ph.have_filter = false;
    }
    ph.state = POSHOLD_ACTIVE;
    const float c[2] = {in.c_roll, in.c_pitch};
    const float a = in.meas_dt_s / (g.vel_tau_s + in.meas_dt_s);
    for (int k = 0; k < 2; k++) {
      ph.c_f[k] = ph.have_filter ? ph.c_f[k] + (c[k] - ph.c_f[k]) * a : c[k];
      // 리시: 밀린 거리가 너무 크면 기준을 끌고 와 되돌아가는 시간을 제한한다.
      const float err = ph.p_ref[k] - ph.p[k];
      if (err > g.leash_m) ph.p_ref[k] = ph.p[k] + g.leash_m;
      if (err < -g.leash_m) ph.p_ref[k] = ph.p[k] - g.leash_m;
      const float ct = holdClamp(g.k_pos * (ph.p_ref[k] - ph.p[k]), g.v_max_ms);
      const float e = ph.c_f[k] - ct;
      ph.prop[k] = g.k_v * e;
      // 조건부 적분: 출력이 포화돼 있고 적분이 포화를 더 키우는 방향이면 멈춘다.
      const float di = g.k_vi * e * in.meas_dt_s;
      const bool saturated = fabsf(ph.prop[k] + ph.i[k]) >= g.out_max_deg;
      if (!saturated || di * ph.i[k] < 0.0f) {
        ph.i[k] = holdClamp(ph.i[k] + di, g.i_max_deg);
      }
    }
    ph.have_filter = true;
  } else if (ph.since_meas_s > POSHOLD_SUSPEND_S) {
    ph.state = POSHOLD_SUSPENDED;
    for (int k = 0; k < 2; k++) {
      // 감쇠는 지금 실제 출력에서 시작한다. 내부 합계가 출력과 다르면(포화, 또는
      // 복구 중 변화율 제한으로 출력이 뒤처진 상태) 그 차이가 한 번에 빠지거나 상한에
      // 머문다(코드 재검토 4회차). 그래서 비례항을 출력에 맞춘 뒤 줄인다.
      ph.prop[k] = ph.out[k] - ph.i[k];
      ph.prop[k] *= decay;
      ph.i[k] *= decay;
    }
  } else if (ph.state == POSHOLD_ACTIVE && ph.since_meas_s > POSHOLD_COAST_S) {
    ph.state = POSHOLD_COAST;          // 마지막 출력을 유지한다
  }

  out.state = ph.state;
  const float slew = POSHOLD_SLEW_DEG_S * dt_s;
  const bool fading =
      ph.state == POSHOLD_OVERRIDE || ph.state == POSHOLD_SUSPENDED;
  for (int k = 0; k < 2; k++) {
    const float target = holdClamp(ph.prop[k] + ph.i[k], g.out_max_deg);
    ph.out[k] = fading ? holdFade(ph.out[k], target, slew)
                       : holdSlew(ph.out[k], target, slew);
  }
  out.candidate_roll_deg = ph.out[0];
  out.candidate_pitch_deg = ph.out[1];
  if (in.enabled) {
    out.applied_roll_deg = out.candidate_roll_deg;
    out.applied_pitch_deg = out.candidate_pitch_deg;
  }
  return out;
}

// ---------------- 고도 유지 ----------------

static const float ALTHOLD_SUSPEND_S = 0.30f;
static const float ALTHOLD_ADJUST_SETTLE_S = 0.50f;

enum AltHoldStateId : uint8_t {
  ALTHOLD_OFF = 0,
  ALTHOLD_ACTIVE,
  ALTHOLD_ADJUST,     // 조종자가 스로틀을 바꾸는 중
  ALTHOLD_SUSPENDED,
};

struct AltHoldGains {
  float k_h;          // us/m
  float k_vz;         // us/(m/s)
  float k_i;          // us/(m·s)
  float i_max_us;
  float out_max_us;
};

static inline AltHoldGains altHoldDefaultGains() {
  // SIL 기체 1 us ≈ 0.0289 m/s² 에서 ω_n ≈ 1.32 rad/s, ζ ≈ 0.66.
  // 실기 추력 계수는 미측정이라 보수적으로 둔다. ±60 us < CTRL_MARGIN 150 us.
  AltHoldGains g;
  g.k_h = 60.0f;
  g.k_vz = 60.0f;
  g.k_i = 15.0f;
  g.i_max_us = 40.0f;
  g.out_max_us = 60.0f;
  return g;
}

struct AltHold {
  uint8_t state;
  bool started;
  bool prev_enabled;
  int ref_throttle;
  float h_ref;
  float i;
  float pd;
  float since_change_s;
  float since_valid_s;
  float out;         // 변화율 제한을 거친 후보 출력, us
};

struct AltHoldInput {
  bool requested = false;
  bool enabled = false;
  bool height_valid = false;
  float h_m = 0.0f;
  float vz_ms = 0.0f;
  int base_throttle = 1000;
  // 참이면 보정을 서서히 0 으로 보내고 쉰다(리셋하지 않는다). 펌웨어는 base 가
  // RC 지상 컷 경계 근처로 내려가면 이것을 켠다.
  bool inhibit = false;
  // 참이면 그 tick 부터 출력 0, 내부 상태(적분·변화율 제한)도 0. 펌웨어는 base 가
  // RC 지상 컷 경계 이하일 때 켠다. 내부 출력을 남기면 1 us 만 올려도 그 값이
  // 한 번에 되살아난다(코드 재검토 P2).
  bool hard_zero = false;
};

struct AltHoldOutput {
  uint8_t state = ALTHOLD_OFF;
  float candidate_us = 0.0f;
  float applied_us = 0.0f;
};

static inline void altHoldReset(AltHold &ah) {
  ah.state = ALTHOLD_OFF;
  ah.started = false;
  ah.prev_enabled = false;
  ah.ref_throttle = 0;
  ah.h_ref = 0.0f;
  ah.i = 0.0f;
  ah.pd = 0.0f;
  ah.since_change_s = 1e9f;
  ah.since_valid_s = 0.0f;
  ah.out = 0.0f;
}

static inline AltHoldOutput altHoldStep(AltHold &ah, const AltHoldGains &g,
                                        const AltHoldInput &in, float dt_s) {
  AltHoldOutput out;
  if (!in.requested) {
    altHoldReset(ah);
    ah.prev_enabled = in.enabled;
    return out;
  }
  if (!ah.started || in.enabled != ah.prev_enabled) {
    altHoldReset(ah);
    ah.started = true;
    ah.prev_enabled = in.enabled;
    ah.ref_throttle = in.base_throttle;
    ah.h_ref = in.h_m;
    ah.state = in.height_valid ? ALTHOLD_ACTIVE : ALTHOLD_SUSPENDED;
  }

  ah.since_valid_s = in.height_valid ? 0.0f : ah.since_valid_s + dt_s;
  if (in.base_throttle != ah.ref_throttle) {
    // 기준 스로틀을 갱신해야 한 번 바꾼 뒤 매 tick 재목표가 반복되지 않는다
    // (반복되면 높이 복원항이 0 으로 남아 지속 상승한다, 검토 ②-5).
    ah.ref_throttle = in.base_throttle;
    ah.since_change_s = 0.0f;
    ah.state = ALTHOLD_ADJUST;
  } else {
    ah.since_change_s += dt_s;
  }

  const float decay = expf(-dt_s / HOLD_DECAY_TAU_S);
  if (in.hard_zero) {
    ah.state = ALTHOLD_SUSPENDED;
    ah.pd = 0.0f;
    ah.i = 0.0f;
    ah.out = 0.0f;
    if (in.height_valid) ah.h_ref = in.h_m;
    out.state = ah.state;
    return out;
  }
  if (in.inhibit) {
    ah.state = ALTHOLD_SUSPENDED;
    ah.pd = ah.out - ah.i;          // 감쇠는 지금 실제 출력에서 시작한다
    ah.pd *= decay;
    ah.i *= decay;
  } else if (ah.since_valid_s > ALTHOLD_SUSPEND_S) {
    // 센서 만료가 ADJUST 보다 먼저다. ADJUST 안에서 거리가 끊기면 종료 조건도 영영
    // 안 맞아 적분 보정이 무기한 남는다(코드 재검토 5회차).
    ah.state = ALTHOLD_SUSPENDED;
    ah.pd = ah.out - ah.i;          // 감쇠는 지금 실제 출력에서 시작한다
    ah.pd *= decay;
    ah.i *= expf(-dt_s / ALTHOLD_I_DECAY_TAU_S);
  } else if (ah.state == ALTHOLD_ADJUST) {
    // 진입 순간 출력은 직전 값에서 이어진다(pd 를 0 으로 바꾸지 않고 감쇠).
    // ±1 us 정밀 버튼 한 번이 수십 us 계단이 되면 안 된다(재검토 ②-P2-4).
    ah.pd *= decay;
    if (in.height_valid) ah.h_ref = in.h_m;
    if (ah.since_change_s >= ALTHOLD_ADJUST_SETTLE_S && in.height_valid) {
      ah.state = ALTHOLD_ACTIVE;
    }
  } else if (ah.state == ALTHOLD_SUSPENDED && in.height_valid) {
    ah.h_ref = in.h_m;
    ah.state = ALTHOLD_ACTIVE;
  }

  if (ah.state == ALTHOLD_ACTIVE && in.height_valid) {
    const float e = ah.h_ref - in.h_m;
    ah.pd = g.k_h * e - g.k_vz * in.vz_ms;
    const float di = g.k_i * e * dt_s;
    const bool saturated = fabsf(ah.pd + ah.i) >= g.out_max_us;
    if (!saturated || di * ah.i < 0.0f) {
      ah.i = holdClamp(ah.i + di, g.i_max_us);
    }
  }
  // ACTIVE 인데 높이가 잠깐(≤ 300 ms) 무효면 마지막 출력을 유지한다.

  out.state = ah.state;
  const float target = holdClamp(ah.pd + ah.i, g.out_max_us);
  const float step = ALTHOLD_SLEW_US_S * dt_s;
  ah.out = (ah.state == ALTHOLD_SUSPENDED) ? holdFade(ah.out, target, step)
                                           : holdSlew(ah.out, target, step);
  out.candidate_us = ah.out;
  if (in.enabled) out.applied_us = out.candidate_us;
  return out;
}
