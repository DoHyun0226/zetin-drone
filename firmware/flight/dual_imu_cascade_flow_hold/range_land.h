#pragma once

#include <math.h>
#include <stdint.h>

// 거리계 기반 자동착륙(flow-hold 스케치 전용). RC 가 끊기면 목표 고도를 일정 속도로
// 낮추며 내려오고, 바닥 근접(원값 < 10 cm)에서 기준 스로틀을 천천히 줄여 접촉을
// 확인한 뒤 착지 컷한다. 거리값을 못 믿게 되면 바닥 근접 전에는 시간 기반 하강으로
// 넘기고(RL_FALLBACK), 바닥 근접 뒤에는 그 자리에서 컷한다(RL_CUT_NEAR_GROUND).
// 1 kHz 로 매 tick 부르며, 펌웨어 상태는 모른다 — 연결은 .ino 가 한다.
//
// 거리 판정은 **채택된 원값**만 본다. 마지막 채택값에서 명령 하강만큼 내려간 예측값과
// 가까운 양수 프레임만 채택하고, 크게 뛴 값은 오래 유지돼도 기준이 되지 않는다.
// 거부가 이어지는 동안에는 허용폭도 예측 중심도 고정이다. 그러지 않으면 높은 곳에서
// 손·풀·테더 줄이 센서 앞에 와 만든 근접값이 몇 단계에 걸쳐 착지로 이어진다
// (계획 Codex 재검토 2-1~2-4).
//
// 설계: docs/superpowers/specs/2026-11-03-range-autoland-design.md
// 계획(상수 근거): docs/superpowers/plans/2026-11-03-range-autoland.md

static const float RL_DESCENT_FAST_MPS = 0.30f;
static const float RL_DESCENT_SLOW_MPS = 0.12f;
static const float RL_APPROACH_M = 0.30f;
static const float RL_REF_LEAD_M = 0.15f;
static const int32_t RL_NEAR_GROUND_MM = 100;     // FLOW_RANGE_MIN_M 0.10 m
// 실측 착지 원값 18–37 mm, 중앙값 29–31 mm(2026-11-03 벤치, 약 6천 샘플)
static const int32_t RL_LANDED_MIN_MM = 10;
static const int32_t RL_LANDED_MAX_MM = 60;
static const uint32_t RL_CONTINUITY_MS = 1000;
static const float RL_MAX_STEP_M = 0.05f;
static const float RL_MAX_VZ_MPS = 0.6f;
static const float RL_CONTACT_DROP_US = 40.0f;
static const uint32_t RL_LANDED_HOLD_MS = 400;
static const float RL_TOUCHDOWN_RAMP_US_S = 100.0f;
static const uint32_t RL_TOUCHDOWN_MAX_MS = 3000;
static const uint32_t RL_HOLD_FADE_MS = 500;      // 바닥 근접 뒤 위치 유지 감쇠
static const uint32_t RL_INVALID_GRACE_MS = 300;  // ALTHOLD_SUSPEND_S 와 같은 성격
static const uint32_t RL_MAX_MS = 12000;

// 텔레메트리 번호다. 뒤에만 덧붙인다.
enum RangeLandMode : uint8_t {
  RL_MODE_NONE = 0,
  RL_MODE_RANGE = 1,
  RL_MODE_TIMED = 2,
  RL_MODE_TIMED_FALLBACK = 3,
};

enum RangeLandStage : uint8_t {
  RL_STAGE_IDLE = 0,
  RL_STAGE_DESCENT = 1,
  RL_STAGE_APPROACH = 2,
  RL_STAGE_NEAR_GROUND = 3,
  RL_STAGE_CONTACT = 4,      // 기준 스로틀 감소가 접촉 확인 문턱에 닿음
};

enum RangeLandCutReason : uint8_t {
  RL_CUT_NONE = 0,
  RL_CUT_CONTACT = 1,
  RL_CUT_NEAR_GROUND_LOSS = 2,
  RL_CUT_NEAR_GROUND_STUCK = 3,
  RL_CUT_BACKSTOP = 4,
};

enum RangeLandAction : uint8_t {
  RL_CONTINUE = 0,
  RL_LANDED,
  RL_TIMEOUT,
  RL_FALLBACK,
  RL_CUT_NEAR_GROUND,
};

struct RangeLandState {
  uint8_t stage;
  float h_ref;
  uint32_t enter_ms;
  bool have_low;
  uint32_t last_low_ms;      // 마지막으로 채택한 원값 ≤ 접근 높이의 시각
  uint32_t near_ground_since_ms;
  bool landed_timing;
  uint32_t landed_since_ms;
  bool invalid_timing;
  uint32_t invalid_since_ms;
  // 추적 기준. tol_mm·pred_mm 은 채택 뒤 첫 신뢰 프레임이 올 때 한 번 정해진다.
  int32_t ref_mm;
  uint32_t ref_ms;
  bool tol_fixed;
  int32_t tol_mm;
  int32_t pred_mm;
  bool rejecting;            // 마지막 신뢰 프레임이 거부됐다
  uint16_t reject_count;
  float drop_us;
};

struct RangeLandInput {
  bool height_fresh = false;
  float h_m = 0.0f;
  int32_t range_mm = -1;             // 마지막 **신뢰** 프레임의 원값
  bool range_fresh = false;          // 그 프레임의 나이 ≤ FLOW_HEIGHT_MAX_AGE_US
  bool range_trusted_new = false;    // 이번 tick 에 새 신뢰 프레임이 왔다
  float hover_us = 0.0f;
  float delivered_collective_us = 0.0f;   // 직전 tick allocator 출력 collective
  float dt_s = 0.0f;
  uint32_t now_ms = 0;
};

struct RangeLandOutput {
  uint8_t stage = RL_STAGE_IDLE;
  RangeLandAction action = RL_CONTINUE;
  uint8_t cut_reason = RL_CUT_NONE;
  float h_ref = 0.0f;
  float vz_ref = 0.0f;
  float drop_us = 0.0f;
  float hold_scale = 0.0f;
};

static inline int32_t rangeLandApproachMm() {
  return (int32_t)lroundf(RL_APPROACH_M * 1000.0f);
}

// 단계의 목표 하강 속도(예측에도 쓴다). 바닥 근접 뒤에는 0 — 기준 스로틀 감소로
// 내려가며, 예측 중심은 마지막 채택값에 머문다.
static inline float rangeLandStageSpeed(uint8_t stage) {
  if (stage == RL_STAGE_DESCENT) return RL_DESCENT_FAST_MPS;
  if (stage == RL_STAGE_APPROACH) return RL_DESCENT_SLOW_MPS;
  return 0.0f;
}

static inline void rangeLandStart(RangeLandState &st, float h_now,
                                  int32_t range_mm_now, uint32_t now_ms) {
  st = RangeLandState{};
  st.enter_ms = now_ms;
  st.h_ref = h_now;
  st.ref_mm = range_mm_now;
  st.ref_ms = now_ms;
  st.stage = RL_STAGE_DESCENT;
  if (range_mm_now <= rangeLandApproachMm()) {
    st.stage = RL_STAGE_APPROACH;
    st.have_low = true;
    st.last_low_ms = now_ms;
  }
}

static inline RangeLandOutput rangeLandStep(RangeLandState &st,
                                            const RangeLandInput &in) {
  RangeLandOutput out;
  const uint32_t now = in.now_ms;

  // ---- 추적: 새 신뢰 프레임 하나를 채택하거나 거부한다 ----
  // 늦게 소비돼 이미 신선하지 않은 프레임은 증거로 쓰지 않는다. 공백이 길수록 허용폭이
  // 넓어 공간 조건을 통과할 수 있다(Codex 코드 검토 P1). 무효는 !range_fresh 가 센다.
  bool accepted = false;
  if (in.range_trusted_new && in.range_fresh) {
    if (!st.tol_fixed) {
      const float gap_ms = (float)(now - st.ref_ms);
      // m/s × ms = mm
      st.tol_mm = (int32_t)lroundf(RL_MAX_STEP_M * 1000.0f + RL_MAX_VZ_MPS * gap_ms);
      st.pred_mm = st.ref_mm -
                   (int32_t)lroundf(rangeLandStageSpeed(st.stage) * gap_ms);
      st.tol_fixed = true;
    }
    const int32_t raw = in.range_mm;
    if (raw > 0) {
      const int64_t err = (int64_t)raw - (int64_t)st.pred_mm;
      accepted = (err < 0 ? -err : err) <= (int64_t)st.tol_mm;
    }
    // 바닥 근접 진입은 바로 전에 낮은 높이를 지나왔을 때만이다.
    const bool low_recent = st.have_low && now - st.last_low_ms <= RL_CONTINUITY_MS;
    if (accepted && st.stage < RL_STAGE_NEAR_GROUND &&
        raw < RL_NEAR_GROUND_MM && !low_recent) {
      accepted = false;
    }
    if (accepted) {
      st.ref_mm = raw;
      st.ref_ms = now;
      st.tol_fixed = false;
      st.rejecting = false;
      if (raw <= rangeLandApproachMm()) {
        st.have_low = true;
        st.last_low_ms = now;
        if (st.stage == RL_STAGE_DESCENT) st.stage = RL_STAGE_APPROACH;
      }
      if (st.stage < RL_STAGE_NEAR_GROUND && raw < RL_NEAR_GROUND_MM) {
        st.stage = RL_STAGE_NEAR_GROUND;
        st.near_ground_since_ms = now;
      }
    } else {
      st.rejecting = true;
      if (st.reject_count < UINT16_MAX) st.reject_count++;
    }
  }

  // ---- 무효 타이머: 채택 프레임으로만 지운다 ----
  const bool invalid_now =
      !in.range_fresh || st.rejecting ||
      (!in.height_fresh && st.ref_mm >= RL_NEAR_GROUND_MM);
  if (accepted && !invalid_now) {
    st.invalid_timing = false;
  } else if (invalid_now && !st.invalid_timing) {
    st.invalid_timing = true;
    st.invalid_since_ms = now;
  }

  // ---- 목표 고도와 바닥 근접 스로틀 감소 ----
  const bool near = st.stage >= RL_STAGE_NEAR_GROUND;
  if (!near) {
    out.vz_ref = -rangeLandStageSpeed(st.stage);
    // 신선한 높이가 있을 때만 목표를 낮춘다(거리 끊김·거부·높이 범위 밖에서는 멈춤).
    // 끊긴 동안 내려 두면 복귀 순간 큰 하강 오차가 된다.
    if (in.height_fresh && !st.rejecting) st.h_ref += out.vz_ref * in.dt_s;
  } else {
    // 바닥 근접에서는 기준 스로틀 감소로 내려간다. 고도 보정은 .ino 가 올리는
    // 방향을 자른다. 목표는 느린 속도로 계속 낮춰 위로 끌지 않게 한다.
    st.h_ref -= RL_DESCENT_SLOW_MPS * in.dt_s;
    st.drop_us += RL_TOUCHDOWN_RAMP_US_S * in.dt_s;
  }
  if (in.height_fresh && !st.rejecting) {
    const float floor_ref = in.h_m - RL_REF_LEAD_M;
    if (st.h_ref < floor_ref) st.h_ref = floor_ref;
  }
  if (st.h_ref < 0.0f) st.h_ref = 0.0f;

  // ---- 접촉 확인 ----
  if (near && st.drop_us >= RL_CONTACT_DROP_US) st.stage = RL_STAGE_CONTACT;
  const bool delivered_drop =
      in.delivered_collective_us <= in.hover_us - RL_CONTACT_DROP_US;
  const bool in_band = in.range_fresh && !st.rejecting &&
                       st.ref_mm >= RL_LANDED_MIN_MM && st.ref_mm <= RL_LANDED_MAX_MM;
  if (st.stage == RL_STAGE_CONTACT && delivered_drop && in_band) {
    if (!st.landed_timing) {
      st.landed_timing = true;
      st.landed_since_ms = now;
    }
  } else {
    st.landed_timing = false;
  }

  // ---- 동작. 우선순위 TIMEOUT > LANDED > CUT_NEAR_GROUND > FALLBACK ----
  const bool invalid_expired =
      st.invalid_timing && now - st.invalid_since_ms >= RL_INVALID_GRACE_MS;
  const bool stuck = near && now - st.near_ground_since_ms >= RL_TOUCHDOWN_MAX_MS;
  if (now - st.enter_ms >= RL_MAX_MS) {
    out.action = RL_TIMEOUT;
    out.cut_reason = RL_CUT_BACKSTOP;
  } else if (st.landed_timing && now - st.landed_since_ms >= RL_LANDED_HOLD_MS) {
    out.action = RL_LANDED;
    out.cut_reason = RL_CUT_CONTACT;
  } else if (near && (invalid_expired || stuck)) {
    out.action = RL_CUT_NEAR_GROUND;
    out.cut_reason = invalid_expired ? RL_CUT_NEAR_GROUND_LOSS : RL_CUT_NEAR_GROUND_STUCK;
  } else if (invalid_expired) {
    out.action = RL_FALLBACK;
  }

  out.stage = st.stage;
  out.h_ref = st.h_ref;
  out.drop_us = st.drop_us;
  if (!near) {
    out.hold_scale = 1.0f;
  } else {
    const float t = (float)(now - st.near_ground_since_ms) / (float)RL_HOLD_FADE_MS;
    out.hold_scale = t >= 1.0f ? 0.0f : 1.0f - t;
  }
  return out;
}
