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
// 바닥 근접 철회 뒤 위치 유지 복원. 적용값 = 후보 × hold_scale 이고 이 곱은 위치 유지의
// 변화율 제한(10°/s) 뒤에 걸린다. 복원 몫은 후보 상한 8° × 1/s = 8°/s 이고, 후보 자체 변화
// (≤ 10°/s)를 더하면 적용값은 최대 약 18°/s 로 변할 수 있다 — 엄격한 상한이 아니라 계단을
// 없애는 것이 목적이다(최종 재검토 Important: 한 tick 에 1 로 튀어 0→7° 계단).
static const uint32_t RL_HOLD_RESTORE_MS = 1000;
static const uint32_t RL_INVALID_GRACE_MS = 300;  // ALTHOLD_SUSPEND_S 와 같은 성격
static const uint32_t RL_MAX_MS = 12000;
// 바닥 근접 뒤 신선한 채택값이 이만큼 이상으로 돌아오면 바닥 근접을 철회한다(상승 외란).
// 철회하지 않으면 1.27 m 를 채택하면서도 정체 컷을 냈다(최종 검토 Important).
static const int32_t RL_NEAR_REVOKE_MM = 150;
// 프레임마다의 허용폭(한 프레임 71 mm, 약 2 m/s) 안 걸음을 이어 붙여 근접값으로 걸어
// 내려가지 못하게, 300 ms 창의 기준(anchor)에서 본 변화도 50 mm + RL_MAX_VZ_MPS·경과
// 안이어야 한다. 명령 속도가 아니라 기체가 낼 수 있는 속도에 대한 한계다 — 명령에 대한
// 여유로 잡으면 추력 변화로 기체가 명령을 벗어날 때마다 센서 이상값으로 폴백한다.
static const uint32_t RL_RATE_WINDOW_MS = 300;
// 진입 전 추적(RangeTrack). 거리 기반 모드는 이 추적이 RL_ENTRY_STABLE_MS 동안 거부 없이
// 이어졌을 때만 고른다. RC 가 끊기는 순간 센서 앞에 온 근접 물체 값을 진입 기준으로 잡지
// 않기 위해서다(Codex 코드 검토 Task 4 P1 — 연속 프레임 수만 세면 세 프레임 유지된 근접값이
// 통과했다). **더 가까운 쪽으로 뛴 값은 아무리 오래 유지돼도 기준이 되지 않는다** — 유지된
// 근접 물체를 면으로 보면 1.2 m 공중 착지 컷이 났다(최종 검토 Critical). 더 먼 쪽으로 뛴
// 값(빠른 상승, 근접 물체가 치워짐)만 RL_TRACK_RESET_MS 유지되면 새 기준이다. 탁자 위로 날아
// 올라간 경우처럼 진짜로 바닥이 가까워진 경우는 그동안 시간 기반 하강을 쓴다.
static const uint32_t RL_ENTRY_STABLE_MS = 500;
static const uint32_t RL_TRACK_RESET_MS = 1000;
// 진입 전 추적에서 프레임이 이만큼 끊기면 이전 안정 이력을 버리고 다시 쌓는다(기준은
// 남긴다). 공백만큼 넓어진 허용폭으로 받은 첫 프레임이 곧바로 안정이 되면 안 된다(Codex
// 코드 검토 P1). 허용폭 계산의 공백도 이 값에서 자른다.
static const uint32_t RL_TRACK_MAX_GAP_MS = 300;

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
  int32_t anchor_mm;         // 300 ms 창 속도 검사의 기준
  uint32_t anchor_ms;
  bool tol_fixed;
  int32_t tol_mm;
  int32_t pred_mm;
  bool rejecting;            // 마지막 신뢰 프레임이 거부됐다
  uint16_t reject_count;
  float drop_us;
  float hold_scale;          // 위치 유지 적용 비율(바닥 근접에서 0 쪽, 철회 뒤 천천히 1 로)
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

// 마지막 채택 뒤 첫 프레임까지의 공백(ms)에 대한 허용폭(mm). m/s × ms = mm.
static inline int32_t rangeLandTolMm(float gap_ms) {
  return (int32_t)lroundf(RL_MAX_STEP_M * 1000.0f + RL_MAX_VZ_MPS * gap_ms);
}

// 두 원값이 이어지는가(둘 다 양수이고 차이가 공백 허용폭 안).
static inline bool rangeLandFramesConsistent(int32_t prev_mm, int32_t raw_mm,
                                             uint32_t gap_ms) {
  if (prev_mm <= 0 || raw_mm <= 0) return false;
  const int64_t d = (int64_t)raw_mm - (int64_t)prev_mm;
  return (d < 0 ? -d : d) <= (int64_t)rangeLandTolMm((float)gap_ms);
}

// 창 기준(anchor)에서 본 변화가 기체 속도 한계 안인가.
static inline bool rangeLandWindowOk(int32_t anchor_mm, uint32_t anchor_ms,
                                     int32_t raw_mm, uint32_t now_ms) {
  const int64_t d = (int64_t)raw_mm - (int64_t)anchor_mm;
  return (d < 0 ? -d : d) <= (int64_t)rangeLandTolMm((float)(now_ms - anchor_ms));
}

// 채택 프레임으로 창 기준을 옮긴다. 기준을 그 값으로 바로 옮기면 갱신마다 고정 여유
// 50 mm 가 새로 생겨 0.6 m/s 보다 빠른 걸음(약 0.76 m/s)을 계속 따라간다(Codex P1). 기준은
// RL_MAX_VZ_MPS 보다 빨리 움직이지 않는다.
static inline void rangeLandMoveAnchor(int32_t &anchor_mm, uint32_t &anchor_ms,
                                       int32_t raw_mm, uint32_t now_ms) {
  if (now_ms - anchor_ms < RL_RATE_WINDOW_MS) return;
  const float max_step = RL_MAX_VZ_MPS * (float)(now_ms - anchor_ms);   // mm
  float step = (float)raw_mm - (float)anchor_mm;
  if (step > max_step) step = max_step;
  if (step < -max_step) step = -max_step;
  anchor_mm += (int32_t)lroundf(step);
  anchor_ms = now_ms;
}

struct RangeTrack {
  bool have;
  uint32_t last_frame_ms;    // 채택·거부와 상관없이 마지막 프레임 시각(공백 판정)
  int32_t ref_mm;
  uint32_t ref_ms;
  int32_t anchor_mm;
  uint32_t anchor_ms;
  bool rejecting;
  uint32_t reject_since_ms;
  uint32_t stable_since_ms;
};

static inline void rangeTrackRestart(RangeTrack &t, int32_t raw_mm, uint32_t now_ms) {
  t.have = true;
  t.ref_mm = t.anchor_mm = raw_mm;
  t.ref_ms = t.anchor_ms = now_ms;
  t.rejecting = false;
  t.stable_since_ms = now_ms;
}

// 진입 전(무장 중, failsafe 밖) 신선한 신뢰 프레임마다 부른다. 조종자가 0.6 m/s 넘게
// 오르내리는 중이면 안정으로 보지 않고, 그때 RC 가 끊기면 시간 기반 하강을 쓴다.
static inline void rangeTrackUpdate(RangeTrack &t, int32_t raw_mm, uint32_t now_ms) {
  uint32_t frame_gap = now_ms - t.last_frame_ms;
  t.last_frame_ms = now_ms;
  if (!t.have) {
    if (raw_mm > 0) rangeTrackRestart(t, raw_mm, now_ms);   // 첫 기준(이륙 전 지면부터)
    return;
  }
  // 무효값·공백은 기준을 지우지 않는다. 지우면 그 다음 근접값이 연속성 검사 없이 새 기준이
  // 된다(최종 재검토 Critical: −1 한 프레임 뒤 200 mm → 1.43 m CUT_LANDED). 안정 이력만 취소한다.
  if (raw_mm <= 0) {
    if (!t.rejecting) {
      t.rejecting = true;
      t.reject_since_ms = now_ms;
    }
    return;
  }
  if (frame_gap > RL_TRACK_MAX_GAP_MS) {
    frame_gap = RL_TRACK_MAX_GAP_MS;
    if (!t.rejecting) {
      t.rejecting = true;
      t.reject_since_ms = now_ms;
    }
  }
  // 허용폭은 직전 프레임과의 공백으로 잰다. 마지막 채택 시각으로 재면 거부가 이어지는 동안
  // 허용폭이 계속 넓어져 유지된 근접값을 결국 받아들인다(하강 추적의 허용폭 고정과 같은 이유).
  const bool ok =
      rangeLandFramesConsistent(t.ref_mm, raw_mm, frame_gap) &&
      rangeLandWindowOk(t.anchor_mm, t.anchor_ms, raw_mm, now_ms);
  if (ok) {
    t.ref_mm = raw_mm;
    t.ref_ms = now_ms;
    rangeLandMoveAnchor(t.anchor_mm, t.anchor_ms, raw_mm, now_ms);
    if (t.rejecting) {
      t.rejecting = false;
      t.stable_since_ms = now_ms;
    }
  } else if (!t.rejecting) {
    t.rejecting = true;
    t.reject_since_ms = now_ms;
  } else if (raw_mm > t.ref_mm && now_ms - t.reject_since_ms >= RL_TRACK_RESET_MS) {
    rangeTrackRestart(t, raw_mm, now_ms);
  }
}

static inline bool rangeTrackStable(const RangeTrack &t, uint32_t now_ms) {
  return t.have && !t.rejecting && now_ms - t.stable_since_ms >= RL_ENTRY_STABLE_MS;
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
  st.hold_scale = 1.0f;
  st.ref_mm = range_mm_now;
  st.ref_ms = now_ms;
  st.anchor_mm = range_mm_now;
  st.anchor_ms = now_ms;
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
      st.tol_mm = rangeLandTolMm(gap_ms);
      st.pred_mm = st.ref_mm -
                   (int32_t)lroundf(rangeLandStageSpeed(st.stage) * gap_ms);
      st.tol_fixed = true;
    }
    const int32_t raw = in.range_mm;
    if (raw > 0) {
      const int64_t err = (int64_t)raw - (int64_t)st.pred_mm;
      accepted = (err < 0 ? -err : err) <= (int64_t)st.tol_mm &&
                 rangeLandWindowOk(st.anchor_mm, st.anchor_ms, raw, now);
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
      rangeLandMoveAnchor(st.anchor_mm, st.anchor_ms, raw, now);
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
      } else if (st.stage >= RL_STAGE_NEAR_GROUND && raw >= RL_NEAR_REVOKE_MM) {
        // 떠올랐다. 바닥 근접 컷 권한을 거두고 접근 하강으로 돌아간다. 기준 스로틀 감소는
        // 같은 속도로 천천히 되돌린다(아래).
        st.stage = raw <= rangeLandApproachMm() ? RL_STAGE_APPROACH : RL_STAGE_DESCENT;
        st.landed_timing = false;
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
    st.drop_us -= RL_TOUCHDOWN_RAMP_US_S * in.dt_s;   // 철회 뒤 되돌림
    if (st.drop_us < 0.0f) st.drop_us = 0.0f;
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
  if (st.stage >= RL_STAGE_NEAR_GROUND) {
    st.hold_scale -= in.dt_s * 1000.0f / (float)RL_HOLD_FADE_MS;
    if (st.hold_scale < 0.0f) st.hold_scale = 0.0f;
  } else {
    st.hold_scale += in.dt_s * 1000.0f / (float)RL_HOLD_RESTORE_MS;
    if (st.hold_scale > 1.0f) st.hold_scale = 1.0f;
  }
  out.hold_scale = st.hold_scale;
  return out;
}
