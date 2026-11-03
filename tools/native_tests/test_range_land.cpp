#include <climits>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "range_land.h"

// 거리 기반 자동착륙 판정(range_land.h). 프레임은 35 ms 마다 하나, 1 kHz 로
// 돌린다. 수치 근거: docs/superpowers/plans/2026-11-03-range-autoland.md

static int g_failures = 0;

static void runCase(const std::string &name, const std::function<void()> &body) {
  try {
    body();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception &error) {
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
    g_failures++;
  }
}

#define CHECK(expr)                                                          \
  do {                                                                       \
    if (!(expr)) throw std::runtime_error(std::string(#expr) + " line " +    \
                                          std::to_string(__LINE__));         \
  } while (0)

static void checkNear(double actual, double expected, double tol,
                      const char *what) {
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tol) {
    throw std::runtime_error(std::string(what) + ": got " +
                             std::to_string(actual) + " want " +
                             std::to_string(expected));
  }
}

static const int kFrameMs = 35;
static const int kNoFrame = -1000000;

struct Rig {
  RangeLandState st{};
  RangeLandOutput out{};
  uint32_t now = 10000;
  uint32_t start_ms = 0;
  int32_t raw = -1;
  uint32_t frame_ms = 0;
  float hover = 1340.0f;
  // NAN 이 아니면 allocator 가 실제로 전달한 collective 를 hover − 이 값으로 고정
  float pinned_drop = NAN;
  RangeLandAction action = RL_CONTINUE;
  uint32_t action_ms = 0;
  uint8_t action_reason = RL_CUT_NONE;
  uint32_t near_ms = 0, contact_ms = 0;
  uint8_t max_stage = RL_STAGE_IDLE;

  void start(int32_t raw0) {
    raw = raw0;
    frame_ms = now;
    start_ms = now;
    rangeLandStart(st, raw0 * 0.001f, raw0, now);
  }

  // age_ms: 신뢰 프레임이지만 늦게 소비돼 이미 이만큼 지난 경우(Core 0 정체 등).
  void step(int32_t value, bool trusted = true, uint32_t age_ms = 0) {
    now++;
    const bool frame = value != kNoFrame;
    if (frame && trusted) {
      raw = value;
      frame_ms = now - age_ms;
    }
    RangeLandInput in;
    in.range_trusted_new = frame && trusted;
    in.range_mm = raw;
    in.range_fresh = now - frame_ms <= 150;
    in.height_fresh = in.range_fresh && raw >= 100 && raw <= 1800;
    in.h_m = raw * 0.001f;
    in.hover_us = hover;
    const float drop = std::isnan(pinned_drop) ? out.drop_us : pinned_drop;
    in.delivered_collective_us = hover - drop;
    in.dt_s = 0.001f;
    in.now_ms = now;
    out = rangeLandStep(st, in);
    if (out.stage > max_stage) max_stage = out.stage;
    if (out.stage >= RL_STAGE_NEAR_GROUND && near_ms == 0) near_ms = now;
    if (out.stage >= RL_STAGE_CONTACT && contact_ms == 0) contact_ms = now;
    if (out.action != RL_CONTINUE && action == RL_CONTINUE) {
      action = out.action;
      action_ms = now;
      action_reason = out.cut_reason;
    }
  }

  // ms 동안 돌린다. 경과 i 가 35 의 배수일 때 rawAt(i) 프레임이 온다.
  // 첫 동작(LANDED/FALLBACK/…)에서 멈춘다.
  void run(int ms, const std::function<int32_t(int)> &rawAt) {
    for (int i = 1; i <= ms && action == RL_CONTINUE; i++) {
      step((i % kFrameMs) == 0 ? rawAt(i) : kNoFrame);
    }
  }

  void hold(int ms, int32_t value) {
    run(ms, [value](int) { return value; });
  }

  // 0.12 m/s 로 from → to mm 로 내려가고 to 에 머문다(전체 ms).
  void descend(int ms, int32_t from, int32_t to) {
    run(ms, [from, to](int i) {
      const int32_t v = from - (int32_t)lround(0.12 * i);
      return v > to ? v : to;
    });
  }
};

int main() {
  runCase("ref_descends_fast_then_slow_and_reports_vz_ref", [] {
    Rig rig;
    rig.start(1000);
    CHECK(rig.st.stage == RL_STAGE_DESCENT);
    // 기체가 목표를 그대로 따라간다.
    for (int i = 1; i <= 1000; i++) {
      rig.step(i % kFrameMs == 0 ? (int32_t)lround(rig.out.h_ref * 1000.0f) : kNoFrame);
    }
    checkNear(rig.out.vz_ref, -RL_DESCENT_FAST_MPS, 1e-6, "vz_ref fast");
    checkNear(rig.out.h_ref, 1.0 - RL_DESCENT_FAST_MPS, 0.02, "h_ref after 1 s");
    for (int i = 1; i <= 3000 && rig.out.stage == RL_STAGE_DESCENT; i++) {
      rig.step(i % kFrameMs == 0 ? (int32_t)lround(rig.out.h_ref * 1000.0f) : kNoFrame);
    }
    CHECK(rig.out.stage == RL_STAGE_APPROACH);
    checkNear(rig.out.vz_ref, -RL_DESCENT_SLOW_MPS, 1e-6, "vz_ref slow");
    const float before = rig.out.h_ref;
    for (int i = 1; i <= 500; i++) {
      rig.step(i % kFrameMs == 0 ? (int32_t)lround(rig.out.h_ref * 1000.0f) : kNoFrame);
    }
    checkNear(before - rig.out.h_ref, 0.5 * RL_DESCENT_SLOW_MPS, 0.01, "slow rate");
    CHECK(rig.action == RL_CONTINUE);
  });

  runCase("ref_never_leads_measured_height_by_more_than_lead", [] {
    Rig rig;
    rig.start(1000);
    float min_ref = 10.0f;
    for (int i = 1; i <= 3000; i++) {
      rig.step(i % kFrameMs == 0 ? 1000 : kNoFrame);
      min_ref = std::fmin(min_ref, rig.out.h_ref);
    }
    CHECK(rig.action == RL_CONTINUE);
    CHECK(rig.st.reject_count == 0);
    CHECK(min_ref >= 1.0f - RL_REF_LEAD_M - 0.002f);
  });

  runCase("short_positive_range_is_not_invalid_after_continuous_approach", [] {
    Rig rig;
    rig.start(400);
    rig.descend(4000, 400, 30);
    CHECK(rig.max_stage >= RL_STAGE_NEAR_GROUND);
    CHECK(rig.action != RL_FALLBACK);
    CHECK(rig.st.reject_count == 0);
  });

  runCase("near_ground_60_to_100mm_is_not_invalid", [] {
    Rig rig;
    rig.start(300);
    rig.descend(3000, 300, 80);
    CHECK(rig.near_ms != 0);
    const uint32_t near = rig.near_ms;
    while (rig.now < near + 1000) rig.step((rig.now % kFrameMs) == 0 ? 80 : kNoFrame);
    CHECK(rig.action == RL_CONTINUE);
    CHECK(rig.out.drop_us >= 0.9f * RL_TOUCHDOWN_RAMP_US_S);
  });

  runCase("jump_from_height_to_landed_band_is_invalid", [] {
    Rig rig;
    rig.start(1500);
    rig.hold(490, 1500);
    const uint32_t first = rig.now + kFrameMs;   // 다음 프레임이 30 mm
    rig.hold(2000, 30);
    CHECK(rig.action == RL_FALLBACK);
    CHECK(rig.action_ms == first + RL_INVALID_GRACE_MS);
    CHECK(rig.max_stage < RL_STAGE_NEAR_GROUND);
  });

  runCase("two_step_jump_does_not_pass_continuity", [] {
    Rig rig;
    rig.start(1500);
    rig.hold(490, 1500);
    const uint32_t first = rig.now + kFrameMs;
    rig.run(2000, [](int i) { return i <= kFrameMs ? 200 : 30; });
    CHECK(rig.action == RL_FALLBACK);
    CHECK(rig.action_ms == first + RL_INVALID_GRACE_MS);
    CHECK(rig.max_stage < RL_STAGE_NEAR_GROUND);
    CHECK(rig.st.ref_mm == 1500);
  });

  runCase("held_jump_never_becomes_reference", [] {
    Rig rig;
    rig.start(1500);
    rig.hold(490, 1500);
    const uint32_t first = rig.now + kFrameMs;
    rig.run(3000, [](int i) { return i <= 600 ? 200 : (i <= 635 ? 130 : 60); });
    CHECK(rig.action == RL_FALLBACK);
    CHECK(rig.action_ms == first + RL_INVALID_GRACE_MS);
    CHECK(rig.max_stage < RL_STAGE_NEAR_GROUND);
  });

  runCase("tolerance_does_not_grow_during_rejection", [] {
    Rig rig;
    rig.start(300);
    CHECK(rig.st.stage == RL_STAGE_APPROACH);
    const uint32_t first = rig.now + kFrameMs;
    for (int i = 1; i <= 280; i++) rig.step(i % kFrameMs == 0 ? 90 : kNoFrame);
    CHECK(rig.st.ref_mm == 300);
    CHECK(rig.st.reject_count == 8);
    rig.hold(1000, 90);
    CHECK(rig.action == RL_FALLBACK);
    CHECK(rig.action_ms == first + RL_INVALID_GRACE_MS);
  });

  runCase("staircase_rejections_do_not_walk_prediction", [] {
    Rig rig;
    rig.start(300);
    const uint32_t first = rig.now + kFrameMs;
    rig.run(2000, [](int i) { return i <= 245 ? 200 : (i <= 490 ? 100 : 40); });
    CHECK(rig.action == RL_FALLBACK);
    CHECK(rig.action_ms == first + RL_INVALID_GRACE_MS);
    CHECK(rig.st.ref_mm == 300);
    CHECK(rig.max_stage < RL_STAGE_NEAR_GROUND);
  });

  runCase("negative_raw_is_never_accepted", [] {
    Rig rig;
    rig.start(150);
    const uint32_t first = rig.now + kFrameMs;
    rig.hold(2000, -1);
    CHECK(rig.action == RL_FALLBACK);
    CHECK(rig.action_ms == first + RL_INVALID_GRACE_MS);
    CHECK(rig.st.ref_mm == 150);
  });

  runCase("contact_needs_delivered_collective_drop", [] {
    Rig lifted;
    lifted.pinned_drop = 5.0f;   // 요청은 내렸지만 allocator 가 거의 다 되돌림
    lifted.start(300);
    lifted.descend(2900, 300, 30);
    CHECK(lifted.max_stage >= RL_STAGE_CONTACT);
    CHECK(lifted.action == RL_CONTINUE);

    Rig normal;
    normal.start(300);
    normal.descend(4000, 300, 30);
    CHECK(normal.action == RL_LANDED);
    CHECK(normal.action_reason == RL_CUT_CONTACT);
    CHECK(normal.action_ms == normal.contact_ms + RL_LANDED_HOLD_MS);
  });

  runCase("normal_descent_frames_are_all_accepted", [] {
    Rig rig;
    rig.start(1500);
    // 계획 속도 그대로 내려가는 기체, 결정적 잡음 ±8 mm, 17 번째 프레임마다 결손(6 %).
    double h = 1500.0;
    int frame = 0;
    for (int i = 1; i <= 9000 && h > 120.0; i++) {
      h -= (h > 300.0 ? RL_DESCENT_FAST_MPS : RL_DESCENT_SLOW_MPS);
      int32_t v = kNoFrame;
      if (i % kFrameMs == 0) {
        frame++;
        const int noise = ((frame * 37) % 17) - 8;
        if (frame % 17 != 0) v = (int32_t)lround(h) + noise;
      }
      rig.step(v);
    }
    CHECK(h <= 120.0);
    CHECK(rig.st.reject_count == 0);
    CHECK(rig.action == RL_CONTINUE);
  });

  runCase("accepted_frame_resets_rejection_timer", [] {
    Rig rig;
    rig.start(1000);
    rig.run(200, [](int) { return 30; });   // 거부 5 프레임, 무효 타이머 진행
    CHECK(rig.st.invalid_timing);
    rig.run(600, [](int) { return 990; });
    CHECK(!rig.st.invalid_timing);
    CHECK(rig.action == RL_CONTINUE);
  });

  runCase("untrusted_frames_are_neither_evidence_nor_invalid", [] {
    Rig rig;
    rig.start(1000);
    // 신뢰 프레임 1000 mm 사이에 시각을 못 믿는 30 mm / −1 프레임이 끼어 있다.
    for (int i = 1; i <= 2000; i++) {
      if (i % kFrameMs == 0) rig.step(1000);
      else if (i % kFrameMs == 17) rig.step((i / kFrameMs) % 2 ? 30 : -1, false);
      else rig.step(kNoFrame);
    }
    CHECK(rig.action == RL_CONTINUE);
    CHECK(rig.max_stage < RL_STAGE_NEAR_GROUND);
    CHECK(rig.st.reject_count == 0);
    // 신뢰 프레임이 끊기면 마지막 신뢰 프레임 나이로만 무효가 된다.
    const uint32_t last = rig.frame_ms;
    for (int i = 1; i <= 1000 && rig.action == RL_CONTINUE; i++) {
      rig.step(i % kFrameMs == 17 ? 30 : kNoFrame, false);
    }
    CHECK(rig.action == RL_FALLBACK);
    CHECK(rig.action_ms == last + 151 + RL_INVALID_GRACE_MS);
  });

  runCase("stale_trusted_frame_is_not_evidence", [] {
    // 마지막 채택 뒤 200 ms 공백이면 허용폭이 170 mm 라, 늦게 소비된 30 mm 프레임이
    // 공간 조건을 통과한다. 신선하지 않은 프레임은 증거가 아니다(Codex 코드 검토 P1).
    Rig rig;
    rig.start(150);
    for (int i = 0; i < 199; i++) rig.step(kNoFrame);
    rig.step(30, true, 200);           // 신뢰 프레임이지만 200 ms 지난 값
    CHECK(rig.max_stage < RL_STAGE_NEAR_GROUND);
    CHECK(rig.st.ref_mm == 150);
    while (rig.action == RL_CONTINUE && rig.now < rig.start_ms + 3000) rig.step(kNoFrame);
    CHECK(rig.action == RL_FALLBACK);
  });

  runCase("extreme_negative_raw_is_rejected_without_overflow", [] {
    Rig rig;
    rig.start(300);
    rig.run(35, [](int) { return (int32_t)INT32_MIN; });
    CHECK(rig.st.reject_count == 1);
    CHECK(rig.st.ref_mm == 300);
    rig.run(35, [](int) { return (int32_t)INT32_MAX; });
    CHECK(rig.st.reject_count == 2);
  });

  runCase("ref_does_not_descend_without_a_fresh_range", [] {
    Rig rig;
    rig.start(1000);
    for (int i = 0; i < 140; i++) rig.step(kNoFrame);   // 아직 신선(≤ 150 ms)
    const float fresh_ref = rig.out.h_ref;
    checkNear(fresh_ref, 1.0 - 0.140 * RL_DESCENT_FAST_MPS, 0.002, "fresh descent");
    for (int i = 0; i < 160; i++) rig.step(kNoFrame);   // 150 ms 넘게 프레임 없음
    checkNear(rig.out.h_ref, 1.0 - 0.150 * RL_DESCENT_FAST_MPS, 0.002, "stops when stale");
  });

  runCase("near_ground_prediction_does_not_move", [] {
    // 바닥 근접·접촉 단계에서는 목표 속도 0 — 예측 중심이 마지막 채택값에 머문다.
    // 80 mm 다음 35 ms 에 149 mm 는 오차 69 ≤ 허용 71 이라 채택, 152 mm 는 72 로 거부.
    for (uint8_t stage : {RL_STAGE_NEAR_GROUND, RL_STAGE_CONTACT}) {
      for (int32_t next : {149, 152}) {
        Rig rig;
        rig.start(300);
        for (int i = 1; i <= 4000 && rig.out.stage < stage; i++) {
          const int32_t v = std::max(80, 300 - (int)lround(0.12 * i));
          rig.step(i % kFrameMs == 0 ? v : kNoFrame);
        }
        CHECK(rig.out.stage == stage);
        // 같은 단계에서 80 mm 프레임 하나를 더 채택해 기준 시각을 맞춘다.
        rig.step(80);
        CHECK(rig.st.ref_mm == 80 && rig.out.stage == stage);
        const uint16_t before = rig.st.reject_count;
        const uint32_t last = rig.st.ref_ms;
        while (rig.now < last + 34) rig.step(kNoFrame);
        rig.step(next);
        CHECK(rig.out.stage == stage);
        CHECK(rig.st.reject_count == before + (next == 149 ? 0 : 1));
      }
    }
  });

  runCase("ref_stops_while_height_is_out_of_range", [] {
    Rig rig;
    rig.start(1790);
    rig.hold(35, 1850);                // 추적은 통과하지만 높이 범위(≤ 1.80 m) 밖
    const float ref = rig.out.h_ref;
    rig.hold(100, 1850);
    checkNear(rig.out.h_ref, ref, 1e-6, "target held while height invalid");
  });

  runCase("landed_requires_contact_drop", [] {
    Rig rig;
    rig.start(300);
    rig.descend(4000, 300, 30);
    CHECK(rig.action == RL_LANDED);
    CHECK(rig.near_ms != 0 && rig.contact_ms > rig.near_ms);
    // 바닥 근접 뒤 drop 이 40 µs 에 닿기까지 약 400 ms, 그 전에는 착지 없음
    CHECK(rig.contact_ms - rig.near_ms >= 395);
    CHECK(rig.action_ms - rig.near_ms >= 795);
  });

  runCase("landed_after_hold_once_drop_reached", [] {
    Rig rig;
    rig.start(300);
    rig.descend(4000, 300, 30);
    CHECK(rig.action == RL_LANDED);
    CHECK(rig.action_ms == rig.contact_ms + RL_LANDED_HOLD_MS);
  });

  runCase("landed_timer_stops_on_stale_frame", [] {
    Rig rig;
    rig.start(300);
    for (int i = 1; i <= 4000 && rig.contact_ms == 0; i++) {
      const int32_t v = std::max(30, 300 - (int)lround(0.12 * i));
      rig.step(i % kFrameMs == 0 ? v : kNoFrame);
    }
    CHECK(rig.contact_ms != 0);
    const uint32_t contact = rig.contact_ms;
    // 150 ms 동안은 프레임이 오고, 그 뒤 끊긴다 → contact+300 무렵 stale.
    while (rig.now < contact + 150) rig.step(rig.now % kFrameMs == 0 ? 30 : kNoFrame);
    while (rig.now < contact + 400) rig.step(kNoFrame);
    CHECK(rig.action == RL_CONTINUE);
    while (rig.action == RL_CONTINUE && rig.now < contact + 2000) rig.step(kNoFrame);
    CHECK(rig.action == RL_CUT_NEAR_GROUND);
    CHECK(rig.action_reason == RL_CUT_NEAR_GROUND_LOSS);
  });

  runCase("fallback_after_grace_before_near_ground", [] {
    Rig rig;
    rig.start(1000);
    rig.hold(490, 1000);
    const uint32_t first = rig.now + kFrameMs;
    while (rig.now < first + RL_INVALID_GRACE_MS - 1) {
      rig.step(((int)(rig.now + 1) - (int)first) % kFrameMs == 0 ? -1 : kNoFrame);
    }
    CHECK(rig.action == RL_CONTINUE);
    rig.step(kNoFrame);
    CHECK(rig.action == RL_FALLBACK);
    CHECK(rig.action_reason == RL_CUT_NONE);
  });

  runCase("loss_after_near_ground_cuts", [] {
    Rig rig;
    rig.start(300);
    rig.descend(2000, 300, 80);
    CHECK(rig.near_ms != 0);
    const uint32_t first = rig.now + kFrameMs;
    rig.hold(1000, -1);
    CHECK(rig.action == RL_CUT_NEAR_GROUND);
    CHECK(rig.action_reason == RL_CUT_NEAR_GROUND_LOSS);
    CHECK(rig.action_ms == first + RL_INVALID_GRACE_MS);
  });

  runCase("stuck_in_near_ground_cuts_after_max", [] {
    Rig rig;
    rig.start(300);
    rig.descend(2000, 300, 80);
    CHECK(rig.near_ms != 0);
    const uint32_t near = rig.near_ms;
    while (rig.now < near + RL_TOUCHDOWN_MAX_MS - 1) {
      rig.step(rig.now % kFrameMs == 0 ? 80 : kNoFrame);
    }
    CHECK(rig.action == RL_CONTINUE);
    rig.step(kNoFrame);
    CHECK(rig.action == RL_CUT_NEAR_GROUND);
    CHECK(rig.action_reason == RL_CUT_NEAR_GROUND_STUCK);
  });

  runCase("timeout_backstop_and_priority", [] {
    {
      Rig rig;
      rig.start(1000);
      rig.hold(RL_MAX_MS + 100, 1000);
      CHECK(rig.action == RL_TIMEOUT);
      CHECK(rig.action_reason == RL_CUT_BACKSTOP);
      CHECK(rig.action_ms == rig.start_ms + RL_MAX_MS);
    }
    // 같은 tick 에 착지가 확정돼도 백스톱이 먼저다.
    Rig probe;
    probe.start(150);
    probe.descend(4000, 150, 30);
    CHECK(probe.action == RL_LANDED);
    const int d = (int)(probe.action_ms - probe.start_ms);
    for (int shift : {0, -1}) {
      Rig rig;
      rig.start(150);
      rig.hold(RL_MAX_MS - d + shift, 150);
      rig.descend(4000, 150, 30);
      if (shift == 0) {
        CHECK(rig.action == RL_TIMEOUT);
        CHECK(rig.action_ms == rig.start_ms + RL_MAX_MS);
      } else {
        CHECK(rig.action == RL_LANDED);
        CHECK(rig.action_ms == rig.start_ms + RL_MAX_MS - 1);
      }
    }
  });

  runCase("hold_scale_fades_after_near_ground", [] {
    Rig rig;
    rig.start(300);
    rig.step(kNoFrame);
    checkNear(rig.out.hold_scale, 1.0, 1e-6, "before near ground");
    float prev = 1.0f;
    bool monotonic = true;
    for (int i = 1; i <= 4000 && rig.action == RL_CONTINUE; i++) {
      const int32_t v = std::max(80, 300 - (int)lround(0.12 * i));
      rig.step(i % kFrameMs == 0 ? v : kNoFrame);
      if (rig.out.hold_scale > prev + 1e-6f) monotonic = false;
      prev = rig.out.hold_scale;
      if (rig.near_ms && rig.now == rig.near_ms + RL_HOLD_FADE_MS) break;
    }
    CHECK(monotonic);
    CHECK(rig.near_ms != 0);
    checkNear(rig.out.hold_scale, 0.0, 1e-6, "faded");
  });

  if (g_failures) {
    std::cerr << g_failures << " case(s) failed\n";
    return 1;
  }
  return 0;
}
