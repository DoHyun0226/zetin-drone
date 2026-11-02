#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "flow_hold.h"

// 부호 기대값은 계획 §2-5/6 의 물리에서 온다: 기수 들림(+pitch)이 전방
// 이동(c_pitch>0)을 멈추고, 좌 기울임(−roll)이 우측 이동(c_roll<0)을 멈춘다.

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
    if (!(expr)) throw std::runtime_error(#expr);                            \
  } while (0)

static void checkNear(double actual, double expected, double tol,
                      const char *what) {
  if (std::fabs(actual - expected) > tol) {
    throw std::runtime_error(std::string(what) + ": got " +
                             std::to_string(actual) + " want " +
                             std::to_string(expected));
  }
}

static const float kTick = 0.001f;

struct PosRig {
  PosHold ph;
  PosHoldGains gains = posHoldDefaultGains();
  PosHoldInput in;
  PosHoldOutput out;
  PosRig() {
    posHoldReset(ph);
    in.requested = true;
    in.enabled = true;
    in.calibrated = true;
  }
  // ms 동안 1 kHz 로 돌리고, 100 ms 마다 측정 c 를 넣는다.
  void run(int ms, float c_roll, float c_pitch, bool meas = true) {
    for (int i = 0; i < ms; i++) {
      in.has_meas = meas && (i % 100 == 99);
      in.meas_ok = true;
      in.c_roll = c_roll;
      in.c_pitch = c_pitch;
      in.meas_dt_s = 0.1f;
      out = posHoldStep(ph, gains, in, kTick);
    }
  }
};

int main() {
  runCase("first output brakes forward, backward, right and left drift", [] {
    struct Case { float cr, cp; int roll_sign, pitch_sign; };
    const Case cases[] = {
        {0.0f, 0.3f, 0, +1},    // 전방 → 기수 들림
        {0.0f, -0.3f, 0, -1},   // 후방 → 기수 내림
        {-0.3f, 0.0f, -1, 0},   // 우측 → 좌 기울임
        {0.3f, 0.0f, +1, 0},    // 좌측 → 우 기울임
    };
    for (const Case &c : cases) {
      PosRig rig;
      rig.run(150, c.cr, c.cp);   // 측정 1개 + 출력 변화율 제한이 따라올 시간
      CHECK(rig.out.state == POSHOLD_ACTIVE);
      const float r = rig.out.candidate_roll_deg, p = rig.out.candidate_pitch_deg;
      CHECK((c.roll_sign > 0) ? r > 0.1f : (c.roll_sign < 0) ? r < -0.1f
                                                            : std::fabs(r) < 1e-4f);
      CHECK((c.pitch_sign > 0) ? p > 0.1f : (c.pitch_sign < 0) ? p < -0.1f
                                                              : std::fabs(p) < 1e-4f);
      checkNear(rig.out.applied_pitch_deg, p, 1e-6, "applied==candidate");
    }
  });

  runCase("after drifting forward and stopping, it pitches up to go back", [] {
    PosRig rig;
    rig.run(2000, 0.0f, 0.3f);     // 2 s 동안 전방 0.3 m/s → 0.6 m 밀림
    rig.run(1000, 0.0f, 0.0f);     // 정지
    CHECK(rig.ph.p[1] > 0.5f);
    CHECK(rig.out.candidate_pitch_deg > 0.1f);
  });

  runCase("output and integrator are clamped; leash limits the reference", [] {
    PosRig rig;
    rig.run(20000, 0.0f, 3.0f);
    CHECK(std::fabs(rig.out.candidate_pitch_deg) <= rig.gains.out_max_deg + 1e-4f);
    CHECK(std::fabs(rig.ph.i[1]) <= rig.gains.i_max_deg + 1e-4f);
    CHECK(std::fabs(rig.ph.p_ref[1] - rig.ph.p[1]) <= rig.gains.leash_m + 1e-4f);
  });

  runCase("the integrator does not wind up while the output saturates", [] {
    PosRig rig;
    rig.gains.k_v = 100.0f;         // 첫 측정부터 포화
    rig.run(3000, 0.0f, 0.5f);
    CHECK(std::fabs(rig.ph.i[1]) < 0.2f);
  });

  runCase("missing measurements coast, then suspend without a step", [] {
    PosRig rig;
    rig.run(1000, 0.0f, 0.3f);
    const float before = rig.out.candidate_pitch_deg;
    const float iPart = rig.ph.i[1];
    CHECK(before > 0.5f);
    // 300 ms 동안은 마지막 출력 유지
    rig.run(250, 0.0f, 0.0f, false);
    CHECK(rig.out.state == POSHOLD_COAST);
    // COAST 는 마지막 목표를 유지한다(출력은 변화율 제한으로 그 목표를 따라갈 뿐 줄지 않는다)
    checkNear(rig.out.candidate_pitch_deg, before, 0.2, "coast holds");
    // 이후 SUSPENDED: tick 당 변화가 작고, 적분항까지 0 으로 간다
    float prev = rig.out.candidate_pitch_deg;
    float maxStep = 0.0f;
    for (int i = 0; i < 1000; i++) {
      rig.run(1, 0.0f, 0.0f, false);
      maxStep = std::max(maxStep, std::fabs(rig.out.candidate_pitch_deg - prev));
      prev = rig.out.candidate_pitch_deg;
    }
    CHECK(rig.out.state == POSHOLD_SUSPENDED);
    CHECK(maxStep < 0.1f);
    (void)iPart;
    checkNear(rig.out.candidate_pitch_deg, 0.0, 0.05, "whole output decays");
    // 측정이 돌아오면 현재 위치를 새 기준으로 다시 잡는다
    rig.run(100, 0.0f, 0.0f);
    CHECK(rig.out.state == POSHOLD_ACTIVE);
    checkNear(rig.ph.p_ref[1], rig.ph.p[1], 1e-5, "re-captured ref");
  });

  runCase("pilot input overrides: ref follows, whole correction fades", [] {
    // 적분이 남으면 −3° 를 요청해도 +5° 적분이 방향을 뒤집는다(재검토 반례).
    PosRig rig;
    rig.run(3000, 0.0f, 0.3f);
    CHECK(rig.ph.i[1] > 0.5f);
    rig.in.pilot_pitch_deg = -3.0f;
    float prev = rig.out.candidate_pitch_deg, maxStep = 0.0f;
    for (int i = 0; i < 1000; i++) {
      rig.run(1, 0.0f, 0.3f);
      maxStep = std::max(maxStep, std::fabs(rig.out.candidate_pitch_deg - prev));
      prev = rig.out.candidate_pitch_deg;
    }
    CHECK(rig.out.state == POSHOLD_OVERRIDE);
    CHECK(maxStep < 0.2f);
    checkNear(rig.ph.p_ref[1], rig.ph.p[1], 1e-5, "ref follows");
    checkNear(rig.out.candidate_pitch_deg, 0.0, 0.01, "correction faded");
    rig.in.pilot_pitch_deg = 0.5f;
    rig.run(100, 0.0f, 0.0f);
    CHECK(rig.out.state == POSHOLD_ACTIVE);
  });

  runCase("override returns authority fast: +8 deg is gone within 300 ms", [] {
    // 변화율 제한(10 °/s)이 감쇠를 늦추면 −3° 명령에도 +2.3° 가 0.3 s 뒤까지 남는다.
    PosRig rig;
    rig.run(3000, 0.0f, 1.5f);
    CHECK(rig.out.candidate_pitch_deg > 7.5f);
    rig.in.pilot_pitch_deg = -3.0f;
    rig.run(100, 0.0f, 1.5f);
    CHECK(rig.out.candidate_pitch_deg < 3.0f);     // e^-1 · 8
    rig.run(200, 0.0f, 1.5f);
    CHECK(std::fabs(rig.out.candidate_pitch_deg) < 0.5f);
  });

  runCase("yaw rate command, yaw motion or heading change overrides", [] {
    {
      PosRig rig;
      rig.run(500, 0.0f, 0.0f);
      rig.in.yaw_rate_cmd_dps = 20.0f;
      rig.run(10, 0.0f, 0.0f);
      CHECK(rig.out.state == POSHOLD_OVERRIDE);
    }
    {
      PosRig rig;
      rig.run(500, 0.0f, 0.0f);
      rig.in.gyro_z_dps = -15.0f;
      rig.run(10, 0.0f, 0.0f);
      CHECK(rig.out.state == POSHOLD_OVERRIDE);
    }
    {
      PosRig rig;
      rig.in.yaw_deg = 170.0f;
      rig.run(500, 0.0f, 0.0f);
      rig.in.yaw_deg = -170.0f;       // wrap 너머 20° 변화
      rig.run(10, 0.0f, 0.0f);
      CHECK(rig.out.state == POSHOLD_OVERRIDE);
      // override 중 기준은 현재 heading 을 따라간다. 175° 는 새 기준(−170°)과
      // 15° 차이라 override 조건(>15°)이 아니고, 유지 시간이 지나면 풀린다.
      rig.in.yaw_deg = 175.0f;
      rig.run(100, 0.0f, 0.0f);
      CHECK(rig.out.state == POSHOLD_ACTIVE);
    }
  });

  runCase("disarmed dry run: candidate is computed, applied stays zero", [] {
    PosRig rig;
    rig.in.enabled = false;
    rig.run(300, 0.0f, 0.3f);
    CHECK(rig.out.candidate_pitch_deg > 0.1f);
    CHECK(rig.out.applied_roll_deg == 0.0f && rig.out.applied_pitch_deg == 0.0f);
    // 활성화 순간 내부 상태를 리셋한다
    rig.in.enabled = true;
    rig.in.has_meas = false;
    rig.out = posHoldStep(rig.ph, rig.gains, rig.in, kTick);
    CHECK(rig.ph.i[1] == 0.0f);
    CHECK(rig.ph.p[1] == 0.0f);
  });

  runCase("recovery after suspend does not step the output", [] {
    PosRig rig;
    rig.run(1000, 0.0f, 0.3f);
    rig.run(2000, 0.0f, 0.3f, false);          // SUSPENDED, 보정이 0 으로
    CHECK(rig.out.state == POSHOLD_SUSPENDED);
    float prev = rig.out.candidate_pitch_deg, maxStep = 0.0f;
    for (int i = 0; i < 500; i++) {
      rig.in.has_meas = (i % 100 == 99);
      rig.in.meas_ok = true;
      rig.in.c_roll = 0.0f;
      rig.in.c_pitch = 0.3f;
      rig.in.meas_dt_s = 0.1f;
      rig.out = posHoldStep(rig.ph, rig.gains, rig.in, kTick);
      maxStep = std::max(maxStep, std::fabs(rig.out.candidate_pitch_deg - prev));
      prev = rig.out.candidate_pitch_deg;
    }
    CHECK(rig.out.state == POSHOLD_ACTIVE);
    CHECK(rig.out.candidate_pitch_deg > 0.5f);
    CHECK(maxStep <= 0.011f);                  // 10 °/s 변화율 제한
  });

  runCase("not requested or uncalibrated is OFF with zero output", [] {
    PosRig rig;
    rig.run(1000, 0.0f, 0.3f);
    rig.in.requested = false;
    rig.run(1, 0.0f, 0.3f);
    CHECK(rig.out.state == POSHOLD_OFF);
    CHECK(rig.out.candidate_pitch_deg == 0.0f && rig.out.applied_pitch_deg == 0.0f);
    PosRig rig2;
    rig2.in.calibrated = false;
    rig2.run(300, 0.0f, 0.3f);
    CHECK(rig2.out.state == POSHOLD_OFF);
  });

  runCase("a measurement not usable for hold (fast rotation) is skipped", [] {
    PosRig rig;
    for (int i = 0; i < 300; i++) {
      rig.in.has_meas = (i % 100 == 99);
      rig.in.meas_ok = false;
      rig.in.c_pitch = 0.3f;
      rig.in.meas_dt_s = 0.1f;
      rig.out = posHoldStep(rig.ph, rig.gains, rig.in, kTick);
    }
    CHECK(rig.ph.p[1] == 0.0f);
    CHECK(rig.out.candidate_pitch_deg == 0.0f);
  });

  // ---------------- 고도 유지 ----------------

  struct AltRig {
    AltHold ah;
    AltHoldGains gains = altHoldDefaultGains();
    AltHoldInput in;
    AltHoldOutput out;
    AltRig() {
      altHoldReset(ah);
      in.requested = true;
      in.enabled = true;
      in.height_valid = true;
      in.h_m = 0.5f;
      in.base_throttle = 1340;
    }
    void run(int ms) {
      for (int i = 0; i < ms; i++) out = altHoldStep(ah, gains, in, kTick);
    }
  };

  runCase("althold: below target adds throttle, climbing subtracts", [] {
    AltRig rig;
    rig.run(10);                       // 0.5 m 를 목표로 잡는다
    CHECK(rig.out.state == ALTHOLD_ACTIVE);
    rig.in.h_m = 0.4f;
    rig.run(10);
    CHECK(rig.out.candidate_us > 0.0f);
    rig.in.h_m = 0.5f;
    rig.in.vz_ms = 0.5f;
    rig.run(100);
    CHECK(rig.out.candidate_us < 0.0f);
    rig.in.h_m = -5.0f;
    rig.in.vz_ms = 0.0f;
    rig.run(20000);
    CHECK(rig.out.candidate_us <= rig.gains.out_max_us + 1e-3f);
    CHECK(std::fabs(rig.ah.i) <= rig.gains.i_max_us + 1e-3f);
  });

  runCase("althold: a throttle change then hold does not climb forever", [] {
    // 간단한 수직 플랜트: 1 us ≈ 0.0289 m/s² (SIL 기체). 기준 1340 us 가 호버.
    AltRig rig;
    double h = 0.5, vz = 0.0;
    auto plant = [&](int ms) {
      for (int i = 0; i < ms; i++) {
        rig.in.h_m = (float)h;
        rig.in.vz_ms = (float)vz;
        rig.out = altHoldStep(rig.ah, rig.gains, rig.in, kTick);
        const double a =
            0.0289 * ((rig.in.base_throttle - 1340) + rig.out.applied_us);
        vz += a * kTick;
        h += vz * kTick;
      }
    };
    plant(2000);
    rig.in.base_throttle = 1370;       // 조종자가 +30 us 올리고 그대로 둔다
    plant(1);
    CHECK(rig.out.state == ALTHOLD_ADJUST);
    plant(12000);
    CHECK(rig.out.state == ALTHOLD_ACTIVE);
    checkNear(vz, 0.0, 0.05, "vz settles");
    const double hAfter = h;
    plant(5000);
    checkNear(h, hAfter, 0.05, "height held");
  });

  runCase("althold: invalid height suspends smoothly; I fades slowly", [] {
    AltRig rig;
    rig.run(10);
    rig.in.h_m = 0.3f;
    rig.run(3000);
    const float iPart = rig.ah.i;
    CHECK(iPart > 1.0f);
    rig.in.height_valid = false;
    rig.run(200);
    CHECK(rig.out.state == ALTHOLD_ACTIVE);   // 300 ms 전에는 유지
    float prev = rig.out.candidate_us, maxStep = 0.0f;
    for (int i = 0; i < 1000; i++) {
      rig.run(1);
      maxStep = std::max(maxStep, std::fabs(rig.out.candidate_us - prev));
      prev = rig.out.candidate_us;
    }
    CHECK(rig.out.state == ALTHOLD_SUSPENDED);
    CHECK(maxStep < 2.0f);
    // 0.7 s 무효 뒤 I 는 exp(−0.7/1.0) ≈ 0.50 배쯤 남는다(한 번에 빠지지 않는다)
    CHECK(rig.out.candidate_us > 0.35f * iPart && rig.out.candidate_us < 0.65f * iPart);
    // 복귀 시 현재 높이를 새 목표로
    rig.in.height_valid = true;
    rig.in.h_m = 0.9f;
    rig.run(1);
    CHECK(rig.out.state == ALTHOLD_ACTIVE);
    checkNear(rig.ah.h_ref, 0.9, 1e-5, "re-captured");
  });

  runCase("althold: a 1 us throttle nudge while climbing is not a step", [] {
    AltRig rig;
    rig.run(10);
    rig.in.vz_ms = 0.8f;
    rig.run(300);
    const float before = rig.out.candidate_us;
    CHECK(before < -40.0f);
    rig.in.base_throttle += 1;
    rig.run(1);
    CHECK(rig.out.state == ALTHOLD_ADJUST);
    CHECK(std::fabs(rig.out.candidate_us - before) < 2.0f);
  });

  runCase("althold: ADJUST -> ACTIVE while moving does not step", [] {
    AltRig rig;
    rig.run(10);
    rig.in.base_throttle += 30;
    rig.in.vz_ms = 0.8f;
    rig.run(400);
    CHECK(rig.out.state == ALTHOLD_ADJUST);
    float prev = rig.out.candidate_us, maxStep = 0.0f;
    for (int i = 0; i < 400; i++) {
      rig.run(1);
      maxStep = std::max(maxStep, std::fabs(rig.out.candidate_us - prev));
      prev = rig.out.candidate_us;
    }
    CHECK(rig.out.state == ALTHOLD_ACTIVE);
    CHECK(maxStep <= 0.31f);                   // 300 us/s 변화율 제한
  });

  runCase("althold: inhibit (low base throttle) fades the output, no step", [] {
    AltRig rig;
    rig.run(10);
    rig.in.h_m = 0.3f;
    rig.run(3000);                     // 양의 보정을 쌓는다
    CHECK(rig.out.applied_us > 10.0f);
    rig.in.inhibit = true;
    float prev = rig.out.applied_us, maxStep = 0.0f;
    for (int i = 0; i < 600; i++) {
      rig.run(1);
      maxStep = std::max(maxStep, std::fabs(rig.out.applied_us - prev));
      prev = rig.out.applied_us;
    }
    CHECK(maxStep < 2.0f);
    CHECK(std::fabs(rig.out.applied_us) < 1.0f);
    CHECK(rig.out.state == ALTHOLD_SUSPENDED);
    // 풀리면 현재 높이를 새 목표로 다시 잡는다
    rig.in.inhibit = false;
    rig.run(1);
    CHECK(rig.out.state == ALTHOLD_ACTIVE);
    checkNear(rig.ah.h_ref, 0.3, 1e-5, "re-captured");
  });

  runCase("althold: hard zero clears the slewed output, resume ramps from 0", [] {
    AltRig rig;
    rig.run(10);
    rig.in.h_m = 0.2f;
    rig.run(3000);
    CHECK(rig.out.applied_us > 30.0f);
    rig.in.hard_zero = true;
    rig.run(1);
    CHECK(rig.out.applied_us == 0.0f && rig.out.candidate_us == 0.0f);
    rig.in.hard_zero = false;
    rig.run(1);
    CHECK(rig.out.applied_us <= 0.31f);             // 계단 없이 0 에서 다시 오른다
  });

  runCase("althold: recovery after a short dropout does not drop the output", [] {
    // 재검토 반례: 0.3 m 오차를 3 s 유지 → 거리 무효 315 ms → 복구. 감쇠 우회가 복구에도
    // 걸리면 한 tick 에 −10 us 가 빠진다.
    AltRig rig;
    rig.run(10);
    rig.in.h_m = 0.3f;
    rig.run(3000);
    rig.in.height_valid = false;
    rig.run(315);
    CHECK(rig.out.state == ALTHOLD_SUSPENDED);
    rig.in.height_valid = true;
    float prev = rig.out.candidate_us, maxStep = 0.0f;
    for (int i = 0; i < 500; i++) {
      rig.run(1);
      maxStep = std::max(maxStep, std::fabs(rig.out.candidate_us - prev));
      prev = rig.out.candidate_us;
    }
    CHECK(rig.out.state == ALTHOLD_ACTIVE);
    CHECK(maxStep <= 0.31f);
  });

  runCase("poshold: recovery while the output is still large does not step", [] {
    PosRig rig;
    rig.run(3000, 0.0f, 0.5f);
    CHECK(rig.out.candidate_pitch_deg > 3.0f);
    rig.run(400, 0.0f, 0.0f, false);             // 잠깐 SUSPENDED
    CHECK(rig.out.state == POSHOLD_SUSPENDED);
    float prev = rig.out.candidate_pitch_deg, maxStep = 0.0f;
    for (int i = 0; i < 500; i++) {
      rig.in.has_meas = (i % 100 == 0);
      rig.in.meas_ok = true;
      rig.in.c_roll = 0.0f;
      rig.in.c_pitch = 0.0f;
      rig.in.meas_dt_s = 0.1f;
      rig.out = posHoldStep(rig.ph, rig.gains, rig.in, kTick);
      maxStep = std::max(maxStep, std::fabs(rig.out.candidate_pitch_deg - prev));
      prev = rig.out.candidate_pitch_deg;
    }
    CHECK(rig.out.state == POSHOLD_ACTIVE);
    CHECK(maxStep <= 0.011f);
  });

  runCase("althold: inhibit during a slewed recovery does not step", [] {
    AltRig rig;
    rig.run(10);
    rig.in.h_m = 0.3f;
    rig.run(3000);
    rig.in.height_valid = false;
    rig.run(315);
    rig.in.height_valid = true;
    rig.run(1);                                   // 복구가 천천히 내려가는 중
    float prev = rig.out.candidate_us, maxStep = 0.0f;
    rig.in.inhibit = true;                        // 바로 다음 tick 에 base 가 경계 아래로
    for (int i = 0; i < 300; i++) {
      rig.run(1);
      maxStep = std::max(maxStep, std::fabs(rig.out.candidate_us - prev));
      prev = rig.out.candidate_us;
    }
    CHECK(maxStep < 2.0f);                        // τ 0.1 s 감쇠의 첫 tick ≈ 1 % 수준
  });

  runCase("poshold: override during a slewed recovery does not step", [] {
    PosRig rig;
    rig.run(3000, 0.0f, 0.5f);
    rig.run(400, 0.0f, 0.0f, false);
    for (int i = 0; i < 101; i++) {               // 복구 측정 하나 + 몇 tick
      rig.in.has_meas = (i == 0);
      rig.in.meas_ok = true;
      rig.in.c_pitch = 0.0f;
      rig.in.meas_dt_s = 0.1f;
      rig.out = posHoldStep(rig.ph, rig.gains, rig.in, kTick);
    }
    CHECK(rig.out.state == POSHOLD_ACTIVE);
    float prev = rig.out.candidate_pitch_deg, maxStep = 0.0f;
    rig.in.has_meas = false;
    rig.in.pilot_pitch_deg = 3.0f;
    for (int i = 0; i < 300; i++) {
      rig.out = posHoldStep(rig.ph, rig.gains, rig.in, kTick);
      maxStep = std::max(maxStep, std::fabs(rig.out.candidate_pitch_deg - prev));
      prev = rig.out.candidate_pitch_deg;
    }
    CHECK(rig.out.state == POSHOLD_OVERRIDE);
    CHECK(maxStep < 0.2f);
    CHECK(std::fabs(rig.out.candidate_pitch_deg) < 0.5f);
  });

  runCase("althold: range loss during ADJUST still suspends and fades I", [] {
    // 재검토 반례: I 40 us → 1 us 스로틀 변경(ADJUST) → 거리 10 s 무효에서 40 us 유지.
    AltRig rig;
    rig.run(10);
    rig.in.h_m = 0.35f;                 // 포화되지 않는 오차(pd ≈ 9 us)로 I 를 상한까지
    rig.run(25000);
    CHECK(rig.ah.i > 35.0f);
    rig.in.base_throttle += 1;
    rig.run(1);
    CHECK(rig.out.state == ALTHOLD_ADJUST);
    rig.in.height_valid = false;
    rig.run(3000);
    CHECK(rig.out.state == ALTHOLD_SUSPENDED);
    CHECK(rig.out.applied_us < 0.1f * 40.0f);  // τ 1 s 로 3 s 면 5 % 남짓
  });

  runCase("althold: disarmed is dry run; arming resets the target", [] {
    AltRig rig;
    rig.in.enabled = false;
    rig.run(10);
    rig.in.h_m = 0.4f;
    rig.run(10);
    CHECK(rig.out.candidate_us > 0.0f);
    CHECK(rig.out.applied_us == 0.0f);
    rig.in.enabled = true;
    rig.run(1);
    checkNear(rig.ah.h_ref, 0.4, 1e-5, "target re-captured on enable");
    CHECK(rig.ah.i == 0.0f);
    rig.in.requested = false;
    rig.run(1);
    CHECK(rig.out.state == ALTHOLD_OFF);
    CHECK(rig.out.applied_us == 0.0f);
  });

  if (g_failures) {
    std::cerr << g_failures << " case(s) failed\n";
    return 1;
  }
  std::cout << "all flow hold cases passed\n";
  return 0;
}
