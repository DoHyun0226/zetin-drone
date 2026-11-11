#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "flow_odometry.h"

// 기대값은 계획 §2 의 물리에서 손으로 계산한다. 센서 광류는 시선 각속도
// ω_meas = (v_y/d − p, −v_x/d − q) 로 만들고, 보정 행렬 M 으로 카운트로 바꾼다.

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

static const double kPi = 3.14159265358979323846;
static const float kScale = (float)(9.5 * 180.0 / kPi);   // counts/rad
// 센서축이 기체축과 90° 돌아 있고 한 축이 뒤집힌 장착: f = M·g
static const float kM[2][2] = {{0.0f, -kScale}, {kScale, 0.0f}};

static FlowOdomConfig makeConfig() {
  FlowOdomConfig cfg = flowOdomDefaultConfig();
  CHECK(flowOdomSetCalibration(cfg, kM, 0));
  return cfg;
}

// 순수 회전에서 센서가 보는 카운트: g(회전 적분) 에 대해 f = M·(−ω_meas 적분)
// 이 아니라, 보정 정의가 "순수 회전이면 f = M·g" 이므로 그대로 M·g 다.
// 병진 성분은 같은 M 으로 −ω_meas·dt 의 병진 부분을 카운트로 바꾼다.
struct Truth {
  double vx = 0.0, vy = 0.0;      // m/s, 기체 FRD
  double p = 0.0, q = 0.0;        // rad/s
  double d = 0.5;                 // m, 광축 거리
};

static void flowCounts(const Truth &t, double dt, int32_t &fx, int32_t &fy) {
  // ω_meas = (vy/d − p, −vx/d − q). 보정 정의상 순수 회전 −(p,q) 가 g=∫(p,q)
  // 에 대응하므로 카운트 = M · (−ω_meas·dt).
  const double ex = -(t.vy / t.d - t.p) * dt;
  const double ey = -(-t.vx / t.d - t.q) * dt;
  fx = (int32_t)std::lround(kM[0][0] * ex + kM[0][1] * ey);
  fy = (int32_t)std::lround(kM[1][0] * ex + kM[1][1] * ey);
}

// 자이로 이력을 1 kHz 로 채운다.
static void fillGyro(FlowGyroHistory &h, uint32_t t0_us, uint32_t t1_us,
                     double p, double q) {
  for (uint32_t t = t0_us; t <= t1_us; t += 1000) {
    flowGyroPush(h, t, (float)p, (float)q, 0.001f);
  }
}

struct Rig {
  FlowOdomConfig cfg = makeConfig();
  FlowOdomState state;
  FlowGyroHistory gyro;
  uint32_t t_us = 1000000;
  Rig() {
    flowGyroReset(gyro);
    flowOdomReset(state);
  }
  // 한 광류 구간을 진행하고 결과를 돌려준다.
  // counts_dt_s: 프레임 카운트가 덮는 시간. 앞 프레임이 유실되면 도착 간격은
  // 200 ms 지만 카운트는 센서 주기 100 ms 분뿐이다.
  FlowOdomOutput step(const Truth &truth, double dt_s = 0.1,
                      bool timing_ok = true, int quality = 120,
                      uint32_t crc_errors = 0, float roll_deg = 0.0f,
                      float pitch_deg = 0.0f, bool range_valid = true,
                      double counts_dt_s = -1.0) {
    const uint32_t t1 = t_us + (uint32_t)std::lround(dt_s * 1e6);
    fillGyro(gyro, t_us + 1000, t1, truth.p, truth.q);
    t_us = t1;
    FlowFrame frame;
    flowCounts(truth, counts_dt_s > 0 ? counts_dt_s : dt_s, frame.fx, frame.fy);
    frame.quality = quality;
    frame.t_us = t1;
    frame.timing_ok = timing_ok;
    (void)crc_errors;
    return flowOdomUpdate(state, cfg, frame, gyro, (float)truth.d,
                          range_valid, roll_deg, pitch_deg);
  }
};

int main() {
  runCase("first frame is discarded, second is used", [] {
    Rig rig;
    Truth t;
    FlowOdomOutput a = rig.step(t);
    CHECK(!a.valid);
    CHECK(a.reject == FLOW_REJECT_FIRST);
    FlowOdomOutput b = rig.step(t);
    CHECK(b.valid);
  });

  runCase("pure rotation gives zero translational velocity", [] {
    Rig rig;
    Truth t;
    t.p = 0.8; t.q = -0.5;
    rig.step(t);
    FlowOdomOutput o = rig.step(t);
    CHECK(o.valid);
    checkNear(o.c_roll, 0.0, 0.02, "c_roll");
    checkNear(o.c_pitch, 0.0, 0.02, "c_pitch");
  });

  runCase("forward 0.3 m/s gives c_pitch=+0.3 (not 17.2: rad, not deg)", [] {
    for (double d : {0.5, 1.5}) {
      Rig rig;
      Truth t;
      t.vx = 0.3; t.d = d;
      rig.step(t);
      FlowOdomOutput o = rig.step(t);
      CHECK(o.valid);
      checkNear(o.c_pitch, 0.3, 0.02, "c_pitch");
      checkNear(o.c_roll, 0.0, 0.02, "c_roll");
    }
  });

  runCase("right 0.3 m/s gives c_roll=-0.3", [] {
    for (double d : {0.5, 1.5}) {
      Rig rig;
      Truth t;
      t.vy = 0.3; t.d = d;
      rig.step(t);
      FlowOdomOutput o = rig.step(t);
      CHECK(o.valid);
      checkNear(o.c_roll, -0.3, 0.02, "c_roll");
      checkNear(o.c_pitch, 0.0, 0.02, "c_pitch");
    }
  });

  runCase("translation plus rotation separates the translation", [] {
    Rig rig;
    Truth t;
    t.vx = -0.2; t.vy = 0.1; t.p = 0.6; t.q = 0.4; t.d = 0.8;
    rig.step(t);
    FlowOdomOutput o = rig.step(t);
    CHECK(o.valid);
    checkNear(o.c_pitch, -0.2, 0.02, "c_pitch");
    checkNear(o.c_roll, -0.1, 0.02, "c_roll");
  });

  runCase("one lost frame: velocity from the last period, span covers the gap", [] {
    // 센서는 100 ms 주기로 보낸다. 앞 프레임이 빠지면 이 프레임은 마지막 100 ms 만
    // 덮는다. 속도는 그 구간으로, 위치 적분은 200 ms 전체로 한다.
    Rig rig;
    Truth t;
    t.vx = 0.3; t.p = 0.5;
    rig.step(t);
    FlowOdomOutput o = rig.step(t, 0.2, true, 120, 0, 0, 0, true, 0.1);
    CHECK(o.valid);
    checkNear(o.dt_s, 0.1, 1e-4, "velocity interval");
    checkNear(o.span_s, 0.2, 1e-4, "position span");
    checkNear(o.c_pitch, 0.3, 0.02, "c_pitch");
    checkNear(o.c_roll, 0.0, 0.02, "c_roll");
    // 두 개 이상 빠지면(300 ms) 버린다. 다음 정상 구간은 다시 쓴다.
    FlowOdomOutput two = rig.step(t, 0.3, true, 120, 0, 0, 0, true, 0.1);
    CHECK(!two.valid);
    CHECK(two.reject == FLOW_REJECT_DT);
    FlowOdomOutput n = rig.step(t);
    CHECK(n.valid);
    checkNear(n.span_s, 0.1, 1e-4, "normal span");
  });

  runCase("unreliable arrival timing is rejected and restarts the chain", [] {
    Rig rig;
    Truth t;
    rig.step(t);
    FlowOdomOutput o = rig.step(t, 0.1, false);
    CHECK(!o.valid);
    CHECK(o.reject == FLOW_REJECT_TIMING);
    // 늦게 찍힌 시각을 다음 구간의 시작점으로 쓰지 않는다(84 ms 구간 통과 방지)
    CHECK(rig.step(t).reject == FLOW_REJECT_FIRST);
    CHECK(rig.step(t).valid);
  });

  runCase("gyro history and intervals survive the 32-bit micros wrap", [] {
    FlowGyroHistory gyro;
    flowGyroReset(gyro);
    const uint32_t start = 0xFFFFFFFFu - 150000u;   // 150 ms 뒤 wrap
    for (uint32_t i = 0; i <= 400; i++) {
      flowGyroPush(gyro, start + i * 1000u, 1.0f, -0.5f, 0.001f);
    }
    float gx = 0.0f, gy = 0.0f;
    // wrap 을 가로지르는 100 ms 구간
    CHECK(flowGyroDelta(gyro, start + 100000u, start + 200000u, gx, gy));
    checkNear(gx, 0.1, 1e-4, "gx across wrap");
    checkNear(gy, -0.05, 1e-4, "gy across wrap");
  });

  runCase("a CRC error elsewhere on the link does not reject a normal interval", [] {
    // 광류 프레임 유실은 도착 간격으로 드러난다. CRC 오류는 거리 프레임일 수도
    // 있으므로 그것만으로 광류 구간을 버리지 않는다.
    Rig rig;
    Truth t;
    rig.step(t, 0.1, true, 120, 0);
    CHECK(rig.step(t, 0.1, true, 120, 1).valid);
  });

  runCase("low quality, invalid range and excess tilt are rejected", [] {
    Rig rig;
    Truth t;
    rig.step(t);
    CHECK(rig.step(t, 0.1, true, 20).reject == FLOW_REJECT_QUALITY);
    rig.step(t);
    CHECK(rig.step(t, 0.1, true, 120, 0, 0, 0, false).reject ==
          FLOW_REJECT_RANGE);
    rig.step(t);
    CHECK(rig.step(t, 0.1, true, 120, 0, 25.0f, 0.0f).reject ==
          FLOW_REJECT_TILT);
  });

  runCase("flow above the 7.4 rad/s sensor limit is rejected", [] {
    Rig rig;
    Truth t;
    t.p = 8.0;
    rig.step(t);
    FlowOdomOutput o = rig.step(t);
    CHECK(!o.valid);
    CHECK(o.reject == FLOW_REJECT_FLOW_RATE);
  });

  runCase("fast rotation is valid but flagged as not usable for hold", [] {
    Rig rig;
    Truth t;
    t.p = 3.0;
    rig.step(t);
    FlowOdomOutput o = rig.step(t);
    CHECK(o.valid);
    CHECK(!o.rot_ok);
    Truth slow;
    slow.p = 0.5;
    rig.step(slow);
    CHECK(rig.step(slow).rot_ok);
  });

  runCase("gyro window honours the configured delay", [] {
    // 센서가 30 ms 늦은 구간을 본다고 하고, 자이로는 구간 중간에 회전을 시작한다.
    FlowOdomConfig cfg = makeConfig();
    CHECK(flowOdomSetCalibration(cfg, kM, 30000));
    FlowGyroHistory gyro;
    flowGyroReset(gyro);
    // 0~1.1 s 정지, 1.1~1.4 s 동안 p = 1 rad/s.
    for (uint32_t t = 1000; t <= 1400000; t += 1000) {
      const double p = (t > 1100000) ? 1.0 : 0.0;
      flowGyroPush(gyro, t, (float)p, 0.0f, 0.001f);
    }
    float gx = 0.0f, gy = 0.0f;
    // 지연 30 ms: 구간 [1.27, 1.37] 은 전부 회전 중 → 0.1 rad
    CHECK(flowGyroDelta(gyro, 1270000, 1370000, gx, gy));
    checkNear(gx, 0.1, 0.002, "gx inside");
    // [1.05, 1.15] 은 절반만 회전 → 0.05 rad
    CHECK(flowGyroDelta(gyro, 1050000, 1150000, gx, gy));
    checkNear(gx, 0.05, 0.002, "gx half");
    // 이력보다 오래된 구간은 실패
    CHECK(!flowGyroDelta(gyro, 100, 200, gx, gy));
    CHECK(!flowGyroDelta(gyro, 1390000, 1500000, gx, gy));
  });

  runCase("uncalibrated config rejects every frame", [] {
    FlowOdomConfig cfg = flowOdomDefaultConfig();
    FlowOdomState state;
    flowOdomReset(state);
    FlowGyroHistory gyro;
    flowGyroReset(gyro);
    fillGyro(gyro, 1000, 300000, 0.0, 0.0);
    FlowFrame f;
    f.t_us = 100000; f.quality = 120; f.timing_ok = true;
    flowOdomUpdate(state, cfg, f, gyro, 0.5f, true, 0, 0);
    f.t_us = 200000;
    FlowOdomOutput o = flowOdomUpdate(state, cfg, f, gyro, 0.5f, true, 0, 0);
    CHECK(!o.valid);
    CHECK(o.reject == FLOW_REJECT_UNCALIBRATED);
  });

  runCase("height: tilt correction, out-of-range and first-frame vz", [] {
    HeightEstimator h;
    heightReset(h);
    heightUpdate(h, -1, 0, 0.0f, 0.0f);
    CHECK(!h.valid);
    heightUpdate(h, 1000, 35000, 20.0f, 0.0f);
    CHECK(h.valid);
    checkNear(h.h_m, std::cos(20.0 * kPi / 180.0), 1e-4, "tilted h");
    checkNear(h.vz_ms, 0.0, 1e-6, "first vz");
    // 0.35 s 동안 0.035 m/35 ms 씩 상승 = 1 m/s. LPF 뒤 1 에 가까워진다.
    uint32_t t = 35000;
    int mm = 1000;
    for (int i = 0; i < 30; i++) {
      t += 35000;
      mm += 35;
      heightUpdate(h, mm, t, 0.0f, 0.0f);
    }
    checkNear(h.vz_ms, 1.0, 0.05, "climb vz");
    // 범위 밖 → 무효, 복귀 첫 프레임은 vz 0 (큰 차분이 들어가지 않는다)
    heightUpdate(h, -1, t + 35000, 0.0f, 0.0f);
    CHECK(!h.valid);
    heightUpdate(h, 500, t + 70000, 0.0f, 0.0f);
    CHECK(h.valid);
    checkNear(h.vz_ms, 0.0, 1e-6, "vz after gap");
    // 작동 범위 밖(1.8 m 초과, 0.1 m 미만)도 무효
    heightUpdate(h, 1900, t + 105000, 0.0f, 0.0f);
    CHECK(!h.valid);
    heightUpdate(h, 90, t + 140000, 0.0f, 0.0f);
    CHECK(!h.valid);
  });

  runCase("accel estimate: body acceleration from the flow velocity", [] {
    // c_pitch = v_x, c_roll = −v_y 이므로 a_x = dc_pitch/dt, a_y = −dc_roll/dt.
    FlowAccelEstimator a;
    flowAccelReset(a);
    float cr = 0.0f, cp = 0.0f;
    uint32_t t = 1000000;
    flowAccelUpdate(a, true, cr, cp, t);
    CHECK(!a.valid);                  // 첫 측정은 차분할 수 없다
    // 전방 0.5 m/s², 우측 0.2 m/s² 로 2 s 가속
    for (int i = 0; i < 20; i++) {
      t += 100000;
      cp += 0.5f * 0.1f;
      cr -= 0.2f * 0.1f;
      flowAccelUpdate(a, true, cr, cp, t);
    }
    CHECK(a.valid);
    checkNear(a.ax_ms2, 0.5, 0.02, "ax");
    checkNear(a.ay_ms2, 0.2, 0.02, "ay");
    const uint32_t lastT = a.t_us;
    // 무효 구간 하나는 추정을 지우지 않는다(나이로 만료는 호출자가 본다).
    flowAccelUpdate(a, false, 0.0f, 0.0f, t + 100000);
    CHECK(a.valid);
    CHECK(a.t_us == lastT);
    // 다음 유효 측정은 마지막 유효 측정과의 실제 간격(200 ms)으로 차분한다.
    t += 200000;
    cp += 0.5f * 0.2f;
    cr -= 0.2f * 0.2f;
    flowAccelUpdate(a, true, cr, cp, t);
    checkNear(a.ax_ms2, 0.5, 0.02, "ax after gap");
    // 간격이 너무 길면(> 350 ms) 차분하지 않고 다시 시작한다.
    t += 500000;
    flowAccelUpdate(a, true, cr, cp + 1.0f, t);
    CHECK(!a.valid);
    // 상한 3 m/s²
    for (int i = 0; i < 10; i++) {
      t += 100000;
      cp += 5.0f;
      flowAccelUpdate(a, true, cr, cp, t);
    }
    CHECK(a.valid);
    CHECK(a.ax_ms2 <= 3.0f + 1e-4f);
  });

  if (g_failures) {
    std::cerr << g_failures << " case(s) failed\n";
    return 1;
  }
  std::cout << "all flow odometry cases passed\n";
  return 0;
}
