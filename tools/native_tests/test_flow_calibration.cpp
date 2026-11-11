#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "flow_calibration.h"

// 합성 보정 데이터. 펌웨어와 같은 순서로 쌓는다: 광류 한 구간마다 지연 후보
// k 별로 [t − dt − k·10ms, t − k·10ms] 의 자이로 적분을 넣는다. 자이로는
// 연속 각속도 모델을 해석적으로 적분해 만들고, 광류는 참 M 과 참 지연으로
// 만든 뒤 정수로 반올림한다. 펌웨어의 풀이를 재사용하지 않는다.

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
static const double kCountsPerRad = 9.5 * 180.0 / kPi;  // Matek 9~10 counts/deg

// 각속도 성분: 두 축이 서로 다른 주파수의 사인 합이다(축 독립 자극).
struct Motion {
  double amp_x_rad_s = 1.2, freq_x_hz = 0.7, phase_x = 0.0;
  double amp_y_rad_s = 1.1, freq_y_hz = 0.45, phase_y = 1.3;
  double leak_x_into_y = 0.0;   // y = leak·x + (1-leak)·y_own (공선 자극)
};

// 각속도의 해석적 적분(누적각, rad).
static double angleX(const Motion &m, double t) {
  const double w = 2.0 * kPi * m.freq_x_hz;
  return m.amp_x_rad_s / w * (1.0 - std::cos(w * t + m.phase_x));
}
static double angleYOwn(const Motion &m, double t) {
  const double w = 2.0 * kPi * m.freq_y_hz;
  return m.amp_y_rad_s / w * (1.0 - std::cos(w * t + m.phase_y));
}
static double angleY(const Motion &m, double t) {
  return m.leak_x_into_y * angleX(m, t) +
         (1.0 - m.leak_x_into_y) * angleYOwn(m, t);
}

// 결정적 의사난수(재현성). [-1, 1).
struct Lcg {
  uint32_t s = 12345u;
  double next() {
    s = s * 1664525u + 1013904223u;
    return ((s >> 8) & 0xFFFFFF) / double(0x800000) - 1.0;
  }
};

struct SynthConfig {
  Motion motion;
  double m_true[2][2] = {{0.0, -kCountsPerRad}, {kCountsPerRad, 0.0}};
  double delay_true_s = 0.030;
  double noise_counts = 2.0;
  int samples = 250;
  double dt_s = 0.100;
};

static FlowCalibrator synthesize(const SynthConfig &c) {
  FlowCalibrator cal;
  flowCalReset(cal);
  Lcg rng;
  for (int i = 1; i <= c.samples; i++) {
    const double t = 1.0 + i * c.dt_s;
    // 참 광류: 센서는 [t − dt − τ, t − τ] 회전을 본다.
    const double t1 = t - c.delay_true_s, t0 = t1 - c.dt_s;
    const double gx = angleX(c.motion, t1) - angleX(c.motion, t0);
    const double gy = angleY(c.motion, t1) - angleY(c.motion, t0);
    const double fx = c.m_true[0][0] * gx + c.m_true[0][1] * gy +
                      c.noise_counts * rng.next();
    const double fy = c.m_true[1][0] * gx + c.m_true[1][1] * gy +
                      c.noise_counts * rng.next();
    const int32_t fxi = (int32_t)std::lround(fx);
    const int32_t fyi = (int32_t)std::lround(fy);
    for (int k = 0; k < FLOW_CAL_DELAY_CANDIDATES; k++) {
      const double d = k * (FLOW_CAL_DELAY_STEP_US * 1e-6);
      const double b1 = t - d, b0 = b1 - c.dt_s;
      flowCalAddSample(cal, k, fxi, fyi,
                       (float)(angleX(c.motion, b1) - angleX(c.motion, b0)),
                       (float)(angleY(c.motion, b1) - angleY(c.motion, b0)));
    }
  }
  return cal;
}

int main() {
  runCase("swapped/flipped axes and 30ms delay are recovered", [] {
    SynthConfig c;
    const FlowCalResult r = flowCalSolve(synthesize(c));
    CHECK(r.ok);
    CHECK(r.reason == FLOW_CAL_OK);
    CHECK(r.delay_us == 30000);
    const double tol = 0.05 * kCountsPerRad;
    checkNear(r.m[0][0], 0.0, tol, "m00");
    checkNear(r.m[0][1], -kCountsPerRad, tol, "m01");
    checkNear(r.m[1][0], kCountsPerRad, tol, "m10");
    checkNear(r.m[1][1], 0.0, tol, "m11");
    CHECK(r.samples == 250);
  });

  runCase("identity mapping with zero delay", [] {
    SynthConfig c;
    c.m_true[0][0] = kCountsPerRad; c.m_true[0][1] = 0.0;
    c.m_true[1][0] = 0.0; c.m_true[1][1] = -kCountsPerRad;
    c.delay_true_s = 0.0;
    const FlowCalResult r = flowCalSolve(synthesize(c));
    CHECK(r.ok);
    CHECK(r.delay_us == 0);
    checkNear(r.m[0][0], kCountsPerRad, 0.05 * kCountsPerRad, "m00");
    checkNear(r.m[1][1], -kCountsPerRad, 0.05 * kCountsPerRad, "m11");
  });

  runCase("too few samples are refused", [] {
    SynthConfig c;
    c.samples = 100;
    const FlowCalResult r = flowCalSolve(synthesize(c));
    CHECK(!r.ok);
    CHECK(r.reason == FLOW_CAL_TOO_FEW);
  });

  runCase("shaking only one axis is refused", [] {
    SynthConfig c;
    c.motion.amp_y_rad_s = 0.0;
    const FlowCalResult r = flowCalSolve(synthesize(c));
    CHECK(!r.ok);
    CHECK(r.reason == FLOW_CAL_LOW_EXCITATION);
  });

  runCase("nearly collinear two-axis motion is refused", [] {
    // 검토 반례: 두 축 RMS 는 충분하지만 둘째 축 독립 성분이 작다.
    SynthConfig c;
    c.motion.leak_x_into_y = 0.97;
    const FlowCalResult r = flowCalSolve(synthesize(c));
    CHECK(!r.ok);
    CHECK(r.reason == FLOW_CAL_COLLINEAR);
  });

  runCase("noise-only flow is refused as a poor fit", [] {
    SynthConfig c;
    c.m_true[0][0] = c.m_true[0][1] = c.m_true[1][0] = c.m_true[1][1] = 0.0;
    c.noise_counts = 200.0;
    const FlowCalResult r = flowCalSolve(synthesize(c));
    CHECK(!r.ok);
    CHECK(r.reason == FLOW_CAL_POOR_FIT);
  });

  runCase("a mixed (non-permutation) mapping is refused", [] {
    SynthConfig c;
    c.m_true[0][0] = kCountsPerRad; c.m_true[0][1] = 0.6 * kCountsPerRad;
    c.m_true[1][0] = 0.0; c.m_true[1][1] = kCountsPerRad;
    const FlowCalResult r = flowCalSolve(synthesize(c));
    CHECK(!r.ok);
    CHECK(r.reason == FLOW_CAL_NOT_PERMUTATION);
  });

  runCase("a scale outside 3..30 counts/deg is refused", [] {
    SynthConfig c;
    const double tiny = 1.5 * 180.0 / kPi;
    c.m_true[0][1] = -tiny; c.m_true[1][0] = tiny;
    c.noise_counts = 0.2;
    const FlowCalResult r = flowCalSolve(synthesize(c));
    CHECK(!r.ok);
    CHECK(r.reason == FLOW_CAL_SCALE_RANGE);
  });

  runCase("reset clears the accumulators", [] {
    SynthConfig c;
    FlowCalibrator cal = synthesize(c);
    flowCalReset(cal);
    const FlowCalResult r = flowCalSolve(cal);
    CHECK(!r.ok);
    CHECK(r.samples == 0);
  });

  runCase("inverse of the recovered matrix maps flow back to rotation", [] {
    SynthConfig c;
    const FlowCalResult r = flowCalSolve(synthesize(c));
    CHECK(r.ok);
    float inv[2][2];
    CHECK(flowCalInvert(r.m, inv));
    // f = M·g 를 u = M⁻¹·f 로 되돌리면 g 가 나온다.
    const float gx = 0.1f, gy = -0.05f;
    const float fx = r.m[0][0] * gx + r.m[0][1] * gy;
    const float fy = r.m[1][0] * gx + r.m[1][1] * gy;
    checkNear(inv[0][0] * fx + inv[0][1] * fy, gx, 1e-5, "ux");
    checkNear(inv[1][0] * fx + inv[1][1] * fy, gy, 1e-5, "uy");
    float singular[2][2] = {{1.0f, 2.0f}, {2.0f, 4.0f}};
    CHECK(!flowCalInvert(singular, inv));
  });

  if (g_failures) {
    std::cerr << g_failures << " case(s) failed\n";
    return 1;
  }
  std::cout << "all flow calibration cases passed\n";
  return 0;
}
