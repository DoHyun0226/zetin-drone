#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "control_allocator.h"

namespace {

int test_count = 0;
int failure_count = 0;

[[noreturn]] void fail(const char *expression, int line,
                       const std::string &detail = {}) {
  std::ostringstream message;
  message << "line " << line << ": " << expression;
  if (!detail.empty()) message << " (" << detail << ")";
  throw std::runtime_error(message.str());
}

#define CHECK(expression) \
  do { \
    if (!(expression)) fail(#expression, __LINE__); \
  } while (false)

template <typename Actual, typename Expected>
void checkEqual(const Actual &actual, const Expected &expected,
                const char *expression, int line) {
  if (!(actual == expected)) {
    std::ostringstream detail;
    detail << "actual=" << actual << ", expected=" << expected;
    fail(expression, line, detail.str());
  }
}

#define CHECK_EQ(actual, expected) \
  checkEqual((actual), (expected), #actual " == " #expected, __LINE__)

void checkNear(float actual, float expected, float tolerance,
               const char *expression, int line) {
  if (!(std::fabs(actual - expected) <= tolerance)) {
    std::ostringstream detail;
    detail << "actual=" << actual << ", expected=" << expected
           << ", tolerance=" << tolerance;
    fail(expression, line, detail.str());
  }
}

#define CHECK_NEAR(actual, expected, tolerance) \
  checkNear((actual), (expected), (tolerance), \
            #actual " ~= " #expected, __LINE__)

void runCase(const std::string &name, const std::function<void()> &body) {
  test_count++;
  try {
    body();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception &error) {
    failure_count++;
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
  }
}

void checkMotors(const ControlAllocation &allocation, int m1, int m2, int m3,
                 int m4) {
  CHECK_EQ(allocation.motor[0], m1);
  CHECK_EQ(allocation.motor[1], m2);
  CHECK_EQ(allocation.motor[2], m3);
  CHECK_EQ(allocation.motor[3], m4);
}

void checkRange(const ControlAllocation &allocation, int low, int high) {
  for (int motor : allocation.motor) {
    CHECK(motor >= low);
    CHECK(motor <= high);
  }
}

}  // namespace

int main() {
  runCase("zero and pure-axis commands preserve the established motor signs", [] {
    // 모터 순서나 roll/pitch/yaw 부호가 바뀐 변이를 잡는다.
    ControlAllocation zero = allocateControl(0.0f, 0.0f, 0.0f, 1175, 1050, 1300);
    checkMotors(zero, 1175, 1175, 1175, 1175);
    CHECK_NEAR(zero.collective_us, 1175.0f, 1e-6f);
    CHECK_NEAR(zero.rp_scale, 1.0f, 1e-6f);
    CHECK_NEAR(zero.yaw_scale, 1.0f, 1e-6f);
    CHECK(!zero.scaled);

    ControlAllocation roll = allocateControl(10.0f, 0.0f, 0.0f, 1200, 1050, 1300);
    checkMotors(roll, 1210, 1190, 1190, 1210);

    ControlAllocation pitch = allocateControl(0.0f, 10.0f, 0.0f, 1200, 1050, 1300);
    checkMotors(pitch, 1190, 1210, 1190, 1210);

    ControlAllocation yaw = allocateControl(0.0f, 0.0f, 10.0f, 1200, 1050, 1300);
    checkMotors(yaw, 1190, 1190, 1210, 1210);
  });

  runCase("unsaturated combined command retains the old integer mix", [] {
    // 비포화 공식, float 합성, 반올림 동작이 바뀐 경우를 잡는다.
    ControlAllocation allocation =
        allocateControl(10.0f, 20.0f, 5.0f, 1175, 1050, 1300);
    checkMotors(allocation, 1160, 1180, 1150, 1210);
    CHECK_NEAR(allocation.collective_us, 1175.0f, 1e-6f);
    CHECK_NEAR(allocation.rp_scale, 1.0f, 1e-6f);
    CHECK_NEAR(allocation.yaw_scale, 1.0f, 1e-6f);
    CHECK(!allocation.scaled);

    // 옛 float 믹서는 M2 를 1179 로 반올림한다. 이 기여분을 합성 전에
    // double 로 올리면 그 모터가 1178 로 잘못 반올림된다.
    ControlAllocation fractional =
        allocateControl(-0.351f, -5.178f, -8.327f, 1175, 1050, 1300);
    checkMotors(fractional, 1188, 1179, 1172, 1161);
    CHECK_NEAR(fractional.collective_us, 1175.0f, 1e-6f);
    CHECK_NEAR(fractional.rp_scale, 1.0f, 1e-6f);
    CHECK_NEAR(fractional.yaw_scale, 1.0f, 1e-6f);
    CHECK(!fractional.scaled);
  });

  runCase("yaw saturation leaves allocated roll-pitch pair differences intact", [] {
    // 옛 균일 자세 스케일이나 yaw 에 우선순위를 주는 allocator 를 잡는다.
    ControlAllocation allocation =
        allocateControl(30.0f, 0.0f, 100.0f, 1100, 1000, 1200);
    checkMotors(allocation, 1060, 1000, 1140, 1200);
    CHECK_NEAR(allocation.rp_scale, 1.0f, 1e-6f);
    CHECK_NEAR(allocation.yaw_scale, 0.7f, 1e-6f);
    CHECK(allocation.scaled);
    CHECK_EQ(allocation.motor[0] - allocation.motor[1], 60);
    CHECK_EQ(allocation.motor[3] - allocation.motor[2], 60);
  });

  runCase("roll-pitch is scaled only after its own span exceeds the motor range", [] {
    // roll-pitch span 스케일 누락이나 yaw 전용 스케일을 RP 에 적용한 경우를 잡는다.
    ControlAllocation allocation =
        allocateControl(150.0f, 0.0f, 0.0f, 1100, 1000, 1200);
    checkMotors(allocation, 1200, 1000, 1000, 1200);
    CHECK_NEAR(allocation.rp_scale, 2.0f / 3.0f, 1e-6f);
    CHECK_NEAR(allocation.yaw_scale, 1.0f, 1e-6f);
    CHECK(allocation.scaled);
  });

  runCase("positive and negative yaw saturate as diagonal-pair mirrors", [] {
    // yaw 쌍의 비대칭 부호나 한쪽만 푸는 yaw 스케일 솔버를 잡는다.
    ControlAllocation positive =
        allocateControl(30.0f, 0.0f, 100.0f, 1100, 1000, 1200);
    ControlAllocation negative =
        allocateControl(30.0f, 0.0f, -100.0f, 1100, 1000, 1200);
    checkMotors(positive, 1060, 1000, 1140, 1200);
    checkMotors(negative, 1200, 1140, 1000, 1060);
    CHECK_NEAR(positive.yaw_scale, 0.7f, 1e-6f);
    CHECK_NEAR(negative.yaw_scale, 0.7f, 1e-6f);
    CHECK_NEAR(positive.rp_scale, negative.rp_scale, 1e-6f);
  });

  runCase("collective moves into the feasible window before any scaling", [] {
    // collective 이동 전 스케일링이나 모터별 클리핑을 잡는다.
    ControlAllocation allocation =
        allocateControl(30.0f, 0.0f, 0.0f, 1040, 1050, 1250);
    checkMotors(allocation, 1110, 1050, 1050, 1110);
    CHECK_NEAR(allocation.collective_us, 1080.0f, 1e-6f);
    CHECK_NEAR(allocation.rp_scale, 1.0f, 1e-6f);
    CHECK_NEAR(allocation.yaw_scale, 1.0f, 1e-6f);
    CHECK(!allocation.scaled);
  });

  runCase("finite extremes and invalid limits always produce normalized in-range motors", [] {
    // 한계 정규화 누락, 마지막 constrain 누락, 유한하지 않은 span 을 잡는다.
    const float extreme = std::numeric_limits<float>::max() / 4.0f;
    for (float sign : {-1.0f, 1.0f}) {
      ControlAllocation allocation = allocateControl(
          sign * extreme, -sign * extreme, sign * extreme, 5000, 1050, 1300);
      checkRange(allocation, 1050, 1300);
      CHECK(std::isfinite(allocation.collective_us));
      CHECK(allocation.rp_scale >= 0.0f && allocation.rp_scale <= 1.0f);
      CHECK(allocation.yaw_scale >= 0.0f && allocation.yaw_scale <= 1.0f);
    }

    ControlAllocation collapsed =
        allocateControl(30.0f, 0.0f, 0.0f, 1500, 1800, 1200);
    checkMotors(collapsed, 1800, 1800, 1800, 1800);
    checkRange(collapsed, 1800, 1800);

    ControlAllocation low =
        allocateControl(0.0f, 0.0f, 0.0f, -100000, 500, 2500);
    ControlAllocation high =
        allocateControl(0.0f, 0.0f, 0.0f, 100000, 500, 2500);
    checkMotors(low, 1000, 1000, 1000, 1000);
    checkMotors(high, 2000, 2000, 2000, 2000);
  });

  std::cout << "[SUMMARY] " << test_count << " cases, " << failure_count
            << " failures\n";
  return failure_count == 0 ? 0 : 1;
}
