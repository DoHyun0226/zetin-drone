#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "yaw_authority.h"

namespace {

int failures = 0;
int cases = 0;

void runCase(const char *name, const std::function<void()> &body) {
  ++cases;
  try {
    body();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception &error) {
    ++failures;
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
  }
}

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) throw std::runtime_error(#expression);                 \
  } while (false)

void checkNear(float actual, float expected, float tolerance,
               const char *expression) {
  if (std::fabs(actual - expected) > tolerance) {
    throw std::runtime_error(
        std::string(expression) + " actual=" + std::to_string(actual) +
        " expected=" + std::to_string(expected));
  }
}

#define CHECK_NEAR(actual, expected, tolerance) \
  checkNear((actual), (expected), (tolerance), #actual " ~= " #expected)

YawAuthorityTracker limitedAt(uint32_t start_ms) {
  YawAuthorityTracker tracker = {};
  updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f, true, false, false,
                     start_ms);
  updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f, true, false, false,
                     start_ms + 150U);
  CHECK(tracker.state == YAW_AUTH_LIMITED);
  return tracker;
}

}  // namespace

int main() {
  runCase("full authority integrates the literal delta", [] {
    CHECK_NEAR(conditionalAxisIntegral(10.0f, 2.0f, 15.0f, 1.0f,
                                       1200, 50.0f),
               12.0f, 1e-6f);
  });

  runCase("limited authority blocks a delta that increases output magnitude", [] {
    CHECK_NEAR(conditionalAxisIntegral(10.0f, 2.0f, 15.0f, 0.4f,
                                       1200, 50.0f),
               10.0f, 1e-6f);
    CHECK_NEAR(conditionalAxisIntegral(-10.0f, -2.0f, -15.0f, 0.4f,
                                       1200, 50.0f),
               -10.0f, 1e-6f);
  });

  runCase("limited authority permits a delta that unwinds requested output", [] {
    CHECK_NEAR(conditionalAxisIntegral(10.0f, -2.0f, 15.0f, 0.4f,
                                       1200, 50.0f),
               8.0f, 1e-6f);
    CHECK_NEAR(conditionalAxisIntegral(-10.0f, 2.0f, -15.0f, 0.4f,
                                       1200, 50.0f),
               -8.0f, 1e-6f);
  });

  runCase("integral contribution clamps at the literal 50us limit", [] {
    CHECK_NEAR(conditionalAxisIntegral(49.0f, 5.0f, 20.0f, 1.0f,
                                       1200, 50.0f),
               50.0f, 1e-6f);
    CHECK_NEAR(conditionalAxisIntegral(-49.0f, -5.0f, -20.0f, 1.0f,
                                       1200, 50.0f),
               -50.0f, 1e-6f);
  });

  runCase("throttle at or below 1100us resets the integral", [] {
    CHECK_NEAR(conditionalAxisIntegral(17.0f, 2.0f, 15.0f, 1.0f,
                                       1100, 50.0f),
               0.0f, 1e-6f);
    CHECK_NEAR(conditionalAxisIntegral(17.0f, 2.0f, 15.0f, 1.0f,
                                       1099, 50.0f),
               0.0f, 1e-6f);
  });

  runCase("entry requires 150ms continuously and changes at the boundary", [] {
    YawAuthorityTracker tracker = {};
    updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f, true, false, false, 100U);
    CHECK(tracker.state == YAW_AUTH_NORMAL);
    updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f, true, false, false, 249U);
    CHECK(tracker.state == YAW_AUTH_NORMAL);
    updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f, true, false, false, 250U);
    CHECK(tracker.state == YAW_AUTH_LIMITED);
  });

  runCase("transient scale recovery clears the entry timer", [] {
    YawAuthorityTracker tracker = {};
    updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f, true, false, false, 100U);
    updateYawAuthority(tracker, 0.6f, 0.0f, 20.0f, true, false, false, 249U);
    updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f, true, false, false, 300U);
    updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f, true, false, false, 449U);
    CHECK(tracker.state == YAW_AUTH_NORMAL);
    updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f, true, false, false, 450U);
    CHECK(tracker.state == YAW_AUTH_LIMITED);
  });

  runCase("limited recovers only after 500ms of scale and rate margin", [] {
    YawAuthorityTracker tracker = limitedAt(100U);
    updateYawAuthority(tracker, 0.9f, 0.0f, 10.0f, true, false, false, 300U);
    CHECK(tracker.state == YAW_AUTH_LIMITED);
    updateYawAuthority(tracker, 0.9f, 0.0f, 9.0f, true, false, false, 301U);
    CHECK(tracker.state == YAW_AUTH_RECOVERING);
    updateYawAuthority(tracker, 0.9f, 0.0f, 9.0f, true, false, false, 800U);
    CHECK(tracker.state == YAW_AUTH_RECOVERING);
    updateYawAuthority(tracker, 0.9f, 0.0f, 9.0f, true, false, false, 801U);
    CHECK(tracker.state == YAW_AUTH_NORMAL);
  });

  runCase("recovery failure returns immediately to limited", [] {
    YawAuthorityTracker tracker = limitedAt(100U);
    updateYawAuthority(tracker, 0.9f, 0.0f, 9.0f, true, false, false, 300U);
    CHECK(tracker.state == YAW_AUTH_RECOVERING);
    updateYawAuthority(tracker, 0.89f, 0.0f, 9.0f, true, false, false, 301U);
    CHECK(tracker.state == YAW_AUTH_LIMITED);
  });

  runCase("pilot yaw input immediately bypasses and resets limitation", [] {
    YawAuthorityTracker tracker = limitedAt(100U);
    updateYawAuthority(tracker, 0.2f, 45.0f, 20.0f, true, false, false, 251U);
    CHECK(tracker.state == YAW_AUTH_NORMAL);
    const YawAuthorityCommand command = applyYawAuthority(
        tracker.state, 22.0f, 17.0f, 45.0f, false);
    CHECK(!command.hold);
    CHECK_NEAR(command.target_angle_deg, 17.0f, 1e-6f);
    CHECK_NEAR(command.target_rate_dps, 45.0f, 1e-6f);
  });

  runCase("failsafe disarm and safety lock each reset to normal", [] {
    YawAuthorityTracker failsafe = limitedAt(100U);
    updateYawAuthority(failsafe, 0.2f, 0.0f, 20.0f,
                       true, true, false, 251U);
    CHECK(failsafe.state == YAW_AUTH_NORMAL);

    YawAuthorityTracker disarmed = limitedAt(100U);
    updateYawAuthority(disarmed, 0.2f, 0.0f, 20.0f,
                       false, false, false, 251U);
    CHECK(disarmed.state == YAW_AUTH_NORMAL);

    YawAuthorityTracker locked = limitedAt(100U);
    updateYawAuthority(locked, 0.2f, 0.0f, 20.0f,
                       true, false, true, 251U);
    CHECK(locked.state == YAW_AUTH_NORMAL);
  });

  runCase("limited and recovering slave heading and command zero rate", [] {
    const YawAuthorityCommand limited = applyYawAuthority(
        YAW_AUTH_LIMITED, 22.0f, 137.0f, -180.0f, true);
    CHECK(!limited.hold);
    CHECK_NEAR(limited.target_angle_deg, 22.0f, 1e-6f);
    CHECK_NEAR(limited.target_rate_dps, 0.0f, 1e-6f);

    const YawAuthorityCommand recovering = applyYawAuthority(
        YAW_AUTH_RECOVERING, 23.0f, 137.0f, 180.0f, true);
    CHECK(!recovering.hold);
    CHECK_NEAR(recovering.target_angle_deg, 23.0f, 1e-6f);
    CHECK_NEAR(recovering.target_rate_dps, 0.0f, 1e-6f);
  });

  runCase("yaw override cannot bypass a limited state", [] {
    const YawAuthorityCommand command = applyYawAuthority(
        YAW_AUTH_LIMITED, -30.0f, 120.0f, -180.0f, true);
    CHECK(!command.hold);
    CHECK_NEAR(command.target_angle_deg, -30.0f, 1e-6f);
    CHECK_NEAR(command.target_rate_dps, 0.0f, 1e-6f);
  });

  runCase("entry duration remains correct across uint32 timestamp wrap", [] {
    YawAuthorityTracker tracker = {};
    constexpr uint32_t start = 0xFFFFFFC0U;
    updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f,
                       true, false, false, start);
    updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f,
                       true, false, false, start + 149U);
    CHECK(tracker.state == YAW_AUTH_NORMAL);
    updateYawAuthority(tracker, 0.5f, 0.0f, 20.0f,
                       true, false, false, start + 150U);
    CHECK(tracker.state == YAW_AUTH_LIMITED);
  });

  if (failures != 0) {
    std::cerr << failures << " yaw-authority case(s) failed\n";
    return 1;
  }
  std::cout << cases << "/" << cases << " yaw-authority cases passed\n";
  return 0;
}
