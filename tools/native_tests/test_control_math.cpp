#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dual_imu_cascade_pwm.ino"

namespace {

int test_count = 0;
int failure_count = 0;

[[noreturn]] void fail(const char *expression, int line, const std::string &detail = {}) {
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

void checkNear(double actual, double expected, double tolerance,
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

void checkMotors(const MotorMix &mix, int m1, int m2, int m3, int m4) {
  CHECK_EQ(mix.motor[0], m1);
  CHECK_EQ(mix.motor[1], m2);
  CHECK_EQ(mix.motor[2], m3);
  CHECK_EQ(mix.motor[3], m4);
}

void checkAllMotorsInRange(const MotorMix &mix, int low, int high) {
  for (int motor : mix.motor) {
    CHECK(motor >= low);
    CHECK(motor <= high);
  }
}

void sendRc(const std::string &command) {
  std::vector<char> buffer(command.begin(), command.end());
  buffer.push_back('\0');
  handleRcCommand(buffer.data());
}

void sendRcr(const std::string &command) {
  std::vector<char> buffer(command.begin(), command.end());
  buffer.push_back('\0');
  handleRcrCommand(buffer.data());
}

void sendUdpCommandOnce(const std::string &command) {
  wifi_udp_fake::incoming_packet = command;
  arduino_fake::stop_on_task_delay = true;
  try {
    udp_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
    arduino_fake::stop_on_task_delay = false;
    return;
  } catch (...) {
    arduino_fake::stop_on_task_delay = false;
    throw;
  }
}

std::vector<std::string> splitCsv(const std::string &payload) {
  std::vector<std::string> fields;
  std::istringstream stream(payload);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

void runPidTicks(uint32_t ticks, uint32_t us_per_tick = 1000) {
  arduino_fake::tick_index = 0;
  arduino_fake::tick_limit = ticks;
  arduino_fake::us_per_tick = us_per_tick;
  try {
    pid_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
  } catch (...) {
    arduino_fake::us_per_tick = 0;
    arduino_fake::tick_limit = 0;
    throw;
  }
  arduino_fake::us_per_tick = 0;
  arduino_fake::tick_limit = 0;
}

void resetRcState() {
  lastRcSeq = 0;
  rcSeqValid = false;
  rcTotalPkts = 0;
  rcDroppedPkts = 0;
  targetAngleX = 9.0f;
  targetAngleY = 8.0f;
  targetAngleZ = 7.0f;
  lastRcMs = 6;
  arduino_fake::millis_value = 100;
  targetYawRate = 0.0f;
}

struct GainSlot {
  const char *name;
  volatile float *value;
};

GainSlot gain_slots[] = {
  {"Kp_Angle_Roll", &Kp_Angle_Roll},
  {"Kp_Angle_Pitch", &Kp_Angle_Pitch},
  {"Kp_Angle_Yaw", &Kp_Angle_Yaw},
  {"Kp_Rate_Roll", &Kp_Rate_Roll},
  {"Ki_Rate_Roll", &Ki_Rate_Roll},
  {"Kd_Rate_Roll", &Kd_Rate_Roll},
  {"Kp_Rate_Pitch", &Kp_Rate_Pitch},
  {"Ki_Rate_Pitch", &Ki_Rate_Pitch},
  {"Kd_Rate_Pitch", &Kd_Rate_Pitch},
  {"Kp_Rate_Yaw", &Kp_Rate_Yaw},
  {"Ki_Rate_Yaw", &Ki_Rate_Yaw},
  {"Kd_Rate_Yaw", &Kd_Rate_Yaw},
};

void seedGains() {
  for (std::size_t index = 0; index < std::size(gain_slots); index++) {
    *gain_slots[index].value = 10.0f + static_cast<float>(index);
  }
}

void checkGainCommand(const char *prefix, const std::vector<std::size_t> &changed) {
  seedGains();
  fs_phase = FS_NONE;
  std::vector<float> before;
  for (const auto &slot : gain_slots) {
    before.push_back(static_cast<float>(*slot.value));
  }

  constexpr float commanded = 77.25f;
  const std::string command = std::string(prefix) + " 77.25";
  handleGainCommand(command.c_str());

  for (std::size_t index = 0; index < std::size(gain_slots); index++) {
    bool should_change = false;
    for (std::size_t changed_index : changed) {
      if (changed_index == index) should_change = true;
    }
    const float expected = should_change ? commanded : before[index];
    CHECK_NEAR(*gain_slots[index].value, expected, 1e-6f);
  }
}

inv_imu_sensor_event_t eventWith(
    int16_t gx, int16_t gy, int16_t gz,
    int16_t ax = 0, int16_t ay = 0, int16_t az = 0) {
  inv_imu_sensor_event_t event = {};
  event.gyro[0] = gx;
  event.gyro[1] = gy;
  event.gyro[2] = gz;
  event.accel[0] = ax;
  event.accel[1] = ay;
  event.accel[2] = az;
  return event;
}

void setFakeAccelMagnitude(float accel_g, uint32_t tick) {
  const int dither = (tick & 1U) ? 1 : -1;
  const int16_t raw =
      static_cast<int16_t>(std::lround(accel_g / ACCEL_SCALE) + dither);
  IMU1.next_event = eventWith(0, 0, 0, 0, 0, raw);
  IMU2.next_event = eventWith(0, 0, 0, 0, 0, -raw);
}

void setFakeYawRateAndAccel(float body_yaw_dps, float accel_g,
                            uint32_t tick) {
  const int dither = (tick & 1U) ? 1 : -1;
  const int16_t accel_raw =
      static_cast<int16_t>(std::lround(accel_g / ACCEL_SCALE) + dither);
  const int16_t imu1_yaw_raw =
      static_cast<int16_t>(std::lround(-body_yaw_dps / GYRO_SCALE));
  IMU1.next_event = eventWith(0, 0, imu1_yaw_raw, 0, 0, accel_raw);
  IMU2.next_event = eventWith(0, 0, -imu1_yaw_raw, 0, 0, -accel_raw);
}

void prepareYawAuthorityFlight() {
  arduino_fake::reset();
  fault_rc = fault_imu1 = fault_imu2 = fault_disagree = fault_attitude = false;
  imu1_frozen_now = imu2_frozen_now = imu_disagree_now = false;
  calibration_ok = true;
  mag_calibrating = false;
  mag_enabled = false;
  safety_lock = true;
  safety_arm_requested = false;
  safety_disarm_requested = false;
  fs_phase = FS_NONE;
  angleX = angleY = angleZ = 0.0f;
  gyro_bias1[0] = gyro_bias1[1] = gyro_bias1[2] = 0.0f;
  gyro_bias2[0] = gyro_bias2[1] = gyro_bias2[2] = 0.0f;
  setFakeYawRateAndAccel(20.0f, 1.0f, 0U);

  sendUdpCommandOnce("start");
  CHECK(safety_arm_requested);
  base_throttle = 1200;
  min_throttle = 1100;
  max_throttle = 1300;
  lastRcMs = millis();
  Kp_Angle_Roll = Kp_Angle_Pitch = Kp_Angle_Yaw = 0.0f;
  Kp_Rate_Roll = Kp_Rate_Pitch = 0.0f;
  Ki_Rate_Roll = Ki_Rate_Pitch = Ki_Rate_Yaw = 0.0f;
  Kd_Rate_Roll = Kd_Rate_Pitch = Kd_Rate_Yaw = 0.0f;
  Kp_Rate_Yaw = 100.0f;
  targetAngleX = targetAngleY = targetAngleZ = 0.0f;
  targetYawRate = 0.0f;
  yaw_hold_now = false;
  yaw_hold_override = false;
  yaw_authority_state = YAW_AUTH_NORMAL;
  iTermRoll = iTermPitch = iTermYaw = 0.0f;
  mixer_scaled = false;
  mixer_yaw_scale = 1.0f;
}

void primeHoverEstimate(float hover_us) {
  hoverTracker = {};
  hoverTracker.estimate_us = hover_us;
  hoverTracker.initialized = true;
  hoverTracker.timing_initialized = true;
  hoverTracker.valid = true;
  hoverTracker.eligible_ms = HOVER_VALID_MS;
  hover_est = hover_us;
  hover_valid = true;
}

void prepareFailsafeFlight(int throttle_us) {
  arduino_fake::reset();
  fault_rc = false;
  fault_imu1 = false;
  fault_imu2 = false;
  fault_disagree = false;
  fault_attitude = false;
  imu1_frozen_now = false;
  imu2_frozen_now = false;
  imu_disagree_now = false;
  calibration_ok = true;
  mag_calibrating = false;
  safety_lock = true;
  fs_phase = FS_NONE;
  angleX = angleY = angleZ = 0.0f;
  gyro_bias1[0] = gyro_bias1[1] = gyro_bias1[2] = 0.0f;
  gyro_bias2[0] = gyro_bias2[1] = gyro_bias2[2] = 0.0f;
  setFakeAccelMagnitude(1.0f, 0);
  sendUdpCommandOnce("start");
  CHECK(safety_lock);
  CHECK(safety_arm_requested);
  runPidTicks(1);
  CHECK(!safety_lock);
  base_throttle = throttle_us;
  primeHoverEstimate((float)throttle_us);
  lastRcMs = 0;
  arduino_fake::millis_value = RC_TIMEOUT_MS + 100;
  arduino_fake::micros_value = arduino_fake::millis_value * 1000U;
}

void prepareResumeCommandState() {
  arduino_fake::reset();
  safety_lock = false;
  safety_disarm_requested = false;
  safety_arm_requested = false;
  failsafe_resume_requested = false;
  fs_phase = FS_DESCENDING;
  fault_rc = true;
  fault_imu1 = false;
  fault_imu2 = false;
  fault_disagree = false;
  fault_attitude = false;
  imu1_frozen_now = false;
  imu2_frozen_now = false;
  imu_disagree_now = false;
  active_imus = 2;
  calibration_ok = true;
  mag_calibrating = false;
  angleX = angleY = angleZ = 0.0f;
  gyro_bias1[0] = gyro_bias1[1] = gyro_bias1[2] = 0.0f;
  gyro_bias2[0] = gyro_bias2[1] = gyro_bias2[2] = 0.0f;
  setFakeAccelMagnitude(1.0f, 0);
  primeHoverEstimate(1360.0f);
  base_throttle = 1360 - FS_DESCENT_DELTA_US;
  min_throttle = max(1050, base_throttle - CTRL_MARGIN);
  max_throttle = min(1900, base_throttle + CTRL_MARGIN);
  arduino_fake::millis_value = 1000;
  arduino_fake::micros_value = 1000000;
  lastRcMs = arduino_fake::millis_value;
  arduino_fake::serial_output.clear();
}

void prepareStartCommandState(uint8_t phase) {
  arduino_fake::reset();
  safety_lock = true;
  safety_disarm_requested = false;
  safety_arm_requested = false;
  failsafe_resume_requested = false;
  fs_phase = phase;
  fault_rc = false;
  fault_imu1 = false;
  fault_imu2 = false;
  fault_disagree = false;
  fault_attitude = false;
  imu1_frozen_now = false;
  imu2_frozen_now = false;
  imu_disagree_now = false;
  active_imus = 2;
  calibration_ok = true;
  mag_calibrating = false;
  angleX = angleY = angleZ = 0.0f;
  gyro_bias1[0] = gyro_bias1[1] = gyro_bias1[2] = 0.0f;
  gyro_bias2[0] = gyro_bias2[1] = gyro_bias2[2] = 0.0f;
  setFakeAccelMagnitude(1.0f, 0);
}

std::size_t countLogOccurrences(const std::string &needle) {
  const std::string &log = arduino_fake::serial_output;
  std::size_t hits = 0;
  for (std::size_t at = log.find(needle);
       at != std::string::npos;
       at = log.find(needle, at + needle.size())) {
    hits++;
  }
  return hits;
}

void checkInFlightOverTiltCut(
    const char *axis, float injected_roll_deg, float injected_pitch_deg) {
  prepareFailsafeFlight(1360);
  lastRcMs = millis();
  runPidTicks(1);
  CHECK(!safety_lock);
  for (int motor : motorOut) CHECK(motor > 1000);

  angleX = injected_roll_deg;
  angleY = injected_pitch_deg;
  lastRcMs = millis();
  arduino_fake::serial_output.clear();
  runPidTicks(1);

  std::cout << "[CONTROL] in-flight over-tilt axis=" << axis
            << " actual_fault=" << static_cast<int>(fault_attitude)
            << " actual_lock=" << static_cast<int>(safety_lock)
            << " actual_motors=" << motorOut[0] << "," << motorOut[1]
            << "," << motorOut[2] << "," << motorOut[3]
            << " expected_fault=1 expected_lock=1"
            << " expected_motors=1000,1000,1000,1000\n";
  CHECK_EQ(fault_attitude, true);
  CHECK_EQ(safety_lock, true);
  for (int motor : motorOut) CHECK_EQ(motor, 1000);
  CHECK_EQ(countLogOccurrences("[FAULT] OVER-TILT"),
           static_cast<std::size_t>(1));
}

}  // namespace

int main() {
  runCase("pid_task 1000 tick은 millis를 약 1000ms 전진시킨다", [] {
    arduino_fake::reset();
    safety_lock = true;
    const uint32_t start_ms = millis();

    runPidTicks(1000);

    const uint32_t elapsed_ms = millis() - start_ms;
    CHECK(elapsed_ms >= 1000U);
    CHECK(elapsed_ms <= 1001U);
  });

  runCase("mix: zero command keeps all motors equal", [] {
    MotorMix mix = mixAndDesaturate(0, 0, 0, 1175, 1050, 1300);
    checkMotors(mix, 1175, 1175, 1175, 1175);
    CHECK(!mix.scaled);
    CHECK_NEAR(mix.collective_us, 1175.0f, 1e-6f);
  });

  runCase("mix: applied collective reports a lower-bound shift exactly", [] {
    MotorMix mix = mixAndDesaturate(-50, 50, 50, 1240, 1090, 1430);

    // descent=1280us, dip=40us, attitude diff minimum=-150us. The probe-only
    // widened lower bound preserves the full dip. Without that extra 40us of
    // lower window this same reachable attitude state delivered 0us while
    // scaled remained false.
    checkMotors(mix, 1090, 1290, 1290, 1290);
    CHECK_NEAR(mix.collective_us, 1240.0f, 1e-6f);
    CHECK_NEAR(1280.0f - mix.collective_us, 40.0f, 1e-6f);
    CHECK(!mix.scaled);
  });

  runCase("mix: margin을 넘은 자세 차동은 전달률 미달로 프로브를 폐기한다", [] {
    // 각 축의 50us 적분 한계에 같은 부호의 10us P 항이 더해진 경우를
    // 모델링한다. widened window에서도 minDiff=-180us라 40us 중 10us만
    // 전달되지만 attitude span은 맞아서 scaled는 계속 false다.
    MotorMix mix = mixAndDesaturate(-60, 60, 60, 1240, 1090, 1430);
    const float delivered_us = 1280.0f - mix.collective_us;
    CHECK_NEAR(mix.collective_us, 1270.0f, 1e-6f);
    CHECK_NEAR(delivered_us, 10.0f, 1e-6f);
    CHECK(!mix.scaled);
    CHECK(delivered_us < FS_PROBE_MIN_DELIVERY_FRAC * 40.0f);

    LandProbeConfig config = FS_LAND_PROBE_CONFIG;
    config.dip_us = 40;
    LandDetector det = {};
    CHECK(!updateLandDetector(det, 1.0f, 1000, 1280, config));
    CHECK(!updateLandDetector(det, 1.0f, 1030, 1280, config));
    recordLandDetectorProbeDelivery(
        det, 1030,
        delivered_us >= FS_PROBE_MIN_DELIVERY_FRAC * config.dip_us,
        config);
    CHECK(!updateLandDetector(det, 1.0f, 1120, 1280, config));
    CHECK_EQ((int)det.probe_state, (int)FS_PROBE_BLOCKED);
    CHECK_EQ((int)det.no_response_count, 0);
  });

  runCase("mix: pure roll follows FL/RR/FR/RL signs", [] {
    MotorMix mix = mixAndDesaturate(10, 0, 0, 1200, 1050, 1300);
    checkMotors(mix, 1210, 1190, 1190, 1210);
  });

  runCase("mix: pure pitch follows FL/RR/FR/RL signs", [] {
    MotorMix mix = mixAndDesaturate(0, 10, 0, 1200, 1050, 1300);
    checkMotors(mix, 1190, 1210, 1190, 1210);
  });

  runCase("mix: pure yaw follows FL/RR/FR/RL signs", [] {
    MotorMix mix = mixAndDesaturate(0, 0, 10, 1200, 1050, 1300);
    checkMotors(mix, 1190, 1190, 1210, 1210);
  });

  runCase("mix: extreme commands remain inside motor limits", [] {
    MotorMix mix = mixAndDesaturate(100000, -200000, 300000,
                                     5000, 1050, 1300);
    checkAllMotorsInRange(mix, 1050, 1300);
  });

  runCase("mix: fitting commands preserve exact torque differences", [] {
    MotorMix mix = mixAndDesaturate(10, 20, 5, 1175, 1050, 1300);
    checkMotors(mix, 1160, 1180, 1150, 1210);
    CHECK(!mix.scaled);
    CHECK_EQ(mix.motor[0] - mix.motor[1], -20);
    CHECK_EQ(mix.motor[2] - mix.motor[3], -60);
  });

  runCase("mix: sketch delegates saturation priority to the control allocator", [] {
    MotorMix mix = mixAndDesaturate(20, 0, 100, 1100, 1000, 1200);
    checkMotors(mix, 1040, 1000, 1160, 1200);
    CHECK(mix.scaled);
    CHECK_NEAR(mix.rp_scale, 1.0f, 1e-6f);
    CHECK_NEAR(mix.yaw_scale, 0.8f, 1e-6f);
  });

  runCase("mix: collective shifts before attitude scaling", [] {
    MotorMix mix = mixAndDesaturate(30, 0, 0, 1040, 1050, 1250);
    checkMotors(mix, 1110, 1050, 1050, 1110);
    CHECK(!mix.scaled);
  });

  runCase("mix: minMotor above maxMotor collapses to a safe bound", [] {
    MotorMix mix = mixAndDesaturate(30, 0, 0, 1500, 1800, 1200);
    checkMotors(mix, 1800, 1800, 1800, 1800);
    CHECK(mix.scaled);
  });

  runCase("mix: out-of-range limits and throttle are constrained", [] {
    MotorMix low = mixAndDesaturate(0, 0, 0, -100000, 500, 2500);
    MotorMix high = mixAndDesaturate(0, 0, 0, 100000, 500, 2500);
    checkMotors(low, 1000, 1000, 1000, 1000);
    checkMotors(high, 2000, 2000, 2000, 2000);
  });

  for (int us : {1000, 1500, 2000}) {
    runCase("writeMotor: " + std::to_string(us) + " us maps to 14-bit duty", [us] {
      arduino_fake::ledc_duty_by_pin.clear();
      writeMotor(pinM1, us);
      const uint32_t expected = (static_cast<uint32_t>(us) * 16383U) / 2500U;
      CHECK_EQ(arduino_fake::ledc_duty_by_pin.at(pinM1), expected);
    });
  }

  runCase("writeMotor: values below 1000 us are constrained first", [] {
    arduino_fake::ledc_duty_by_pin.clear();
    writeMotor(pinM2, -500);
    CHECK_EQ(arduino_fake::ledc_duty_by_pin.at(pinM2), 1000U * 16383U / 2500U);
  });

  runCase("writeMotor: values above 2000 us are constrained first", [] {
    arduino_fake::ledc_duty_by_pin.clear();
    writeMotor(pinM3, 9000);
    CHECK_EQ(arduino_fake::ledc_duty_by_pin.at(pinM3), 2000U * 16383U / 2500U);
  });

  runCase("RC watchdog: timestamp slightly in the future is not timed out", [] {
    CHECK(!rcTimedOut(1000U, 1001U));
  });

  runCase("RC watchdog: timestamp far in the future is not timed out", [] {
    CHECK(!rcTimedOut(1000U, 5000U));
  });

  runCase("RC watchdog: exact timeout boundary is not timed out", [] {
    CHECK(!rcTimedOut(1000U + RC_TIMEOUT_MS, 1000U));
  });

  runCase("RC watchdog: one millisecond past timeout is timed out", [] {
    CHECK(rcTimedOut(1000U + RC_TIMEOUT_MS + 1U, 1000U));
  });

  runCase("RC watchdog: zero age is not timed out", [] {
    CHECK(!rcTimedOut(1000U, 1000U));
  });

  runCase("RC watchdog: short age across millis wrap is not timed out", [] {
    CHECK(!rcTimedOut(10U, 0xFFFFFFF0U));
  });

  runCase("RC watchdog: expired age across millis wrap is timed out", [] {
    CHECK(rcTimedOut(1000U, 0xFFFFFF00U));
  });

  runCase("RC: valid sequenced packet updates counters and targets", [] {
    resetRcState();
    sendRc("rc 10 1.5 -2.5 3.5");
    CHECK_EQ(lastRcSeq, 10U);
    CHECK_EQ(rcTotalPkts, 1U);
    CHECK_EQ(rcDroppedPkts, 0U);
    CHECK_NEAR(targetAngleX, 1.5f, 1e-6f);
    CHECK_NEAR(targetAngleY, -2.5f, 1e-6f);
    CHECK_NEAR(targetAngleZ, 3.5f, 1e-6f);
    CHECK_EQ(lastRcMs, 100U);
  });

  runCase("RC: 하강 중에는 회계와 watchdog만 갱신하고 목표각은 보존한다", [] {
    resetRcState();
    fs_phase = FS_DESCENDING;
    sendRc("rc 10 1.5 -2.5 3.5");
    CHECK_EQ(lastRcSeq, 10U);
    CHECK_EQ(rcTotalPkts, 1U);
    CHECK_EQ(rcDroppedPkts, 0U);
    CHECK_NEAR(targetAngleX, 9.0f, 1e-6f);
    CHECK_NEAR(targetAngleY, 8.0f, 1e-6f);
    CHECK_NEAR(targetAngleZ, 7.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 100U);
    fs_phase = FS_NONE;
  });

  runCase("rcr: 하강 중에는 회계와 watchdog만 갱신하고 목표값은 보존한다", [] {
    resetRcState();
    lastRcSeq = 5U;
    rcSeqValid = true;
    rcTotalPkts = 7U;
    rcDroppedPkts = 2U;
    targetYawRate = 37.0f;
    fs_phase = FS_DESCENDING;

    sendRcr("rcr 8 1.5 -2.5 135");

    const uint32_t observed_last_seq = lastRcSeq;
    const uint32_t observed_total = rcTotalPkts;
    const uint32_t observed_dropped = rcDroppedPkts;
    const uint32_t observed_last_ms = lastRcMs;
    const float observed_target_x = targetAngleX;
    const float observed_target_y = targetAngleY;
    const float observed_yaw_rate = targetYawRate;
    fs_phase = FS_NONE;

    CHECK_EQ(observed_last_seq, 8U);
    CHECK_EQ(observed_total, 8U);
    CHECK_EQ(observed_dropped, 4U);
    CHECK_EQ(observed_last_ms, 100U);
    CHECK_NEAR(observed_target_x, 9.0f, 1e-6f);
    CHECK_NEAR(observed_target_y, 8.0f, 1e-6f);
    CHECK_NEAR(observed_yaw_rate, 37.0f, 1e-6f);
  });

  runCase("RC: duplicate sequence is counted and discarded", [] {
    resetRcState();
    sendRc("rc 10 1 2 3");
    arduino_fake::millis_value = 200;
    sendRc("rc 10 9 9 9");
    CHECK_EQ(lastRcSeq, 10U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 1U);
    CHECK_NEAR(targetAngleX, 1.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 100U);
  });

  runCase("RC: stale sequence is counted and discarded", [] {
    resetRcState();
    sendRc("rc 10 1 2 3");
    arduino_fake::millis_value = 200;
    sendRc("rc 9 9 9 9");
    CHECK_EQ(lastRcSeq, 10U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 1U);
    CHECK_NEAR(targetAngleX, 1.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 100U);
  });

  runCase("RC: sequence gap adds N minus one dropped packets", [] {
    resetRcState();
    sendRc("rc 10 1 2 3");
    sendRc("rc 14 4 5 6");
    CHECK_EQ(lastRcSeq, 14U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 3U);
    CHECK_NEAR(targetAngleX, 4.0f, 1e-6f);
  });

  runCase("RC: uint32 sequence wrap is forward progress", [] {
    resetRcState();
    lastRcSeq = std::numeric_limits<uint32_t>::max() - 1U;
    rcSeqValid = true;
    sendRc("rc 4294967295 1 2 3");
    sendRc("rc 0 4 5 6");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 0U);
    CHECK_NEAR(targetAngleX, 4.0f, 1e-6f);
  });

  runCase("RC: duplicate zero after wrap is counted and discarded", [] {
    resetRcState();
    lastRcSeq = std::numeric_limits<uint32_t>::max();
    rcSeqValid = true;
    sendRc("rc 0 1 2 3");
    arduino_fake::millis_value = 200;
    sendRc("rc 0 9 9 9");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 1U);
    CHECK_NEAR(targetAngleX, 1.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 100U);
  });

  runCase("RC: gap immediately after wrapped zero is accounted", [] {
    resetRcState();
    lastRcSeq = std::numeric_limits<uint32_t>::max();
    rcSeqValid = true;
    sendRc("rc 0 1 2 3");
    sendRc("rc 2 4 5 6");
    CHECK_EQ(lastRcSeq, 2U);
    CHECK_EQ(rcTotalPkts, 2U);
    CHECK_EQ(rcDroppedPkts, 1U);
    CHECK_NEAR(targetAngleX, 4.0f, 1e-6f);
  });

  runCase("RC: two-field bench form is accepted", [] {
    resetRcState();
    sendRc("rc -4 5");
    CHECK_NEAR(targetAngleX, -4.0f, 1e-6f);
    CHECK_NEAR(targetAngleY, 5.0f, 1e-6f);
    CHECK_NEAR(targetAngleZ, 7.0f, 1e-6f);
    CHECK_EQ(rcTotalPkts, 0U);
    CHECK_EQ(lastRcMs, 100U);
  });

  runCase("RC: three-field bench form is accepted", [] {
    resetRcState();
    sendRc("rc -4 5 6");
    CHECK_NEAR(targetAngleX, -4.0f, 1e-6f);
    CHECK_NEAR(targetAngleY, 5.0f, 1e-6f);
    CHECK_NEAR(targetAngleZ, 6.0f, 1e-6f);
    CHECK_EQ(rcTotalPkts, 0U);
  });

  runCase("RC: fifth argument is rejected", [] {
    resetRcState();
    sendRc("rc 10 1 2 3 extra");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 0U);
    CHECK_NEAR(targetAngleX, 9.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 6U);
  });

  runCase("RC: malformed numeric fields are rejected", [] {
    resetRcState();
    sendRc("rc nope 1 2 3");
    sendRc("rc 10 1oops 2 3");
    sendRc("rc x 2");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 0U);
    CHECK_NEAR(targetAngleX, 9.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 6U);
  });

  runCase("RC: sequences with a leading sign are rejected", [] {
    resetRcState();
    sendRc("rc -1 1 2 3");
    sendRc("rc +1 1 2 3");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 0U);
    CHECK_NEAR(targetAngleX, 9.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 6U);
  });

  runCase("RC: sequence above uint32 range is rejected", [] {
    resetRcState();
    sendRc("rc 4294967296 1 2 3");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK_EQ(rcTotalPkts, 0U);
    CHECK_NEAR(targetAngleX, 9.0f, 1e-6f);
    CHECK_EQ(lastRcMs, 6U);
  });

  runCase("RC: start resets sequence validity before the next baseline", [] {
    resetRcState();
    lastRcSeq = 77U;
    rcSeqValid = true;
    calibration_ok = true;
    fs_phase = FS_NONE;
    angleX = angleY = 0.0f;
    imu1_frozen_now = imu2_frozen_now = false;
    imu_disagree_now = false;
    sendUdpCommandOnce("start");
    CHECK_EQ(lastRcSeq, 0U);
    CHECK(!rcSeqValid);

    sendRc("rc 100 1 2 3");
    CHECK_EQ(rcTotalPkts, 1U);
    CHECK_EQ(rcDroppedPkts, 0U);
    CHECK_EQ(lastRcSeq, 100U);
    CHECK(rcSeqValid);
  });

  runCase("trim: 절대값이 저장된다", [] {
    trim_roll = 0.0f; trim_pitch = 0.0f;
    sendUdpCommandOnce("trim 1.5 -2.0");
    CHECK_NEAR(trim_roll, 1.5f, 1e-4f);
    CHECK_NEAR(trim_pitch, -2.0f, 1e-4f);
  });

  runCase("trim: +-TRIM_MAX_DEG로 클램프된다", [] {
    trim_roll = 0.0f; trim_pitch = 0.0f;
    sendUdpCommandOnce("trim 50 -50");
    CHECK_NEAR(trim_roll, TRIM_MAX_DEG, 1e-4f);
    CHECK_NEAR(trim_pitch, -TRIM_MAX_DEG, 1e-4f);
  });

  runCase("trim: 잘못된 입력은 무시된다", [] {
    trim_roll = 1.0f; trim_pitch = 2.0f;
    sendUdpCommandOnce("trim abc 1");
    sendUdpCommandOnce("trim 1");
    sendUdpCommandOnce("trim 1 2 3");
    CHECK_NEAR(trim_roll, 1.0f, 1e-4f);
    CHECK_NEAR(trim_pitch, 2.0f, 1e-4f);
  });

  runCase("trim: 무장 중에도 수락된다", [] {
    trim_roll = 0.0f;
    safety_lock = false;
    fs_phase = FS_NONE;
    sendUdpCommandOnce("trim 3 0");
    CHECK_NEAR(trim_roll, 3.0f, 1e-4f);
    safety_lock = true;
  });

  runCase("trim: 자동착륙 하강 중에는 거부되고 기존값을 보존한다", [] {
    trim_roll = 1.5f;
    trim_pitch = -2.0f;
    safety_lock = false;
    fs_phase = FS_DESCENDING;
    arduino_fake::serial_output.clear();

    sendUdpCommandOnce("trim 10 -10");

    CHECK_NEAR(trim_roll, 1.5f, 1e-4f);
    CHECK_NEAR(trim_pitch, -2.0f, 1e-4f);
    CHECK(countLogOccurrences("Trim refused (auto-land descending)") == 1U);
    fs_phase = FS_NONE;
    safety_lock = true;
  });

  runCase("목표각은 트림을 더한 뒤 +-30도로 클램프된다", [] {
    resetRcState();
    trim_roll = 10.0f; trim_pitch = -10.0f;
    sendRcr("rcr 1 25 -25 0");
    // 25 + 10 = 35 -> 30 으로 클램프 (35가 아니다)
    CHECK_NEAR(targetAngleX, 30.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -30.0f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("트림이 목표각에 실제로 더해진다", [] {
    resetRcState();
    trim_roll = 2.0f; trim_pitch = -1.0f;
    sendRcr("rcr 1 0 0 0");
    CHECK_NEAR(targetAngleX, 2.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -1.0f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("start와 stop은 트림을 지우지 않는다", [] {
    trim_roll = 4.0f; trim_pitch = -3.0f;
    calibration_ok = true;
    safety_lock = true;
    fs_phase = FS_NONE;
    sendUdpCommandOnce("start");
    CHECK_NEAR(trim_roll, 4.0f, 1e-4f);
    sendUdpCommandOnce("stop");
    runPidTicks(1);
    CHECK_NEAR(trim_roll, 4.0f, 1e-4f);
    CHECK_NEAR(trim_pitch, -3.0f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("RC 타임아웃이 즉시 컷이 아니라 자동착륙으로 간다", [] {
    calibration_ok = true;
    safety_lock = true;
    fs_phase = FS_NONE;
    sendUdpCommandOnce("start");
    runPidTicks(1);
    CHECK(!safety_lock);
    fs_phase = FS_NONE;
    base_throttle = 1360;
    primeHoverEstimate(1360.0f);

    // rc가 끊긴 지 오래된 상태를 만든다
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;
    runPidTicks(1);

    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK(!safety_lock);                       // 아직 컷이 아니다
    CHECK_EQ(base_throttle, 1360 - FS_DESCENT_DELTA_US);
  });

  runCase("호버 이력 없는 RC 타임아웃은 즉시 한 번만 컷한다", [] {
    calibration_ok = true;
    safety_lock = true;
    fs_phase = FS_NONE;
    sendUdpCommandOnce("start");
    runPidTicks(1);
    CHECK(!safety_lock);
    base_throttle = 1199;
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;
    arduino_fake::serial_output.clear();

    runPidTicks(50);

    CHECK(fault_rc);
    CHECK(safety_lock);
    CHECK_EQ((int)fs_phase, (int)FS_NONE);
    CHECK_EQ(motorOut[0], 1000);
    CHECK_EQ(countLogOccurrences("[FAULT] RC TIMEOUT -> CUT (NO HOVER EST)"),
             static_cast<std::size_t>(1));
    CHECK_EQ(countLogOccurrences("[FAULT] RC TIMEOUT -> AUTO-LAND"),
             static_cast<std::size_t>(0));
  });

  runCase("보수적 절대 하한 1150us 이하는 지상으로 보고 즉시 컷한다", [] {
    prepareFailsafeFlight(1360);
    base_throttle = 1150;
    arduino_fake::serial_output.clear();

    runPidTicks(50);

    CHECK(fault_rc);
    CHECK(safety_lock);
    CHECK_EQ((int)fs_phase, (int)FS_NONE);
    CHECK_EQ(motorOut[0], 1000);
    CHECK_EQ(countLogOccurrences("[FAULT] RC TIMEOUT -> GROUND CUT"),
             static_cast<std::size_t>(1));
  });

  runCase("R2: 오염된 높은 hover_est는 공중 1340us를 즉시 컷하지 않는다", [] {
    prepareFailsafeFlight(1340);
    primeHoverEstimate(1464.0f);
    base_throttle = 1340;
    arduino_fake::serial_output.clear();

    runPidTicks(1);

    CHECK(fault_rc);
    CHECK(!safety_lock);
    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK_EQ(base_throttle, 1464 - FS_DESCENT_DELTA_US);
    CHECK_EQ(countLogOccurrences("[FAULT] RC TIMEOUT -> GROUND CUT"),
             static_cast<std::size_t>(0));
    CHECK_EQ(countLogOccurrences("[FAULT] RC TIMEOUT -> AUTO-LAND"),
             static_cast<std::size_t>(1));
  });

  runCase("stop은 자동착륙 중에도 즉시 컷이다", [] {
    fs_phase = FS_DESCENDING;
    safety_lock = false;
    sendUdpCommandOnce("stop");
    CHECK(!safety_lock);
    CHECK(safety_disarm_requested);
    CHECK_EQ(base_throttle, 1000);
    runPidTicks(1);
    CHECK(safety_lock);
    CHECK(!safety_disarm_requested);
  });

  runCase("동시에 대기한 safety 요청은 disarm이 arm보다 우선한다", [] {
    arduino_fake::reset();
    safety_lock = true;
    safety_arm_requested = true;
    safety_disarm_requested = true;
    setFakeAccelMagnitude(1.0f, 0);

    runPidTicks(1);

    CHECK(safety_lock);
    CHECK(!safety_arm_requested);
    CHECK(!safety_disarm_requested);
  });

  runCase("start 거부 경로는 arm 요청을 세우지 않는다", [] {
    arduino_fake::reset();
    safety_lock = true;
    safety_arm_requested = false;
    safety_disarm_requested = false;
    fs_phase = FS_NONE;
    calibration_ok = false;
    mag_calibrating = false;
    angleX = angleY = 0.0f;
    imu1_frozen_now = imu2_frozen_now = false;
    imu_disagree_now = false;

    sendUdpCommandOnce("start");

    CHECK(safety_lock);
    CHECK(!safety_arm_requested);
    CHECK(!safety_disarm_requested);
  });

  runCase("start는 stop 적용 뒤 남은 하강 phase에서 재무장을 거부한다", [] {
    prepareStartCommandState(FS_DESCENDING);
    fault_rc = true;
    base_throttle = 1000;
    primeHoverEstimate(1360.0f);
    arduino_fake::serial_output.clear();

    sendUdpCommandOnce("start");

    CHECK_EQ(safety_arm_requested, false);
    CHECK_EQ(safety_lock, true);
    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK_EQ(fault_rc, true);
    CHECK_EQ(base_throttle, 1000);
    CHECK(hoverTracker.valid);
    CHECK_NEAR(hover_est, 1360.0f, 1e-4f);
    CHECK_EQ(countLogOccurrences("START REFUSED (auto-land descending)"),
             static_cast<std::size_t>(1));
  });

  runCase("start의 hover tracker reset은 다음 pid tick에서만 적용된다", [] {
    prepareStartCommandState(FS_CUT_ABORT);
    primeHoverEstimate(1420.0f);

    sendUdpCommandOnce("start");

    std::cout << "[CONTROL] hover reset before pid"
              << " actual_valid=" << static_cast<int>(hoverTracker.valid)
              << " actual_est=" << hover_est
              << " expected_valid=1 expected_est=1420\n";
    CHECK(safety_lock);
    CHECK(safety_arm_requested);
    CHECK_EQ(hoverTracker.valid, true);
    CHECK_NEAR(hover_est, 1420.0f, 1e-4f);

    runPidTicks(1);

    std::cout << "[CONTROL] hover reset after pid"
              << " actual_valid=" << static_cast<int>(hoverTracker.valid)
              << " actual_est=" << hover_est
              << " expected_valid=0 expected_est=0\n";
    CHECK(!safety_lock);
    CHECK_EQ(hoverTracker.initialized, false);
    CHECK_EQ(hoverTracker.valid, false);
    CHECK_NEAR(hover_est, 0.0f, 1e-4f);
    CHECK_EQ(hover_valid, false);
  });

  for (uint8_t terminal_phase :
       {FS_CUT_LANDED, FS_CUT_TIMEOUT, FS_CUT_ABORT}) {
    runCase("start는 종료 phase " + std::to_string((int)terminal_phase) +
                "에서 재시동을 허용한다",
            [terminal_phase] {
      prepareStartCommandState(terminal_phase);

      sendUdpCommandOnce("start");

      CHECK_EQ(safety_arm_requested, true);
      runPidTicks(1);
      CHECK_EQ(safety_lock, false);
      CHECK_EQ((int)fs_phase, (int)FS_NONE);
      CHECK_EQ(base_throttle, 1100);
      for (int motor : motorOut) CHECK(motor > 1000);
    });
  }

  runCase("R3: pending arm 중복 start는 이미 무장 예약으로 처리한다", [] {
    arduino_fake::reset();
    safety_lock = true;
    safety_arm_requested = false;
    safety_disarm_requested = false;
    fs_phase = FS_NONE;
    calibration_ok = true;
    mag_calibrating = false;
    angleX = angleY = 0.0f;
    imu1_frozen_now = imu2_frozen_now = false;
    imu_disagree_now = false;

    sendUdpCommandOnce("start");
    CHECK(safety_lock);
    CHECK(safety_arm_requested);

    fault_rc = true;
    base_throttle = 1177;
    primeHoverEstimate(1420.0f);
    arduino_fake::serial_output.clear();
    sendUdpCommandOnce("start");

    CHECK(safety_lock);
    CHECK(safety_arm_requested);
    CHECK(fault_rc);
    CHECK_EQ(base_throttle, 1177);
    CHECK(hoverTracker.valid);
    CHECK_NEAR(hover_est, 1420.0f, 1e-4f);
    CHECK_EQ(countLogOccurrences(">>> START ignored (already armed)"),
             static_cast<std::size_t>(1));
  });

  runCase("pid_task 고장 감지는 요청 없이 safety_lock을 직접 세운다", [] {
    arduino_fake::reset();
    safety_lock = false;
    safety_arm_requested = false;
    safety_disarm_requested = false;
    fault_imu1 = true;
    fault_imu2 = true;
    fault_disagree = false;
    setFakeAccelMagnitude(1.0f, 0);

    runPidTicks(1);

    CHECK(safety_lock);
    CHECK(!safety_arm_requested);
    CHECK(!safety_disarm_requested);
    fault_imu1 = false;
    fault_imu2 = false;
  });

  runCase("회귀: 비행 중 roll 과도기울기는 즉시 모터를 컷한다", [] {
    checkInFlightOverTiltCut("roll", SAFETY_ANGLE + 10.0f, 0.0f);
  });

  runCase("회귀: 비행 중 pitch 과도기울기는 즉시 모터를 컷한다", [] {
    checkInFlightOverTiltCut("pitch", 0.0f, SAFETY_ANGLE + 10.0f);
  });

  runCase("resume은 FS_DESCENDING이 아니면 거부된다", [] {
    prepareResumeCommandState();
    fs_phase = FS_NONE;

    sendUdpCommandOnce("resume");

    CHECK(!failsafe_resume_requested);
    CHECK(countLogOccurrences("RESUME REFUSED phase") == 1U);
  });

  runCase("resume은 RC가 timeout이면 거부된다", [] {
    prepareResumeCommandState();
    lastRcMs = arduino_fake::millis_value - RC_TIMEOUT_MS - 1U;

    sendUdpCommandOnce("resume");

    CHECK(!failsafe_resume_requested);
    CHECK(countLogOccurrences("RESUME REFUSED rc") == 1U);
  });

  runCase("resume은 과도기울기면 거부된다", [] {
    prepareResumeCommandState();
    angleX = SAFETY_ANGLE + 1.0f;

    sendUdpCommandOnce("resume");

    CHECK(!failsafe_resume_requested);
    CHECK(countLogOccurrences("RESUME REFUSED tilt") == 1U);
  });

  runCase("resume은 사용 가능한 IMU가 없으면 거부된다", [] {
    prepareResumeCommandState();
    active_imus = 0;

    sendUdpCommandOnce("resume");

    CHECK(!failsafe_resume_requested);
    CHECK(countLogOccurrences("RESUME REFUSED imu") == 1U);
  });

  runCase("resume은 hover_valid가 false면 거부된다", [] {
    prepareResumeCommandState();
    hover_valid = false;

    sendUdpCommandOnce("resume");

    CHECK(!failsafe_resume_requested);
    CHECK(countLogOccurrences("RESUME REFUSED hover") == 1U);
  });

  runCase("resume 수락은 Core 1에서 phase와 hover throttle을 복원한다", [] {
    prepareResumeCommandState();

    sendUdpCommandOnce("resume");

    CHECK(failsafe_resume_requested);
    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK_EQ(base_throttle, 1360 - FS_DESCENT_DELTA_US);

    runPidTicks(1);

    CHECK(!failsafe_resume_requested);
    CHECK_EQ((int)fs_phase, (int)FS_NONE);
    CHECK_EQ(base_throttle, 1360);
    CHECK_EQ(min_throttle, max(1050, 1360 - CTRL_MARGIN));
    CHECK_EQ(max_throttle, min(1900, 1360 + CTRL_MARGIN));
    CHECK(!fault_rc);
    CHECK(!safety_lock);
    CHECK(countLogOccurrences(">>> RESUME") == 1U);
  });

  runCase("R4: 최초 진입 3x FS_MAX 뒤 resume만 거부하고 하강은 유지한다", [] {
    prepareFailsafeFlight(1360);
    arduino_fake::serial_output.clear();

    uint8_t phase_at_previous_hook = fs_phase;
    uint32_t episode_start_tick = std::numeric_limits<uint32_t>::max();
    uint32_t fresh_rc_gap_ticks = 0;
    int resume_attempts = 0;
    int accepted_resumes = 0;
    arduino_fake::pre_tick_hook = [&](uint32_t tick) {
      setFakeAccelMagnitude(
          fs_probe_state == FS_PROBE_DIP ? 0.70f : 1.0f, tick);

      if (fs_phase == FS_DESCENDING &&
          phase_at_previous_hook != FS_DESCENDING) {
        episode_start_tick = tick;
      }
      if (fs_phase == FS_NONE &&
          phase_at_previous_hook == FS_DESCENDING) {
        accepted_resumes++;
        fresh_rc_gap_ticks = 20U;
      }

      if (fs_phase == FS_DESCENDING &&
          episode_start_tick != std::numeric_limits<uint32_t>::max() &&
          tick - episode_start_tick == FS_MAX_MS - 10U) {
        lastRcMs = millis();
        sendUdpCommandOnce("resume");
        resume_attempts++;
        if (countLogOccurrences("RESUME REFUSED cumulative") > 0U) {
          // 거부 자체가 자동착륙을 끝내지 않았는지 같은 tick에서 관찰한다.
          arduino_fake::tick_limit = tick;
        }
      } else if (fs_phase == FS_NONE && accepted_resumes > 0) {
        if (fresh_rc_gap_ticks > 0U) {
          lastRcMs = millis();
          fresh_rc_gap_ticks--;
        } else {
          lastRcMs = millis() - RC_TIMEOUT_MS - 1U;
        }
      }
      phase_at_previous_hook = fs_phase;
    };

    runPidTicks(3U * FS_MAX_MS + 200U);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK_EQ(resume_attempts, 3);
    CHECK_EQ(accepted_resumes, 2);
    CHECK_EQ(countLogOccurrences("RESUME REFUSED cumulative"),
             static_cast<std::size_t>(1));
    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK(!safety_lock);
  });

  runCase("pending resume 뒤 stop은 resume을 취소하고 ABORT로 끝난다", [] {
    prepareResumeCommandState();

    sendUdpCommandOnce("resume");
    CHECK(failsafe_resume_requested);
    sendUdpCommandOnce("stop");

    CHECK(!failsafe_resume_requested);
    CHECK(safety_disarm_requested);
    CHECK_EQ(base_throttle, 1000);
    runPidTicks(1);
    CHECK(safety_lock);
    CHECK_EQ((int)fs_phase, (int)FS_CUT_ABORT);
    for (int motor : motorOut) CHECK_EQ(motor, 1000);
  });

  runCase("회귀: stale failsafe phase로 무장돼도 첫 tick 뒤 RC 감시가 복구된다", [] {
    arduino_fake::reset();
    fault_rc = false;
    fault_imu1 = false;
    fault_imu2 = false;
    fault_disagree = false;
    fault_attitude = false;
    imu1_frozen_now = false;
    imu2_frozen_now = false;
    imu_disagree_now = false;
    calibration_ok = true;
    mag_calibrating = false;
    safety_lock = true;
    fs_phase = FS_CUT_ABORT;
    angleX = angleY = angleZ = 0.0f;
    gyro_bias1[0] = gyro_bias1[1] = gyro_bias1[2] = 0.0f;
    gyro_bias2[0] = gyro_bias2[1] = gyro_bias2[2] = 0.0f;
    setFakeAccelMagnitude(1.0f, 0);

    sendUdpCommandOnce("start");
    CHECK(safety_lock);
    CHECK(safety_arm_requested);
    CHECK_EQ((int)fs_phase, (int)FS_CUT_ABORT);
    runPidTicks(1);
    CHECK(!safety_lock);
    CHECK(!safety_arm_requested);
    CHECK_EQ((int)fs_phase, (int)FS_NONE);

    base_throttle = 1360;
    primeHoverEstimate(1360.0f);
    bool phase_cleared_after_first_tick = false;
    arduino_fake::pre_tick_hook = [&](uint32_t tick) {
      if (tick == 1U) {
        phase_cleared_after_first_tick = (fs_phase == FS_NONE);
      }
      setFakeAccelMagnitude(1.0f, tick);
    };
    runPidTicks(RC_TIMEOUT_MS + 20U);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK(phase_cleared_after_first_tick);
    CHECK(fault_rc);
    CHECK(!safety_lock);
    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
  });

  runCase("회귀: 하강 목표는 0이 아니라 트림값이다", [] {
    // 사용자가 잡은 결함. 0을 명령하면 트림이 보정하던 흐름이 그대로 재현돼
    // 5초에 수 미터를 흘러간다(설계 문서 §3).
    trim_roll = 2.0f; trim_pitch = -1.5f;
    calibration_ok = true;
    safety_lock = true;
    fs_phase = FS_NONE;
    sendUdpCommandOnce("start");
    runPidTicks(1);  // Core 1에서 arm과 hover reset 요청을 먼저 적용한다.
    CHECK(!safety_lock);
    fs_phase = FS_NONE;
    base_throttle = 1360;
    primeHoverEstimate(1360.0f);
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;
    runPidTicks(1);

    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK_NEAR(targetAngleX, 2.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -1.5f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("회귀: 하강 중 rc가 돌아와도 제어권이 복귀하지 않는다", [] {
    // 매 tick 덮어쓰기가 없으면 rc 파서가 targetAngle을 다시 써서
    // '자동 복귀 없음' 결정과 어긋난다(설계 문서 §1).
    trim_roll = 0.0f; trim_pitch = 0.0f;
    calibration_ok = true;
    safety_lock = true;
    fs_phase = FS_NONE;
    sendUdpCommandOnce("start");
    runPidTicks(1);  // Core 1에서 arm과 hover reset 요청을 먼저 적용한다.
    CHECK(!safety_lock);
    fs_phase = FS_NONE;
    base_throttle = 1360;
    primeHoverEstimate(1360.0f);
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;
    lastRcSeq = 0;
    rcSeqValid = false;
    rcTotalPkts = 0;
    rcDroppedPkts = 0;
    arduino_fake::pre_tick_hook = [](uint32_t tick) {
      if (tick == 1U) {
        sendRcr("rcr 1 25 -25 80");     // 링크 복귀: 큰 조종 입력
      }
    };
    runPidTicks(2);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK_EQ(lastRcSeq, 1U);                  // 패킷은 실제로 수락됐다
    CHECK_EQ(rcDroppedPkts, 0U);
    CHECK_NEAR(targetAngleX, 0.0f, 1e-4f);   // 스틱 입력이 무시된다
    CHECK_NEAR(targetAngleY, 0.0f, 1e-4f);
    CHECK_NEAR(targetYawRate, 0.0f, 1e-4f);
    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);   // 상태도 그대로
    CHECK_EQ(base_throttle, 1360 - FS_DESCENT_DELTA_US);
  });

  runCase("회귀: 하강 중 th 명령은 collective 창과 모터를 올리지 못한다", [] {
    constexpr int entry_throttle = 1360;
    prepareFailsafeFlight(entry_throttle);
    arduino_fake::pre_tick_hook = [](uint32_t tick) {
      if (tick == 1U) sendUdpCommandOnce("th 1800");
    };

    runPidTicks(3);
    arduino_fake::pre_tick_hook = nullptr;

    const int descent_throttle =
        entry_throttle - FS_DESCENT_DELTA_US;
    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK_EQ(base_throttle, descent_throttle);
    CHECK_EQ(min_throttle, max(1050, descent_throttle - CTRL_MARGIN));
    CHECK_EQ(max_throttle, min(1900, descent_throttle + CTRL_MARGIN));
    for (int motor : motorOut) CHECK(motor < entry_throttle);
  });

  runCase("회귀: 하강 중 4인자 rc는 yaw 스냅샷과 rate를 바꾸지 못한다", [] {
    constexpr float entry_yaw_deg = 17.0f;
    prepareFailsafeFlight(1360);
    mag_enabled = false;
    yaw_hold_override = false;
    angleZ = entry_yaw_deg;
    lastRcSeq = 0;
    rcSeqValid = false;
    rcTotalPkts = 0;
    rcDroppedPkts = 0;
    bool rc_accounted_by_parser = false;
    arduino_fake::pre_tick_hook = [&](uint32_t tick) {
      if (tick == 1U) {
        sendRc("rc 1 0 0 90");
        rc_accounted_by_parser =
            lastRcSeq == 1U && rcTotalPkts == 1U &&
            rcDroppedPkts == 0U && lastRcMs == millis();
      }
      setFakeAccelMagnitude(1.0f, tick);
    };

    runPidTicks(6);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK(rc_accounted_by_parser);
    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK_NEAR(targetAngleZ, entry_yaw_deg, 1e-4f);
    CHECK_NEAR(tgtRate[2], 0.0f, 1e-3f);
  });

  runCase("회귀: 진입 로그는 하강 내내 딱 한 번만 나간다", [] {
    // 가드가 없으면 rcTimedOut이 참인 동안 1kHz로 재실행돼 Serial.println이
    // 115200 baud TX 버퍼를 포화시키고, pid_task가 블로킹되면 500ms 태스크
    // 워치독이 비행 중 재부팅을 일으킨다. 이 태스크에서 가장 위험한 실패 모드다.
    calibration_ok = true;
    safety_lock = true;
    fs_phase = FS_NONE;
    sendUdpCommandOnce("start");
    runPidTicks(1);  // Core 1에서 arm과 hover reset 요청을 먼저 적용한다.
    CHECK(!safety_lock);
    fs_phase = FS_NONE;
    base_throttle = 1360;
    primeHoverEstimate(1360.0f);
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;

    arduino_fake::serial_output.clear();
    runPidTicks(50);

    const std::string &log = arduino_fake::serial_output;
    std::size_t hits = 0;
    for (std::size_t at = log.find("[FAULT] RC TIMEOUT -> AUTO-LAND");
         at != std::string::npos;
         at = log.find("[FAULT] RC TIMEOUT -> AUTO-LAND", at + 1)) {
      hits++;
    }
    CHECK_EQ(hits, static_cast<std::size_t>(1));
  });

  runCase("자동착륙 종료 후에는 재시동 없이 날 수 없다", [] {
    prepareFailsafeFlight(1360);
    arduino_fake::pre_tick_hook = [](uint32_t tick) {
      setFakeAccelMagnitude(1.0f, tick);
    };
    runPidTicks(FS_MAX_MS + 10U);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK(safety_lock);                        // 백스톱으로 잠겼다
    CHECK_EQ(motorOut[0], 1000);
  });

  runCase("프로브는 판정하지 않는다: 지상 무반응이 쌓여도 백스톱까지 하강한다", [] {
    prepareFailsafeFlight(1360);
    uint32_t cut_tick = std::numeric_limits<uint32_t>::max();
    arduino_fake::pre_tick_hook = [&](uint32_t tick) {
      if (fs_phase == FS_CUT_LANDED
          && cut_tick == std::numeric_limits<uint32_t>::max()) {
        cut_tick = tick;
      }
      setFakeAccelMagnitude(1.0f, tick);
    };

    // 아래 관측 시각까지 후보 조건을 충족해도 컷하지 않는지를 본다.
    // 관측 시각은 FS_PROBE_CONFIRM_N에서 유도해 상수 변경도 따라간다.
    // 2026-08-01의 "지면 응답이 공중보다 큼" 해석은 철회됐다. 두 자료 모두
    // 지면이었으며 유효한 실패 관측은 015448의 지면 미검출이다. 현재 펌웨어는
    // landed=false로 후보 반환값을 컷에서 제외하므로, 1g 무반응을 계속 넣어도
    // CUT_LANDED로 가면 안 되고 백스톱까지 하강 상태를 유지해야 한다.
    const uint32_t confirm_ms =
        FS_MIN_DESCEND_MS
        + FS_PROBE_PERIOD_MS * (uint32_t)FS_PROBE_CONFIRM_N
        + FS_PROBE_DIP_MS;
    runPidTicks(confirm_ms + 20U);

    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK(!safety_lock);
    CHECK_EQ(cut_tick, std::numeric_limits<uint32_t>::max());
    // 프로브 자체는 계속 돌아 텔레메트리를 채운다 — 다음 판별식 설계용 데이터다.
    CHECK(fs_probe_no_response >= FS_PROBE_CONFIRM_N);

    // 백스톱은 fs_enter_ms 기준이라 tick 수와 1:1이 아니다. 넉넉히 돌린다.
    runPidTicks(FS_MAX_MS + 200U);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK_EQ((int)fs_phase, (int)FS_CUT_TIMEOUT);
    CHECK(safety_lock);
    CHECK_EQ(motorOut[0], 1000);
  });

  runCase("공중에서 매 딥이 반응하면 FS_MAX_MS에서 TIMEOUT으로 간다", [] {
    prepareFailsafeFlight(1360);
    uint32_t cut_tick = std::numeric_limits<uint32_t>::max();
    arduino_fake::pre_tick_hook = [&](uint32_t tick) {
      if (fs_phase == FS_CUT_TIMEOUT
          && cut_tick == std::numeric_limits<uint32_t>::max()) {
        cut_tick = tick;
      }
      const float accel_g =
          fs_probe_state == FS_PROBE_DIP ? 0.82f : 1.0f;
      setFakeAccelMagnitude(accel_g, tick);
    };

    runPidTicks(FS_MAX_MS + 10U);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK_EQ((int)fs_phase, (int)FS_CUT_TIMEOUT);
    CHECK(safety_lock);
    CHECK_EQ(motorOut[0], 1000);
    CHECK(cut_tick >= FS_MAX_MS);
    CHECK(cut_tick <= FS_MAX_MS + 2U);
  });

  runCase("회귀: 적분기 한계 자세에서도 프로브 딥 40us가 온전히 전달된다", [] {
    constexpr int hover_throttle = 1340;
    constexpr int descent_throttle =
        hover_throttle - FS_DESCENT_DELTA_US;
    // 딥 크기는 FS_PROBE_DIP_FRAC 에서 유도한다.
    const int probe_dip_us = (int)lroundf(
        (hover_throttle - 1000.0f) * FS_PROBE_DIP_FRAC);

    prepareFailsafeFlight(hover_throttle);
    const float saved_kp_roll = Kp_Rate_Roll;
    const float saved_ki_roll = Ki_Rate_Roll;
    const float saved_kd_roll = Kd_Rate_Roll;
    const float saved_kp_pitch = Kp_Rate_Pitch;
    const float saved_ki_pitch = Ki_Rate_Pitch;
    const float saved_kd_pitch = Kd_Rate_Pitch;
    const float saved_kp_yaw = Kp_Rate_Yaw;
    const float saved_ki_yaw = Ki_Rate_Yaw;
    const float saved_kd_yaw = Kd_Rate_Yaw;
    Kp_Rate_Roll = Ki_Rate_Roll = Kd_Rate_Roll = 0.0f;
    Kp_Rate_Pitch = Ki_Rate_Pitch = Kd_Rate_Pitch = 0.0f;
    Kp_Rate_Yaw = Ki_Rate_Yaw = Kd_Rate_Yaw = 0.0f;
    iTermRoll = -I_TERM_MAX_US;
    iTermPitch = I_TERM_MAX_US;
    iTermYaw = I_TERM_MAX_US;
    arduino_fake::pre_tick_hook = [](uint32_t tick) {
      setFakeAccelMagnitude(
          fs_probe_state == FS_PROBE_DIP ? 0.82f : 1.0f, tick);
    };

    runPidTicks(FS_MIN_DESCEND_MS + 5U);
    arduino_fake::pre_tick_hook = nullptr;

    const uint8_t observed_probe_state = fs_probe_state;
    const int observed_base = base_throttle;
    const int observed_min = min_throttle;
    const int observed_max = max_throttle;
    const bool observed_scaled = mixer_scaled;
    const int observed_motors[4] = {
        motorOut[0], motorOut[1], motorOut[2], motorOut[3]};
    Kp_Rate_Roll = saved_kp_roll;
    Ki_Rate_Roll = saved_ki_roll;
    Kd_Rate_Roll = saved_kd_roll;
    Kp_Rate_Pitch = saved_kp_pitch;
    Ki_Rate_Pitch = saved_ki_pitch;
    Kd_Rate_Pitch = saved_kd_pitch;
    Kp_Rate_Yaw = saved_kp_yaw;
    Ki_Rate_Yaw = saved_ki_yaw;
    Kd_Rate_Yaw = saved_kd_yaw;

    // diff minimum is exactly -150us. Without widening the probe-only lower
    // window by 40us, the mixer silently shifted collective back to 1280us,
    // delivered no dip, and still reported scaled=false.
    CHECK_EQ((int)observed_probe_state, (int)FS_PROBE_DIP);
    CHECK_EQ(observed_base, descent_throttle - probe_dip_us);
    CHECK_EQ(
        observed_min,
        max(1050, descent_throttle - CTRL_MARGIN - probe_dip_us));
    CHECK_EQ(observed_max, min(1900, descent_throttle + CTRL_MARGIN));
    CHECK(!observed_scaled);
    CHECK_EQ(observed_motors[0], 1090);
    CHECK_EQ(observed_motors[1], 1290);
    CHECK_EQ(observed_motors[2], 1290);
    CHECK_EQ(observed_motors[3], 1290);
  });

  runCase("모든 프로브가 믹서에 막히면 LANDED가 아니라 백스톱 컷한다", [] {
    prepareFailsafeFlight(1340);
    const float saved_kp_roll = Kp_Rate_Roll;
    const float saved_ki_roll = Ki_Rate_Roll;
    const float saved_kd_roll = Kd_Rate_Roll;
    const float saved_kp_pitch = Kp_Rate_Pitch;
    const float saved_ki_pitch = Ki_Rate_Pitch;
    const float saved_kd_pitch = Kd_Rate_Pitch;
    const float saved_kp_yaw = Kp_Rate_Yaw;
    const float saved_ki_yaw = Ki_Rate_Yaw;
    const float saved_kd_yaw = Kd_Rate_Yaw;
    Kp_Rate_Roll = Ki_Rate_Roll = Kd_Rate_Roll = 0.0f;
    Kp_Rate_Pitch = Ki_Rate_Pitch = Kd_Rate_Pitch = 0.0f;
    Kp_Rate_Yaw = Ki_Rate_Yaw = Kd_Rate_Yaw = 0.0f;
    arduino_fake::serial_output.clear();
    arduino_fake::pre_tick_hook = [](uint32_t tick) {
      // 합성 PID 출력 ±60us는 I 한계 ±50us + 같은 방향 P 10us에 해당한다.
      iTermRoll = -60.0f;
      iTermPitch = 60.0f;
      iTermYaw = 60.0f;
      setFakeAccelMagnitude(1.0f, tick);
    };

    runPidTicks(FS_MAX_MS + 10U);
    arduino_fake::pre_tick_hook = nullptr;

    const uint8_t observed_phase = fs_phase;
    const uint8_t observed_no_response = fs_probe_no_response;
    const std::size_t blocked_logs =
        countLogOccurrences("AUTO-LAND PROBE BLOCKED");
    Kp_Rate_Roll = saved_kp_roll;
    Ki_Rate_Roll = saved_ki_roll;
    Kd_Rate_Roll = saved_kd_roll;
    Kp_Rate_Pitch = saved_kp_pitch;
    Ki_Rate_Pitch = saved_ki_pitch;
    Kd_Rate_Pitch = saved_kd_pitch;
    Kp_Rate_Yaw = saved_kp_yaw;
    Ki_Rate_Yaw = saved_ki_yaw;
    Kd_Rate_Yaw = saved_kd_yaw;

    CHECK_EQ((int)observed_phase, (int)FS_CUT_TIMEOUT);
    CHECK_EQ((int)observed_no_response, 0);
    CHECK_EQ(blocked_logs, static_cast<std::size_t>(1));
  });

  runCase("R1: 프로브 딥은 1.3x 호버 여유의 11.8%인 52us다", [] {
    constexpr int hover_throttle = 1442;
    constexpr int descent_throttle =
        hover_throttle - FS_DESCENT_DELTA_US;
    const int expected_probe_dip_us = (int)lroundf(
        (hover_throttle - 1000.0f) * FS_PROBE_DIP_FRAC);
    prepareFailsafeFlight(hover_throttle);
    arduino_fake::pre_tick_hook = [](uint32_t tick) {
      setFakeAccelMagnitude(
          fs_probe_state == FS_PROBE_DIP ? 0.82f : 1.0f, tick);
    };

    runPidTicks(FS_MIN_DESCEND_MS + 5U);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK_EQ((int)fs_probe_state, (int)FS_PROBE_DIP);
    CHECK_EQ(base_throttle, descent_throttle - expected_probe_dip_us);
    CHECK_EQ(
        min_throttle,
        max(1050, descent_throttle - CTRL_MARGIN - expected_probe_dip_us));
    CHECK_EQ(max_throttle, min(1900, descent_throttle + CTRL_MARGIN));
    for (int motor : motorOut) {
      CHECK_EQ(motor, descent_throttle - expected_probe_dip_us);
    }
  });

  runCase("R1: 방어적 낮은 hover_est의 딥 clamp는 로그에 드러난다", [] {
    prepareFailsafeFlight(1200);
    primeHoverEstimate(1100.0f);
    base_throttle = 1200;
    arduino_fake::serial_output.clear();
    arduino_fake::pre_tick_hook = [](uint32_t tick) {
      setFakeAccelMagnitude(
          fs_probe_state == FS_PROBE_DIP ? 0.70f : 1.0f, tick);
    };

    runPidTicks(FS_MIN_DESCEND_MS + 5U);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK_EQ((int)fs_probe_state, (int)FS_PROBE_DIP);
    CHECK_EQ(base_throttle, 1020);
    CHECK_EQ(countLogOccurrences("AUTO-LAND PROBE DIP CLAMPED"),
             static_cast<std::size_t>(1));
  });

  runCase("1000us 아래 딥은 UNAVAILABLE로 공개하고 TIMEOUT에 맡긴다", [] {
    prepareFailsafeFlight(1200);
    // 실제 collective는 절대 지상컷 하한보다 높지만 방어적으로 오염된 낮은
    // 추정치가 들어온 경우다: descent=1010, clamp된 dip=20 -> 990us.
    primeHoverEstimate(1070.0f);
    base_throttle = 1200;
    arduino_fake::pre_tick_hook = [](uint32_t tick) {
      setFakeAccelMagnitude(1.0f, tick);
    };

    runPidTicks(FS_MAX_MS + 10U);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK_EQ((int)fs_probe_state, (int)FS_PROBE_UNAVAILABLE);
    CHECK_EQ((int)fs_probe_no_response, 0);
    CHECK_EQ((int)fs_phase, (int)FS_CUT_TIMEOUT);
    CHECK(safety_lock);
  });

  runCase("자동착륙 중 IMU가 전멸하면 FS_CUT_ABORT로 끝난다", [] {
    fs_phase = FS_DESCENDING;
    safety_lock = false;
    fault_imu1 = true;
    fault_imu2 = true;
    fault_disagree = false;
    arduino_fake::millis_value = 100;
    runPidTicks(1);

    const uint8_t phase_after_imu_loss = fs_phase;
    fault_imu1 = false;
    fault_imu2 = false;
    CHECK_EQ((int)phase_after_imu_loss, (int)FS_CUT_ABORT);
  });

  runCase("자동착륙 중 safety_lock이 서면 FS_CUT_ABORT로 끝난다", [] {
    fs_phase = FS_DESCENDING;
    safety_lock = true;
    fault_imu1 = false;
    fault_imu2 = false;
    fault_disagree = false;
    arduino_fake::millis_value = 100;
    runPidTicks(1);
    CHECK_EQ((int)fs_phase, (int)FS_CUT_ABORT);
  });

  runCase("이미 끝난 상태는 ABORT로 덮어쓰지 않는다", [] {
    fs_phase = FS_CUT_LANDED;
    safety_lock = true;
    fault_imu1 = false;
    fault_imu2 = false;
    fault_disagree = false;
    arduino_fake::millis_value = 100;
    runPidTicks(1);
    CHECK_EQ((int)fs_phase, (int)FS_CUT_LANDED);
  });

  runCase("rcr: 정상 패킷이 yaw 각속도와 roll/pitch를 설정한다", [] {
    resetRcState();
    sendRcr("rcr 1 5.5 -6.5 45.0");
    CHECK_NEAR(targetAngleX, 5.5f, 1e-4f);
    CHECK_NEAR(targetAngleY, -6.5f, 1e-4f);
    CHECK_NEAR(targetYawRate, 45.0f, 1e-4f);
  });

  runCase("rcr: roll/pitch는 +-30도로 클램프된다", [] {
    resetRcState();
    sendRcr("rcr 1 90 -90 0");
    CHECK_NEAR(targetAngleX, 30.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -30.0f, 1e-4f);
  });

  runCase("rcr: yaw 각속도는 +-180dps로 클램프된다", [] {
    resetRcState();
    sendRcr("rcr 1 0 0 500");
    CHECK_NEAR(targetYawRate, 180.0f, 1e-4f);
    sendRcr("rcr 2 0 0 -500");
    CHECK_NEAR(targetYawRate, -180.0f, 1e-4f);
  });

  runCase("rcr: 역순/중복 seq는 폐기되고 드롭으로 계수된다", [] {
    resetRcState();
    sendRcr("rcr 10 0 0 20");
    const uint32_t dropped_before = rcDroppedPkts;
    sendRcr("rcr 9 0 0 99");
    CHECK_NEAR(targetYawRate, 20.0f, 1e-4f);
    CHECK_EQ(rcDroppedPkts, dropped_before + 1U);
  });

  runCase("rcr: seq 건너뜀이 드롭 수에 반영된다", [] {
    resetRcState();
    sendRcr("rcr 10 0 0 0");
    sendRcr("rcr 14 0 0 0");
    CHECK_EQ(rcDroppedPkts, 3U);
  });

  runCase("rcr: 인자 수 불일치와 비수치는 거부된다", [] {
    resetRcState();
    sendRcr("rcr 1 0 0 30");
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
    sendRcr("rcr 2 0 0");          // 인자 부족
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
    sendRcr("rcr 3 0 0 abc");      // 비수치
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
    sendRcr("rcr 4 0 0 10 99");    // 여분 필드
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
  });

  runCase("rcr: udp_task 디스패치가 rc와 구분해서 처리한다", [] {
    resetRcState();
    sendUdpCommandOnce("rcr 1 0 0 60");
    CHECK_NEAR(targetYawRate, 60.0f, 1e-4f);
  });

  struct GainCommandCase {
    const char *prefix;
    std::vector<std::size_t> changed;
  };
  const std::vector<GainCommandCase> gain_cases = {
    {"rp", {3, 6}}, {"ri", {4, 7}}, {"rd", {5, 8}},
    {"ap", {0, 1}}, {"ar", {0}}, {"at", {1}}, {"ay", {2}},
    {"yp", {9}}, {"yi", {10}}, {"yd", {11}},
    {"pa", {3, 6}}, {"ia", {4, 7}}, {"da", {5, 8}},
    {"pr", {3}}, {"ir", {4}}, {"dr", {5}},
    {"pp", {6}}, {"ip", {7}}, {"dp", {8}},
    {"py", {9}}, {"iy", {10}}, {"dy", {11}},
  };
  for (const auto &gain_case : gain_cases) {
    runCase(std::string("gain: ") + gain_case.prefix + " writes only its target",
            [gain_case] { checkGainCommand(gain_case.prefix, gain_case.changed); });
  }

  runCase("gain: 대표 명령은 자동착륙 하강 중 거부되고 기존값을 보존한다", [] {
    for (const char *prefix : {"pa", "dr", "ay"}) {
      seedGains();
      std::vector<float> before;
      for (const auto &slot : gain_slots) {
        before.push_back(static_cast<float>(*slot.value));
      }
      fs_phase = FS_DESCENDING;
      arduino_fake::serial_output.clear();

      sendUdpCommandOnce(std::string(prefix) + " 77.25");

      for (std::size_t index = 0; index < std::size(gain_slots); index++) {
        CHECK_NEAR(*gain_slots[index].value, before[index], 1e-6f);
      }
      CHECK_EQ(countLogOccurrences(
                   "PID gain refused (auto-land descending)"),
               static_cast<std::size_t>(1));
    }
    fs_phase = FS_NONE;
  });

  runCase("gains 조회는 자동착륙 하강 중에도 응답하고 값을 바꾸지 않는다", [] {
    arduino_fake::reset();
    wifi_udp_fake::reset();
    seedGains();
    std::vector<float> before;
    for (const auto &slot : gain_slots) {
      before.push_back(static_cast<float>(*slot.value));
    }
    fs_phase = FS_DESCENDING;

    sendUdpCommandOnce("gains");

    CHECK(wifi_udp_fake::telemetry_output.rfind("GAINS,", 0) == 0);
    for (std::size_t index = 0; index < std::size(gain_slots); index++) {
      CHECK_NEAR(*gain_slots[index].value, before[index], 1e-6f);
    }
    CHECK_EQ(countLogOccurrences("refused (auto-land descending)"),
             static_cast<std::size_t>(0));
    fs_phase = FS_NONE;
  });

  runCase("gain: negative values are rejected", [] {
    seedGains();
    const float before = Kp_Rate_Pitch;
    handleGainCommand("pp -0.01");
    CHECK_NEAR(Kp_Rate_Pitch, before, 1e-6f);
  });

  runCase("gain: values above 100 are rejected", [] {
    seedGains();
    const float before = Kp_Rate_Pitch;
    handleGainCommand("pp 100.01");
    CHECK_NEAR(Kp_Rate_Pitch, before, 1e-6f);
  });

  runCase("gain: lower boundary zero is accepted", [] {
    seedGains();
    handleGainCommand("pp 0");
    CHECK_NEAR(Kp_Rate_Pitch, 0.0f, 1e-6f);
  });

  runCase("gain: upper boundary 100 is accepted", [] {
    seedGains();
    handleGainCommand("pp 100");
    CHECK_NEAR(Kp_Rate_Pitch, 100.0f, 1e-6f);
  });

  runCase("gain: malformed values leave gains unchanged", [] {
    seedGains();
    const float before = Kp_Rate_Pitch;
    handleGainCommand("pp 2.5oops");
    CHECK_NEAR(Kp_Rate_Pitch, before, 1e-6f);
  });

  runCase("parseFloatStrict: trailing garbage is rejected", [] {
    float value = 9.0f;
    CHECK(!parseFloatStrict("2.5oops", value));
    CHECK_NEAR(value, 9.0f, 1e-6f);
  });

  runCase("parseFloatStrict: trailing spaces and tabs are accepted", [] {
    float value = 0.0f;
    CHECK(parseFloatStrict("2.5 \t", value));
    CHECK_NEAR(value, 2.5f, 1e-6f);
  });

  runCase("parseFloatStrict: nan and infinity are rejected", [] {
    float value = 9.0f;
    CHECK(!parseFloatStrict("nan", value));
    CHECK(!parseFloatStrict("inf", value));
    CHECK(!parseFloatStrict("-inf", value));
    CHECK_NEAR(value, 9.0f, 1e-6f);
  });

  runCase("parseFloatStrict: empty input is rejected", [] {
    float value = 9.0f;
    CHECK(!parseFloatStrict("", value));
    CHECK_NEAR(value, 9.0f, 1e-6f);
  });

  runCase("parseIntStrict: trailing garbage is rejected", [] {
    long value = 9;
    CHECK(!parseIntStrict("12oops", value));
    CHECK_EQ(value, 9L);
  });

  runCase("parseIntStrict: trailing spaces and tabs are accepted", [] {
    long value = 0;
    CHECK(parseIntStrict("12 \t", value));
    CHECK_EQ(value, 12L);
  });

  runCase("parseIntStrict: empty input is rejected", [] {
    long value = 9;
    CHECK(!parseIntStrict("", value));
    CHECK_EQ(value, 9L);
  });

  runCase("parseIntStrict: ESP32 long maximum is accepted", [] {
    long value = 0;
    CHECK(parseIntStrict("2147483647", value));
    CHECK_EQ(value, 2147483647L);
  });

  runCase("parseIntStrict: above ESP32 long maximum saturates at LONG_MAX", [] {
    long value = 9;
    CHECK(parseIntStrict("2147483648", value));
    CHECK_EQ(value, std::numeric_limits<long>::max());
  });

  runCase("parseIntStrict: below ESP32 long minimum saturates at LONG_MIN", [] {
    long value = 9;
    CHECK(parseIntStrict("-2147483649", value));
    CHECK_EQ(value, std::numeric_limits<long>::min());
  });

  runCase("parseIntStrict: strtol overflow saturates at LONG_MAX", [] {
    long value = 9;
    CHECK(parseIntStrict("999999999999999999999999999999", value));
    CHECK_EQ(value, std::numeric_limits<long>::max());
  });

  runCase("parseIntStrict: saturation is safe at th and yaw call sites", [] {
    base_throttle = 1100;
    min_throttle = 1050;
    max_throttle = 1250;
    yaw_hold_override = false;
    angleZ = 42.0f;
    targetAngleZ = -7.0f;

    sendUdpCommandOnce("th 999999999999999999999999999999");
    CHECK_EQ(base_throttle, 1900);
    CHECK_EQ(min_throttle, 1750);
    CHECK_EQ(max_throttle, 1900);

    sendUdpCommandOnce("yaw 999999999999999999999999999999");
    CHECK(!yaw_hold_override);
    CHECK_NEAR(targetAngleZ, -7.0f, 1e-6f);
  });

  runCase("yaw 1은 무장 중 거부되고 yaw 0은 언제나 수락된다", [] {
    yaw_hold_override = false;
    safety_lock = false;                       // 무장 상태
    sendUdpCommandOnce("yaw 1");
    CHECK(!yaw_hold_override);                 // 거부

    safety_lock = true;                        // 시동 해제 상태
    sendUdpCommandOnce("yaw 1");
    CHECK(yaw_hold_override);                  // 수락

    safety_lock = false;
    sendUdpCommandOnce("yaw 0");
    CHECK(!yaw_hold_override);                 // 끄기는 무장 중에도 허용
  });

  runCase("잠금 진입 헬퍼는 오버라이드를 엣지에서 1회만 해제한다", [] {
    yaw_hold_override = true;
    bool wasLocked = false;
    enterLockedState(wasLocked);
    CHECK(!yaw_hold_override);      // 무장 해제 엣지에서 해제
    CHECK(wasLocked);

    // 잠긴 상태가 유지되는 동안에는 다시 켤 수 있어야 한다
    // (매 tick 지우면 시동 해제 상태에서 yaw 1을 켤 수 없다)
    yaw_hold_override = true;
    enterLockedState(wasLocked);
    CHECK(yaw_hold_override);
  });

  runCase("mag 기준 요청은 소비 직후 다시 게시돼도 다음 소비까지 보존된다", [] {
    mag_reference_pending = true;
    const bool first_request = takeMagReferenceRequest();
    CHECK(first_request);
    CHECK(!mag_reference_pending);

    // Core 0 요청이 Core 1의 이전 소비 뒤에 끼어든 경우를 순서대로 재현한다.
    requestMagReferenceUpdate();
    CHECK(mag_reference_pending);
    const bool second_request = takeMagReferenceRequest();
    CHECK(second_request);
    CHECK(!mag_reference_pending);
  });

  runCase("mag command enables lazily and disables without changing yaw", [] {
    safety_lock = true;  // 최초 lazy init(블로킹 I2C)은 disarmed에서만 허용된다.
    mag_enabled = false;
    mag_ready = false;
    angleZ = 42.0f;
    bmm.begin_result = 0;

    sendUdpCommandOnce("mag 1");
    CHECK(mag_enabled);
    CHECK(mag_ready);
    CHECK_NEAR(angleZ, 42.0f, 1e-6f);

    sendUdpCommandOnce("mag 0");
    CHECK(!mag_enabled);
    CHECK_NEAR(angleZ, 42.0f, 1e-6f);
  });

  runCase("mag enable refused while armed before init", [] {
    safety_lock = false;  // armed
    mag_enabled = false;
    mag_ready = false;
    bmm.begin_result = 0;
    arduino_fake::serial_output.clear();

    sendUdpCommandOnce("mag 1");

    CHECK(!mag_enabled);
    CHECK(!mag_ready);  // 블로킹 init이 실행되지 않아야 한다.
    CHECK(arduino_fake::serial_output.find(
              "Mag refused (armed, not initialized)") != std::string::npos);
    safety_lock = true;
  });

  runCase("mag command rejects values other than zero and one", [] {
    mag_enabled = false;
    sendUdpCommandOnce("mag 2");
    CHECK(!mag_enabled);
    sendUdpCommandOnce("mag -1");
    CHECK(!mag_enabled);
  });

  runCase("magc는 자동착륙 하강 중 거부되고 기존 계수를 보존한다", [] {
    mag_comp_x = 0.10f;
    mag_comp_y = -0.20f;
    mag_comp_z = 0.30f;
    fs_phase = FS_DESCENDING;
    arduino_fake::serial_output.clear();

    sendUdpCommandOnce("magc 1 2 3");

    CHECK_NEAR(mag_comp_x, 0.10f, 1e-6f);
    CHECK_NEAR(mag_comp_y, -0.20f, 1e-6f);
    CHECK_NEAR(mag_comp_z, 0.30f, 1e-6f);
    CHECK_EQ(countLogOccurrences(
                 "Mag comp refused (auto-land descending)"),
             static_cast<std::size_t>(1));
    fs_phase = FS_NONE;
  });

  runCase("magc는 FS_NONE에서 정상 적용된다", [] {
    mag_comp_x = mag_comp_y = mag_comp_z = 0.0f;
    fs_phase = FS_NONE;

    sendUdpCommandOnce("magc 0.125 -0.25 0.5");

    CHECK_NEAR(mag_comp_x, 0.125f, 1e-6f);
    CHECK_NEAR(mag_comp_y, -0.25f, 1e-6f);
    CHECK_NEAR(mag_comp_z, 0.5f, 1e-6f);
  });

  runCase("magcal reports sample spans and delegates fitting to the host", [] {
    safety_lock = true;
    mag_enabled = false;
    mag_ready = false;
    mag_calibrating = false;
    bmm.begin_result = 0;
    bmm.next_data.float_x = 10.0f;
    bmm.next_data.float_y = -4.0f;
    bmm.next_data.float_z = 8.0f;
    arduino_fake::serial_output.clear();

    sendUdpCommandOnce("magcal 1");
    CHECK(mag_calibrating);
    CHECK(!mag_enabled);

    // magcal 1을 처리한 udp_task 반복 끝에서 첫 값이 이미 한 번 읽힌다.
    sampleMagnetometer(20U);
    bmm.next_data.float_x = -2.0f;
    bmm.next_data.float_y = 8.0f;
    bmm.next_data.float_z = -6.0f;
    sampleMagnetometer(40U);

    arduino_fake::serial_output.clear();
    sendUdpCommandOnce("magcal 0");
    CHECK(!mag_calibrating);
    CHECK(arduino_fake::serial_output.find(
              "[MAGCAL] samples=3") != std::string::npos);
    CHECK(arduino_fake::serial_output.find(
              "span X=12.000000 uT") != std::string::npos);
    CHECK(arduino_fake::serial_output.find(
              "Y=12.000000 uT") != std::string::npos);
    CHECK(arduino_fake::serial_output.find(
              "Z=14.000000 uT") != std::string::npos);
    CHECK(arduino_fake::serial_output.find(
              "scripts/magcal_fit.py") != std::string::npos);
    CHECK(arduino_fake::serial_output.find(
              "MAG_HARD_IRON_OFFSET_") == std::string::npos);
  });

  runCase("magcal is refused while armed", [] {
    safety_lock = false;
    mag_calibrating = false;
    mag_ready = false;
    bmm.begin_result = 0;
    arduino_fake::serial_output.clear();

    sendUdpCommandOnce("magcal 1");

    CHECK(!mag_calibrating);
    CHECK(!mag_ready);
    CHECK(arduino_fake::serial_output.find(
              "Magcal refused (armed)") != std::string::npos);
    safety_lock = true;
  });

  runCase("start is refused while magcal is active", [] {
    safety_lock = true;
    calibration_ok = true;
    angleX = 0.0f;
    angleY = 0.0f;
    imu1_frozen_now = false;
    imu2_frozen_now = false;
    imu_disagree_now = false;
    mag_calibrating = true;
    fs_phase = FS_NONE;

    sendUdpCommandOnce("start");

    CHECK(safety_lock);
    mag_calibrating = false;
  });

  runCase("compute_alpha: below soft threshold is static", [] {
    CHECK_EQ(compute_alpha(1.0f - ACC_DEV_SOFT * 0.5f, 0, 0), ALPHA_STATIC);
  });

  runCase("compute_alpha: soft boundary selects normal", [] {
    CHECK_EQ(compute_alpha(1.0f - ACC_DEV_SOFT, 0, 0), ALPHA_NORMAL);
  });

  runCase("compute_alpha: between thresholds is normal", [] {
    CHECK_EQ(compute_alpha(1.0f - 0.20f, 0, 0), ALPHA_NORMAL);
  });

  runCase("compute_alpha: hard boundary selects dynamic", [] {
    CHECK_EQ(compute_alpha(1.0f - ACC_DEV_HARD, 0, 0), ALPHA_DYN);
  });

  runCase("compute_alpha: above hard threshold is dynamic", [] {
    CHECK_EQ(compute_alpha(1.0f - 0.40f, 0, 0), ALPHA_DYN);
  });

  runCase("checkFreeze: first call initializes without freezing", [] {
    FreezeMon monitor;
    CHECK(!checkFreeze(monitor, eventWith(1, 2, 3, 4, 5, 6), 10));
    CHECK(monitor.init);
    CHECK_EQ(monitor.since, 0U);
  });

  runCase("checkFreeze: repeated sub-threshold deltas freeze after timeout", [] {
    FreezeMon monitor;
    auto event = eventWith(1, 2, 3, 4, 5, 6);
    CHECK(!checkFreeze(monitor, event, 10));
    CHECK(!checkFreeze(monitor, event, 100));
    CHECK(!checkFreeze(monitor, event, 399));
    CHECK(checkFreeze(monitor, event, 400));
  });

  runCase("checkFreeze: real movement resets the timer", [] {
    FreezeMon monitor;
    auto still = eventWith(10, 20, 30);
    auto moved = eventWith(12, 20, 30);
    CHECK(!checkFreeze(monitor, still, 10));
    CHECK(!checkFreeze(monitor, still, 100));
    CHECK(!checkFreeze(monitor, moved, 350));
    CHECK_EQ(monitor.since, 0U);
    CHECK(!checkFreeze(monitor, moved, 400));
    CHECK(!checkFreeze(monitor, moved, 699));
    CHECK(checkFreeze(monitor, moved, 700));
  });

  runCase("checkFreeze: cancelling per-axis magnitudes still count as movement", [] {
    FreezeMon monitor;
    auto first = eventWith(10, 20, 0);
    auto cancelling = eventWith(11, 19, 0);
    CHECK(!checkFreeze(monitor, first, 10));
    CHECK(!checkFreeze(monitor, first, 100));
    CHECK(!checkFreeze(monitor, cancelling, 350));
    CHECK_EQ(monitor.since, 0U);
    CHECK(!checkFreeze(monitor, cancelling, 400));
    CHECK(!checkFreeze(monitor, cancelling, 450));
  });

  runCase("LowPassFilter: alpha follows dt over rc plus dt", [] {
    constexpr float cutoff = 40.0f;
    constexpr float dt = 0.001f;
    LowPassFilter filter(cutoff, dt);
    const float rc = 1.0f / (2.0f * PI * cutoff);
    CHECK_NEAR(filter.alpha, dt / (rc + dt), 1e-7f);
  });

  runCase("LowPassFilter: reset sets state and default reset clears it", [] {
    LowPassFilter filter(40.0f, 0.001f);
    filter.update(10.0f);
    filter.reset(5.0f);
    CHECK_NEAR(filter.last, 5.0f, 1e-6f);
    CHECK_NEAR(filter.update(5.0f), 5.0f, 1e-6f);
    filter.reset();
    CHECK_NEAR(filter.last, 0.0f, 1e-6f);
  });

  runCase("3901-L0X: UART 프레임이 텔레메트리 상태까지 도달한다", [] {
    // 파서 단위 시험은 test_msp_sensor.cpp 가 한다. 여기서 확인하는 것은
    // **배선**이다 — Serial1 -> pollMspSensor -> msp_* 전역.
    arduino_fake::reset();
    mspSensor = MspSensorParser{};
    msp_range_quality = -1;
    msp_flow_quality = -1;
    msp_last_range_ms = 0;
    msp_last_flow_ms = 0;

    auto push = [](std::initializer_list<uint8_t> bytes) {
      for (uint8_t b : bytes) arduino_fake::serial1_rx.push_back(b);
    };
    // MSP2_SENSOR_RANGEFINDER, quality=200, distance=1500mm
    uint8_t frame[] = {'$', 'X', '<', 0x00, 0x01, 0x1F, 0x05, 0x00,
                       200, 0xDC, 0x05, 0x00, 0x00};
    uint8_t crc = 0;
    for (int i = 3; i < 13; i++) crc = mspCrc8DvbS2(crc, frame[i]);
    for (uint8_t b : frame) arduino_fake::serial1_rx.push_back(b);
    arduino_fake::serial1_rx.push_back(crc);
    (void)push;

    pollMspSensor(1000);
    CHECK_EQ((long)msp_range_mm, 1500L);
    CHECK_EQ(msp_range_quality, 200);

    // 신선도: STALE 시간이 지나면 quality 가 -1 이 되어야 한다. 값 자체는
    // 건드리지 않는다 — 모듈이 범위 밖에 음수를 쓰므로 센티넬로 못 쓴다.
    pollMspSensor(1000 + MSP_SENSOR_STALE_MS + 1);
    CHECK_EQ(msp_range_quality, -1);
    CHECK_EQ((long)msp_range_mm, 1500L);
  });

  runCase("per-IMU telemetry uses the fused body-frame axis and sign contract", [] {
    const float g1[3] = {11.0f, -22.0f, 33.0f};
    const float a1[3] = {2048.0f, -4096.0f, 1024.0f};
    const float g2[3] = {-44.0f, 55.0f, -66.0f};
    const float a2[3] = {-1024.0f, 3072.0f, -512.0f};

    const ImuTelemetrySample sample = makeImuTelemetrySample(g1, a1, g2, a2);

    CHECK_NEAR(sample.imu1GyroX, -22.0f, 1e-6f);
    CHECK_NEAR(sample.imu1GyroY, -11.0f, 1e-6f);
    CHECK_NEAR(sample.imu1GyroZ, -33.0f, 1e-6f);
    CHECK_NEAR(sample.imu1AccelX, -2.0f, 1e-6f);
    CHECK_NEAR(sample.imu1AccelY, -1.0f, 1e-6f);
    CHECK_NEAR(sample.imu1AccelZ, 0.5f, 1e-6f);
    CHECK_NEAR(sample.imu2GyroX, 55.0f, 1e-6f);
    CHECK_NEAR(sample.imu2GyroY, 44.0f, 1e-6f);
    CHECK_NEAR(sample.imu2GyroZ, 66.0f, 1e-6f);
    CHECK_NEAR(sample.imu2AccelX, 1.5f, 1e-6f);
    CHECK_NEAR(sample.imu2AccelY, 0.5f, 1e-6f);
    CHECK_NEAR(sample.imu2AccelZ, -0.25f, 1e-6f);

    CHECK_NEAR(
        (sample.imu1GyroX + sample.imu2GyroX) * 0.5f, 16.5f, 1e-6f);
    CHECK_NEAR(
        (sample.imu1GyroY + sample.imu2GyroY) * 0.5f, 16.5f, 1e-6f);
    CHECK_NEAR(
        (sample.imu1GyroZ + sample.imu2GyroZ) * 0.5f, 16.5f, 1e-6f);
    CHECK_NEAR(
        (sample.imu1AccelX + sample.imu2AccelX) * 0.5f, -0.25f, 1e-6f);
    CHECK_NEAR(
        (sample.imu1AccelY + sample.imu2AccelY) * 0.5f, -0.25f, 1e-6f);
    CHECK_NEAR(
        (sample.imu1AccelZ + sample.imu2AccelZ) * 0.5f, 0.125f, 1e-6f);
  });

  runCase("raw IMU ring fills, drains, and preserves FIFO order across wrap", [] {
    ImuRawRing ring = {};
    for (uint32_t value = 0; value < IMU_RAW_RING_SIZE; value++) {
      ImuRawSample sample = {};
      sample.dt_us = static_cast<uint16_t>(value);
      sample.imu1_gyro[0] = static_cast<int16_t>(value);
      CHECK(imuRawRingPush(ring, sample, 100000U + value));
    }
    CHECK_EQ(imuRawRingCount(ring), IMU_RAW_RING_SIZE);

    for (uint32_t value = 0; value < 300; value++) {
      ImuRawSample sample = {};
      CHECK(imuRawRingPop(ring, sample));
      CHECK_EQ(sample.imu1_gyro[0], static_cast<int16_t>(value));
    }
    for (uint32_t value = IMU_RAW_RING_SIZE;
         value < IMU_RAW_RING_SIZE + 300U; value++) {
      ImuRawSample sample = {};
      sample.dt_us = static_cast<uint16_t>(value);
      sample.imu1_gyro[0] = static_cast<int16_t>(value);
      CHECK(imuRawRingPush(ring, sample, 100000U + value));
    }
    CHECK_EQ(imuRawRingCount(ring), IMU_RAW_RING_SIZE);

    for (uint32_t value = 300; value < IMU_RAW_RING_SIZE + 300U; value++) {
      ImuRawSample sample = {};
      CHECK(imuRawRingPop(ring, sample));
      CHECK_EQ(sample.imu1_gyro[0], static_cast<int16_t>(value));
    }
    ImuRawSample empty = {};
    CHECK(!imuRawRingPop(ring, empty));
    CHECK_EQ(imuRawRingCount(ring), 0U);
  });

  runCase("raw IMU ring drops the new sample when full without corrupting old data", [] {
    ImuRawRing ring = {};
    for (uint32_t value = 0; value < IMU_RAW_RING_SIZE; value++) {
      ImuRawSample sample = {};
      sample.imu2_accel[2] = static_cast<int16_t>(value);
      CHECK(imuRawRingPush(ring, sample, 200000U + value));
    }

    ImuRawSample rejected = {};
    rejected.imu2_accel[2] = -1234;
    CHECK(!imuRawRingPush(ring, rejected, 999999U));
    CHECK_EQ(ring.dropped, 1U);
    CHECK_EQ(imuRawRingCount(ring), IMU_RAW_RING_SIZE);

    for (uint32_t value = 0; value < IMU_RAW_RING_SIZE; value++) {
      ImuRawSample sample = {};
      CHECK(imuRawRingPop(ring, sample));
      CHECK_EQ(sample.imu2_accel[2], static_cast<int16_t>(value));
    }
  });

  runCase("raw IMU record and packet preserve register counts and little endian offsets", [] {
    const auto e1 = eventWith(
        0x1357, -2, std::numeric_limits<int16_t>::min(),
        -300, 400, -500);
    const auto e2 = eventWith(
        -600, 700, -800,
        900, -1000, 1100);
    const ImuRawSample sample =
        makeImuRawSample(0x1234, e1, e2, (uint8_t)FS_DESCENDING);

    CHECK_EQ(sizeof(ImuRawSample), 27U);
    CHECK_EQ(sample.failsafe_phase, (uint8_t)FS_DESCENDING);
    CHECK_EQ(sample.imu1_gyro[0], 0x1357);
    CHECK_EQ(sample.imu1_gyro[1], -2);
    CHECK_EQ(sample.imu1_gyro[2], std::numeric_limits<int16_t>::min());
    CHECK_EQ(sample.imu1_accel[0], -300);
    CHECK_EQ(sample.imu2_gyro[0], -600);
    CHECK_EQ(sample.imu2_accel[2], 1100);

    ImuRawDatagram datagram = {};
    const size_t packetSize = buildImuRawDatagram(
        datagram, 0x12345678U, 0xA1B2C3D4U, 0x01020304U, &sample, 1);
    CHECK_EQ(packetSize, 47U);
    const auto *bytes = reinterpret_cast<const uint8_t *>(&datagram);
    CHECK_EQ(bytes[0], static_cast<uint8_t>('Z'));
    CHECK_EQ(bytes[1], static_cast<uint8_t>('I'));
    CHECK_EQ(bytes[2], static_cast<uint8_t>('M'));
    CHECK_EQ(bytes[3], static_cast<uint8_t>('U'));
    CHECK_EQ(bytes[4], IMU_RAW_VERSION);
    CHECK_EQ(bytes[5], 1U);
    CHECK_EQ(bytes[6], 0U);
    CHECK_EQ(bytes[7], 0U);
    CHECK_EQ(bytes[8], 0x78U);
    CHECK_EQ(bytes[9], 0x56U);
    CHECK_EQ(bytes[10], 0x34U);
    CHECK_EQ(bytes[11], 0x12U);
    CHECK_EQ(bytes[12], 0xD4U);
    CHECK_EQ(bytes[13], 0xC3U);
    CHECK_EQ(bytes[14], 0xB2U);
    CHECK_EQ(bytes[15], 0xA1U);
    CHECK_EQ(bytes[16], 0x04U);
    CHECK_EQ(bytes[17], 0x03U);
    CHECK_EQ(bytes[18], 0x02U);
    CHECK_EQ(bytes[19], 0x01U);
    CHECK_EQ(bytes[20], 0x34U);
    CHECK_EQ(bytes[21], 0x12U);
    CHECK_EQ(bytes[22], 0x57U);
    CHECK_EQ(bytes[23], 0x13U);
    CHECK_EQ(bytes[24], 0xFEU);
    CHECK_EQ(bytes[25], 0xFFU);
    CHECK_EQ(bytes[26], 0x00U);
    CHECK_EQ(bytes[27], 0x80U);
    // v2: failsafe_phase 는 샘플의 마지막 바이트다 (20 헤더 + 26 = 46).
    CHECK_EQ(bytes[46], (uint8_t)FS_DESCENDING);
  });

  runCase("raw UDP command defaults on and raw zero disables production", [] {
    CHECK(raw_stream_enabled);
    arduino_fake::serial_output.clear();

    sendUdpCommandOnce("raw 2");
    CHECK(raw_stream_enabled);

    sendUdpCommandOnce("raw 0");
    CHECK(!raw_stream_enabled);
    CHECK(arduino_fake::serial_output.find("Raw IMU OFF") != std::string::npos);

    imuRawRing = {};
    connectionEstablished = true;
    runPidTicks(1);
    CHECK_EQ(imuRawRingCount(imuRawRing), 0U);
  });

  runCase("pid task does not publish or drop raw samples before connection", [] {
    imuRawRing = {};
    ImuRawSample queued = {};
    for (uint32_t value = 0; value < IMU_RAW_RING_SIZE; value++) {
      CHECK(imuRawRingPush(imuRawRing, queued, 100000U + value));
    }
    rawProducerTimeValid = false;
    raw_stream_enabled = true;
    connectionEstablished = false;
    fault_imu1 = false;
    fault_imu2 = false;
    fault_disagree = false;

    runPidTicks(1);

    CHECK_EQ(imuRawRingCount(imuRawRing), IMU_RAW_RING_SIZE);
    CHECK_EQ(imuRawRing.dropped, 0U);
  });

  runCase("pid task publishes raw registers while connected and enabled", [] {
    imuRawRing = {};
    rawProducerTimeValid = false;
    raw_stream_enabled = true;
    connectionEstablished = true;
    fault_imu1 = false;
    fault_imu2 = false;
    fault_disagree = false;
    IMU1.next_event = eventWith(11, -12, 13, -14, 15, -16);
    IMU2.next_event = eventWith(-21, 22, -23, 24, -25, 26);

    runPidTicks(1);
    CHECK_EQ(imuRawRingCount(imuRawRing), 1U);
    ImuRawSample sample = {};
    CHECK(imuRawRingPop(imuRawRing, sample));
    CHECK_EQ(sample.imu1_gyro[0], 11);
    CHECK_EQ(sample.imu1_accel[0], -14);
    CHECK_EQ(sample.imu2_gyro[0], -21);
    CHECK_EQ(sample.imu2_accel[2], 26);
  });

  runCase("yaw limitation does not stop roll and pitch integration", [] {
    arduino_fake::reset();
    fault_rc = fault_imu1 = fault_imu2 = fault_disagree = fault_attitude = false;
    imu1_frozen_now = imu2_frozen_now = imu_disagree_now = false;
    calibration_ok = true;
    mag_calibrating = false;
    safety_lock = true;
    fs_phase = FS_NONE;
    angleX = angleY = angleZ = 0.0f;
    gyro_bias1[0] = gyro_bias1[1] = gyro_bias1[2] = 0.0f;
    gyro_bias2[0] = gyro_bias2[1] = gyro_bias2[2] = 0.0f;
    setFakeAccelMagnitude(1.0f, 0);
    sendUdpCommandOnce("start");
    runPidTicks(1);
    CHECK(!safety_lock);

    base_throttle = 1200;
    min_throttle = 1100;
    max_throttle = 1300;
    lastRcMs = millis();
    Kp_Angle_Roll = Kp_Angle_Pitch = Kp_Angle_Yaw = 1.0f;
    Kp_Rate_Roll = Kp_Rate_Pitch = Kp_Rate_Yaw = 1.0f;
    Ki_Rate_Roll = Ki_Rate_Pitch = Ki_Rate_Yaw = 1.0f;
    Kd_Rate_Roll = Kd_Rate_Pitch = Kd_Rate_Yaw = 0.0f;
    targetAngleX = 5.0f;
    targetAngleY = 5.0f;
    targetYawRate = 100.0f;
    iTermRoll = iTermPitch = iTermYaw = 0.0f;

    runPidTicks(1);

    CHECK(iTermRoll > 0.0f);
    CHECK(iTermPitch > 0.0f);
    CHECK_NEAR(iTermYaw, 0.0f, 1e-7f);
  });

  runCase("pid task consumes prior yaw scale for the 150ms authority gate", [] {
    prepareYawAuthorityFlight();
    bool saw_limited_allocation = false;
    bool saw_limited_state = false;
    uint32_t first_limited_allocation_ms = 0U;
    uint32_t limited_state_elapsed_ms = 0U;
    float limited_target_angle = 0.0f;
    float limited_current_angle = 0.0f;
    float limited_target_rate = 0.0f;
    arduino_fake::pre_tick_hook = [&](uint32_t tick) {
      setFakeYawRateAndAccel(20.0f, 1.0f, tick);
      if (!saw_limited_allocation && mixer_scaled) {
        saw_limited_allocation = true;
        first_limited_allocation_ms = millis();
      }
      if (saw_limited_allocation &&
          (uint32_t)(millis() - first_limited_allocation_ms) <= 150U) {
        CHECK_EQ(yaw_authority_state, (int)YAW_AUTH_NORMAL);
      }
      if (yaw_authority_state == YAW_AUTH_LIMITED) {
        saw_limited_state = true;
        limited_state_elapsed_ms =
            (uint32_t)(millis() - first_limited_allocation_ms);
        limited_target_angle = targetAngleZ;
        limited_current_angle = angleZ;
        limited_target_rate = tgtRate[2];
        throw arduino_fake::TaskDelayExit{};
      }
    };

    runPidTicks(400U);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK(saw_limited_allocation);
    CHECK(saw_limited_state);
    CHECK_EQ(limited_state_elapsed_ms, 151U);
    CHECK_NEAR(limited_target_angle, limited_current_angle, 1e-5f);
    CHECK_NEAR(limited_target_rate, 0.0f, 1e-6f);
  });

  runCase("dual IMU loss immediately resets limited yaw authority state", [] {
    prepareYawAuthorityFlight();
    bool injected_dual_loss = false;
    bool observed_immediate_lock = false;
    bool authority_was_limited = false;
    bool allocation_was_limited = false;
    int state_at_immediate_lock = -1;
    float scale_before_dual_loss = 1.0f;
    float scale_at_immediate_lock = -1.0f;
    arduino_fake::pre_tick_hook = [&](uint32_t tick) {
      setFakeYawRateAndAccel(20.0f, 1.0f, tick);
      if (!injected_dual_loss && yaw_authority_state == YAW_AUTH_LIMITED) {
        authority_was_limited = true;
        allocation_was_limited = mixer_scaled;
        scale_before_dual_loss = mixer_yaw_scale;
        fault_imu1 = true;
        fault_imu2 = true;
        injected_dual_loss = true;
      } else if (injected_dual_loss && safety_lock) {
        observed_immediate_lock = true;
        state_at_immediate_lock = yaw_authority_state;
        scale_at_immediate_lock = mixer_yaw_scale;
        throw arduino_fake::TaskDelayExit{};
      }
    };

    runPidTicks(400U);
    arduino_fake::pre_tick_hook = nullptr;

    CHECK(authority_was_limited);
    CHECK(allocation_was_limited);
    CHECK(scale_before_dual_loss < 1.0f);
    CHECK(injected_dual_loss);
    CHECK(observed_immediate_lock);
    CHECK(safety_lock);
    CHECK_EQ(active_imus, 0);
    for (int motor : motorOut) CHECK_EQ(motor, 1000);
    CHECK_EQ(state_at_immediate_lock, (int)YAW_AUTH_NORMAL);
    CHECK_NEAR(scale_at_immediate_lock, 1.0f, 1e-6f);
  });

  runCase("telemetry packet uses one allocation snapshot for legacy and axis scales", [] {
    arduino_fake::reset();
    wifi_udp_fake::reset();
    connectionEstablished = true;
    calibration_ok = true;
    active_imus = 2;

    MotorMix limited = {};
    limited.collective_us = 1210.0f;
    limited.rp_scale = 0.8f;
    limited.yaw_scale = 0.4f;
    limited.scaled = true;
    publishAllocationTelemetry(
        limited, 11.0f, 12.0f, 13.0f, 1.0f, 2.0f, 3.0f,
        YAW_AUTH_LIMITED);

    // Deliberately disagree with the published tick. Field 19 must come from
    // the same protected record as fields 66 and 67, not this live global.
    mixer_scaled = false;
    sendTelemetry();

    const std::vector<std::string> fields =
        splitCsv(wifi_udp_fake::telemetry_output);
    CHECK_EQ(fields.size(), 75U);
    CHECK_EQ(fields[18], "1");
    CHECK_EQ(fields[65], "0.800");
    CHECK_EQ(fields[66], "0.400");
  });

  runCase("default allocation reset clears legacy and axis diagnostics", [] {
    MotorMix limited = {};
    limited.collective_us = 1200.0f;
    limited.rp_scale = 0.7f;
    limited.yaw_scale = 0.3f;
    limited.scaled = true;
    mixer_scaled = true;
    publishAllocationTelemetry(
        limited, 10.0f, 20.0f, 30.0f, 1.0f, 2.0f, 3.0f,
        YAW_AUTH_LIMITED);

    // The post-mix lock path relies only on this common reset helper.
    resetAllocationTelemetry();
    const AllocationTelemetrySample sample =
        readAllocationTelemetrySnapshot();

    CHECK(!mixer_scaled);
    CHECK_NEAR(sample.rp_scale, 1.0f, 1e-6f);
    CHECK_NEAR(sample.yaw_scale, 1.0f, 1e-6f);
    CHECK_EQ(sample.yaw_authority_state, (int)YAW_AUTH_NORMAL);
  });

  std::cout << "\n" << (test_count - failure_count) << "/" << test_count
            << " native control-math cases passed\n";
  return failure_count == 0 ? 0 : 1;
}
