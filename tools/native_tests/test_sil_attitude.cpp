#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "control_allocator.h"

// Test-only allocator dispatch. The production allocator is still called
// unchanged for normal SIL runs. The negative-control run selects an
// independent legacy policy before the sketch is included, so the real PID
// tick receives the same scale that produces motorOut and drives the plant.
static bool silUseUniformAllocator = false;
static ControlAllocation silLastDispatchedAllocation = {
    {1000, 1000, 1000, 1000}, 1000.0f, 1.0f, 1.0f, false};

static ControlAllocation silUniformAllocation(float roll, float pitch,
                                               float yaw, int throttle,
                                               int minMotor, int maxMotor) {
  // Independent legacy policy: compose all three axes first, then apply one
  // span scale to every axis. This does not call or reproduce the production
  // RP-first priority solver.
  minMotor = std::max(1000, std::min(2000, minMotor));
  maxMotor = std::max(minMotor, std::min(2000, maxMotor));
  const double rollValue = std::isfinite(roll) ? roll : 0.0;
  const double pitchValue = std::isfinite(pitch) ? pitch : 0.0;
  const double yawValue = std::isfinite(yaw) ? yaw : 0.0;
  std::array<double, 4> diff = {
      -pitchValue + rollValue - yawValue,
      pitchValue - rollValue - yawValue,
      -pitchValue - rollValue + yawValue,
      pitchValue + rollValue + yawValue,
  };
  const auto bounds = std::minmax_element(diff.begin(), diff.end());
  const double available = static_cast<double>(maxMotor - minMotor);
  const double span = *bounds.second - *bounds.first;
  double scale = 1.0;
  if (span > available && span > 0.0) scale = available / span;
  for (double &value : diff) value *= scale;

  const auto scaledBounds = std::minmax_element(diff.begin(), diff.end());
  const double collectiveLow = minMotor - *scaledBounds.first;
  const double collectiveHigh = maxMotor - *scaledBounds.second;
  const double collective = std::max(
      collectiveLow,
      std::min(collectiveHigh, static_cast<double>(throttle)));

  ControlAllocation out = {};
  out.collective_us = static_cast<float>(collective);
  out.rp_scale = static_cast<float>(scale);
  out.yaw_scale = static_cast<float>(scale);
  out.scaled = scale < 1.0;
  for (std::size_t index = 0; index < diff.size(); ++index) {
    const int rounded = static_cast<int>(std::lround(collective + diff[index]));
    out.motor[index] = std::max(minMotor, std::min(maxMotor, rounded));
  }
  return out;
}

static ControlAllocation silDispatchAllocateControl(
    float roll, float pitch, float yaw, int throttle,
    int minMotor, int maxMotor) {
  silLastDispatchedAllocation =
      silUseUniformAllocator
          ? silUniformAllocation(roll, pitch, yaw, throttle,
                                 minMotor, maxMotor)
          : allocateControl(roll, pitch, yaw, throttle,
                            minMotor, maxMotor);
  return silLastDispatchedAllocation;
}

#define allocateControl silDispatchAllocateControl
#include "dual_imu_cascade_pwm.ino"
#undef allocateControl

namespace {

constexpr double kDt = 0.001;
constexpr double kDegToRad = PI / 180.0;
constexpr double kRadToDeg = 180.0 / PI;
constexpr double kGravityMs2 = 9.81;

#ifdef SIL_INJECT_SIGN_FAULT
constexpr bool kInjectRollSignFault = true;
#else
constexpr bool kInjectRollSignFault = false;
#endif

int test_count = 0;
int failure_count = 0;
int report_count = 0;
int report_failure_count = 0;

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

#define CHECK_MSG(expression, detail) \
  do { \
    if (!(expression)) fail(#expression, __LINE__, (detail)); \
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

void checkAtMost(double actual, double limit,
                 const char *expression, int line) {
  if (!(actual <= limit)) {
    std::ostringstream detail;
    detail << "actual=" << actual << ", limit=" << limit;
    fail(expression, line, detail.str());
  }
}

#define CHECK_LE(actual, limit) \
  checkAtMost((actual), (limit), #actual " <= " #limit, __LINE__)

void checkAtLeast(double actual, double limit,
                  const char *expression, int line) {
  if (!(actual >= limit)) {
    std::ostringstream detail;
    detail << "actual=" << actual << ", limit=" << limit;
    fail(expression, line, detail.str());
  }
}

#define CHECK_GE(actual, limit) \
  checkAtLeast((actual), (limit), #actual " >= " #limit, __LINE__)

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

void runReport(const std::string &name, const std::function<void()> &body) {
  report_count++;
  try {
    body();
    std::cout << "[REPORT] " << name << " complete\n";
  } catch (const std::exception &error) {
    report_failure_count++;
    std::cerr << "[REPORT-FAIL] " << name << ": " << error.what() << '\n';
  }
}

struct PlantParameters {
  // 실측값이 없는 소형 쿼드의 guessed 파라미터다. 정착시간과 정상상태 수치는
  // 이 값에 의존하지만, 합리적 범위에서 제어 부호와 유계성 판정은 강건하다.
  double arm_length_m = 0.12;
  double ix_kg_m2 = 0.003;
  double iy_kg_m2 = 0.003;
  double iz_kg_m2 = 0.006;
  double thrust_per_us_n = 0.0025;
  // 4 * 0.0025 N/us * (1340 - 1000) us / 9.81: hover 1340us에서
  // 총추력과 중력이 평형을 이루는 질량이다(약 0.3466 kg).
  double mass_kg = 4.0 * 0.0025 * (1340.0 - 1000.0) / kGravityMs2;
  // 기본 접지는 **무게추 고정 벤치의 응답에 맞춘 모델**이다.
  //
  // 2026-08-01 이전에는 80 N/m였는데 그것은 정적 눌림 4.25cm짜리 폼 매트에
  // 해당한다. 그 모델의 지면 프로브 응답은 0.0464~0.0512g로, 실측
  // (2026-07-29 벤치, 프롭 ON·무게추, 하드 플로어) 0.0010~0.0040g의 10배가
  // 넘었다. 당시 "실기 공중 CUT_LANDED"로 읽은 010251은 실제로는 지면이었다.
  // 따라서 실기 공중 오판 방지에 임계 <0.0070이 필요하다거나, 어떤 임계도
  // 두 자료를 동시에 만족하지 못한다는 결론은 철회됐다.
  //
  // 강성을 훑어 실측 밴드에 맞췄다:
  //   k=  80 → 0.0464g   k= 400 → 0.0232g   k=1500 → 0.0052g
  //   k=2000 → 0.0026g   k=2500 → 0.0011g   k=3000 이상 → 0.0000g
  // 2000 N/m가 이 고정 벤치의 0.0010~0.0040g 밴드 안에 들어간다. 자유롭게
  // 흔들리는 실제 착륙장치의 접촉을 검증한 값은 아니다. 3000 이상에서 응답이
  // 거의 사라지는 것도 이 모델과 조건 안의 결과다. docs/failsafe_land_research.md.
  //
  // 무른 표면은 사라진 게 아니라 R6 케이스로 따로 검증한다(아래 참조).
  double k_ground = 2000.0;  // N/m
  double c_ground = 52.65;   // N*s/m (임계감쇠 2*sqrt(k*m))
  double k_ge = 0.15;
  double z_ge = 0.15;      // m

  double arm_projection_m() const { return arm_length_m / std::sqrt(2.0); }
  double yaw_moment_arm_m() const { return 0.06 * arm_projection_m(); }
};

const PlantParameters kPlantParameters;

struct PlantState {
  double phi = 0.0;
  double theta = 0.0;
  double psi = 0.0;
  double p = 0.0;
  double q = 0.0;
  double r = 0.0;
  double z_m = 0.0;
  double vz_ms = 0.0;
};

struct Disturbance {
  double x_nm = 0.0;
  double y_nm = 0.0;
  double z_nm = 0.0;
  double vertical_force_n = 0.0;
};

struct RunConfig {
  PlantState initial;
  uint32_t ticks = 0;
  float target_roll_deg = 0.0f;
  float target_pitch_deg = 0.0f;
  float target_yaw_deg = 0.0f;
  float ki_roll = 0.005f;
  float ki_pitch = 0.005f;
  float ki_yaw = 0.05f;
  bool inject_roll_sign_fault = false;
  bool vertical_enabled = false;
  bool accel_noise_enabled = false;
  double accel_noise_sd_g = 0.04;
  double accel_common_mode_g = 0.0;
  uint32_t accel_common_mode_start_tick =
      std::numeric_limits<uint32_t>::max();
  int base_throttle_us = 1150;
  uint32_t rc_disconnect_tick = std::numeric_limits<uint32_t>::max();
  uint32_t rc_reconnect_tick = std::numeric_limits<uint32_t>::max();
  uint32_t resume_tick = std::numeric_limits<uint32_t>::max();
  PlantParameters plant_parameters = kPlantParameters;
  double vertical_drag_n_per_ms = 0.0;
  uint32_t vertical_drag_start_tick =
      std::numeric_limits<uint32_t>::max();
  std::function<Disturbance(uint32_t)> disturbance_for_interval;
  std::function<int(uint32_t)> throttle_for_tick;
  std::function<void(uint32_t, PlantState &)> state_for_tick;
  // Test-only negative-control policy. This never enters flight firmware.
  bool uniform_allocator_oracle = false;
};

struct PlantStep {
  double vertical_acceleration_ms2 = 0.0;
  double specific_force_g = 1.0;
  double ground_force_n = 0.0;
};

struct Sample {
  uint32_t tick = 0;
  double time_s = 0.0;
  PlantState plant;
  double estimated_roll_deg = 0.0;
  double estimated_pitch_deg = 0.0;
  std::array<int, 4> motors = {1000, 1000, 1000, 1000};
  double i_roll_us = 0.0;
  double i_pitch_us = 0.0;
  double i_yaw_us = 0.0;
  double mixer_rp_scale = 1.0;
  double mixer_yaw_scale = 1.0;
  double plant_allocator_scale = 1.0;
  double target_yaw_rate_dps = 0.0;
  int yaw_authority_state = YAW_AUTH_NORMAL;
  int pid_loop_hz = 0;
  bool yaw_hold = false;
  bool mixer_scaled_now = false;
  bool safety_locked = false;
  bool fault_rc_now = false;
  bool fault_imu1_now = false;
  bool fault_imu2_now = false;
  bool fault_disagree_now = false;
  bool fault_attitude_now = false;
  uint8_t failsafe_phase = FS_NONE;
  int base_throttle_us = 1000;
  uint8_t failsafe_probe_state = FS_PROBE_WAIT;
  uint8_t failsafe_probe_no_response = 0;
  double failsafe_probe_response_g = 0.0;
  double accel_magnitude_g = 1.0;
  double vertical_acceleration_ms2 = 0.0;
  double specific_force_g = 1.0;
  double ground_force_n = 0.0;
};

struct RunResult {
  std::vector<Sample> samples;
  bool all_finite = true;
  uint32_t first_nonfinite_tick = std::numeric_limits<uint32_t>::max();
  bool all_motors_in_range = true;
  uint32_t first_bad_motor_tick = std::numeric_limits<uint32_t>::max();
  bool safety_lock_ever = false;
  uint32_t first_safety_lock_tick = std::numeric_limits<uint32_t>::max();
  bool raw_saturated = false;
  uint32_t first_raw_saturation_tick = std::numeric_limits<uint32_t>::max();
  double max_abs_roll_deg = 0.0;
  uint32_t max_abs_roll_tick = 0;
  double max_abs_pitch_deg = 0.0;
  uint32_t max_abs_pitch_tick = 0;
  double max_abs_i_roll_us = 0.0;
  uint32_t max_abs_i_roll_tick = 0;
  double max_abs_i_pitch_us = 0.0;
  uint32_t max_abs_i_pitch_tick = 0;
  double max_abs_i_yaw_us = 0.0;
};

void sendUdpCommandOnce(const std::string &command) {
  wifi_udp_fake::incoming_packet = command;
  arduino_fake::stop_on_task_delay = true;
  bool stopped = false;
  try {
    udp_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
    stopped = true;
  } catch (...) {
    arduino_fake::stop_on_task_delay = false;
    throw;
  }
  arduino_fake::stop_on_task_delay = false;
  CHECK_MSG(stopped, "udp_task did not stop at the shim delay");
}

void runPidTicksForHarness(uint32_t ticks) {
  arduino_fake::tick_index = 0;
  arduino_fake::tick_limit = ticks;
  try {
    pid_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
  } catch (...) {
    arduino_fake::tick_limit = 0;
    throw;
  }
  arduino_fake::tick_limit = 0;
}

int16_t roundedRaw(double value, bool &saturated) {
  if (!std::isfinite(value)) {
    saturated = true;
    return 0;
  }
  const double low = static_cast<double>(std::numeric_limits<int16_t>::min());
  const double high = static_cast<double>(std::numeric_limits<int16_t>::max());
  if (value < low || value > high) saturated = true;
  const double clipped = std::max(low, std::min(high, value));
  return static_cast<int16_t>(std::lround(clipped));
}

int16_t addRaw(int16_t value, int delta, bool &saturated) {
  const int sum = static_cast<int>(value) + delta;
  if (sum < std::numeric_limits<int16_t>::min() ||
      sum > std::numeric_limits<int16_t>::max()) {
    saturated = true;
  }
  return static_cast<int16_t>(std::max(
      static_cast<int>(std::numeric_limits<int16_t>::min()),
      std::min(static_cast<int>(std::numeric_limits<int16_t>::max()), sum)));
}

bool injectImuFromPlantImpl(const PlantState &state, uint32_t tick,
                            const double *specific_force_g) {
  bool saturated = false;
  const double p_dps = state.p * kRadToDeg;
  const double q_dps = state.q * kRadToDeg;
  const double r_dps = state.r * kRadToDeg;

  inv_imu_sensor_event_t e1 = {};
  // Physics source: invert the validated accel sensor->body rotation for the
  // roll/pitch gyro too. Do not derive these signs from the firmware gyro map.
  e1.gyro[0] = roundedRaw(-q_dps / GYRO_SCALE, saturated);
  e1.gyro[1] = roundedRaw(p_dps / GYRO_SCALE, saturated);
  // Yaw keeps its separately bench-verified convention.
  e1.gyro[2] = roundedRaw(-r_dps / GYRO_SCALE, saturated);

  const double gbx = -std::sin(state.theta);
  const double gby = std::sin(state.phi) * std::cos(state.theta);
  const double gbz = std::cos(state.phi) * std::cos(state.theta);
  if (specific_force_g == nullptr) {
    // vertical_enabled=false의 기존 경로. 연산 순서까지 보존한다.
    e1.accel[0] = roundedRaw(-gby / ACCEL_SCALE, saturated);
    e1.accel[1] = roundedRaw(gbx / ACCEL_SCALE, saturated);
    e1.accel[2] = roundedRaw(gbz / ACCEL_SCALE, saturated);
  } else {
    e1.accel[0] =
        roundedRaw(*specific_force_g * -gby / ACCEL_SCALE, saturated);
    e1.accel[1] =
        roundedRaw(*specific_force_g * gbx / ACCEL_SCALE, saturated);
    e1.accel[2] =
        roundedRaw(*specific_force_g * gbz / ACCEL_SCALE, saturated);
  }

  // 정착 후에도 freeze 감시가 동일 프레임으로 오인하지 않게 하는 결정적
  // zero-mean dither다. gyro 1 LSB는 약 0.061 dps, accel은 2 LSB다.
  const int toggle = (tick & 1U) ? -1 : 1;
  e1.gyro[2] = addRaw(e1.gyro[2], toggle, saturated);
  e1.accel[2] = addRaw(e1.accel[2], 2 * toggle, saturated);

  inv_imu_sensor_event_t e2 = {};
  for (int axis = 0; axis < 3; axis++) {
    const int sign = IMU2_SIGN[axis] < 0.0f ? -1 : 1;
    e2.gyro[axis] = addRaw(0, sign * static_cast<int>(e1.gyro[axis]), saturated);
    e2.accel[axis] = addRaw(0, sign * static_cast<int>(e1.accel[axis]), saturated);
  }

  IMU1.next_event = e1;
  IMU2.next_event = e2;
  IMU1.read_status = 0;
  IMU2.read_status = 0;
  return saturated;
}

bool injectImuFromPlant(const PlantState &state, uint32_t tick) {
  return injectImuFromPlantImpl(state, tick, nullptr);
}

bool injectImuFromPlant(const PlantState &state, uint32_t tick,
                        double specific_force_g) {
  return injectImuFromPlantImpl(state, tick, &specific_force_g);
}

void resetFirmwareState(const PlantState &initial) {
  arduino_fake::reset();
  arduino_fake::stop_on_task_delay = false;
  wifi_udp_fake::reset();

  connectionEstablished = false;
  laptopIP = IPAddress();
  laptopPort = 0;
  packetBuffer[0] = '\0';

  Kp_Angle_Roll = 6.0f;
  Kp_Angle_Pitch = 6.0f;
  Kp_Angle_Yaw = 3.0f;
  Kp_Rate_Roll = 0.50f;
  Ki_Rate_Roll = 0.005f;
  Kd_Rate_Roll = 0.015f;
  Kp_Rate_Pitch = 0.50f;
  Ki_Rate_Pitch = 0.005f;
  Kd_Rate_Pitch = 0.015f;
  Kp_Rate_Yaw = 1.50f;
  Ki_Rate_Yaw = 0.05f;
  Kd_Rate_Yaw = 0.0f;

  base_throttle = 1000;
  min_throttle = 1050;
  max_throttle = 1300;
  yaw_hold_override = false;
  safety_lock = true;
  safety_disarm_requested = false;
  safety_arm_requested = false;
  failsafe_resume_requested = false;
  fs_phase = FS_NONE;
  fs_probe_state = FS_PROBE_WAIT;
  fs_probe_no_response = 0;
  fs_probe_response_g = 0.0f;
  hover_est = 0.0f;
  hover_valid = false;
  calibration_ok = true;

  targetAngleX = 0.0f;
  targetAngleY = 0.0f;
  targetAngleZ = 0.0f;
  targetYawRate = 0.0f;
  angleX = static_cast<float>(initial.phi * kRadToDeg);
  angleY = static_cast<float>(initial.theta * kRadToDeg);
  angleZ = static_cast<float>(initial.psi * kRadToDeg);
  gyroX = static_cast<float>(initial.p * kRadToDeg);
  gyroY = static_cast<float>(initial.q * kRadToDeg);
  gyroZ = static_cast<float>(initial.r * kRadToDeg);
  accX = accY = accZ = 0.0f;
  for (int index = 0; index < 4; index++) motorOut[index] = 1000;
  for (int axis = 0; axis < 3; axis++) tgtRate[axis] = 0.0f;
  pidLoopHz = 0;
  iTermRoll = iTermPitch = iTermYaw = 0.0f;

  for (int axis = 0; axis < 3; axis++) {
    gyro_bias1[axis] = 0.0f;
    gyro_bias2[axis] = 0.0f;
  }
  accel_scale1 = 1.0f;
  accel_scale2 = 1.0f;
  lastRcMs = 0;
  fault_rc = false;
  fault_imu1 = false;
  fault_imu2 = false;
  fault_disagree = false;
  fault_attitude = false;
  active_imus = 2;
  mixer_scaled = false;
  imu1_frozen_now = false;
  imu2_frozen_now = false;
  imu_disagree_now = false;
  lastRcSeq = 0;
  rcSeqValid = false;
  rcTotalPkts = 0;
  rcDroppedPkts = 0;
  yaw_authority_state = YAW_AUTH_NORMAL;
  resetAllocationTelemetry();

  IMU1.next_event = {};
  IMU2.next_event = {};
  IMU1.read_status = 0;
  IMU2.read_status = 0;
  (void)injectImuFromPlant(initial, 0);
}

Disturbance disturbanceAt(const RunConfig &config, uint32_t interval,
                          const PlantState &state) {
  Disturbance disturbance =
      config.disturbance_for_interval
          ? config.disturbance_for_interval(interval)
          : Disturbance{};
  if (interval >= config.vertical_drag_start_tick) {
    disturbance.vertical_force_n -=
        config.vertical_drag_n_per_ms * state.vz_ms;
  }
  return disturbance;
}

double deterministicAccelNoiseG(uint32_t tick, double target_sd_g) {
  // 세 로그는 20Hz(|a|만)라 10Hz를 넘는 축별/프롭 진동의 근거가 없다.
  // 따라서 관측되고 착지 LPF를 통과하는 1~8Hz만 합성한다. 10초 창에서
  // 서로소 cycle 수(11/17/23/41/73)를 갖는 고정 위상 사인 합이라 매 실행이
  // 동일하며, 이론 RMS가 target_sd_g가 되도록 가중치를 정규화한다.
  constexpr std::array<double, 5> frequencies_hz = {
      1.1, 1.7, 2.3, 4.1, 7.3};
  constexpr std::array<double, 5> weights = {
      1.0, 0.75, 0.50, 0.20, 0.10};
  constexpr std::array<double, 5> phases_rad = {
      0.31, 1.17, -0.83, 2.29, -1.91};
  constexpr double weight_square_sum =
      1.0 * 1.0 + 0.75 * 0.75 + 0.50 * 0.50 +
      0.20 * 0.20 + 0.10 * 0.10;
  const double time_s = tick * kDt;
  double unit_sum = 0.0;
  for (std::size_t index = 0; index < frequencies_hz.size(); index++) {
    unit_sum += weights[index] *
                std::sin(2.0 * PI * frequencies_hz[index] * time_s +
                         phases_rad[index]);
  }
  return target_sd_g * std::sqrt(2.0 / weight_square_sum) * unit_sum;
}

PlantStep integratePlant(PlantState &state, const Disturbance &disturbance,
                         bool inject_roll_sign_fault, bool vertical_enabled,
                         const PlantParameters &parameters) {
  std::array<double, 4> applied = {
      static_cast<double>(motorOut[0]), static_cast<double>(motorOut[1]),
      static_cast<double>(motorOut[2]), static_cast<double>(motorOut[3])};

  if (inject_roll_sign_fault) {
    // 뮤테이션은 mixer roll 축의 부호 하나(roll allocation column 전체)를
    // R -> -R로 뒤집는다. 모터 하나의 계수만 바꾸면 authority가 절반으로
    // 줄 뿐 부호가 유지되어 발산 결함이 아니다. 아래 기체 레이아웃/r×F
    // 토크식은 그대로 두므로 검증이 tautology가 되지 않는다.
    const double roll_mode =
        (applied[0] - applied[1] - applied[2] + applied[3]) * 0.25;
    applied[0] -= 2.0 * roll_mode;
    applied[1] += 2.0 * roll_mode;
    applied[2] += 2.0 * roll_mode;
    applied[3] -= 2.0 * roll_mode;
    for (double &motor : applied) {
      motor = std::max(static_cast<double>(min_throttle),
                       std::min(static_cast<double>(max_throttle), motor));
    }
  }

  std::array<double, 4> thrust = {};
  for (std::size_t index = 0; index < thrust.size(); index++) {
    thrust[index] = parameters.thrust_per_us_n *
                    std::max(0.0, applied[index] - 1000.0);
  }

  const double a = parameters.arm_projection_m();
  const double tau_x = a * (thrust[0] - thrust[1] - thrust[2] + thrust[3]);
  const double tau_y = a * (-thrust[0] + thrust[1] - thrust[2] + thrust[3]);
  // CW(+)/CCW(-) 반작용 관례다. 실제 yaw 부호는 전원 벤치에서 확정한다.
  const double tau_z = parameters.yaw_moment_arm_m() *
                       (thrust[0] + thrust[1] - thrust[2] - thrust[3]);

  PlantStep step;
  if (vertical_enabled) {
    const double height_m = std::max(0.0, state.z_m);
    // 지면 아래 spring 압축에서는 고도를 0으로 clamp해 ge를 [1, 1+k_ge]에
    // 묶는다. 물리적인 지면 위(z>=0)에서는 요구된 지수식 그대로다.
    const double ground_effect =
        1.0 + parameters.k_ge * std::exp(-height_m / parameters.z_ge);
    const double total_thrust_n =
        ground_effect *
        (thrust[0] + thrust[1] + thrust[2] + thrust[3]);
    if (state.z_m < 0.0) {
      step.ground_force_n =
          std::max(0.0, -parameters.k_ground * state.z_m -
                            parameters.c_ground * state.vz_ms);
    }
    step.vertical_acceleration_ms2 =
        total_thrust_n * std::cos(state.phi) * std::cos(state.theta) /
            parameters.mass_kg -
        kGravityMs2 + step.ground_force_n / parameters.mass_kg +
        disturbance.vertical_force_n / parameters.mass_kg;
    step.specific_force_g =
        (step.vertical_acceleration_ms2 + kGravityMs2) / kGravityMs2;
    // 준음적분: 새 속도로 위치를 갱신해 1ms spring contact를 안정적으로 푼다.
    state.vz_ms += step.vertical_acceleration_ms2 * kDt;
    state.z_m += state.vz_ms * kDt;
  }

  const double old_p = state.p;
  const double old_q = state.q;
  const double old_r = state.r;
  state.phi += old_p * kDt;
  state.theta += old_q * kDt;
  state.psi += old_r * kDt;
  state.p += (tau_x + disturbance.x_nm) / parameters.ix_kg_m2 * kDt;
  state.q += (tau_y + disturbance.y_nm) / parameters.iy_kg_m2 * kDt;
  state.r += (tau_z + disturbance.z_nm) / parameters.iz_kg_m2 * kDt;
  return step;
}

bool finiteState(const PlantState &state) {
  return std::isfinite(state.phi) && std::isfinite(state.theta) &&
         std::isfinite(state.psi) && std::isfinite(state.p) &&
         std::isfinite(state.q) && std::isfinite(state.r) &&
         std::isfinite(state.z_m) && std::isfinite(state.vz_ms);
}

void appendSample(RunResult &result, uint32_t tick, const PlantState &state,
                  const PlantStep &plant_step,
                  double plant_allocator_scale = 1.0) {
  Sample sample;
  sample.tick = tick;
  sample.time_s = tick * kDt;
  sample.plant = state;
  sample.estimated_roll_deg = angleX;
  sample.estimated_pitch_deg = angleY;
  sample.i_roll_us = iTermRoll;
  sample.i_pitch_us = iTermPitch;
  sample.i_yaw_us = iTermYaw;
  const AllocationTelemetrySample allocation =
      readAllocationTelemetrySnapshot();
  sample.mixer_rp_scale = allocation.rp_scale;
  sample.mixer_yaw_scale = allocation.yaw_scale;
  sample.plant_allocator_scale = plant_allocator_scale;
  sample.target_yaw_rate_dps = tgtRate[2];
  sample.yaw_authority_state = allocation.yaw_authority_state;
  sample.pid_loop_hz = pidLoopHz;
  sample.yaw_hold = yaw_hold_now;
  sample.mixer_scaled_now = mixer_scaled;
  sample.safety_locked = safety_lock;
  sample.fault_rc_now = fault_rc;
  sample.fault_imu1_now = fault_imu1;
  sample.fault_imu2_now = fault_imu2;
  sample.fault_disagree_now = fault_disagree;
  sample.fault_attitude_now = fault_attitude;
  sample.failsafe_phase = fs_phase;
  sample.base_throttle_us = base_throttle;
  sample.failsafe_probe_state = fs_probe_state;
  sample.failsafe_probe_no_response = fs_probe_no_response;
  sample.failsafe_probe_response_g = fs_probe_response_g;
  sample.accel_magnitude_g =
      std::sqrt(static_cast<double>(accX) * accX +
                static_cast<double>(accY) * accY +
                static_cast<double>(accZ) * accZ);
  sample.vertical_acceleration_ms2 = plant_step.vertical_acceleration_ms2;
  sample.specific_force_g = plant_step.specific_force_g;
  sample.ground_force_n = plant_step.ground_force_n;
  for (int index = 0; index < 4; index++) sample.motors[index] = motorOut[index];
  result.samples.push_back(sample);

  const bool firmware_finite =
      std::isfinite(static_cast<double>(angleX)) &&
      std::isfinite(static_cast<double>(angleY)) &&
      std::isfinite(static_cast<double>(angleZ)) &&
      std::isfinite(static_cast<double>(gyroX)) &&
      std::isfinite(static_cast<double>(gyroY)) &&
      std::isfinite(static_cast<double>(gyroZ)) &&
      std::isfinite(static_cast<double>(iTermRoll)) &&
      std::isfinite(static_cast<double>(iTermPitch)) &&
      std::isfinite(static_cast<double>(iTermYaw));
  if (result.all_finite && (!finiteState(state) || !firmware_finite)) {
    result.all_finite = false;
    result.first_nonfinite_tick = tick;
  }

  for (int motor : sample.motors) {
    if (result.all_motors_in_range && (motor < 1000 || motor > 2000)) {
      result.all_motors_in_range = false;
      result.first_bad_motor_tick = tick;
    }
  }
  if (!result.safety_lock_ever && sample.safety_locked) {
    result.safety_lock_ever = true;
    result.first_safety_lock_tick = tick;
  }

  const double abs_roll_deg = std::fabs(state.phi * kRadToDeg);
  const double abs_pitch_deg = std::fabs(state.theta * kRadToDeg);
  if (abs_roll_deg > result.max_abs_roll_deg) {
    result.max_abs_roll_deg = abs_roll_deg;
    result.max_abs_roll_tick = tick;
  }
  if (abs_pitch_deg > result.max_abs_pitch_deg) {
    result.max_abs_pitch_deg = abs_pitch_deg;
    result.max_abs_pitch_tick = tick;
  }
  const double abs_i_roll_us = std::fabs(sample.i_roll_us);
  const double abs_i_pitch_us = std::fabs(sample.i_pitch_us);
  const double abs_i_yaw_us = std::fabs(sample.i_yaw_us);
  if (abs_i_roll_us > result.max_abs_i_roll_us) {
    result.max_abs_i_roll_us = abs_i_roll_us;
    result.max_abs_i_roll_tick = tick;
  }
  if (abs_i_pitch_us > result.max_abs_i_pitch_us) {
    result.max_abs_i_pitch_us = abs_i_pitch_us;
    result.max_abs_i_pitch_tick = tick;
  }
  if (abs_i_yaw_us > result.max_abs_i_yaw_us) {
    result.max_abs_i_yaw_us = abs_i_yaw_us;
  }
}

RunResult runSil(const RunConfig &config) {
  CHECK_MSG(config.ticks > 0, "tick limit 0 would make pid_task unbounded");
  struct AllocatorPolicyGuard {
    bool previous;
    explicit AllocatorPolicyGuard(bool use_uniform)
        : previous(silUseUniformAllocator) {
      silUseUniformAllocator = use_uniform;
      silLastDispatchedAllocation = {
          {1000, 1000, 1000, 1000}, 1000.0f, 1.0f, 1.0f, false};
    }
    ~AllocatorPolicyGuard() { silUseUniformAllocator = previous; }
  } allocator_policy(config.uniform_allocator_oracle);
  PlantState state = config.initial;
  resetFirmwareState(state);

  sendUdpCommandOnce("start");
  CHECK_MSG(safety_lock && safety_arm_requested,
            "start command did not publish an arm request");
  if (config.base_throttle_us == 1150) {
    sendUdpCommandOnce("th 1150");
  } else {
    sendUdpCommandOnce("th " + std::to_string(config.base_throttle_us));
  }
  CHECK_EQ(base_throttle, config.base_throttle_us);
  CHECK_EQ(min_throttle, std::max(1050, config.base_throttle_us - CTRL_MARGIN));
  CHECK_EQ(max_throttle, std::min(1900, config.base_throttle_us + CTRL_MARGIN));
  runPidTicksForHarness(1);
  CHECK_MSG(!safety_lock && !safety_arm_requested,
            "Core 1 did not apply the arm request");

  targetAngleX = config.target_roll_deg;
  targetAngleY = config.target_pitch_deg;
  targetAngleZ = config.target_yaw_deg;
  angleX = static_cast<float>(state.phi * kRadToDeg);
  angleY = static_cast<float>(state.theta * kRadToDeg);
  angleZ = static_cast<float>(state.psi * kRadToDeg);
  Ki_Rate_Roll = config.ki_roll;
  Ki_Rate_Pitch = config.ki_pitch;
  Ki_Rate_Yaw = config.ki_yaw;

  RunResult result;
  result.samples.reserve(static_cast<std::size_t>(config.ticks) + 1U);
  PlantStep plant_step;
  arduino_fake::pre_tick_hook = [&](uint32_t tick) {
    if (tick > 0) {
      plant_step = integratePlant(
          state, disturbanceAt(config, tick - 1U, state),
          config.inject_roll_sign_fault, config.vertical_enabled,
          config.plant_parameters);
    }
    if (config.state_for_tick) config.state_for_tick(tick, state);
    if (config.throttle_for_tick) {
      const int scheduled_throttle = config.throttle_for_tick(tick);
      base_throttle = scheduled_throttle;
      min_throttle = std::max(1050, scheduled_throttle - CTRL_MARGIN);
      max_throttle = std::min(1900, scheduled_throttle + CTRL_MARGIN);
    }
    arduino_fake::millis_value += 1U;
    arduino_fake::micros_value += 1000U;
    if (tick < config.rc_disconnect_tick ||
        tick >= config.rc_reconnect_tick) {
      lastRcMs = millis();
    }
    if (tick == config.resume_tick) sendUdpCommandOnce("resume");
    bool raw_saturated_now = false;
    if (config.accel_noise_enabled) {
      CHECK_MSG(config.accel_noise_sd_g >= 0.0,
                "accel noise sd must be non-negative");
      const double nominal_specific_force_g =
          config.vertical_enabled ? plant_step.specific_force_g : 1.0;
      const double common_mode_g =
          tick >= config.accel_common_mode_start_tick
              ? config.accel_common_mode_g
              : 0.0;
      raw_saturated_now = injectImuFromPlant(
          state, tick,
          nominal_specific_force_g +
              deterministicAccelNoiseG(tick, config.accel_noise_sd_g) +
              common_mode_g);
    } else {
      // 기본 OFF는 5e5f51e의 기존 분기와 연산 순서를 그대로 보존한다.
      raw_saturated_now =
          config.vertical_enabled
              ? injectImuFromPlant(state, tick, plant_step.specific_force_g)
              : injectImuFromPlant(state, tick);
    }
    if (raw_saturated_now && !result.raw_saturated) {
      result.first_raw_saturation_tick = tick;
    }
    result.raw_saturated = raw_saturated_now || result.raw_saturated;
    appendSample(result, tick, state, plant_step,
                 silLastDispatchedAllocation.rp_scale);
  };
  arduino_fake::tick_index = 0;
  arduino_fake::tick_limit = config.ticks;

  bool stopped = false;
  try {
    pid_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
    stopped = true;
  } catch (...) {
    arduino_fake::pre_tick_hook = nullptr;
    arduino_fake::tick_limit = 0;
    throw;
  }
  arduino_fake::pre_tick_hook = nullptr;
  arduino_fake::tick_limit = 0;

  CHECK_MSG(stopped, "pid_task did not stop at tick_limit");
  CHECK_EQ(result.samples.size(), static_cast<std::size_t>(config.ticks) + 1U);
  return result;
}

struct EstimatorTrackingResult {
  double estimate_at_ramp_end_deg = 0.0;
  double estimate_at_deadline_deg = 0.0;
  uint32_t first_90_percent_tick = std::numeric_limits<uint32_t>::max();
  bool raw_saturated = false;
};

EstimatorTrackingResult runPrescribedEstimatorRamp(bool roll) {
  constexpr uint32_t kRampTicks = 1000;
  constexpr uint32_t kDeadlineTick = kRampTicks + 150;
  constexpr double kCommandDeg = 30.0;
  constexpr double kRampRateDegPerSec =
      kCommandDeg / (kRampTicks * kDt);
  constexpr double kNinetyPercentDeg = 0.9 * kCommandDeg;

  PlantState initial;
  resetFirmwareState(initial);

  EstimatorTrackingResult result;
  arduino_fake::pre_tick_hook = [&](uint32_t tick) {
    const double estimate_deg = roll ? angleX : angleY;
    if (tick == kRampTicks) {
      result.estimate_at_ramp_end_deg = estimate_deg;
    }
    if (tick == kDeadlineTick) {
      result.estimate_at_deadline_deg = estimate_deg;
    }
    if (result.first_90_percent_tick ==
            std::numeric_limits<uint32_t>::max() &&
        estimate_deg >= kNinetyPercentDeg) {
      result.first_90_percent_tick = tick;
    }

    arduino_fake::millis_value += 1U;
    arduino_fake::micros_value += 1000U;
    lastRcMs = millis();

    const double ramp_fraction =
        std::min(1.0, static_cast<double>(tick) / kRampTicks);
    PlantState prescribed;
    if (roll) {
      prescribed.phi = ramp_fraction * kCommandDeg * kDegToRad;
      prescribed.p =
          tick < kRampTicks ? kRampRateDegPerSec * kDegToRad : 0.0;
    } else {
      prescribed.theta = ramp_fraction * kCommandDeg * kDegToRad;
      prescribed.q =
          tick < kRampTicks ? kRampRateDegPerSec * kDegToRad : 0.0;
    }
    result.raw_saturated =
        injectImuFromPlant(prescribed, tick) || result.raw_saturated;
  };
  arduino_fake::tick_limit = kDeadlineTick;

  bool stopped = false;
  try {
    pid_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
    stopped = true;
  } catch (...) {
    arduino_fake::pre_tick_hook = nullptr;
    arduino_fake::tick_limit = 0;
    throw;
  }
  arduino_fake::pre_tick_hook = nullptr;
  arduino_fake::tick_limit = 0;

  CHECK_MSG(stopped, "pid_task did not stop at estimator ramp deadline");
  return result;
}

double meanAbsTailDeg(const RunResult &result, bool roll, std::size_t count) {
  CHECK_MSG(!result.samples.empty(), "trajectory is empty");
  count = std::min(count, result.samples.size());
  const std::size_t begin = result.samples.size() - count;
  double sum = 0.0;
  for (std::size_t index = begin; index < result.samples.size(); index++) {
    const double angle = roll ? result.samples[index].plant.phi
                              : result.samples[index].plant.theta;
    sum += std::fabs(angle * kRadToDeg);
  }
  return sum / static_cast<double>(count);
}

double settlingTime90(const RunResult &result, bool roll,
                      double initial_abs_deg) {
  const double band_deg = 0.1 * initial_abs_deg;
  std::size_t last_outside = 0;
  bool was_outside = false;
  for (std::size_t index = 0; index < result.samples.size(); index++) {
    const double angle_deg = (roll ? result.samples[index].plant.phi
                                   : result.samples[index].plant.theta) * kRadToDeg;
    if (!std::isfinite(angle_deg)) {
      return std::numeric_limits<double>::infinity();
    }
    if (std::fabs(angle_deg) > band_deg) {
      was_outside = true;
      last_outside = index;
    }
  }
  if (!was_outside) return 0.0;
  if (last_outside + 1U >= result.samples.size()) {
    return std::numeric_limits<double>::infinity();
  }
  return result.samples[last_outside + 1U].time_s;
}

const Sample &sampleAtTick(const RunResult &result, uint32_t tick) {
  CHECK_MSG(tick < result.samples.size(), "requested sample is outside trajectory");
  return result.samples[tick];
}

std::string tickDetail(const char *label, uint32_t tick) {
  std::ostringstream detail;
  detail << label << " at tick " << tick;
  return detail.str();
}

std::string metricTickDetail(const char *label, double value,
                             uint32_t tick, double limit) {
  std::ostringstream detail;
  detail << label << "=" << value << " at tick " << tick
         << ", limit=" << limit;
  return detail.str();
}

std::string transitionDetail(const char *label, double from_value,
                             uint32_t from_tick, double to_value,
                             uint32_t to_tick) {
  std::ostringstream detail;
  detail << label << ": " << from_value << " at tick " << from_tick
         << " -> " << to_value << " at tick " << to_tick;
  return detail.str();
}

RunConfig constantRollDisturbance(float ki_roll, double torque_nm,
                                  uint32_t ticks) {
  RunConfig config;
  config.ticks = ticks;
  config.ki_roll = ki_roll;
  config.disturbance_for_interval = [torque_nm](uint32_t) {
    return Disturbance{torque_nm, 0.0, 0.0};
  };
  return config;
}

RunConfig constantYawDisturbance(float ki_yaw, double torque_nm,
                                 uint32_t ticks) {
  RunConfig config;
  config.ticks = ticks;
  config.ki_yaw = ki_yaw;
  config.disturbance_for_interval = [torque_nm](uint32_t) {
    return Disturbance{0.0, 0.0, torque_nm};
  };
  return config;
}

constexpr uint32_t kLowThrottleTick = 1000U;
constexpr uint32_t kLowThrottleDisturbanceTick = 1200U;
constexpr uint32_t kLowThrottleRunTicks = 2500U;
constexpr uint32_t kPostLimitedObservationTicks = 500U;
constexpr double kLowThrottleYawTorqueNm = 0.030;
constexpr double kLowThrottlePitchTorqueNm = 0.025;

RunConfig lowThrottleYawPitchConfig(bool uniform_allocator_oracle) {
  RunConfig config;
  config.ticks = kLowThrottleRunTicks;
  config.base_throttle_us = 1300;
  config.uniform_allocator_oracle = uniform_allocator_oracle;
  config.throttle_for_tick = [](uint32_t tick) {
    return tick < kLowThrottleTick ? 1300 : 1270;
  };
  config.disturbance_for_interval = [](uint32_t tick) {
    if (tick < kLowThrottleDisturbanceTick) return Disturbance{};
    return Disturbance{
        0.0, kLowThrottlePitchTorqueNm, kLowThrottleYawTorqueNm};
  };
  return config;
}

double maxAbsPitchFromTick(const RunResult &result, uint32_t first_tick) {
  double maximum = 0.0;
  for (const Sample &sample : result.samples) {
    if (sample.tick < first_tick) continue;
    maximum = std::max(
        maximum, std::fabs(sample.plant.theta * kRadToDeg));
  }
  return maximum;
}

double tailMeanAbsYawRateDps(const RunResult &result, std::size_t tail) {
  const std::size_t n = result.samples.size();
  const std::size_t start = n > tail ? n - tail : 0;
  double sum = 0.0;
  for (std::size_t index = start; index < n; index++) {
    sum += std::fabs(result.samples[index].plant.r) * kRadToDeg;
  }
  return sum / static_cast<double>(n - start);
}

const char *failsafePhaseName(uint8_t phase) {
  switch (phase) {
    case FS_NONE: return "FS_NONE";
    case FS_DESCENDING: return "FS_DESCENDING";
    case FS_CUT_LANDED: return "FS_CUT_LANDED";
    case FS_CUT_TIMEOUT: return "FS_CUT_TIMEOUT";
    case FS_CUT_ABORT: return "FS_CUT_ABORT";
    default: return "FS_UNKNOWN";
  }
}

struct FailsafeTrace {
  std::vector<std::pair<uint32_t, uint8_t>> transitions;
  uint32_t entry_tick = std::numeric_limits<uint32_t>::max();
  uint32_t terminal_tick = std::numeric_limits<uint32_t>::max();
  uint32_t contact_tick = std::numeric_limits<uint32_t>::max();
  uint32_t probe_evaluation_count = 0;
  uint8_t terminal_phase = FS_NONE;
  double entry_z_m = std::numeric_limits<double>::quiet_NaN();
  double terminal_z_m = std::numeric_limits<double>::quiet_NaN();
  double accel_min_g = std::numeric_limits<double>::infinity();
  double accel_max_g = -std::numeric_limits<double>::infinity();
  double max_probe_response_g = 0.0;
  uint8_t last_probe_no_response = 0;
};

FailsafeTrace analyzeFailsafeTrace(const RunResult &result) {
  FailsafeTrace trace;
  if (result.samples.empty()) return trace;

  uint8_t previous_phase = result.samples.front().failsafe_phase;
  trace.transitions.push_back({result.samples.front().tick, previous_phase});
  std::size_t entry_index = result.samples.size();
  std::size_t terminal_index = result.samples.size();
  for (std::size_t index = 1; index < result.samples.size(); index++) {
    const Sample &previous = result.samples[index - 1U];
    const Sample &sample = result.samples[index];
    if (sample.failsafe_phase != previous_phase) {
      trace.transitions.push_back({sample.tick, sample.failsafe_phase});
      previous_phase = sample.failsafe_phase;
    }
    if (entry_index == result.samples.size() &&
        sample.failsafe_phase == FS_DESCENDING) {
      entry_index = index;
      trace.entry_tick = sample.tick;
      trace.entry_z_m = sample.plant.z_m;
    }
    if (trace.contact_tick == std::numeric_limits<uint32_t>::max() &&
        previous.plant.z_m > 0.0 && sample.plant.z_m <= 0.0) {
      trace.contact_tick = sample.tick;
    }
    if (terminal_index == result.samples.size() &&
        sample.failsafe_phase != FS_NONE &&
        sample.failsafe_phase != FS_DESCENDING) {
      terminal_index = index;
      trace.terminal_tick = sample.tick;
      trace.terminal_phase = sample.failsafe_phase;
      trace.terminal_z_m = sample.plant.z_m;
    }
  }

  if (entry_index == result.samples.size()) return trace;
  const std::size_t last_index =
      terminal_index == result.samples.size()
          ? result.samples.size() - 1U
          : terminal_index;
  uint8_t previous_probe_state = FS_PROBE_WAIT;
  for (std::size_t index = entry_index; index <= last_index; index++) {
    const Sample &sample = result.samples[index];
    trace.accel_min_g = std::min(trace.accel_min_g, sample.accel_magnitude_g);
    trace.accel_max_g = std::max(trace.accel_max_g, sample.accel_magnitude_g);
    if (sample.failsafe_probe_state == FS_PROBE_EVALUATE &&
        previous_probe_state != FS_PROBE_EVALUATE) {
      trace.probe_evaluation_count++;
      trace.max_probe_response_g =
          std::max(trace.max_probe_response_g,
                   sample.failsafe_probe_response_g);
    }
    trace.last_probe_no_response = sample.failsafe_probe_no_response;
    previous_probe_state = sample.failsafe_probe_state;
  }
  return trace;
}

std::string tickSeconds(uint32_t tick) {
  if (tick == std::numeric_limits<uint32_t>::max()) return "none";
  std::ostringstream text;
  text << std::fixed << std::setprecision(3) << tick * kDt << "s";
  return text.str();
}

void printFailsafeTrace(const char *label, const RunResult &result) {
  const FailsafeTrace trace = analyzeFailsafeTrace(result);
  std::cout << "[SIL] " << label << " transitions=";
  for (std::size_t index = 0; index < trace.transitions.size(); index++) {
    if (index > 0) std::cout << " -> ";
    std::cout << failsafePhaseName(trace.transitions[index].second)
              << "@" << tickSeconds(trace.transitions[index].first);
  }
  std::cout << '\n';

  const uint8_t final_phase =
      result.samples.empty() ? FS_NONE : result.samples.back().failsafe_phase;
  const double elapsed_s =
      trace.entry_tick == std::numeric_limits<uint32_t>::max() ||
              trace.terminal_tick == std::numeric_limits<uint32_t>::max()
          ? std::numeric_limits<double>::quiet_NaN()
          : (trace.terminal_tick - trace.entry_tick) * kDt;
  std::cout << "[SIL] " << label
            << " final_phase=" << failsafePhaseName(final_phase)
            << "(" << static_cast<int>(final_phase) << ")"
            << " elapsed=" << elapsed_s << "s"
            << " entry_z=" << trace.entry_z_m << "m"
            << " terminal_z=" << trace.terminal_z_m << "m"
            << " delta_z=" << trace.terminal_z_m - trace.entry_z_m << "m"
            << " contact=" << tickSeconds(trace.contact_tick)
            << " accel_min=" << trace.accel_min_g << "g"
            << " accel_max=" << trace.accel_max_g << "g\n";
  std::cout << "[SIL] " << label
            << " probe_evaluations=" << trace.probe_evaluation_count
            << " max_probe_response=" << trace.max_probe_response_g << "g"
            << " no_response=" << static_cast<int>(trace.last_probe_no_response)
            << " landed_before_contact="
            << (trace.terminal_phase == FS_CUT_LANDED &&
                trace.terminal_tick < trace.contact_tick)
            << '\n';
}

std::vector<double> groundProbeResponses(const RunResult &result,
                                         uint32_t contact_tick) {
  std::vector<double> responses;
  uint8_t previous_probe_state = FS_PROBE_WAIT;
  for (const Sample &sample : result.samples) {
    if (sample.tick >= contact_tick && sample.plant.z_m <= 0.005 &&
        std::fabs(sample.plant.vz_ms) < 0.10 &&
        sample.failsafe_probe_state == FS_PROBE_EVALUATE &&
        previous_probe_state != FS_PROBE_EVALUATE) {
      responses.push_back(sample.failsafe_probe_response_g);
    }
    previous_probe_state = sample.failsafe_probe_state;
  }
  return responses;
}

// 중단·resume 없는 현재 하강 시험은 백스톱으로 끝난다. 이 헬퍼는 선택된 SIL
// 모델·초기 조건에서 접촉 뒤에 종료하는지 검사한다. 010251 공중 컷 해석은
// 철회됐으며, 이 검사 통과도 다른 고도·조건의 타임아웃 공중 컷을 배제하지 않는다.
void checkTerminatesNotBeforeContact(const char *label,
                                     const FailsafeTrace &trace) {
  std::ostringstream detail;
  detail << label << " terminal=" << failsafePhaseName(trace.terminal_phase)
         << "@" << tickSeconds(trace.terminal_tick)
         << " contact=" << tickSeconds(trace.contact_tick);
  CHECK_MSG(trace.contact_tick != std::numeric_limits<uint32_t>::max()
                && trace.terminal_tick >= trace.contact_tick
                && (trace.terminal_phase == FS_CUT_LANDED
                    || trace.terminal_phase == FS_CUT_TIMEOUT),
            detail.str());
}

void printProbeEvaluations(const char *label, const RunResult &result) {
  uint8_t previous_probe_state = FS_PROBE_WAIT;
  for (const Sample &sample : result.samples) {
    if (sample.failsafe_probe_state == FS_PROBE_EVALUATE &&
        previous_probe_state != FS_PROBE_EVALUATE) {
      std::cout << "[SIL] " << label
                << " probe@" << tickSeconds(sample.tick)
                << " z=" << sample.plant.z_m
                << "m vz=" << sample.plant.vz_ms
                << "m/s response=" << sample.failsafe_probe_response_g
                << "g no_response="
                << static_cast<int>(sample.failsafe_probe_no_response)
                << '\n';
    }
    previous_probe_state = sample.failsafe_probe_state;
  }
}

constexpr uint32_t kHoverWarmupTicks = 3000;
constexpr uint32_t kV2ClimbTicks = 200;
constexpr uint32_t kV2DisconnectTick =
    kHoverWarmupTicks + kV2ClimbTicks;

void configureNormalLandingSurface(RunConfig &config) {
  // V1~V4의 일반 착지 표면은 임계감쇠로 둔다. k=2000N/m, m=0.3466kg에서
  // c_crit=2*sqrt(k*m)=약 52.65N*s/m다. V5 바운스는 별도 임펄스로 만든다.
  config.plant_parameters.c_ground =
      2.0 * std::sqrt(config.plant_parameters.k_ground *
                      config.plant_parameters.mass_kg);
}

RunConfig makeV3Config(double k_ge) {
  RunConfig config;
  config.initial.z_m = 0.5;
  config.ticks =
      kHoverWarmupTicks + RC_TIMEOUT_MS + FS_MAX_MS + 1000U;
  config.base_throttle_us = 1340;
  config.vertical_enabled = true;
  config.rc_disconnect_tick = kHoverWarmupTicks;
  config.plant_parameters.k_ge = k_ge;
  configureNormalLandingSurface(config);
  return config;
}

RunConfig makeV1Config(bool noise_enabled = false,
                       double noise_sd_g = 0.04) {
  RunConfig config;
  config.initial.z_m = 1.0;
  config.ticks =
      kHoverWarmupTicks + RC_TIMEOUT_MS + FS_MAX_MS + 1000U;
  config.base_throttle_us = 1340;
  config.vertical_enabled = true;
  config.rc_disconnect_tick = kHoverWarmupTicks;
  config.accel_noise_enabled = noise_enabled;
  config.accel_noise_sd_g = noise_sd_g;
  configureNormalLandingSurface(config);
  return config;
}

RunConfig makeV2Config() {
  RunConfig config;
  config.initial.z_m = 1.0;
  config.ticks =
      kV2DisconnectTick + RC_TIMEOUT_MS + FS_MAX_MS + 1000U;
  config.base_throttle_us = 1340;
  config.vertical_enabled = true;
  config.rc_disconnect_tick = kV2DisconnectTick;
  config.plant_parameters.k_ge = 0.0;
  configureNormalLandingSurface(config);
  config.throttle_for_tick = [](uint32_t tick) {
    return tick < kHoverWarmupTicks ? 1340 : 1450;
  };
  return config;
}

RunConfig makeV2bConfig() {
  RunConfig config;
  config.initial.z_m = 0.0;
  config.ticks = RC_TIMEOUT_MS + 1000U;
  config.base_throttle_us = 1450;
  config.vertical_enabled = true;
  config.rc_disconnect_tick = 0;
  config.plant_parameters.k_ge = 0.0;
  return config;
}

RunConfig makeV4Config(bool noise_enabled = false,
                       double noise_sd_g = 0.04) {
  RunConfig config;
  config.initial.z_m = 2.0;
  config.ticks =
      kHoverWarmupTicks + RC_TIMEOUT_MS + FS_MAX_MS + 1000U;
  config.base_throttle_us = 1340;
  config.vertical_enabled = true;
  config.rc_disconnect_tick = kHoverWarmupTicks;
  config.plant_parameters.k_ge = 0.0;
  config.accel_noise_enabled = noise_enabled;
  config.accel_noise_sd_g = noise_sd_g;
  configureNormalLandingSurface(config);
  config.throttle_for_tick = [](uint32_t tick) {
    return tick < kHoverWarmupTicks ? 1340 : 1400;
  };
  config.state_for_tick = [](uint32_t tick, PlantState &state) {
    if (tick == kHoverWarmupTicks) state.vz_ms = -1.2;
  };
  const double weight_n =
      config.plant_parameters.mass_kg * kGravityMs2;
  const double descent_balance_n =
      weight_n * FS_DESCENT_DELTA_US / (1340.0 - 1000.0);
  const uint32_t failsafe_entry_tick =
      kHoverWarmupTicks + RC_TIMEOUT_MS;
  // 1.2m/s 하강에서 추력 부족분을 상쇄하는 선형 항력이다. v_z=0인 지면에서는
  // 0이 되어 ground reaction이 실제 무게 부족분을 지지한다.
  config.vertical_drag_n_per_ms = descent_balance_n / 1.2;
  config.vertical_drag_start_tick = failsafe_entry_tick;
  config.disturbance_for_interval =
      [weight_n, failsafe_entry_tick](uint32_t tick) {
    if (tick >= failsafe_entry_tick &&
        tick < failsafe_entry_tick + 150U) {
      return Disturbance{0.0, 0.0, 0.0, -0.20 * weight_n};
    }
    if (tick >= failsafe_entry_tick + 150U &&
        tick < failsafe_entry_tick + 200U) {
      return Disturbance{0.0, 0.0, 0.0, 0.40 * weight_n};
    }
    return Disturbance{};
  };
  return config;
}

RunConfig makeV5Config() {
  RunConfig config = makeV1Config();
  config.initial.z_m = 0.5;
  config.ticks =
      kHoverWarmupTicks + RC_TIMEOUT_MS + FS_MAX_MS + 1500U;
  config.plant_parameters.k_ge = 0.0;
  // 첫 지상 무반응 프로브 직후 1.0m/s 위쪽 임펄스를 준다. confirm=2이면
  // 다음 공중 프로브가 카운트를 지우지만 confirm=1 변조는 이미 컷한 뒤다.
  config.state_for_tick = [bounced = false](uint32_t, PlantState &state) mutable {
    if (!bounced && fs_probe_no_response == 1 && state.z_m <= 0.0) {
      state.z_m = 0.0;
      state.vz_ms = 1.0;
      bounced = true;
    }
  };
  return config;
}

constexpr uint32_t kV6FailsafeEntryTick =
    kHoverWarmupTicks + RC_TIMEOUT_MS + 1U;
constexpr uint32_t kV6PostResumeTicks = 400U;

RunConfig makeV6Config(uint32_t resume_delay_ticks) {
  RunConfig config;
  config.initial.z_m = 20.0;
  config.base_throttle_us = 1340;
  config.vertical_enabled = true;
  config.plant_parameters.k_ge = 0.0;
  config.rc_disconnect_tick = kHoverWarmupTicks;
  config.rc_reconnect_tick =
      kV6FailsafeEntryTick + resume_delay_ticks;
  config.resume_tick = config.rc_reconnect_tick;
  config.ticks = config.resume_tick + kV6PostResumeTicks + 2U;
  return config;
}

struct ResumeMeasurement {
  uint32_t delay_ticks = 0;
  uint32_t transition_tick = 0;
  double residual_vz_ms = 0.0;
  double post_height_loss_m = 0.0;
  double mean_post_acceleration_ms2 = 0.0;
};

ResumeMeasurement measureV6(const RunResult &result,
                            uint32_t resume_delay_ticks) {
  std::size_t transition_index = result.samples.size();
  for (std::size_t index = 1; index < result.samples.size(); index++) {
    if (result.samples[index - 1U].failsafe_phase == FS_DESCENDING &&
        result.samples[index].failsafe_phase == FS_NONE) {
      transition_index = index;
      break;
    }
  }
  CHECK_MSG(transition_index < result.samples.size(),
            "V6 did not transition FS_DESCENDING -> FS_NONE");
  const Sample &transition = result.samples[transition_index];
  const uint32_t post_tick =
      transition.tick + kV6PostResumeTicks;
  const Sample &post = sampleAtTick(result, post_tick);

  double acceleration_sum = 0.0;
  uint32_t acceleration_count = 0;
  for (uint32_t tick = transition.tick + 1U;
       tick <= transition.tick + 100U; tick++) {
    acceleration_sum += sampleAtTick(result, tick).vertical_acceleration_ms2;
    acceleration_count++;
  }

  ResumeMeasurement measurement;
  measurement.delay_ticks = resume_delay_ticks;
  measurement.transition_tick = transition.tick;
  measurement.residual_vz_ms = transition.plant.vz_ms;
  measurement.post_height_loss_m =
      transition.plant.z_m - post.plant.z_m;
  measurement.mean_post_acceleration_ms2 =
      acceleration_sum / static_cast<double>(acceleration_count);

  CHECK_EQ(transition.base_throttle_us, 1340);
  CHECK(!transition.safety_locked);
  CHECK_MSG(std::fabs(measurement.mean_post_acceleration_ms2) < 0.02,
            "V6 hover restore did not suppress descent acceleration");
  CHECK_MSG(measurement.residual_vz_ms < 0.0,
            "V6 unexpectedly removed the residual descent velocity");
  CHECK_MSG(measurement.post_height_loss_m > 0.0,
            "V6 unexpectedly recovered or held altitude");
  return measurement;
}

}  // namespace

int main() {
  static_assert(sizeof(long) == 4, "SIL은 ESP32와 같은 32비트 long이 필요하다");
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "[SIL] SIL_INJECT_SIGN_FAULT="
            << (kInjectRollSignFault ? "ON" : "OFF") << '\n';

  runCase("shim: tick hook order, limit, and reset", [] {
    arduino_fake::reset();
    std::vector<uint32_t> observed;
    arduino_fake::pre_tick_hook = [&](uint32_t tick) { observed.push_back(tick); };
    arduino_fake::tick_limit = 2;

    TickType_t wake = 0;
    vTaskDelayUntil(&wake, 1);
    vTaskDelayUntil(&wake, 1);
    bool stopped = false;
    try {
      vTaskDelayUntil(&wake, 1);
    } catch (const arduino_fake::TaskDelayExit &) {
      stopped = true;
    }

    CHECK(stopped);
    CHECK_EQ(observed.size(), static_cast<std::size_t>(3));
    CHECK_EQ(observed[0], 0U);
    CHECK_EQ(observed[1], 1U);
    CHECK_EQ(observed[2], 2U);
    CHECK_EQ(arduino_fake::tick_index, 3U);

    arduino_fake::reset();
    CHECK(!arduino_fake::pre_tick_hook);
    CHECK_EQ(arduino_fake::tick_index, 0U);
    CHECK_EQ(arduino_fake::tick_limit, 0U);
  });

  runCase("helper: non-finite trajectory cannot settle", [] {
    RunResult result;
    Sample initial;
    initial.plant.phi = 8.0 * kDegToRad;
    result.samples.push_back(initial);
    Sample invalid;
    invalid.tick = 1;
    invalid.time_s = kDt;
    invalid.plant.phi = std::numeric_limits<double>::quiet_NaN();
    result.samples.push_back(invalid);
    CHECK(!std::isfinite(settlingTime90(result, true, 8.0)));
  });

  runCase("calibration: per-IMU scale corrects an injected 1.007g", [] {
    PlantState state;
    resetFirmwareState(state);

    constexpr double kInjectedAccelG = 1.007;
    bool saturated = false;
    inv_imu_sensor_event_t imu1 = {};
    imu1.accel[2] =
        roundedRaw(kInjectedAccelG / ACCEL_SCALE, saturated);
    inv_imu_sensor_event_t imu2 = {};
    imu2.accel[2] = static_cast<int16_t>(-imu1.accel[2]);
    IMU1.next_event = imu1;
    IMU2.next_event = imu2;

    CHECK(!saturated);
    CHECK(calibrate_bias());
    CHECK_LE(std::fabs(accel_scale1 - 1.0 / kInjectedAccelG), 0.001);
    CHECK_LE(std::fabs(accel_scale2 - 1.0 / kInjectedAccelG), 0.001);

    arduino_fake::tick_limit = 1;
    bool stopped = false;
    try {
      pid_task(nullptr);
    } catch (const arduino_fake::TaskDelayExit &) {
      stopped = true;
    }
    arduino_fake::tick_limit = 0;
    CHECK_MSG(stopped, "pid_task did not stop after calibrated accel sample");

    const double corrected_accel_g =
        std::sqrt(static_cast<double>(accX) * accX +
                  static_cast<double>(accY) * accY +
                  static_cast<double>(accZ) * accZ);
    std::ostringstream detail;
    detail << "actual=" << corrected_accel_g
           << "g, expected=1.0000+/-0.0020g";
    CHECK_MSG(std::fabs(corrected_accel_g - 1.0) <= 0.002,
              detail.str());
    std::cout << "[SIL] accel calibration injected=" << kInjectedAccelG
              << "g scale1=" << accel_scale1
              << " scale2=" << accel_scale2
              << " corrected=" << corrected_accel_g << "g\n";
  });

  runCase("calibration: implausible accel magnitude keeps unity scale", [] {
    PlantState state;
    resetFirmwareState(state);

    bool saturated = false;
    inv_imu_sensor_event_t imu1 = {};
    imu1.accel[2] = roundedRaw(1.30 / ACCEL_SCALE, saturated);
    inv_imu_sensor_event_t imu2 = {};
    imu2.accel[2] = static_cast<int16_t>(-imu1.accel[2]);
    IMU1.next_event = imu1;
    IMU2.next_event = imu2;

    CHECK(!saturated);
    CHECK(calibrate_bias());
    CHECK_LE(std::fabs(accel_scale1 - 1.0f), 1e-6);
    CHECK_LE(std::fabs(accel_scale2 - 1.0f), 1e-6);
    CHECK_MSG(
        arduino_fake::serial_output.find("outside [0.8,1.2], scale=1.0") !=
            std::string::npos,
        "implausible accel calibration did not emit the safety warning");
  });

  runCase("telemetry: target angles, mag, 3901-L0X and magcal append after field 55", [] {
    PlantState state;
    resetFirmwareState(state);
    connectionEstablished = true;
    hover_est = 1337.5f;
    hover_valid = true;
    fs_probe_state = FS_PROBE_UNAVAILABLE;
    fs_probe_no_response = 2;
    fs_probe_response_g = 0.0125f;
    imuSampleSnapshot = {
        1.25f, -2.5f, 3.75f,
        0.125f, -0.25f, 0.5f,
        -4.5f, 5.25f, -6.0f,
        -0.625f, 0.75f, -0.875f};
    targetAngleX = 4.25f;
    targetAngleY = -3.5f;
    targetAngleZ = 172.75f;
    mag_enabled = true;
    mag_calibrating = false;

    sendTelemetry();

    const std::string &packet = wifi_udp_fake::telemetry_output;
    const std::size_t field_count =
        packet.empty()
            ? 0U
            : 1U + static_cast<std::size_t>(
                       std::count(packet.begin(), packet.end(), ','));
    std::ostringstream detail;
    // 3901-L0X 가 없는 SIL 에서는 Range/Flow 가 기본값이고 quality 는 -1 —
    // "신선한 프레임 없음"이 그대로 드러나야 한다.
    const std::string expected_suffix =
        ",1337.50,1,3,2,0.013"
        ",1.25,-2.50,3.75,0.125,-0.250,0.500"
        ",-4.50,5.25,-6.00,-0.625,0.750,-0.875"
        ",4.25,-3.50,172.75,1"
        ",0,-1,0,0,-1,0"
        ",1.000,1.000,1000.00,0.00,0.00,0.00"
        ",0.00,0.00,0.00,0";
    detail << "actual field_count=" << field_count
           << ", packet_suffix="
           << packet.substr(
                  packet.size() > expected_suffix.size()
                      ? packet.size() - expected_suffix.size()
                      : 0U)
           << "; expected field_count=75 and suffix " << expected_suffix;
    CHECK_MSG(field_count == 75U, detail.str());
    CHECK_MSG(
        packet.size() >= expected_suffix.size() &&
            packet.compare(
                packet.size() - expected_suffix.size(),
                expected_suffix.size(),
                expected_suffix) == 0,
        detail.str());
  });

  runCase("vertical: 1g specific force preserves legacy IMU raw bytes", [] {
    PlantState state;
    state.phi = 8.0 * kDegToRad;
    state.theta = -6.0 * kDegToRad;
    state.p = 12.0 * kDegToRad;
    state.q = -9.0 * kDegToRad;
    state.r = 4.0 * kDegToRad;

    resetFirmwareState(state);
    (void)injectImuFromPlant(state, 42);
    const inv_imu_sensor_event_t legacy_imu1 = IMU1.next_event;
    const inv_imu_sensor_event_t legacy_imu2 = IMU2.next_event;

    (void)injectImuFromPlant(state, 42, 1.0);
    for (int axis = 0; axis < 3; axis++) {
      CHECK_EQ(IMU1.next_event.gyro[axis], legacy_imu1.gyro[axis]);
      CHECK_EQ(IMU1.next_event.accel[axis], legacy_imu1.accel[axis]);
      CHECK_EQ(IMU2.next_event.gyro[axis], legacy_imu2.gyro[axis]);
      CHECK_EQ(IMU2.next_event.accel[axis], legacy_imu2.accel[axis]);
    }
  });

  runCase("vertical: 1340us hovers when ground effect is disabled", [] {
    RunConfig config;
    config.initial.z_m = 1.0;
    config.ticks = 1000;
    config.base_throttle_us = 1340;
    config.vertical_enabled = true;
    config.plant_parameters.k_ge = 0.0;
    const RunResult result = runSil(config);

    CHECK_MSG(result.all_finite,
              tickDetail("non-finite vertical hover state",
                         result.first_nonfinite_tick));
    CHECK_LE(std::fabs(result.samples.back().plant.z_m - 1.0), 0.002);
    CHECK_LE(std::fabs(result.samples.back().plant.vz_ms), 0.005);
  });

  runCase("vertical: 0.4mps contact stays shallow and produces a pulse", [] {
    RunConfig config;
    config.initial.z_m = 0.0;
    config.initial.vz_ms = -0.4;
    config.ticks = 500;
    config.base_throttle_us = 1280;
    config.vertical_enabled = true;
    config.plant_parameters.k_ge = 0.0;
    const RunResult result = runSil(config);

    double min_z_m = 0.0;
    double peak_upward_accel_g = 0.0;
    uint32_t pulse_samples = 0;
    for (const Sample &sample : result.samples) {
      min_z_m = std::min(min_z_m, sample.plant.z_m);
      const double upward_accel_g =
          sample.vertical_acceleration_ms2 / kGravityMs2;
      peak_upward_accel_g = std::max(peak_upward_accel_g, upward_accel_g);
      if (upward_accel_g >= 0.25) pulse_samples++;
    }

    std::cout << "[SIL] vertical contact min_z=" << min_z_m
              << "m peak_upward_accel=" << peak_upward_accel_g
              << "g duration>=0.25g=" << pulse_samples << "ms\n";
    // 하드 플로어(k=2000N/m)의 서명: 얕고 짧고 날카롭다.
    CHECK_GE(min_z_m, -0.005);
    CHECK_LE(min_z_m, -0.001);
    CHECK_GE(peak_upward_accel_g, 4.0);
    CHECK_GE(pulse_samples, 5U);
    CHECK_LE(pulse_samples, 60U);
  });

  // R6: 무른 표면은 사라진 게 아니라 여기서 명시적으로 검증한다. 2026-08-01
  // 이전에는 이것이 **유일한** 지면 모델이었고(k=80N/m, 정적 눌림 4.25cm),
  // 그래서 SIL이 실기와 어긋났다. 이제 하드 플로어가 기본이고 이쪽이 옵션이다.
  runCase("R6: 무른 표면은 깊고 느리게 눌린다 (하드 플로어와 서명이 다르다)", [] {
    RunConfig config;
    config.initial.z_m = 0.0;
    config.initial.vz_ms = -0.4;
    config.ticks = 500;
    config.base_throttle_us = 1280;
    config.vertical_enabled = true;
    config.plant_parameters.k_ground = 80.0;
    configureNormalLandingSurface(config);
    const RunResult result = runSil(config);

    double min_z_m = 0.0;
    double peak_upward_accel_g = 0.0;
    unsigned pulse_samples = 0;
    for (const Sample &sample : result.samples) {
      min_z_m = std::min(min_z_m, sample.plant.z_m);
      const double upward_accel_g =
          sample.vertical_acceleration_ms2 / kGravityMs2;
      peak_upward_accel_g = std::max(peak_upward_accel_g, upward_accel_g);
      if (upward_accel_g >= 0.25) pulse_samples++;
    }
    std::cout << "[SIL] R6 soft contact min_z=" << min_z_m
              << "m peak_upward_accel=" << peak_upward_accel_g
              << "g duration>=0.25g=" << pulse_samples << "ms\n";
    // 하드 플로어(-0.002m, 20ms)보다 훨씬 깊고 길다. 절대값이 아니라 그
    // **차이**가 요지다 — 같은 접촉이 표면에 따라 5배 깊게 눌린다.
    CHECK_LE(min_z_m, -0.008);
    CHECK_GE(pulse_samples, 40U);
  });

  runCase("vertical: RC disconnect enters the real firmware failsafe", [] {
    RunConfig config = makeV1Config();
    config.ticks = kHoverWarmupTicks + RC_TIMEOUT_MS + 100U;
    const RunResult result = runSil(config);

    CHECK_EQ(result.samples.back().failsafe_phase,
             static_cast<uint8_t>(FS_DESCENDING));
    CHECK_MSG(std::isfinite(result.samples.back().accel_magnitude_g),
              "recorded firmware accel magnitude is non-finite");
  });

  runCase("vertical: ground effect and vertical force scale specific force", [] {
    RunConfig baseline_config;
    baseline_config.initial.z_m = 1.0;
    baseline_config.ticks = 2;
    baseline_config.base_throttle_us = 1340;
    baseline_config.vertical_enabled = true;
    baseline_config.plant_parameters.k_ge = 0.0;
    const RunResult baseline = runSil(baseline_config);

    RunConfig ground_effect_config = baseline_config;
    ground_effect_config.initial.z_m = 0.0;
    ground_effect_config.plant_parameters.k_ge = 0.15;
    const RunResult ground_effect = runSil(ground_effect_config);

    RunConfig force_config = baseline_config;
    const double weight_n =
        force_config.plant_parameters.mass_kg * kGravityMs2;
    force_config.disturbance_for_interval = [weight_n](uint32_t) {
      return Disturbance{0.0, 0.0, 0.0, 0.40 * weight_n};
    };
    const RunResult force = runSil(force_config);

    CHECK_LE(std::fabs(baseline.samples[1].specific_force_g - 1.0), 1e-9);
    CHECK_LE(std::fabs(ground_effect.samples[1].specific_force_g - 1.15),
             1e-9);
    CHECK_LE(std::fabs(force.samples[1].specific_force_g - 1.40), 1e-9);
  });

  runCase("vibration: configured sd survives the firmware 5Hz LPF", [] {
    constexpr uint32_t kWarmupTicks = 2000;
    constexpr uint32_t kMeasureTicks = 10000;
    constexpr std::array<uint32_t, 6> kRepeatTicks = {
        0, 1, 137, 2048, 7123, kWarmupTicks + kMeasureTicks};
    std::array<double, kRepeatTicks.size()> first_sequence = {};
    std::array<double, kRepeatTicks.size()> second_sequence = {};
    for (std::size_t index = 0; index < kRepeatTicks.size(); index++) {
      first_sequence[index] =
          deterministicAccelNoiseG(kRepeatTicks[index], 0.04);
    }
    for (std::size_t index = 0; index < kRepeatTicks.size(); index++) {
      second_sequence[index] =
          deterministicAccelNoiseG(kRepeatTicks[index], 0.04);
    }
    CHECK(first_sequence == second_sequence);

    RunConfig config;
    config.initial.z_m = 10.0;
    config.ticks = kWarmupTicks + kMeasureTicks;
    config.base_throttle_us = 1340;
    config.vertical_enabled = true;
    config.plant_parameters.k_ge = 0.0;
    config.accel_noise_enabled = true;
    config.accel_noise_sd_g = 0.04;
    const RunResult result = runSil(config);

    double filtered_g = result.samples.front().accel_magnitude_g;
    double raw_sum_g = 0.0;
    double raw_sum_sq_g = 0.0;
    double filtered_sum_g = 0.0;
    double filtered_sum_sq_g = 0.0;
    uint32_t sample_count = 0;
    for (const Sample &sample : result.samples) {
      filtered_g += FS_LAND_LPF_ALPHA *
                    (sample.accel_magnitude_g - filtered_g);
      if (sample.tick <= kWarmupTicks) continue;
      raw_sum_g += sample.accel_magnitude_g;
      raw_sum_sq_g +=
          sample.accel_magnitude_g * sample.accel_magnitude_g;
      filtered_sum_g += filtered_g;
      filtered_sum_sq_g += filtered_g * filtered_g;
      sample_count++;
    }

    const double raw_mean_g = raw_sum_g / sample_count;
    const double filtered_mean_g = filtered_sum_g / sample_count;
    const double raw_sd_g = std::sqrt(
        raw_sum_sq_g / sample_count - raw_mean_g * raw_mean_g);
    const double filtered_sd_g = std::sqrt(
        filtered_sum_sq_g / sample_count -
        filtered_mean_g * filtered_mean_g);
    const double attenuation =
        1.0 - filtered_sd_g / raw_sd_g;
    std::cout << "[SIL] vibration sd_config=" << config.accel_noise_sd_g
              << "g raw_sd=" << raw_sd_g
              << "g lpf5_sd=" << filtered_sd_g
              << "g attenuation=" << 100.0 * attenuation << "%\n";

    CHECK_LE(std::fabs(raw_sd_g - config.accel_noise_sd_g), 0.002);
    CHECK_GE(attenuation, 0.0);
    CHECK_LE(attenuation, 0.10);
  });

  constexpr std::array<double, 3> kV3GroundEffects = {0.15, 0.25, 0.35};
  std::array<RunResult, kV3GroundEffects.size()> v3_results;
  runCase("V3: hover entry descends through the ground-effect region", [&] {
    for (std::size_t index = 0; index < kV3GroundEffects.size(); index++) {
      v3_results[index] = runSil(makeV3Config(kV3GroundEffects[index]));
      const FailsafeTrace trace = analyzeFailsafeTrace(v3_results[index]);
      CHECK_MSG(std::isfinite(trace.entry_z_m), "V3 did not enter failsafe");
      CHECK_MSG(std::isfinite(trace.terminal_z_m), "V3 did not terminate");
      CHECK_MSG(trace.contact_tick != std::numeric_limits<uint32_t>::max(),
                "V3 did not pass through the ground-effect region to contact");
      CHECK_MSG(trace.terminal_z_m - trace.entry_z_m < 0.0,
                "V3 terminal delta_z is not descending");
      CHECK_MSG(
          trace.terminal_phase != FS_CUT_LANDED ||
              trace.terminal_tick >= trace.contact_tick,
          "V3 declared landing before first contact");
    }
  });

  runCase("gyro transform: prescribed +30deg roll tracks by 150ms", [] {
    constexpr uint32_t kDeadlineTick = 1150;
    const EstimatorTrackingResult result =
        runPrescribedEstimatorRamp(true);
    std::cout << "[SIL] gyro-axis roll estimate@1.000s="
              << result.estimate_at_ramp_end_deg
              << "deg estimate@1.150s=" << result.estimate_at_deadline_deg
              << "deg first>=27deg_tick=" << result.first_90_percent_tick
              << '\n';

    CHECK_MSG(!result.raw_saturated,
              "roll prescribed-motion synthetic IMU raw saturated");
    CHECK_MSG(result.estimate_at_deadline_deg > 0.0,
              "roll estimate has the wrong sign");
    CHECK_MSG(result.estimate_at_deadline_deg >= 27.0,
              "roll estimate did not reach 90% of +30deg");
    CHECK_MSG(result.first_90_percent_tick <= kDeadlineTick,
              "roll estimate did not reach 90% by 150ms after the ramp");
  });

  runCase("gyro transform: prescribed +30deg pitch tracks by 150ms", [] {
    constexpr uint32_t kDeadlineTick = 1150;
    const EstimatorTrackingResult result =
        runPrescribedEstimatorRamp(false);
    std::cout << "[SIL] gyro-axis pitch estimate@1.000s="
              << result.estimate_at_ramp_end_deg
              << "deg estimate@1.150s=" << result.estimate_at_deadline_deg
              << "deg first>=27deg_tick=" << result.first_90_percent_tick
              << '\n';

    CHECK_MSG(!result.raw_saturated,
              "pitch prescribed-motion synthetic IMU raw saturated");
    CHECK_MSG(result.estimate_at_deadline_deg > 0.0,
              "pitch estimate has the wrong sign");
    CHECK_MSG(result.estimate_at_deadline_deg >= 27.0,
              "pitch estimate did not reach 90% of +30deg");
    CHECK_MSG(result.first_90_percent_tick <= kDeadlineTick,
              "pitch estimate did not reach 90% by 150ms after the ramp");
  });

  runCase("S1: roll/pitch attitude hold converges", [] {
    RunConfig config;
    config.initial.phi = 8.0 * kDegToRad;
    config.initial.theta = -6.0 * kDegToRad;
    config.ticks = 3000;
    config.inject_roll_sign_fault = kInjectRollSignFault;
    const RunResult result = runSil(config);
    const double roll_tail = meanAbsTailDeg(result, true, 500);
    const double pitch_tail = meanAbsTailDeg(result, false, 500);
    std::cout << "[SIL] S1 max|phi|=" << result.max_abs_roll_deg
              << "deg@" << result.max_abs_roll_tick
              << " max|theta|=" << result.max_abs_pitch_deg
              << "deg@" << result.max_abs_pitch_tick
              << " tail500_mean|phi|=" << roll_tail
              << "deg tail500_mean|theta|=" << pitch_tail << "deg\n";

    CHECK_MSG(result.max_abs_roll_deg < 12.0,
              metricTickDetail("max|phi| deg", result.max_abs_roll_deg,
                               result.max_abs_roll_tick, 12.0));
    CHECK_MSG(result.max_abs_pitch_deg < 12.0,
              metricTickDetail("max|theta| deg", result.max_abs_pitch_deg,
                               result.max_abs_pitch_tick, 12.0));
    CHECK_MSG(roll_tail < 1.5,
              metricTickDetail("tail500 mean|phi| deg", roll_tail,
                               result.samples.back().tick, 1.5));
    CHECK_MSG(pitch_tail < 1.5,
              metricTickDetail("tail500 mean|theta| deg", pitch_tail,
                               result.samples.back().tick, 1.5));
    CHECK_MSG(result.all_motors_in_range,
              tickDetail("motor output left [1000,2000]",
                         result.first_bad_motor_tick));
    CHECK_MSG(result.all_finite,
              tickDetail("non-finite SIL state", result.first_nonfinite_tick));
    CHECK_MSG(!result.raw_saturated,
              tickDetail("synthetic IMU raw saturated",
                         result.first_raw_saturation_tick));
    CHECK_MSG(!result.samples.back().safety_locked,
              tickDetail("safety lock active", result.samples.back().tick));
  });

  runCase("S2: roll/pitch settling symmetry", [] {
    RunConfig roll_config;
    roll_config.initial.phi = 8.0 * kDegToRad;
    roll_config.ticks = 3000;
    const RunResult roll_result = runSil(roll_config);

    RunConfig pitch_config;
    pitch_config.initial.theta = 8.0 * kDegToRad;
    pitch_config.ticks = 3000;
    const RunResult pitch_result = runSil(pitch_config);

    CHECK_MSG(roll_result.all_finite,
              tickDetail("non-finite roll symmetry state",
                         roll_result.first_nonfinite_tick));
    CHECK_MSG(pitch_result.all_finite,
              tickDetail("non-finite pitch symmetry state",
                         pitch_result.first_nonfinite_tick));
    CHECK_MSG(!roll_result.raw_saturated,
              tickDetail("roll symmetry IMU raw saturated",
                         roll_result.first_raw_saturation_tick));
    CHECK_MSG(!pitch_result.raw_saturated,
              tickDetail("pitch symmetry IMU raw saturated",
                         pitch_result.first_raw_saturation_tick));

    const double roll_settle = settlingTime90(roll_result, true, 8.0);
    const double pitch_settle = settlingTime90(pitch_result, false, 8.0);
    const double ratio = roll_settle / pitch_settle;
    std::cout << "[SIL] S2 settle90 roll=" << roll_settle
              << "s pitch=" << pitch_settle << "s ratio=" << ratio << '\n';

    CHECK_MSG(std::isfinite(roll_settle), "roll did not settle inside 3 s");
    CHECK_MSG(std::isfinite(pitch_settle), "pitch did not settle inside 3 s");
    CHECK_GE(ratio, 0.6);
    CHECK_LE(ratio, 1.6);
  });

  runCase("S2b: low-throttle yaw saturation preserves pitch authority", [] {
    const double yaw_torque_per_us =
        4.0 * kPlantParameters.yaw_moment_arm_m() *
        kPlantParameters.thrust_per_us_n;
    const double available_yaw_torque_nm =
        0.5 * (1520.0 - 1050.0) * yaw_torque_per_us;
    CHECK_MSG(kLowThrottleYawTorqueNm > available_yaw_torque_nm,
              "fixture yaw torque does not exceed the 1270us motor authority");
    CHECK_MSG(kLowThrottlePitchTorqueNm < kLowThrottleYawTorqueNm,
              "fixture pitch torque must remain smaller than yaw torque");

    const RunResult priority = runSil(lowThrottleYawPitchConfig(false));
    const RunResult uniform = runSil(lowThrottleYawPitchConfig(true));

    double min_rp_scale = 1.0;
    double min_yaw_scale = 1.0;
    uint32_t first_yaw_limited_tick =
        std::numeric_limits<uint32_t>::max();
    uint32_t first_rp_limited_tick =
        std::numeric_limits<uint32_t>::max();
    uint32_t limited_tick = std::numeric_limits<uint32_t>::max();
    uint32_t saturated_target_run = 0;
    uint32_t max_saturated_target_run = 0;
    int min_pid_hz = std::numeric_limits<int>::max();
    int max_pid_hz = 0;
    bool motor_limits_valid = true;
    bool faults_clear = true;
    bool failsafe_clear = true;
    bool uniform_faults_clear = true;
    bool uniform_failsafe_clear = true;
    uint32_t uniform_scaled_samples = 0;
    double uniform_max_scale_mismatch = 0.0;
    double uniform_max_axis_scale_mismatch = 0.0;
    double pre_disturbance_max_pitch_deg = 0.0;

    for (const Sample &sample : priority.samples) {
      if (sample.tick < kLowThrottleDisturbanceTick) {
        pre_disturbance_max_pitch_deg = std::max(
            pre_disturbance_max_pitch_deg,
            std::fabs(sample.plant.theta * kRadToDeg));
      } else {
        min_rp_scale = std::min(min_rp_scale, sample.mixer_rp_scale);
        min_yaw_scale = std::min(min_yaw_scale, sample.mixer_yaw_scale);
        if (first_yaw_limited_tick ==
                std::numeric_limits<uint32_t>::max() &&
            sample.mixer_yaw_scale < 0.999) {
          first_yaw_limited_tick = sample.tick;
        }
        if (first_rp_limited_tick ==
                std::numeric_limits<uint32_t>::max() &&
            sample.mixer_rp_scale < 0.999) {
          first_rp_limited_tick = sample.tick;
        }
        if (limited_tick == std::numeric_limits<uint32_t>::max() &&
            sample.yaw_authority_state == YAW_AUTH_LIMITED) {
          limited_tick = sample.tick;
        }
      }

      if (limited_tick != std::numeric_limits<uint32_t>::max() &&
          sample.tick >= limited_tick) {
        if (std::fabs(sample.target_yaw_rate_dps) >= 179.5) {
          saturated_target_run++;
          max_saturated_target_run = std::max(
              max_saturated_target_run, saturated_target_run);
        } else {
          saturated_target_run = 0;
        }
      }
      if (sample.tick >= 1500U && sample.pid_loop_hz > 0) {
        min_pid_hz = std::min(min_pid_hz, sample.pid_loop_hz);
        max_pid_hz = std::max(max_pid_hz, sample.pid_loop_hz);
      }

      if (sample.tick != kLowThrottleTick) {
        const int expected_max =
            sample.tick < kLowThrottleTick ? 1550 : 1520;
        for (int motor : sample.motors) {
          motor_limits_valid =
              motor_limits_valid && motor >= 1050 && motor <= expected_max;
        }
      }
      faults_clear =
          faults_clear && !sample.fault_rc_now && !sample.fault_imu1_now &&
          !sample.fault_imu2_now && !sample.fault_disagree_now &&
          !sample.fault_attitude_now;
      failsafe_clear = failsafe_clear && sample.failsafe_phase == FS_NONE;
    }
    for (const Sample &sample : uniform.samples) {
      if (sample.plant_allocator_scale < 0.999) {
        uniform_scaled_samples++;
        uniform_max_scale_mismatch = std::max(
            uniform_max_scale_mismatch,
            std::fabs(sample.mixer_rp_scale -
                      sample.plant_allocator_scale));
        uniform_max_axis_scale_mismatch = std::max(
            uniform_max_axis_scale_mismatch,
            std::fabs(sample.mixer_rp_scale - sample.mixer_yaw_scale));
      }
      uniform_faults_clear =
          uniform_faults_clear && !sample.fault_rc_now &&
          !sample.fault_imu1_now && !sample.fault_imu2_now &&
          !sample.fault_disagree_now && !sample.fault_attitude_now;
      uniform_failsafe_clear =
          uniform_failsafe_clear && sample.failsafe_phase == FS_NONE;
    }

    const double priority_pitch_deg =
        maxAbsPitchFromTick(priority, kLowThrottleDisturbanceTick);
    const double uniform_pitch_deg =
        maxAbsPitchFromTick(uniform, kLowThrottleDisturbanceTick);
    const uint32_t post_limited_observation_ticks =
        limited_tick == std::numeric_limits<uint32_t>::max()
            ? 0U
            : priority.samples.back().tick - limited_tick;
    std::cout << "[SIL] S2b throttle=1300->1270us yaw_torque="
              << kLowThrottleYawTorqueNm << "Nm authority="
              << available_yaw_torque_nm << "Nm pitch_torque="
              << kLowThrottlePitchTorqueNm << "Nm min_rp_scale="
              << min_rp_scale << " min_yaw_scale=" << min_yaw_scale
              << " first_yaw_limited="
              << (first_yaw_limited_tick ==
                          std::numeric_limits<uint32_t>::max()
                      ? -1
                      : static_cast<int>(first_yaw_limited_tick))
              << " first_rp_limited="
              << (first_rp_limited_tick ==
                          std::numeric_limits<uint32_t>::max()
                      ? -1
                      : static_cast<int>(first_rp_limited_tick))
              << " authority_limited="
              << (limited_tick == std::numeric_limits<uint32_t>::max()
                      ? -1
                      : static_cast<int>(limited_tick))
              << " post_limited_observation_ms="
              << post_limited_observation_ticks
              << " max_sat_target_run_ms=" << max_saturated_target_run
              << " pitch_priority=" << priority_pitch_deg
              << "deg pitch_uniform=" << uniform_pitch_deg
              << "deg uniform_scaled_samples=" << uniform_scaled_samples
              << " uniform_max_scale_mismatch="
              << uniform_max_scale_mismatch
              << " uniform_max_axis_scale_mismatch="
              << uniform_max_axis_scale_mismatch
              << " pid_hz=" << min_pid_hz << ".." << max_pid_hz
              << "\n";

    CHECK_MSG(sampleAtTick(priority, kLowThrottleTick - 1U)
                      .base_throttle_us == 1300 &&
                  sampleAtTick(priority, kLowThrottleTick + 1U)
                      .base_throttle_us == 1270,
              "fixture did not establish 1300us then descend to 1270us");
    CHECK_MSG(pre_disturbance_max_pitch_deg < 0.25,
              "normal 1300us/settled-1270us segment was not level");
    CHECK_MSG(first_yaw_limited_tick !=
                  std::numeric_limits<uint32_t>::max(),
              "yaw allocation never limited under excessive yaw torque");
    CHECK_MSG(first_yaw_limited_tick < first_rp_limited_tick,
              "roll/pitch allocation limited before yaw");
    CHECK_MSG(min_rp_scale >= 0.999,
              "roll/pitch was scaled even though its own demand fit");
    CHECK_MSG(min_yaw_scale <= YAW_AUTH_ENTRY_SCALE,
              "yaw allocation did not reach the authority-entry threshold");
    CHECK_MSG(limited_tick != std::numeric_limits<uint32_t>::max(),
              "yaw authority state never reached LIMITED");
    CHECK_MSG(post_limited_observation_ticks >=
                  kPostLimitedObservationTicks,
              "SIL ended before the complete 500ms post-LIMITED window");
    CHECK_MSG(max_saturated_target_run < 500U,
              "target yaw rate remained at +/-180dps for 0.5s after LIMITED");
    CHECK_MSG(uniform_scaled_samples > 0U,
              "uniform oracle never exercised a scaled allocation");
    CHECK_MSG(uniform_max_scale_mismatch <= 1.0e-6,
              "uniform plant delivery disagreed with PID anti-windup scale");
    CHECK_MSG(uniform_max_axis_scale_mismatch <= 1.0e-6,
              "uniform PID tick received unequal RP and yaw scales");
    CHECK_MSG(priority_pitch_deg + 0.25 < uniform_pitch_deg,
              "priority allocation did not beat the independent uniform oracle");
    CHECK_MSG(priority_pitch_deg < 15.0,
              "priority allocation exceeded the 15deg pitch safety gate");
    CHECK_MSG(priority.all_motors_in_range && motor_limits_valid,
              "motor output left the active PWM bounds");
    CHECK_MSG(priority.all_finite && !priority.raw_saturated,
              "SIL state or synthetic IMU left its valid numeric range");
    CHECK_MSG(uniform.all_finite && !uniform.raw_saturated,
              "uniform oracle left its valid numeric range");
    CHECK_MSG(!uniform.safety_lock_ever && uniform_faults_clear &&
                  uniform_failsafe_clear,
              "uniform oracle tripped safety or failsafe before comparison");
    CHECK_MSG(!priority.safety_lock_ever && faults_clear && failsafe_clear,
              "manual low-throttle scenario tripped safety or failsafe");
    CHECK_MSG(min_pid_hz >= 999 && max_pid_hz <= 1001,
              "PID loop cadence left the 1kHz invariant");
  });

  runCase("S3: anti-windup clamps and recovers after saturation", [] {
    constexpr uint32_t kPreloadEnd = 8000;
    constexpr uint32_t kSaturationEnd = 8040;
    constexpr uint32_t kRunEnd = 13000;
    RunConfig config;
    config.ticks = kRunEnd;
    // 요구된 Ki sweep의 0.5 값을 사용해 native 실행 시간 안에 clamp 경로를
    // 충분히 자극한다. 펌웨어의 ±50 us 한계 자체는 바꾸지 않는다.
    config.ki_roll = 0.5f;
    config.ki_pitch = 0.5f;
    config.disturbance_for_interval = [](uint32_t tick) {
      if (tick < 2000U) {
        return Disturbance{0.060 * static_cast<double>(tick) / 2000.0,
                           0.0, 0.0};
      }
      if (tick < kPreloadEnd) return Disturbance{0.060, 0.0, 0.0};
      if (tick < kSaturationEnd) return Disturbance{0.300, 0.0, 0.0};
      return Disturbance{};
    };
    const RunResult result = runSil(config);

    uint32_t scaled_samples = 0;
    uint32_t max_roll_tick = 0;
    double max_roll_deg = 0.0;
    for (const Sample &sample : result.samples) {
      if (sample.tick >= kPreloadEnd && sample.tick <= kSaturationEnd &&
          sample.mixer_scaled_now) {
        scaled_samples++;
      }
      const double roll_deg = std::fabs(sample.plant.phi * kRadToDeg);
      if (roll_deg > max_roll_deg) {
        max_roll_deg = roll_deg;
        max_roll_tick = sample.tick;
      }
    }
    const double removal_i = sampleAtTick(result, kSaturationEnd).i_roll_us;
    const double final_i = result.samples.back().i_roll_us;

    // 별도 짧은 런은 clamp에 닿기 전에 포화시켜 !mix.scaled 조건 자체를
    // 검증한다. 연속 scaled tick에서 iTerm이 한 비트도 움직이면 실패한다.
    RunConfig gate_config;
    gate_config.ticks = 1500;
    gate_config.ki_roll = 0.5f;
    gate_config.ki_pitch = 0.5f;
    gate_config.disturbance_for_interval = [](uint32_t tick) {
      return tick < 60U ? Disturbance{0.300, 0.0, 0.0} : Disturbance{};
    };
    const RunResult gate_result = runSil(gate_config);
    uint32_t gate_hold_pairs = 0;
    double gate_max_i_delta = 0.0;
    uint32_t gate_max_i_delta_tick = 0;
    for (std::size_t index = 1; index < gate_result.samples.size(); index++) {
      const Sample &previous = gate_result.samples[index - 1U];
      const Sample &current = gate_result.samples[index];
      const bool below_clamp =
          std::fabs(previous.i_roll_us) < I_TERM_MAX_US - 5.0 &&
          std::fabs(current.i_roll_us) < I_TERM_MAX_US - 5.0;
      if (previous.mixer_scaled_now && current.mixer_scaled_now && below_clamp) {
        gate_hold_pairs++;
        const double delta =
            std::fabs(current.i_roll_us - previous.i_roll_us);
        if (delta > gate_max_i_delta) {
          gate_max_i_delta = delta;
          gate_max_i_delta_tick = current.tick;
        }
      }
    }
    std::cout << "[SIL] S3 max|iRoll|=" << result.max_abs_i_roll_us
              << "us@" << result.max_abs_i_roll_tick
              << " max|iPitch|=" << result.max_abs_i_pitch_us
              << "us@" << result.max_abs_i_pitch_tick
              << " scaled_samples=" << scaled_samples
              << " removal_iRoll=" << removal_i
              << "us final_iRoll=" << final_i
              << "us max|phi|=" << result.max_abs_roll_deg
              << "deg@" << max_roll_tick
              << " preload_phi="
              << sampleAtTick(result, kPreloadEnd).plant.phi * kRadToDeg
              << "deg gate_hold_pairs=" << gate_hold_pairs
              << " gate_max_i_delta=" << gate_max_i_delta
              << "us@" << gate_max_i_delta_tick << "\n";

    CHECK_MSG(result.max_abs_i_roll_us <= I_TERM_MAX_US + 0.001,
              metricTickDetail("max|iTermRoll| us", result.max_abs_i_roll_us,
                               result.max_abs_i_roll_tick,
                               I_TERM_MAX_US + 0.001));
    CHECK_MSG(result.max_abs_i_pitch_us <= I_TERM_MAX_US + 0.001,
              metricTickDetail("max|iTermPitch| us", result.max_abs_i_pitch_us,
                               result.max_abs_i_pitch_tick,
                               I_TERM_MAX_US + 0.001));
    CHECK_MSG(result.max_abs_i_roll_us >= I_TERM_MAX_US - 1.0,
              metricTickDetail("max|iTermRoll| us", result.max_abs_i_roll_us,
                               result.max_abs_i_roll_tick,
                               I_TERM_MAX_US - 1.0));
    CHECK_MSG(std::fabs(removal_i) >= I_TERM_MAX_US - 1.0,
              metricTickDetail("removal |iTermRoll| us", std::fabs(removal_i),
                               kSaturationEnd, I_TERM_MAX_US - 1.0));
    CHECK_MSG(scaled_samples > 0, "disturbance did not exercise mixer saturation");
    CHECK_MSG(!result.safety_lock_ever,
              tickDetail("S3 invalidated by safety lock",
                         result.first_safety_lock_tick));
    CHECK_MSG(result.all_finite,
              tickDetail("non-finite S3 state", result.first_nonfinite_tick));
    CHECK_MSG(!result.raw_saturated,
              tickDetail("S3 synthetic IMU raw saturated",
                         result.first_raw_saturation_tick));
    CHECK_MSG(std::fabs(final_i) + 1.0 < std::fabs(removal_i),
              transitionDetail("iTermRoll recovery", removal_i,
                               kSaturationEnd, final_i,
                               result.samples.back().tick));
    CHECK_MSG(gate_result.all_finite,
              tickDetail("non-finite saturation-gate probe state",
                         gate_result.first_nonfinite_tick));
    CHECK_MSG(!gate_result.raw_saturated,
              tickDetail("saturation-gate probe IMU raw saturated",
                         gate_result.first_raw_saturation_tick));
    CHECK_MSG(!gate_result.safety_lock_ever,
              tickDetail("saturation-gate probe hit safety lock",
                         gate_result.first_safety_lock_tick));
    CHECK_MSG(gate_hold_pairs >= 5,
              "saturation-gate probe lacked a contiguous below-clamp interval");
    CHECK_MSG(gate_max_i_delta <= 1e-7,
              metricTickDetail("scaled iTermRoll delta us", gate_max_i_delta,
                               gate_max_i_delta_tick, 1e-7));
  });

  runReport("S4 integrator authority (numbers only)", [] {
    const double roll_torque_per_us =
        4.0 * kPlantParameters.arm_projection_m() *
        kPlantParameters.thrust_per_us_n;
    const double disturbance_nm = roll_torque_per_us * 0.50 * 6.0 * 2.5;

    const RunResult p_only = runSil(
        constantRollDisturbance(0.0f, disturbance_nm, 10000));
    const double p_only_ss = meanAbsTailDeg(p_only, true, 500);
    std::cout << "[SIL] S4 P-only Ki=0.000 10s |phi_ss|="
              << p_only_ss << "deg iTermRoll="
              << p_only.samples.back().i_roll_us
              << "us raw_saturated=" << p_only.raw_saturated << "\n";

    const RunResult nominal = runSil(
        constantRollDisturbance(0.005f, disturbance_nm, 10000));
    std::cout << "[SIL] S4 Ki=0.005 |phi|(1s,2s,5s,10s)="
              << std::fabs(sampleAtTick(nominal, 1000).plant.phi * kRadToDeg) << ","
              << std::fabs(sampleAtTick(nominal, 2000).plant.phi * kRadToDeg) << ","
              << std::fabs(sampleAtTick(nominal, 5000).plant.phi * kRadToDeg) << ","
              << std::fabs(sampleAtTick(nominal, 10000).plant.phi * kRadToDeg)
              << "deg final_iTermRoll=" << nominal.samples.back().i_roll_us
              << "us authority="
              << 100.0 * std::fabs(nominal.samples.back().i_roll_us) /
                            I_TERM_MAX_US
              << "% of +/-50us raw_saturated=" << nominal.raw_saturated << "\n";

    const auto print_sweep = [](double ki, const RunResult &result) {
      std::cout << "[SIL] S4 sweep Ki=" << ki
                << " 10s |phi|="
                << std::fabs(result.samples.back().plant.phi * kRadToDeg)
                << "deg iTermRoll=" << result.samples.back().i_roll_us
                << "us raw_saturated=" << result.raw_saturated << "\n";
    };
    print_sweep(0.005, nominal);
    const RunResult ki_005 = runSil(
        constantRollDisturbance(0.05f, disturbance_nm, 10000));
    print_sweep(0.05, ki_005);
    const RunResult ki_05 = runSil(
        constantRollDisturbance(0.5f, disturbance_nm, 10000));
    print_sweep(0.5, ki_05);

    std::cout << "[SIL] S4 note: guessed plant parameters make settling/steady-state "
                 "numbers order-of-magnitude; only the conclusion that Ki=0.005 is "
                 "effectively inactive on a 10s hover timescale is used.\n";
  });

  runReport("S5 yaw damping (numbers only)", [] {
    RunConfig config;
    config.initial.r = 60.0 * kDegToRad;
    config.ticks = 2000;
    const RunResult result = runSil(config);
    std::cout << "[SIL] S5 signed r(dps) t=0,0.5,1.0,1.5,2.0s: "
              << sampleAtTick(result, 0).plant.r * kRadToDeg << ","
              << sampleAtTick(result, 500).plant.r * kRadToDeg << ","
              << sampleAtTick(result, 1000).plant.r * kRadToDeg << ","
              << sampleAtTick(result, 1500).plant.r * kRadToDeg << ","
              << sampleAtTick(result, 2000).plant.r * kRadToDeg
              << " raw_saturated=" << result.raw_saturated << '\n';
    std::cout << "[SIL] S5 note: yaw reaction-torque sign is convention-dependent; "
                 "confirm prop direction on a powered bench before asserting damping.\n";
  });

  runCase("S6 yaw 적분 해금: hold가 아닌 구간에서도 적분기가 누적된다", [] {
    // SIL yaw 플랜트의 반작용 토크 부호는 미해결이다(S5 참조: runReport로만
    // 남기고 단언하지 않는다). 따라서 폐루프 정상상태 각속도로는 단언할 수
    // 없다. 이 케이스는 코드 변경 자체 — "hold가 아니어도 iTermYaw가
    // 누적된다" — 만 검증하며 플랜트 부호와 무관하다.
    const double yaw_torque_per_us =
        4.0 * kPlantParameters.yaw_moment_arm_m() *
        kPlantParameters.thrust_per_us_n;
    const double disturbance_nm = yaw_torque_per_us * 1.50 * 20.0;

    RunConfig config =
        constantYawDisturbance(0.05f, disturbance_nm, 3000);
    // heading hold는 외란으로 풀리지 않는다. 실제 스틱 명령으로 rate 모드에
    // 진입시켜 이 테스트의 본래 목적(그 모드에서 yaw 적분 누적)을 검증한다.
    config.state_for_tick = [](uint32_t, PlantState &) {
      targetYawRate = 45.0f;
    };
    const RunResult result = runSil(config);

    std::size_t rate_mode_samples = 0;
    double max_i_yaw_rate_mode = 0.0;
    for (const Sample &sample : result.samples) {
      if (sample.yaw_hold) continue;
      rate_mode_samples++;
      max_i_yaw_rate_mode =
          std::max(max_i_yaw_rate_mode, std::fabs(sample.i_yaw_us));
    }

    // 판별력 확인: hold가 아닌 구간이 실제로 있어야 이 테스트가 의미를 갖는다.
    CHECK_MSG(rate_mode_samples > 100,
              "hold가 아닌 구간이 거의 없어 해금 여부를 가릴 수 없다");
    // 해금 전에는 hold가 아니면 iTermYaw가 0으로 묶였다. 여기서 red/green이 갈린다.
    CHECK_MSG(max_i_yaw_rate_mode > 0.0,
              "rate 모드에서 yaw 적분기가 전혀 누적되지 않았다 (해금 실패)");
    CHECK_LE(result.max_abs_i_yaw_us, static_cast<double>(I_TERM_MAX_US));
    std::cout << "[SIL] S6 rate-mode samples=" << rate_mode_samples
              << " max|iTermYaw| in rate mode=" << max_i_yaw_rate_mode
              << "us, overall max=" << result.max_abs_i_yaw_us << "us\n";
  });

  runReport("S6b yaw closed-loop rate (numbers only)", [] {
    // 플랜트 yaw 부호가 확정되기 전까지 수치만 남긴다. S5와 같은 취급이다.
    const double yaw_torque_per_us =
        4.0 * kPlantParameters.yaw_moment_arm_m() *
        kPlantParameters.thrust_per_us_n;
    const double disturbance_nm = yaw_torque_per_us * 1.50 * 20.0;
    const RunResult p_only =
        runSil(constantYawDisturbance(0.0f, disturbance_nm, 3000));
    const RunResult with_i =
        runSil(constantYawDisturbance(0.05f, disturbance_nm, 3000));
    std::cout << "[SIL] S6b tail |r| P-only="
              << tailMeanAbsYawRateDps(p_only, 500) << "dps Ki=0.05="
              << tailMeanAbsYawRateDps(with_i, 500)
              << "dps -- 플랜트 yaw 부호 미해결, 벤치 Stage D에서 확정\n";
  });

  runCase("R1: mass 1.3x noise 0.08g phase sweep has no airborne landing", [] {
    constexpr double kMassScale = 1.30;
    constexpr double kNoiseSdG = 0.08;
    constexpr uint32_t kNoisePeriodTicks = 10000U;
    constexpr uint32_t kPhaseStepTicks = 100U;
    constexpr int kExpectedDipUs = 52;
    int airborne_false_landings = 0;
    int wrong_dip_runs = 0;

    for (uint32_t phase_ticks = 0;
         phase_ticks < kNoisePeriodTicks;
         phase_ticks += kPhaseStepTicks) {
      RunConfig config;
      config.initial.z_m = 3.0;
      config.vertical_enabled = true;
      config.plant_parameters.mass_kg =
          kPlantParameters.mass_kg * kMassScale;
      config.plant_parameters.k_ground = 2000.0;
      config.plant_parameters.c_ground =
          2.0 * std::sqrt(config.plant_parameters.k_ground *
                          config.plant_parameters.mass_kg);
      config.plant_parameters.k_ge = 0.0;
      config.base_throttle_us = 1000 + static_cast<int>(std::lround(
          config.plant_parameters.mass_kg * kGravityMs2 /
          (4.0 * config.plant_parameters.thrust_per_us_n)));
      config.rc_disconnect_tick = 5000U + phase_ticks;
      config.ticks =
          config.rc_disconnect_tick + RC_TIMEOUT_MS + FS_MAX_MS + 1500U;
      config.accel_noise_enabled = true;
      config.accel_noise_sd_g = kNoiseSdG;

      const RunResult result = runSil(config);
      const FailsafeTrace trace = analyzeFailsafeTrace(result);
      if (trace.terminal_phase == FS_CUT_LANDED &&
          trace.terminal_tick < trace.contact_tick) {
        airborne_false_landings++;
      }
      int max_observed_dip_us = 0;
      const int descent_throttle =
          config.base_throttle_us - FS_DESCENT_DELTA_US;
      for (const Sample &sample : result.samples) {
        if (sample.failsafe_probe_state == FS_PROBE_DIP) {
          max_observed_dip_us = std::max(
              max_observed_dip_us,
              descent_throttle - sample.base_throttle_us);
        }
      }
      if (max_observed_dip_us != kExpectedDipUs) {
        wrong_dip_runs++;
      }
    }

    constexpr int kPhaseCount =
        static_cast<int>(kNoisePeriodTicks / kPhaseStepTicks);
    std::cout << "[R1] mass_scale=" << kMassScale
              << " noise_sd_g=" << kNoiseSdG
              << " phases=" << kPhaseCount
              << " airborne_false_landings=" << airborne_false_landings
              << " rate_pct="
              << 100.0 * airborne_false_landings / kPhaseCount
              << " wrong_dip_runs=" << wrong_dip_runs
              << " expected=0/100 (0%), dip=52us in every run\n";
    CHECK_EQ(airborne_false_landings, 0);
    CHECK_EQ(wrong_dip_runs, 0);
  });

  RunResult v1_no_noise;
  RunResult v1_noise_004;
  runCase("V1: noise OFF/ON normal descent lands after contact", [&] {
    std::cout << "[SIL] V1 noise comparison OFF(label=V1) then "
                 "ON(sd=0.04g)\n";
    v1_no_noise = runSil(makeV1Config());
    printFailsafeTrace("V1", v1_no_noise);
    printProbeEvaluations("V1", v1_no_noise);
    checkTerminatesNotBeforeContact("V1", analyzeFailsafeTrace(v1_no_noise));
    v1_noise_004 = runSil(makeV1Config(true, 0.04));
    printFailsafeTrace("V1 noise=ON sd=0.04g", v1_noise_004);
    checkTerminatesNotBeforeContact(
        "V1 noise=ON sd=0.04g", analyzeFailsafeTrace(v1_noise_004));

    const FailsafeTrace trace = analyzeFailsafeTrace(v1_no_noise);
    const std::vector<double> responses =
        groundProbeResponses(v1_no_noise, trace.contact_tick);
    // 프로브가 판정에서 빠진 뒤로 필요한 개수는 CONFIRM_N 과 무관하다. 확인할
    // 것은 "지면에서도 프로브가 계속 돌며 텔레메트리를 채운다"는 것뿐이고,
    // 그 기록이 다음 판별식 설계의 입력이다. FS_MAX_MS 를 3s 로 줄이면서
    // 접지 후 남는 프로브 수도 줄었다.
    CHECK_MSG(responses.size() >= 2,
              "V1 did not record settled-ground probe responses");
    const double max_ground_response =
        *std::max_element(responses.begin(), responses.end());
    std::cout << "[SIL] V1 ground probe responses=";
    for (std::size_t index = 0; index < responses.size(); index++) {
      if (index > 0) std::cout << ",";
      std::cout << responses[index] << "g";
    }
    std::cout << " max=" << max_ground_response
              << "g threshold=" << FS_PROBE_RESPONSE_G << "g\n";
    CHECK_MSG(max_ground_response < FS_PROBE_RESPONSE_G,
              "settled ground absorbed less of the dip than expected");
  });

  runCase("V1: accel noise sd 0.02/0.04/0.06g all land", [&] {
    constexpr std::array<double, 3> kNoiseSweepG = {0.02, 0.04, 0.06};
    double first_failure_sd_g = std::numeric_limits<double>::quiet_NaN();
    for (double noise_sd_g : kNoiseSweepG) {
      const RunResult result =
          noise_sd_g == 0.04
              ? v1_noise_004
              : runSil(makeV1Config(true, noise_sd_g));
      std::ostringstream label;
      label << std::fixed << std::setprecision(2)
            << "V1 noise sweep sd=" << noise_sd_g << "g";
      printFailsafeTrace(label.str().c_str(), result);
      const FailsafeTrace trace = analyzeFailsafeTrace(result);
      checkTerminatesNotBeforeContact(label.str().c_str(), trace);
      if (!std::isfinite(first_failure_sd_g) &&
          trace.terminal_phase != FS_CUT_LANDED) {
        first_failure_sd_g = noise_sd_g;
      }
    }
    std::cout << "[SIL] V1 noise sweep first_landing_failure_sd="
              << (std::isfinite(first_failure_sd_g)
                      ? std::to_string(first_failure_sd_g) + "g"
                      : "none_through_0.06g")
              << '\n';

    RunConfig common_mode = makeV1Config(true, 0.06);
    common_mode.accel_common_mode_start_tick =
        kHoverWarmupTicks + RC_TIMEOUT_MS;
    common_mode.accel_common_mode_g = -0.08;
    const RunResult common_mode_result = runSil(common_mode);
    constexpr const char *kCommonModeLabel =
        "V1 noise sweep sd=0.06g common-mode trough=-0.08g";
    printFailsafeTrace(kCommonModeLabel, common_mode_result);
    checkTerminatesNotBeforeContact(
        kCommonModeLabel, analyzeFailsafeTrace(common_mode_result));
  });

  runCase("V2: hover estimate makes climbing link loss descend", [] {
    const RunResult result = runSil(makeV2Config());
    printFailsafeTrace("V2", result);
    printProbeEvaluations("V2", result);
    const FailsafeTrace trace = analyzeFailsafeTrace(result);
    CHECK_MSG(std::isfinite(trace.entry_z_m), "V2 did not enter failsafe");
    CHECK_MSG(std::isfinite(trace.terminal_z_m), "V2 did not terminate");
    const double delta_z_m = trace.terminal_z_m - trace.entry_z_m;
    std::ostringstream detail;
    detail << "actual delta_z=" << delta_z_m << "m, expected delta_z<0";
    CHECK_MSG(delta_z_m < 0.0, detail.str());
    // 현재는 landed=false이므로 CONFIRM_N과 무관하게 백스톱으로 종료한다.
    // V2의 상승 중 링크 단절 조건에서 실제 접촉 시각과 종료 시각을 비교한다.
    // 아래 검사는 이 SIL 조건의 접지 전 종료를 막으며 실기 고도 범위를 보장하지 않는다.
    checkTerminatesNotBeforeContact("V2", trace);
    const double contact_to_cut_s =
        (trace.terminal_tick - trace.contact_tick) * kDt;
    // 아래 confirm_budget_s 식은 검출기를 제어에 쓰던 당시부터 남은 지연 상한
    // 검사다. 현재 컷 지연이 CONFIRM_N으로 결정된다는 뜻이 아니며, 접지 후
    // 최소 확정 시간이나 실기 지연을 증명하지 않는다. CONFIRM_N을 4로 올린
    // 근거였던 2026-08-01 공중 CUT_LANDED 관측도 철회됐다. 여기서는 선택된
    // V2 모델의 접촉→컷 지연이 이 기존 수치 상한을 넘지 않는지만 검사한다.
    const double confirm_budget_s =
        std::min(FS_PROBE_CONFIRM_N * (FS_PROBE_PERIOD_MS / 1000.0) + 0.4,
                 FS_MAX_MS / 1000.0)
        + FS_PROBE_PERIOD_MS / 1000.0;
    std::cout << "[SIL] V2 contact_to_cut=" << contact_to_cut_s
              << "s expected<=" << confirm_budget_s << "s\n";
    CHECK_MSG(contact_to_cut_s <= confirm_budget_s,
              "V2 contact-to-cut delay did not materially beat timeout");
  });

  runCase("V2b: link loss without prior hover cuts immediately", [] {
    const RunResult result = runSil(makeV2bConfig());
    printFailsafeTrace("V2b", result);

    uint32_t first_locked_tick = std::numeric_limits<uint32_t>::max();
    bool entered_descending = false;
    for (const Sample &sample : result.samples) {
      if (first_locked_tick == std::numeric_limits<uint32_t>::max() &&
          sample.safety_locked) {
        first_locked_tick = sample.tick;
      }
      entered_descending =
          entered_descending || sample.failsafe_phase == FS_DESCENDING;
    }
    std::ostringstream detail;
    detail << "actual first_locked_tick="
           << (first_locked_tick == std::numeric_limits<uint32_t>::max()
                   ? "none"
                   : std::to_string(first_locked_tick))
           << ", entered_descending=" << entered_descending
           << "; expected first_locked_tick<="
           << RC_TIMEOUT_MS + 2U << " and entered_descending=0";
    CHECK_MSG(first_locked_tick <= RC_TIMEOUT_MS + 2U &&
                  !entered_descending,
              detail.str());
    std::cout << "[SIL] V2b first_locked="
              << tickSeconds(first_locked_tick)
              << " entered_descending=" << entered_descending << '\n';
  });

  runCase("V3: k_ge 0.15/0.25/0.35 has no precontact landing", [&] {
    std::cout << "[SIL] V3 setup z0=0.5000m vz0=0.0000m/s "
                 "entry_throttle=1340us descent_throttle=1280us "
                 "external_force=0\n";
    double first_precontact_landing_k_ge =
        std::numeric_limits<double>::quiet_NaN();
    for (std::size_t index = 0; index < kV3GroundEffects.size(); index++) {
      std::ostringstream label;
      label << std::fixed << std::setprecision(2)
            << "V3 k_ge=" << kV3GroundEffects[index];
      printFailsafeTrace(label.str().c_str(), v3_results[index]);
      printProbeEvaluations(label.str().c_str(), v3_results[index]);
      const FailsafeTrace trace = analyzeFailsafeTrace(v3_results[index]);
      double precontact_accel_max_g =
          -std::numeric_limits<double>::infinity();
      for (const Sample &sample : v3_results[index].samples) {
        if (sample.tick < trace.entry_tick ||
            sample.tick >= trace.contact_tick) {
          continue;
        }
        precontact_accel_max_g =
            std::max(precontact_accel_max_g, sample.accel_magnitude_g);
      }
      std::cout << "[SIL] " << label.str()
                << " precontact_accel_max=" << precontact_accel_max_g
                << "g probe_response_threshold=" << FS_PROBE_RESPONSE_G
                << "g\n";
      const bool landed_before_contact =
          trace.terminal_phase == FS_CUT_LANDED &&
          trace.terminal_tick < trace.contact_tick;
      if (!std::isfinite(first_precontact_landing_k_ge) &&
          landed_before_contact) {
        first_precontact_landing_k_ge = kV3GroundEffects[index];
      }
      CHECK_MSG(!landed_before_contact,
                "V3 declared landing before first contact");
    }
    std::cout << "[SIL] V3 boundary first_precontact_landing_k_ge="
              << (std::isfinite(first_precontact_landing_k_ge)
                      ? std::to_string(first_precontact_landing_k_ge)
                      : "none_through_0.35")
              << '\n';
  });

  runCase("V4: gust then steady descent lands only after contact", [] {
    std::cout << "[SIL] V4 noise comparison OFF(label=V4) then "
                 "ON(sd=0.04g)\n";
    std::cout << "[SIL] V4 setup hover=3.000s then vz=-1.2000m/s "
                 "down=-0.20g/150ms up=+0.40g/50ms then "
                 "linear drag balance at -1.2000m/s\n";
    const RunResult result = runSil(makeV4Config());
    printFailsafeTrace("V4", result);
    printProbeEvaluations("V4", result);
    checkTerminatesNotBeforeContact("V4", analyzeFailsafeTrace(result));
    const uint32_t quiet_start_tick =
        kHoverWarmupTicks + RC_TIMEOUT_MS + 202U;
    const uint32_t quiet_end_tick = quiet_start_tick + 400U;
    const Sample &quiet_start = sampleAtTick(result, quiet_start_tick);
    const Sample &quiet_400ms = sampleAtTick(result, quiet_end_tick);
    double quiet_min_accel_g = std::numeric_limits<double>::infinity();
    double quiet_max_accel_g = -std::numeric_limits<double>::infinity();
    for (uint32_t tick = quiet_start_tick; tick <= quiet_end_tick; tick++) {
      const Sample &sample = sampleAtTick(result, tick);
      quiet_min_accel_g =
          std::min(quiet_min_accel_g, sample.accel_magnitude_g);
      quiet_max_accel_g =
          std::max(quiet_max_accel_g, sample.accel_magnitude_g);
    }
    std::cout << "[SIL] V4 quiet400 vz=" << quiet_start.plant.vz_ms
              << "->" << quiet_400ms.plant.vz_ms << "m/s z="
              << quiet_start.plant.z_m << "->" << quiet_400ms.plant.z_m
              << "m accel_min=" << quiet_min_accel_g
              << "g accel_max=" << quiet_max_accel_g << "g\n";
    const RunResult noisy = runSil(makeV4Config(true, 0.04));
    printFailsafeTrace("V4 noise=ON sd=0.04g", noisy);
    printProbeEvaluations("V4 noise=ON sd=0.04g", noisy);
    checkTerminatesNotBeforeContact(
        "V4 noise=ON sd=0.04g", analyzeFailsafeTrace(noisy));
  });

  runCase("V5: one ground probe then bounce does not confirm while airborne", [] {
    const RunResult result = runSil(makeV5Config());
    printFailsafeTrace("V5 bounce", result);
    printProbeEvaluations("V5 bounce", result);
    const FailsafeTrace trace = analyzeFailsafeTrace(result);

    uint32_t bounce_start_tick = std::numeric_limits<uint32_t>::max();
    uint32_t bounce_end_tick = std::numeric_limits<uint32_t>::max();
    for (std::size_t index = 1; index < result.samples.size(); index++) {
      const Sample &previous = result.samples[index - 1U];
      const Sample &sample = result.samples[index];
      if (sample.tick <= trace.contact_tick) continue;
      if (bounce_start_tick == std::numeric_limits<uint32_t>::max() &&
          previous.plant.z_m <= 0.0 && sample.plant.z_m > 0.0) {
        bounce_start_tick = sample.tick;
      }
      if (bounce_start_tick != std::numeric_limits<uint32_t>::max() &&
          bounce_end_tick == std::numeric_limits<uint32_t>::max() &&
          previous.plant.z_m > 0.0 && sample.plant.z_m <= 0.0) {
        bounce_end_tick = sample.tick;
        break;
      }
    }

    CHECK_MSG(
        bounce_start_tick != std::numeric_limits<uint32_t>::max() &&
            bounce_end_tick != std::numeric_limits<uint32_t>::max(),
        "V5 did not produce a complete post-contact airborne interval");
    bool landed_while_airborne = false;
    for (const Sample &sample : result.samples) {
      if (sample.tick >= bounce_start_tick &&
          sample.tick < bounce_end_tick &&
          sample.failsafe_phase == FS_CUT_LANDED) {
        landed_while_airborne = true;
      }
    }
    std::cout << "[SIL] V5 bounce airborne="
              << tickSeconds(bounce_start_tick) << "->"
              << tickSeconds(bounce_end_tick)
              << " landed_while_airborne=" << landed_while_airborne
              << " terminal=" << tickSeconds(trace.terminal_tick) << '\n';
    CHECK_MSG(!landed_while_airborne,
              "V5 confirmed landing during the airborne bounce");
    CHECK_MSG(trace.terminal_tick >= bounce_end_tick,
              "V5 terminal landing preceded bounce re-contact");
    checkTerminatesNotBeforeContact("V5", trace);
  });

  runCase("V6: later explicit resume leaves more descent velocity and loss", [] {
    constexpr std::array<uint32_t, 3> kResumeDelays = {
        300U, 1000U, 2000U};
    std::array<ResumeMeasurement, kResumeDelays.size()> measurements;
    for (std::size_t index = 0; index < kResumeDelays.size(); index++) {
      measurements[index] =
          measureV6(runSil(makeV6Config(kResumeDelays[index])),
                    kResumeDelays[index]);
      const ResumeMeasurement &measurement = measurements[index];
      std::cout << "[SIL] V6 resume_delay="
                << measurement.delay_ticks * kDt
                << "s transition=" << tickSeconds(measurement.transition_tick)
                << " residual_vz=" << measurement.residual_vz_ms
                << "m/s post_" << kV6PostResumeTicks * kDt
                << "s_height_loss=" << measurement.post_height_loss_m
                << "m mean_post_accel="
                << measurement.mean_post_acceleration_ms2 << "m/s^2\n";
    }
    for (std::size_t index = 1; index < measurements.size(); index++) {
      CHECK_MSG(
          std::fabs(measurements[index].residual_vz_ms) >
              std::fabs(measurements[index - 1U].residual_vz_ms),
          "V6 residual descent speed did not grow with resume delay");
      CHECK_MSG(
          measurements[index].post_height_loss_m >
              measurements[index - 1U].post_height_loss_m,
          "V6 post-resume height loss did not grow with resume delay");
    }
  });

  std::cout << "\n" << (test_count - failure_count) << "/" << test_count
            << " hard native attitude SIL cases passed; "
            << (report_count - report_failure_count) << "/" << report_count
            << " report sections completed\n";
  return failure_count + report_failure_count;
}
