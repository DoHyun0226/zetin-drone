#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dual_imu_cascade_flow_hold.ino"

// dual_imu_cascade_flow_hold 를 호스트에서 폐루프로 돌린다. 플랜트는 자세 SIL 과
// 같은 소형 쿼드 추정 파라미터에 수평 병진을 더했다. 광류 센서는 "광축이
// 지난 프레임에 맞힌 지면점이 이번 프레임에 기체 좌표에서 얼마나 벗어났는가"를
// 기하로 계산한다 — 계획 §2 의 시선 각속도 식을 쓰지 않으므로, 그 식에서 나온
// 펌웨어 부호가 틀리면 이 시험이 잡는다. 센서 축은 기체 축과 90° 돌고 한 축이
// 뒤집힌 장착으로 둔다(보정이 그것을 찾아야 한다).

namespace {

constexpr double kDt = 0.001;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kG = 9.81;
constexpr double kCountsPerRad = 9.5 * kRadToDeg;   // Matek 9~10 counts/deg

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
  do { if (!(expression)) fail(#expression, __LINE__); } while (false)
#define CHECK_MSG(expression, detail) \
  do { if (!(expression)) fail(#expression, __LINE__, (detail)); } while (false)

std::string fmt(double value) {
  std::ostringstream s;
  s << value;
  return s.str();
}

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

// ---------------- 플랜트 ----------------

struct Params {
  double arm_m = 0.12;
  double ix = 0.003, iy = 0.003, iz = 0.006;
  double thrust_per_us = 0.0025;
  double mass = 4.0 * 0.0025 * (1340.0 - 1000.0) / kG;   // 1340 us 호버
  double drag = 0.05;                                     // N/(m/s)
  double arm_proj() const { return arm_m / std::sqrt(2.0); }
  double yaw_arm() const { return 0.06 * arm_proj(); }
};

struct State {
  double phi = 0, theta = 0, psi = 0;
  double p = 0, q = 0, r = 0;
  double n = 0, e = 0, h = 0.5;        // 북·동·높이(위 +)
  double vn = 0, ve = 0, vh = 0;
};

// 기체→월드(NED) 회전 행렬 R = Rz(ψ)·Ry(θ)·Rx(φ)
std::array<std::array<double, 3>, 3> rotation(const State &s) {
  const double cf = std::cos(s.phi), sf = std::sin(s.phi);
  const double ct = std::cos(s.theta), st = std::sin(s.theta);
  const double cp = std::cos(s.psi), sp = std::sin(s.psi);
  return {{{ct * cp, sf * st * cp - cf * sp, cf * st * cp + sf * sp},
           {ct * sp, sf * st * sp + cf * cp, cf * st * sp - sf * cp},
           {-st, sf * ct, cf * ct}}};
}

std::array<double, 3> toBody(const std::array<std::array<double, 3>, 3> &R,
                             const std::array<double, 3> &w) {
  return {R[0][0] * w[0] + R[1][0] * w[1] + R[2][0] * w[2],
          R[0][1] * w[0] + R[1][1] * w[1] + R[2][1] * w[2],
          R[0][2] * w[0] + R[1][2] * w[1] + R[2][2] * w[2]};
}

struct Disturbance {
  double force_n = 0, force_e = 0, force_up = 0;   // N
};

// 한 tick 적분. 반환은 월드 NED 가속도(가속도계 주입용).
std::array<double, 3> integrate(State &s, const Params &P, const Disturbance &d) {
  std::array<double, 4> thrust;
  for (int i = 0; i < 4; i++) {
    thrust[i] = P.thrust_per_us * std::max(0.0, (double)motorOut[i] - 1000.0);
  }
  const double a = P.arm_proj();
  const double tx = a * (thrust[0] - thrust[1] - thrust[2] + thrust[3]);
  const double ty = a * (-thrust[0] + thrust[1] - thrust[2] + thrust[3]);
  // 믹서는 +yaw 에 M3/M4 를 올린다. 실기체는 그 출력으로 yaw rate 가 감쇠하므로
  // (비행에서 확인) +yaw 출력이 +r 을 만드는 극성이어야 한다.
  const double tz = P.yaw_arm() * (-thrust[0] - thrust[1] + thrust[2] + thrust[3]);
  const double T = thrust[0] + thrust[1] + thrust[2] + thrust[3];

  const auto R = rotation(s);
  // 추력은 기체 −z. NED 가속도 = R·(0,0,−T)/m + g + (외란 − 항력)/m
  const double vd = -s.vh;
  std::array<double, 3> acc = {
      -R[0][2] * T / P.mass + (d.force_n - P.drag * s.vn) / P.mass,
      -R[1][2] * T / P.mass + (d.force_e - P.drag * s.ve) / P.mass,
      -R[2][2] * T / P.mass + kG + (-d.force_up - P.drag * vd) / P.mass};
  s.vn += acc[0] * kDt;
  s.ve += acc[1] * kDt;
  s.vh += -acc[2] * kDt;
  s.n += s.vn * kDt;
  s.e += s.ve * kDt;
  s.h += s.vh * kDt;
  if (s.h < 0.0) {
    // 지면에 닿으면 수평으로 미끄러지지 않는다(마찰). 없으면 착지 뒤 외란에
    // 끝없이 밀려 "위치 유지 폭주"처럼 보인다.
    s.h = 0.0;
    s.vh = std::max(0.0, s.vh);
    s.vn = s.ve = 0.0;
  }

  s.phi += s.p * kDt;
  s.theta += s.q * kDt;
  s.psi += s.r * kDt;
  s.p += tx / P.ix * kDt;
  s.q += ty / P.iy * kDt;
  s.r += tz / P.iz * kDt;
  return acc;
}

int16_t raw16(double v) {
  v = std::max(-32768.0, std::min(32767.0, v));
  return (int16_t)std::lround(v);
}

// 자세 SIL 과 같은 센서→기체 매핑의 역. 가속도계는 비력 −(a − g)/g 를 기체로.
void injectImu(const State &s, const std::array<double, 3> &acc_ned, uint32_t tick) {
  const auto R = rotation(s);
  const std::array<double, 3> reading = toBody(
      R, {(0.0 - acc_ned[0]) / kG, (0.0 - acc_ned[1]) / kG, (kG - acc_ned[2]) / kG});
  inv_imu_sensor_event_t e1 = {};
  e1.gyro[0] = raw16(-s.q * kRadToDeg / GYRO_SCALE);
  e1.gyro[1] = raw16(s.p * kRadToDeg / GYRO_SCALE);
  e1.gyro[2] = raw16(-s.r * kRadToDeg / GYRO_SCALE);
  e1.accel[0] = raw16(-reading[1] / ACCEL_SCALE);
  e1.accel[1] = raw16(reading[0] / ACCEL_SCALE);
  e1.accel[2] = raw16(reading[2] / ACCEL_SCALE);
  const int toggle = (tick & 1U) ? -1 : 1;   // freeze 감시용 결정적 dither
  e1.gyro[2] = raw16(e1.gyro[2] + toggle);
  e1.accel[2] = raw16(e1.accel[2] + 2 * toggle);
  inv_imu_sensor_event_t e2 = {};
  for (int k = 0; k < 3; k++) {
    const int sign = IMU2_SIGN[k] < 0.0f ? -1 : 1;
    e2.gyro[k] = raw16(sign * (int)e1.gyro[k]);
    e2.accel[k] = raw16(sign * (int)e1.accel[k]);
  }
  IMU1.next_event = e1;
  IMU2.next_event = e2;
  IMU1.read_status = 0;
  IMU2.read_status = 0;
}

// ---------------- 3901-L0X 모델 ----------------

uint8_t crc8(const std::vector<uint8_t> &bytes, size_t from) {
  uint8_t crc = 0;
  for (size_t i = from; i < bytes.size(); i++) {
    crc ^= bytes[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

std::vector<uint8_t> mspFrame(uint16_t cmd, const std::vector<uint8_t> &payload,
                              bool corrupt) {
  std::vector<uint8_t> f = {'$', 'X', '<', 0,
                            (uint8_t)(cmd & 0xFF), (uint8_t)(cmd >> 8),
                            (uint8_t)(payload.size() & 0xFF),
                            (uint8_t)(payload.size() >> 8)};
  f.insert(f.end(), payload.begin(), payload.end());
  uint8_t crc = crc8(f, 3);
  if (corrupt) crc ^= 0x5A;
  f.push_back(crc);
  return f;
}

void putI32(std::vector<uint8_t> &v, int32_t x) {
  const uint32_t u = (uint32_t)x;
  for (int k = 0; k < 4; k++) v.push_back((uint8_t)((u >> (8 * k)) & 0xFF));
}

struct SensorModel {
  double delay_s = 0.0;
  int drop_every = 0;       // n 번째 광류 프레임마다 유실
  int corrupt_every = 0;    // n 번째 광류 프레임마다 CRC 손상
  uint32_t range_off_from = std::numeric_limits<uint32_t>::max();
  uint32_t range_off_to = 0;

  // 지난 경계에서 광축이 맞힌 지면점(NED)
  bool have_point = false;
  std::array<double, 3> point = {0, 0, 0};
  double carry_x = 0, carry_y = 0;
  int frame_index = 0;
  struct Pending { uint32_t tick; std::vector<uint8_t> bytes; };
  std::deque<Pending> pending;

  static std::array<double, 3> boresightHit(const State &s) {
    const auto R = rotation(s);
    const std::array<double, 3> b = {R[0][2], R[1][2], R[2][2]};   // 기체 +z(아래)
    const double t = s.h / std::max(1e-6, b[2]);
    return {s.n + t * b[0], s.e + t * b[1], s.h > 0 ? s.h : 0.0};   // [2]=지면까지 깊이 기록용
  }

  // 지면점의 기체 좌표 방향 → 광축에서 벗어난 각 (기체 x 방향, 기체 y 방향)
  static std::array<double, 2> offset(const State &s, const std::array<double, 3> &g) {
    const auto R = rotation(s);
    // 기체 위치(NED): (n, e, −h). 지면은 D = 0.
    const std::array<double, 3> rel = {g[0] - s.n, g[1] - s.e, 0.0 - (-s.h)};
    const auto bd = toBody(R, rel);
    return {std::atan2(bd[0], bd[2]), std::atan2(bd[1], bd[2])};
  }

  // 센서 장착: 센서 x = 기체 x 방향 벗어남, 센서 y = −(기체 y 방향 벗어남).
  // roll 은 기체 y 방향, pitch 는 기체 x 방향으로 벗어나게 하므로 M 은 반대각이다.
  static std::array<double, 2> sensorAxes(const std::array<double, 2> &off) {
    return {off[0], -off[1]};
  }

  void tick(uint32_t tick, const State &s) {
    // 광류: 100 ms 경계마다 한 구간. 센서 지연만큼 늦게 내보낸다.
    if (tick % 100 == 0) {
      if (have_point) {
        const auto ax = sensorAxes(offset(s, point));
        const double fx = ax[0] * kCountsPerRad + carry_x;
        const double fy = ax[1] * kCountsPerRad + carry_y;
        const int32_t ix = (int32_t)std::lround(fx), iy = (int32_t)std::lround(fy);
        carry_x = fx - ix;
        carry_y = fy - iy;
        frame_index++;
        std::vector<uint8_t> payload = {150};
        putI32(payload, ix);
        putI32(payload, iy);
        const bool drop = drop_every && frame_index % drop_every == 0;
        const bool corrupt = corrupt_every && frame_index % corrupt_every == 0;
        if (!drop) {
          pending.push_back({tick + (uint32_t)std::lround(delay_s * 1000.0),
                             mspFrame(0x1F02, payload, corrupt)});
        }
      }
      point = boresightHit(s);
      have_point = true;
    }
    // 거리: 35 ms 마다 광축 거리(mm). 범위 밖·끊김은 −1.
    if (tick % 35 == 0) {
      const auto R = rotation(s);
      const double slant = s.h / std::max(1e-6, R[2][2]);
      int32_t mm = (int32_t)std::lround(slant * 1000.0);
      if (mm < 80 || mm > 2000) mm = -1;
      if (tick >= range_off_from && tick < range_off_to) mm = -1;
      std::vector<uint8_t> payload = {255};
      putI32(payload, mm);
      pending.push_back({tick, mspFrame(0x1F01, payload, false)});
    }
    while (!pending.empty() && pending.front().tick <= tick) {
      for (uint8_t b : pending.front().bytes) arduino_fake::serial1_rx.push_back(b);
      pending.pop_front();
    }
    std::stable_sort(pending.begin(), pending.end(),
                     [](const Pending &a, const Pending &b) { return a.tick < b.tick; });
  }
};

// 순수 회전에 대한 이 센서 모델의 응답 M_true [counts/rad] 를 수치 미분으로 구한다.
std::array<std::array<float, 2>, 2> trueCalibration() {
  std::array<std::array<float, 2>, 2> M{};
  for (int axis = 0; axis < 2; axis++) {
    State s;
    s.h = 0.5;
    const auto g = SensorModel::boresightHit(s);
    const double eps = 1e-4;
    if (axis == 0) s.phi = eps; else s.theta = eps;
    const auto ax = SensorModel::sensorAxes(SensorModel::offset(s, g));
    M[0][axis] = (float)(ax[0] * kCountsPerRad / eps);
    M[1][axis] = (float)(ax[1] * kCountsPerRad / eps);
  }
  return M;
}

// ---------------- 펌웨어 하네스 ----------------

void sendUdp(const std::string &command) {
  wifi_udp_fake::incoming_packet = command;
  arduino_fake::stop_on_task_delay = true;
  try {
    udp_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
  }
  arduino_fake::stop_on_task_delay = false;
}

void resetFirmware(const State &s) {
  arduino_fake::reset();
  arduino_fake::stop_on_task_delay = false;
  wifi_udp_fake::reset();
  connectionEstablished = false;
  Kp_Angle_Roll = 6.0f; Kp_Angle_Pitch = 6.0f; Kp_Angle_Yaw = 3.0f;
  // 펌웨어 출하값과 같게 둔다(자세 SIL 의 0.005 가 아니다).
  Kp_Rate_Roll = 0.50f; Ki_Rate_Roll = 0.20f; Kd_Rate_Roll = 0.015f;
  Kp_Rate_Pitch = 0.50f; Ki_Rate_Pitch = 0.20f; Kd_Rate_Pitch = 0.015f;
  Kp_Rate_Yaw = 1.50f; Ki_Rate_Yaw = 0.05f; Kd_Rate_Yaw = 0.0f;
  base_throttle = 1000; min_throttle = 1050; max_throttle = 1300;
  safety_lock = true;
  safety_disarm_requested = safety_arm_requested = false;
  failsafe_resume_requested = false;
  fs_phase = FS_NONE;
  hover_est = 0.0f; hover_valid = false;
  calibration_ok = true;
  // 스케치의 트림 기본값은 실기체 값이다. SIL 기체는 트림 없이 수평이다.
  trim_roll = trim_pitch = 0.0f;
  targetAngleX = targetAngleY = targetAngleZ = 0.0f;
  targetYawRate = 0.0f;
  angleX = (float)(s.phi * kRadToDeg);
  angleY = (float)(s.theta * kRadToDeg);
  angleZ = (float)(s.psi * kRadToDeg);
  for (int i = 0; i < 4; i++) motorOut[i] = 1000;
  iTermRoll = iTermPitch = iTermYaw = 0.0f;
  for (int k = 0; k < 3; k++) { gyro_bias1[k] = gyro_bias2[k] = 0.0f; }
  accel_scale1 = accel_scale2 = 1.0f;
  lastRcMs = 0;
  fault_rc = fault_imu1 = fault_imu2 = fault_disagree = fault_attitude = false;
  active_imus = 2;
  imu1_frozen_now = imu2_frozen_now = imu_disagree_now = false;
  lastRcSeq = 0; rcSeqValid = false;
  mspSensor = MspSensorParser();
  resetFlowHoldForHarness();
  IMU1.next_event = {}; IMU2.next_event = {};
}

struct Run {
  State s;
  Params P;
  SensorModel sensor;
  std::function<Disturbance(uint32_t)> disturbance;
  std::function<void(uint32_t, State &)> kinematic;   // 자세 강제(보정 시험)
  std::function<void(uint32_t)> on_tick;
  uint32_t rc_disconnect_tick = std::numeric_limits<uint32_t>::max();
  // Core 0 정체 흉내: 이 구간에는 pollMspSensor 를 부르지 않아 바이트가 쌓인다.
  uint32_t stall_from = std::numeric_limits<uint32_t>::max(), stall_to = 0;
  double max_failsafe_rate_err = 0.0;
  // 기록
  double max_pos_err = 0, max_tilt_deg = 0;
  double n0 = 0, e0 = 0;
  bool all_finite = true;
  bool applied_during_failsafe = false;

  void arm(int throttle) {
    sendUdp("start");
    sendUdp("th " + std::to_string(throttle));
    runTicks(1);
    CHECK_MSG(!safety_lock, "arm request was not applied");
  }

  void runTicks(uint32_t ticks) {
    uint32_t base_tick = arduino_fake::tick_index;
    (void)base_tick;
    std::array<double, 3> acc = {0, 0, kG};
    arduino_fake::pre_tick_hook = [&](uint32_t local) {
      const uint32_t tick = globalTick++;
      (void)local;
      if (kinematic) {
        kinematic(tick, s);
        acc = {0, 0, 0};
      } else if (tick > 0) {
        acc = integrate(s, P, disturbance ? disturbance(tick) : Disturbance{});
      }
      arduino_fake::millis_value += 1U;
      arduino_fake::micros_value += 1000U;
      if (tick < rc_disconnect_tick) lastRcMs = millis();
      sensor.tick(tick, s);
      if (!(tick >= stall_from && tick < stall_to)) {
        pollMspSensor(millis());        // Core 0 폴링을 1 kHz 로 흉내낸다
      }
      injectImu(s, acc, tick);
      if (on_tick) on_tick(tick);
      const double dn = s.n - n0, de = s.e - e0;
      max_pos_err = std::max(max_pos_err, std::sqrt(dn * dn + de * de));
      max_tilt_deg = std::max({max_tilt_deg, std::fabs(s.phi) * kRadToDeg,
                               std::fabs(s.theta) * kRadToDeg});
      if (!std::isfinite(s.n + s.e + s.h + s.phi + s.theta)) all_finite = false;
      if (fs_phase != FS_NONE && !safety_lock) {
        // 직전 tick 의 목표 각속도가 위치 유지 보정 없이 계산됐는지(outer 분주 잔류 없음)
        const double expect = std::max(-300.0, std::min(300.0,
            ((double)targetAngleY - (double)angleY) * Kp_Angle_Pitch));
        max_failsafe_rate_err =
            std::max(max_failsafe_rate_err, std::fabs((double)tgtRate[1] - expect));
      }
      if (fs_phase != FS_NONE) {
        const FlowHoldTelemetry t = readFlowHoldTelemetry();
        if (t.applied_roll_deg != 0.0f || t.applied_pitch_deg != 0.0f ||
            t.applied_alt_us != 0.0f) {
          applied_during_failsafe = true;
        }
      }
    };
    arduino_fake::tick_index = 0;
    arduino_fake::tick_limit = ticks;
    try {
      pid_task(nullptr);
    } catch (const arduino_fake::TaskDelayExit &) {
    }
    arduino_fake::pre_tick_hook = nullptr;
    arduino_fake::tick_limit = 0;
  }
  uint32_t globalTick = 0;

  double speed() const { return std::sqrt(s.vn * s.vn + s.ve * s.ve); }
};

void setTrueCalibration(uint32_t delay_us, double sign = 1.0) {
  const auto Mt = trueCalibration();
  float M[2][2] = {{(float)(sign * Mt[0][0]), (float)(sign * Mt[0][1])},
                   {(float)(sign * Mt[1][0]), (float)(sign * Mt[1][1])}};
  CHECK(flowOdomSetCalibration(flowOdomCfg, M, delay_us));
}

// 드리프트 호버: 기체 좌표로 vn 초기 속도와 북쪽 외란(기울기 상당)을 준다.
Run hoverRun(double height, double v0, double force_tilt_deg) {
  Run run;
  run.s.h = height;
  run.s.vn = v0;
  resetFirmware(run.s);
  const double force = run.P.mass * kG * std::tan(force_tilt_deg * kDegToRad);
  run.disturbance = [force](uint32_t) {
    Disturbance d;
    d.force_n = force;
    return d;
  };
  return run;
}

}  // namespace

int main() {
  runCase("true calibration of the sensor model is a signed permutation", [] {
    const auto M = trueCalibration();
    CHECK(std::fabs(M[0][0]) < 0.01 * kCountsPerRad);
    CHECK(std::fabs(std::fabs(M[0][1]) - kCountsPerRad) < 0.01 * kCountsPerRad);
    CHECK(std::fabs(std::fabs(M[1][0]) - kCountsPerRad) < 0.01 * kCountsPerRad);
    CHECK(std::fabs(M[1][1]) < 0.01 * kCountsPerRad);
  });

  runCase("S0: poshold off, armed hover stays level and finite", [] {
    Run run = hoverRun(0.5, 0.0, 0.0);
    run.arm(1340);
    run.runTicks(3000);
    CHECK(run.all_finite);
    CHECK_MSG(run.max_tilt_deg < 2.0, fmt(run.max_tilt_deg));
  });

  runCase("flowcal: hand-shaking roll then pitch recovers M and delay", [] {
    Run run;
    run.s.h = 0.5;
    resetFirmware(run.s);
    run.sensor.delay_s = 0.020;
    // 0~12 s roll, 12~24 s pitch, 24~30 s 두 축 다른 주파수로. 위치·높이는 고정.
    run.kinematic = [](uint32_t tick, State &s) {
      const double t = tick * kDt;
      const double w1 = 2 * kPi * 0.8, w2 = 2 * kPi * 0.55;
      double roll = 0, pitch = 0, p = 0, q = 0;
      if (t < 12) { roll = 0.25 * std::sin(w1 * t); p = 0.25 * w1 * std::cos(w1 * t); }
      else if (t < 24) { pitch = 0.25 * std::sin(w1 * t); q = 0.25 * w1 * std::cos(w1 * t); }
      else {
        roll = 0.2 * std::sin(w1 * t); p = 0.2 * w1 * std::cos(w1 * t);
        pitch = 0.2 * std::sin(w2 * t); q = 0.2 * w2 * std::cos(w2 * t);
      }
      s.phi = roll; s.theta = pitch; s.p = p; s.q = q; s.r = 0;
      s.n = s.e = 0; s.h = 0.5; s.vn = s.ve = s.vh = 0;
    };
    sendUdp("flowcal 1");
    run.runTicks(30000);
    sendUdp("flowcal 0");
    run.runTicks(2);
    const FlowCalResult r = lastFlowCalResultForHarness();
    CHECK_MSG(r.ok, "reason=" + fmt(r.reason) + " residual=" + fmt(r.residual) +
                        " cond=" + fmt(r.cond));
    CHECK(flowOdomCfg.calibrated);
    const auto Mt = trueCalibration();
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        CHECK_MSG(std::fabs(r.m[i][j] - Mt[i][j]) < 0.05 * kCountsPerRad,
                  "m" + fmt(i) + fmt(j) + "=" + fmt(r.m[i][j]) + " want " + fmt(Mt[i][j]));
      }
    }
    CHECK_MSG(r.delay_us >= 10000 && r.delay_us <= 30000, fmt(r.delay_us));
  });

  runCase("flowcal is refused while armed and blocks start while running", [] {
    Run run = hoverRun(0.5, 0.0, 0.0);
    sendUdp("flowcal 1");
    CHECK(flowCalRunningForHarness());
    sendUdp("start");
    CHECK(safety_lock && !safety_arm_requested);
    sendUdp("flowcal 0");
    run.runTicks(2);
    CHECK(!flowCalRunningForHarness());
    run.arm(1340);
    sendUdp("flowcal 1");
    CHECK(!flowCalRunningForHarness());
  });

  runCase("poshold is refused without a calibration", [] {
    Run run = hoverRun(0.5, 0.0, 0.0);
    // 스케치에 보정 상수가 들어 있어도 미보정 경로를 시험한다.
    flowOdomCfg = flowOdomDefaultConfig();
    sendUdp("poshold 1");
    CHECK(!poshold_requested);
  });

  // IMU 보정: 부팅 때는 저장값(NVS > 컴파일 값)을 쓰고, `imucal` 명령 때만 정지 보정한다.
  runCase("boot loads the compiled IMU calibration when NVS is empty", [] {
    preferences_fake::reset();
    for (int k = 0; k < 3; k++) gyro_bias1[k] = gyro_bias2[k] = 99.0f;
    CHECK(STORED_IMU_CAL_VALID);
    CHECK(loadImuCalibration());
    for (int k = 0; k < 3; k++) {
      CHECK(gyro_bias1[k] == STORED_GYRO_BIAS1[k]);
      CHECK(gyro_bias2[k] == STORED_GYRO_BIAS2[k]);
    }
    CHECK(accel_scale1 == STORED_ACCEL_SCALE1 && accel_scale2 == STORED_ACCEL_SCALE2);
  });

  runCase("an IMU calibration saved to NVS wins over the compiled one", [] {
    preferences_fake::reset();
    const float b1[3] = {0.1f, 0.2f, 0.3f}, b2[3] = {-0.1f, -0.2f, -0.3f};
    CHECK(saveImuCalibration(b1, b2, 1.01f, 0.99f));
    CHECK(loadImuCalibration());
    for (int k = 0; k < 3; k++) {
      CHECK(gyro_bias1[k] == b1[k]);
      CHECK(gyro_bias2[k] == b2[k]);
    }
    CHECK_MSG(std::fabs(accel_scale1 - 1.01f) < 1e-6f && std::fabs(accel_scale2 - 0.99f) < 1e-6f,
              "s1=" + fmt(accel_scale1) + " s2=" + fmt(accel_scale2));
  });

  runCase("an implausible NVS IMU calibration is ignored", [] {
    preferences_fake::reset();
    const float b1[3] = {50.0f, 0.0f, 0.0f}, b2[3] = {0.0f, 0.0f, 0.0f};
    CHECK(saveImuCalibration(b1, b2, 1.0f, 1.0f));
    CHECK(loadImuCalibration());
    CHECK(gyro_bias1[0] == STORED_GYRO_BIAS1[0]);
  });

  runCase("imucal measures a still drone, applies and saves the result", [] {
    preferences_fake::reset();
    Run run = hoverRun(0.5, 0.0, 0.0);
    run.kinematic = [](uint32_t, State &) {};   // 바닥에 정지(1 g, 회전 없음)
    for (int k = 0; k < 3; k++) gyro_bias1[k] = gyro_bias2[k] = 5.0f;
    calibration_ok = false;
    sendUdp("imucal");
    CHECK(imuCalRunningForHarness());
    sendUdp("start");
    CHECK_MSG(safety_lock && !safety_arm_requested, "start must be refused during imucal");
    run.runTicks(IMU_CAL_SAMPLES + 50);
    sendUdp("gains");   // Core 0 한 바퀴: 결과 출력·NVS 저장
    CHECK(!imuCalRunningForHarness());
    const ImuCalResult r = lastImuCalResultForHarness();
    CHECK_MSG(r.ok, "reason=" + fmt(r.reason));
    for (int k = 0; k < 3; k++) {
      CHECK_MSG(std::fabs(gyro_bias1[k]) < 0.1f, "b1[" + fmt(k) + "]=" + fmt(gyro_bias1[k]));
      CHECK_MSG(std::fabs(gyro_bias2[k]) < 0.1f, "b2[" + fmt(k) + "]=" + fmt(gyro_bias2[k]));
    }
    CHECK(calibration_ok);
    CHECK(arduino_fake::serial_output.find(">>> IMUCAL OK") != std::string::npos);
    for (int k = 0; k < 3; k++) gyro_bias1[k] = 7.0f;
    CHECK(loadImuCalibration());
    CHECK_MSG(std::fabs(gyro_bias1[0]) < 0.1f, "saved b1[0]=" + fmt(gyro_bias1[0]));
  });

  runCase("imucal rejects motion and keeps the previous calibration", [] {
    preferences_fake::reset();
    Run run = hoverRun(0.5, 0.0, 0.0);
    run.kinematic = [](uint32_t tick, State &st) {
      st.p = 0.3 * std::sin(2.0 * kPi * tick / 400.0);   // 약 ±17 dps
    };
    for (int k = 0; k < 3; k++) gyro_bias1[k] = gyro_bias2[k] = 0.5f;
    sendUdp("imucal");
    run.runTicks(IMU_CAL_SAMPLES + 50);
    sendUdp("gains");
    const ImuCalResult r = lastImuCalResultForHarness();
    CHECK(!r.ok && r.reason == IMUCAL_MOVED);
    for (int k = 0; k < 3; k++) CHECK(gyro_bias1[k] == 0.5f && gyro_bias2[k] == 0.5f);
    CHECK(preferences_fake::store.empty());
    CHECK(arduino_fake::serial_output.find(">>> IMUCAL MOVED") != std::string::npos);
  });

  runCase("imucal is refused while armed", [] {
    Run run = hoverRun(0.5, 0.0, 0.0);
    run.arm(1340);
    sendUdp("imucal");
    CHECK(!imuCalRunningForHarness());
  });

  runCase("the compiled calibration is applied at reset", [] {
    resetFlowHoldForHarness();
    CHECK(flowOdomCfg.calibrated == FLOW_CAL_VALID);
  });

  // yaw=0 대칭 조건만 쓰면 yaw 토크 극성이 거꾸로여도 드러나지 않는다(Codex 교차검증).
  // 실기체는 비행에서 yaw rate 감쇠가 확인됐으므로 SIL 도 감쇠해야 한다.
  for (double r0_dps : {5.0, -5.0}) {
    runCase("yaw rate loop damps an initial " + fmt(r0_dps) + " deg/s yaw rate", [r0_dps] {
      Run run = hoverRun(0.5, 0.0, 0.0);
      run.arm(1340);
      run.s.r = r0_dps / kRadToDeg;
      run.runTicks(1000);
      CHECK(run.all_finite);
      CHECK_MSG(std::fabs(run.s.r * kRadToDeg) < 1.0,
                "r=" + fmt(run.s.r * kRadToDeg) + " deg/s");
    });
  }

  runCase("poshold holds through a 20 deg/s yaw kick", [] {
    Run run = hoverRun(0.5, 0.3, 1.0);
    setTrueCalibration(0);
    run.arm(1340);
    sendUdp("poshold 1");
    run.runTicks(2000);
    run.s.r = 20.0 / kRadToDeg;
    run.runTicks(10000);
    CHECK(run.all_finite);
    CHECK_MSG(std::fabs(run.s.r * kRadToDeg) < 1.0, "r=" + fmt(run.s.r * kRadToDeg));
    CHECK_MSG(run.speed() < 0.05, "speed=" + fmt(run.speed()));
    CHECK_MSG(run.max_tilt_deg < 10.0, fmt(run.max_tilt_deg));
    CHECK_MSG(run.s.h > 0.25, "h=" + fmt(run.s.h));
  });

  double positiveMaxErr = 0.0;
  for (double drag : {0.05, 0.30}) {
    runCase("S1: 0.5 m, 0.3 m/s drift + 1 deg bias is stopped and held, drag " +
                fmt(drag), [&, drag] {
      Run run = hoverRun(0.5, 0.3, 1.0);
      run.P.drag = drag;
      setTrueCalibration(0);
      run.arm(1340);
      sendUdp("poshold 1");
      CHECK(poshold_requested);
      run.runTicks(12000);
      CHECK(run.all_finite);
      CHECK_MSG(run.speed() < 0.05, "speed=" + fmt(run.speed()));
      CHECK_MSG(run.max_pos_err < 0.5, "max_pos_err=" + fmt(run.max_pos_err));
      CHECK_MSG(run.max_tilt_deg < 10.0, fmt(run.max_tilt_deg));
      // 착지해서 멈춘 것을 수렴으로 읽지 않는다(재검토 코드 P2-9).
      CHECK_MSG(run.s.h > 0.25, "h=" + fmt(run.s.h));
      CHECK(!safety_lock);
      CHECK_MSG(readFlowHoldTelemetry().poshold_state == POSHOLD_ACTIVE ||
                    readFlowHoldTelemetry().poshold_state == POSHOLD_COAST,
                "state=" + fmt(readFlowHoldTelemetry().poshold_state));
      positiveMaxErr = std::max(positiveMaxErr, run.max_pos_err);
    });
  }

  runCase("negative control: flipped calibration sign makes it worse", [&] {
    Run run = hoverRun(0.5, 0.3, 1.0);
    setTrueCalibration(0, -1.0);
    run.arm(1340);
    sendUdp("poshold 1");
    run.runTicks(12000);
    CHECK_MSG(!run.all_finite || run.max_pos_err > 2.0 * positiveMaxErr + 0.3,
              "flipped max_pos_err=" + fmt(run.max_pos_err) +
                  " good=" + fmt(positiveMaxErr));
  });

  runCase("S2: 1.5 m with 30 ms sensor delay converges without oscillation", [] {
    Run run = hoverRun(1.5, 0.3, 1.0);
    run.sensor.delay_s = 0.030;
    setTrueCalibration(30000);
    run.arm(1340);
    sendUdp("poshold 1");
    run.runTicks(15000);
    CHECK(run.all_finite);
    CHECK_MSG(run.speed() < 0.05, "speed=" + fmt(run.speed()));
    CHECK_MSG(run.max_tilt_deg < 10.0, fmt(run.max_tilt_deg));
    CHECK_MSG(run.s.h > 1.0, "h=" + fmt(run.s.h));
    CHECK(!safety_lock);
    CHECK(readFlowHoldTelemetry().poshold_state == POSHOLD_ACTIVE ||
          readFlowHoldTelemetry().poshold_state == POSHOLD_COAST);
  });

  runCase("S3: 3.5 deg bias is absorbed by the integrator", [] {
    // 고정 스로틀로 3.5° 기울어 있으면 수직 추력이 줄어 7 s 에 0.5 m 를 내려온다.
    // 실제 호버처럼 고도 유지를 같이 켠다.
    Run run = hoverRun(0.5, 0.0, 3.5);
    setTrueCalibration(0);
    run.arm(1340);
    sendUdp("althold 1");
    sendUdp("poshold 1");
    run.runTicks(20000);
    CHECK(run.all_finite);
    CHECK_MSG(run.speed() < 0.05, "speed=" + fmt(run.speed()));
    CHECK_MSG(run.max_pos_err < 1.5, "max_pos_err=" + fmt(run.max_pos_err));
    CHECK_MSG(run.s.h > 0.3, "h=" + fmt(run.s.h));
  });

  runCase("23% lost/corrupted flow frames: degraded but no runaway", [] {
    // 7번째마다 유실, 11번째마다 CRC 손상. 하나가 빠지면 마지막 주기로 속도를,
    // 간격 전체로 위치를 적분한다. 둘이 붙어 빠진 구간은 버리므로 위치는 조금씩
    // 새지만(유실 없음 0.09 m → 이 조건 약 1 m) 속도가 커지거나 발산하지 않아야 한다.
    Run run = hoverRun(0.5, 0.3, 1.0);
    run.sensor.drop_every = 7;
    run.sensor.corrupt_every = 11;
    setTrueCalibration(0);
    run.arm(1340);
    sendUdp("althold 1");
    sendUdp("poshold 1");
    double maxLateSpeed = 0.0;
    run.on_tick = [&](uint32_t tick) {
      if (tick > 8000) maxLateSpeed = std::max(maxLateSpeed, run.speed());
    };
    run.runTicks(20000);
    CHECK(run.all_finite);
    CHECK_MSG(maxLateSpeed < 0.15, "max late speed=" + fmt(maxLateSpeed));
    CHECK_MSG(run.max_pos_err < 1.5, "max_pos_err=" + fmt(run.max_pos_err));
  });

  runCase("RC loss: applied hold is zero from the failsafe tick on", [] {
    // 3.5° 외란으로 보정이 3.6° 쯤 걸린 상태에서 링크를 끊는다. outer loop 가
    // 250 Hz 로 분주되므로 끊는 위상 4개를 다 본다(위상에 따라 잔류가 숨는다).
    for (uint32_t phase : {100u, 101u, 102u, 103u}) {
      Run run = hoverRun(0.6, 0.0, 3.5);
      setTrueCalibration(0);
      run.arm(1340);
      sendUdp("poshold 1");
      sendUdp("althold 1");
      run.runTicks(6000);                 // hover_est 가 유효해지고 보정이 쌓인다
      CHECK(std::fabs(readFlowHoldTelemetry().applied_pitch_deg) > 2.0f);
      run.rc_disconnect_tick = run.globalTick + phase;
      run.runTicks(1500);
      CHECK(fs_phase != FS_NONE);
      CHECK(!run.applied_during_failsafe);
      CHECK_MSG(run.max_failsafe_rate_err < 3.0,
                "phase " + fmt(phase) + " target-rate residual=" +
                    fmt(run.max_failsafe_rate_err));
      CHECK(!poshold_requested && !althold_requested);
    }
  });

  runCase("IMU loss then re-arm does not resurrect the old correction", [] {
    Run run = hoverRun(0.5, 0.0, 3.5);
    setTrueCalibration(0);
    run.arm(1340);
    sendUdp("althold 1");
    sendUdp("poshold 1");
    run.runTicks(8000);                       // 적분이 수 도 쌓인다
    CHECK(std::fabs(posHold.i[1]) > 1.0f);
    fault_imu1 = true;                        // 두 IMU 신뢰 상실 → 즉시 잠금 경로
    fault_imu2 = true;
    run.runTicks(5);
    CHECK(safety_lock);
    CHECK(flowGyro.count == 0);               // 공백을 보간하지 않도록 이력이 지워졌다
    CHECK(!flowOdomState.have_prev);
    run.s = State();                          // 다시 세워 놓고 재무장
    run.s.h = 0.5;
    run.disturbance = nullptr;
    run.arm(1340);
    double maxApplied = 0.0;
    run.on_tick = [&](uint32_t) {
      const FlowHoldTelemetry t = readFlowHoldTelemetry();
      maxApplied = std::max({maxApplied, std::fabs((double)t.applied_roll_deg),
                             std::fabs((double)t.applied_pitch_deg)});
    };
    run.runTicks(60);
    CHECK_MSG(maxApplied < 0.7, "applied right after re-arm=" + fmt(maxApplied));
  });

  runCase("althold adds nothing while base throttle is at the ground-cut line", [] {
    Run run = hoverRun(0.5, 0.0, 0.0);
    run.arm(1340);
    sendUdp("althold 1");
    const uint32_t start = run.globalTick;
    run.disturbance = [start](uint32_t tick) {
      Disturbance d;
      if (tick >= start + 500) d.force_up = -0.15;  // 약 15 us 상당, 위로 보정을 쌓게 한다
      return d;
    };
    run.runTicks(6000);
    CHECK(readFlowHoldTelemetry().applied_alt_us > 5.0f);
    sendUdp("th " + std::to_string(FS_GROUND_CUT_MAX_US));
    // 훅은 pid tick 앞에서 돈다. 명령 직후 첫 관측은 명령 이전 tick 의 텔레메트리다.
    bool violated = false;
    int seen = 0;
    run.on_tick = [&](uint32_t) {
      if (seen++ == 0) return;
      if (readFlowHoldTelemetry().applied_alt_us != 0.0f) violated = true;
    };
    run.runTicks(500);
    CHECK(base_throttle <= FS_GROUND_CUT_MAX_US);
    CHECK(!violated);
  });

  runCase("a Core 0 stall does not feed stale range frames into vz", [] {
    // 수평 자세로 1 m/s 등속 상승(기구학). 정체가 없으면 vz 추정은 1.0 에 붙는다.
    // 쌓였던 프레임을 현재 시각으로 차분하면 그 근처에서 vz 가 튄다(수정 전 코드로
    // 돌리면 0.1 m/s 이상, 수정 후 0.03 미만). 정체 직후 첫 폴링뿐 아니라 바이트 예산
    // 때문에 남은 backlog 를 꺼내는 다음 폴링도 걸러야 한다.
    double worst = 0.0;
    for (uint32_t stall : {40u, 70u, 100u, 130u}) {
      for (uint32_t phase : {0u, 10u, 20u, 30u}) {
        Run run;
        run.s.h = 0.2;
        resetFirmware(run.s);
        run.kinematic = [](uint32_t tick, State &s) {
          s = State();
          s.h = 0.2 + 1.0 * tick * kDt;
          s.vh = 1.0;
        };
        run.stall_from = 900 + phase;
        run.stall_to = run.stall_from + stall;
        double maxErr = 0.0;
        run.on_tick = [&](uint32_t tick) {
          // 700 ms 이후: vz LPF(τ 0.15 s)가 시작 과도에서 충분히 정착한 뒤
          if (tick >= 700 && tick < 1450 && heightEst.valid) {
            maxErr = std::max(maxErr, std::fabs((double)heightEst.vz_ms - 1.0));
          }
        };
        run.runTicks(1450);
        worst = std::max(worst, maxErr);
      }
    }
    CHECK_MSG(worst < 0.03, "worst vz error around a stall=" + fmt(worst));
  });

  runCase("althold: a vertical push is recovered within 0.1 m", [] {
    Run run = hoverRun(0.5, 0.0, 0.0);
    run.arm(1340);
    sendUdp("althold 1");
    run.runTicks(1000);
    const uint32_t start = run.globalTick;
    run.disturbance = [start](uint32_t tick) {
      Disturbance d;
      if (tick >= start + 1000) d.force_up = -0.2;   // 약 20 us 상당
      return d;
    };
    run.runTicks(12000);
    CHECK(run.all_finite);
    CHECK_MSG(std::fabs(run.s.h - 0.5) < 0.1, "h=" + fmt(run.s.h));
    CHECK_MSG(std::fabs(run.s.vh) < 0.05, "vh=" + fmt(run.s.vh));
  });

  runCase("althold: throttle +30 us then hold settles at a new height", [] {
    Run run = hoverRun(0.5, 0.0, 0.0);
    run.arm(1340);
    sendUdp("althold 1");
    run.runTicks(1000);
    sendUdp("th 1370");
    run.runTicks(12000);
    CHECK(run.all_finite);
    CHECK_MSG(std::fabs(run.s.vh) < 0.05, "vh=" + fmt(run.s.vh));
    const double h1 = run.s.h;
    run.runTicks(4000);
    CHECK_MSG(std::fabs(run.s.h - h1) < 0.05, "h1=" + fmt(h1) + " h2=" + fmt(run.s.h));
    CHECK_MSG(run.s.h < 1.5, "h=" + fmt(run.s.h));
  });

  runCase("althold: a 350 ms range dropout costs less than 0.1 m", [] {
    Run run = hoverRun(0.5, 0.0, 0.0);
    run.arm(1340);
    sendUdp("althold 1");
    const uint32_t start = run.globalTick;
    // 1 s 뒤부터 −20 us 상당 하강 외란을 학습시킨 뒤 거리 끊김
    run.disturbance = [start](uint32_t tick) {
      Disturbance d;
      if (tick >= start + 1000) d.force_up = -0.2;
      return d;
    };
    run.runTicks(10000);
    const double before = run.s.h;
    run.sensor.range_off_from = run.globalTick;
    run.sensor.range_off_to = run.globalTick + 350;
    double minH = before;
    run.on_tick = [&](uint32_t) { minH = std::min(minH, run.s.h); };
    run.runTicks(2000);
    CHECK_MSG(before - minH < 0.1, "drop=" + fmt(before - minH));
  });

  std::cout << test_count - failure_count << "/" << test_count << " passed\n";
  return failure_count ? 1 : 0;
}
