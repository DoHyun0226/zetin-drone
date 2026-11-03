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
  // 배터리 처짐: tick → 추력 배율(없으면 1)
  std::function<double(uint32_t)> thrust_scale_at;
  // 바닥 근처 추력 증가: h = 0 에서 +gain, 0.2 m 에서 0 으로 선형
  double ground_effect_gain = 0.0;
  // 테더 줄: tick → 이 높이 아래로 내려가지 않는다(없으면 지면만)
  std::function<double(uint32_t)> tether_floor;
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
  double torque_roll = 0;                          // N·m (자세 외란)
};

// 한 tick 적분. 반환은 월드 NED 가속도(가속도계 주입용).
std::array<double, 3> integrate(State &s, const Params &P, const Disturbance &d,
                                uint32_t tick) {
  double scale = P.thrust_scale_at ? P.thrust_scale_at(tick) : 1.0;
  if (P.ground_effect_gain != 0.0) {
    scale *= 1.0 + P.ground_effect_gain * std::max(0.0, 1.0 - s.h / 0.2);
  }
  std::array<double, 4> thrust;
  for (int i = 0; i < 4; i++) {
    thrust[i] = scale * P.thrust_per_us * std::max(0.0, (double)motorOut[i] - 1000.0);
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
    acc[2] = std::min(acc[2], 0.0);   // 지면이 받친다: 가속도계는 정지 1 g 를 본다
  }
  if (P.tether_floor) {
    const double floor_h = P.tether_floor(tick);
    if (s.h < floor_h) {
      s.h = floor_h;
      s.vh = std::max(0.0, s.vh);
      acc[2] = std::min(acc[2], 0.0);   // 줄이 받친다
    }
  }

  s.phi += s.p * kDt;
  s.theta += s.q * kDt;
  s.psi += s.r * kDt;
  s.p += (tx + d.torque_roll) / P.ix * kDt;
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
  // 센서가 착륙장치 바닥보다 이만큼 위에 있다(실측 착지 원값 중앙값 약 30 mm). 플랜트
  // 높이 s.h 는 착륙장치 바닥이고, 거리·광류 기하는 모두 센서 높이 s.h + 이 값으로 계산한다.
  int ground_offset_mm = 0;
  // 참이면 80 mm 미만도 보고한다(실기 3901-L0X 는 착지 상태 18–37 mm 를 보고).
  bool report_short = false;
  // 결정적 의사난수 ±noise_mm
  int noise_mm = 0;
  uint32_t noise_state = 0x1234567u;
  // (tick, 모델 원값) → 보낼 원값. 근접 물체·끊김 주입용.
  std::function<int32_t(uint32_t, int32_t)> range_override;

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

  void tick(uint32_t tick, const State &plant) {
    State s = plant;                       // 센서 원점
    s.h += ground_offset_mm * 0.001;
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
      if (noise_mm > 0) {
        noise_state = noise_state * 1664525u + 1013904223u;
        mm += (int32_t)((noise_state >> 16) % (uint32_t)(2 * noise_mm + 1)) - noise_mm;
      }
      if ((report_short ? mm <= 0 : mm < 80) || mm > 2000) mm = -1;
      if (tick >= range_off_from && tick < range_off_to) mm = -1;
      if (range_override) mm = range_override(tick, mm);
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
        acc = integrate(s, P, disturbance ? disturbance(tick) : Disturbance{}, tick);
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

// 자동착륙 진입 직전에 호버 추정을 편향시킨다(추정이 틀린 경우를 흉내). 진입 뒤에는
// fs_phase 가 FS_NONE 이 아니라 추정이 더 갱신되지 않는다.
void forceHoverEstimate(float us) {
  hoverTracker.estimate_us = us;
  hover_est = us;
}

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

// ---------------- 거리 기반 자동착륙 ----------------

// 호버 중 링크를 끊고 끝까지 지켜본다. 관측은 pid tick 앞(hook)이라 전역 값은 직전
// tick 의 결과다.
struct LandWatch {
  uint32_t disconnect_tick = 0, entry_tick = 0, end_tick = 0, near_tick = 0;
  uint32_t fallback_tick = 0, touch_tick = 0;
  uint8_t end_phase = FS_NONE, cut_reason = RL_CUT_NONE;
  bool saw_range = false, saw_timed = false, saw_fallback = false;
  double h_at_end = -1.0, h_at_near = -1.0, vh_at_touch = 0.0, prev_vh = 0.0, h_at_fallback = -1.0;
  int prev_req = 0, req_before_fallback = 0, req_at_entry_before = 0, req_at_entry = 0;
  bool req_rose_after_near = false, req_rose_after_fallback = false;
  double min_h_near = 1e9, max_rise_near = 0.0, max_vh_up_near = 0.0;
  double max_tilt_deg = 0.0, max_horiz_dev = 0.0, n_entry = 0.0, e_entry = 0.0;
  float applied_at_end = 0.0f;
  bool applied_seen = false;
};

struct LandCase {
  Run run;
  LandWatch w;
  std::function<void(uint32_t)> extra;   // 사건 주입(링크·거리·외란·resume)

  explicit LandCase(double h0, double tilt_deg = 0.0) : run(hoverRun(h0, 0.0, tilt_deg)) {
    run.sensor.ground_offset_mm = 30;
    run.sensor.report_short = true;
    run.sensor.noise_mm = 8;
    setTrueCalibration(0);
  }

  void warmup(uint32_t ticks = 6000, bool pilot_althold = true) {
    run.arm(1340);
    if (pilot_althold) sendUdp("althold 1");
    sendUdp("poshold 1");
    run.runTicks(ticks);
  }

  // 한 번의 runTicks 안에서 링크를 끊고 끝까지 본다.
  void land(uint32_t ticks = 20000, uint32_t disconnect_after = 100) {
    w.disconnect_tick = run.globalTick + disconnect_after;
    run.rc_disconnect_tick = w.disconnect_tick;
    run.on_tick = [this](uint32_t tick) {
      observe(tick);
      if (extra) extra(tick);
    };
    run.runTicks(ticks);
  }

  void observe(uint32_t tick) {
    const State &s = run.s;
    const int req = rl_requested_collective_us;
    if (!w.entry_tick && fs_phase == FS_DESCENDING) {
      w.entry_tick = tick;
      w.req_at_entry_before = w.prev_req;
      w.req_at_entry = req;
      w.n_entry = s.n;
      w.e_entry = s.e;
    }
    const bool descending = fs_phase == FS_DESCENDING && !safety_lock;
    if (descending) {
      if (rl_mode == RL_MODE_RANGE) w.saw_range = true;
      if (rl_mode == RL_MODE_TIMED) w.saw_timed = true;
      w.max_tilt_deg = std::max({w.max_tilt_deg, std::fabs(s.phi) * kRadToDeg,
                                 std::fabs(s.theta) * kRadToDeg});
      w.max_horiz_dev = std::max(w.max_horiz_dev,
                                 std::hypot(s.n - w.n_entry, s.e - w.e_entry));
      const FlowHoldTelemetry t = readFlowHoldTelemetry();
      w.applied_at_end = std::max(std::fabs(t.applied_roll_deg), std::fabs(t.applied_pitch_deg));
      if (w.applied_at_end > 0.2f) w.applied_seen = true;
    }
    if (descending && rl_mode == RL_MODE_TIMED_FALLBACK && !w.fallback_tick) {
      w.fallback_tick = tick;
      w.h_at_fallback = s.h;
      w.saw_fallback = true;
      w.req_before_fallback = w.prev_req;
    }
    if (w.fallback_tick && descending && tick <= w.fallback_tick + 1000 &&
        req > w.req_before_fallback) {
      w.req_rose_after_fallback = true;
    }
    if (descending && rl_stage >= RL_STAGE_NEAR_GROUND) {
      if (!w.near_tick) {
        w.near_tick = tick;
        w.h_at_near = s.h;
      } else if (req > w.prev_req) {
        w.req_rose_after_near = true;
      }
      w.min_h_near = std::min(w.min_h_near, s.h);
      w.max_rise_near = std::max(w.max_rise_near, s.h - w.min_h_near);
      w.max_vh_up_near = std::max(w.max_vh_up_near, s.vh);
    }
    if (w.entry_tick && !w.touch_tick && s.h <= 0.0) {
      w.touch_tick = tick;
      w.vh_at_touch = w.prev_vh;
    }
    if (w.entry_tick && !w.end_tick && fs_phase != FS_DESCENDING) {
      w.end_tick = tick;
      w.end_phase = fs_phase;
      w.cut_reason = rl_cut_reason;
      w.h_at_end = s.h;
    }
    w.prev_vh = s.vh;
    w.prev_req = req;
  }
};

// RC 를 되살린다(그 뒤 다시 끊을 수 있게). 하네스는 hook 앞에서 lastRcMs 를 갱신하므로
// resume 은 다음 tick 부터 요청해야 RC 가 신선하다.
void restoreRc(Run &run) { run.rc_disconnect_tick = std::numeric_limits<uint32_t>::max(); }
void requestResume() { failsafe_resume_requested = true; }

}  // namespace

int main() {
  runCase("sil_sensor_reports_landed_range", [] {
    // 실측(2026-11-03): 착륙장치로 바닥에 선 기체의 원값 18–37 mm, 품질 255.
    Run run;
    run.s.h = 0.0;
    resetFirmware(run.s);
    run.sensor.ground_offset_mm = 30;
    run.sensor.report_short = true;
    run.sensor.noise_mm = 8;
    int32_t lo = 100000, hi = -100000;
    int frames = 0;
    uint32_t seen = 0;
    run.on_tick = [&](uint32_t) {
      FlowSensorSnapshot snap;
      portENTER_CRITICAL(&flowSensorMux);
      snap = flowSensorSnapshot;
      portEXIT_CRITICAL(&flowSensorMux);
      if (snap.range_seq != seen) {
        seen = snap.range_seq;
        frames++;
        lo = std::min(lo, snap.range_mm);
        hi = std::max(hi, snap.range_mm);
      }
    };
    run.runTicks(2000);
    CHECK_MSG(frames >= 50, "frames=" + fmt(frames));
    CHECK_MSG(lo >= 18 && hi <= 38, "lo=" + fmt(lo) + " hi=" + fmt(hi));
    CHECK_MSG(hi - lo >= 8, "noise spread=" + fmt(hi - lo));
  });

  runCase("sil_tether_floor_and_override_shape_the_range", [] {
    Run run;
    run.s.h = 0.3;
    resetFirmware(run.s);
    run.sensor.ground_offset_mm = 30;
    run.sensor.report_short = true;
    run.P.tether_floor = [](uint32_t) { return 0.05; };
    run.sensor.range_override = [](uint32_t tick, int32_t mm) {
      return tick >= 1000 && tick < 1100 ? (int32_t)30 : mm;
    };
    std::vector<int32_t> seen;
    uint32_t seq = 0;
    run.on_tick = [&](uint32_t) {
      FlowSensorSnapshot snap;
      portENTER_CRITICAL(&flowSensorMux);
      snap = flowSensorSnapshot;
      portEXIT_CRITICAL(&flowSensorMux);
      if (snap.range_seq != seq) {
        seq = snap.range_seq;
        seen.push_back(snap.range_mm);
      }
    };
    run.runTicks(1500);               // 시동 안 함: 떨어져 줄에 걸린다
    CHECK_MSG(std::fabs(run.s.h - 0.05) < 1e-9, "h=" + fmt(run.s.h));
    // 줄에 걸려 정지한 기체의 가속도계는 1 g 를 본다(지지력 반영).
    for (int i = 0; i < 4; i++) motorOut[i] = 1000;
    State hung = run.s;
    const auto acc = integrate(hung, run.P, Disturbance{}, 2000);
    CHECK_MSG(std::fabs(acc[2]) < 1e-9, "acc_down at rest on the tether=" + fmt(acc[2]));
    CHECK(std::count(seen.begin(), seen.end(), 30) >= 2);
    CHECK(seen.back() == 80);
  });

  runCase("sil_ground_offset_moves_the_sensor_origin_for_flow_too", [] {
    // 오프셋은 센서 높이다. 같은 수평 이동이 센서 높이 0.13 m 에서 보이는 각도는
    // 0.10 m 일 때의 0.10/0.13 배다.
    auto flowAngle = [](int offset_mm) {
      State so;
      so.h = 0.10 + offset_mm * 0.001;
      const auto g = SensorModel::boresightHit(so);
      so.n += 0.01;                    // 기체 1 cm 전진
      return SensorModel::offset(so, g)[0];
    };
    const double a0 = flowAngle(0), a30 = flowAngle(30);
    CHECK_MSG(std::fabs(a30 / a0 - 0.10 / 0.13) < 0.01, "ratio=" + fmt(a30 / a0));
    // 펌웨어가 받는 광류도 센서 높이 기준이다: 같은 이동에서 오프셋 30 mm 의 광류 크기는
    // 오프셋 0 의 약 0.77 배다.
    auto flowCounts = [](int offset_mm) {
      Run run;
      run.s.h = 0.10;
      resetFirmware(run.s);
      run.sensor.ground_offset_mm = offset_mm;
      run.sensor.report_short = true;
      run.kinematic = [](uint32_t tick, State &s) {
        s = State(); s.h = 0.10; s.n = 0.2 * tick * kDt; s.vn = 0.2;
      };
      run.runTicks(450);
      return std::make_pair((double)std::labs(msp_flow_x) + std::labs(msp_flow_y),
                            (int)msp_range_mm);
    };
    const auto f0 = flowCounts(0), f30 = flowCounts(30);
    CHECK_MSG(f0.first > 5.0, "flow counts=" + fmt(f0.first));
    CHECK_MSG(std::fabs(f30.first / f0.first - 0.10 / 0.13) < 0.08,
              "flow ratio=" + fmt(f30.first / f0.first));
    CHECK_MSG(std::abs(f30.second - 130) <= 1, "range=" + fmt(f30.second));
  });

  runCase("sil_thrust_scale_and_ground_effect_change_thrust", [] {
    State s;
    s.h = 0.5;
    Params P;
    for (int i = 0; i < 4; i++) motorOut[i] = 1340;
    const auto base = integrate(s, P, Disturbance{}, 0);
    s = State(); s.h = 0.5;
    P.thrust_scale_at = [](uint32_t) { return 0.9; };
    const auto sag = integrate(s, P, Disturbance{}, 0);
    CHECK(sag[2] > base[2] + 0.5);    // NED: 추력이 줄면 아래로 더 가속
    s = State(); s.h = 0.02;
    P.thrust_scale_at = nullptr;
    P.ground_effect_gain = 0.1;
    const auto ge = integrate(s, P, Disturbance{}, 0);
    CHECK(ge[2] < base[2] - 0.5);
    for (int i = 0; i < 4; i++) motorOut[i] = 1000;
  });

  runCase("forceHoverEstimate biases the failsafe reference", [] {
    Run run = hoverRun(0.6, 0.0, 0.0);
    run.arm(1340);
    run.runTicks(3000);
    CHECK(hover_valid);
    forceHoverEstimate(1370.0f);
    CHECK(hover_est == 1370.0f && hoverTracker.estimate_us == 1370.0f);
    CHECK(hover_valid);
  });

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
      // 거리 기반 자동착륙은 위치 유지를 켠다(sil_poshold_applied_during_range_descent).
      // 이 시험은 진입 때 거리가 끊긴 시간 기반 모드에서 보정이 0 인지 본다.
      run.sensor.range_off_from = run.globalTick;
      run.sensor.range_off_to = std::numeric_limits<uint32_t>::max();
      run.runTicks(1500);
      CHECK(fs_phase != FS_NONE);
      CHECK(rl_mode == RL_MODE_TIMED);
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


  // ---------------- 거리 기반 자동착륙 ----------------

  for (double h0 : {0.6, 1.2}) {
    runCase(h0 == 0.6 ? "sil_lands_from_0_6m_with_cut_landed"
                      : "sil_lands_from_1_2m_with_cut_landed", [h0] {
      LandCase c(h0);
      c.warmup();
      c.land();
      CHECK(c.w.saw_range && !c.w.saw_fallback);
      CHECK_MSG(c.w.end_phase == FS_CUT_LANDED, "phase=" + fmt(c.w.end_phase));
      CHECK(c.w.cut_reason == RL_CUT_CONTACT);
      CHECK_MSG(c.w.h_at_end < 0.02, "h at cut=" + fmt(c.w.h_at_end));
    });
  }

  runCase("sil_lands_from_1_7m_within_backstop", [] {
    LandCase c(1.7);
    c.warmup();
    c.land();
    CHECK(c.w.saw_range);
    CHECK_MSG(c.w.end_phase == FS_CUT_LANDED, "phase=" + fmt(c.w.end_phase));
    CHECK_MSG(c.w.end_tick - c.w.entry_tick < RL_MAX_MS,
              "elapsed=" + fmt(c.w.end_tick - c.w.entry_tick));
  });

  for (float bias : {30.0f, -30.0f}) {
    runCase(bias > 0 ? "sil_hover_bias_plus_30us_still_lands"
                     : "sil_hover_bias_minus_30us_lands_without_hard_hit", [bias] {
      LandCase c(1.0);
      c.warmup();
      c.extra = [&c, bias](uint32_t tick) {
        if (tick >= c.w.disconnect_tick && fs_phase == FS_NONE) {
          forceHoverEstimate(1340.0f + bias);
        }
      };
      c.land();
      CHECK(c.w.saw_range);
      CHECK_MSG(c.w.end_phase == FS_CUT_LANDED, "phase=" + fmt(c.w.end_phase));
      CHECK_MSG(c.w.vh_at_touch > -0.5, "touchdown vh=" + fmt(c.w.vh_at_touch));
    });
  }

  runCase("sil_battery_sag_10pct_still_lands", [] {
    LandCase c(1.2);
    c.warmup();
    const uint32_t from = c.run.globalTick + 600;
    c.run.P.thrust_scale_at = [from](uint32_t tick) {
      if (tick <= from) return 1.0;
      return std::max(0.9, 1.0 - 0.1 * (tick - from) / 5000.0);
    };
    c.land();
    CHECK(c.w.saw_range);
    CHECK_MSG(c.w.end_phase == FS_CUT_LANDED, "phase=" + fmt(c.w.end_phase));
  });

  runCase("sil_ground_effect_still_reaches_near_ground", [] {
    LandCase c(0.8);
    c.run.P.ground_effect_gain = 0.10;
    c.warmup();
    c.land();
    CHECK(c.w.near_tick != 0);
    CHECK_MSG(c.w.end_phase == FS_CUT_LANDED, "phase=" + fmt(c.w.end_phase));
  });

  runCase("sil_poshold_applied_during_range_descent", [] {
    LandCase c(1.0, 3.0);
    c.warmup(10000);                   // 위치 유지가 외란을 흡수할 때까지
    // 바닥 근접 뒤 적용 보정은 후보 × hold_scale(500 ms 에 1→0) 을 넘지 않는다.
    double worst_excess = 0.0;
    c.extra = [&](uint32_t) {
      if (fs_phase != FS_DESCENDING || rl_stage < RL_STAGE_NEAR_GROUND) return;
      const FlowHoldTelemetry t = readFlowHoldTelemetry();
      const float since = (float)(millis() - rangeLand.near_ground_since_ms);
      const float scale = std::max(0.0f, 1.0f - since / (float)RL_HOLD_FADE_MS);
      const double cand = std::max(std::fabs(t.candidate_roll_deg), std::fabs(t.candidate_pitch_deg));
      const double appl = std::max(std::fabs(t.applied_roll_deg), std::fabs(t.applied_pitch_deg));
      worst_excess = std::max(worst_excess, appl - (scale + 0.01) * cand - 1e-3);
    };
    c.land();
    CHECK_MSG(worst_excess <= 0.0, "applied exceeded the faded candidate by " + fmt(worst_excess));
    CHECK(c.w.saw_range);
    CHECK(c.w.applied_seen);
    CHECK_MSG(c.w.max_horiz_dev < 0.5, "drift=" + fmt(c.w.max_horiz_dev));
    CHECK_MSG(c.w.applied_at_end < 0.05f, "applied at contact=" + fmt(c.w.applied_at_end));
    CHECK(c.w.end_phase == FS_CUT_LANDED);
  });

  for (float bias : {0.0f, 30.0f}) {
    // +30 us 편향이면 고도 보정이 음수로 크게 걸려 있다가, 바닥 근접에서 높이가 무효가
    // 되며 0 쪽으로 감쇠한다. 래칫이 없으면 그만큼 collective 가 오른다.
    runCase("sil_no_collective_rise_after_near_ground bias " + fmt(bias), [bias] {
      LandCase c(0.6);
      c.warmup();
      c.extra = [&c, bias](uint32_t tick) {
        if (bias != 0.0f && tick >= c.w.disconnect_tick && fs_phase == FS_NONE) {
          forceHoverEstimate(1340.0f + bias);
        }
      };
      c.land();
      CHECK(c.w.near_tick != 0);
      CHECK(!c.w.req_rose_after_near);
    });
  }

  runCase("sil_near_object_does_not_pull_altitude", [] {
    // 1.5 m 에서 0.5 m 근접값이 1 s 동안 들어온다. 거부된 값을 고도 제어에 넣으면
    // 1 m 아래에 있다고 보고 끌어올린다 — 폴백(300 ms)까지 하강이 거의 멈춘다
    // (가드 없이 돌리면 −0.34 → −0.01 m/s).
    LandCase c(1.5);
    c.warmup();
    uint32_t from = 0;
    double h_from = 0.0, max_climb = 0.0, vh_from = 0.0, max_vh = -9.0;
    c.extra = [&](uint32_t tick) {
      if (!from && c.w.entry_tick && tick >= c.w.entry_tick + 800) {
        from = tick;
        h_from = c.run.s.h;
        vh_from = c.run.s.vh;
      }
      if (from && tick < from + 1000) max_climb = std::max(max_climb, c.run.s.h - h_from);
      if (from && tick < from + 400) max_vh = std::max(max_vh, c.run.s.vh);
    };
    c.run.sensor.range_override = [&from](uint32_t tick, int32_t mm) {
      return from && tick >= from && tick < from + 1000 ? (int32_t)500 : mm;
    };
    c.land();
    CHECK(from != 0);
    CHECK_MSG(max_climb < 0.03, "climb during the near object=" + fmt(max_climb));
    CHECK_MSG(max_vh - vh_from < 0.1, "descent slowed by " + fmt(max_vh - vh_from));
    CHECK(c.w.end_phase != FS_CUT_LANDED || c.w.h_at_end < 0.02);
  });

  runCase("sil_dropout_mid_descent_falls_back_to_timed", [] {
    LandCase c(1.2);
    c.warmup();
    c.extra = [&c](uint32_t tick) {
      if (rl_mode == RL_MODE_RANGE && c.run.s.h < 0.8 &&
          c.run.sensor.range_off_to == 0) {
        c.run.sensor.range_off_from = tick;
        c.run.sensor.range_off_to = tick + 6000;
      }
    };
    c.land();
    CHECK(c.w.saw_range && c.w.saw_fallback);
    CHECK_MSG(c.w.end_phase == FS_CUT_TIMEOUT, "phase=" + fmt(c.w.end_phase));
    const int32_t budget = (int32_t)(c.w.end_tick - c.w.fallback_tick);
    CHECK_MSG(std::abs(budget - (int32_t)FS_MAX_MS) <= 2, "timed budget=" + fmt(budget));
  });

  runCase("sil_fallback_does_not_raise_collective", [] {
    // 하강 중 추력이 서서히 30 % 늘면(실제 필요한 collective 가 호버 추정보다 낮아짐)
    // 시간 기반 하강값(추정 − 60)이 실제로 내던 collective 보다 높다. 상한이 없으면 폴백
    // 순간 위로 튄다.
    LandCase c(1.2);
    c.warmup();
    c.run.P.thrust_scale_at = [&c](uint32_t tick) {
      if (!c.w.entry_tick || tick <= c.w.entry_tick) return 1.0;
      return std::min(1.3, 1.0 + 0.3 * (tick - c.w.entry_tick) / 3000.0);
    };
    c.extra = [&c](uint32_t tick) {
      if (rl_mode == RL_MODE_RANGE && c.run.s.h < 0.6 &&
          c.run.sensor.range_off_to == 0) {
        c.run.sensor.range_off_from = tick;
        c.run.sensor.range_off_to = tick + 6000;
      }
    };
    c.land();
    CHECK(c.w.saw_range);
    CHECK(c.w.fallback_tick != 0);
    CHECK_MSG(c.w.req_before_fallback < 1340 - FS_DESCENT_DELTA_US,
              "cap not exercised, req=" + fmt(c.w.req_before_fallback) +
              " h@fallback=" + fmt(c.w.h_at_fallback));
    CHECK(!c.w.req_rose_after_fallback);
  });

  runCase("sil_two_step_near_object_does_not_cut", [] {
    LandCase c(1.5);
    c.warmup();
    uint32_t from = 0;
    c.run.sensor.range_override = [&c, &from](uint32_t tick, int32_t mm) {
      if (!from && c.w.entry_tick && tick >= c.w.entry_tick + 800) from = tick;
      if (from && tick < from + 35) return (int32_t)200;
      if (from && tick < from + 1035) return (int32_t)30;
      return mm;
    };
    c.land();
    CHECK(from != 0);
    CHECK(c.w.end_phase != FS_CUT_LANDED);
    CHECK(c.w.saw_fallback);
  });

  runCase("sil_backlogged_near_frame_is_not_contact_evidence", [] {
    LandCase c(1.0);
    c.warmup();
    uint32_t stall = 0;
    c.extra = [&c, &stall](uint32_t tick) {
      if (!stall && rl_mode == RL_MODE_RANGE && c.run.s.h < 0.5) {
        stall = tick;
        c.run.stall_from = tick;
        c.run.stall_to = tick + 100;
      }
    };
    c.run.sensor.range_override = [&stall](uint32_t tick, int32_t mm) {
      return stall && tick >= stall && tick < stall + 100 ? (int32_t)30 : mm;
    };
    c.land();
    CHECK(stall != 0);
    CHECK(c.w.near_tick != 0);
    CHECK_MSG(c.w.h_at_near < 0.12, "near ground entered at h=" + fmt(c.w.h_at_near));
    CHECK(!c.w.saw_fallback);
    CHECK(c.w.end_phase == FS_CUT_LANDED);
  });

  runCase("sil_resume_near_ground_has_no_step", [] {
    LandCase c(0.6);
    c.run.P.tether_floor = [](uint32_t) { return 0.05; };
    c.warmup();
    uint32_t resumed = 0;
    int max_step = 0, prev = 0;
    c.extra = [&](uint32_t tick) {
      if (!resumed && c.w.near_tick && tick == c.w.near_tick + 1000) {
        restoreRc(c.run);
        resumed = tick;
      }
      if (resumed && tick == resumed + 1) requestResume();
      if (resumed && tick > resumed + 1 && tick < resumed + 1500) {
        max_step = std::max(max_step, base_throttle - prev);
      }
      prev = base_throttle;
    };
    c.land(6000);
    CHECK(resumed != 0);
    CHECK(fs_phase == FS_NONE && !safety_lock);
    CHECK_MSG(max_step <= 1, "largest base step after resume=" + fmt(max_step));
    CHECK(std::abs(base_throttle - 1340) <= 1);
    CHECK(!rl_resume_handover);
  });

  runCase("sil_rc_loss_during_resume_handover_reenters", [] {
    LandCase c(0.6);
    c.run.P.tether_floor = [](uint32_t) { return 0.05; };
    c.warmup();
    uint32_t restored = 0;
    int base_at_resume = 0, req_at_resume = 0;
    bool reentered = false, locked = false, rose = false;
    c.extra = [&](uint32_t tick) {
      if (!restored && c.w.near_tick && base_throttle <= FS_GROUND_CUT_MAX_US) {
        restoreRc(c.run);
        restored = tick;
      }
      if (restored && tick == restored + 1) c.run.rc_disconnect_tick = tick;   // 마지막 RC
      if (restored && tick == restored + 499) {
        requestResume();
      }
      if (restored && tick == restored + 501) {
        base_at_resume = base_throttle;
        req_at_resume = rl_requested_collective_us;
      }
      if (restored && tick > restored + 501 && tick < restored + 1500) {
        if (safety_lock) locked = true;
        if (fs_phase == FS_DESCENDING) reentered = true;
        if (rl_requested_collective_us > req_at_resume + 2) rose = true;
      }
    };
    c.land(8000);
    CHECK(restored != 0);
    CHECK_MSG(base_at_resume <= FS_GROUND_CUT_MAX_US + 2, "base=" + fmt(base_at_resume));
    CHECK(reentered);
    CHECK_MSG(!locked || fs_phase != FS_NONE, "ground cut during the handover");
    CHECK(!rose);
  });

  runCase("sil_rc_loss_after_handover_uses_ground_rule", [] {
    LandCase c(0.6);
    c.run.P.tether_floor = [](uint32_t) { return 0.05; };
    c.warmup();
    uint32_t resumed = 0, low = 0;
    c.extra = [&](uint32_t tick) {
      if (!resumed && c.w.near_tick && tick == c.w.near_tick + 1000) {
        restoreRc(c.run);
        resumed = tick;
      }
      if (resumed && tick == resumed + 1) requestResume();
      if (resumed && !low && tick > resumed + 10 && !rl_resume_handover) {
        base_throttle = 1100;            // 조종자가 바닥 스로틀로 내렸다
        low = tick;
      }
      if (low && tick == low + 10) c.run.rc_disconnect_tick = tick;
    };
    c.land(8000);
    CHECK(low != 0);
    CHECK(safety_lock);
    CHECK(fs_phase == FS_NONE);          // GROUND CUT: 자동착륙에 들어가지 않는다
  });

  runCase("sil_near_ground_roll_disturbance_bounded", [] {
    LandCase c(0.6);
    c.warmup();
    c.run.disturbance = [&c](uint32_t tick) {
      Disturbance d;
      if (c.w.near_tick && tick >= c.w.near_tick) {
        d.torque_roll = 0.03 * std::sin(2.0 * kPi * 2.0 * (tick - c.w.near_tick) * kDt);
      }
      return d;
    };
    c.land();
    CHECK(c.w.near_tick != 0);
    CHECK(!c.w.req_rose_after_near);
    CHECK_MSG(c.w.max_rise_near <= 0.02, "rise=" + fmt(c.w.max_rise_near));
    CHECK_MSG(c.w.max_vh_up_near <= 0.05, "vh up=" + fmt(c.w.max_vh_up_near));
    CHECK(c.w.end_phase == FS_CUT_LANDED ||
          (c.w.end_phase == FS_CUT_TIMEOUT && c.w.cut_reason == RL_CUT_NEAR_GROUND_STUCK));
    if (c.w.end_phase == FS_CUT_LANDED) CHECK(c.w.h_at_end <= 0.01);
    CHECK(c.w.end_tick - c.w.near_tick <= RL_TOUCHDOWN_MAX_MS + 2);
  });

  runCase("sil_resume_reentry_starts_a_new_episode", [] {
    LandCase c(1.2);
    c.warmup();
    uint32_t resumed = 0, second = 0, second_enter = 0, first_enter = 0;
    c.extra = [&](uint32_t tick) {
      if (!resumed && c.w.entry_tick && tick == c.w.entry_tick + 2000) {
        restoreRc(c.run);
        resumed = tick;
      }
      if (resumed && tick == resumed + 1) requestResume();
      if (!first_enter && c.w.entry_tick) first_enter = rangeLand.enter_ms;
      if (resumed && tick == resumed + 100) c.run.rc_disconnect_tick = tick;
      if (resumed && !second && tick > resumed + 100 && fs_phase == FS_DESCENDING) {
        second = tick;
        second_enter = rangeLand.enter_ms;
      }
    };
    c.land(25000);
    CHECK(resumed != 0 && second != 0);
    CHECK(first_enter != 0 && second_enter >= first_enter + 2100);
    CHECK_MSG(fs_phase == FS_CUT_LANDED, "phase=" + fmt(fs_phase));
  });

  runCase("sil_loss_after_near_ground_cuts_not_timed", [] {
    LandCase c(0.6);
    c.run.P.tether_floor = [](uint32_t) { return 0.05; };
    c.warmup();
    uint32_t off = 0;
    c.extra = [&](uint32_t tick) {
      if (!off && c.w.near_tick && tick == c.w.near_tick + 500) {
        off = tick;
        c.run.sensor.range_off_from = tick;
        c.run.sensor.range_off_to = tick + 10000;
      }
    };
    c.land(8000);
    CHECK(off != 0);
    CHECK(!c.w.saw_fallback);
    CHECK_MSG(c.w.end_phase == FS_CUT_TIMEOUT, "phase=" + fmt(c.w.end_phase));
    CHECK(c.w.cut_reason == RL_CUT_NEAR_GROUND_LOSS);
    CHECK_MSG(c.w.end_tick - off <= RL_INVALID_GRACE_MS + 40,
              "cut after " + fmt(c.w.end_tick - off) + " ms");
  });

  runCase("sil_near_object_at_1_5m_does_not_cut_landed", [] {
    LandCase c(1.5);
    c.warmup();
    c.run.sensor.range_override = [&c](uint32_t tick, int32_t mm) {
      return c.w.entry_tick && tick >= c.w.entry_tick + 500 && tick < c.w.entry_tick + 1500
                 ? (int32_t)30 : mm;
    };
    c.land();
    CHECK(c.w.end_phase != FS_CUT_LANDED);
  });

  for (uint32_t lead : {40u, 110u, 400u}) {
    // 1.5 m 에서 RC 가 끊기기 lead ms 전부터 근접 물체 값(200 mm)이 들어오고, 진입 뒤
    // 130→60→30 mm 로 이어진다. 그 값을 진입 기준으로 잡으면 공중 1 m 위에서 착지로
    // 간다(Codex P1: 세 프레임만 유지돼도 연속 카운터를 통과했다).
    runCase("sil_near_object_at_entry_tick_uses_timed lead " + fmt(lead), [lead] {
      LandCase c(1.5);
      c.warmup();
      uint32_t first = 0;
      c.run.sensor.range_override = [&](uint32_t tick, int32_t mm) {
        if (tick + lead < c.w.disconnect_tick + RC_TIMEOUT_MS) return mm;
        if (!first) first = tick;
        if (!c.w.entry_tick) return (int32_t)200;
        const uint32_t k = (tick - c.w.entry_tick) / 35;
        return k == 0 ? (int32_t)130 : (k == 1 ? (int32_t)60 : (int32_t)30);
      };
      c.land(8000);
      CHECK(first != 0);
      CHECK(c.w.entry_tick != 0);
      CHECK(first <= c.w.entry_tick);
      CHECK_MSG(!c.w.saw_range, "range mode chosen from a near-object entry value");
      CHECK(c.w.end_phase != FS_CUT_LANDED);
    });
  }

  runCase("sil_stale_consumed_frame_does_not_pollute_vz", [] {
    // 정상 1500 mm 사이에, 이미 165 ms 지난 200 mm 신뢰 프레임이 늦게 소비된다(Codex P2).
    // 그 값을 높이 추정에 넣으면 다음 정상 프레임 뒤 가짜 상승속도가 고도 제어에 남는다.
    LandCase c(1.6);
    c.warmup();
    uint32_t seen = 0, spiked = 0;
    double vh_before = 0.0, worst = 0.0;
    c.extra = [&](uint32_t tick) {
      if (!spiked && rl_mode == RL_MODE_RANGE && c.w.entry_tick &&
          tick >= c.w.entry_tick + 1000) {
        portENTER_CRITICAL(&flowSensorMux);
        if (flowSensorSnapshot.range_seq != seen && seen != 0) {
          flowSensorSnapshot.range_mm = 200;
          flowSensorSnapshot.range_t_us -= 165000;
          spiked = tick;
          vh_before = c.run.s.vh;
        }
        seen = flowSensorSnapshot.range_seq;
        portEXIT_CRITICAL(&flowSensorMux);
      } else if (!spiked) {
        portENTER_CRITICAL(&flowSensorMux);
        seen = flowSensorSnapshot.range_seq;
        portEXIT_CRITICAL(&flowSensorMux);
      }
      if (spiked && tick > spiked && tick < spiked + 600) {
        worst = std::max(worst, std::fabs(c.run.s.vh - vh_before));
      }
    };
    c.land();
    CHECK(spiked != 0);
    CHECK(!c.w.saw_fallback);
    CHECK_MSG(worst < 0.1, "vertical speed disturbed by " + fmt(worst));
    CHECK(c.w.end_phase == FS_CUT_LANDED);
  });

  runCase("sil_single_rejected_frame_does_not_kick_althold", [] {
    // 1500→200→1500 mm 처럼 한 프레임만 튀면 거부되지만, 그 프레임으로 차분한 수직속도가
    // 다음 정상 프레임 뒤 고도 제어에 들어가면 하강을 크게 밀어낸다(Codex P2). 하강 이득
    // k_vz 60 에서 수정 전 0.22 m/s, 추정을 비우면 0.087, 되돌리면 0.016. k_vz 120 에서 0.06.
    LandCase c(1.6);
    c.warmup();
    uint32_t spike = 0;
    double vh_before = 0.0, worst = 0.0;
    c.run.sensor.range_override = [&](uint32_t tick, int32_t mm) {
      if (!spike && rl_mode == RL_MODE_RANGE && c.w.entry_tick &&
          tick >= c.w.entry_tick + 1000) {
        spike = tick;
        vh_before = c.run.s.vh;
        return (int32_t)200;
      }
      return mm;
    };
    c.extra = [&](uint32_t tick) {
      if (spike && tick > spike && tick < spike + 600) {
        worst = std::max(worst, std::fabs(c.run.s.vh - vh_before));
      }
    };
    c.land();
    CHECK(spike != 0);
    CHECK(!c.w.saw_fallback);
    CHECK_MSG(worst < 0.1, "vertical speed disturbed by " + fmt(worst));
    CHECK(c.w.end_phase == FS_CUT_LANDED);
  });

  runCase("sil_frame_published_after_tick_start_is_not_lost", [] {
    // Core 0 가 pid tick 시작 뒤 프레임을 게시하면(도착 시각 > tick 시각) 그 tick 에는
    // 아직 오지 않은 프레임이다. 먼저 소비하고 stale 로 버리면 정상 거리로도 폴백한다.
    LandCase c(1.0);
    c.warmup();
    uint32_t seen = 0;
    int bumped = 0;
    c.extra = [&](uint32_t) {
      if (rl_mode != RL_MODE_RANGE) return;
      portENTER_CRITICAL(&flowSensorMux);
      if (flowSensorSnapshot.range_seq != seen) {
        seen = flowSensorSnapshot.range_seq;
        flowSensorSnapshot.range_t_us += 500;   // tick 시작 0.5 ms 뒤 게시
        bumped++;
      }
      portEXIT_CRITICAL(&flowSensorMux);
    };
    c.land();
    CHECK(bumped > 20);
    CHECK(!c.w.saw_fallback);
    CHECK(c.w.end_phase == FS_CUT_LANDED);
  });

  runCase("sil_bias_sag_matrix_lands_softly", [] {
    // 시작 높이 × 호버 편향(±30 us, 계획 범위) × 배터리 처짐 10 %. 모두 폴백 없이 착지 컷,
    // 접지 직전 하강 속도 0.5 m/s 미만(최종 검토: −30 us·처짐이 겹치면 정상 값을 거부하고
    // 폴백해 −2.2 m/s 로 접지했다).
    std::string bad;
    for (double h0 : {0.6, 1.0, 1.7}) {
      for (float bias : {-30.0f, 0.0f, 30.0f}) {
        for (int sag : {0, 1}) {
          LandCase c(h0);
          c.warmup();
          if (sag) {
            const uint32_t from = c.run.globalTick + 600;
            c.run.P.thrust_scale_at = [from](uint32_t tick) {
              return tick <= from ? 1.0 : std::max(0.9, 1.0 - 0.1 * (tick - from) / 5000.0);
            };
          }
          c.extra = [&c, bias](uint32_t tick) {
            if (bias != 0.0f && tick >= c.w.disconnect_tick && fs_phase == FS_NONE) {
              forceHoverEstimate(1340.0f + bias);
            }
          };
          c.land();
          if (c.w.end_phase != FS_CUT_LANDED || c.w.saw_fallback || c.w.vh_at_touch <= -0.5) {
            bad += " [h0=" + fmt(h0) + " bias=" + fmt(bias) + " sag=" + fmt(sag) +
                   " phase=" + fmt(c.w.end_phase) + " fb=" + fmt(c.w.saw_fallback) +
                   " vh=" + fmt(c.w.vh_at_touch) + "]";
          }
        }
      }
    }
    CHECK_MSG(bad.empty(), bad);
  });

  for (bool invalid_first : {false, true}) {
  // invalid_first: 200 mm 앞에 −1 한 프레임(최종 재검토 Critical: 음수 한 프레임이 추적을 비웠다).
  runCase(std::string("sil_held_near_object_before_rc_loss_does_not_land") +
          (invalid_first ? " after -1" : ""), [invalid_first] {
    // 1.5 m 에서 RC 상실 2.5 s 전부터 원값이 200 mm 로 유지되고, 진입 뒤 30 mm 로 내려간다
    // (최종 검토 Critical: 1.226 m 에서 CUT_LANDED). 조종자 고도 유지는 끈다 — 켜 두면 200 mm 를
    // 보고 기체를 끌어올려 다른 시나리오가 된다.
    LandCase c(1.5);
    c.warmup(6000, false);
    c.run.sensor.range_override = [&c, invalid_first](uint32_t tick, int32_t mm) {
      if (tick + 2500 < c.w.disconnect_tick + RC_TIMEOUT_MS) return mm;
      if (invalid_first && tick + 2500 < c.w.disconnect_tick + RC_TIMEOUT_MS + 35) return (int32_t)-1;
      if (!c.w.entry_tick) return (int32_t)200;
      // 진입 뒤 0.4 m/s 로 낮춘다(추적 허용 안의 속도).
      return std::max((int32_t)30, 200 - (int32_t)lround(0.4 * (tick - c.w.entry_tick)));
    };
    c.land(11000, 3000);             // 끊김을 늦춰 근접값이 진입 전 2.5 s 유지되게
    CHECK(c.w.entry_tick != 0);
    CHECK(!c.w.saw_range);
    CHECK(c.w.end_phase != FS_CUT_LANDED);
  });
  }

  runCase("sil_outlier_frame_does_not_disturb_flow_velocity", [] {
    // 광류 프레임과 같은 tick 에 1800 mm 이상값이 오면, 판정 전에 광류가 그 높이로 속도를
    // 환산해 수평 속도·가속도 추정이 튄다(최종 검토 Important: 0.29 → 1.55 m/s).
    LandCase c(0.8, 1.0);
    c.warmup(8000);
    uint32_t spike = 0;
    double base_v = 0.0, worst = 0.0;
    c.run.sensor.range_override = [&](uint32_t tick, int32_t mm) {
      if (!spike && rl_mode == RL_MODE_RANGE && c.run.s.h < 0.4 && tick % 700 == 0) {
        spike = tick;
        const FlowHoldTelemetry t = readFlowHoldTelemetry();
        base_v = std::hypot(t.c_roll, t.c_pitch);
        return (int32_t)1800;
      }
      return mm;
    };
    c.extra = [&](uint32_t tick) {
      if (spike && tick > spike && tick < spike + 400) {
        const FlowHoldTelemetry t = readFlowHoldTelemetry();
        worst = std::max(worst, std::hypot(t.c_roll, t.c_pitch) - base_v);
      }
    };
    c.land();
    CHECK(spike != 0);
    CHECK_MSG(worst < 0.15, "flow velocity jumped by " + fmt(worst));
  });

  runCase("sil_tether_floor_5cm_cuts_near_ground", [] {
    LandCase c(0.6);
    c.run.P.tether_floor = [](uint32_t) { return 0.05; };
    c.warmup();
    c.land(10000);
    CHECK(c.w.near_tick != 0);
    CHECK_MSG(c.w.end_phase == FS_CUT_TIMEOUT, "phase=" + fmt(c.w.end_phase));
    CHECK(c.w.cut_reason == RL_CUT_NEAR_GROUND_STUCK);
    const int32_t after = (int32_t)(c.w.end_tick - c.w.near_tick);
    CHECK_MSG(std::abs(after - (int32_t)RL_TOUCHDOWN_MAX_MS) <= 2, "after=" + fmt(after));
  });

  runCase("sil_tether_floor_2cm_cut_is_harmless", [] {
    LandCase c(0.6);
    c.run.P.tether_floor = [](uint32_t) { return 0.02; };
    c.warmup();
    c.land(10000);
    CHECK(c.w.end_tick != 0);
    CHECK(c.w.end_phase == FS_CUT_LANDED || c.w.end_phase == FS_CUT_TIMEOUT);
    CHECK_MSG(c.w.h_at_end <= 0.021, "h at cut=" + fmt(c.w.h_at_end));
  });

  runCase("sil_core0_stall_does_not_force_fallback", [] {
    for (uint32_t phase : {0u, 13u, 27u}) {
      LandCase c(1.0);
      c.warmup();
      uint32_t stalls = 0;
      c.extra = [&](uint32_t tick) {
        if (rl_mode == RL_MODE_RANGE && c.w.entry_tick &&
            tick == c.w.entry_tick + 700 + phase + 600 * stalls && stalls < 4) {
          c.run.stall_from = tick;
          c.run.stall_to = tick + 100;
          stalls++;
        }
      };
      c.land();
      CHECK(stalls == 4);
      CHECK_MSG(!c.w.saw_fallback, "phase " + fmt(phase));
      CHECK(c.w.end_phase == FS_CUT_LANDED);
    }
  });

  runCase("sil_height_invalid_at_entry_uses_timed", [] {
    LandCase c(0.8);
    c.warmup();
    c.extra = [&c](uint32_t tick) {
      if (tick == c.w.disconnect_tick) {
        c.run.sensor.range_off_from = tick;
        c.run.sensor.range_off_to = tick + 10000;
      }
    };
    c.land(6000);
    CHECK(c.w.saw_timed && !c.w.saw_range);
    CHECK(c.w.end_phase == FS_CUT_TIMEOUT);
  });

  runCase("sil_hover_invalid_still_cuts_immediately", [] {
    LandCase c(0.8);
    c.warmup(200);                     // 호버 추정이 유효해지기 전
    CHECK(!hover_valid);
    c.land(2000);
    CHECK(safety_lock);
    CHECK(c.w.entry_tick == 0);
  });

  runCase("sil_resume_during_range_descent", [] {
    LandCase c(1.7);                   // 4 s 뒤에도 0.5 m 쯤 남는다
    c.warmup();
    uint32_t resumed = 0;
    double h_after = -1.0;
    uint8_t phase_after = FS_DESCENDING, mode_after = RL_MODE_RANGE;
    c.extra = [&](uint32_t tick) {
      if (!resumed && c.w.entry_tick && tick == c.w.entry_tick + 4000) {
        restoreRc(c.run);
        resumed = tick;
      }
      if (resumed && tick == resumed + 1) requestResume();
      if (resumed && tick == resumed + 5) {
        h_after = c.run.s.h;
        phase_after = fs_phase;
        mode_after = rl_mode;
      }
    };
    c.land(5000);
    CHECK(resumed != 0);
    // resume 뒤 스로틀은 조종자 것이다(고도 유지는 꺼져 있다). 직후 상태만 본다.
    CHECK(phase_after == FS_NONE && !safety_lock);
    CHECK(mode_after == RL_MODE_NONE);
    CHECK_MSG(h_after > 0.3, "h right after resume=" + fmt(h_after));
  });

  runCase("sil_telemetry_reports_range_landing_fields", [] {
    LandCase c(0.6);
    c.warmup();
    std::vector<std::string> during;
    c.extra = [&](uint32_t) {
      if (during.empty() && rl_stage == RL_STAGE_APPROACH) {
        wifi_udp_fake::reset();
        connectionEstablished = true;
        sendTelemetry();
        std::stringstream ss(wifi_udp_fake::telemetry_output);
        std::string f;
        while (std::getline(ss, f, ',')) during.push_back(f);
      }
    };
    c.land();
    CHECK(c.w.end_phase == FS_CUT_LANDED);
    CHECK_MSG(during.size() == 96, "fields=" + fmt(during.size()));
    CHECK(during[91] == "1");          // Land_Mode RANGE
    CHECK(during[92] == "2");          // Land_Stage APPROACH
    CHECK(std::stod(during[93]) > 0.0 && std::stod(during[93]) < 0.4);
    CHECK(during[94] == "0");
    CHECK(std::stoi(during[95]) > 1000);
    wifi_udp_fake::reset();
    connectionEstablished = true;
    sendTelemetry();
    std::vector<std::string> after;
    std::stringstream ss(wifi_udp_fake::telemetry_output);
    std::string f;
    while (std::getline(ss, f, ',')) after.push_back(f);
    CHECK(after.size() == 96);
    CHECK(after[94] == "1");           // Land_Cut_Reason CONTACT, kept after the cut
    // 요청·실제 collective 는 같은 스냅숏이다: 컷 뒤 둘 다 1000(차이 0).
    CHECK_MSG(std::fabs(std::stod(after[67]) - std::stod(after[95])) < 0.5,
              "collective " + after[67] + " vs requested " + after[95]);
    // 하강 중에도 같은 tick 의 값이다(이 하강은 allocator 상승이 없다).
    CHECK_MSG(std::fabs(std::stod(during[67]) - std::stod(during[95])) < 0.5,
              "collective " + during[67] + " vs requested " + during[95]);
  });

  runCase("sil_range_landing_across_the_micros_wrap", [] {
    // micros 는 약 71.6 분마다 감긴다. 추적 시각을 micros/1000 으로 만들면 그 순간 공백이
    // 40 억 ms 로 계산된다(Codex P2). 진입 전 추적 도중(warmup 끝 3.0 s 와 진입 3.6 s 사이)
    // 에 감기게 한다.
    LandCase c(0.8);
    arduino_fake::micros_value = 0xFFFFFFFFu - 3300000u;   // 약 3.3 s 뒤 감김
    c.warmup(3000);
    c.land();
    CHECK(c.w.saw_range);
    CHECK(!c.w.saw_fallback);
    CHECK_MSG(c.w.end_phase == FS_CUT_LANDED, "phase=" + fmt(c.w.end_phase));
    CHECK(rangeLand.reject_count == 0);
  });

  runCase("sil_mode_switch_is_smooth", [] {
    LandCase c(1.0);
    c.warmup();
    c.land();
    CHECK(c.w.saw_range);
    CHECK_MSG(std::abs(c.w.req_at_entry - c.w.req_at_entry_before) <= 20,
              "entry step=" + fmt(c.w.req_at_entry - c.w.req_at_entry_before));
    CHECK_MSG(c.w.max_tilt_deg < 8.0, "tilt=" + fmt(c.w.max_tilt_deg));
  });

  std::cout << test_count - failure_count << "/" << test_count << " passed\n";
  return failure_count ? 1 : 0;
}
