// SIL 플랜트와 3901-L0X 센서 모델. test_sil_flow_hold.cpp 와 차분 하네스
// (tools/native_tests/diff_harness) 가 같이 쓴다. 스케치(zetin_fc.ino)와 <array>·
// <vector>·<cmath> 를 먼저 include 한 뒤, 익명 namespace 안에서 include 한다.
constexpr double kDt = 0.001;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kG = 9.81;
constexpr double kCountsPerRad = 9.5 * kRadToDeg;   // Matek 9~10 counts/deg

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
