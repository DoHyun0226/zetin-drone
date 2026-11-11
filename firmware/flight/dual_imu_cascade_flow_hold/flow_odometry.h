#pragma once

#include <math.h>
#include <stdint.h>

// 3901-L0X 광류·거리 → 보정 공간 수평 속도와 높이.
//
// 아래를 보는 센서가 광축 거리 d 에서 기체 속도 v(FRD), 각속도 (p,q) 를 가지면
// 시선 각속도는 ω = (v_y/d − p, −v_x/d − q) 다. 보정 행렬 M 은 순수 회전에서
// f = M·g (g = ∫(p,q)dt [rad]) 로 구했으므로 u = M⁻¹f 는 "같은 광류를 만들
// 자이로 적분"이고, u − g 가 병진 성분만이다. 그것에 d/dt 를 곱한
// c = (−v_y, v_x) 를 "보정 공간 속도"라 부른다. c 의 부호는 이동을 멈추는
// 자세 변화의 부호와 같다. 유도: docs/superpowers/plans/2026-10-29-flow-position-hold.md §2
//
// 위 유도는 FRD(+pitch = 기수 들림)로 썼다. 이 펌웨어의 실기체 규약은
// +pitch = 기수 숙임(+roll = 우측 날개 내림)이라 실기체에서는
// c_pitch = −v_전방, c_roll = −v_우측 이다. 제어 규칙(c 와 같은 부호로 목표각에
// 더함)은 두 부호가 함께 뒤집혀 그대로 제동한다: 전방 이동 → c_pitch < 0 →
// 목표 pitch 감소 = 기수 들림. 2026-10-30 프롭 없는 손 시험에서 확인했다.
//
// MSP 프레임에는 송신 seq 도 측정 구간 시각도 없다. 센서가 100 ms 주기로 보내므로
// 유실은 도착 간격으로 드러난다. 정상(80~130 ms)이면 그 구간을, 프레임 하나가
// 빠진 간격(160~260 ms)이면 마지막 절반(한 주기)을 이 프레임의 측정 구간으로 보고,
// 위치 적분은 간격 전체(span)로 한다. 그 밖은 버린다. 자이로는 이력 링에서
// [구간 끝 − 구간 길이 − 지연, 구간 끝 − 지연] 을 꺼낸다.

static const int FLOW_GYRO_RING_SIZE = 512;     // 1 kHz 로 512 ms
static const int FLOW_MIN_QUALITY = 50;         // 벤치 실측 93~162
static const float FLOW_DT_MIN_S = 0.080f;      // 실측 99~102 ms
static const float FLOW_DT_MAX_S = 0.130f;
static const float FLOW_DT_ONE_LOST_MIN_S = 0.160f;   // 한 프레임 유실 ≈ 200 ms
static const float FLOW_DT_ONE_LOST_MAX_S = 0.260f;
static const float FLOW_RANGE_MIN_M = 0.10f;    // 사양 80~2000 mm 에 여유
static const float FLOW_RANGE_MAX_M = 1.80f;
static const float FLOW_MAX_TILT_DEG = 20.0f;
static const float FLOW_SENSOR_MAX_RAD_S = 7.4f;  // PMW3901 데이터시트
// 회전 상쇄 오차는 회전율 × 시간 오차에 비례한다. 이보다 빠른 회전 구간은
// 위치 유지에 쓰지 않는다(보정에는 쓴다).
static const float FLOW_MAX_ROT_RAD_S = 2.0f;
static const float HEIGHT_VZ_TAU_S = 0.15f;
static const float HEIGHT_MAX_GAP_S = 0.30f;

// ---------------- 자이로 이력 ----------------

struct FlowGyroHistory {
  uint32_t t_us[FLOW_GYRO_RING_SIZE];
  float cum_x[FLOW_GYRO_RING_SIZE];
  float cum_y[FLOW_GYRO_RING_SIZE];
  uint32_t head;    // 다음에 쓸 칸
  uint32_t count;
  float acc_x, acc_y;
};

static inline void flowGyroReset(FlowGyroHistory &h) {
  h.head = 0;
  h.count = 0;
  h.acc_x = 0.0f;
  h.acc_y = 0.0f;
}

// rate [rad/s] 를 dt 동안 적분한 누적각을 t_us 시각 값으로 남긴다.
static inline void flowGyroPush(FlowGyroHistory &h, uint32_t t_us,
                                float rate_x, float rate_y, float dt_s) {
  h.acc_x += rate_x * dt_s;
  h.acc_y += rate_y * dt_s;
  h.t_us[h.head] = t_us;
  h.cum_x[h.head] = h.acc_x;
  h.cum_y[h.head] = h.acc_y;
  h.head = (h.head + 1) % FLOW_GYRO_RING_SIZE;
  if (h.count < (uint32_t)FLOW_GYRO_RING_SIZE) h.count++;
}

// i 번째로 오래된 항목(0 = 가장 오래됨)
static inline uint32_t flowGyroIndex(const FlowGyroHistory &h, uint32_t i) {
  return (h.head + FLOW_GYRO_RING_SIZE - h.count + i) % FLOW_GYRO_RING_SIZE;
}

// t_us 시각의 누적각(선형 보간). 이력 범위 밖이면 false.
static inline bool flowGyroAngleAt(const FlowGyroHistory &h, uint32_t t_us,
                                   float &ax, float &ay) {
  if (h.count < 2) return false;
  const uint32_t oldest = h.t_us[flowGyroIndex(h, 0)];
  const uint32_t newest = h.t_us[flowGyroIndex(h, h.count - 1)];
  // micros() wrap 에도 맞도록 가장 오래된 항목 기준 오프셋으로 비교한다.
  const uint32_t span = newest - oldest;
  const uint32_t off = t_us - oldest;
  if (off > span) return false;
  uint32_t lo = 0, hi = h.count - 1;
  while (hi - lo > 1) {
    const uint32_t mid = (lo + hi) / 2;
    if (h.t_us[flowGyroIndex(h, mid)] - oldest <= off) lo = mid;
    else hi = mid;
  }
  const uint32_t a = flowGyroIndex(h, lo), b = flowGyroIndex(h, hi);
  const uint32_t ta = h.t_us[a] - oldest, tb = h.t_us[b] - oldest;
  const float w = (tb > ta) ? (float)(off - ta) / (float)(tb - ta) : 0.0f;
  ax = h.cum_x[a] + w * (h.cum_x[b] - h.cum_x[a]);
  ay = h.cum_y[a] + w * (h.cum_y[b] - h.cum_y[a]);
  return true;
}

static inline bool flowGyroDelta(const FlowGyroHistory &h, uint32_t t0_us,
                                 uint32_t t1_us, float &gx, float &gy) {
  float x0, y0, x1, y1;
  if (!flowGyroAngleAt(h, t0_us, x0, y0) || !flowGyroAngleAt(h, t1_us, x1, y1)) {
    return false;
  }
  gx = x1 - x0;
  gy = y1 - y0;
  return true;
}

// ---------------- 광류 → 보정 공간 속도 ----------------

enum FlowReject : uint8_t {
  FLOW_REJECT_NONE = 0,
  FLOW_REJECT_UNCALIBRATED,
  FLOW_REJECT_TIMING,      // Core 0 폴링이 늦어 도착 시각을 믿을 수 없음
  FLOW_REJECT_FIRST,       // 구간 시작점이 없음(부팅·재개·타이밍 리셋 직후)
  FLOW_REJECT_DT,          // 도착 간격 이상(둘 이상 유실 등)
  FLOW_REJECT_QUALITY,
  FLOW_REJECT_RANGE,
  FLOW_REJECT_TILT,
  FLOW_REJECT_GYRO_HISTORY,
  FLOW_REJECT_FLOW_RATE,   // 센서 한계 7.4 rad/s 초과
};

struct FlowOdomConfig {
  bool calibrated;
  float m[2][2];       // counts/rad
  float m_inv[2][2];
  uint32_t delay_us;
};

struct FlowOdomState {
  bool have_prev;
  uint32_t prev_t_us;
};

struct FlowFrame {
  int32_t fx = 0, fy = 0;
  int quality = 0;
  uint32_t t_us = 0;
  bool timing_ok = false;
};

struct FlowOdomOutput {
  bool valid = false;
  bool rot_ok = false;     // 위치 유지에 써도 되는 회전율인가
  uint8_t reject = FLOW_REJECT_NONE;
  float dt_s = 0.0f;       // 이 프레임 카운트가 덮는 구간(속도 환산용)
  float span_s = 0.0f;     // 직전 프레임 이후 경과(위치 적분용)
  float c_roll = 0.0f, c_pitch = 0.0f;   // m/s, 보정 공간
  float gx = 0.0f, gy = 0.0f;            // 구간 자이로 적분, rad
};

static inline FlowOdomConfig flowOdomDefaultConfig() {
  FlowOdomConfig cfg;
  cfg.calibrated = false;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      cfg.m[i][j] = 0.0f;
      cfg.m_inv[i][j] = 0.0f;
    }
  }
  cfg.delay_us = 0;
  return cfg;
}

static inline bool flowOdomSetCalibration(FlowOdomConfig &cfg,
                                          const float m[2][2],
                                          uint32_t delay_us) {
  const float det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
  const float scale = fabsf(m[0][0]) + fabsf(m[0][1]) +
                      fabsf(m[1][0]) + fabsf(m[1][1]);
  if (!(scale > 0.0f) || fabsf(det) <= 1e-6f * scale * scale) return false;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) cfg.m[i][j] = m[i][j];
  }
  cfg.m_inv[0][0] = m[1][1] / det;
  cfg.m_inv[0][1] = -m[0][1] / det;
  cfg.m_inv[1][0] = -m[1][0] / det;
  cfg.m_inv[1][1] = m[0][0] / det;
  cfg.delay_us = delay_us;
  cfg.calibrated = true;
  return true;
}

static inline void flowOdomReset(FlowOdomState &s) {
  s.have_prev = false;
  s.prev_t_us = 0;
}

// 새 광류 프레임 하나를 처리한다. d_m 은 광축 거리(slant, 거리계 원값),
// range_valid 는 그 값이 신선하고 작동 범위 안인지다.
static inline FlowOdomOutput flowOdomUpdate(
    FlowOdomState &s, const FlowOdomConfig &cfg, const FlowFrame &frame,
    const FlowGyroHistory &gyro, float d_m, bool range_valid,
    float roll_deg, float pitch_deg) {
  FlowOdomOutput out;
  if (!frame.timing_ok) {
    // 이 프레임의 도착 시각은 다음 구간의 시작점으로도 못 쓴다.
    s.have_prev = false;
    out.reject = FLOW_REJECT_TIMING;
    return out;
  }
  if (!s.have_prev) {
    s.have_prev = true;
    s.prev_t_us = frame.t_us;
    out.reject = FLOW_REJECT_FIRST;
    return out;
  }
  const uint32_t t1 = frame.t_us;
  out.span_s = (float)(t1 - s.prev_t_us) * 1e-6f;
  s.prev_t_us = t1;
  if (out.span_s >= FLOW_DT_MIN_S && out.span_s <= FLOW_DT_MAX_S) {
    out.dt_s = out.span_s;
  } else if (out.span_s >= FLOW_DT_ONE_LOST_MIN_S &&
             out.span_s <= FLOW_DT_ONE_LOST_MAX_S) {
    out.dt_s = 0.5f * out.span_s;
  } else {
    out.reject = FLOW_REJECT_DT;
    return out;
  }
  if (frame.quality < FLOW_MIN_QUALITY) {
    out.reject = FLOW_REJECT_QUALITY;
    return out;
  }
  if (!range_valid || !(d_m >= FLOW_RANGE_MIN_M && d_m <= FLOW_RANGE_MAX_M)) {
    out.reject = FLOW_REJECT_RANGE;
    return out;
  }
  if (fabsf(roll_deg) > FLOW_MAX_TILT_DEG || fabsf(pitch_deg) > FLOW_MAX_TILT_DEG) {
    out.reject = FLOW_REJECT_TILT;
    return out;
  }
  // 미보정 판정은 다른 게이트 뒤에 둔다. 보정 중에는 보정값이 없으므로
  // reject == UNCALIBRATED 가 "구간 자체는 유효하다"는 뜻이 된다.
  if (!cfg.calibrated) {
    out.reject = FLOW_REJECT_UNCALIBRATED;
    return out;
  }
  const uint32_t dt_us = (uint32_t)lroundf(out.dt_s * 1e6f);
  const uint32_t w1 = t1 - cfg.delay_us;
  const uint32_t w0 = w1 - dt_us;
  if (!flowGyroDelta(gyro, w0, w1, out.gx, out.gy)) {
    out.reject = FLOW_REJECT_GYRO_HISTORY;
    return out;
  }
  const float fx = (float)frame.fx, fy = (float)frame.fy;
  const float ux = cfg.m_inv[0][0] * fx + cfg.m_inv[0][1] * fy;
  const float uy = cfg.m_inv[1][0] * fx + cfg.m_inv[1][1] * fy;
  if (sqrtf(ux * ux + uy * uy) / out.dt_s > FLOW_SENSOR_MAX_RAD_S) {
    out.reject = FLOW_REJECT_FLOW_RATE;
    return out;
  }
  out.c_roll = (ux - out.gx) * d_m / out.dt_s;
  out.c_pitch = (uy - out.gy) * d_m / out.dt_s;
  out.rot_ok = sqrtf(out.gx * out.gx + out.gy * out.gy) / out.dt_s <=
               FLOW_MAX_ROT_RAD_S;
  out.valid = true;
  return out;
}

// ---------------- 높이·수직속도 ----------------

struct HeightEstimator {
  bool valid;
  float d_m;        // 광축 거리(slant)
  float h_m;        // 수직 높이 = d·cos(roll)·cos(pitch)
  float vz_ms;      // 위로 +
  uint32_t last_t_us;
};

static inline void heightReset(HeightEstimator &h) {
  h.valid = false;
  h.d_m = 0.0f;
  h.h_m = 0.0f;
  h.vz_ms = 0.0f;
  h.last_t_us = 0;
}

// 새 거리 프레임 하나. range_mm 은 모듈 원값(범위 밖이면 음수).
static inline void heightUpdate(HeightEstimator &h, int32_t range_mm,
                                uint32_t t_us, float roll_deg, float pitch_deg) {
  const float d = (float)range_mm * 0.001f;
  if (!(d >= FLOW_RANGE_MIN_M && d <= FLOW_RANGE_MAX_M)) {
    h.valid = false;
    return;
  }
  const float hv = d * cosf(roll_deg * (float)M_PI / 180.0f) *
                   cosf(pitch_deg * (float)M_PI / 180.0f);
  const float dt = (float)(t_us - h.last_t_us) * 1e-6f;
  if (!h.valid || !(dt > 0.0f) || dt > HEIGHT_MAX_GAP_S) {
    // 무효 → 유효 첫 프레임은 차분하지 않는다. 직전 값과의 차가 수 m/s 로
    // 튀어 고도 유지에 들어가는 것을 막는다.
    h.vz_ms = 0.0f;
  } else {
    const float raw = (hv - h.h_m) / dt;
    h.vz_ms += (raw - h.vz_ms) * (dt / (HEIGHT_VZ_TAU_S + dt));
  }
  h.d_m = d;
  h.h_m = hv;
  h.last_t_us = t_us;
  h.valid = true;
}

// ---------------- 광류 속도 → 기체 수평 가속도 ----------------
//
// 기울어 가속하는 쿼드의 가속도계는 추력 방향(기체 z)만 본다. 상보필터는 이것을
// "수평"으로 읽어 기울기 추정을 0° 쪽으로 끌고 가고(시정수 약 2 s), 실제 기울기가
// 명령보다 커진다. 위치 유지 루프에 느린 적분기가 하나 더 생긴 셈이라 공기 저항이
// 작으면 발산한다(SIL 실측: 저항 0.05 N/(m/s) 에서 진폭 증가). 광류로 잰 수평
// 가속도를 가속도계 값에서 빼 주면 추정이 실제 기울기를 따라간다.
// 앞으로 가속(a_x>0)은 가속도계에 기수 들림 −a_x/g 로 보이므로 bodyAx 에 +a_x/g 를,
// 오른쪽 가속은 bodyAy 에 +a_y/g 를 더한다.

static const float FLOW_ACCEL_TAU_S = 0.2f;
static const float FLOW_ACCEL_MAX_MS2 = 3.0f;
// 유효 측정 사이 간격이 이보다 길면 차분하지 않는다. 프레임 하나가 빠지면
// 유효 측정 간격은 300 ms(빠진 구간 + 그 뒤 dt 로 거부되는 구간)가 된다.
static const float FLOW_ACCEL_MAX_GAP_S = 0.35f;

struct FlowAccelEstimator {
  bool valid;
  bool have_prev;
  float prev_c_roll, prev_c_pitch;
  // 보정 공간 c 의 차분. 실기체에서 ax = −a_전방, ay = +a_우측 이고, 이 부호라야
  // bodyAx(+a_전방/g)·bodyAy(−a_우측/g) 의 동적 성분을 더해서 상쇄한다.
  float ax_ms2, ay_ms2;
  uint32_t t_us;            // 마지막 유효 갱신 시각
};

static inline void flowAccelReset(FlowAccelEstimator &a) {
  a.valid = false;
  a.have_prev = false;
  a.prev_c_roll = a.prev_c_pitch = 0.0f;
  a.ax_ms2 = a.ay_ms2 = 0.0f;
  a.t_us = 0;
}

static inline float flowAccelClamp(float v) {
  return v > FLOW_ACCEL_MAX_MS2 ? FLOW_ACCEL_MAX_MS2
                                : (v < -FLOW_ACCEL_MAX_MS2 ? -FLOW_ACCEL_MAX_MS2 : v);
}

// 광류 구간마다 호출한다. 무효 구간은 추정을 바꾸지 않는다 — 프레임 하나 유실로
// 보정이 꺼지면 그때마다 자세 추정이 흔들린다. 만료는 호출자가 t_us 나이로 판단한다.
static inline void flowAccelUpdate(FlowAccelEstimator &a, bool meas_ok,
                                   float c_roll, float c_pitch, uint32_t t_us) {
  if (!meas_ok) return;
  const float dt = (float)(t_us - a.t_us) * 1e-6f;
  if (a.have_prev && dt > 0.0f && dt <= FLOW_ACCEL_MAX_GAP_S) {
    const float rawX = (c_pitch - a.prev_c_pitch) / dt;     // a_x = dc_pitch/dt
    const float rawY = -(c_roll - a.prev_c_roll) / dt;      // a_y = −dc_roll/dt
    if (a.valid) {
      const float k = dt / (FLOW_ACCEL_TAU_S + dt);
      a.ax_ms2 = flowAccelClamp(a.ax_ms2 + (rawX - a.ax_ms2) * k);
      a.ay_ms2 = flowAccelClamp(a.ay_ms2 + (rawY - a.ay_ms2) * k);
    } else {
      a.ax_ms2 = flowAccelClamp(rawX);
      a.ay_ms2 = flowAccelClamp(rawY);
    }
    a.valid = true;
  } else {
    a.valid = false;      // 첫 측정이거나 간격이 너무 길다: 다시 시작
  }
  a.have_prev = true;
  a.prev_c_roll = c_roll;
  a.prev_c_pitch = c_pitch;
  a.t_us = t_us;
}
