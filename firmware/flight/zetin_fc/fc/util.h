// zetin_fc/fc/util.h — 3. 유틸: LPF, raw/지연 링 함수, IMU freeze 감시, rcTimedOut.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once

// ==========================================================
// 3. 유틸
// ==========================================================
class LowPassFilter {
public:
  float alpha, last = 0.0f;
  LowPassFilter(float cutoff_hz, float dt) {
    float rc = 1.0f / (2.0f * PI * cutoff_hz);
    alpha = dt / (rc + dt);
  }
  float update(float in) { last += alpha * (in - last); return last; }
  void reset(float value = 0.0f) { last = value; }
};

// Arduino sketch preprocessor의 자동 함수 원형보다 먼저 보여야 하는 반환 타입.
using MotorMix = ControlAllocation;

struct ImuTelemetrySample {
  float imu1GyroX, imu1GyroY, imu1GyroZ;
  float imu1AccelX, imu1AccelY, imu1AccelZ;
  float imu2GyroX, imu2GyroY, imu2GyroZ;
  float imu2AccelX, imu2AccelY, imu2AccelZ;
};

// IMU별 freeze 감시 (raw 레지스터 값이 멈췄는지)
struct FreezeMon {
  int16_t  lastGyro[3] = {0, 0, 0};
  int16_t  lastAccel[3] = {0, 0, 0};
  uint32_t since = 0;
  bool     init  = false;
};

static inline uint32_t imuRawRingCount(const ImuRawRing &ring) {
  const uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE);
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  return head - tail;
}

static inline bool imuRawRingPush(
    ImuRawRing &ring, const ImuRawSample &sample, uint32_t sampleUs) {
  const uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_RELAXED);
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  if (head - tail >= IMU_RAW_RING_SIZE) {
    __atomic_fetch_add(&ring.dropped, 1, __ATOMIC_RELAXED);
    return false;
  }
  if (head == tail) {
    __atomic_store_n(&ring.first_t_us, sampleUs, __ATOMIC_RELAXED);
  }
  ring.samples[head & IMU_RAW_RING_MASK] = sample;
  __atomic_store_n(&ring.head, head + 1, __ATOMIC_RELEASE);
  return true;
}

static inline bool imuRawRingPop(ImuRawRing &ring, ImuRawSample &sample) {
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_RELAXED);
  if (tail == __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE)) return false;
  sample = ring.samples[tail & IMU_RAW_RING_MASK];
  __atomic_store_n(&ring.tail, tail + 1, __ATOMIC_RELEASE);
  return true;
}

static inline ImuRawSample makeImuRawSample(
    uint16_t dtUs, const inv_imu_sensor_event_t &e1,
    const inv_imu_sensor_event_t &e2, uint8_t failsafePhase) {
  ImuRawSample sample = {};
  sample.dt_us = dtUs;
  sample.failsafe_phase = failsafePhase;
  for (int axis = 0; axis < 3; axis++) {
    sample.imu1_gyro[axis] = e1.gyro[axis];
    sample.imu1_accel[axis] = e1.accel[axis];
    sample.imu2_gyro[axis] = e2.gyro[axis];
    sample.imu2_accel[axis] = e2.accel[axis];
  }
  return sample;
}

static inline size_t buildImuRawDatagram(
    ImuRawDatagram &datagram, uint32_t batchSeq, uint32_t baseTUs,
    uint32_t dropped, const ImuRawSample *samples, uint8_t nSamples) {
  if (nSamples == 0 || nSamples > IMU_RAW_BATCH_MAX) return 0;
  memcpy(datagram.header.magic, "ZIMU", 4);
  datagram.header.version = IMU_RAW_VERSION;
  datagram.header.n_samples = nSamples;
  datagram.header.reserved = 0;
  datagram.header.batch_seq = batchSeq;
  datagram.header.base_t_us = baseTUs;
  datagram.header.dropped = dropped;
  memcpy(datagram.samples, samples, nSamples * sizeof(ImuRawSample));
  return sizeof(ImuRawHeader) + nSamples * sizeof(ImuRawSample);
}

static inline ImuTelemetrySample makeImuTelemetrySample(
    const float g1[3], const float a1[3],
    const float g2[3], const float a2[3]) {
  return {
    g1[1], -g1[0], -g1[2],
    a1[1] * ACCEL_SCALE, -a1[0] * ACCEL_SCALE, a1[2] * ACCEL_SCALE,
    g2[1], -g2[0], -g2[2],
    a2[1] * ACCEL_SCALE, -a2[0] * ACCEL_SCALE, a2[2] * ACCEL_SCALE
  };
}
#if WIFI_LATENCY_DEBUG
static inline void recordLatEvent(LatRing &ring, uint32_t tMs,
                                  uint32_t durUs, LatKind kind) {
  uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_RELAXED);
  uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  if (head - tail >= LAT_RING_SIZE) {
    __atomic_fetch_add(&ring.dropped, 1, __ATOMIC_RELAXED);
    return;
  }
  ring.events[head & LAT_RING_MASK] = {tMs, durUs, (uint8_t)kind};
  __atomic_store_n(&ring.head, head + 1, __ATOMIC_RELEASE);
}

static inline bool popLatEvent(LatRing &ring, LatEvt &event) {
  uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_RELAXED);
  if (tail == __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE)) return false;
  event = ring.events[tail & LAT_RING_MASK];
  __atomic_store_n(&ring.tail, tail + 1, __ATOMIC_RELEASE);
  return true;
}

static const char *latKindName(uint8_t kind) {
  switch (kind) {
    case LAT_UDP_LOOP:  return "UDP_LOOP";
    case LAT_UDP_PARSE: return "UDP_PARSE";
    case LAT_UDP_SEND:  return "UDP_SEND";
    case LAT_PID:       return "PID";
    default:            return "UNKNOWN";
  }
}

static void drainLatRing(LatRing &ring) {
  LatEvt event;
  while (popLatEvent(ring, event)) {
    if (event.kind == LAT_PID) {
      if (event.dur_us > latPidMaxUs) latPidMaxUs = event.dur_us;
      if (event.dur_us > 5000U) latPidOver5++;
    } else {
      if (event.dur_us > latUdpMaxUs) latUdpMaxUs = event.dur_us;
      if (event.kind == LAT_UDP_LOOP) {
        if (event.dur_us > 30000U) latUdpLoopOver30++;
        if (event.dur_us > 100000U) latUdpLoopOver100++;
        if (event.dur_us > 300000U) latUdpLoopOver300++;
      }
    }
    Serial.printf("[LAT] t=%lu kind=%s dur_us=%lu\n",
                  (unsigned long)event.t_ms, latKindName(event.kind),
                  (unsigned long)event.dur_us);
  }
}
#endif
static bool checkFreeze(FreezeMon &m, const inv_imu_sensor_event_t &e, uint32_t nowMs) {
  if (!m.init) {
    for (int k = 0; k < 3; k++) {
      m.lastGyro[k] = e.gyro[k];
      m.lastAccel[k] = e.accel[k];
    }
    m.init = true;
    return false;
  }

  // abs(x)+abs(y)+abs(z)의 변화만 보면 축 변화가 서로 상쇄될 수 있다.
  // 각 레지스터의 변화량을 직접 더해 통신 정지/동일 프레임 반복을 감시한다.
  int32_t delta = 0;
  for (int k = 0; k < 3; k++) {
    delta += abs((int32_t)e.gyro[k] - m.lastGyro[k]);
    delta += abs((int32_t)e.accel[k] - m.lastAccel[k]);
  }

  bool frozen = false;
  if (delta <= FROZEN_DELTA_RAW) {
    if (m.since == 0) m.since = nowMs;
    else if (nowMs - m.since >= IMU_FROZEN_MS) frozen = true;
  } else {
    m.since = 0;
  }
  for (int k = 0; k < 3; k++) {
    m.lastGyro[k] = e.gyro[k];
    m.lastAccel[k] = e.accel[k];
  }
  return frozen;
}

// RC 워치독. nowMs는 루프 맨 위에서 뜬 값이라 udp_task(코어0)가 그 뒤에
// lastRcMs를 갱신하면 lastRcMs가 nowMs보다 미래일 수 있다. 부호 있는 차이로
// 비교해야 unsigned 언더플로로 즉시 타임아웃이 터지지 않는다. millis() wrap도
// 모듈러 뺄셈이라 그대로 안전하다.
static inline bool rcTimedOut(uint32_t nowMs, uint32_t lastMs) {
  return (int32_t)(nowMs - lastMs) > (int32_t)RC_TIMEOUT_MS;
}

