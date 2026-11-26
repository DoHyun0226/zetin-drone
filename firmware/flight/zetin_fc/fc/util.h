// zetin_fc/fc/util.h — 3. 유틸: LPF, raw/지연 링 함수, IMU freeze 감시, rcTimedOut.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once

// ==========================================================
// 3. 유틸
// ==========================================================
// 1차 저역통과 필터(LPF): 빠르게 흔들리는 노이즈는 줄이고 느린 변화만 통과시킨다
//  RC 회로와 같은 식: rc = 1/(2π·fc), alpha = dt/(rc+dt). 매 호출마다 출력이 입력 쪽으로 alpha 비율만큼 다가간다
//  예) fc 80Hz, dt 1ms → alpha ≈ 0.33 / fc 40Hz → alpha ≈ 0.20 (차단 주파수가 낮을수록 더 부드럽고 더 늦다)
class LowPassFilter {
public:
  // alpha: 반영 비율, last: 직전 출력(필터 상태)
  float alpha, last = 0.0f;
  LowPassFilter(float cutoff_hz, float dt) {
    // cutoff_hz를 설정했을 때 RC 회로의 시정수(계산해보면 나옴)
    float rc = 1.0f / (2.0f * PI * cutoff_hz);
    alpha = dt / (rc + dt);
  }
  // 새 입력을 넣고 필터 출력을 돌려준다
  float update(float in) { last += alpha * (in - last); return last; }
  // 상태 초기화 (잠금/재시동 시 이전 값이 튀지 않게)
  void reset(float value = 0.0f) { last = value; }
};

// Arduino sketch preprocessor의 자동 함수 원형보다 먼저 보여야 하는 반환 타입.
// 예전 이름(MotorMix)을 새 allocator 결과 구조체의 별칭으로 유지
using MotorMix = ControlAllocation;

// 텔레메트리용 IMU별 값 (IMU1 축 기준 → body 축으로 변환된 값). gyro는 dps, accel은 g
struct ImuTelemetrySample {
  float imu1GyroX, imu1GyroY, imu1GyroZ;
  float imu1AccelX, imu1AccelY, imu1AccelZ;
  float imu2GyroX, imu2GyroY, imu2GyroZ;
  float imu2AccelX, imu2AccelY, imu2AccelZ;
};

// IMU별 freeze 감시 (raw 레지스터 값이 멈췄는지)
struct FreezeMon {
  // 직전 tick 자이로 레지스터 원값
  int16_t  lastGyro[3] = {0, 0, 0};
  // 직전 tick 가속도 레지스터 원값
  int16_t  lastAccel[3] = {0, 0, 0};
  // 값 변화가 없기 시작한 시각 (0 = 지금 변하는 중)
  uint32_t since = 0;
  // 첫 값을 받았는가
  bool     init  = false;
};

// [링 버퍼 lock-free 동작] 생산자(Core 1)는 head만, 소비자(Core 0)는 tail만 증가시킨다
//  head, tail은 계속 커지는 카운터이고 실제 칸 위치는 & MASK로 구한다. (head - tail) = 들어 있는 개수
//  __atomic_*: 두 코어가 동시에 읽고 쓰는 변수를 안전하게 다루는 GCC 내장 함수
//   - RELEASE(쓰기): 이 저장 전에 한 메모리 쓰기(샘플 복사)가 다른 코어에 먼저 보이도록 보장
//   - ACQUIRE(읽기): 이 읽기 뒤의 메모리 읽기가 상대가 RELEASE 전에 쓴 내용을 보도록 보장
//   - RELAXED: 순서 보장 없이 값만 원자적으로 (자기 자신만 쓰는 변수나 통계 카운터)
//  이 짝 덕분에 mutex 없이도 "샘플을 다 쓴 뒤에 head가 올라간 것"을 소비자가 보게 된다
//
// 링 버퍼에 쌓여 있는 샘플 수
static inline uint32_t imuRawRingCount(const ImuRawRing &ring) {
  const uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE);
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  return head - tail;
}

// [생산자, Core 1] 샘플 1개를 넣는다. 가득 차 있으면 버리고 dropped를 1 올린다 (pid_task를 기다리게 하지 않음)
static inline bool imuRawRingPush(
    ImuRawRing &ring, const ImuRawSample &sample, uint32_t sampleUs) {
  const uint32_t head = __atomic_load_n(&ring.head, __ATOMIC_RELAXED);
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_ACQUIRE);
  if (head - tail >= IMU_RAW_RING_SIZE) {
    __atomic_fetch_add(&ring.dropped, 1, __ATOMIC_RELAXED);
    return false;
  }
  if (head == tail) {
    // 비어 있던 버퍼의 첫 샘플이면 절대 시각을 기록 (소비자가 dt_us를 누적해 시각을 복원하는 기준점)
    __atomic_store_n(&ring.first_t_us, sampleUs, __ATOMIC_RELAXED);
  }
  // 먼저 샘플을 복사하고
  ring.samples[head & IMU_RAW_RING_MASK] = sample;
  // 그다음 head를 올려 공개한다
  __atomic_store_n(&ring.head, head + 1, __ATOMIC_RELEASE);
  return true;
}

// [소비자, Core 0] 샘플 1개를 꺼낸다. 비어 있으면 false
static inline bool imuRawRingPop(ImuRawRing &ring, ImuRawSample &sample) {
  const uint32_t tail = __atomic_load_n(&ring.tail, __ATOMIC_RELAXED);
  if (tail == __atomic_load_n(&ring.head, __ATOMIC_ACQUIRE)) return false;
  sample = ring.samples[tail & IMU_RAW_RING_MASK];
  __atomic_store_n(&ring.tail, tail + 1, __ATOMIC_RELEASE);
  return true;
}

// 두 IMU의 레지스터 원값(보정 전)과 시간 간격, 자동착륙 단계를 27B 샘플 하나로 묶는다
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

// 헤더(ZIMU)를 채우고 샘플 n개를 복사해 보낼 패킷을 만든다. 반환값 = 실제 보낼 바이트 수 (20 + 27×n)
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

// IMU1 센서 축 값을 body 축(텔레메트리 기준)으로 바꾼다. pid_task의 bodyGx/bodyGy/bodyGz 변환과 같은 부호
//  body x = 센서 y, body y = -센서 x, gyro z는 -센서 z (accel z는 +). accel은 여기서 ACCEL_SCALE을 곱해 g 단위로
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
// [IMU freeze 감시] 센서 값이 IMU_FROZEN_MS(300ms) 동안 사실상 안 변하면 true
//  살아 있는 센서는 정지 상태에서도 노이즈로 값이 계속 조금씩 바뀐다 → 완전히 같은 값 반복 = SPI 통신 끊김/센서 정지
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
