// zetin_fc/fc/imu_snapshots.h — 1 kHz IMU 샘플·raw 레지스터 게시(Core 1 → Core 0).
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
static inline void publishImuSample(const ImuTelemetrySample &sample) {
  portENTER_CRITICAL(&imuSampleMux);
  imuSampleSnapshot = sample;
  portEXIT_CRITICAL(&imuSampleMux);
}

static inline ImuTelemetrySample readImuSampleSnapshot() {
  portENTER_CRITICAL(&imuSampleMux);
  const ImuTelemetrySample sample = imuSampleSnapshot;
  portEXIT_CRITICAL(&imuSampleMux);
  return sample;
}

static inline void publishImuRawRegisters(
    const inv_imu_sensor_event_t &e1, const inv_imu_sensor_event_t &e2,
    uint32_t sampleUs, uint8_t failsafePhase) {
  uint16_t dtUs = 0;
  if (rawProducerTimeValid) {
    const uint32_t elapsedUs = sampleUs - rawProducerLastUs;
    dtUs = elapsedUs > 65535UL
        ? 0xFFFFU
        : static_cast<uint16_t>(elapsedUs);
  }
  const ImuRawSample sample = makeImuRawSample(dtUs, e1, e2, failsafePhase);
  if (imuRawRingPush(imuRawRing, sample, sampleUs)) {
    rawProducerLastUs = sampleUs;
    rawProducerTimeValid = true;
  }
}

