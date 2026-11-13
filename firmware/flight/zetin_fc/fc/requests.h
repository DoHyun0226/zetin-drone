// zetin_fc/fc/requests.h — 안전·resume·자기계·IMU 스냅숏 요청과 게시.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
static inline void requestSafetyDisarm() {
  portENTER_CRITICAL(&safetyRequestMux);
  safety_disarm_requested = true;
  safety_arm_requested = false;
  failsafe_resume_requested = false;
  hover_reset_requested = false;
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline void requestSafetyArm() {
  portENTER_CRITICAL(&safetyRequestMux);
  if (!safety_disarm_requested) {
    hover_reset_requested = true;
    safety_arm_requested = true;
  }
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline bool configChangeAllowed(const char *setting) {
  if (fs_phase != FS_DESCENDING) return true;
  Serial.printf(">>> %s refused (auto-land descending)\n", setting);
  return false;
}

static inline void applyPendingSafetyRequest() {
  portENTER_CRITICAL(&safetyRequestMux);
  if (safety_disarm_requested) {
    safety_lock = true;
    safety_disarm_requested = false;
    safety_arm_requested = false;
    hover_reset_requested = false;
  } else if (safety_arm_requested) {
    if (hover_reset_requested) {
      // Core 1이 reset을 끝낸 뒤에만 safety_lock=false를 공개한다.
      hover_valid = false;
      hover_est = 0.0f;
      hoverTracker = {};
    }
    hover_reset_requested = false;
    safety_lock = false;
    safety_arm_requested = false;
  }
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline void requestFailsafeResume() {
  portENTER_CRITICAL(&safetyRequestMux);
  failsafe_resume_requested = true;
  portEXIT_CRITICAL(&safetyRequestMux);
}

static inline bool takeFailsafeResumeRequest() {
  portENTER_CRITICAL(&safetyRequestMux);
  const bool requested = failsafe_resume_requested;
  failsafe_resume_requested = false;
  portEXIT_CRITICAL(&safetyRequestMux);
  return requested;
}

static inline ResumeRefusalReason resumeRefusalReason(uint32_t nowMs) {
  if (fs_phase != FS_DESCENDING) return RESUME_REFUSED_PHASE;
  if (fs_first_enter_valid &&
      (uint32_t)(nowMs - fs_first_enter_ms) >= FS_RESUME_MAX_MS) {
    return RESUME_REFUSED_CUMULATIVE;
  }
  if (rcTimedOut(nowMs, lastRcMs)) return RESUME_REFUSED_RC;
  if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
    return RESUME_REFUSED_TILT;
  }
  if (active_imus <= 0
      || (imu1_frozen_now && imu2_frozen_now)
      || imu_disagree_now) {
    return RESUME_REFUSED_IMU;
  }
  if (!hover_valid) return RESUME_REFUSED_HOVER;
  return RESUME_ALLOWED;
}

static inline void logResumeRefusal(ResumeRefusalReason reason) {
  Serial.printf(">>> RESUME REFUSED %s\n", resumeRefusalName(reason));
}

static inline void publishMagSample(
    float x, float y, float z, uint32_t sample_ms) {
  // I2C read happens before this lock. The critical section only publishes
  // four words, so pid_task never waits on the blocking transaction.
  portENTER_CRITICAL(&magSnapshotMux);
  magFieldX = x;
  magFieldY = y;
  magFieldZ = z;
  magSampleMs = sample_ms;
  magSampleValid = true;
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline bool readMagSnapshot(MagSnapshot &snapshot) {
  portENTER_CRITICAL(&magSnapshotMux);
  bool valid = magSampleValid;
  snapshot.x = magFieldX;
  snapshot.y = magFieldY;
  snapshot.z = magFieldZ;
  snapshot.sample_ms = magSampleMs;
  portEXIT_CRITICAL(&magSnapshotMux);
  return valid;
}

static inline void publishMagTelemetry(
    float x, float y, float z, bool calibration_raw) {
  portENTER_CRITICAL(&magSnapshotMux);
  // Core 1 may already have entered its normal-domain block when Core 0
  // starts calibration. Do not let that late writer overwrite a raw snapshot.
  if (calibration_raw || !mag_calibrating) {
    magTelemX = x;
    magTelemY = y;
    magTelemZ = z;
    magTelemCalActive = calibration_raw;
  }
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline void readMagTelemetry(
    float &x, float &y, float &z, bool &calibration_raw) {
  portENTER_CRITICAL(&magSnapshotMux);
  x = magTelemX;
  y = magTelemY;
  z = magTelemZ;
  calibration_raw = magTelemCalActive;
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline void requestMagReferenceUpdate() {
  portENTER_CRITICAL(&magSnapshotMux);
  mag_reference_pending = true;
  portEXIT_CRITICAL(&magSnapshotMux);
}

static inline bool takeMagReferenceRequest() {
  portENTER_CRITICAL(&magSnapshotMux);
  const bool requested = mag_reference_pending;
  mag_reference_pending = false;
  portEXIT_CRITICAL(&magSnapshotMux);
  return requested;
}

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

