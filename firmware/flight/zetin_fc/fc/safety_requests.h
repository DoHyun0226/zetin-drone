// zetin_fc/fc/safety_requests.h — 무장·해제·resume 요청과 Core 1 적용, 하강 중 설정 변경 거부, resume 거부 사유.
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
