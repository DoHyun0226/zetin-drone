// zetin_fc/fc/safety_requests.h — 무장·해제·resume 요청과 Core 1 적용, 하강 중 설정 변경 거부, resume 거부 사유.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// ---------- Core 0 ↔ Core 1 공유 상태 접근 함수 ----------
// portENTER_CRITICAL/EXIT: 스핀락으로 임계구역을 만든다. 그 사이 코드는 다른 코어와 동시에 실행되지 않는다
//  안에서는 블로킹 작업(I2C, 시리얼 출력) 없이 변수 복사만 짧게 한다 → pid_task가 오래 기다리지 않게
//
// "stop": 시동 해제 요청. 대기 중인 다른 요청은 모두 취소 (disarm이 항상 우선)
static inline void requestSafetyDisarm() {
  portENTER_CRITICAL(&safetyRequestMux);
  safety_disarm_requested = true;
  safety_arm_requested = false;
  failsafe_resume_requested = false;
  hover_reset_requested = false;
  portEXIT_CRITICAL(&safetyRequestMux);
}

// "start": 시동 요청. 이미 disarm 요청이 대기 중이면 무시 (stop이 이긴다). 호버 추정 초기화도 함께 요청
static inline void requestSafetyArm() {
  portENTER_CRITICAL(&safetyRequestMux);
  if (!safety_disarm_requested) {
    hover_reset_requested = true;
    safety_arm_requested = true;
  }
  portEXIT_CRITICAL(&safetyRequestMux);
}

// 게인/트림/자기 보정 같은 설정 변경 허용 여부. 자동착륙 하강 중에는 거부 (하강 도중 동작이 바뀌지 않게)
static inline bool configChangeAllowed(const char *setting) {
  if (fs_phase != FS_DESCENDING) return true;
  Serial.printf(">>> %s refused (auto-land descending)\n", setting);
  return false;
}

// [Core 1, 매 tick] 대기 중인 요청을 실제 safety_lock에 반영한다. disarm 요청이 있으면 arm보다 먼저 처리
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

// "resume" 요청을 걸어 둔다. 실제 허용 여부는 pid_task가 resumeRefusalReason()으로 다시 확인한다
static inline void requestFailsafeResume() {
  portENTER_CRITICAL(&safetyRequestMux);
  failsafe_resume_requested = true;
  portEXIT_CRITICAL(&safetyRequestMux);
}

// 요청이 있었는지 읽으면서 동시에 지운다 (한 요청이 한 번만 처리되게: "take")
static inline bool takeFailsafeResumeRequest() {
  portENTER_CRITICAL(&safetyRequestMux);
  const bool requested = failsafe_resume_requested;
  failsafe_resume_requested = false;
  portEXIT_CRITICAL(&safetyRequestMux);
  return requested;
}

// resume을 허용할지 검사. 위에서부터 순서대로 확인해 처음 걸린 거부 사유를 돌려준다 (전부 통과하면 ALLOWED)
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
