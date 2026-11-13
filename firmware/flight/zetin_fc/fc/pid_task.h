// zetin_fc/fc/pid_task.h — 8. pid_task (Core 1, 1 kHz).
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// ==========================================================
// 8. PID 태스크 (Core 1, 1kHz)
// ==========================================================
// pid_task 의 루프 몸통을 실행 순서 그대로 단계 함수로 나눴다. 단계 경계는 실제 순서를
// 따른다 — 안전·resume → 광류·거리 갱신과 유지 후보 → 호버·RC 상실·하강 → 유지 적용 확정
// 순이며, RC 상실 진입은 이번 tick 에 갱신한 높이·거리 추적·호버 유효를 쓴다.
// - 루프 밖 지역(필터, 감시기, 분주 카운터, 착지 감지기 …)은 PidLoopState: pid_task 가 시작할
//   때 스택에 한 번 만든다(정적·전역이 아니다). 초기값과 순서는 원래 선언 그대로.
// - 단계를 건너는 tick 지역은 PidTick: 매 tick 새로 만든다. 단계 안에서만 쓰는 지역은 그
//   단계의 지역으로 남는다. 공유 전역은 구조체로 복사하지 않고 원래 자리에서 읽는다.
// - 단계 함수는 always_inline 이라 스택 프레임은 pid_task 하나다(태스크 스택 4096 B).
// - 원래 continue 세 곳(쓸 IMU 없음, 잠금, 출력 직전 잠금)은 PID_TICK_ENDED 를 돌려주고,
//   호출부가 그 tick 의 나머지 단계를 건너뛴다.
// 검증: tools/diff_firmware_behaviour.py(차분 하네스), tools/prove_same_firmware.py.

enum PidTickFlow : uint8_t { PID_TICK_GO_ON = 0, PID_TICK_ENDED = 1 };

// pid_task 의 루프 밖 지역(태스크 수명 동안 유지).
struct PidLoopState {
  const TickType_t period = pdMS_TO_TICKS(1);   // vTaskDelayUntil 기반 (busy-wait 제거)
  TickType_t wake = xTaskGetTickCount();
  const float dt = 0.001f;

  LowPassFilter lpfD_Roll{40, dt}, lpfD_Pitch{40, dt}, lpfD_Yaw{40, dt};
  LowPassFilter lpfG1[3] = {{GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}};
  LowPassFilter lpfG2[3] = {{GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}, {GYRO_SW_LPF_HZ, dt}};
  float prevGyroX = 0.0f, prevGyroY = 0.0f, prevGyroZ = 0.0f;

  FreezeMon fm1, fm2;
  uint32_t disagreeSince = 0;
  int altHoldAppliedPrevUs = 0;
  bool prevHoldNonZero = false;
  int lastFlowReject = FLOW_REJECT_FIRST;
  float lastFlowDtMs = 0.0f, lastFlowCRoll = 0.0f, lastFlowCPitch = 0.0f;

  float targetRateRoll = 0, targetRatePitch = 0, targetRateYaw = 0;
  uint8_t outerCnt = 0;
  uint8_t magFusionCnt = 0;
  uint32_t lastMicros = micros();
  uint32_t loopCount = 0;
  uint32_t loopMarkerMs = millis();
  bool wasLocked = true;
  LandDetector landDet = {};
  LandProbeConfig landProbeConfig = FS_LAND_PROBE_CONFIG;
  int fs_probe_dip_us = FS_PROBE_DIP_MIN_US;
  uint32_t fs_enter_ms = 0;
  float fs_hold_yaw = 0.0f;
  bool fs_probe_blocked_logged = false;
  YawAuthorityTracker yawAuthority = {};

  inv_imu_sensor_event_t e1 = {}, e2 = {};
};

// 한 tick 안에서 단계를 건너는 지역. 매 tick 새로 만든다(원래 선언 자리에서 값을 쓴다).
struct PidTick {
  uint32_t nowMs;
  bool checkProbeDelivery;
  int probeReferenceThrottle;
  uint32_t probeElapsedMs;
  uint32_t nowUs;
  float realDt;
  float rawDt;
  float bodyGx;
  float bodyGy;
  float bodyGz;
  float bodyAx;
  float bodyAy;
  float bodyAz;
  bool rlRangeFresh;
  RangeLandOutput rlOut;
  bool rlStepped;
  bool heightFresh;
  PosHoldOutput phOut;
  AltHoldOutput ahOut;
  int rlAltUs;
  float rlHoldScale;
  float holdRollDeg;
  float holdPitchDeg;
  int altHoldUs;
  bool forceOuterRecompute;
  float eRoll;
  float ePitch;
  float eYaw;
  float iRollUsed;
  float iPitchUsed;
  float iYawUsed;
  float pidRoll;
  float pidPitch;
  float pidYaw;
};

// tick 시작: 지연 보정, 무장·스위치 요청 적용, 루프 Hz, dt.
static inline __attribute__((always_inline)) void pidBeginTick(PidLoopState &L, PidTick &T) {
  auto &period = L.period;
  auto &wake = L.wake;
  auto &dt = L.dt;
  auto &lastMicros = L.lastMicros;
  auto &loopCount = L.loopCount;
  auto &loopMarkerMs = L.loopMarkerMs;
  auto &nowMs = T.nowMs;
  auto &checkProbeDelivery = T.checkProbeDelivery;
  auto &probeReferenceThrottle = T.probeReferenceThrottle;
  auto &probeElapsedMs = T.probeElapsedMs;
  auto &nowUs = T.nowUs;
  auto &realDt = T.realDt;
  auto &rawDt = T.rawDt;
  TickType_t afterWake = xTaskGetTickCount();
  if ((TickType_t)(afterWake - wake) > period) {
    // 큰 지연 뒤 밀린 tick을 연속 실행하지 않는다(센서 stale read/D항 spike 방지).
    wake = afterWake;
  }
  nowMs = millis();
  checkProbeDelivery = false;
  probeReferenceThrottle = 0;
  probeElapsedMs = 0;
  applyRangeLandRequest();
  applyPendingSafetyRequest();
  loopCount++;
  uint32_t loopElapsedMs = nowMs - loopMarkerMs;
  if (loopElapsedMs >= 1000) {
    pidLoopHz = (int)((loopCount * 1000UL) / loopElapsedMs);
    loopCount = 0;
    loopMarkerMs = nowMs;
  }
  nowUs = micros();
  realDt = (nowUs - lastMicros) / 1e6f;
  // 광류 회전 상쇄는 실제 경과 시간이 필요하다. 아래 치환(>10 ms → 1 ms)을
  // 이력 적분에 쓰면 공백 동안의 회전을 잃는다.
  rawDt = realDt;
#if WIFI_LATENCY_DEBUG
  uint32_t rawDtUs = nowUs - lastMicros;
  if (rawDtUs > WIFI_LATENCY_PID_THRESHOLD_US) {
    recordLatEvent(pidLatRing, nowMs, rawDtUs, LAT_PID);
  }
#endif
  lastMicros = nowUs;
  // 지연 후 vTaskDelayUntil이 catch-up할 때 지나치게 작은 dt로 D항이 튀는 것을 방지.
  if (realDt < 0.0002f || realDt > 0.01f) realDt = dt;
}

// 센서 읽기, freeze·불일치, IMU 융합, body 좌표 변환, 광류 자이로 이력. 쓸 IMU 가 없으면 잠그고 이번 tick 을 끝낸다.
static inline __attribute__((always_inline)) PidTickFlow pidReadImus(PidLoopState &L, PidTick &T) {
  auto &lpfD_Roll = L.lpfD_Roll;
  auto &lpfD_Pitch = L.lpfD_Pitch;
  auto &lpfD_Yaw = L.lpfD_Yaw;
  auto &lpfG1 = L.lpfG1;
  auto &lpfG2 = L.lpfG2;
  auto &fm1 = L.fm1;
  auto &fm2 = L.fm2;
  auto &disagreeSince = L.disagreeSince;
  auto &altHoldAppliedPrevUs = L.altHoldAppliedPrevUs;
  auto &prevHoldNonZero = L.prevHoldNonZero;
  auto &targetRateRoll = L.targetRateRoll;
  auto &targetRatePitch = L.targetRatePitch;
  auto &targetRateYaw = L.targetRateYaw;
  auto &wasLocked = L.wasLocked;
  auto &yawAuthority = L.yawAuthority;
  auto &e1 = L.e1;
  auto &e2 = L.e2;
  auto &nowMs = T.nowMs;
  auto &nowUs = T.nowUs;
  auto &rawDt = T.rawDt;
  auto &bodyGx = T.bodyGx;
  auto &bodyGy = T.bodyGy;
  auto &bodyGz = T.bodyGz;
  auto &bodyAx = T.bodyAx;
  auto &bodyAy = T.bodyAy;
  auto &bodyAz = T.bodyAz;
  // ---------- 센서 읽기 ----------
  int readStatus1 = IMU1.getDataFromRegisters(e1);
  int readStatus2 = IMU2.getDataFromRegisters(e2);
  imuCalStep(readStatus1 == 0 && readStatus2 == 0, e1, e2);

  // 읽기 오류 시 event가 갱신되지 않으므로 동일 프레임으로 취급되어 freeze로 귀결된다.
  // fault는 비행 중 자동 복구하지 않고 다음 수동 start 때만 재평가/해제한다.
  bool frozen1 = checkFreeze(fm1, e1, nowMs);
  bool frozen2 = checkFreeze(fm2, e2, nowMs);
  imu1_frozen_now = frozen1;
  imu2_frozen_now = frozen2;
  if (frozen1 && !fault_imu1) Serial.printf("[FAULT] IMU1 FROZEN (read=%d)\n", readStatus1);
  if (frozen2 && !fault_imu2) Serial.printf("[FAULT] IMU2 FROZEN (read=%d)\n", readStatus2);
  if (frozen1) fault_imu1 = true;
  if (frozen2) fault_imu2 = true;

  // OFF hot path는 이 bool 검사 하나뿐이다. ON일 때만 micros()와 27B 복사를 한다.
  // freeze 판정 직후, 부호/scale/bias/LPF/body 변환 전의 레지스터 원값을 게시한다.
  if (__atomic_load_n(&raw_stream_enabled, __ATOMIC_ACQUIRE)
      && connectionEstablished) {
    // fs_phase 는 이 함수보다 뒤에서 갱신되므로 직전 tick 값이다(1ms).
    publishImuRawRegisters(e1, e2, micros(), (uint8_t)fs_phase);
  }

  // IMU1 sensor frame 기준 각속도 (IMU2 축 정렬 + 개별 bias 보정)
  float g1[3] = {
    e1.gyro[0]*GYRO_SCALE - gyro_bias1[0],
    e1.gyro[1]*GYRO_SCALE - gyro_bias1[1],
    e1.gyro[2]*GYRO_SCALE - gyro_bias1[2] };
  float g2[3] = {
    IMU2_SIGN_X*e2.gyro[0]*GYRO_SCALE - gyro_bias2[0],
    IMU2_SIGN_Y*e2.gyro[1]*GYRO_SCALE - gyro_bias2[1],
    IMU2_SIGN_Z*e2.gyro[2]*GYRO_SCALE - gyro_bias2[2] };
  // 소프트웨어 LPF. 제어와 IMU 불일치 판정 모두 필터된 각속도를 쓴다.
  // (진동으로 순간 차이가 15dps를 넘어 비행 중 disagree 컷이 나는 것을 방지)
  for (int k = 0; k < 3; k++) {
    g1[k] = lpfG1[k].update(g1[k]);
    g2[k] = lpfG2[k].update(g2[k]);
  }
  float a1[3] = {
    (float)e1.accel[0] * accel_scale1,
    (float)e1.accel[1] * accel_scale1,
    (float)e1.accel[2] * accel_scale1 };
  float a2[3] = {
    IMU2_SIGN_X*e2.accel[0] * accel_scale2,
    IMU2_SIGN_Y*e2.accel[1] * accel_scale2,
    IMU2_SIGN_Z*e2.accel[2] * accel_scale2 };

  // LPF 후 gyro와 LPF 없는 accel을 같은 1kHz 프레임에서 body 좌표로 게시한다.
  // 변환 계산은 락 밖에서 끝내고 임계구역에서는 float 12개만 복사한다.
  const ImuTelemetrySample currentImuSample =
      makeImuTelemetrySample(g1, a1, g2, a2);
  publishImuSample(currentImuSample);

  // 현재 샘플 불일치는 재시동 거부 판단에도 사용한다.
  bool disagreeSample = false;
  if (!frozen1 && !frozen2) {
    for (int k = 0; k < 3; k++) {
      if (fabsf(g1[k] - g2[k]) > GYRO_DISAGREE_DPS) disagreeSample = true;
    }
  }
  imu_disagree_now = disagreeSample;

  // latch된 센서는 비행 중 값이 다시 움직여도 자동 재투입하지 않는다.
  bool h1 = !fault_imu1, h2 = !fault_imu2;

  // ---------- IMU 융합 (진짜 redundancy) ----------
  // 둘 다 정상: 불일치 검사 후 평균 / 하나만 정상: 그것만 / 중재 불가: fault
  if (h1 && h2) {
    if (disagreeSample) {
      if (disagreeSince == 0) disagreeSince = nowMs;
      else if (nowMs - disagreeSince >= IMU_DISAGREE_MS) {
        if (!fault_disagree) Serial.println("[FAULT] DUAL IMU DISAGREEMENT");
        fault_disagree = true;
      }
    } else {
      disagreeSince = 0;
    }
  } else {
    disagreeSince = 0;
  }

  float fg[3], fa[3];
  if (h1 && h2 && !fault_disagree) {
    for (int k = 0; k < 3; k++) { fg[k] = (g1[k]+g2[k])*0.5f; fa[k] = (a1[k]+a2[k])*0.5f; }
    active_imus = 2;
  } else if (h1 && !h2) {
    for (int k = 0; k < 3; k++) { fg[k] = g1[k]; fa[k] = a1[k]; }
    active_imus = 1;
  } else if (h2 && !h1) {
    for (int k = 0; k < 3; k++) { fg[k] = g2[k]; fa[k] = a2[k]; }
    active_imus = 1;
  } else {
    // 둘 다 freeze 또는 서로 안 맞음 -> 신뢰 불가
    active_imus = 0;
    if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;
    safety_lock = true;
    // 이 경로는 hold 처리 전에 빠지므로 시동 경계를 hold 가 못 본다. 여기서
    // 지워야 재무장 때 이전 보정이 되살아나지 않는다(재검토 코드 P1-1).
    posHoldReset(posHold);
    altHoldReset(altHold);
    flowAccelReset(flowAccel);
    flowAccelUsable = false;
    altHoldAppliedPrevUs = 0;
    // 이 동안 자이로 이력이 쌓이지 않는다. 이력을 지우지 않으면 재무장 뒤 링 보간이
    // 그 공백을 회전 0 으로 메워 가짜 병진 속도가 된다(코드 재검토 P2).
    flowGyroReset(flowGyro);
    flowOdomReset(flowOdomState);
    prevHoldNonZero = false;
    mixer_scaled = false;
    iTermRoll = iTermPitch = iTermYaw = 0.0f;
    targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
    resetYawAuthority(yawAuthority);
    yaw_authority_state = YAW_AUTH_NORMAL;
    resetAllocationTelemetry();
    lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
    enterLockedState(wasLocked);
    motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
    tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
    stopMotors();
    return PID_TICK_ENDED;
  }

  // Roll/pitch gyro now matches the validated accel transform; the inverted
  // signs caused estimator lag and rate-loop anti-damping.
  // bodyGz stays unchanged pending separate bench yaw re-verification.
  bodyGx =  fg[1];
  bodyGy = -fg[0];
  bodyGz = -fg[2];
  bodyAx =  fa[1] * ACCEL_SCALE;
  bodyAy = -fa[0] * ACCEL_SCALE;
  bodyAz =  fa[2] * ACCEL_SCALE;
  gyroX = bodyGx; gyroY = bodyGy; gyroZ = bodyGz;
  accX = bodyAx; accY = bodyAy; accZ = bodyAz;
  // 광류 회전 상쇄용 roll/pitch 누적각 이력(rad). 시동 해제 중에도 쌓는다.
  // 10 ms 넘는 공백은 그 사이 각속도를 모르므로 이력과 광류 구간을 새로 시작한다.
  if (!(rawDt > 0.0f) || rawDt > 0.01f) {
    flowGyroReset(flowGyro);
    flowOdomReset(flowOdomState);
    flowGyroPush(flowGyro, nowUs, 0.0f, 0.0f, 0.0f);
  } else {
    flowGyroPush(flowGyro, nowUs, bodyGx * (PI / 180.0f),
                 bodyGy * (PI / 180.0f), rawDt);
  }
  return PID_TICK_GO_ON;
}

// 상보필터 자세 추정, 자기계 yaw 융합(250 Hz).
static inline __attribute__((always_inline)) void pidEstimateAttitude(PidLoopState &L, PidTick &T) {
  auto &altHoldAppliedPrevUs = L.altHoldAppliedPrevUs;
  auto &magFusionCnt = L.magFusionCnt;
  auto &nowMs = T.nowMs;
  auto &realDt = T.realDt;
  auto &bodyGx = T.bodyGx;
  auto &bodyGy = T.bodyGy;
  auto &bodyGz = T.bodyGz;
  auto &bodyAx = T.bodyAx;
  auto &bodyAy = T.bodyAy;
  auto &bodyAz = T.bodyAz;
  // ---------- 자세 추정 (상보필터) ----------
  angleX += bodyGx * realDt;
  angleY += bodyGy * realDt;
  if (fabsf(bodyGz) > YAW_DEADZONE) angleZ += bodyGz * realDt;

  // 위치 유지 중에는 광류로 잰 수평 가속도를 가속도계 값에서 빼서 기울기를
  // 본다. 빼지 않으면 가속 중 추정이 수평 쪽으로 끌려 위치 루프가 발산한다.
  float tiltAx = bodyAx, tiltAy = bodyAy;
  if (flowAccelUsable) {
    tiltAx += flowAccel.ax_ms2 / 9.81f;
    tiltAy += flowAccel.ay_ms2 / 9.81f;
  }
  float alpha = compute_alpha(tiltAx, tiltAy, bodyAz);
  float accAngleX = atan2f(tiltAy, sqrtf(tiltAx*tiltAx + bodyAz*bodyAz)) * 180.0f / PI;
  float accAngleY = atan2f(-tiltAx, sqrtf(tiltAy*tiltAy + bodyAz*bodyAz)) * 180.0f / PI;
  angleX = alpha * angleX + (1.0f - alpha) * accAngleX;
  angleY = alpha * angleY + (1.0f - alpha) * accAngleY;

  // outerCnt는 safety-lock 경로에서 매 tick reset되므로 독립 divider를 쓴다.
  // 이로써 armed/disarmed 모두 정확히 250Hz, K_MAG tau≈4s를 유지한다.
  if (magFusionCnt == 0 && mag_enabled) {
    MagSnapshot mag;
    if (readMagSnapshot(mag)
        // 코어0 갱신이 코어1의 nowMs보다 미래일 수 있어 부호 있는 차이로 비교한다.
        && (int32_t)(nowMs - mag.sample_ms) <= (int32_t)MAG_STALE_MS
        && mag.x*mag.x + mag.y*mag.y + mag.z*mag.z > 1e-6f) {
      const float magThrDelta =
          (float)(base_throttle + altHoldAppliedPrevUs - MAG_THROTTLE_REF_US);
      mag.x -= mag_comp_x * magThrDelta;
      mag.y -= mag_comp_y * magThrDelta;
      mag.z -= mag_comp_z * magThrDelta;
      publishMagTelemetry(mag.x, mag.y, mag.z, false);
      float heading = tiltCompensatedMagHeadingDeg(
          mag.x, mag.y, mag.z, angleX, angleY);
      if (isfinite(heading)) {
        magHeading = heading;
        const bool update_reference = takeMagReferenceRequest();
        if (update_reference) {
          // Relative heading: enabling mag or resetting angleZ must not
          // command a turn toward magnetic north.
          magYawReferenceOffset = wrapDeg(angleZ - heading);
        }
        float referencedHeading =
            wrapDeg(heading + magYawReferenceOffset);
        angleZ += magYawCorrectionDeg(angleZ, referencedHeading, K_MAG);
      }
    }
  }
  magFusionCnt++;
  if (magFusionCnt >= OUTER_DIV) magFusionCnt = 0;
}

// 과도 기울기, resume 요청, resume 인계, 무장 첫 tick 정리. RC 상실 판정보다 먼저여야 한다.
static inline __attribute__((always_inline)) void pidSafetyAndResume(PidLoopState &L, PidTick &T) {
  auto &lpfD_Roll = L.lpfD_Roll;
  auto &lpfD_Pitch = L.lpfD_Pitch;
  auto &lpfD_Yaw = L.lpfD_Yaw;
  auto &prevGyroX = L.prevGyroX;
  auto &prevGyroY = L.prevGyroY;
  auto &prevGyroZ = L.prevGyroZ;
  auto &targetRateRoll = L.targetRateRoll;
  auto &targetRatePitch = L.targetRatePitch;
  auto &targetRateYaw = L.targetRateYaw;
  auto &outerCnt = L.outerCnt;
  auto &wasLocked = L.wasLocked;
  auto &yawAuthority = L.yawAuthority;
  auto &nowMs = T.nowMs;
  auto &realDt = T.realDt;
  auto &bodyGx = T.bodyGx;
  auto &bodyGy = T.bodyGy;
  auto &bodyGz = T.bodyGz;
  // ---------- 안전 검사 ----------
  if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
    if (!fault_attitude) Serial.printf("[FAULT] OVER-TILT R:%.1f P:%.1f\n", angleX, angleY);
    fault_attitude = true;
    safety_lock = true;
  }
  if (takeFailsafeResumeRequest()) {
    const ResumeRefusalReason refusal = resumeRefusalReason(nowMs);
    if (refusal != RESUME_ALLOWED) {
      logResumeRefusal(refusal);
    } else {
      const bool rangeEpisode = rl_mode == RL_MODE_RANGE ||
                                rl_mode == RL_MODE_TIMED_FALLBACK ||
                                rl_base_ceiling != RL_NO_CAP;
      fs_phase = FS_NONE;
      fault_rc = false;
      base_throttle = (int)lroundf(hover_est);
      if (rangeEpisode) {
        // 거리 기반 하강의 collective 는 호버와 한참 다를 수 있다(바닥 근접 감소).
        // 호버로 계단을 만들지 않고 그 순간 값에서 천천히 옮긴다.
        rl_resume_handover = true;
        rlHandoverBase = (float)rl_requested_collective_us;
        base_throttle = rl_requested_collective_us;
        rlHandoverSet = base_throttle;
      }
      min_throttle = max(1050, base_throttle - CTRL_MARGIN);
      max_throttle = min(1900, base_throttle + CTRL_MARGIN);
      Serial.printf(">>> RESUME hover=%d start=%d\n", (int)lroundf(hover_est),
                    base_throttle);
      rangeLandResetFirmware();
      rl_resume_handover = rangeEpisode;
    }
  }
  if (rl_resume_handover) {
    if (fs_phase != FS_NONE || safety_lock || base_throttle != rlHandoverSet) {
      rl_resume_handover = false;   // 조종자가 스로틀을 바꿨거나 잠금
    } else {
      const float target = hover_est;
      const float step = RL_RESUME_SLEW_US_S * realDt;
      const float d = target - rlHandoverBase;
      rlHandoverBase += d > step ? step : (d < -step ? -step : d);
      base_throttle = (int)lroundf(rlHandoverBase);
      min_throttle = max(1050, base_throttle - CTRL_MARGIN);
      max_throttle = min(1900, base_throttle + CTRL_MARGIN);
      rlHandoverSet = base_throttle;
      if (fabsf(target - rlHandoverBase) < 0.5f) rl_resume_handover = false;
    }
  }
  // 잠금 해제 첫 tick의 D kick과 이전 목표 rate 잔류를 제거한다.
  // 이 블록은 RC timeout 진입 가드보다 먼저 실행해야 한다. 뒤에 두면 같은
  // tick에서 새로 진입한 FS_DESCENDING까지 FS_NONE으로 덮어쓰게 된다.
  if (wasLocked && !safety_lock) {
    // start가 남긴 이전 비행의 terminal phase와 start/abort 경합 결과를
    // Core 1에서 해제해 fs_phase의 런타임 writer를 pid_task 하나로 유지한다.
    fs_phase = FS_NONE;
    prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;
    lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
    targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
    outerCnt = 0;
    fs_first_enter_ms = 0;
    fs_first_enter_valid = false;
    rangeLandResetFirmware();
    resetYawAuthority(yawAuthority);
    yaw_authority_state = YAW_AUTH_NORMAL;
    resetAllocationTelemetry();
    wasLocked = false;
  }
}

// 광류 보정 명령, 거리·광류 프레임 소비, 거리 기반 착지 판정, 위치·고도 유지 후보.
static inline __attribute__((always_inline)) void pidFlowUpdateAndHoldCandidates(PidLoopState &L, PidTick &T) {
  auto &lastFlowReject = L.lastFlowReject;
  auto &lastFlowDtMs = L.lastFlowDtMs;
  auto &lastFlowCRoll = L.lastFlowCRoll;
  auto &lastFlowCPitch = L.lastFlowCPitch;
  auto &nowMs = T.nowMs;
  auto &nowUs = T.nowUs;
  auto &realDt = T.realDt;
  auto &bodyGz = T.bodyGz;
  auto &rlRangeFresh = T.rlRangeFresh;
  auto &rlOut = T.rlOut;
  auto &rlStepped = T.rlStepped;
  auto &heightFresh = T.heightFresh;
  auto &phOut = T.phOut;
  auto &ahOut = T.ahOut;
  // ---------- 광류·거리: 보정, 속도·높이 추정, 위치·고도 유지 후보 ----------
  const uint8_t flowCalCmd = takeFlowCalCommand();
  if (flowCalCmd == FLOWCAL_CMD_START && safety_lock) {
    flowCalReset(flowCalibrator);
    flowcal_state = FLOWCAL_RUNNING;
  } else if (flowCalCmd == FLOWCAL_CMD_STOP && flowcal_state == FLOWCAL_RUNNING) {
    flowCalResult = flowCalSolve(flowCalibrator);
    if (flowCalResult.ok &&
        flowOdomSetCalibration(flowOdomCfg, flowCalResult.m, flowCalResult.delay_us)) {
      flowcal_state = FLOWCAL_DONE_OK;
      // 행렬·지연이 바뀌면 이전 속도와 새 속도의 차분이 가짜 가속도가 된다.
      posHoldReset(posHold);
      flowAccelReset(flowAccel);
      flowOdomReset(flowOdomState);
    } else {
      flowcal_state = FLOWCAL_DONE_FAILED;
    }
    flowcal_result_pending = true;   // Core 0 가 출력한다
  }
  if (!safety_lock && flowcal_state == FLOWCAL_RUNNING) {
    flowcal_state = FLOWCAL_DONE_FAILED;   // 보정 중 시동은 start 가 막지만 방어
  }

  FlowSensorSnapshot sensorSnap;
  portENTER_CRITICAL(&flowSensorMux);
  sensorSnap = flowSensorSnapshot;
  portEXIT_CRITICAL(&flowSensorMux);
  bool rlTrustedNew = false;
  // 이번 신뢰 프레임을 넣기 전 높이 추정. 거리 기반 하강이 그 프레임을 거부하면 되돌린다.
  HeightEstimator heightBeforeFrame = heightEst;
  // 도착 시각이 이번 tick 보다 늦은 프레임(Core 0 가 tick 시작 뒤 게시)은 아직 소비하지
  // 않는다. 먼저 소비하면 시각 차분이 음수(unsigned 로 거대)라 stale 로 버려진다.
  const bool rangeReady =
      sensorSnap.range_seq != rangeSeqSeen &&
      (int32_t)(nowUs - sensorSnap.range_t_us) >= 0;
  if (rangeReady) {
    rangeSeqSeen = sensorSnap.range_seq;
    // Core 0 가 막혔다 풀리며 꺼낸 프레임은 시각을 믿을 수 없다. 현재 시각으로
    // 차분하면 가짜 수직속도가 고도 유지에 들어간다(재검토 코드 P1-3). 버린다.
    // 늦게 소비돼 이미 수명이 지난 프레임도 시각 차분에 쓰지 않는다 — 다음 정상 프레임
    // 뒤 가짜 수직속도가 남는다(Codex 코드 검토 Task 4 P2).
    const bool rangeFrameFresh =
        (uint32_t)(nowUs - sensorSnap.range_t_us) <= FLOW_HEIGHT_MAX_AGE_US;
    if (sensorSnap.range_timing_ok && rangeFrameFresh) {
      heightBeforeFrame = heightEst;
      heightUpdate(heightEst, sensorSnap.range_mm, sensorSnap.range_t_us,
                   angleX, angleY);
      // 추적 시각은 millis 축으로 옮긴다. micros/1000 은 71.6 분마다 엉뚱하게 감긴다.
      const uint32_t frameMs =
          nowMs - (uint32_t)(nowUs - sensorSnap.range_t_us) / 1000U;
      rangeTrackUpdate(rlPreTrack, sensorSnap.range_mm, frameMs);
      rlRangeMs = frameMs;
      rlRangeMm = sensorSnap.range_mm;
      rlRangeTUs = sensorSnap.range_t_us;
      rlRangeSeen = true;
      rlTrustedNew = true;
    }
  }
  rlRangeFresh =
      rlRangeSeen && (uint32_t)(nowUs - rlRangeTUs) <= FLOW_HEIGHT_MAX_AGE_US;
  // 거리 기반 하강의 판정은 광류·위치 유지보다 먼저 한다. 거부한 프레임의 높이가 광류 속도
  // 환산·수평 가속도 추정에 들어가지 않게 그 자리에서 되돌린다(최종 검토 Important).
  rlStepped =
      !safety_lock && fs_phase == FS_DESCENDING && rl_mode == RL_MODE_RANGE;
  if (rlStepped) {
    RangeLandInput rlIn;
    rlIn.height_fresh =
        heightEst.valid &&
        (uint32_t)(nowUs - heightEst.last_t_us) <= FLOW_HEIGHT_MAX_AGE_US;
    rlIn.h_m = heightEst.h_m;
    rlIn.range_mm = rlRangeMm;
    rlIn.range_fresh = rlRangeFresh;
    rlIn.range_trusted_new = rlTrustedNew;
    rlIn.hover_us = hover_est;
    rlIn.delivered_collective_us = rl_delivered_collective_us;
    rlIn.dt_s = realDt;
    rlIn.now_ms = nowMs;
    rlOut = rangeLandStep(rangeLand, rlIn);
    if (rlTrustedNew && rangeLand.rejecting) {
      // 거부된 프레임은 높이 추정에 이미 들어갔다. 그 차분 수직속도가 다음 정상 프레임
      // 뒤 고도 제어에 남지 않게, 그 프레임을 받기 전 추정으로 되돌린다(Codex 코드 검토
      // Task 4 P2). 비우면 수직속도를 0 부터 다시 쌓아 그동안 하강을 밀어낸다.
      heightEst = heightBeforeFrame;
    }
  }
  heightFresh =
      heightEst.valid &&
      (uint32_t)(nowUs - heightEst.last_t_us) <= FLOW_HEIGHT_MAX_AGE_US;
  FlowOdomOutput flowOut;
  bool flowNew = false;
  // 자이로 이력이 도착 시각까지 쌓인 뒤에 쓴다. Core 0 의 도착 시각이 이번 tick
  // 보다 늦을 수 있다(두 코어 모두 esp_timer 기준). 도착 시각 자체를 기다려야
  // 보정의 지연 후보 0 ms 끝점까지 덮인다(재검토 코드 P2-7).
  const bool flowReady =
      sensorSnap.flow_seq != flowSeqSeen &&
      (int32_t)(nowUs - sensorSnap.flow_t_us) >= 0;
  if (flowReady) {
    // 한 tick 에 두 프레임 이상 밀렸으면 중간 구간을 모르므로 체인을 다시 잡는다.
    if ((uint32_t)(sensorSnap.flow_seq - flowSeqSeen) > 1U) {
      flowOdomReset(flowOdomState);
    }
    flowSeqSeen = sensorSnap.flow_seq;
    flowNew = true;
    FlowFrame frame;
    frame.fx = sensorSnap.flow_x;
    frame.fy = sensorSnap.flow_y;
    frame.quality = sensorSnap.flow_quality;
    frame.t_us = sensorSnap.flow_t_us;
    frame.timing_ok = sensorSnap.flow_timing_ok;
    flowOut = flowOdomUpdate(flowOdomState, flowOdomCfg, frame, flowGyro,
                             heightEst.d_m, heightFresh, angleX, angleY);
    const bool intervalOk = flowOut.reject == FLOW_REJECT_NONE ||
                            flowOut.reject == FLOW_REJECT_UNCALIBRATED;
    if (flowcal_state == FLOWCAL_RUNNING && intervalOk &&
        heightEst.d_m >= FLOW_CAL_MIN_D_M && heightEst.d_m <= FLOW_CAL_MAX_D_M) {
      // 모든 지연 후보가 같은 샘플을 갖도록, 이력이 하나라도 모자라면 버린다.
      const uint32_t dtUs = (uint32_t)lroundf(flowOut.dt_s * 1e6f);
      float gx[FLOW_CAL_DELAY_CANDIDATES], gy[FLOW_CAL_DELAY_CANDIDATES];
      bool allOk = true;
      for (int k = 0; k < FLOW_CAL_DELAY_CANDIDATES && allOk; k++) {
        const uint32_t w1 = frame.t_us - (uint32_t)k * FLOW_CAL_DELAY_STEP_US;
        allOk = flowGyroDelta(flowGyro, w1 - dtUs, w1, gx[k], gy[k]);
      }
      if (allOk) {
        for (int k = 0; k < FLOW_CAL_DELAY_CANDIDATES; k++) {
          flowCalAddSample(flowCalibrator, k, frame.fx, frame.fy, gx[k], gy[k]);
        }
      }
    }
  }

  // 후보 출력. 적용 여부는 failsafe 판정이 끝난 뒤(아래 safety_lock 블록 직전)
  // 이번 tick 의 최종 상태로 다시 가른다.
  // 거리 기반 자동착륙 중에는 위치 유지를 켠다. 조종 입력은 0 으로 본다(OVERRIDE
  // 로 기준이 끌려가지 않게). 적분·출력은 이어받는다.
  const bool rangeHoldEnabled =
      !safety_lock && fs_phase == FS_DESCENDING && rl_mode == RL_MODE_RANGE;
  const bool holdEnabled = (!safety_lock && fs_phase == FS_NONE) || rangeHoldEnabled;
  PosHoldInput phIn;
  phIn.requested = poshold_requested || rangeHoldEnabled;
  phIn.enabled = holdEnabled;
  phIn.calibrated = flowOdomCfg.calibrated;
  phIn.has_meas = flowNew;
  phIn.meas_ok = flowNew && flowOut.valid && flowOut.rot_ok;
  phIn.c_roll = flowOut.c_roll;
  phIn.c_pitch = flowOut.c_pitch;
  phIn.meas_dt_s = flowOut.dt_s;
  phIn.meas_span_s = flowOut.span_s;
  phIn.pilot_roll_deg = rangeHoldEnabled ? 0.0f : pilotRollCmd;
  phIn.pilot_pitch_deg = rangeHoldEnabled ? 0.0f : pilotPitchCmd;
  phIn.yaw_rate_cmd_dps = rangeHoldEnabled ? 0.0f : targetYawRate;
  phIn.gyro_z_dps = bodyGz;
  phIn.yaw_deg = angleZ;
  phOut = posHoldStep(posHold, posHoldGains, phIn, realDt);
  AltHoldInput ahIn;
  ahIn.requested = althold_requested;
  ahIn.enabled = holdEnabled;
  // RC 끊김 판정은 base ≤ FS_GROUND_CUT_MAX_US 를 "지상"으로 본다. 고도 유지가
  // 그 아래 base 에 +Δ 를 얹어 띄우면 링크 상실 때 공중 컷이 된다. 그래서
  // base 가 컷 경계 + 최대 보정 아래로 내려가면 보정을 감쇠시킨다(재검토 ②-P1).
  // 리셋하면 그 순간 보정이 계단으로 빠지므로 감쇠로 한다.
  ahIn.inhibit =
      base_throttle < FS_GROUND_CUT_MAX_US + (int)lroundf(altHoldGains.out_max_us);
  ahIn.hard_zero = base_throttle <= FS_GROUND_CUT_MAX_US;
  ahIn.height_valid = heightFresh;
  ahIn.h_m = heightEst.h_m;
  ahIn.vz_ms = heightEst.vz_ms;
  ahIn.base_throttle = base_throttle;
  ahOut = altHoldStep(altHold, altHoldGains, ahIn, realDt);
  if (flowNew) {
    flowAccelUpdate(flowAccel, flowOut.valid && flowOut.rot_ok,
                    flowOut.c_roll, flowOut.c_pitch, sensorSnap.flow_t_us);
  }
  flowAccelUsable =
      flowAccel.valid &&
      (phOut.state == POSHOLD_ACTIVE || phOut.state == POSHOLD_COAST) &&
      (uint32_t)(nowUs - flowAccel.t_us) <= FLOW_ACCEL_MAX_AGE_US;
  if (flowNew) {
    lastFlowReject = flowOut.reject;
    lastFlowDtMs = flowOut.dt_s * 1000.0f;
    lastFlowCRoll = flowOut.c_roll;
    lastFlowCPitch = flowOut.c_pitch;
  }
}

// 호버 추정, RC 상실 진입(이번 tick 의 높이·거리 추적·호버 유효 사용), 거리 기반·시간 기반 하강.
static inline __attribute__((always_inline)) void pidHoverAndFailsafe(PidLoopState &L, PidTick &T) {
  auto &altHoldAppliedPrevUs = L.altHoldAppliedPrevUs;
  auto &landDet = L.landDet;
  auto &landProbeConfig = L.landProbeConfig;
  auto &fs_probe_dip_us = L.fs_probe_dip_us;
  auto &fs_enter_ms = L.fs_enter_ms;
  auto &fs_hold_yaw = L.fs_hold_yaw;
  auto &fs_probe_blocked_logged = L.fs_probe_blocked_logged;
  auto &nowMs = T.nowMs;
  auto &checkProbeDelivery = T.checkProbeDelivery;
  auto &probeReferenceThrottle = T.probeReferenceThrottle;
  auto &probeElapsedMs = T.probeElapsedMs;
  auto &realDt = T.realDt;
  auto &rlRangeFresh = T.rlRangeFresh;
  auto &rlOut = T.rlOut;
  auto &rlStepped = T.rlStepped;
  auto &heightFresh = T.heightFresh;
  auto &rlAltUs = T.rlAltUs;
  auto &rlHoldScale = T.rlHoldScale;
  const float accelMag =
      sqrtf(accX*accX + accY*accY + accZ*accZ);
  // 고도 유지가 collective 를 바꾸므로 hover 추정은 실제 collective 로 한다
  // (직전 tick 의 적용값, 1 ms 지연).
  const int hoverCollective = base_throttle + altHoldAppliedPrevUs;
  const bool hoverSampleEligible =
      !safety_lock && fs_phase == FS_NONE && !rl_resume_handover &&
      fabsf(angleX) < HOVER_LEVEL_MAX_DEG &&
      fabsf(angleY) < HOVER_LEVEL_MAX_DEG &&
      fabsf(accelMag - 1.0f) <= HOVER_ACCEL_TOL_G &&
      hoverCollective > HOVER_MIN_THROTTLE_US;
  updateHoverThrottleEstimator(
      hoverTracker, hoverSampleEligible, hoverCollective, nowMs, realDt,
      HOVER_LPF_TAU_S, HOVER_VALID_MS);
  hover_est = hoverTracker.estimate_us;
  hover_valid = hoverTracker.valid;

  // RC 타임아웃은 유효 호버 추정치가 없거나 보수적 절대 지상 스로틀이면
  // 즉시 컷하고, 그 외에는 추정 호버를 기준으로 자동착륙한다. hover_est는
  // 하강 목표에만 쓰며, 오염된 높은 값이 공중 즉시컷 경계를 올리지 못한다.
  // 진입 블록 전체를 fs_phase 가드 안에 둔다 — 가드가 없으면 하강 내내
  // rcTimedOut이 참이라 이 블록이 1kHz로 재실행되고, Serial.println이 TX
  // 버퍼를 포화시켜 pid_task를 블로킹하면 500ms 태스크 워치독이 비행 중
  // 재부팅을 일으킨다.
  // 시간 기반 하강의 시작 상태. RC 상실 진입과 거리 기반 하강의 폴백이 같이 쓴다.
  auto startTimedDescent = [&]() {
    fs_enter_ms = nowMs;
    landDet = {};
    landProbeConfig = FS_LAND_PROBE_CONFIG;
    bool dipClamped = false;
    fs_probe_dip_us = failsafeProbeDipUs(
        hover_est, FS_PROBE_DIP_FRAC, FS_PROBE_DIP_MIN_US,
        CTRL_MARGIN, dipClamped);
    landProbeConfig.dip_us = fs_probe_dip_us;
    if (dipClamped) {
      Serial.printf(
          "[WARN] AUTO-LAND PROBE DIP CLAMPED hover=%.1f applied=%d\n",
          hover_est, fs_probe_dip_us);
    }
    fs_probe_state = FS_PROBE_WAIT;
    fs_probe_no_response = 0;
    fs_probe_response_g = 0.0f;
    fs_probe_blocked_logged = false;
    base_throttle = min(failsafeDescentThrottle(
                            (int)lroundf(hover_est), FS_DESCENT_DELTA_US),
                        rl_base_ceiling);
  };
  if (fs_phase == FS_NONE && !safety_lock && rcTimedOut(nowMs, lastRcMs)) {
    fault_rc = true;
    // resume 인계 중의 낮은 base 는 지상이라서가 아니라 인계 중이라서다. 그 base 로
    // 지상 컷을 판정하면 공중 컷이 된다.
    const bool reentry = rl_resume_handover;
    const int groundJudgeUs =
        reentry ? max((int)base_throttle, (int)lroundf(hover_est)) : (int)base_throttle;
    rl_resume_handover = false;
    if (!hover_valid) {
      safety_lock = true;
      Serial.println("[FAULT] RC TIMEOUT -> CUT (NO HOVER EST)");  // 한 번만
    } else if (groundJudgeUs <= FS_GROUND_CUT_MAX_US) {
      safety_lock = true;
      Serial.println("[FAULT] RC TIMEOUT -> GROUND CUT");  // 한 번만
    } else {
      fs_phase = FS_DESCENDING;
      fs_hold_yaw = angleZ;
      if (!fs_first_enter_valid) {
        fs_first_enter_ms = nowMs;
        fs_first_enter_valid = true;
      }
      const int ceiling = reentry ? rl_requested_collective_us : RL_NO_CAP;
      rangeLandResetFirmware();
      rl_base_ceiling = ceiling;
      startTimedDescent();
      if (rangeland_enabled && heightFresh && rlRangeFresh && rlRangeMm > 0 &&
          rangeTrackStable(rlPreTrack, rlRangeMs)) {
        rl_mode = RL_MODE_RANGE;
        rangeLandStart(rangeLand, heightEst.h_m, rlRangeMm, nowMs);
        rl_stage = rangeLand.stage;
        rl_h_ref = rangeLand.h_ref;
        base_throttle = min((int)lroundf(hover_est), rl_base_ceiling);
        // 위치 유지 기준점을 진입 위치로 옮긴다. 적분·출력은 그대로 이어받는다.
        posHold.p_ref[0] = posHold.p[0];
        posHold.p_ref[1] = posHold.p[1];
        Serial.println("[FAULT] RC TIMEOUT -> AUTO-LAND (RANGE)");   // 한 번만
      } else {
        rl_mode = RL_MODE_TIMED;
        Serial.println("[FAULT] RC TIMEOUT -> AUTO-LAND");   // 한 번만
      }
    }
  }

  // 하강 중에는 목표를 매 tick 덮어쓴다. rc/rcr 파서도 하강 중 목표 변경을
  // 막으며, 링크 복구만으로는 하강을 끝내지 않는다(명시적 resume 필요).
  rlAltUs = 0;
  rlHoldScale = 1.0f;
  if (fs_phase == FS_DESCENDING) {
    targetAngleX = trim_roll;
    targetAngleY = trim_pitch;
    targetAngleZ = fs_hold_yaw;
    targetYawRate = 0.0f;
  }
  if (rlStepped && fs_phase == FS_DESCENDING && rl_mode == RL_MODE_RANGE) {
    // 거리 기반 하강: 판정은 위(광류 앞)에서 했다. 고도는 외부 목표를 따르는 AltHold.
    rl_h_ref = rlOut.h_ref;
    if (rlOut.stage != rl_stage) {
      rl_stage = rlOut.stage;
      Serial.printf(">>> RANGE LAND stage=%d range=%ld\n", (int)rl_stage,
                    (long)rlRangeMm);   // 전이 때 한 번
    }
    if (rlOut.action == RL_FALLBACK) {
      // 바닥 근접 전에 거리를 잃었다. 시간 기반 하강을 새로 시작하되(한 번의 예산
      // FS_MAX_MS) 에피소드 내내 지금 요청 collective 를 넘지 않는다.
      rl_mode = RL_MODE_TIMED_FALLBACK;
      rl_base_ceiling = min(rl_base_ceiling, (int)rl_requested_collective_us);
      startTimedDescent();
      Serial.println(">>> RANGE LAND -> TIMED FALLBACK");   // 한 번
    } else if (rlOut.action != RL_CONTINUE) {
      // 착지 컷은 접촉이 확인된 경우뿐이다. 바닥 근접 뒤 상실·정체, 백스톱은 TIMEOUT.
      const FailsafePhase next =
          rlOut.action == RL_LANDED ? FS_CUT_LANDED : FS_CUT_TIMEOUT;
      rl_cut_reason = rlOut.cut_reason;
      fs_phase = next;
      safety_lock = true;      // 아래 safety_lock 블록이 이번 tick에 정리한다
      Serial.printf(">>> AUTO-LAND END phase=%d reason=%d\n", (int)next,
                    (int)rl_cut_reason);
    } else {
      const int rlBase = min((int)lroundf(hover_est - rlOut.drop_us), rl_base_ceiling);
      base_throttle = rlBase;
      min_throttle = max(1050, rlBase - CTRL_MARGIN);
      max_throttle = min(1900, rlBase + CTRL_MARGIN);
      AltHoldInput rlAh;
      rlAh.requested = true;
      rlAh.enabled = true;
      // 거부된 프레임이 만든 높이로 고도를 맞추지 않는다.
      rlAh.height_valid = heightFresh && !rangeLand.rejecting;
      rlAh.h_m = heightEst.h_m;
      rlAh.vz_ms = heightEst.vz_ms;
      rlAh.base_throttle = rlBase;
      rlAh.external_ref = true;
      rlAh.h_ref_ext = rlOut.h_ref;
      rlAh.vz_ref_ext = rlOut.vz_ref;
      // 바닥 근접에서 올리는 보정을 막는 것은 아래 래칫이다. 고도 보정 자체를 0 이하로
      // 자르면(no_positive) 호버 편향을 메우던 적분이 한 번에 빠져 −30 us 편향에서 접지
      // 속도가 0.64 m/s 가 됐다(최종 검토). 래칫은 합계가 오르지 않게만 한다.
      rlAh.no_positive = false;
      const AltHoldOutput rlAhOut =
          altHoldStep(rlAltHold, rlAltHoldGains, rlAh, realDt);
      rlAltUs = (int)lroundf(rlAhOut.applied_us);
      if (rlOut.stage >= RL_STAGE_NEAR_GROUND) {
        // 바닥 근접에 있는 동안 요청 collective 는 오르지 않는다.
        const int total = min(rlBase + rlAltUs, rl_collective_cap);
        rl_collective_cap = total;
        rlAltUs = total - rlBase;
      } else {
        rl_collective_cap = RL_NO_CAP;   // 철회됐다(떠올랐다): 래칫을 푼다
      }
      rlHoldScale = rlOut.hold_scale;
    }
  }
  if (fs_phase == FS_DESCENDING && rl_mode != RL_MODE_RANGE) {
    const int descentThrottle =
        min(failsafeDescentThrottle((int)lroundf(hover_est), FS_DESCENT_DELTA_US),
            rl_base_ceiling);

    const uint32_t elapsed = nowMs - fs_enter_ms;
    // 프로브는 계속 돌리되 **착지 판정에는 쓰지 않는다.**
    //
    // 2026-08-01의 "010251 공중 CUT_LANDED" 해석은 철회됐다. 20Hz 단일
    // 샘플을 자유낙하로 오독했으며 1kHz 기록과 조종자 확인은 접지를 지지한다.
    // 따라서 "지면 응답이 공중보다 큼 / 판별 방향 반대 / 오분류 9/18"도
    // 근거가 없다. 유효한 실패 관측은 015448의 접지 응답 10회가 임계 0.03
    // 위여서 착지를 확정하지 못했다는 것이다. 023904의 접지 전 응답 2회만으로
    // 공중 오검출률을 검증할 수 없어 검출기는 제어에 복귀시키지 않았다.
    //
    // 따라서 자동착륙은 **시간 기반 하강**이다: FS_MAX_MS 동안 내려간 뒤
    // 백스톱이 자른다. 이 경로는 CUT_LANDED를 선택하지 않지만, 접지 전에
    // 예산이 끝나면 TIMEOUT 공중 컷은 가능하다. 접지 후에는 남은 시간만큼
    // 프롭이 돈다. 고도·수직속도 피드백이나 접지 확인은 하지 않는다.
    //
    // 프로브 딥은 실제 스로틀에 계속 적용된다 — 텔레메트리(41~43)로 기록해 다음 판별식을
    // 오프라인 데이터로 설계할 수 있다. 분석은 docs/failsafe_land_research.md.
    const bool probe_says_landed = updateLandDetector(
        landDet, accelMag, elapsed, descentThrottle,
        landProbeConfig);
    (void)probe_says_landed;
    const bool landed = false;
    fs_probe_state = landDet.probe_state;
    fs_probe_no_response = landDet.no_response_count;
    fs_probe_response_g = landDet.last_response_g;
    if (landDet.probe_state == FS_PROBE_BLOCKED &&
        !fs_probe_blocked_logged) {
      fs_probe_blocked_logged = true;
      Serial.println("[WARN] AUTO-LAND PROBE BLOCKED");  // 하강당 최대 1회
    }

    const bool probeActive = landDetectorProbeActive(landDet);
    checkProbeDelivery = probeActive;
    probeReferenceThrottle = descentThrottle;
    probeElapsedMs = elapsed;
    base_throttle =
        descentThrottle -
        (probeActive ? fs_probe_dip_us : 0);
    // 딥 tick에만 collective 하한을 요청 딥만큼 넓힌다. 자세 차동이 기존
    // 하한을 밀어 올려 딥을 자르면 검출기가 공중을 지면으로 오판할 수 있다.
    // 상한은 그대로 두고 1050us 절대 하한도 기존 clamp로 보존한다.
    min_throttle = max(
        1050,
        descentThrottle - CTRL_MARGIN - (probeActive ? fs_probe_dip_us : 0));
    max_throttle = min(1900, descentThrottle + CTRL_MARGIN);
    const FailsafePhase next = failsafeStep(landed, elapsed, FS_MAX_MS);
    if (next != FS_DESCENDING) {
      fs_phase = next;
      safety_lock = true;      // 아래 safety_lock 블록이 이번 tick에 정리한다
      Serial.printf(">>> AUTO-LAND END phase=%d\n", (int)next);
    }
  }
}

// 이번 tick 최종 상태로 위치·고도 유지 적용값 확정, 광류 텔레메트리 게시.
static inline __attribute__((always_inline)) void pidApplyHoldAndPublish(PidLoopState &L, PidTick &T) {
  auto &altHoldAppliedPrevUs = L.altHoldAppliedPrevUs;
  auto &prevHoldNonZero = L.prevHoldNonZero;
  auto &lastFlowReject = L.lastFlowReject;
  auto &lastFlowDtMs = L.lastFlowDtMs;
  auto &lastFlowCRoll = L.lastFlowCRoll;
  auto &lastFlowCPitch = L.lastFlowCPitch;
  auto &phOut = T.phOut;
  auto &ahOut = T.ahOut;
  auto &rlAltUs = T.rlAltUs;
  auto &rlHoldScale = T.rlHoldScale;
  auto &holdRollDeg = T.holdRollDeg;
  auto &holdPitchDeg = T.holdPitchDeg;
  auto &altHoldUs = T.altHoldUs;
  auto &forceOuterRecompute = T.forceOuterRecompute;
  // 위치·고도 유지 적용값 확정. failsafe 에 들어간 바로 이 tick 부터 0 이다.
  // failsafe 가 끝나도 자동으로 다시 켜지지 않도록 요청을 끈다.
  // 거리 기반 자동착륙 중에는 위치 유지(바닥 근접 뒤 감쇠)와 하강 고도 제어를 적용한다.
  const bool rangeHoldApply =
      !safety_lock && fs_phase == FS_DESCENDING && rl_mode == RL_MODE_RANGE;
  const bool holdApply = (!safety_lock && fs_phase == FS_NONE) || rangeHoldApply;
  if (fs_phase != FS_NONE) {
    poshold_requested = false;
    althold_requested = false;
  }
  const float holdScale = rangeHoldApply ? rlHoldScale : 1.0f;
  holdRollDeg = holdApply ? phOut.applied_roll_deg * holdScale : 0.0f;
  holdPitchDeg = holdApply ? phOut.applied_pitch_deg * holdScale : 0.0f;
  // base ≤ FS_GROUND_CUT_MAX_US 에서는 보정을 더하지 않는다. RC 끊김 판정이 그
  // base 를 "지상"으로 보므로, 그때 +Δ 로 떠 있으면 공중 컷이 된다(재검토 코드
  // P1-2). 그 위 60 us 구간에서는 inhibit 로 미리 감쇠시켜 이 경계의 계단을 줄인다.
  // 거리 기반 하강은 이미 failsafe 안이라 그 경계를 쓰지 않는다.
  altHoldUs =
      rangeHoldApply ? rlAltUs
      : (holdApply && base_throttle > FS_GROUND_CUT_MAX_US)
          ? (int)lroundf(ahOut.applied_us) : 0;
  altHoldAppliedPrevUs = altHoldUs;
  // 적용 보정이 꺼지는 tick 에는 outer loop 를 바로 다시 계산한다. 250 Hz 분주
  // 때문에 이전 보정이 반영된 목표 각속도가 최대 3 ms 남는다(재검토 코드 P2-4).
  const bool holdNonZero = holdRollDeg != 0.0f || holdPitchDeg != 0.0f;
  forceOuterRecompute = prevHoldNonZero && !holdNonZero;
  prevHoldNonZero = holdNonZero;
  {
    FlowHoldTelemetry t;
    t.flow_seq = flowSeqSeen;
    t.flow_dt_ms = lastFlowDtMs;
    t.flow_reject = lastFlowReject;
    t.c_roll = lastFlowCRoll;
    t.c_pitch = lastFlowCPitch;
    t.pos_r = posHold.p[0] - posHold.p_ref[0];
    t.pos_p = posHold.p[1] - posHold.p_ref[1];
    t.height_m = heightEst.valid ? heightEst.h_m : -1.0f;
    t.vz_ms = heightEst.vz_ms;
    t.poshold_state = phOut.state;
    t.candidate_roll_deg = phOut.candidate_roll_deg;
    t.candidate_pitch_deg = phOut.candidate_pitch_deg;
    t.applied_roll_deg = holdRollDeg;
    t.applied_pitch_deg = holdPitchDeg;
    t.althold_state = ahOut.state;
    t.candidate_alt_us = ahOut.candidate_us;
    t.applied_alt_us = (float)altHoldUs;
    t.flowcal_state = flowcal_state;
    t.flowcal_samples = flowCalibrator.acc[0].n;
    publishFlowHoldTelemetry(t);
  }
}

// 잠금이면 정리하고 모터를 멈춘 뒤 이번 tick 을 끝낸다.
static inline __attribute__((always_inline)) PidTickFlow pidSafetyGate(PidLoopState &L, PidTick &T) {
  auto &lpfD_Roll = L.lpfD_Roll;
  auto &lpfD_Pitch = L.lpfD_Pitch;
  auto &lpfD_Yaw = L.lpfD_Yaw;
  auto &prevGyroX = L.prevGyroX;
  auto &prevGyroY = L.prevGyroY;
  auto &prevGyroZ = L.prevGyroZ;
  auto &targetRateRoll = L.targetRateRoll;
  auto &targetRatePitch = L.targetRatePitch;
  auto &targetRateYaw = L.targetRateYaw;
  auto &outerCnt = L.outerCnt;
  auto &wasLocked = L.wasLocked;
  auto &yawAuthority = L.yawAuthority;
  auto &bodyGx = T.bodyGx;
  auto &bodyGy = T.bodyGy;
  auto &bodyGz = T.bodyGz;
  if (safety_lock) {
    if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;
    mixer_scaled = false;
    iTermRoll = iTermPitch = iTermYaw = 0.0f;
    targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
    targetYawRate = 0.0f;
    targetAngleZ = angleZ;
    yaw_hold_now = false;
    resetYawAuthority(yawAuthority);
    yaw_authority_state = YAW_AUTH_NORMAL;
    resetAllocationTelemetry();
    prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;
    lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
    outerCnt = 0;
    enterLockedState(wasLocked);
    motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
    tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
    stopMotors();
    return PID_TICK_ENDED;
  }
  return PID_TICK_GO_ON;
}

// 바깥 루프(250 Hz): yaw 권한·heading 잠금, 각도 → 목표 각속도.
static inline __attribute__((always_inline)) void pidOuterLoop(PidLoopState &L, PidTick &T) {
  auto &targetRateRoll = L.targetRateRoll;
  auto &targetRatePitch = L.targetRatePitch;
  auto &targetRateYaw = L.targetRateYaw;
  auto &outerCnt = L.outerCnt;
  auto &yawAuthority = L.yawAuthority;
  auto &nowMs = T.nowMs;
  auto &bodyGz = T.bodyGz;
  auto &holdRollDeg = T.holdRollDeg;
  auto &holdPitchDeg = T.holdPitchDeg;
  auto &forceOuterRecompute = T.forceOuterRecompute;
  // ---------- Outer loop (250Hz): 각도 -> 목표 각속도 ----------
  // yaw는 각속도 명령 + 자동 heading 잠금. hold가 아닌 동안 setpoint를
  // 매 tick 현재 heading으로 슬레이빙해 stale setpoint를 원천 차단한다.
  updateYawAuthority(
      yawAuthority, mixer_yaw_scale, targetYawRate, bodyGz,
      true, fs_phase != FS_NONE, false, nowMs,
      YAW_RATE_DEADZONE, YAW_HOLD_SETTLE_DPS);
  yaw_authority_state = static_cast<int>(yawAuthority.state);

  const YawOuter yawOuter = updateYawOuter(
      targetYawRate, bodyGz, angleZ, targetAngleZ,
      yaw_hold_now, yaw_hold_override,
      YAW_RATE_DEADZONE, YAW_HOLD_SETTLE_DPS);
  const float normalTargetRateYaw = yawTargetRateDps(
      yawOuter, targetYawRate, angleZ, Kp_Angle_Yaw, MAX_TARGET_RATE_YAW);
  const YawAuthorityCommand yawCommand = applyYawAuthority(
      yawAuthority.state, angleZ, yawOuter.target_angle_deg,
      normalTargetRateYaw, yawOuter.hold);
  targetAngleZ = yawCommand.target_angle_deg;
  yaw_hold_now = yawCommand.hold;

  if (forceOuterRecompute) outerCnt = 0;
  if (outerCnt == 0) {
    // 위치 유지 보정은 조종+트림 목표에 같은 부호로 더한다(flow_hold.h).
    const float rollTarget = constrain(targetAngleX + holdRollDeg,
                                       -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
    const float pitchTarget = constrain(targetAngleY + holdPitchDeg,
                                        -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
    targetRateRoll = constrain((rollTarget - angleX) * Kp_Angle_Roll,
                               -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
    targetRatePitch = constrain((pitchTarget - angleY) * Kp_Angle_Pitch,
                                -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
    targetRateYaw = yawCommand.target_rate_dps;
  } else if (yawAuthority.state != YAW_AUTH_NORMAL) {
    targetRateYaw = 0.0f;
  }
  outerCnt++;
  if (outerCnt >= OUTER_DIV) outerCnt = 0;
}

// 안쪽 루프(1 kHz): 각속도 PID.
static inline __attribute__((always_inline)) void pidInnerLoop(PidLoopState &L, PidTick &T) {
  auto &lpfD_Roll = L.lpfD_Roll;
  auto &lpfD_Pitch = L.lpfD_Pitch;
  auto &lpfD_Yaw = L.lpfD_Yaw;
  auto &prevGyroX = L.prevGyroX;
  auto &prevGyroY = L.prevGyroY;
  auto &prevGyroZ = L.prevGyroZ;
  auto &targetRateRoll = L.targetRateRoll;
  auto &targetRatePitch = L.targetRatePitch;
  auto &targetRateYaw = L.targetRateYaw;
  auto &realDt = T.realDt;
  auto &bodyGx = T.bodyGx;
  auto &bodyGy = T.bodyGy;
  auto &bodyGz = T.bodyGz;
  auto &eRoll = T.eRoll;
  auto &ePitch = T.ePitch;
  auto &eYaw = T.eYaw;
  auto &iRollUsed = T.iRollUsed;
  auto &iPitchUsed = T.iPitchUsed;
  auto &iYawUsed = T.iYawUsed;
  auto &pidRoll = T.pidRoll;
  auto &pidPitch = T.pidPitch;
  auto &pidYaw = T.pidYaw;
  // ---------- Inner loop (1kHz): 각속도 PID ----------
  eRoll  = targetRateRoll  - bodyGx;
  ePitch = targetRatePitch - bodyGy;
  eYaw   = targetRateYaw   - bodyGz;

  // D는 gyro 미분에만 건다 (setpoint/outer loop 오염 제거)
  float dRoll  = lpfD_Roll.update((bodyGx - prevGyroX) / realDt);
  float dPitch = lpfD_Pitch.update((bodyGy - prevGyroY) / realDt);
  float dYaw   = lpfD_Yaw.update((bodyGz - prevGyroZ) / realDt);
  prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;

  iRollUsed = iTermRoll;
  iPitchUsed = iTermPitch;
  iYawUsed = iTermYaw;
  pidRoll  = Kp_Rate_Roll *eRoll  + iRollUsed  - Kd_Rate_Roll *dRoll;
  pidPitch = Kp_Rate_Pitch*ePitch + iPitchUsed - Kd_Rate_Pitch*dPitch;
  // yaw 각도 유지가 꺼져 있어도 rate 감쇠(target 0)는 항상 동작한다.
  // 모터 토크 불균형과 롤·피치 보정의 반작용으로 생기는 자유 회전을 억제한다.
  pidYaw   = Kp_Rate_Yaw*eYaw + iYawUsed - Kd_Rate_Yaw*dYaw;
}

// desaturation·anti-windup, 모터 출력. 잠금이면 이번 tick 을 끝낸다.
static inline __attribute__((always_inline)) PidTickFlow pidMixAndOutput(PidLoopState &L, PidTick &T) {
  auto &targetRateRoll = L.targetRateRoll;
  auto &targetRatePitch = L.targetRatePitch;
  auto &targetRateYaw = L.targetRateYaw;
  auto &wasLocked = L.wasLocked;
  auto &landDet = L.landDet;
  auto &landProbeConfig = L.landProbeConfig;
  auto &fs_probe_dip_us = L.fs_probe_dip_us;
  auto &checkProbeDelivery = T.checkProbeDelivery;
  auto &probeReferenceThrottle = T.probeReferenceThrottle;
  auto &probeElapsedMs = T.probeElapsedMs;
  auto &realDt = T.realDt;
  auto &altHoldUs = T.altHoldUs;
  auto &eRoll = T.eRoll;
  auto &ePitch = T.ePitch;
  auto &eYaw = T.eYaw;
  auto &iRollUsed = T.iRollUsed;
  auto &iPitchUsed = T.iPitchUsed;
  auto &iYawUsed = T.iYawUsed;
  auto &pidRoll = T.pidRoll;
  auto &pidPitch = T.pidPitch;
  auto &pidYaw = T.pidYaw;
  // ---------- 모터 desaturation + 포화 기반 anti-windup ----------
  // 고도 유지 보정은 collective 와 자세 여유 창을 같이 옮긴다. 창을 그대로
  // 두면 +Δ 만큼 위쪽 자세 여유가 줄어 포화가 일찍 온다.
  const int throttle = base_throttle + altHoldUs;
  const int mixMin = constrain(min_throttle + altHoldUs, 1050, 1900);
  const int mixMax = constrain(max_throttle + altHoldUs, 1050, 1900);
  MotorMix mix = mixAndDesaturate(pidRoll, pidPitch, pidYaw,
                                  throttle, mixMin, mixMax);
  mixer_yaw_scale = mix.yaw_scale;
  rl_requested_collective_us = throttle;
  rl_delivered_collective_us = mix.collective_us;
  publishAllocationTelemetry(
      mix, pidRoll, pidPitch, pidYaw,
      iRollUsed, iPitchUsed, iYawUsed, yaw_authority_state, throttle);
  if (checkProbeDelivery) {
    const float deliveredUs =
        (float)probeReferenceThrottle - mix.collective_us;
    const bool deliveryValid =
        deliveredUs >= FS_PROBE_MIN_DELIVERY_FRAC * fs_probe_dip_us;
    recordLandDetectorProbeDelivery(
        landDet, probeElapsedMs, deliveryValid, landProbeConfig);
  }

  // scale은 자세 명령이 실제로 잘린 경우다. collective 이동만 일어난 경우에는
  // 자세 authority가 보존되므로 적분을 계속한다.
  // 적분은 모터 출력 기여(µs) 단위로 누적하고 I_TERM_MAX_US로 제한한다.
  // (이전 errorSum ±200deg 클램프는 기본 Ki에서 최대 기여 1µs라 적분이
  //  사실상 없는 것과 같아 CG 치우침 같은 정상상태 오차를 못 잡았다.)
  iTermRoll = conditionalAxisIntegral(
      iTermRoll, Ki_Rate_Roll * eRoll * realDt, pidRoll,
      mix.rp_scale, throttle, I_TERM_MAX_US);
  iTermPitch = conditionalAxisIntegral(
      iTermPitch, Ki_Rate_Pitch * ePitch * realDt, pidPitch,
      mix.rp_scale, throttle, I_TERM_MAX_US);
  // yaw 적분은 rate/hold 두 모드 모두에서 돈다. 안쪽 루프의 임무가
  // "목표 각속도 추종"이고, P 단독으로는 모터 토크 불균형 같은 일정
  // 외란에서 정상상태 각속도 오차가 남아 정착 임계치 아래로 내려가지
  // 않는다(2026-07-27 실측 최대 +16.6dps). 그러면 자동 잠금이 영영
  // 걸리지 않는다.
  iTermYaw = conditionalAxisIntegral(
      iTermYaw, Ki_Rate_Yaw * eYaw * realDt, pidYaw,
      mix.yaw_scale, throttle, I_TERM_MAX_US);

  // 모터 PWM의 단일 writer는 PID task로 유지한다.
  if (safety_lock) {
    resetAllocationTelemetry();
    yaw_authority_state = YAW_AUTH_NORMAL;
    motorOut[0] = 1000; motorOut[1] = 1000; motorOut[2] = 1000; motorOut[3] = 1000;
    tgtRate[0] = 0.0f; tgtRate[1] = 0.0f; tgtRate[2] = 0.0f;
    stopMotors();
    enterLockedState(wasLocked);
    return PID_TICK_ENDED;
  }
  writeMotor(pinM1, mix.motor[0]);
  writeMotor(pinM2, mix.motor[1]);
  writeMotor(pinM3, mix.motor[2]);
  writeMotor(pinM4, mix.motor[3]);
  motorOut[0] = mix.motor[0]; motorOut[1] = mix.motor[1];
  motorOut[2] = mix.motor[2]; motorOut[3] = mix.motor[3];
  tgtRate[0] = targetRateRoll; tgtRate[1] = targetRatePitch; tgtRate[2] = targetRateYaw;
  return PID_TICK_GO_ON;
}

void pid_task(void *pv) {
  // 모터 정지 수단(stopMotors, RC timeout 검사)이 모두 이 태스크 안에 있으므로,
  // SPI 행업 등으로 태스크가 블로킹되면 모터가 마지막 PWM으로 고정된다.
  // 태스크 워치독이 panic 재부팅을 강제하고, 재부팅 후 ESC는 PWM 신호
  // 소실로 정지하며 setup()의 stopMotors()가 정지 신호를 복원한다.
  esp_task_wdt_add(NULL);

  PidLoopState L;

  while (true) {
    vTaskDelayUntil(&L.wake, L.period);
    esp_task_wdt_reset();
    PidTick T;
    pidBeginTick(L, T);
    if (pidReadImus(L, T) == PID_TICK_ENDED) continue;
    pidEstimateAttitude(L, T);
    pidSafetyAndResume(L, T);
    pidFlowUpdateAndHoldCandidates(L, T);
    pidHoverAndFailsafe(L, T);
    pidApplyHoldAndPublish(L, T);
    if (pidSafetyGate(L, T) == PID_TICK_ENDED) continue;
    pidOuterLoop(L, T);
    pidInnerLoop(L, T);
    if (pidMixAndOutput(L, T) == PID_TICK_ENDED) continue;
  }
}
