// zetin_fc/fc/udp_task.h — 9. udp_task (Core 0): 명령 처리, 텔레메트리·raw 송신, 센서 폴링.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// ==========================================================
// 9. UDP 태스크 (Core 0) — char* 파싱 (String heap 회피)
// ==========================================================
static char *trimCommand(char *s) {
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  char *end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) end--;
  *end = '\0';
  return s;
}

static bool parseFloatStrict(const char *text, float &out) {
  char *end;
  float value = strtof(text, &end);
  if (end == text || !isfinite(value)) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  out = value;
  return true;
}

static bool parseIntStrict(const char *text, long &out) {
  char *end;
  long value = strtol(text, &end, 10);
  if (end == text) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  out = value;
  return true;
}

static bool setRcTargets(float x, float y, float z, bool hasYaw) {
  if (!isfinite(x) || !isfinite(y) || (hasYaw && !isfinite(z))) return false;
  if (fs_phase == FS_DESCENDING) {
    lastRcMs = millis();
    return true;
  }
  // 위치 유지의 조종 개입 판정용. 트림을 뺀 역산은 클램프에 걸리면 틀린다.
  pilotRollCmd = x;
  pilotPitchCmd = y;
  // 트림을 더한 뒤 클램프해야 총합이 ±30°로 제한된다.
  targetAngleX = constrain(x + trim_roll,  -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  targetAngleY = constrain(y + trim_pitch, -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  if (hasYaw) targetAngleZ = z;
  lastRcMs = millis();
  return true;
}

// 표준 형식: rc <seq> <roll> <pitch> <yaw>
// 벤치 수동 테스트용으로 rc <roll> <pitch> [yaw]도 허용한다.
static void handleRcCommand(char *buf) {
  char *save = nullptr;
  char *token = strtok_r(buf, " \t", &save); // "rc"
  (void)token;
  char *arg[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 4 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (strtok_r(nullptr, " \t", &save) != nullptr) return; // 여분 필드 거부

  if (count == 4) {
    if (arg[0][0] == '-' || arg[0][0] == '+') return;
    char *seqEnd;
    errno = 0;
    unsigned long seqLong = strtoul(arg[0], &seqEnd, 10);
    if (seqEnd == arg[0] || *seqEnd != '\0' || errno == ERANGE ||
        seqLong > 0xFFFFFFFFUL) return;

    float x, y, z;
    if (!parseFloatStrict(arg[1], x) || !parseFloatStrict(arg[2], y) ||
        !parseFloatStrict(arg[3], z)) return;

    uint32_t seq = (uint32_t)seqLong;
    rcTotalPkts = rcTotalPkts + 1;
    if (rcSeqValid) {
      int32_t advance = (int32_t)(seq - lastRcSeq); // uint32 wrap도 정상 처리
      if (advance <= 0) {
        rcDroppedPkts = rcDroppedPkts + 1;
        return; // 지연 도착/중복 패킷 폐기
      }
      if (advance > 1) rcDroppedPkts += (uint32_t)(advance - 1);
    }
    lastRcSeq = seq;
    rcSeqValid = true;
    setRcTargets(x, y, z, true);
    return;
  }

  if (count == 2 || count == 3) {
    float x, y, z = 0.0f;
    if (!parseFloatStrict(arg[0], x) || !parseFloatStrict(arg[1], y)) return;
    if (count == 3 && !parseFloatStrict(arg[2], z)) return;
    setRcTargets(x, y, z, count == 3);
  }
}

// rcr <seq> <roll> <pitch> <yaw_rate>
// yaw는 각속도(dps) 명령이다. seq 처리와 워치독 급이는 rc와 상태를 공유한다.
static void handleRcrCommand(char *buf) {
  char *save = nullptr;
  (void)strtok_r(buf, " \t", &save); // "rcr"
  char *arg[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 4 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (count != 4) return;
  if (strtok_r(nullptr, " \t", &save) != nullptr) return; // 여분 필드 거부

  if (arg[0][0] == '-' || arg[0][0] == '+') return;
  char *seqEnd;
  errno = 0;
  unsigned long seqLong = strtoul(arg[0], &seqEnd, 10);
  if (seqEnd == arg[0] || *seqEnd != '\0' || errno == ERANGE ||
      seqLong > 0xFFFFFFFFUL) return;

  float x, y, rate;
  if (!parseFloatStrict(arg[1], x) || !parseFloatStrict(arg[2], y) ||
      !parseFloatStrict(arg[3], rate)) return;

  uint32_t seq = (uint32_t)seqLong;
  rcTotalPkts = rcTotalPkts + 1;
  if (rcSeqValid) {
    int32_t advance = (int32_t)(seq - lastRcSeq);
    if (advance <= 0) {
      rcDroppedPkts = rcDroppedPkts + 1;
      return;
    }
    if (advance > 1) rcDroppedPkts += (uint32_t)(advance - 1);
  }
  lastRcSeq = seq;
  rcSeqValid = true;

  if (!setRcTargets(x, y, 0.0f, false)) return;   // roll/pitch만, yaw 각도는 건드리지 않는다
  if (fs_phase == FS_DESCENDING) return;
  targetYawRate = constrain(rate, -MAX_TARGET_RATE_YAW, MAX_TARGET_RATE_YAW);
}

static void handleGainCommand(const char *buf) {
  if (strlen(buf) < 3) return;
  float value;
  if (!parseFloatStrict(buf + 2, value) || value < 0.0f || value > 100.0f) return;
  if (!configChangeAllowed("PID gain")) return;

  // Cascade 전용 공통 명령
  if      (strncmp(buf, "rp", 2) == 0) { Kp_Rate_Roll = value; Kp_Rate_Pitch = value; }
  else if (strncmp(buf, "ri", 2) == 0) { Ki_Rate_Roll = value; Ki_Rate_Pitch = value; }
  else if (strncmp(buf, "rd", 2) == 0) { Kd_Rate_Roll = value; Kd_Rate_Pitch = value; }
  else if (strncmp(buf, "ap", 2) == 0) { Kp_Angle_Roll = value; Kp_Angle_Pitch = value; }
  else if (strncmp(buf, "ar", 2) == 0) Kp_Angle_Roll = value;
  else if (strncmp(buf, "at", 2) == 0) Kp_Angle_Pitch = value;
  else if (strncmp(buf, "ay", 2) == 0) Kp_Angle_Yaw = value;
  else if (strncmp(buf, "yp", 2) == 0) Kp_Rate_Yaw = value;
  else if (strncmp(buf, "yi", 2) == 0) Ki_Rate_Yaw = value;
  else if (strncmp(buf, "yd", 2) == 0) Kd_Rate_Yaw = value;

  // scripts/drone_control.py --keyboard 명령 호환: P/I/D는 inner rate PID에 대응
  else if (strncmp(buf, "pa", 2) == 0) { Kp_Rate_Roll = value; Kp_Rate_Pitch = value; }
  else if (strncmp(buf, "ia", 2) == 0) { Ki_Rate_Roll = value; Ki_Rate_Pitch = value; }
  else if (strncmp(buf, "da", 2) == 0) { Kd_Rate_Roll = value; Kd_Rate_Pitch = value; }
  else if (strncmp(buf, "pr", 2) == 0) Kp_Rate_Roll = value;
  else if (strncmp(buf, "ir", 2) == 0) Ki_Rate_Roll = value;
  else if (strncmp(buf, "dr", 2) == 0) Kd_Rate_Roll = value;
  else if (strncmp(buf, "pp", 2) == 0) Kp_Rate_Pitch = value;
  else if (strncmp(buf, "ip", 2) == 0) Ki_Rate_Pitch = value;
  else if (strncmp(buf, "dp", 2) == 0) Kd_Rate_Pitch = value;
  else if (strncmp(buf, "py", 2) == 0) Kp_Rate_Yaw = value;
  else if (strncmp(buf, "iy", 2) == 0) Ki_Rate_Yaw = value;
  else if (strncmp(buf, "dy", 2) == 0) Kd_Rate_Yaw = value;
}

// 첫 14개 필드는 기존 PC 스크립트와 호환된다. 뒤 필드는 cascade 진단 확장:
// fault_imu1,fault_imu2,fault_disagree,active_imus,scaled,fault_attitude,
// calibration_ok,armed. 그 뒤 Tier1 8개와 MagHeading, Mag_X/Y/Z, Yaw_Hold,
// Failsafe_Phase, Trim_Roll/Pitch, Hover_Est/Valid와 프로브 상태/연속
// 무반응/최근 차분 반응, IMU1 gyro/accel XYZ, IMU2 gyro/accel XYZ를
// 보낸 뒤 목표 roll/pitch/yaw 각도와 필드 59 Mag_Enabled, 3901-L0X 5개
// 필드, 필드 65 Mag_Cal_Active, 필드 66~75 allocator/PID/I/yaw authority를
// append-only로 보낸다. IMU별 값은 융합값과 같은 body frame이다.
// 이 스케치는 인덱스 75~90 에 광류 위치·고도 유지 16개 필드를 더 붙인다
// (Flow_Seq … FlowCal_Samples, scripts/ground/telemetry_schema.py).
static void sendTelemetry() {
  if (!connectionEstablished) return;
  bool criticalFault = (active_imus == 0) || fault_attitude || !calibration_ok;
  const float hoverEstSnapshot = hover_est;
  const bool hoverValidSnapshot = hover_valid;
  const uint8_t probeStateSnapshot = fs_probe_state;
  const uint8_t probeNoResponseSnapshot = fs_probe_no_response;
  const float probeResponseSnapshot = fs_probe_response_g;
  const AllocationTelemetrySample allocationSample =
      readAllocationTelemetrySnapshot();
  const ImuTelemetrySample imuSample = readImuSampleSnapshot();
  const FlowHoldTelemetry flowT = readFlowHoldTelemetry();
  // 거리 기반 자동착륙(92~96)과 36번 단계는 allocation 스냅숏의 같은 tick 값이다.
  const uint8_t landModeSnapshot = allocationSample.land_mode;
  const uint8_t landStageSnapshot = allocationSample.land_stage;
  const float landRefSnapshot = allocationSample.land_ref_m;
  const uint8_t landCutSnapshot = allocationSample.land_cut;
  const int landReqSnapshot = allocationSample.requested_us;   // 68번과 같은 스냅숏
  float magTelemXSnapshot = 0.0f;
  float magTelemYSnapshot = 0.0f;
  float magTelemZSnapshot = 0.0f;
  bool magTelemCalActiveSnapshot = false;
  readMagTelemetry(
      magTelemXSnapshot, magTelemYSnapshot, magTelemZSnapshot,
      magTelemCalActiveSnapshot);
  udp.beginPacket(laptopIP, laptopPort);
  udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%d,%d,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d,%ld,%d,%ld,%ld,%d,%d,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,"
             "%lu,%.1f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.2f,%.2f,%d,%.1f,%d,%lu,"
             "%d,%d,%.2f,%d,%d,%d",
             angleX, angleY, angleZ,
             gyroX, gyroY, gyroZ,
             accX, accY, accZ,
             base_throttle,
             (int)fault_rc, (int)criticalFault,
             (unsigned long)rcTotalPkts, (unsigned long)rcDroppedPkts,
             (int)fault_imu1, (int)fault_imu2, (int)fault_disagree,
             active_imus, (int)allocationSample.scaled, (int)fault_attitude,
             (int)calibration_ok, (int)!safety_lock,
             motorOut[0], motorOut[1], motorOut[2], motorOut[3], pidLoopHz,
             tgtRate[0], tgtRate[1], tgtRate[2], magHeading,
             magTelemXSnapshot, magTelemYSnapshot, magTelemZSnapshot,
             (int)yaw_hold_now,
             (int)allocationSample.fs_phase, trim_roll, trim_pitch,
             hoverEstSnapshot, (int)hoverValidSnapshot,
             (int)probeStateSnapshot, (int)probeNoResponseSnapshot,
             probeResponseSnapshot,
             imuSample.imu1GyroX, imuSample.imu1GyroY, imuSample.imu1GyroZ,
             imuSample.imu1AccelX, imuSample.imu1AccelY, imuSample.imu1AccelZ,
             imuSample.imu2GyroX, imuSample.imu2GyroY, imuSample.imu2GyroZ,
             imuSample.imu2AccelX, imuSample.imu2AccelY, imuSample.imu2AccelZ,
             targetAngleX, targetAngleY, targetAngleZ, (int)mag_enabled,
             (long)msp_range_mm, msp_range_quality,
             (long)msp_flow_x, (long)msp_flow_y, msp_flow_quality,
             (int)magTelemCalActiveSnapshot,
             allocationSample.rp_scale, allocationSample.yaw_scale,
             allocationSample.collective_us,
             allocationSample.pid_roll_us, allocationSample.pid_pitch_us,
             allocationSample.pid_yaw_us,
             allocationSample.i_roll_us, allocationSample.i_pitch_us,
             allocationSample.i_yaw_us,
             allocationSample.yaw_authority_state,
             (unsigned long)flowT.flow_seq, flowT.flow_dt_ms, flowT.flow_reject,
             flowT.c_roll, flowT.c_pitch, flowT.pos_r, flowT.pos_p,
             flowT.height_m, flowT.vz_ms, flowT.poshold_state,
             flowT.candidate_roll_deg, flowT.candidate_pitch_deg,
             flowT.althold_state, flowT.candidate_alt_us,
             flowT.flowcal_state, (unsigned long)flowT.flowcal_samples,
             (int)landModeSnapshot, (int)landStageSnapshot, landRefSnapshot,
             (int)landCutSnapshot, landReqSnapshot, (int)rangeland_enabled);
  udp.endPacket();
}

static bool sendImuRawBatch() {
  if (!connectionEstablished) return false;
  uint32_t queued = imuRawRingCount(imuRawRing);
  if (queued == 0) return false;
  const uint8_t nSamples = static_cast<uint8_t>(
      min(queued, static_cast<uint32_t>(IMU_RAW_BATCH_MAX)));
  uint32_t baseTUs = 0;
  uint8_t copied = 0;
  for (; copied < nSamples; copied++) {
    ImuRawSample sample = {};
    if (!imuRawRingPop(imuRawRing, sample)) break;
    uint32_t sampleUs;
    if (!rawConsumerTimeValid) {
      sampleUs = __atomic_load_n(&imuRawRing.first_t_us, __ATOMIC_ACQUIRE);
    } else {
      sampleUs = rawConsumerLastUs + sample.dt_us;
    }
    if (copied == 0) baseTUs = sampleUs;
    rawConsumerLastUs = sampleUs;
    rawConsumerTimeValid = true;
    imuRawBatch[copied] = sample;
  }
  if (copied == 0) return false;

  const uint32_t dropped =
      __atomic_load_n(&imuRawRing.dropped, __ATOMIC_RELAXED);
  const size_t packetSize = buildImuRawDatagram(
      imuRawDatagram, rawBatchSeq, baseTUs, dropped, imuRawBatch, copied);
  udp.beginPacket(laptopIP, laptopPort);
  udp.write(reinterpret_cast<const uint8_t *>(&imuRawDatagram), packetSize);
  udp.endPacket();
  rawBatchSeq++;
  return true;
}

static bool sendImuCalibration() {
  if (!connectionEstablished) return false;
  memcpy(imuCalDatagram.magic, "ZCAL", 4);
  imuCalDatagram.version = IMU_RAW_VERSION;
  memset(imuCalDatagram.reserved, 0, sizeof(imuCalDatagram.reserved));
  for (int axis = 0; axis < 3; axis++) {
    imuCalDatagram.gyro_bias1[axis] = gyro_bias1[axis];
    imuCalDatagram.gyro_bias2[axis] = gyro_bias2[axis];
    imuCalDatagram.imu2_sign[axis] = IMU2_SIGN[axis];
  }
  imuCalDatagram.accel_scale1 = accel_scale1;
  imuCalDatagram.accel_scale2 = accel_scale2;
  imuCalDatagram.gyro_scale = GYRO_SCALE;
  imuCalDatagram.accel_scale = ACCEL_SCALE;
  udp.beginPacket(laptopIP, laptopPort);
  udp.write(reinterpret_cast<const uint8_t *>(&imuCalDatagram),
            sizeof(imuCalDatagram));
  udp.endPacket();
  return true;
}

// 3901-L0X UART 폴링. **논블로킹**: available() 만큼만, 반복당 상한까지.
// 프레임이 완성될 때마다 도착 시각과 함께 Core 1 에 게시한다. 위치·고도
// 유지와 보정은 Core 1 이 이 스냅샷으로 한다.
static void pollMspSensor(uint32_t nowMs) {
  const uint32_t pollUs = micros();
  // 직전 폴링과 너무 멀면 이번에 완성되는 프레임은 FIFO 에서 늦게 꺼낸 것이다.
  const bool timingOk =
      mspPollSeen && !mspBacklog &&
      (uint32_t)(pollUs - mspLastPollUs) <= FLOW_POLL_GAP_MAX_US;
  mspLastPollUs = pollUs;
  mspPollSeen = true;
  int budget = MSP_SENSOR_MAX_BYTES_PER_TICK;
  while (budget-- > 0 && Serial1.available() > 0) {
    const MspParseResult result =
        mspSensorParseByte(mspSensor, (uint8_t)Serial1.read());
    if (result == MSP_PARSE_RANGEFINDER) {
      msp_range_mm = mspSensor.range.distance_mm;
      msp_range_quality = (int)mspSensor.range.quality;
      msp_last_range_ms = nowMs;
      portENTER_CRITICAL(&flowSensorMux);
      flowSensorSnapshot.range_seq++;
      flowSensorSnapshot.range_mm = mspSensor.range.distance_mm;
      flowSensorSnapshot.range_t_us = pollUs;
      flowSensorSnapshot.range_timing_ok = timingOk;
      flowSensorSnapshot.crc_errors = mspSensor.crc_errors;
      portEXIT_CRITICAL(&flowSensorMux);
    } else if (result == MSP_PARSE_OPTIC_FLOW) {
      msp_flow_x = mspSensor.flow.motion_x;
      msp_flow_y = mspSensor.flow.motion_y;
      msp_flow_quality = (int)mspSensor.flow.quality;
      msp_last_flow_ms = nowMs;
      portENTER_CRITICAL(&flowSensorMux);
      flowSensorSnapshot.flow_seq++;
      flowSensorSnapshot.flow_x = mspSensor.flow.motion_x;
      flowSensorSnapshot.flow_y = mspSensor.flow.motion_y;
      flowSensorSnapshot.flow_quality = (int)mspSensor.flow.quality;
      flowSensorSnapshot.flow_t_us = pollUs;
      flowSensorSnapshot.flow_timing_ok = timingOk;
      flowSensorSnapshot.crc_errors = mspSensor.crc_errors;
      portEXIT_CRITICAL(&flowSensorMux);
    } else if (result == MSP_PARSE_CRC_ERROR) {
      portENTER_CRITICAL(&flowSensorMux);
      flowSensorSnapshot.crc_errors = mspSensor.crc_errors;
      portEXIT_CRITICAL(&flowSensorMux);
    } else if (result == MSP_PARSE_UNKNOWN &&
               mspSensor.last_unknown_cmd != msp_reported_unknown) {
      // function 상수가 틀렸다면 "값이 안 나온다"가 아니라 이 줄로 드러난다.
      msp_reported_unknown = mspSensor.last_unknown_cmd;
      Serial.printf("[MSP] unknown function 0x%04X\n",
                    mspSensor.last_unknown_cmd);
    }
  }
  mspBacklog = Serial1.available() > 0;
  // 신선하지 않으면 quality 를 -1 로. 값 자체는 건드리지 않는다 —
  // distance_mm 는 모듈이 범위 밖에서 음수를 쓰므로 센티넬로 못 쓴다.
  if ((int32_t)(nowMs - msp_last_range_ms) > (int32_t)MSP_SENSOR_STALE_MS) {
    msp_range_quality = -1;
  }
  if ((int32_t)(nowMs - msp_last_flow_ms) > (int32_t)MSP_SENSOR_STALE_MS) {
    msp_flow_quality = -1;
  }
}

static const char *flowCalReasonName(uint8_t reason) {
  switch (reason) {
    case FLOW_CAL_OK: return "OK";
    case FLOW_CAL_TOO_FEW: return "TOO_FEW";
    case FLOW_CAL_LOW_EXCITATION: return "LOW_EXCITATION";
    case FLOW_CAL_COLLINEAR: return "COLLINEAR";
    case FLOW_CAL_POOR_FIT: return "POOR_FIT";
    case FLOW_CAL_NOT_PERMUTATION: return "NOT_PERMUTATION";
    case FLOW_CAL_SCALE_RANGE: return "SCALE_RANGE";
  }
  return "?";
}

// Core 1 이 보정을 끝내면 한 번 출력한다. 붙여 넣을 상수 줄도 같이 낸다.
static void reportFlowCalResult() {
  if (!flowcal_result_pending) return;
  flowcal_result_pending = false;
  FlowCalResult r = flowCalResult;
  if (!isfinite(r.cond)) r.cond = 1e9f;   // 한 축만 흔들면 무한대. 표시만 유한값으로
  Serial.printf(">>> FLOWCAL %s samples=%lu delay=%lums residual=%.3f cond=%.2f corr=%.2f\n",
                flowCalReasonName(r.reason), (unsigned long)r.samples,
                (unsigned long)(r.delay_us / 1000U), (double)r.residual,
                (double)r.cond, (double)r.corr);
  if (r.ok) {
    Serial.printf("const bool     FLOW_CAL_VALID = true;\n"
                  "const float    FLOW_CAL_M[2][2] = {{%.2ff, %.2ff}, {%.2ff, %.2ff}};\n"
                  "const uint32_t FLOW_CAL_DELAY_US = %lu;\n",
                  (double)r.m[0][0], (double)r.m[0][1],
                  (double)r.m[1][0], (double)r.m[1][1],
                  (unsigned long)r.delay_us);
  }
  if (connectionEstablished) {
    udp.beginPacket(laptopIP, laptopPort);
    udp.printf("FLOWCAL,%d,%s,%.2f,%.2f,%.2f,%.2f,%lu,%.4f,%.2f,%.3f,%lu",
               (int)r.ok, flowCalReasonName(r.reason),
               (double)r.m[0][0], (double)r.m[0][1],
               (double)r.m[1][0], (double)r.m[1][1],
               (unsigned long)r.delay_us, (double)r.residual,
               (double)r.cond, (double)r.corr, (unsigned long)r.samples);
    udp.endPacket();
  }
}

// Core 1 이 imucal 을 끝내면 한 번: 성공이면 NVS 에 저장하고 결과를 보고한다.
static void reportImuCalResult() {
  if (!imucal_result_pending) return;
  imucal_result_pending = false;
  const ImuCalResult r = imuCalResult;
  bool saved = false;
  if (r.ok) saved = saveImuCalibration(r.bias1, r.bias2, r.scale1, r.scale2);
  imucal_running = false;
  Serial.printf(">>> IMUCAL %s maxSD=%.3f saved=%d\n",
                imuCalReasonName(r.reason), (double)r.max_sd, (int)saved);
  if (connectionEstablished) {
    udp.beginPacket(laptopIP, laptopPort);
    udp.printf("IMUCAL,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.3f,%d",
               (int)r.ok, imuCalReasonName(r.reason),
               (double)r.bias1[0], (double)r.bias1[1], (double)r.bias1[2],
               (double)r.bias2[0], (double)r.bias2[1], (double)r.bias2[2],
               (double)r.scale1, (double)r.scale2, (double)r.max_sd, (int)saved);
    udp.endPacket();
  }
}

// hg <name> <value>: 위치·고도 유지 gain. 시동 해제에서만.
static void handleHoldGainCommand(char *buf) {
  char *save = nullptr;
  (void)strtok_r(buf, " \t", &save);   // "hg"
  char *name = strtok_r(nullptr, " \t", &save);
  char *arg = strtok_r(nullptr, " \t", &save);
  if (name == nullptr || arg == nullptr || strtok_r(nullptr, " \t", &save) != nullptr) return;
  float v;
  if (!parseFloatStrict(arg, v) || v < 0.0f || v > 1000.0f) return;
  if (!safety_lock || safety_arm_requested) {
    Serial.println(">>> hold gain refused (armed)");
    return;
  }
  if      (strcmp(name, "kpos") == 0)  posHoldGains.k_pos = v;
  else if (strcmp(name, "kv") == 0)    posHoldGains.k_v = v;
  else if (strcmp(name, "kvi") == 0)   posHoldGains.k_vi = v;
  else if (strcmp(name, "vmax") == 0)  posHoldGains.v_max_ms = v;
  else if (strcmp(name, "imax") == 0)  posHoldGains.i_max_deg = v;
  else if (strcmp(name, "omax") == 0)  posHoldGains.out_max_deg = v;
  else if (strcmp(name, "leash") == 0) posHoldGains.leash_m = v;
  else if (strcmp(name, "kh") == 0)    altHoldGains.k_h = v;
  else if (strcmp(name, "kvz") == 0)   altHoldGains.k_vz = v;
  else if (strcmp(name, "khi") == 0)   altHoldGains.k_i = v;
  else if (strcmp(name, "ahmax") == 0) altHoldGains.out_max_us = v;
  else { Serial.printf(">>> hold gain unknown: %s\n", name); return; }
  Serial.printf(">>> hold gain %s = %.3f\n", name, (double)v);
}

// 시동 요청. 조건을 모두 통과해야 Core 1 에 무장을 요청한다.
static inline __attribute__((always_inline)) void udpCmdStart() {
  bool overTilt = fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE;
  bool noUsableImu = imu1_frozen_now && imu2_frozen_now;
  if (fs_phase == FS_DESCENDING) {
    Serial.println(">>> START REFUSED (auto-land descending)");
  }
  else if (!safety_lock || safety_arm_requested) {
    // 이미 시동 상태. 지상국은 start를 여러 번 재전송하므로, 지연
    // 도착한 중복 start가 비행 중 fault latch를 지우고 스로틀 창을
    // 리셋하는 것을 막는다.
    Serial.println(">>> START ignored (already armed)");
  }
  else if (flowcal_running) {
    Serial.println(">>> START REFUSED (flowcal running)");
  }
  else if (imucal_running) {
    Serial.println(">>> START REFUSED (imucal running)");
  }
  else if (!calibration_ok || overTilt || noUsableImu
           || imu_disagree_now || mag_calibrating) {
    Serial.printf(">>> START REFUSED calib=%d tilt=%d imu=%d disagree=%d magcal=%d\n",
                  (int)calibration_ok, (int)overTilt,
                  (int)noUsableImu, (int)imu_disagree_now,
                  (int)mag_calibrating);
  } else {
    // safety_lock=true인 동안 Core 0 소유 비행 상태를 초기화한다.
    // hover tracker는 requestSafetyArm()이 Core 1 reset을 함께 요청한다.
    fault_rc = false;
    fault_imu1 = imu1_frozen_now;
    fault_imu2 = imu2_frozen_now;
    fault_disagree = false;
    fault_attitude = false;
    lastRcSeq = 0;
    rcSeqValid = false;
    lastRcMs = millis();
    base_throttle = 1100; min_throttle = 1050; max_throttle = 1250;
    targetAngleX = 0.0f;
    targetAngleY = 0.0f;
    targetAngleZ = 0.0f;
    angleZ = 0.0f;
    requestMagReferenceUpdate();
    fs_probe_state = FS_PROBE_WAIT;
    fs_probe_no_response = 0;
    fs_probe_response_g = 0.0f;
    requestSafetyArm();
    Serial.println(">>> START");
  }
}

// 즉시 시동 해제 요청.
static inline __attribute__((always_inline)) void udpCmdStop() {
  poshold_requested = false;
  althold_requested = false;
  requestSafetyDisarm();
  base_throttle = 1000;
  Serial.println(">>> STOP");
}

// 자동착륙 중 조종권 복귀 요청.
static inline __attribute__((always_inline)) void udpCmdResume() {
  const ResumeRefusalReason refusal = resumeRefusalReason(millis());
  if (refusal != RESUME_ALLOWED) {
    logResumeRefusal(refusal);
  } else {
    requestFailsafeResume();
  }
}

// 기준 스로틀과 스로틀 창.
static inline __attribute__((always_inline)) void udpCmdThrottle(char *buf) {
  long parsed;
  if (parseIntStrict(buf + 2, parsed)) {
    int nb = constrain((int)parsed, 1000, 1900);
    base_throttle = nb;
    min_throttle = max(1050, nb - CTRL_MARGIN);
    max_throttle = min(1900, nb + CTRL_MARGIN);
  }
}

// heading 잠금 강제(시동 해제 때만 켬).
static inline __attribute__((always_inline)) void udpCmdYawHold(char *buf) {
  long enabled;
  if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
    if (enabled == 0) {
      yaw_hold_override = false;
      Serial.println(">>> Yaw hold OFF (auto)");
    } else if (!safety_lock) {
      // 비행 중 임의로 켜면 지상국 setpoint와 어긋난 채 최대 권한
      // 슬램이 날 수 있다. 시동 해제 상태에서만 켤 수 있게 한다.
      Serial.println(">>> Yaw hold refused (armed)");
    } else {
      targetAngleZ = angleZ;
      yaw_hold_override = true;
      Serial.println(">>> Yaw hold ON (override)");
    }
  }
}

// 자기계 raw 캡처 시작·끝.
static inline __attribute__((always_inline)) void udpCmdMagcal(char *buf) {
  long enabled;
  if (parseIntStrict(buf + 6, enabled) && (enabled == 0 || enabled == 1)) {
    if (enabled == 0) {
      stopMagCalibration();
    } else if (!safety_lock) {
      Serial.println(">>> Magcal refused (armed)");
    } else if (initMagnetometer()) {
      startMagCalibration();
    }
  }
}

// 모터 전류 간섭 보정 계수.
static inline __attribute__((always_inline)) void udpCmdMagComp(char *buf) {
  char *save = nullptr;
  char *token = strtok_r(buf, " \t", &save); // "magc"
  (void)token;
  char *arg[3] = {nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 3 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (count == 3 && strtok_r(nullptr, " \t", &save) == nullptr) {
    float x, y, z;
    if (parseFloatStrict(arg[0], x) && parseFloatStrict(arg[1], y) &&
        parseFloatStrict(arg[2], z)) {
      if (configChangeAllowed("Mag comp")) {
        mag_comp_x = x;
        mag_comp_y = y;
        mag_comp_z = z;
        Serial.printf(">>> mag comp = %.4f %.4f %.4f uT/us\n",
                      (double)mag_comp_x, (double)mag_comp_y, (double)mag_comp_z);
      }
    }
  }
}

// 자기계 yaw 융합 켜고 끄기.
static inline __attribute__((always_inline)) void udpCmdMag(char *buf) {
  long enabled;
  if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
    if (enabled == 0) {
      mag_enabled = false;
      requestMagReferenceUpdate();
      Serial.println(">>> Mag OFF");
    } else if (mag_calibrating) {
      Serial.println(">>> Mag refused (magcal active)");
    } else if (!mag_ready && !safety_lock) {
      // First-time init does blocking I2C/delay(100); only when disarmed.
      Serial.println(">>> Mag refused (armed, not initialized)");
    } else if (initMagnetometer()) {
      requestMagReferenceUpdate();
      mag_enabled = true;
      Serial.println(">>> Mag ON");
    }
  }
}

// 1 kHz raw IMU 스트림.
static inline __attribute__((always_inline)) void udpCmdRaw(char *buf) {
  long enabled;
  if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
    const bool wasEnabled =
        __atomic_load_n(&raw_stream_enabled, __ATOMIC_ACQUIRE);
    if (enabled == 0) {
      __atomic_store_n(&raw_stream_enabled, false, __ATOMIC_RELEASE);
      Serial.println(">>> Raw IMU OFF");
    } else {
      if (!wasEnabled) {
        const uint32_t head =
            __atomic_load_n(&imuRawRing.head, __ATOMIC_ACQUIRE);
        __atomic_store_n(&imuRawRing.tail, head, __ATOMIC_RELEASE);
        rawProducerTimeValid = false;
        rawConsumerTimeValid = false;
        rawLastSendMs = millis();
        rawLastCalMs = rawLastSendMs - 1000U;
        __atomic_store_n(
            &raw_stream_enabled, true, __ATOMIC_RELEASE);
      }
      Serial.println(">>> Raw IMU ON");
    }
  }
}

// 절대 트림.
static inline __attribute__((always_inline)) void udpCmdTrim(char *buf) {
  char *save = nullptr;
  (void)strtok_r(buf, " \t", &save);          // "trim"
  char *arg0 = strtok_r(nullptr, " \t", &save);
  char *arg1 = strtok_r(nullptr, " \t", &save);
  if (arg0 != nullptr && arg1 != nullptr &&
      strtok_r(nullptr, " \t", &save) == nullptr) {
    float r, p;
    if (parseFloatStrict(arg0, r) && parseFloatStrict(arg1, p)) {
      if (configChangeAllowed("Trim")) {
        trim_roll  = constrain(r, -TRIM_MAX_DEG, TRIM_MAX_DEG);
        trim_pitch = constrain(p, -TRIM_MAX_DEG, TRIM_MAX_DEG);
        Serial.printf(">>> Trim R:%.2f P:%.2f\n", trim_roll, trim_pitch);
      }
    }
  }
}

// 광류 위치 유지 요청.
static inline __attribute__((always_inline)) void udpCmdPosHold(char *buf) {
  long enabled;
  if (parseIntStrict(buf + 7, enabled) && (enabled == 0 || enabled == 1)) {
    if (enabled == 0) {
      poshold_requested = false;
      Serial.println(">>> PosHold OFF");
    } else if (!flowOdomCfg.calibrated) {
      Serial.println(">>> PosHold refused (flow not calibrated)");
    } else {
      poshold_requested = true;
      Serial.println(">>> PosHold ON");
    }
  }
}

// 거리 기반 자동착륙 스위치 요청.
static inline __attribute__((always_inline)) void udpCmdRangeLand(char *buf) {
  long enabled;
  if (parseIntStrict(buf + 9, enabled) && (enabled == 0 || enabled == 1)) {
    requestRangeLand((int8_t)enabled);   // 허용 판단은 Core 1 (시동 해제 때만)
  }
}

// 고도 유지 요청.
static inline __attribute__((always_inline)) void udpCmdAltHold(char *buf) {
  long enabled;
  if (parseIntStrict(buf + 7, enabled) && (enabled == 0 || enabled == 1)) {
    althold_requested = (enabled == 1);
    Serial.println(enabled ? ">>> AltHold ON" : ">>> AltHold OFF");
  }
}

// 광류 보정 시작·끝.
static inline __attribute__((always_inline)) void udpCmdFlowCal(char *buf) {
  long enabled;
  if (parseIntStrict(buf + 7, enabled) && (enabled == 0 || enabled == 1)) {
    if (enabled == 0) {
      if (flowcal_running) {
        flowcal_running = false;
        requestFlowCalCommand(FLOWCAL_CMD_STOP);
        Serial.println(">>> FlowCal STOP (solving)");
      }
    } else if (!safety_lock || safety_arm_requested) {
      Serial.println(">>> FlowCal refused (armed)");
    } else if (imucal_running) {
      Serial.println(">>> FlowCal refused (imucal running)");
    } else {
      flowcal_running = true;
      requestFlowCalCommand(FLOWCAL_CMD_START);
      Serial.println(">>> FlowCal START - shake roll, then pitch, 40~70cm");
    }
  }
}

// 정지 IMU 보정 요청.
static inline __attribute__((always_inline)) void udpCmdImuCal() {
  // 바닥에 가만히 둔 상태에서만. 2 s 동안 Core 1 이 샘플을 모은다.
  if (!safety_lock || safety_arm_requested) {
    Serial.println(">>> IMUCAL refused (armed)");
  } else if (flowcal_running) {
    Serial.println(">>> IMUCAL refused (flowcal running)");
  } else if (!imucal_running) {
    imucal_running = true;
    portENTER_CRITICAL(&flowRequestMux);
    imucal_request = true;
    portEXIT_CRITICAL(&flowRequestMux);
    Serial.println(">>> IMUCAL START - keep the drone still on the floor (2 s)");
  }
}

// 현재 PID 게인 응답.
static inline __attribute__((always_inline)) void udpCmdGains() {
  if (connectionEstablished) {
    udp.beginPacket(laptopIP, laptopPort);
    udp.printf("GAINS,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
               Kp_Angle_Roll, Kp_Angle_Pitch, Kp_Angle_Yaw,
               Kp_Rate_Roll, Kp_Rate_Pitch, Kp_Rate_Yaw,
               Ki_Rate_Roll, Ki_Rate_Pitch, Ki_Rate_Yaw,
               Kd_Rate_Roll, Kd_Rate_Pitch, Kd_Rate_Yaw);
    udp.endPacket();
  }
}

// 명령 하나를 처리한다. 비교 순서는 그대로 둔다(접두사가 겹친다: rcr/rc,
// magcal/magc/mag, raw/rangeland). 나머지는 PID 게인 명령으로 넘긴다.
static inline __attribute__((always_inline)) void udpDispatchCommand(char *buf) {
  if (strncmp(buf, "rcr", 3) == 0 && (buf[3] == ' ' || buf[3] == '\t')) {
    handleRcrCommand(buf);
  }
  else if (strncmp(buf, "rc", 2) == 0 && (buf[2] == ' ' || buf[2] == '\t')) {
    handleRcCommand(buf);
  }
  else if (strcmp(buf, "start") == 0) {
    udpCmdStart();
  }
  else if (strcmp(buf, "stop") == 0) {
    udpCmdStop();
  }
  else if (strcmp(buf, "resume") == 0) {
    udpCmdResume();
  }
  else if (strncmp(buf, "th", 2) == 0) {
    udpCmdThrottle(buf);
  }
  else if (strncmp(buf, "yaw", 3) == 0) {
    udpCmdYawHold(buf);
  }
  else if (strncmp(buf, "magcal", 6) == 0) {
    udpCmdMagcal(buf);
  }
  else if (strncmp(buf, "magc", 4) == 0) {
    udpCmdMagComp(buf);
  }
  else if (strncmp(buf, "mag", 3) == 0) {
    udpCmdMag(buf);
  }
  else if (strncmp(buf, "raw", 3) == 0) {
    udpCmdRaw(buf);
  }
  else if (strncmp(buf, "trim", 4) == 0 &&
           (buf[4] == ' ' || buf[4] == '\t')) {
    udpCmdTrim(buf);
  }
  else if (strncmp(buf, "poshold", 7) == 0) {
    udpCmdPosHold(buf);
  }
  else if (strncmp(buf, "rangeland", 9) == 0 &&
           (buf[9] == ' ' || buf[9] == '\t')) {
    udpCmdRangeLand(buf);
  }
  else if (strncmp(buf, "althold", 7) == 0) {
    udpCmdAltHold(buf);
  }
  else if (strncmp(buf, "flowcal", 7) == 0) {
    udpCmdFlowCal(buf);
  }
  else if (strcmp(buf, "imucal") == 0) {
    udpCmdImuCal();
  }
  else if (strncmp(buf, "hg", 2) == 0 && (buf[2] == ' ' || buf[2] == '\t')) {
    handleHoldGainCommand(buf);
  }
  else if (strcmp(buf, "gains") == 0) {
    udpCmdGains();
  }
  else {
    handleGainCommand(buf);
  }
}

// 받은 패킷 하나: 보낸 쪽을 텔레메트리 목적지로 등록하고 명령을 처리한다.
static inline __attribute__((always_inline)) void udpReceivePacket() {
  laptopIP = udp.remoteIP();
  laptopPort = udp.remotePort();
  connectionEstablished = true;

  int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1); // off-by-one 방지
  if (len > 0) {
    packetBuffer[len] = '\0';
    char *buf = trimCommand(packetBuffer);

    udpDispatchCommand(buf);
  }
}

// 매 반복의 센서 폴링과 송신: 자기계 샘플, 3901-L0X 폴링, 보정 결과, raw IMU 배치·보정
// 상수(한 반복에 하나), 20 Hz 텔레메트리. lastSend·lastRawCheck 는 udp_task 의 지역이다.
static inline __attribute__((always_inline)) void udpPeriodicSends(uint32_t &lastSend,
                                                                   uint32_t &lastRawCheck) {
  uint32_t now = millis();
  if (mag_enabled || mag_calibrating) sampleMagnetometer(now);
  pollMspSensor(now);
  reportFlowCalResult();
  reportImuCalResult();
  bool sentImuPacket = false;
  if (now - lastRawCheck >= 10U) {
    lastRawCheck = now;
    bool sentRawPacket = false;
    if (__atomic_load_n(&raw_stream_enabled, __ATOMIC_ACQUIRE)
        && connectionEstablished) {
      const uint32_t queued = imuRawRingCount(imuRawRing);
      if (queued >= IMU_RAW_BATCH_MAX
          || (queued > 0 && now - rawLastSendMs >= 50U)) {
        sentImuPacket = sendImuRawBatch();
        sentRawPacket = sentImuPacket;
        if (sentImuPacket) rawLastSendMs = now;
      }
      // 한 udp_task 반복에서는 ZIMU 또는 ZCAL 중 최대 하나만 추가 전송한다.
      if (!sentRawPacket && now - rawLastCalMs >= 1000U) {
        sentImuPacket = sendImuCalibration();
        if (sentImuPacket) rawLastCalMs = now;
      }
    }
  }
  // 새 바이너리 경로는 한 반복에 최대 한 데이터그램만 보낸다. 같은 시각의
  // 기존 ASCII 텔레메트리는 다음 반복으로 미뤄 20Hz 주기를 유지한다.
  if (!sentImuPacket && now - lastSend >= 50) {
    lastSend = now;
#if WIFI_LATENCY_DEBUG
    uint32_t sendStartUs = micros();
#endif
    sendTelemetry();
#if WIFI_LATENCY_DEBUG
    uint32_t sendDurUs = micros() - sendStartUs;
    if (sendDurUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
      recordLatEvent(udpLatRing, millis(), sendDurUs, LAT_UDP_SEND);
    }
#endif
  }
}

void udp_task(void *pv) {
  uint32_t lastSend = 0;
  uint32_t lastRawCheck = 0;
#if WIFI_LATENCY_DEBUG
  uint32_t prevTopUs = micros();
#endif
  while (true) {
#if WIFI_LATENCY_DEBUG
    uint32_t topUs = micros();
    uint32_t loopGapUs = topUs - prevTopUs;
    prevTopUs = topUs;
    if (loopGapUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
      recordLatEvent(udpLatRing, millis(), loopGapUs, LAT_UDP_LOOP);
    }
    uint32_t parseStartUs = micros();
#endif
    int packetSize = udp.parsePacket();
#if WIFI_LATENCY_DEBUG
    uint32_t parseDurUs = micros() - parseStartUs;
    if (parseDurUs > WIFI_LATENCY_UDP_THRESHOLD_US) {
      recordLatEvent(udpLatRing, millis(), parseDurUs, LAT_UDP_PARSE);
    }
#endif
    if (packetSize) {
      udpReceivePacket();
    }
    udpPeriodicSends(lastSend, lastRawCheck);
    vTaskDelay(1);
  }
}
