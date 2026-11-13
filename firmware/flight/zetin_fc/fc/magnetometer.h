// zetin_fc/fc/magnetometer.h — BMM350 초기화·보정·샘플링.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
static bool initMagnetometer() {
  if (mag_ready) return true;

  pinMode(LDO_SHD, OUTPUT);
  digitalWrite(LDO_SHD, HIGH);
  delay(100);
  Wire.begin(BMM_SDA, BMM_SCL);
  uint8_t status = bmm.begin();
  if (status != 0) {
    Serial.printf("[MAG] BMM350 init failed (%u)\n", (unsigned)status);
    return false;
  }
  bmm.setOperationMode(eBmm350NormalMode);
  bmm.setRate(BMM350_DATA_RATE_50HZ);
  mag_ready = true;
  lastMagReadMs = millis() - MAG_SAMPLE_PERIOD_MS;
  Serial.println("[MAG] BMM350 ready @50Hz");
  return true;
}

static void startMagCalibration() {
  portENTER_CRITICAL(&magSnapshotMux);
  const bool alreadyActive = mag_calibrating;
  portEXIT_CRITICAL(&magSnapshotMux);
  if (alreadyActive) {
    Serial.println("[MAGCAL] already active");
    return;
  }
  mag_enabled = false;
  requestMagReferenceUpdate();
  portENTER_CRITICAL(&magSnapshotMux);
  mag_calibrating = true;
  portEXIT_CRITICAL(&magSnapshotMux);
  magCalMin[0] = FLT_MAX;
  magCalMin[1] = FLT_MAX;
  magCalMin[2] = FLT_MAX;
  magCalMax[0] = -FLT_MAX;
  magCalMax[1] = -FLT_MAX;
  magCalMax[2] = -FLT_MAX;
  magCalSampleCount = 0;
  Serial.println("[MAGCAL] logging raw XYZ; rotate drone, then send 'magcal 0'");
}

static void stopMagCalibration() {
  MagSnapshot calibrated = {};
  const bool have_sample =
      magCalSampleCount > 0U && readMagSnapshot(calibrated);
  if (have_sample) {
    const float magThrDelta =
        (float)(base_throttle - MAG_THROTTLE_REF_US);
    calibrated.x -= mag_comp_x * magThrDelta;
    calibrated.y -= mag_comp_y * magThrDelta;
    calibrated.z -= mag_comp_z * magThrDelta;
  }

  portENTER_CRITICAL(&magSnapshotMux);
  mag_calibrating = false;
  if (have_sample) {
    magTelemX = calibrated.x;
    magTelemY = calibrated.y;
    magTelemZ = calibrated.z;
  }
  magTelemCalActive = false;
  portEXIT_CRITICAL(&magSnapshotMux);
  if (magCalSampleCount == 0U) {
    Serial.println("[MAGCAL] no samples");
    return;
  }
  Serial.printf("[MAGCAL] samples=%lu\n", (unsigned long)magCalSampleCount);
  Serial.printf("[MAGCAL] span X=%.6f uT Y=%.6f uT Z=%.6f uT\n",
                magCalMax[0] - magCalMin[0],
                magCalMax[1] - magCalMin[1],
                magCalMax[2] - magCalMin[2]);
  Serial.println(
      "[MAGCAL] save the telemetry CSV and run scripts/drone_logs.py magcal <flight_log.csv>");
}

static inline void applyMagCalibration(
    const float raw[3], const float hard_iron[3],
    const float soft_iron[3][3], float out[3]) {
  const float centered[3] = {
    raw[0] - hard_iron[0],
    raw[1] - hard_iron[1],
    raw[2] - hard_iron[2],
  };
  const float body_sign[3] = {
    MAG_BODY_SIGN_X, MAG_BODY_SIGN_Y, MAG_BODY_SIGN_Z,
  };
  for (int axis = 0; axis < 3; axis++) {
    float transformed = 0.0f;
    for (int source = 0; source < 3; source++) {
      transformed += soft_iron[axis][source] * centered[source];
    }
    out[axis] = body_sign[axis] * transformed;
  }
}

static void sampleMagnetometer(uint32_t now_ms) {
  if (!mag_ready || now_ms - lastMagReadMs < MAG_SAMPLE_PERIOD_MS) return;
  lastMagReadMs = now_ms;

  // Blocking I2C read: caller is udp_task on core 0, never pid_task.
  sBmm350MagData_t data = bmm.getGeomagneticData();
  const float raw[3] = {data.float_x, data.float_y, data.float_z};
  if (!isfinite(raw[0]) || !isfinite(raw[1]) || !isfinite(raw[2])) return;

  if (mag_calibrating) {
    publishMagTelemetry(raw[0], raw[1], raw[2], true);
    for (int axis = 0; axis < 3; axis++) {
      if (raw[axis] < magCalMin[axis]) magCalMin[axis] = raw[axis];
      if (raw[axis] > magCalMax[axis]) magCalMax[axis] = raw[axis];
    }
    magCalSampleCount++;
  }

  float calibrated[3];
  applyMagCalibration(raw, MAG_HARD_IRON, MAG_SOFT_IRON, calibrated);
  publishMagSample(calibrated[0], calibrated[1], calibrated[2], now_ms);
}

