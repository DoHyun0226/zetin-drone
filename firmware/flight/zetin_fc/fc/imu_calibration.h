// zetin_fc/fc/imu_calibration.h — 6. gyro bias·가속도 보정, NVS 저장·불러오기, imucal.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// ==========================================================
// 6. Gyro bias 캘리브레이션
// ==========================================================
// sign[]은 각 IMU를 drone(IMU1) frame으로 맞추는 부호.
// 정지 상태에서 1ms 간격으로 2000개를 읽어
//  bias_out       : 축별 자이로 평균 (정지 중이면 진짜 각속도는 0이므로 평균 = 영점 오차)
//  sd_out         : 축별 표준편차 (크면 캘리브레이션 중 기체가 움직였다는 뜻)
//  accel_mean_out : |accel| 평균 (정지 중이면 1g여야 함 → 가속도 배율 보정에 사용)
// 분산 = E[x²] - (E[x])² 으로 한 번 훑으면서 평균과 표준편차를 같이 구한다
static bool measure_imu_bias(ICM42670 &imu, const float sign[3],
                             float bias_out[3], float sd_out[3],
                             float &accel_mean_out) {
  double sum[3] = {0,0,0}, sum_sq[3] = {0,0,0};
  double accel_mag_sum = 0.0;
  inv_imu_sensor_event_t e = {};
  int samples = 0;
  int attempts = 0;
  const int maxAttempts = BIAS_CALIB_SAMPLES + 100;
  while (samples < BIAS_CALIB_SAMPLES && attempts < maxAttempts) {
    attempts++;
    if (imu.getDataFromRegisters(e) != 0) {
      delayMicroseconds(1000);
      continue;
    }
    float g[3] = {
      sign[0] * e.gyro[0] * GYRO_SCALE,
      sign[1] * e.gyro[1] * GYRO_SCALE,
      sign[2] * e.gyro[2] * GYRO_SCALE
    };
    for (int k = 0; k < 3; k++) { sum[k] += g[k]; sum_sq[k] += (double)g[k]*g[k]; }
    const double ax = (double)e.accel[0] * ACCEL_SCALE;
    const double ay = (double)e.accel[1] * ACCEL_SCALE;
    const double az = (double)e.accel[2] * ACCEL_SCALE;
    accel_mag_sum += sqrt(ax*ax + ay*ay + az*az);
    samples++;
    delayMicroseconds(1000);
  }
  if (samples != BIAS_CALIB_SAMPLES) return false;
  accel_mean_out = (float)(accel_mag_sum / samples);
  for (int k = 0; k < 3; k++) {
    double mean = sum[k] / samples;
    double var  = sum_sq[k] / samples - mean*mean;
    if (var < 0) var = 0;
    bias_out[k] = (float)mean;
    sd_out[k]   = (float)sqrt(var);
  }
  return true;
}

// 정지 시 |accel| 평균이 1g가 되도록 하는 배율 = 1 / 평균. 0.8~1.2g 밖이면 센서 이상으로 보고 보정하지 않음(1.0)
static float guarded_accel_scale(float accel_mean_g, const char *imu_name) {
  if (accel_mean_g < ACCEL_CAL_MIN_G || accel_mean_g > ACCEL_CAL_MAX_G) {
    Serial.printf(
        "[CALIB] WARN %s accel |a| %.4fg outside [0.8,1.2], scale=1.0\n",
        imu_name, accel_mean_g);
    return 1.0f;
  }
  return 1.0f / accel_mean_g;
}

// 부팅 시 두 IMU의 자이로 bias와 가속도 배율을 측정. 움직임(표준편차 > 1dps)이 감지되면 최대 3번 다시 시도
// 실패하면 calibration_ok = false → start 명령이 거부된다
static bool calibrate_bias() {
  float sd1[3], sd2[3];
  accel_scale1 = 1.0f;
  accel_scale2 = 1.0f;
  for (int a = 1; a <= BIAS_CALIB_RETRIES; a++) {
    Serial.printf("[CALIB] attempt %d/%d (hold still)...\n", a, BIAS_CALIB_RETRIES);
    float accel_mean1 = 0.0f, accel_mean2 = 0.0f;
    bool read1 = measure_imu_bias(
        IMU1, IMU1_SIGN, gyro_bias1, sd1, accel_mean1);
    bool read2 = measure_imu_bias(
        IMU2, IMU2_SIGN, gyro_bias2, sd2, accel_mean2);
    if (!read1 || !read2) {
      Serial.printf("[CALIB] sensor read failed (imu1=%d imu2=%d)\n", (int)read1, (int)read2);
      continue;
    }
    float max_sd = max(max(max(sd1[0],sd1[1]),sd1[2]),
                       max(max(sd2[0],sd2[1]),sd2[2]));
    Serial.printf("[CALIB] IMU1 %.3f %.3f %.3f | IMU2 %.3f %.3f %.3f (maxSD %.3f)\n",
                  gyro_bias1[0],gyro_bias1[1],gyro_bias1[2],
                  gyro_bias2[0],gyro_bias2[1],gyro_bias2[2], max_sd);
    if (max_sd <= BIAS_MOVEMENT_THRESH) {
      accel_scale1 = guarded_accel_scale(accel_mean1, "IMU1");
      accel_scale2 = guarded_accel_scale(accel_mean2, "IMU2");
      Serial.printf(
          "[CALIB] accel mean IMU1 %.4fg scale %.6f | "
          "IMU2 %.4fg scale %.6f\n",
          accel_mean1, accel_scale1, accel_mean2, accel_scale2);
      Serial.println("[CALIB] OK");
      return true;
    }
    Serial.println("[CALIB] movement detected, retry");
  }
  Serial.println("[CALIB] FAIL: reboot and calibrate on a stationary surface");
  return false;
}

static bool imuCalPlausible(const float b1[3], const float b2[3], float s1, float s2) {
  for (int k = 0; k < 3; k++) {
    if (!isfinite(b1[k]) || !isfinite(b2[k])) return false;
    if (fabsf(b1[k]) > IMU_CAL_MAX_BIAS_DPS || fabsf(b2[k]) > IMU_CAL_MAX_BIAS_DPS) return false;
  }
  const float lo = 1.0f / ACCEL_CAL_MAX_G, hi = 1.0f / ACCEL_CAL_MIN_G;
  return isfinite(s1) && isfinite(s2) && s1 >= lo && s1 <= hi && s2 >= lo && s2 <= hi;
}

static bool saveImuCalibration(const float b1[3], const float b2[3], float s1, float s2) {
  StoredImuCal c = {};
  c.magic = IMU_CAL_MAGIC;
  for (int k = 0; k < 3; k++) { c.bias1[k] = b1[k]; c.bias2[k] = b2[k]; }
  c.scale1 = s1;
  c.scale2 = s2;
  Preferences prefs;
  if (!prefs.begin("zimu", false)) return false;
  const bool ok = prefs.putBytes("cal", &c, sizeof(c)) == sizeof(c);
  prefs.end();
  return ok;
}

// 부팅 때 보정 대신 부른다. NVS(지상국 imucal 결과) > 컴파일 값. 둘 다 없으면 false.
static bool loadImuCalibration() {
  StoredImuCal c = {};
  Preferences prefs;
  bool fromNvs = false;
  if (prefs.begin("zimu", true)) {
    fromNvs = prefs.getBytesLength("cal") == sizeof(c) &&
              prefs.getBytes("cal", &c, sizeof(c)) == sizeof(c) &&
              c.magic == IMU_CAL_MAGIC &&
              imuCalPlausible(c.bias1, c.bias2, c.scale1, c.scale2);
    prefs.end();
  }
  if (!fromNvs) {
    if (!STORED_IMU_CAL_VALID) return false;
    for (int k = 0; k < 3; k++) { c.bias1[k] = STORED_GYRO_BIAS1[k]; c.bias2[k] = STORED_GYRO_BIAS2[k]; }
    c.scale1 = STORED_ACCEL_SCALE1;
    c.scale2 = STORED_ACCEL_SCALE2;
  }
  for (int k = 0; k < 3; k++) { gyro_bias1[k] = c.bias1[k]; gyro_bias2[k] = c.bias2[k]; }
  accel_scale1 = c.scale1;
  accel_scale2 = c.scale2;
  Serial.printf("[CALIB] stored (%s) IMU1 %.3f %.3f %.3f | IMU2 %.3f %.3f %.3f\n",
                fromNvs ? "NVS" : "compiled",
                gyro_bias1[0], gyro_bias1[1], gyro_bias1[2],
                gyro_bias2[0], gyro_bias2[1], gyro_bias2[2]);
  return true;
}

static const char *imuCalReasonName(uint8_t r) {
  switch (r) {
    case IMUCAL_OK: return "OK";
    case IMUCAL_MOVED: return "MOVED";
    case IMUCAL_ACCEL: return "ACCEL";
    case IMUCAL_READ: return "READ";
    case IMUCAL_ARMED: return "ARMED";
    default: return "?";
  }
}

// Core 1. 시동 해제 중 매 tick 정지 샘플을 모으고, 다 모이면 판정해 적용한다.
static void imuCalStep(bool readOk, const inv_imu_sensor_event_t &a,
                       const inv_imu_sensor_event_t &b) {
  if (takeImuCalRequest() && safety_lock) {
    imuCalAcc = ImuCalAccum();
    imuCalAcc.active = true;
  }
  if (!imuCalAcc.active) return;
  ImuCalResult r = {};
  bool done = false;
  if (!safety_lock) {
    r.reason = IMUCAL_ARMED;
    done = true;
  } else {
    imuCalAcc.ticks++;
    if (readOk) {
      for (int k = 0; k < 3; k++) {
        const double g1 = IMU1_SIGN[k] * a.gyro[k] * GYRO_SCALE;
        const double g2 = IMU2_SIGN[k] * b.gyro[k] * GYRO_SCALE;
        imuCalAcc.sum1[k] += g1; imuCalAcc.sq1[k] += g1 * g1;
        imuCalAcc.sum2[k] += g2; imuCalAcc.sq2[k] += g2 * g2;
      }
      const double ax1 = a.accel[0] * ACCEL_SCALE, ay1 = a.accel[1] * ACCEL_SCALE, az1 = a.accel[2] * ACCEL_SCALE;
      const double ax2 = b.accel[0] * ACCEL_SCALE, ay2 = b.accel[1] * ACCEL_SCALE, az2 = b.accel[2] * ACCEL_SCALE;
      imuCalAcc.amag1 += sqrt(ax1 * ax1 + ay1 * ay1 + az1 * az1);
      imuCalAcc.amag2 += sqrt(ax2 * ax2 + ay2 * ay2 + az2 * az2);
      imuCalAcc.n++;
    }
    if (imuCalAcc.n >= (uint32_t)IMU_CAL_SAMPLES) {
      const double n = imuCalAcc.n;
      float maxSd = 0.0f;
      for (int k = 0; k < 3; k++) {
        const double m1 = imuCalAcc.sum1[k] / n, m2 = imuCalAcc.sum2[k] / n;
        const double v1 = imuCalAcc.sq1[k] / n - m1 * m1, v2 = imuCalAcc.sq2[k] / n - m2 * m2;
        maxSd = fmaxf(maxSd, (float)sqrt(v1 > 0 ? v1 : 0));
        maxSd = fmaxf(maxSd, (float)sqrt(v2 > 0 ? v2 : 0));
        r.bias1[k] = (float)m1;
        r.bias2[k] = (float)m2;
      }
      const float am1 = (float)(imuCalAcc.amag1 / n), am2 = (float)(imuCalAcc.amag2 / n);
      r.max_sd = maxSd;
      if (maxSd > BIAS_MOVEMENT_THRESH) {
        r.reason = IMUCAL_MOVED;
      } else if (am1 < ACCEL_CAL_MIN_G || am1 > ACCEL_CAL_MAX_G ||
                 am2 < ACCEL_CAL_MIN_G || am2 > ACCEL_CAL_MAX_G) {
        r.reason = IMUCAL_ACCEL;
      } else {
        r.ok = true;
        r.reason = IMUCAL_OK;
        r.scale1 = 1.0f / am1;
        r.scale2 = 1.0f / am2;
        for (int k = 0; k < 3; k++) { gyro_bias1[k] = r.bias1[k]; gyro_bias2[k] = r.bias2[k]; }
        accel_scale1 = r.scale1;
        accel_scale2 = r.scale2;
        calibration_ok = true;
      }
      done = true;
    } else if (imuCalAcc.ticks >= IMU_CAL_MAX_TICKS) {
      r.reason = IMUCAL_READ;
      done = true;
    }
  }
  if (done) {
    imuCalAcc.active = false;
    imuCalResult = r;
    imucal_result_pending = true;   // Core 0 가 저장·출력한다
  }
}
