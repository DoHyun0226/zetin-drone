// zetin_fc/fc/magnetometer.h — BMM350 자기계: 공유 상태, 샘플·텔레메트리 스냅숏과 기준 갱신 요청, 초기화·보정·샘플링.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
// Core 0 writer -> Core 1 reader. 짝수 seq만 완성된 벡터다.
// "mag 1": 자기계를 yaw 융합에 사용
volatile bool  mag_enabled = false;
// BMM350 초기화 완료
volatile bool  mag_ready = false;
// "magcal 1": 캘리브레이션용 원시값 수집 중
volatile bool  mag_calibrating = false;
// 보정된 최신 자기장 (Core 0 → Core 1)
volatile float magFieldX = 0.0f, magFieldY = 0.0f, magFieldZ = 0.0f;
// 마지막 기울기 보정 heading (텔레메트리)
volatile float magHeading = 0.0f;
// 텔레메트리로 보낼 자기장
volatile float magTelemX = 0.0f, magTelemY = 0.0f, magTelemZ = 0.0f;
// 위 값이 캘리브레이션용 원시값인가
volatile bool magTelemCalActive = false;
// 최신 자기장 측정 시각
volatile uint32_t magSampleMs = 0;
// 한 번이라도 측정값이 들어왔는가
volatile bool magSampleValid = false;
// 다음 융합 때 기준 오프셋을 다시 잡아라
volatile bool mag_reference_pending = true;

// 자기계 관련 공유 변수 보호
portMUX_TYPE magSnapshotMux = portMUX_INITIALIZER_UNLOCKED;
// 나침반 heading(자북 기준)을 angleZ 기준으로 옮기는 오프셋 = angleZ - heading (기준을 잡는 순간 값)
//  mag을 켜거나 start로 angleZ를 0으로 만들 때 다시 잡는다 → 그 순간 오차가 0이라 갑자기 북쪽으로 도는 일이 없다
float magYawReferenceOffset = 0.0f;

// 캘리브레이션 중 축별 최소/최대 (span 출력용. 실제 보정값은 PC의 magcal_fit.py가 계산)
float magCalMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
float magCalMax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
uint32_t magCalSampleCount = 0;
// 마지막 BMM350 읽기 시각 (20ms 간격 유지)
uint32_t lastMagReadMs = 0;

// [Core 0 → Core 1] 보정된 자기장 최신값을 게시 / 읽기 (publish = 쓰는 쪽, read = 읽는 쪽)
static inline void publishMagSample(
    float x, float y, float z, uint32_t sample_ms) {
  // I2C 읽기는 이 잠금 전에 끝난다. 임계구역은 네 워드만 게시하므로 pid_task 가
  // 막히는 I2C 전송을 기다리는 일은 없다.
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

// 텔레메트리로 보낼 자기장 값을 게시. calibration_raw=true면 캘리브레이션용 원시값, false면 간섭 보정까지 끝난 값
static inline void publishMagTelemetry(
    float x, float y, float z, bool calibration_raw) {
  portENTER_CRITICAL(&magSnapshotMux);
  // Core 0 이 보정을 시작할 때 Core 1 이 이미 보정값 영역 블록에 들어가 있을 수 있다.
  // 그 늦은 쓰기가 raw 스냅숏을 덮어쓰지 못하게 한다.
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

// 다음 자기계 융합 때 magYawReferenceOffset을 다시 잡으라고 요청 / 그 요청을 꺼내며 지우기
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

// BMM350 자기계 초기화 (처음 한 번만). delay(100)와 I2C 통신으로 블로킹되므로 Core 0에서, 가능하면 시동 해제 상태에서 호출
static bool initMagnetometer() {
  if (mag_ready) return true;

  pinMode(LDO_SHD, OUTPUT);
  // 자기계 전원 켜기
  digitalWrite(LDO_SHD, HIGH);
  // 전원 안정화 대기
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
  // 다음 udp_task 반복에서 바로 첫 샘플을 읽도록
  lastMagReadMs = millis() - MAG_SAMPLE_PERIOD_MS;
  Serial.println("[MAG] BMM350 ready @50Hz");
  return true;
}

// "magcal 1": 캘리브레이션 시작. 융합을 끄고 원시값을 텔레메트리로 내보낸다
//  기체를 여러 방향으로 돌린 뒤 "magcal 0" → PC에서 CSV를 magcal_fit.py로 타원 피팅해 MAG_HARD_IRON/SOFT_IRON을 구한다
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

// "magcal 0": 캘리브레이션 종료. 텔레메트리를 보정값으로 되돌리고 축별 span(최대-최소)을 출력한다
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

// 자기계 보정: out = body_sign × (SOFT_IRON 행렬 × (raw - HARD_IRON))
//  1) hard-iron을 빼 타원의 중심을 원점으로 옮기고  2) soft-iron 행렬로 타원을 구로 편 뒤  3) 축 부호를 body에 맞춘다
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

// [Core 0] 20ms(50Hz)마다 BMM350을 읽어 보정 후 Core 1에 게시. 캘리브레이션 중이면 원시값도 함께 내보낸다
static void sampleMagnetometer(uint32_t now_ms) {
  if (!mag_ready || now_ms - lastMagReadMs < MAG_SAMPLE_PERIOD_MS) return;
  lastMagReadMs = now_ms;

  // 막히는 I2C 읽기다. 부르는 쪽은 Core 0 의 udp_task 이고 pid_task 는 부르지 않는다.
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
