// dual_imu_flix_quat_pwm — flix(https://github.com/okalachev/flix) 아키텍처를
// ZETIN 하드웨어(ESP32-S3 + 듀얼 ICM42670 + PWM ESC)에 이식한 비행 펌웨어.
//
// dual_imu_cascade_pwm과의 차이:
// - 자세 추정: 축별 상보필터 대신 쿼터니언 적분 + 착지 시 가속도 보정 +
//   비행 중 수평 가정 보정 (flix estimate.ino 이식).
// - 제어: 자세 오차를 up-벡터 회전벡터로 계산하는 flix 캐스케이드.
//   내부 단위는 SI(rad, rad/s)와 정규화 토크(1.0 = 모터 전 구간)다.
// - 믹서: flix와 동일한 X-quad 부호. 프로펠러 FL/RR=CW, FR/RL=CCW 기준.
// 듀얼 IMU 융합/freeze/불일치 감시, 워치독, UDP 프로토콜은 기존 것을 유지한다.
//
// ==== 좌표계 ====
// body frame은 오른손 FLU(x 앞, y 왼쪽, z 위)다. IMU1 sensor frame 기준:
//   body = (-sensor_y, +sensor_x, +sensor_z)   (z축 +90도 회전)
// roll+ = 오른쪽이 내려감, pitch+ = 기수가 내려감, yaw+ = 위에서 볼 때 CCW.
// 이 매핑은 구형 펌웨어에서 실기 검증된 roll/pitch 자이로 변환과 같고,
// yaw 부호만 오른손 좌표계에 맞게 반전된다(구형: CW+, 신형: CCW+).
// 구형 펌웨어의 가속도 x,y 부호는 자이로 매핑과 강체 조건에서 양립할 수 없어
// 자이로 기준으로 통일했다. 정지 시 body accel이 (0,0,+1g) 근처인지
// 캘리브레이션에서 검증하며, 어긋나면 시동을 거부한다. 첫 비행 전 반드시
// README의 벤치 부호 점검을 수행할 것.

#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <errno.h>

#include "vector.h"
#include "quaternion.h"
#include "lpf.h"
#include "pid.h"

// ==========================================================
// 1. 튜닝 파라미터 (UDP로 실시간 변경 가능)
// ==========================================================
// [Outer] 각도 P: 자세 오차(rad) -> 목표 각속도(rad/s). flix 기본값.
volatile float Kp_Angle_Roll  = 6.0f;
volatile float Kp_Angle_Pitch = 6.0f;
volatile float Kp_Angle_Yaw   = 3.0f;

// [Inner] 각속도 PID: 오차(rad/s) -> 정규화 토크. flix 기본값.
// 주의: dual_imu_cascade_pwm과 단위가 다르다 (µs/deg/s 아님).
volatile float Kp_Rate_Roll  = 0.05f, Ki_Rate_Roll  = 0.2f, Kd_Rate_Roll  = 0.001f;
volatile float Kp_Rate_Pitch = 0.05f, Ki_Rate_Pitch = 0.2f, Kd_Rate_Pitch = 0.001f;
volatile float Kp_Rate_Yaw   = 0.3f,  Ki_Rate_Yaw   = 0.0f, Kd_Rate_Yaw   = 0.0f;

volatile int  base_throttle = 1000;
volatile int  min_throttle  = 1050;
volatile int  max_throttle  = 1300;
volatile bool yaw_enabled   = false;   // "yaw 1" 로 활성화

// ==========================================================
// 2. 시스템 상수
// ==========================================================
const char* WIFI_SSID    = "Drone_Tuning";
const char* WIFI_PASS    = "12345678";
const int   UDP_PORT     = 4210;
const int   WIFI_CHANNEL = 6;

const int pinM1   = 4;   // FL (CW)
const int pinM2   = 5;   // RR (CW)
const int pinM3   = 6;   // FR (CCW)
const int pinM4   = 7;   // RL (CCW)
const int SPI_CS1 = 10;  // IMU1
const int SPI_CS2 = 9;   // IMU2

const int ESC_FREQ    = 400;
const int ESC_RES     = 14;
const int ESC_PERIOD  = 2500;                 // us
const int ESC_MAXDUTY = (1 << ESC_RES) - 1;

const float GYRO_SCALE  = 1.0f / 16.4f;       // raw -> deg/s
const float ACCEL_SCALE = 1.0f / 2048.0f;     // raw -> g

// 추정/제어 상수 (flix 기본값 기반)
const float RATES_LPF_ALPHA   = 0.2f;         // 각속도 LPF, ~40Hz @1kHz
const float RATES_D_LPF_ALPHA = 0.2f;         // D항 LPF, ~40Hz @1kHz
const float RATE_I_WINDUP     = 0.3f;         // 적분항 한계 (정규화 토크)
const float ACC_WEIGHT        = 0.003f;       // 착지 시 가속도 보정 가중치
const float LEVEL_WEIGHT      = 0.0002f;      // 비행 중 수평 가정 보정 가중치
const float TORQUE_TO_US      = 1000.0f;      // 정규화 토크 1.0 = 1000µs
const float MAX_TARGET_ANGLE_RP = 30.0f * DEG_TO_RAD;  // UDP 자세 명령 제한
const float MAX_TARGET_RATE_RP  = 300.0f * DEG_TO_RAD; // outer-loop 출력 제한
const float MAX_TARGET_RATE_YAW = 180.0f * DEG_TO_RAD;
const float SAFETY_TILT_DEG   = 60.0f;
const int   THROTTLE_IDLE_US  = 1100;         // 이하면 착지/적분 리셋 취급

// IMU2는 x,z축이 IMU1 대비 반전 (하드웨어 측정 결과, docs/history 참조).
const float IMU2_SIGN_X = -1.0f;
const float IMU2_SIGN_Y =  1.0f;
const float IMU2_SIGN_Z = -1.0f;
static const float IMU1_SIGN[3] = { 1.0f, 1.0f, 1.0f };
static const float IMU2_SIGN[3] = { IMU2_SIGN_X, IMU2_SIGN_Y, IMU2_SIGN_Z };

// --- 안전/redundancy 임계값 (dual_imu_cascade_pwm과 동일) ---
const uint32_t RC_TIMEOUT_MS      = 500;
const uint32_t PID_WDT_TIMEOUT_MS = 500;
const int32_t  FROZEN_DELTA_RAW   = 1;
const uint32_t IMU_FROZEN_MS      = 300;
const float    GYRO_DISAGREE_DPS  = 15.0f;
const uint32_t IMU_DISAGREE_MS    = 150;
// 불일치 비교 전용 LPF (~13Hz @1kHz). 두 IMU는 보드 위 위치가 달라
// 진동에서 순간 차이가 15dps를 넘을 수 있으므로 저주파 성분만 비교한다.
const float    DISAGREE_LPF_ALPHA = 0.08f;

const int   BIAS_CALIB_SAMPLES   = 2000;
const float BIAS_MOVEMENT_THRESH = 1.0f;      // 축별 stddev 상한 (deg/s)
const int   BIAS_CALIB_RETRIES   = 3;
const float ACC_REST_NORM_TOL    = 0.3f;      // 정지 가속도 크기 허용 오차 (g)
const float ACC_REST_MIN_Z       = 0.5f;      // 정지 body accel z 최소값 (g)

// ==========================================================
// 3. 유틸
// ==========================================================
// Arduino sketch preprocessor의 자동 함수 원형은 첫 함수 정의 앞에 삽입되므로,
// 함수 시그니처에 쓰이는 타입은 모두 그보다 먼저 정의한다.
struct MotorMix {
  int motor[4];
  bool scaled;
};

// IMU별 freeze 감시 상태 (raw 레지스터 값이 멈췄는지)
struct FreezeMon {
  int16_t  lastGyro[3] = {0, 0, 0};
  int16_t  lastAccel[3] = {0, 0, 0};
  uint32_t since = 0;
  bool     init  = false;
};

// IMU1 sensor frame -> body FLU frame.
static inline Vector sensorToBody(const float v[3]) {
  return Vector(-v[1], v[0], v[2]);
}

static float wrapAngle(float angle) {
  angle = fmodf(angle, 2.0f * PI);
  if (angle > PI) angle -= 2.0f * PI;
  else if (angle < -PI) angle += 2.0f * PI;
  return angle;
}

static bool checkFreeze(FreezeMon &m, const inv_imu_sensor_event_t &e, uint32_t nowMs) {
  if (!m.init) {
    for (int k = 0; k < 3; k++) {
      m.lastGyro[k] = e.gyro[k];
      m.lastAccel[k] = e.accel[k];
    }
    m.init = true;
    return false;
  }

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
// 모듈러 뺄셈이라 그대로 안전하다. (dual_imu_cascade_pwm과 동일)
static inline bool rcTimedOut(uint32_t nowMs, uint32_t lastMs) {
  return (int32_t)(nowMs - lastMs) > (int32_t)RC_TIMEOUT_MS;
}

// ==========================================================
// 4. 시스템 변수
// ==========================================================
WiFiUDP   udp;
char      packetBuffer[256];
IPAddress laptopIP;
int       laptopPort            = 0;
bool      connectionEstablished = false;

// startGyro/startAccel은 ODR·FSR만 설정하고 UI 필터 대역폭은 칩 전원
// 기본값(bypass) 그대로 둔다. 그러면 프롭 진동이 필터 없이 들어오므로
// protected icm_driver에 접근해 하드웨어 LPF를 켠다 (dual_imu_cascade_pwm과 동일).
class ICM42670WithLPF : public ICM42670 {
public:
  ICM42670WithLPF(SPIClass &spi_bus, uint8_t cs) : ICM42670(spi_bus, cs) {}
  int setLowPassFilters() {
    int rc = 0;
    rc |= inv_imu_set_gyro_ln_bw(&icm_driver, GYRO_CONFIG1_GYRO_FILT_BW_121);
    rc |= inv_imu_set_accel_ln_bw(&icm_driver, ACCEL_CONFIG1_ACCEL_FILT_BW_25);
    return rc;
  }
};

ICM42670WithLPF IMU1(SPI, SPI_CS1);
ICM42670WithLPF IMU2(SPI, SPI_CS2);

volatile bool  safety_lock = true;
// RC 목표 (rad, body FLU 규약)
volatile float targetRollRad  = 0.0f;
volatile float targetPitchRad = 0.0f;
volatile float targetYawRad   = 0.0f;

// 추정 상태 — pid_task만 쓴다.
Quaternion attitude;
Vector rates; // rad/s, LPF 적용
LowPassFilter<Vector> ratesFilter(RATES_LPF_ALPHA);

// 각속도 PID — pid_task만 쓴다. 게인은 매 tick volatile에서 복사.
PID rollRatePID (0.05f, 0.2f, 0.001f, RATE_I_WINDUP, RATES_D_LPF_ALPHA);
PID pitchRatePID(0.05f, 0.2f, 0.001f, RATE_I_WINDUP, RATES_D_LPF_ALPHA);
PID yawRatePID  (0.3f,  0.0f, 0.0f,   RATE_I_WINDUP, RATES_D_LPF_ALPHA);

// 텔레메트리 스냅샷 (pid_task가 쓰고 udp_task가 읽음, deg / deg/s / g)
volatile float telRoll = 0.0f, telPitch = 0.0f, telYaw = 0.0f;
volatile float telGyroX = 0.0f, telGyroY = 0.0f, telGyroZ = 0.0f;
volatile float telAccX = 0.0f, telAccY = 0.0f, telAccZ = 0.0f;
volatile float telTiltDeg = 0.0f;
volatile float yawNowRad = 0.0f;   // yaw 명령의 setpoint 동기화용
// start 시 yaw를 0으로 리셋한다 (구형 펌웨어와 동일한 지상국 기대치).
// attitude는 pid_task 소유이므로 플래그로 요청한다.
volatile bool  requestYawReset = false;

float gyro_bias1[3] = {0, 0, 0};
float gyro_bias2[3] = {0, 0, 0};
float acc_rest_body[3] = {0, 0, 1}; // 캘리브레이션 시 정지 가속도 (g, body)

volatile uint32_t lastRcMs        = 0;
volatile bool     fault_rc        = false;
volatile bool     fault_imu1      = false;
volatile bool     fault_imu2      = false;
volatile bool     fault_disagree  = false;
volatile bool     fault_attitude  = false;
volatile int      active_imus     = 2;
volatile bool     mixer_scaled    = false;
volatile bool     calibration_ok  = false;

volatile bool     imu1_frozen_now  = false;
volatile bool     imu2_frozen_now  = false;
volatile bool     imu_disagree_now = false;

volatile uint32_t lastRcSeq     = 0;
volatile bool     rcSeqValid    = false;
volatile uint32_t rcTotalPkts   = 0;
volatile uint32_t rcDroppedPkts = 0;

// ==========================================================
// 5. 모터
// ==========================================================
void writeMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);
  uint32_t duty = ((uint32_t)us * ESC_MAXDUTY) / ESC_PERIOD;
  ledcWrite(pin, duty);
}
void stopMotors() {
  writeMotor(pinM1, 1000); writeMotor(pinM2, 1000);
  writeMotor(pinM3, 1000); writeMotor(pinM4, 1000);
}

// 자세 차동 명령을 먼저 보존하고 collective를 이동한다. 그래도 범위를 넘을
// 때만 모든 자세 명령을 같은 비율로 축소해 토크 비율을 유지한다.
static MotorMix desaturateMix(const float diffIn[4], int throttle,
                              int minMotor, int maxMotor) {
  MotorMix out;
  minMotor = constrain(minMotor, 1000, 2000);
  maxMotor = constrain(maxMotor, minMotor, 2000);

  float diff[4] = { diffIn[0], diffIn[1], diffIn[2], diffIn[3] };
  float minDiff = diff[0], maxDiff = diff[0];
  for (int i = 1; i < 4; i++) {
    minDiff = min(minDiff, diff[i]);
    maxDiff = max(maxDiff, diff[i]);
  }

  const float available = (float)(maxMotor - minMotor);
  const float span = maxDiff - minDiff;
  float scale = 1.0f;
  if (span > available && span > 0.0f) scale = available / span;
  out.scaled = scale < 0.9999f;

  if (out.scaled) {
    for (int i = 0; i < 4; i++) diff[i] *= scale;
    minDiff *= scale;
    maxDiff *= scale;
  }

  const float collectiveLo = minMotor - minDiff;
  const float collectiveHi = maxMotor - maxDiff;
  const float collective = min(max((float)throttle, collectiveLo), collectiveHi);

  for (int i = 0; i < 4; i++) {
    out.motor[i] = constrain((int)lroundf(collective + diff[i]), minMotor, maxMotor);
  }
  return out;
}

// ==========================================================
// 6. 캘리브레이션 (자이로 bias + 정지 가속도 검증)
// ==========================================================
// sign[]은 각 IMU를 IMU1 sensor frame으로 맞추는 부호.
static bool measure_imu_rest(ICM42670 &imu, const float sign[3],
                             float bias_out[3], float sd_out[3],
                             float acc_out[3]) {
  double sum[3] = {0, 0, 0}, sum_sq[3] = {0, 0, 0}, acc_sum[3] = {0, 0, 0};
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
    for (int k = 0; k < 3; k++) {
      float g = sign[k] * e.gyro[k] * GYRO_SCALE;
      sum[k] += g;
      sum_sq[k] += (double)g * g;
      acc_sum[k] += sign[k] * e.accel[k] * ACCEL_SCALE;
    }
    samples++;
    delayMicroseconds(1000);
  }
  if (samples != BIAS_CALIB_SAMPLES) return false;
  for (int k = 0; k < 3; k++) {
    double mean = sum[k] / samples;
    double var  = sum_sq[k] / samples - mean * mean;
    if (var < 0) var = 0;
    bias_out[k] = (float)mean;
    sd_out[k]   = (float)sqrt(var);
    acc_out[k]  = (float)(acc_sum[k] / samples);
  }
  return true;
}

static bool calibrate_rest() {
  float sd1[3], sd2[3], acc1[3], acc2[3];
  for (int a = 1; a <= BIAS_CALIB_RETRIES; a++) {
    Serial.printf("[CALIB] attempt %d/%d (hold still)...\n", a, BIAS_CALIB_RETRIES);
    bool read1 = measure_imu_rest(IMU1, IMU1_SIGN, gyro_bias1, sd1, acc1);
    bool read2 = measure_imu_rest(IMU2, IMU2_SIGN, gyro_bias2, sd2, acc2);
    if (!read1 || !read2) {
      Serial.printf("[CALIB] sensor read failed (imu1=%d imu2=%d)\n", (int)read1, (int)read2);
      continue;
    }
    float max_sd = max(max(max(sd1[0], sd1[1]), sd1[2]),
                       max(max(sd2[0], sd2[1]), sd2[2]));
    Serial.printf("[CALIB] IMU1 %.3f %.3f %.3f | IMU2 %.3f %.3f %.3f (maxSD %.3f)\n",
                  gyro_bias1[0], gyro_bias1[1], gyro_bias1[2],
                  gyro_bias2[0], gyro_bias2[1], gyro_bias2[2], max_sd);
    if (max_sd > BIAS_MOVEMENT_THRESH) {
      Serial.println("[CALIB] movement detected, retry");
      continue;
    }

    // 정지 가속도를 body frame으로 옮겨 크기와 z부호를 검증한다.
    // 실패하면 좌표계 매핑이 하드웨어와 어긋난 것이므로 시동을 막는다.
    float accSensor[3];
    for (int k = 0; k < 3; k++) accSensor[k] = (acc1[k] + acc2[k]) * 0.5f;
    Vector accBody = sensorToBody(accSensor);
    Serial.printf("[CALIB] rest accel body %.3f %.3f %.3f (norm %.3f)\n",
                  accBody.x, accBody.y, accBody.z, accBody.norm());
    if (fabsf(accBody.norm() - 1.0f) > ACC_REST_NORM_TOL) {
      Serial.println("[CALIB] FAIL: rest accel norm out of range (check ACCEL_SCALE/mounting)");
      return false;
    }
    if (accBody.z < ACC_REST_MIN_Z) {
      Serial.println("[CALIB] FAIL: rest accel z is not +1g — sensorToBody() axis map "
                     "does not match the hardware; fix it before flying");
      return false;
    }
    acc_rest_body[0] = accBody.x;
    acc_rest_body[1] = accBody.y;
    acc_rest_body[2] = accBody.z;
    Serial.println("[CALIB] OK");
    return true;
  }
  Serial.println("[CALIB] FAIL: reboot and calibrate on a stationary surface");
  return false;
}

// ==========================================================
// 7. 자세 추정 (flix estimate.ino 이식)
// ==========================================================
static void applyGyro(const Vector &gyroBody, float dt) {
  rates = ratesFilter.update(gyroBody);
  attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

static void applyAcc(const Vector &accBody) {
  // 착지 상태에서만 중력 방향으로 자세를 당긴다.
  bool landed = (safety_lock || base_throttle <= THROTTLE_IDLE_US)
             && fabsf(accBody.norm() - 1.0f) < 0.1f;
  if (!landed) return;
  Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
  Vector correction = Vector::rotationVectorBetween(accBody, up) * ACC_WEIGHT;
  attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

static void applyLevel() {
  // 비행 중에는 조종사가 기체를 대체로 수평으로 유지한다고 가정하고
  // 아주 약하게 수평으로 당겨 자이로 드리프트를 막는다.
  if (safety_lock || base_throttle <= THROTTLE_IDLE_US) return;
  Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
  Vector correction = Vector::rotationVectorBetween(Vector(0, 0, 1), up) * LEVEL_WEIGHT;
  attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

// ==========================================================
// 8. 제어 (flix control.ino 이식)
// ==========================================================
static Vector controlAttitude(const Vector &euler) {
  float yawSetpoint = yaw_enabled ? targetYawRad : euler.z;
  Quaternion attitudeTarget =
      Quaternion::fromEuler(Vector(targetRollRad, targetPitchRad, yawSetpoint));

  const Vector up(0, 0, 1);
  Vector upActual = Quaternion::rotateVector(up, attitude);
  Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);
  Vector error = Vector::rotationVectorBetween(upTarget, upActual);

  Vector target;
  target.x = constrain(Kp_Angle_Roll * error.x,
                       -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
  target.y = constrain(Kp_Angle_Pitch * error.y,
                       -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
  target.z = yaw_enabled
           ? constrain(Kp_Angle_Yaw * wrapAngle(targetYawRad - euler.z),
                       -MAX_TARGET_RATE_YAW, MAX_TARGET_RATE_YAW)
           : 0.0f;
  return target;
}

// ==========================================================
// 9. PID 태스크 (Core 1, 1kHz)
// ==========================================================
void pid_task(void *pv) {
  // 모터 정지 수단이 모두 이 태스크 안에 있으므로, 태스크가 블로킹되면
  // 워치독이 panic 재부팅을 강제한다 (dual_imu_cascade_pwm과 동일 설계).
  esp_task_wdt_add(NULL);

  const TickType_t period = pdMS_TO_TICKS(1);
  TickType_t wake = xTaskGetTickCount();
  const float dtNominal = 0.001f;

  FreezeMon fm1, fm2;
  uint32_t disagreeSince = 0;
  LowPassFilter<Vector> disagreeLpf1(DISAGREE_LPF_ALPHA);
  LowPassFilter<Vector> disagreeLpf2(DISAGREE_LPF_ALPHA);
  uint32_t lastMicros = micros();
  bool wasLocked = true;

  inv_imu_sensor_event_t e1 = {}, e2 = {};

  while (true) {
    vTaskDelayUntil(&wake, period);
    esp_task_wdt_reset();
    TickType_t afterWake = xTaskGetTickCount();
    if ((TickType_t)(afterWake - wake) > period) {
      // 큰 지연 뒤 밀린 tick을 연속 실행하지 않는다.
      wake = afterWake;
    }
    uint32_t nowMs = millis();
    uint32_t nowUs = micros();
    float dt = (nowUs - lastMicros) / 1e6f;
    lastMicros = nowUs;
    if (dt < 0.0002f || dt > 0.01f) dt = dtNominal;

    // ---------- 센서 읽기 ----------
    int readStatus1 = IMU1.getDataFromRegisters(e1);
    int readStatus2 = IMU2.getDataFromRegisters(e2);

    bool frozen1 = checkFreeze(fm1, e1, nowMs);
    bool frozen2 = checkFreeze(fm2, e2, nowMs);
    imu1_frozen_now = frozen1;
    imu2_frozen_now = frozen2;
    if (frozen1 && !fault_imu1) Serial.printf("[FAULT] IMU1 FROZEN (read=%d)\n", readStatus1);
    if (frozen2 && !fault_imu2) Serial.printf("[FAULT] IMU2 FROZEN (read=%d)\n", readStatus2);
    if (frozen1) fault_imu1 = true;
    if (frozen2) fault_imu2 = true;

    // IMU1 sensor frame 기준 (IMU2 축 정렬 + 개별 bias 보정, deg/s와 raw)
    float g1[3] = {
      e1.gyro[0] * GYRO_SCALE - gyro_bias1[0],
      e1.gyro[1] * GYRO_SCALE - gyro_bias1[1],
      e1.gyro[2] * GYRO_SCALE - gyro_bias1[2] };
    float g2[3] = {
      IMU2_SIGN_X * e2.gyro[0] * GYRO_SCALE - gyro_bias2[0],
      IMU2_SIGN_Y * e2.gyro[1] * GYRO_SCALE - gyro_bias2[1],
      IMU2_SIGN_Z * e2.gyro[2] * GYRO_SCALE - gyro_bias2[2] };
    float a1[3] = { (float)e1.accel[0], (float)e1.accel[1], (float)e1.accel[2] };
    float a2[3] = { IMU2_SIGN_X * e2.accel[0], IMU2_SIGN_Y * e2.accel[1],
                    IMU2_SIGN_Z * e2.accel[2] };

    // 진동으로 인한 비행 중 disagree 컷을 막기 위해 저역 성분만 비교한다.
    Vector g1f = disagreeLpf1.update(Vector(g1[0], g1[1], g1[2]));
    Vector g2f = disagreeLpf2.update(Vector(g2[0], g2[1], g2[2]));
    bool disagreeSample = false;
    if (!frozen1 && !frozen2) {
      if (fabsf(g1f.x - g2f.x) > GYRO_DISAGREE_DPS ||
          fabsf(g1f.y - g2f.y) > GYRO_DISAGREE_DPS ||
          fabsf(g1f.z - g2f.z) > GYRO_DISAGREE_DPS) disagreeSample = true;
    }
    imu_disagree_now = disagreeSample;

    // latch된 센서는 비행 중 값이 다시 움직여도 자동 재투입하지 않는다.
    bool h1 = !fault_imu1, h2 = !fault_imu2;

    // ---------- IMU 융합 ----------
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
      for (int k = 0; k < 3; k++) { fg[k] = (g1[k] + g2[k]) * 0.5f; fa[k] = (a1[k] + a2[k]) * 0.5f; }
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
      safety_lock = true;
      mixer_scaled = false;
      rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
      wasLocked = true;
      stopMotors();
      continue;
    }

    // ---------- body frame 변환 + 자세 추정 ----------
    Vector gyroBody = sensorToBody(fg) * DEG_TO_RAD;   // rad/s
    Vector accBody  = sensorToBody(fa) * ACCEL_SCALE;  // g

    applyGyro(gyroBody, dt);
    applyAcc(accBody);
    applyLevel();

    if (requestYawReset) {
      attitude.setYaw(0);
      requestYawReset = false;
    }

    Vector euler = attitude.toEuler();
    Vector upBody = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
    float tiltDeg = degrees(acosf(constrain(upBody.z, -1.0f, 1.0f)));

    telRoll  = degrees(euler.x);
    telPitch = degrees(euler.y);
    telYaw   = degrees(euler.z);
    yawNowRad = euler.z;
    telTiltDeg = tiltDeg;
    telGyroX = degrees(rates.x); telGyroY = degrees(rates.y); telGyroZ = degrees(rates.z);
    telAccX = accBody.x; telAccY = accBody.y; telAccZ = accBody.z;

    // ---------- 안전 검사 ----------
    if (tiltDeg > SAFETY_TILT_DEG) {
      if (!fault_attitude) Serial.printf("[FAULT] OVER-TILT %.1f deg\n", tiltDeg);
      fault_attitude = true;
      safety_lock = true;
    }
    if (!safety_lock && rcTimedOut(nowMs, lastRcMs)) {
      fault_rc = true; safety_lock = true;
      Serial.println("[FAULT] RC TIMEOUT");
    }
    if (safety_lock) {
      mixer_scaled = false;
      rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
      wasLocked = true;
      stopMotors();
      continue;
    }

    // 잠금 해제 첫 tick의 D kick과 적분 잔류를 제거한다.
    if (wasLocked) {
      rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
      wasLocked = false;
    }

    // ---------- 캐스케이드 제어 ----------
    rollRatePID.p  = Kp_Rate_Roll;  rollRatePID.i  = Ki_Rate_Roll;  rollRatePID.d  = Kd_Rate_Roll;
    pitchRatePID.p = Kp_Rate_Pitch; pitchRatePID.i = Ki_Rate_Pitch; pitchRatePID.d = Kd_Rate_Pitch;
    yawRatePID.p   = Kp_Rate_Yaw;   yawRatePID.i   = Ki_Rate_Yaw;   yawRatePID.d   = Kd_Rate_Yaw;

    Vector ratesTarget = controlAttitude(euler);

    // yaw 비활성 시에도 rate 댐핑(목표 0)은 유지한다.
    float tx = rollRatePID.update (ratesTarget.x - rates.x, rates.x, dt);
    float ty = pitchRatePID.update(ratesTarget.y - rates.y, rates.y, dt);
    float tz = yawRatePID.update  (ratesTarget.z - rates.z, rates.z, dt);

    // ---------- 믹서 (flix 부호) + desaturation ----------
    float diff[4] = {
      ( tx - ty + tz) * TORQUE_TO_US,   // M1 FL (CW)
      (-tx + ty + tz) * TORQUE_TO_US,   // M2 RR (CW)
      (-tx - ty - tz) * TORQUE_TO_US,   // M3 FR (CCW)
      ( tx + ty - tz) * TORQUE_TO_US,   // M4 RL (CCW)
    };
    const int throttle = base_throttle;
    MotorMix mix = desaturateMix(diff, throttle, min_throttle, max_throttle);
    mixer_scaled = mix.scaled;

    // 조건부 적분: 자세 명령이 잘리지 않았고 스로틀이 비행 영역일 때만.
    if (throttle > THROTTLE_IDLE_US && !mix.scaled) {
      rollRatePID.integrate(dt);
      pitchRatePID.integrate(dt);
      if (yaw_enabled) yawRatePID.integrate(dt);
      else yawRatePID.integralTerm = 0;
    } else if (throttle <= THROTTLE_IDLE_US) {
      rollRatePID.integralTerm = 0;
      pitchRatePID.integralTerm = 0;
      yawRatePID.integralTerm = 0;
    } else if (!yaw_enabled) {
      yawRatePID.integralTerm = 0;
    }

    writeMotor(pinM1, mix.motor[0]);
    writeMotor(pinM2, mix.motor[1]);
    writeMotor(pinM3, mix.motor[2]);
    writeMotor(pinM4, mix.motor[3]);
  }
}

// ==========================================================
// 10. UDP 태스크 (Core 0) — char* 파싱 (String heap 회피)
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

// rc 명령의 각도는 deg 단위, body FLU 규약 (roll+ 우측 하강, pitch+ 기수 하강,
// yaw+ 위에서 CCW).
static bool setRcTargets(float x, float y, float z, bool hasYaw) {
  if (!isfinite(x) || !isfinite(y) || (hasYaw && !isfinite(z))) return false;
  targetRollRad  = constrain(radians(x), -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  targetPitchRad = constrain(radians(y), -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  if (hasYaw) targetYawRad = wrapAngle(radians(z));
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

static void handleGainCommand(const char *buf) {
  if (strlen(buf) < 3) return;
  float value;
  if (!parseFloatStrict(buf + 2, value) || value < 0.0f || value > 100.0f) return;

  // 게인 단위: rate PID는 토크/(rad/s), angle P는 (rad/s)/rad.
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

// 필드 구성은 docs/udp_protocol.md의 22필드 스키마와 동일하다.
// 마지막 Armed 필드로 지상국이 START 거부/자동 disarm을 감지한다.
static void sendTelemetry() {
  if (!connectionEstablished) return;
  bool criticalFault = (active_imus == 0) || fault_attitude || !calibration_ok;
  udp.beginPacket(laptopIP, laptopPort);
  udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%d,%d,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d",
             telRoll, telPitch, telYaw,
             telGyroX, telGyroY, telGyroZ,
             telAccX, telAccY, telAccZ,
             base_throttle,
             (int)fault_rc, (int)criticalFault,
             (unsigned long)rcTotalPkts, (unsigned long)rcDroppedPkts,
             (int)fault_imu1, (int)fault_imu2, (int)fault_disagree,
             active_imus, (int)mixer_scaled, (int)fault_attitude,
             (int)calibration_ok, (int)!safety_lock);
  udp.endPacket();
}

void udp_task(void *pv) {
  const int CTRL_MARGIN = 150;
  uint32_t lastSend = 0;
  while (true) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      laptopIP = udp.remoteIP();
      laptopPort = udp.remotePort();
      connectionEstablished = true;

      int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
      if (len > 0) {
        packetBuffer[len] = '\0';
        char *buf = trimCommand(packetBuffer);

        if (strncmp(buf, "rc", 2) == 0 && (buf[2] == ' ' || buf[2] == '\t')) {
          handleRcCommand(buf);
        }
        else if (strcmp(buf, "start") == 0) {
          bool overTilt = telTiltDeg > SAFETY_TILT_DEG;
          bool noUsableImu = imu1_frozen_now && imu2_frozen_now;
          if (!safety_lock) {
            // 이미 시동 상태. 지상국은 start를 여러 번 재전송하므로, 지연
            // 도착한 중복 start가 비행 중 fault latch를 지우고 스로틀 창을
            // 리셋하는 것을 막는다.
            Serial.println(">>> START ignored (already armed)");
          }
          else if (!calibration_ok || overTilt || noUsableImu || imu_disagree_now) {
            safety_lock = true;
            Serial.printf(">>> START REFUSED calib=%d tilt=%d imu=%d disagree=%d\n",
                          (int)calibration_ok, (int)overTilt,
                          (int)noUsableImu, (int)imu_disagree_now);
          } else {
            safety_lock = true; // 상태를 모두 초기화한 뒤 마지막에 해제
            fault_rc = false;
            fault_imu1 = imu1_frozen_now;
            fault_imu2 = imu2_frozen_now;
            fault_disagree = false;
            fault_attitude = false;
            lastRcSeq = 0;
            rcSeqValid = false;
            lastRcMs = millis();
            base_throttle = 1100; min_throttle = 1050; max_throttle = 1250;
            targetRollRad = 0.0f;
            targetPitchRad = 0.0f;
            targetYawRad = 0.0f;
            requestYawReset = true; // yaw 기준을 0으로 (rc yaw 목표와 정렬)
            safety_lock = false;
            Serial.println(">>> START");
          }
        }
        else if (strcmp(buf, "stop") == 0) {
          safety_lock = true;
          base_throttle = 1000;
          Serial.println(">>> STOP");
        }
        else if (strncmp(buf, "th", 2) == 0) {
          long parsed;
          if (parseIntStrict(buf + 2, parsed)) {
            int nb = constrain((int)parsed, 1000, 1900);
            base_throttle = nb;
            min_throttle = max(1050, nb - CTRL_MARGIN);
            max_throttle = min(1900, nb + CTRL_MARGIN);
          }
        }
        else if (strncmp(buf, "yaw", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            targetYawRad = yawNowRad; // 활성화 순간 setpoint jump 방지
            yaw_enabled = (enabled == 1);
            Serial.printf(">>> Yaw %s\n", yaw_enabled ? "ON" : "OFF");
          }
        }
        else {
          handleGainCommand(buf);
        }
      }
    }

    uint32_t now = millis();
    if (now - lastSend >= 50) {
      lastSend = now;
      sendTelemetry();
    }
    vTaskDelay(1);
  }
}

// ==========================================================
// 11. setup / loop
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(SPI_CS1, OUTPUT); pinMode(SPI_CS2, OUTPUT);
  digitalWrite(SPI_CS1, HIGH); digitalWrite(SPI_CS2, HIGH);  // float CS 버스 경합 방지
  delay(100);

  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  udp.begin(UDP_PORT);

  SPI.begin(12, 13, 11, -1);  // SCK, MISO, MOSI; CS는 각 IMU 객체가 관리

  bool esc_ok = ledcAttach(pinM1, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM2, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM3, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM4, ESC_FREQ, ESC_RES);
  if (!esc_ok) while (1) { Serial.println("[FAULT] ESC attach FAIL"); delay(1000); }
  stopMotors();

  if (IMU1.begin() < 0) while (1) { Serial.println("[FAULT] IMU1 FAIL"); delay(1000); }
  if (IMU2.begin() < 0) while (1) { Serial.println("[FAULT] IMU2 FAIL"); delay(1000); }
  int sensorStartStatus = 0;
  sensorStartStatus |= IMU1.startAccel(1600, 16);
  sensorStartStatus |= IMU1.startGyro(1600, 2000);
  sensorStartStatus |= IMU2.startAccel(1600, 16);
  sensorStartStatus |= IMU2.startGyro(1600, 2000);
  sensorStartStatus |= IMU1.setLowPassFilters();
  sensorStartStatus |= IMU2.setLowPassFilters();
  if (sensorStartStatus != 0) {
    while (1) { Serial.printf("[FAULT] IMU START FAIL (%d)\n", sensorStartStatus); delay(1000); }
  }
  delay(500);

  calibration_ok = calibrate_rest();

  // 초기 자세: 정지 가속도로 즉시 수렴시킨다 (applyAcc의 고정점과 동일식).
  if (calibration_ok) {
    Vector accRest(acc_rest_body[0], acc_rest_body[1], acc_rest_body[2]);
    attitude = Quaternion::fromRotationVector(
        Vector::rotationVectorBetween(accRest, Vector(0, 0, 1)));
  }

  // pid_task 전용 태스크 워치독. timeout 시 panic 재부팅으로 마지막 PWM
  // 고정 상태에서 벗어난다.
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = PID_WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK &&
      esp_task_wdt_init(&wdt_config) != ESP_OK) {
    while (1) { Serial.println("[FAULT] TASK WDT INIT FAIL"); delay(1000); }
  }

  BaseType_t pidTaskOk = xTaskCreatePinnedToCore(pid_task, "PID", 8192, NULL, 2, NULL, 1);
  BaseType_t udpTaskOk = xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 1, NULL, 0);
  if (pidTaskOk != pdPASS || udpTaskOk != pdPASS) {
    safety_lock = true;
    while (1) { Serial.println("[FAULT] TASK CREATE FAIL"); delay(1000); }
  }

  Serial.println("DUAL_IMU_FLIX_QUAT READY");
}

void loop() {
  // UDP 객체는 udp_task 한 곳에서만 접근해 cross-core race를 피한다.
  delay(1000);
}
