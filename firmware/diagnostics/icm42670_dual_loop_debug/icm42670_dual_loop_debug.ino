// ==========================================================
// DUAL_IMU_PID_DEBUG — DIAGNOSTIC ONLY, NO FLIGHT CONTROL
// ----------------------------------------------------------
// Purpose: collect evidence about loop rate, dt clamp behavior,
// and adaptive-alpha usage during the dual-IMU + complementary
// filter pipeline used by archive/legacy_flight/dual_imu_pid_pwm.
//
// Safety: ESCs are held at 1000us (idle) for the whole run.
// No PID, no motor mixing, no start/stop command.
// Pair with scripts/archive/receive_dual_imu_debug.py
// ==========================================================

#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ----- WiFi / UDP -----
const char* WIFI_SSID    = "DUAL_IMU_DEBUG";
const char* WIFI_PASS    = "12345678";
const int   UDP_PORT     = 4210;
const int   WIFI_CHANNEL = 6;

// ----- Pins (same as flight firmware) -----
const int pinM1   = 4;
const int pinM2   = 5;
const int pinM3   = 6;
const int pinM4   = 7;
const int SPI_CS1 = 10;
const int SPI_CS2 = 9;

const int ESC_FREQ   = 400;
const int ESC_RES    = 14;
const int ESC_PERIOD = 2500;

// ----- IMU scales -----
const float GYRO_SCALE   = 1.0f / 16.4f;
const float ACCEL_SCALE  = 1.0f / 2048.0f;

const float IMU2_SIGN_X = -1.0f;
const float IMU2_SIGN_Y =  1.0f;
const float IMU2_SIGN_Z = -1.0f;

const int      BIAS_CALIB_SAMPLES         = 2000;
const float    BIAS_CALIB_MOVEMENT_THRESH = 1.0f;
const int      BIAS_CALIB_RETRIES         = 3;

// ----- Filter params (identical to flight firmware) -----
const float LPF_ALPHA_ACC  = 1.00f;
const float LPF_ALPHA_GYRO = 1.0f;

const float ACCEL_DEV_SOFT = 0.1f;
const float ACCEL_DEV_HARD = 0.3f;
const float ALPHA_STATIC   = 0.99f;
const float ALPHA_NORMAL   = 0.995f;
const float ALPHA_DYNAMIC  = 0.999f;

const float YAW_DEADZONE = 0.3f;

// ----- Globals -----
WiFiUDP   udp;
IPAddress laptopIP;
int       laptopPort            = 0;
bool      connectionEstablished = false;

ICM42670 IMU1(SPI, SPI_CS1);
ICM42670 IMU2(SPI, SPI_CS2);

float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f;
volatile float raw_gx = 0.0f, raw_gy = 0.0f, raw_gz = 0.0f;
volatile float raw_ax = 0.0f, raw_ay = 0.0f, raw_az = 0.0f;
float lpf_ax = 0.0f, lpf_ay = 0.0f, lpf_az = 0.0f;
float lpf_gx = 0.0f, lpf_gy = 0.0f, lpf_gz = 0.0f;

float gyro_bias1[3] = {0.0f, 0.0f, 0.0f};
float gyro_bias2[3] = {0.0f, 0.0f, 0.0f};

// Diagnostic accumulators reset every 50ms telemetry tick.
// 32-bit reads/writes on ESP32-S3 are atomic so we use plain volatiles.
volatile uint32_t diag_loop_count  = 0;
volatile uint32_t diag_max_dt_us   = 0;
volatile uint32_t diag_sum_dt_us   = 0;
volatile uint32_t diag_clamp_count = 0;
volatile float    diag_last_alpha  = 0.0f;

// ==========================================================
// Motor pins held at idle for safety
// ==========================================================
void writeMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);
  uint32_t duty = ((uint32_t)us * 16383UL) / ESC_PERIOD;
  ledcWrite(pin, duty);
}

void idleMotors() {
  writeMotor(pinM1, 1000);
  writeMotor(pinM2, 1000);
  writeMotor(pinM3, 1000);
  writeMotor(pinM4, 1000);
}

// ==========================================================
// Adaptive complementary alpha (matches flight firmware)
// ==========================================================
static inline float compute_alpha(float ax, float ay, float az) {
  float mag = sqrtf(ax*ax + ay*ay + az*az);
  float dev = fabsf(mag - 1.0f);
  if      (dev < ACCEL_DEV_SOFT) return ALPHA_STATIC;
  else if (dev < ACCEL_DEV_HARD) return ALPHA_NORMAL;
  else                            return ALPHA_DYNAMIC;
}

// ==========================================================
// Startup bias calibration (matches flight firmware)
// ==========================================================
static void measure_imu_bias(ICM42670 &imu, const float sign[3],
                             float bias_out[3], float stddev_out[3]) {
  double sum[3]    = {0.0, 0.0, 0.0};
  double sum_sq[3] = {0.0, 0.0, 0.0};
  inv_imu_sensor_event_t e;
  for (int i = 0; i < BIAS_CALIB_SAMPLES; i++) {
    imu.getDataFromRegisters(e);
    float g[3] = {
      sign[0] * e.gyro[0] * GYRO_SCALE,
      sign[1] * e.gyro[1] * GYRO_SCALE,
      sign[2] * e.gyro[2] * GYRO_SCALE
    };
    for (int k = 0; k < 3; k++) {
      sum[k]    += g[k];
      sum_sq[k] += (double)g[k] * g[k];
    }
    delayMicroseconds(1000);
  }
  for (int k = 0; k < 3; k++) {
    double mean = sum[k] / BIAS_CALIB_SAMPLES;
    double var  = (sum_sq[k] / BIAS_CALIB_SAMPLES) - (mean * mean);
    if (var < 0) var = 0;
    bias_out[k]   = (float)mean;
    stddev_out[k] = (float)sqrt(var);
  }
}

static const float IMU1_SIGN[3] = { 1.0f, 1.0f, 1.0f };
static const float IMU2_SIGN[3] = { IMU2_SIGN_X, IMU2_SIGN_Y, IMU2_SIGN_Z };

static void calibrate_bias() {
  float sd1[3], sd2[3];
  for (int attempt = 1; attempt <= BIAS_CALIB_RETRIES; attempt++) {
    Serial.printf("[CALIB] attempt %d/%d (hold still)...\n", attempt, BIAS_CALIB_RETRIES);
    measure_imu_bias(IMU1, IMU1_SIGN, gyro_bias1, sd1);
    measure_imu_bias(IMU2, IMU2_SIGN, gyro_bias2, sd2);
    float max_sd = max(max(max(sd1[0], sd1[1]), sd1[2]),
                       max(max(sd2[0], sd2[1]), sd2[2]));
    Serial.printf("[CALIB] IMU1 bias x=%.3f y=%.3f z=%.3f (sd %.3f %.3f %.3f)\n",
                  gyro_bias1[0], gyro_bias1[1], gyro_bias1[2], sd1[0], sd1[1], sd1[2]);
    Serial.printf("[CALIB] IMU2 bias x=%.3f y=%.3f z=%.3f (sd %.3f %.3f %.3f) [drone frame]\n",
                  gyro_bias2[0], gyro_bias2[1], gyro_bias2[2], sd2[0], sd2[1], sd2[2]);
    if (max_sd <= BIAS_CALIB_MOVEMENT_THRESH) {
      Serial.println("[CALIB] OK");
      return;
    }
    Serial.printf("[CALIB] movement detected (sd %.3f > %.3f), retrying\n",
                  max_sd, BIAS_CALIB_MOVEMENT_THRESH);
  }
  Serial.println("[CALIB] WARN: all retries failed, using last measurement");
}

// ==========================================================
// 1kHz attitude task (Core 1) — same data flow as flight code
// minus PID / motor mixing, plus diagnostic counters.
// ==========================================================
void attitude_task(void *pvParameters) {
  const unsigned long LOOP_INTERVAL = 1000;
  unsigned long nextLoopTime = micros();
  unsigned long lastTime     = micros();

  inv_imu_sensor_event_t e1, e2;

  IMU1.getDataFromRegisters(e1);
  IMU2.getDataFromRegisters(e2);
  float gx1 =                  e1.gyro[0]  * GYRO_SCALE - gyro_bias1[0];
  float gy1 =                  e1.gyro[1]  * GYRO_SCALE - gyro_bias1[1];
  float gz1 =                  e1.gyro[2]  * GYRO_SCALE - gyro_bias1[2];
  float gx2 = IMU2_SIGN_X    * e2.gyro[0]  * GYRO_SCALE - gyro_bias2[0];
  float gy2 = IMU2_SIGN_Y    * e2.gyro[1]  * GYRO_SCALE - gyro_bias2[1];
  float gz2 = IMU2_SIGN_Z    * e2.gyro[2]  * GYRO_SCALE - gyro_bias2[2];
  lpf_ax =  ((e1.accel[0] + IMU2_SIGN_X * e2.accel[0]) * 0.5f) * ACCEL_SCALE;
  lpf_ay = -((e1.accel[1] + IMU2_SIGN_Y * e2.accel[1]) * 0.5f) * ACCEL_SCALE;
  lpf_az =  ((e1.accel[2] + IMU2_SIGN_Z * e2.accel[2]) * 0.5f) * ACCEL_SCALE;
  lpf_gx =  (gx1 + gx2) * 0.5f;
  lpf_gy = -(gy1 + gy2) * 0.5f;
  lpf_gz =  (gz1 + gz2) * 0.5f;

  while (true) {
    unsigned long now = micros();
    if (now < nextLoopTime) { vTaskDelay(0); continue; }
    nextLoopTime = now + LOOP_INTERVAL;

    IMU1.getDataFromRegisters(e1);
    IMU2.getDataFromRegisters(e2);

    float gx1 =                  e1.gyro[0]  * GYRO_SCALE - gyro_bias1[0];
    float gy1 =                  e1.gyro[1]  * GYRO_SCALE - gyro_bias1[1];
    float gz1 =                  e1.gyro[2]  * GYRO_SCALE - gyro_bias1[2];
    float gx2 = IMU2_SIGN_X    * e2.gyro[0]  * GYRO_SCALE - gyro_bias2[0];
    float gy2 = IMU2_SIGN_Y    * e2.gyro[1]  * GYRO_SCALE - gyro_bias2[1];
    float gz2 = IMU2_SIGN_Z    * e2.gyro[2]  * GYRO_SCALE - gyro_bias2[2];

    raw_ax =  ((e1.accel[0] + IMU2_SIGN_X * e2.accel[0]) * 0.5f) * ACCEL_SCALE;
    raw_ay = -((e1.accel[1] + IMU2_SIGN_Y * e2.accel[1]) * 0.5f) * ACCEL_SCALE;
    raw_az =  ((e1.accel[2] + IMU2_SIGN_Z * e2.accel[2]) * 0.5f) * ACCEL_SCALE;
    raw_gx =  (gx1 + gx2) * 0.5f;
    raw_gy = -(gy1 + gy2) * 0.5f;
    raw_gz =  (gz1 + gz2) * 0.5f;

    lpf_ax = LPF_ALPHA_ACC  * raw_ax + (1.0f - LPF_ALPHA_ACC)  * lpf_ax;
    lpf_ay = LPF_ALPHA_ACC  * raw_ay + (1.0f - LPF_ALPHA_ACC)  * lpf_ay;
    lpf_az = LPF_ALPHA_ACC  * raw_az + (1.0f - LPF_ALPHA_ACC)  * lpf_az;
    lpf_gx = LPF_ALPHA_GYRO * raw_gx + (1.0f - LPF_ALPHA_GYRO) * lpf_gx;
    lpf_gy = LPF_ALPHA_GYRO * raw_gy + (1.0f - LPF_ALPHA_GYRO) * lpf_gy;
    lpf_gz = LPF_ALPHA_GYRO * raw_gz + (1.0f - LPF_ALPHA_GYRO) * lpf_gz;

    // Record real (un-clamped) dt for diagnosis, then apply same clamp
    // as flight code so integration math is identical.
    uint32_t real_dt_us = (uint32_t)(now - lastTime);
    diag_loop_count++;
    diag_sum_dt_us += real_dt_us;
    if (real_dt_us > diag_max_dt_us) diag_max_dt_us = real_dt_us;
    if (real_dt_us > 2000)           diag_clamp_count++;

    float dt = real_dt_us / 1000000.0f;
    if (dt > 0.002f) dt = 0.001f;
    lastTime = now;

    // Axis remap: sensor X <-> drone Y (pitch), sensor Y <-> drone X (roll), sensor Z negated for yaw.
    // - angleX (roll, drone X)  uses sensor Y gyro (lpf_gy) and sensor X accel (-lpf_ax)
    // - angleY (pitch, drone Y) uses sensor X gyro (lpf_gx) and sensor Y accel ( lpf_ay)
    // - angleZ (yaw)            integrates -lpf_gz so CW from above is positive
    float accAngleX = atan2f(-lpf_ax, sqrtf(lpf_ay*lpf_ay + lpf_az*lpf_az)) * 180.0f / PI;
    float accAngleY = atan2f( lpf_ay, sqrtf(lpf_ax*lpf_ax + lpf_az*lpf_az)) * 180.0f / PI;
    float alpha = compute_alpha(lpf_ax, lpf_ay, lpf_az);
    diag_last_alpha = alpha;

    angleX = alpha * (angleX + lpf_gy * dt) + (1.0f - alpha) * accAngleX;
    angleY = alpha * (angleY + lpf_gx * dt) + (1.0f - alpha) * accAngleY;
    if (fabsf(lpf_gz) > YAW_DEADZONE) angleZ += -lpf_gz * dt;
  }
}

// ==========================================================
// setup / loop
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(SPI_CS1, OUTPUT);
  pinMode(SPI_CS2, OUTPUT);
  digitalWrite(SPI_CS1, HIGH);
  digitalWrite(SPI_CS2, HIGH);
  delay(100);

  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  udp.begin(UDP_PORT);

  SPI.begin(12, 13, 11, SPI_CS1);

  bool esc_ok = ledcAttach(pinM1, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM2, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM3, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM4, ESC_FREQ, ESC_RES);
  if (!esc_ok) {
    while (1) { Serial.println("[FAULT] ESC pin attach FAILED"); delay(1000); }
  }
  idleMotors();

  if (IMU1.begin() < 0) {
    while (1) { Serial.println("[FAULT] IMU1 INIT FAILED"); delay(1000); }
  }
  if (IMU2.begin() < 0) {
    while (1) { Serial.println("[FAULT] IMU2 INIT FAILED"); delay(1000); }
  }
  IMU1.startAccel(1600, 16);
  IMU1.startGyro(1600, 2000);
  IMU2.startAccel(1600, 16);
  IMU2.startGyro(1600, 2000);
  delay(500);

  calibrate_bias();

  xTaskCreatePinnedToCore(attitude_task, "ATT", 4096, NULL, 1, NULL, 1);

  Serial.println("DUAL_IMU_PID_DEBUG READY (motors idle, diagnostics streaming)");
}

void loop() {
  // Passively learn laptop IP from any incoming UDP packet
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    laptopIP   = udp.remoteIP();
    laptopPort = udp.remotePort();
    connectionEstablished = true;
    char buf[64];
    udp.read(buf, sizeof(buf) - 1);
  }

  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime < 50) { delay(1); return; }
  lastSendTime = millis();

  // Re-affirm motors are idle every tick (belt + suspenders)
  idleMotors();

  if (!connectionEstablished) return;

  // Snapshot + reset diagnostic accumulators
  uint32_t loops    = diag_loop_count;  diag_loop_count  = 0;
  uint32_t max_dt   = diag_max_dt_us;   diag_max_dt_us   = 0;
  uint32_t sum_dt   = diag_sum_dt_us;   diag_sum_dt_us   = 0;
  uint32_t clamps   = diag_clamp_count; diag_clamp_count = 0;
  float    alpha    = diag_last_alpha;

  float avg_dt_us = loops > 0 ? (float)sum_dt / loops : 0.0f;

  udp.beginPacket(laptopIP, laptopPort);
  udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu,%lu,%.1f,%lu,%.4f",
             angleX, angleY, angleZ,
             raw_gx, raw_gy, raw_gz,
             raw_ax, raw_ay, raw_az,
             loops, max_dt, avg_dt_us, clamps, alpha);
  udp.endPacket();
}
