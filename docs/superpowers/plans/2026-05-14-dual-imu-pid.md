# PWM_TEST_DUAL_IMU_PID 구현 계획

> 과거 문서: 대체된 듀얼 IMU PID 반복 작업을 설명한다.
> 현행 비행 제어 후보:
> [`dual_imu_cascade_pwm`](../../../firmware/archive/legacy_flight/dual_imu_cascade_pwm/).
> 보관된 결과물:
> [`dual_imu_pid_pwm`](../../../firmware/archive/legacy_flight/dual_imu_pid_pwm/).

> **에이전트 작업자를 위한 안내:** 필수 서브 스킬: 이 계획을 태스크 단위로 구현하려면 superpowers:subagent-driven-development(권장) 또는 superpowers:executing-plans를 사용한다. 각 단계는 추적을 위해 체크박스(`- [ ]`) 구문을 사용한다.

**목표:** 단일 IMU `PWM_TEST_IMU_PID.ino`에서 나타난 비행 중 드리프트를 제거하는 새로운 듀얼 IMU 비행 제어 스케치를 만든다. 시동 시 자이로 바이어스 보정, 런타임 바이어스 추정, 고장 검출을 포함한 듀얼 IMU 융합, 적응형 상보 필터를 추가하면서, `scripts/tune_pid.py`와의 기존 UDP 프로토콜은 100% 호환되도록 유지한다.

**아키텍처:** FreeRTOS 2-태스크 구조(Core 1: PID 1kHz, Core 0: UDP)를 갖는 단일 Arduino .ino 파일. 공유 SPI에 연결된 두 개의 ICM42670 IMU(CS=10, CS=9). 각 태스크는 점진적으로 구축된다: 골격 → 듀얼 읽기 → 바이어스 보정 → 융합 + 고장 검출 → 적응형 alpha → 런타임 바이어스 추정 → 최종 통합.

**기술 스택:** ESP32-S3 상의 Arduino 프레임워크, ICM42670P 라이브러리, FreeRTOS, WiFi/UDP, ledcAttach PWM.

**사양:** [`2026-05-14-dual-imu-pid-design.md`](../specs/2026-05-14-dual-imu-pid-design.md)

## 펌웨어 테스트 전략

이것은 하드웨어 펌웨어이므로 단위 테스트를 실행할 수 없다. 각 태스크에서의 검증은 다음과 같다:

1. `arduino-cli compile`를 사용한 **컴파일 확인** — 프로그램 방식, 자동
2. 사용자에 의한 **하드웨어 검증** — 플래시, 시리얼 모니터 관찰, 각 태스크에 기술된 수동 테스트

컴파일 실패 = 커밋 전 중단. 하드웨어 검증은 각 태스크에 문서화되어 있으며, 사용자가 수동으로 실행하고 문제가 있으면 보고한다.

## 히스토리 컴파일 명령 (지원되지 않음; 실행하지 말 것)

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/archive/legacy_flight/dual_imu_pid_pwm/
```

과거의 예상 출력은 종료 코드 0과 함께 `Used library ... Used platform ...`
였다. 이 보관된 대상은 더 이상 빌드가 지원되지 않으므로, 현재 검증의 일부로
이 명령들을 수정하거나 실행하지 말 것.

---

## 파일 구조

기존 examples/ 패턴에 맞춰, 새 폴더에 단일 스케치 파일:

- **생성:** `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino`

다른 파일은 없다. 헤더나 라이브러리 분리 없음 — 기존 예제와의 응집성을 유지하며, 컨텍스트에 담아두기에 충분히 짧다.

---

## Task 1: 듀얼 IMU 초기화 + 단일 읽기로 골격 구성

**목표:** 새 폴더와 `.ino` 파일을 생성한다. `PWM_TEST_IMU_PID.ino`의 기본 구조를 복사하되 두 개의 `ICM42670` 인스턴스를 둔다. IMU1만 읽는다(평균화 로직은 Task 3으로 미룸). 이 태스크는 원본과 기능적으로 동등한 빌드를 만들며, IMU2가 IMU1과 함께 초기화될 수 있음을 이미 검증한다.

**파일:**
- 생성: `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino`

- [ ] **Step 1: 골격과 함께 폴더 및 파일 생성**

`firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino`를 다음 내용으로 작성한다:

```cpp
#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================================
// 1. Tuning parameters (unchanged from single-IMU version)
// ==========================================================
const float LPF_ALPHA_ACC  = 0.10f;
const float LPF_ALPHA_GYRO = 0.50f;

volatile float Kp_Roll  = 2.5f,  Ki_Roll  = 0.005f, Kd_Roll  = 1.2f;
volatile float Kp_Pitch = 2.5f,  Ki_Pitch = 0.005f, Kd_Pitch = 1.2f;
volatile float Kp_Yaw   = 3.5f,  Ki_Yaw   = 0.0f,   Kd_Yaw   = 0.0f;

volatile int  base_throttle = 1000;
volatile int  min_throttle  = 1050;
volatile int  max_throttle  = 1300;
volatile bool yaw_enabled   = false;

// ==========================================================
// 2. System constants
// ==========================================================
const char* WIFI_SSID    = "Drone_Tuning";
const char* WIFI_PASS    = "12345678";
const int   UDP_PORT     = 4210;
const int   WIFI_CHANNEL = 6;

const int pinM1   = 4;   // FL
const int pinM2   = 5;   // RR
const int pinM3   = 6;   // FR
const int pinM4   = 7;   // RL
const int SPI_CS1 = 10;  // IMU1 CS
const int SPI_CS2 = 9;   // IMU2 CS (replaces former LDO_PIN)

const int ESC_FREQ   = 400;
const int ESC_RES    = 14;
const int ESC_PERIOD = 2500;

const float GYRO_SCALE   = 1.0f / 16.4f;
const float ACCEL_SCALE  = 1.0f / 2048.0f;
const float SAFETY_ANGLE = 45.0f;
const float YAW_DEADZONE = 0.3f;

const uint32_t RC_TIMEOUT_MS     = 500;
const float    IMU_FROZEN_THRESH = 0.001f;
const uint32_t IMU_FROZEN_MS     = 200;

// ==========================================================
// 3. System variables
// ==========================================================
WiFiUDP   udp;
char      packetBuffer[256];
IPAddress laptopIP;
int       laptopPort            = 0;
bool      connectionEstablished = false;

ICM42670 IMU1(SPI, SPI_CS1);
ICM42670 IMU2(SPI, SPI_CS2);

volatile bool  safety_lock  = true;
volatile float targetAngleX = 0.0f, targetAngleY = 0.0f, targetAngleZ = 0.0f;

float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f;
float errorSumRoll = 0.0f, errorSumPitch = 0.0f, errorSumYaw = 0.0f;

volatile float raw_gx = 0.0f, raw_gy = 0.0f, raw_gz = 0.0f;
volatile float raw_ax = 0.0f, raw_ay = 0.0f, raw_az = 0.0f;
float lpf_ax = 0.0f, lpf_ay = 0.0f, lpf_az = 0.0f;
float lpf_gx = 0.0f, lpf_gy = 0.0f, lpf_gz = 0.0f;

volatile uint32_t lastRcTimeMs   = 0;
volatile bool     fault_rc       = false;
volatile bool     fault_imu      = false;   // OR of fault_imu1/2 for telemetry compatibility
float             prev_gx        = 0.0f;
uint32_t          imuFrozenSince = 0;

volatile uint32_t lastRcSeq      = 0;
volatile uint32_t rcTotalPkts    = 0;
volatile uint32_t rcDroppedPkts  = 0;

// ==========================================================
// 4. Motor control
// ==========================================================
void writeMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);
  uint32_t duty = ((uint32_t)us * 16383UL) / ESC_PERIOD;
  ledcWrite(pin, duty);
}

void stopMotors() {
  writeMotor(pinM1, 1000);
  writeMotor(pinM2, 1000);
  writeMotor(pinM3, 1000);
  writeMotor(pinM4, 1000);
}

// ==========================================================
// 5. Fault detection (single-IMU version for now; refined in Task 4)
// ==========================================================
bool check_rc_timeout() {
  if (safety_lock) return false;
  uint32_t elapsed = millis() - lastRcTimeMs;
  if (elapsed > RC_TIMEOUT_MS) {
    fault_rc    = true;
    safety_lock = true;
    Serial.printf("[FAULT] RC TIMEOUT (%ums)\n", elapsed);
    return true;
  }
  return false;
}

bool check_imu_frozen() {
  if (fabsf(raw_gx - prev_gx) < IMU_FROZEN_THRESH) {
    if (imuFrozenSince == 0) imuFrozenSince = millis();
    if (millis() - imuFrozenSince > IMU_FROZEN_MS) {
      fault_imu   = true;
      safety_lock = true;
      Serial.printf("[FAULT] IMU FROZEN (%ums)\n", millis() - imuFrozenSince);
      return true;
    }
  } else {
    imuFrozenSince = 0;
  }
  prev_gx = raw_gx;
  return false;
}

bool check_attitude() {
  if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
    safety_lock = true;
    Serial.printf("[FAULT] OVER-TILT (Roll:%.1f Pitch:%.1f)\n", angleX, angleY);
    return true;
  }
  return false;
}

// ==========================================================
// 6. PID task (Core 1, 1kHz) — single IMU read for now
// ==========================================================
const float ALPHA_COMP = 0.995f;  // temporary; replaced in Task 5

void pid_task(void *pvParameters) {
  const unsigned long LOOP_INTERVAL = 1000;
  unsigned long nextLoopTime = micros();
  unsigned long lastTime      = micros();

  inv_imu_sensor_event_t e1;

  IMU1.getDataFromRegisters(e1);
  lpf_ax =  e1.accel[0] * ACCEL_SCALE;
  lpf_ay = -e1.accel[1] * ACCEL_SCALE;
  lpf_az =  e1.accel[2] * ACCEL_SCALE;
  lpf_gx =  e1.gyro[0]  * GYRO_SCALE;
  lpf_gy = -e1.gyro[1]  * GYRO_SCALE;
  lpf_gz =  e1.gyro[2]  * GYRO_SCALE;

  while (true) {
    unsigned long now = micros();
    if (now < nextLoopTime) { vTaskDelay(0); continue; }
    nextLoopTime = now + LOOP_INTERVAL;

    IMU1.getDataFromRegisters(e1);
    raw_ax =  e1.accel[0] * ACCEL_SCALE;
    raw_ay = -e1.accel[1] * ACCEL_SCALE;
    raw_az =  e1.accel[2] * ACCEL_SCALE;
    raw_gx =  e1.gyro[0]  * GYRO_SCALE;
    raw_gy = -e1.gyro[1]  * GYRO_SCALE;
    raw_gz =  e1.gyro[2]  * GYRO_SCALE;

    lpf_ax = LPF_ALPHA_ACC  * raw_ax + (1.0f - LPF_ALPHA_ACC)  * lpf_ax;
    lpf_ay = LPF_ALPHA_ACC  * raw_ay + (1.0f - LPF_ALPHA_ACC)  * lpf_ay;
    lpf_az = LPF_ALPHA_ACC  * raw_az + (1.0f - LPF_ALPHA_ACC)  * lpf_az;
    lpf_gx = LPF_ALPHA_GYRO * raw_gx + (1.0f - LPF_ALPHA_GYRO) * lpf_gx;
    lpf_gy = LPF_ALPHA_GYRO * raw_gy + (1.0f - LPF_ALPHA_GYRO) * lpf_gy;
    lpf_gz = LPF_ALPHA_GYRO * raw_gz + (1.0f - LPF_ALPHA_GYRO) * lpf_gz;

    float dt = (now - lastTime) / 1000000.0f;
    if (dt > 0.002f) dt = 0.001f;
    lastTime = now;

    float accAngleX = atan2f(lpf_ay, sqrtf(lpf_ax*lpf_ax + lpf_az*lpf_az)) * 180.0f / PI;
    float accAngleY = atan2f(-lpf_ax, sqrtf(lpf_ay*lpf_ay + lpf_az*lpf_az)) * 180.0f / PI;
    angleX = ALPHA_COMP * (angleX + lpf_gx * dt) + (1.0f - ALPHA_COMP) * accAngleX;
    angleY = ALPHA_COMP * (angleY + lpf_gy * dt) + (1.0f - ALPHA_COMP) * accAngleY;
    if (fabsf(lpf_gz) > YAW_DEADZONE) angleZ += lpf_gz * dt;

    if (check_imu_frozen() || check_rc_timeout() || check_attitude() || safety_lock) {
      stopMotors();
      vTaskDelay(0);
      continue;
    }

    float errorRoll  = targetAngleX - angleX;
    float errorPitch = targetAngleY - angleY;
    float errorYaw   = targetAngleZ - angleZ;

    if (base_throttle < 1100) {
      errorSumRoll = errorSumPitch = errorSumYaw = 0.0f;
    } else {
      if (fabsf(errorRoll)  < 25.0f) errorSumRoll  = constrain(errorSumRoll  + errorRoll  * dt, -15.0f, 15.0f);
      if (fabsf(errorPitch) < 25.0f) errorSumPitch = constrain(errorSumPitch + errorPitch * dt, -15.0f, 15.0f);
      if (fabsf(errorYaw)   < 25.0f) errorSumYaw   = constrain(errorSumYaw   + errorYaw   * dt, -15.0f, 15.0f);
    }

    float pid_roll  = errorRoll  * Kp_Roll  + errorSumRoll  * Ki_Roll  - lpf_gx * Kd_Roll;
    float pid_pitch = errorPitch * Kp_Pitch + errorSumPitch * Ki_Pitch - lpf_gy * Kd_Pitch;
    float pid_yaw   = yaw_enabled
                    ? (errorYaw * Kp_Yaw + errorSumYaw * Ki_Yaw - lpf_gz * Kd_Yaw)
                    : 0.0f;

    writeMotor(pinM1, constrain((int)(base_throttle - pid_pitch + pid_roll - pid_yaw), min_throttle, max_throttle));
    writeMotor(pinM2, constrain((int)(base_throttle + pid_pitch - pid_roll - pid_yaw), min_throttle, max_throttle));
    writeMotor(pinM3, constrain((int)(base_throttle - pid_pitch - pid_roll + pid_yaw), min_throttle, max_throttle));
    writeMotor(pinM4, constrain((int)(base_throttle + pid_pitch + pid_roll + pid_yaw), min_throttle, max_throttle));
  }
}

// ==========================================================
// 7. UDP task (Core 0) — identical to original
// ==========================================================
void udp_task(void *pvParameters) {
  const int CTRL_MARGIN = 150;

  while (true) {
    int packetSize = udp.parsePacket();
    if (!packetSize) { vTaskDelay(2); continue; }

    laptopIP   = udp.remoteIP();
    laptopPort = udp.remotePort();
    connectionEstablished = true;

    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len <= 0) { vTaskDelay(2); continue; }
    packetBuffer[len] = '\0';

    String cmd = String(packetBuffer);
    cmd.trim();

    if (cmd.startsWith("rc ")) {
      int s1 = cmd.indexOf(' ');
      int s2 = cmd.indexOf(' ', s1 + 1);
      int s3 = cmd.indexOf(' ', s2 + 1);
      int s4 = cmd.indexOf(' ', s3 + 1);

      if (s1 > 0 && s2 > 0 && s3 > 0) {
        uint32_t seq = (uint32_t)cmd.substring(s1 + 1, s2).toInt();
        rcTotalPkts++;

        if (seq <= lastRcSeq && lastRcSeq != 0) {
          rcDroppedPkts++;
          vTaskDelay(1);
          continue;
        }
        lastRcSeq    = seq;
        lastRcTimeMs = millis();

        targetAngleX = cmd.substring(s2 + 1, s3).toFloat();
        targetAngleY = cmd.substring(s3 + 1, s4 > 0 ? s4 : cmd.length()).toFloat();
        if (s4 > 0) targetAngleZ = cmd.substring(s4 + 1).toFloat();
      }
    }
    else if (cmd == "start") {
      fault_rc = fault_imu = false;
      lastRcSeq     = 0;
      lastRcTimeMs  = millis();
      safety_lock   = false;
      base_throttle = 1100; min_throttle = 1050; max_throttle = 1250;
      targetAngleX  = targetAngleY = targetAngleZ = 0.0f;
      angleZ        = errorSumRoll = errorSumPitch = errorSumYaw = 0.0f;
      Serial.println(">>> START");
    }
    else if (cmd == "stop") {
      safety_lock   = true;
      base_throttle = 1000;
      stopMotors();
      Serial.println(">>> STOP");
    }
    else if (cmd.startsWith("yaw ")) {
      yaw_enabled = (cmd.substring(4).toInt() == 1);
      Serial.printf(">>> Yaw: %s\n", yaw_enabled ? "ON" : "OFF");
    }
    else if (cmd.startsWith("th ")) {
      int new_base  = cmd.substring(3).toInt();
      base_throttle = new_base;
      min_throttle  = max(1050, new_base - CTRL_MARGIN);
      max_throttle  = min(1900, new_base + CTRL_MARGIN);
    }
    else {
      float val = cmd.substring(2).toFloat();
      if      (cmd.startsWith("pa")) { Kp_Roll = Kp_Pitch = val; }
      else if (cmd.startsWith("da")) { Kd_Roll = Kd_Pitch = val; }
      else if (cmd.startsWith("ia")) { Ki_Roll = Ki_Pitch = val; }
      else if (cmd.startsWith("pp")) { Kp_Pitch = val; }
      else if (cmd.startsWith("dp")) { Kd_Pitch = val; }
      else if (cmd.startsWith("ip")) { Ki_Pitch = val; }
      else if (cmd.startsWith("pr")) { Kp_Roll  = val; }
      else if (cmd.startsWith("dr")) { Kd_Roll  = val; }
      else if (cmd.startsWith("ir")) { Ki_Roll  = val; }
      else if (cmd.startsWith("py")) { Kp_Yaw   = val; }
      else if (cmd.startsWith("dy")) { Kd_Yaw   = val; }
      else if (cmd.startsWith("iy")) { Ki_Yaw   = val; }
    }

    vTaskDelay(1);
  }
}

// ==========================================================
// 8. setup / loop
// ==========================================================
void setup() {
  Serial.begin(115200);

  // Both CS HIGH before SPI init (prevents bus contention from floating CS)
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
  stopMotors();

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

  xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 0, NULL, 0);

  Serial.println("SYSTEM READY (Task 1 scaffold: dual IMU init, single IMU read)");
}

void loop() {
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime < 50) return;
  lastSendTime = millis();

  if (!connectionEstablished) return;

  udp.beginPacket(laptopIP, laptopPort);
  udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%lu,%lu",
             angleX, angleY, angleZ,
             raw_gx, raw_gy, raw_gz,
             raw_ax, raw_ay, raw_az,
             base_throttle,
             (int)fault_rc,
             (int)fault_imu,
             rcTotalPkts,
             rcDroppedPkts);
  udp.endPacket();
}
```

- [ ] **Step 2: 컴파일**

실행:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/archive/legacy_flight/dual_imu_pid_pwm/
```

예상: 종료 코드 0, 오류 없음. 오류가 있으면 수정하고 깨끗해질 때까지 다시 실행한다.

- [ ] **Step 3: 하드웨어 검증 (사용자)**

사용자가 플래시하고 시리얼에 `SYSTEM READY (Task 1 scaffold...)`가 표시되며 `IMU1 INIT FAILED` / `IMU2 INIT FAILED`가 없는지 확인한다. 드론은 시동되면 안 된다.

- [ ] **Step 4: 커밋**

```bash
git add firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino
git commit -m "feat: scaffold PWM_TEST_DUAL_IMU_PID with dual IMU init"
```

---

## Task 2: 두 IMU를 모두 읽고 원시값 평균화 (아직 바이어스 없음)

**목표:** `pid_task`의 단일 IMU1 읽기를 두 IMU 모두에서 읽어 단순 평균을 사용하도록 교체한다. 아직 바이어스 보정이나 고장 검출은 없다 — 그것은 Task 3과 Task 4에서 다룬다.

**파일:**
- 수정: `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino` (PID 태스크만)

- [ ] **Step 1: `pid_task`의 초기 읽기 수정**

`inv_imu_sensor_event_t e1;`과 첫 번째 읽기 블록(주석 "// 6. PID task..." 바로 다음)을 다음으로 교체한다:

```cpp
  inv_imu_sensor_event_t e1, e2;

  IMU1.getDataFromRegisters(e1);
  IMU2.getDataFromRegisters(e2);
  lpf_ax =  ((e1.accel[0] + e2.accel[0]) * 0.5f) * ACCEL_SCALE;
  lpf_ay = -((e1.accel[1] + e2.accel[1]) * 0.5f) * ACCEL_SCALE;
  lpf_az =  ((e1.accel[2] + e2.accel[2]) * 0.5f) * ACCEL_SCALE;
  lpf_gx =  ((e1.gyro[0]  + e2.gyro[0])  * 0.5f) * GYRO_SCALE;
  lpf_gy = -((e1.gyro[1]  + e2.gyro[1])  * 0.5f) * GYRO_SCALE;
  lpf_gz =  ((e1.gyro[2]  + e2.gyro[2])  * 0.5f) * GYRO_SCALE;
```

- [ ] **Step 2: `pid_task`의 루프 내부 읽기 수정**

루프 내부의 `IMU1.getDataFromRegisters(e1);` 블록(6개의 raw_* 대입)을 다음으로 교체한다:

```cpp
    IMU1.getDataFromRegisters(e1);
    IMU2.getDataFromRegisters(e2);
    raw_ax =  ((e1.accel[0] + e2.accel[0]) * 0.5f) * ACCEL_SCALE;
    raw_ay = -((e1.accel[1] + e2.accel[1]) * 0.5f) * ACCEL_SCALE;
    raw_az =  ((e1.accel[2] + e2.accel[2]) * 0.5f) * ACCEL_SCALE;
    raw_gx =  ((e1.gyro[0]  + e2.gyro[0])  * 0.5f) * GYRO_SCALE;
    raw_gy = -((e1.gyro[1]  + e2.gyro[1])  * 0.5f) * GYRO_SCALE;
    raw_gz =  ((e1.gyro[2]  + e2.gyro[2])  * 0.5f) * GYRO_SCALE;
```

- [ ] **Step 3: 준비 메시지 갱신**

`Serial.println("SYSTEM READY (Task 1 ...)");` 줄을 다음으로 교체한다:

```cpp
  Serial.println("SYSTEM READY (Task 2: dual IMU averaged)");
```

- [ ] **Step 4: 컴파일**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/archive/legacy_flight/dual_imu_pid_pwm/
```

- [ ] **Step 5: 하드웨어 검증 (사용자)**

플래시한 뒤 scripts/tune_pid.py를 연다. 텔레메트리를 관찰한다: 드론이 정지 상태일 때 `angleX`/`angleY`는 0에 가깝고 안정적이어야 한다. `raw_gx/y/z`는 이전 단일 IMU 버전과 비슷한 크기여야 한다(같은 방향의 두 IMU를 평균하면 평균값은 같고 노이즈는 줄어든다).

- [ ] **Step 6: 커밋**

```bash
git add firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino
git commit -m "feat: read both IMUs and average raw values"
```

---

## Task 3: 시동 시 자이로 바이어스 보정

**목표:** IMU 초기화 후 `setup()`에서 실행되는 `calibrate_bias()`를 추가한다. IMU당 2000개의 샘플을 측정하고, 움직임을 확인하며, IMU별 바이어스 배열을(**드론 좌표계**로, IMU2 부호 보정을 적용하여) 저장하고, 최대 3회 재시도한다.

> **참고 (Task 2 수정):** IMU2는 IMU1 대비 x축과 z축이 반전되어 장착되어 있다; `IMU2_SIGN_X/Y/Z` 상수는 Task 2 수정에서 섹션 2에 추가되었다. Task 3은 `gyro_bias2`가 `gyro_bias1`과 동일한 (IMU1/드론) 좌표계에 있도록 바이어스 측정 시 이 부호들을 적용해야 한다. 그렇지 않으면 평균화 중의 바이어스 감산이 잘못된 좌표계에서 이루어진다.

**파일:**
- 수정: `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino`

- [ ] **Step 1: 섹션 2에 바이어스 상수 추가 (`YAW_DEADZONE` 줄 다음)**

```cpp
const int      BIAS_CALIB_SAMPLES         = 2000;
const float    BIAS_CALIB_MOVEMENT_THRESH = 1.0f;   // deg/s std-dev limit per axis
const int      BIAS_CALIB_RETRIES         = 3;
```

- [ ] **Step 2: 섹션 3에 바이어스 상태 변수 추가 (`lpf_gx/y/z` 줄 다음)**

```cpp
// Per-IMU gyro bias (deg/s), filled by calibrate_bias()
float gyro_bias1[3] = {0.0f, 0.0f, 0.0f};
float gyro_bias2[3] = {0.0f, 0.0f, 0.0f};
```

- [ ] **Step 3: `pid_task` 앞에 `calibrate_bias()` 함수 추가**

`// 6. PID task` 주석 앞에 삽입한다:

```cpp
// ==========================================================
// Startup gyro bias calibration
// ==========================================================
// `sign` brings the IMU's gyro into the drone (IMU1) frame.
// For IMU1 pass {+1, +1, +1}; for IMU2 pass {IMU2_SIGN_X, IMU2_SIGN_Y, IMU2_SIGN_Z}.
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
    delayMicroseconds(1000);  // ~1 kHz sampling
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
```

이 단계 이후, `gyro_bias1`과 `gyro_bias2`는 모두 **IMU1/드론 좌표계**로 저장된다(누적 중에 IMU2의 축 부호 보정이 적용됨). 이후 코드는 부호 보정된 자이로 값에서 바이어스를 감산한다 — Step 5 참조.

- [ ] **Step 4: setup()에서 IMU startAccel/startGyro 이후, xTaskCreatePinnedToCore 이전에 `calibrate_bias()` 호출**

`setup()`에서 다음 블록을 찾는다:

```cpp
  IMU2.startAccel(1600, 16);
  IMU2.startGyro(1600, 2000);
  delay(500);

  xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
```

다음으로 변경한다:

```cpp
  IMU2.startAccel(1600, 16);
  IMU2.startGyro(1600, 2000);
  delay(500);

  calibrate_bias();

  xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
```

- [ ] **Step 5: `pid_task` 읽기에 바이어스 보정 적용**

`pid_task`에서 루프 내부 듀얼 읽기 블록을 IMU2 부호 보정을 적용하고 IMU별 바이어스를 감산하도록 변경한다(바이어스는 Step 3에서 이미 드론 좌표계에 있음). 루프 내부 읽기 블록 전체(Task 2에서 `IMU2_SIGN_X * e2.gyro[0]` 등을 사용하던 것)를 다음으로 교체한다:

```cpp
    IMU1.getDataFromRegisters(e1);
    IMU2.getDataFromRegisters(e2);

    // Sign-corrected (drone-frame) gyro reads, then subtract bias
    float gx1 =                  e1.gyro[0]  * GYRO_SCALE - gyro_bias1[0];
    float gy1 =                  e1.gyro[1]  * GYRO_SCALE - gyro_bias1[1];
    float gz1 =                  e1.gyro[2]  * GYRO_SCALE - gyro_bias1[2];
    float gx2 = IMU2_SIGN_X    * e2.gyro[0]  * GYRO_SCALE - gyro_bias2[0];
    float gy2 = IMU2_SIGN_Y    * e2.gyro[1]  * GYRO_SCALE - gyro_bias2[1];
    float gz2 = IMU2_SIGN_Z    * e2.gyro[2]  * GYRO_SCALE - gyro_bias2[2];

    // Accel: average sign-corrected reads, then apply drone Y flip at the end
    raw_ax =  ((e1.accel[0] + IMU2_SIGN_X * e2.accel[0]) * 0.5f) * ACCEL_SCALE;
    raw_ay = -((e1.accel[1] + IMU2_SIGN_Y * e2.accel[1]) * 0.5f) * ACCEL_SCALE;
    raw_az =  ((e1.accel[2] + IMU2_SIGN_Z * e2.accel[2]) * 0.5f) * ACCEL_SCALE;
    // Gyro: average bias-corrected per-IMU values, then apply drone Y flip
    raw_gx =  (gx1 + gx2) * 0.5f;
    raw_gy = -(gy1 + gy2) * 0.5f;
    raw_gz =  (gz1 + gz2) * 0.5f;
```

근거: `gx1/gy1/gz1`과 `gx2/gy2/gz2`는 바이어스 감산 후 각 IMU 자체 좌표계에서의 IMU별 자이로 측정값이다(바이어스는 Step 3에서 Y 반전 이전에 측정됨). `raw_gy`에 대한 바깥쪽 Y 부호 반전은 결과를 드론의 Y축 규약으로 가져오며, `atan2f`와 PID가 사용하는 원래의 부호 동작과 일치시킨다.

사전 루프 초기화도 유사하게 교체한다:

```cpp
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
```

중요한 부호 주의사항: IMU1의 바이어스(`gyro_bias1`)는 원시 `e1.gyro[k] * GYRO_SCALE`(Y 반전 없음)로 측정되었다. IMU2도 동일하지만 누적 중에 부호 보정이 적용된다. 따라서 pid_task에서는 동일하게 부호 보정된, Y 반전 이전 값에서 바이어스를 감산한다. 드론 좌표계의 Y 반전은 맨 마지막에 `raw_gy`와 `lpf_gy`에 한 번만 적용된다.

- [ ] **Step 6: 준비 메시지 갱신**

```cpp
  Serial.println("SYSTEM READY (Task 3: bias calibration)");
```

- [ ] **Step 7: 컴파일**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/archive/legacy_flight/dual_imu_pid_pwm/
```

- [ ] **Step 8: 하드웨어 검증 (사용자)**

플래시한다. 부팅 시 시리얼에 `[CALIB] attempt 1/3`, 이어서 두 IMU에 대한 `[CALIB] IMU1 bias x=... y=... z=...`, 그다음 `[CALIB] OK`가 표시되어야 한다. 바이어스 값은 일반적으로 -2 ~ +2 deg/s 범위여야 한다. 부팅 후 드론이 정지 상태이면 `raw_gx/y/z`(텔레메트리)는 0에 매우 가깝게 유지되어야 한다.

그런 다음 드론을 1-2분간 그대로 두고(**시동하지 말 것**) `angleZ`를 관찰한다 — 이제 yaw 적분이 바이어스 보정된 자이로를 사용하므로 이전 버전보다 훨씬 적게 드리프트해야 한다.

- [ ] **Step 9: 커밋**

```bash
git add firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino
git commit -m "feat: startup gyro bias calibration for both IMUs"
```

---

## Task 4: 폴백을 갖춘 IMU별 고장 검출 + 불일치 확인

**목표:** 단일 `check_imu_frozen()`을 폴백을 허용하는 IMU별 고착 검출로 교체하고, 두 IMU가 크게 불일치하면 안전 잠금을 거는 새로운 `check_imu_disagree()`를 추가한다.

**파일:**
- 수정: `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino`

- [ ] **Step 1: 섹션 2에 새로운 불일치 상수 추가**

`IMU_FROZEN_MS` 줄 다음에:

```cpp
const float    IMU_DISAGREE_GYRO  = 30.0f;  // deg/s
const float    IMU_DISAGREE_ACCEL = 0.5f;   // G
const uint32_t IMU_DISAGREE_MS    = 100;
```

- [ ] **Step 2: 섹션 3에서 단일 IMU 고장 상태를 IMU별 상태로 교체**

찾을 부분:

```cpp
volatile bool     fault_imu      = false;   // OR of fault_imu1/2 for telemetry compatibility
float             prev_gx        = 0.0f;
uint32_t          imuFrozenSince = 0;
```

다음으로 교체:

```cpp
volatile bool fault_imu  = false;   // OR of fault_imu1/2 for telemetry compatibility
volatile bool fault_imu1 = false;
volatile bool fault_imu2 = false;

float    prev_gx1 = 0.0f, prev_gx2 = 0.0f;
uint32_t frozenSince1 = 0, frozenSince2 = 0;
uint32_t disagreeSince = 0;

// Latest per-IMU corrected gyro/accel (filled each loop)
float gx1_c = 0, gy1_c = 0, gz1_c = 0, ax1_c = 0, ay1_c = 0, az1_c = 0;
float gx2_c = 0, gy2_c = 0, gz2_c = 0, ax2_c = 0, ay2_c = 0, az2_c = 0;
```

- [ ] **Step 3: `check_imu_frozen()`을 IMU별 버전으로 교체**

`check_imu_frozen()` 함수 전체를 다음으로 교체한다:

```cpp
// Returns true if BOTH IMUs frozen (lethal). Sets individual fault flags.
static bool check_imu_frozen() {
  bool f1 = (fabsf(gx1_c - prev_gx1) < IMU_FROZEN_THRESH);
  bool f2 = (fabsf(gx2_c - prev_gx2) < IMU_FROZEN_THRESH);

  if (f1) {
    if (frozenSince1 == 0) frozenSince1 = millis();
    if (millis() - frozenSince1 > IMU_FROZEN_MS) fault_imu1 = true;
  } else {
    frozenSince1 = 0;
    fault_imu1 = false;
  }

  if (f2) {
    if (frozenSince2 == 0) frozenSince2 = millis();
    if (millis() - frozenSince2 > IMU_FROZEN_MS) fault_imu2 = true;
  } else {
    frozenSince2 = 0;
    fault_imu2 = false;
  }

  prev_gx1 = gx1_c;
  prev_gx2 = gx2_c;

  fault_imu = fault_imu1 || fault_imu2;

  if (fault_imu1 && fault_imu2) {
    safety_lock = true;
    Serial.println("[FAULT] BOTH IMUS FROZEN");
    return true;
  }
  return false;
}

// Returns true if IMUs disagree beyond thresholds for >IMU_DISAGREE_MS.
static bool check_imu_disagree() {
  float dgx = fabsf(gx1_c - gx2_c);
  float dgy = fabsf(gy1_c - gy2_c);
  float dgz = fabsf(gz1_c - gz2_c);
  float dax = fabsf(ax1_c - ax2_c);
  float day = fabsf(ay1_c - ay2_c);
  float daz = fabsf(az1_c - az2_c);

  bool gyro_bad  = (dgx > IMU_DISAGREE_GYRO)  || (dgy > IMU_DISAGREE_GYRO)  || (dgz > IMU_DISAGREE_GYRO);
  bool accel_bad = (dax > IMU_DISAGREE_ACCEL) || (day > IMU_DISAGREE_ACCEL) || (daz > IMU_DISAGREE_ACCEL);

  if (gyro_bad || accel_bad) {
    if (disagreeSince == 0) disagreeSince = millis();
    if (millis() - disagreeSince > IMU_DISAGREE_MS) {
      // If one is already known frozen, trust the other one — skip lock
      if (fault_imu1 ^ fault_imu2) {
        return false;
      }
      safety_lock = true;
      Serial.printf("[FAULT] IMU DISAGREE (dg %.2f %.2f %.2f, da %.2f %.2f %.2f)\n",
                    dgx, dgy, dgz, dax, day, daz);
      return true;
    }
  } else {
    disagreeSince = 0;
  }
  return false;
}
```

- [ ] **Step 4: `pid_task`의 루프 내부 읽기를 재구성하여 IMU별 보정값을 채우고 폴백과 함께 융합**

보정값 `gx1_c..az2_c`는 **드론 좌표계**로 저장된다(IMU2에 대해 부호 보정되고 Y는 이미 반전됨). 융합은 이들을 평균하여 `raw_*`에 직접 기록한다.

Task 3의 루프 내부 듀얼 읽기 블록을 다음으로 교체한다:

```cpp
    IMU1.getDataFromRegisters(e1);
    IMU2.getDataFromRegisters(e2);

    // Per-IMU gyro in drone frame (sign-corrected, Y-flipped, bias-subtracted)
    gx1_c =  (              e1.gyro[0] * GYRO_SCALE - gyro_bias1[0]);
    gy1_c = -(              e1.gyro[1] * GYRO_SCALE - gyro_bias1[1]);
    gz1_c =  (              e1.gyro[2] * GYRO_SCALE - gyro_bias1[2]);
    gx2_c =  (IMU2_SIGN_X * e2.gyro[0] * GYRO_SCALE - gyro_bias2[0]);
    gy2_c = -(IMU2_SIGN_Y * e2.gyro[1] * GYRO_SCALE - gyro_bias2[1]);
    gz2_c =  (IMU2_SIGN_Z * e2.gyro[2] * GYRO_SCALE - gyro_bias2[2]);

    // Per-IMU accel in drone frame (sign-corrected, Y-flipped)
    ax1_c =  (              e1.accel[0]) * ACCEL_SCALE;
    ay1_c = -(              e1.accel[1]) * ACCEL_SCALE;
    az1_c =  (              e1.accel[2]) * ACCEL_SCALE;
    ax2_c =  (IMU2_SIGN_X * e2.accel[0]) * ACCEL_SCALE;
    ay2_c = -(IMU2_SIGN_Y * e2.accel[1]) * ACCEL_SCALE;
    az2_c =  (IMU2_SIGN_Z * e2.accel[2]) * ACCEL_SCALE;

    // Fuse: average if both healthy, else use the healthy one
    if (fault_imu1 && !fault_imu2) {
      raw_gx = gx2_c; raw_gy = gy2_c; raw_gz = gz2_c;
      raw_ax = ax2_c; raw_ay = ay2_c; raw_az = az2_c;
    } else if (fault_imu2 && !fault_imu1) {
      raw_gx = gx1_c; raw_gy = gy1_c; raw_gz = gz1_c;
      raw_ax = ax1_c; raw_ay = ay1_c; raw_az = az1_c;
    } else {
      raw_gx = (gx1_c + gx2_c) * 0.5f;
      raw_gy = (gy1_c + gy2_c) * 0.5f;
      raw_gz = (gz1_c + gz2_c) * 0.5f;
      raw_ax = (ax1_c + ax2_c) * 0.5f;
      raw_ay = (ay1_c + ay2_c) * 0.5f;
      raw_az = (az1_c + az2_c) * 0.5f;
    }
```

사전 루프 초기 읽기도 유사하게 갱신하여 `gx1_c..az2_c`를 채우고, (융합 후 이제 존재하는) `raw_*`로부터 `lpf_*`를 시딩한다:

Task 3의 사전 루프 초기화를 다음으로 교체한다:

```cpp
  IMU1.getDataFromRegisters(e1);
  IMU2.getDataFromRegisters(e2);

  gx1_c =  (              e1.gyro[0] * GYRO_SCALE - gyro_bias1[0]);
  gy1_c = -(              e1.gyro[1] * GYRO_SCALE - gyro_bias1[1]);
  gz1_c =  (              e1.gyro[2] * GYRO_SCALE - gyro_bias1[2]);
  gx2_c =  (IMU2_SIGN_X * e2.gyro[0] * GYRO_SCALE - gyro_bias2[0]);
  gy2_c = -(IMU2_SIGN_Y * e2.gyro[1] * GYRO_SCALE - gyro_bias2[1]);
  gz2_c =  (IMU2_SIGN_Z * e2.gyro[2] * GYRO_SCALE - gyro_bias2[2]);
  ax1_c =  (              e1.accel[0]) * ACCEL_SCALE;
  ay1_c = -(              e1.accel[1]) * ACCEL_SCALE;
  az1_c =  (              e1.accel[2]) * ACCEL_SCALE;
  ax2_c =  (IMU2_SIGN_X * e2.accel[0]) * ACCEL_SCALE;
  ay2_c = -(IMU2_SIGN_Y * e2.accel[1]) * ACCEL_SCALE;
  az2_c =  (IMU2_SIGN_Z * e2.accel[2]) * ACCEL_SCALE;

  lpf_ax = (ax1_c + ax2_c) * 0.5f;
  lpf_ay = (ay1_c + ay2_c) * 0.5f;
  lpf_az = (az1_c + az2_c) * 0.5f;
  lpf_gx = (gx1_c + gx2_c) * 0.5f;
  lpf_gy = (gy1_c + gy2_c) * 0.5f;
  lpf_gz = (gz1_c + gz2_c) * 0.5f;
```

- [ ] **Step 5: PID 루프의 고장 게이트 갱신**

다음 줄을 찾는다:

```cpp
    if (check_imu_frozen() || check_rc_timeout() || check_attitude() || safety_lock) {
```

다음으로 교체:

```cpp
    bool fatal = check_imu_frozen();
    fatal      = fatal || check_imu_disagree();
    if (fatal || check_rc_timeout() || check_attitude() || safety_lock) {
```

- [ ] **Step 6: udp_task의 `start` 명령에서 IMU별 고장 리셋**

찾을 부분:

```cpp
    else if (cmd == "start") {
      fault_rc = fault_imu = false;
```

다음으로 교체:

```cpp
    else if (cmd == "start") {
      fault_rc = fault_imu = fault_imu1 = fault_imu2 = false;
      frozenSince1 = frozenSince2 = disagreeSince = 0;
```

- [ ] **Step 7: 준비 메시지 갱신**

```cpp
  Serial.println("SYSTEM READY (Task 4: per-IMU fault detect + fusion fallback)");
```

- [ ] **Step 8: 컴파일**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/archive/legacy_flight/dual_imu_pid_pwm/
```

- [ ] **Step 9: 하드웨어 검증 (사용자)**

프로펠러를 장착하지 않은 상태에서 두 가지 테스트:

a) **두 IMU 모두 정상 동작**: 시리얼에 FAULT 메시지가 표시되지 않아야 하고, 텔레메트리 `fault_imu`는 0이어야 한다.

b) **한 IMU 분리 테스트**: 드론 전원을 끈 상태에서 IMU2(CS=9)를 분리한다. 전원을 켠다. 부팅 시 IMU2가 0 또는 노이즈를 읽은 채로 `[CALIB]`가 로그된다; IMU1은 정상이어야 한다. 부팅 후 시리얼에는 결국 IMU2에 대한 `[FAULT] IMU FROZEN` 형태의 메시지가 로그되어야 한다(또는 불일치 확인이 먼저 트리거될 수 있음). 텔레메트리 `fault_imu`는 1이 되지만 루프는 계속 IMU1 데이터를 `raw_*`에 공급한다. 드론은 시동되면 안 된다(불일치가 발동하면 safety_lock). 전원을 끄고 IMU2를 다시 연결한 뒤 반복한다: 다음 부팅에서 고장이 해제되는지 확인한다.

- [ ] **Step 10: 커밋**

```bash
git add firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino
git commit -m "feat: per-IMU fault detect with fallback and disagree check"
```

---

## Task 5: 적응형 상보 필터

**목표:** 고정된 `ALPHA_COMP = 0.995f`를 1G로부터의 가속도계 편차에 따라 0.99, 0.995, 또는 0.999를 반환하는 함수로 교체한다.

**파일:**
- 수정: `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino`

- [ ] **Step 1: `ALPHA_COMP` 상수를 3단계 상수 및 임계값 상수로 교체**

찾을 부분:

```cpp
const float ALPHA_COMP = 0.995f;  // temporary; replaced in Task 5
```

다음으로 교체:

```cpp
const float ACCEL_DEV_SOFT = 0.1f;   // G
const float ACCEL_DEV_HARD = 0.3f;   // G
const float ALPHA_STATIC   = 0.99f;
const float ALPHA_NORMAL   = 0.995f;
const float ALPHA_DYNAMIC  = 0.999f;

static inline float compute_alpha(float ax, float ay, float az) {
  float mag = sqrtf(ax*ax + ay*ay + az*az);
  float dev = fabsf(mag - 1.0f);
  if      (dev < ACCEL_DEV_SOFT) return ALPHA_STATIC;
  else if (dev < ACCEL_DEV_HARD) return ALPHA_NORMAL;
  else                            return ALPHA_DYNAMIC;
}
```

- [ ] **Step 2: 상보 필터에서 `compute_alpha()` 사용**

`pid_task`에서 `angleX`/`angleY`를 계산하는 줄들을 찾는다:

```cpp
    angleX = ALPHA_COMP * (angleX + lpf_gx * dt) + (1.0f - ALPHA_COMP) * accAngleX;
    angleY = ALPHA_COMP * (angleY + lpf_gy * dt) + (1.0f - ALPHA_COMP) * accAngleY;
```

다음으로 교체:

```cpp
    float alpha = compute_alpha(lpf_ax, lpf_ay, lpf_az);
    angleX = alpha * (angleX + lpf_gx * dt) + (1.0f - alpha) * accAngleX;
    angleY = alpha * (angleY + lpf_gy * dt) + (1.0f - alpha) * accAngleY;
```

- [ ] **Step 3: 준비 메시지 갱신**

```cpp
  Serial.println("SYSTEM READY (Task 5: adaptive complementary filter)");
```

- [ ] **Step 4: 컴파일**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/archive/legacy_flight/dual_imu_pid_pwm/
```

- [ ] **Step 5: 하드웨어 검증 (사용자)**

플래시한다. 드론이 정지 상태일 때 텔레메트리의 `angleX`/`angleY`는 1분 동안 Task 3보다 **드리프트가 적어야** 한다(alpha=0.99가 가속도 보정을 강화하기 때문). 드론을 손으로 세게 흔들 때 각도가 폭주하면 안 된다(alpha=0.999가 혼란 상황에서 자이로 우세로 만든다).

- [ ] **Step 6: 커밋**

```bash
git add firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino
git commit -m "feat: adaptive complementary filter based on accel magnitude"
```

---

## Task 6: 런타임 저속 바이어스 추정

**목표:** 드론이 유휴 상태(`base_throttle < 1100`)이면서 실제로 정지 상태(`|gyro| < 2 deg/s`)일 때, `gyro_bias1`과 `gyro_bias2`를 EMA α=0.0005로 천천히 갱신한다.

**파일:**
- 수정: `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino`

- [ ] **Step 1: 섹션 2에 런타임 바이어스 상수 추가**

불일치 상수 다음에:

```cpp
const float RUNTIME_BIAS_ALPHA      = 0.0005f;
const float RUNTIME_BIAS_GYRO_LIMIT = 2.0f;   // deg/s
```

- [ ] **Step 2: `pid_task` 상단에 헬퍼 추가 (`inv_imu_sensor_event_t e1, e2;` 선언 바로 다음)**

`pid_task` 내부, 지역 선언 다음이자 사전 루프 초기 읽기 이전에 추가한다:

```cpp
  auto maybe_update_bias = [&](float sgx, float sgy, float sgz,
                               float (&bias)[3]) {
    // sgx/sgy/sgz are sign-corrected pre-Y-flip gyro (drone frame except Y flip),
    // matching how bias was measured in setup. Bias is stored in the same frame.
    if (base_throttle >= 1100) return;
    if (fabsf(sgx - bias[0]) > RUNTIME_BIAS_GYRO_LIMIT) return;
    if (fabsf(sgy - bias[1]) > RUNTIME_BIAS_GYRO_LIMIT) return;
    if (fabsf(sgz - bias[2]) > RUNTIME_BIAS_GYRO_LIMIT) return;
    bias[0] = bias[0] * (1.0f - RUNTIME_BIAS_ALPHA) + sgx * RUNTIME_BIAS_ALPHA;
    bias[1] = bias[1] * (1.0f - RUNTIME_BIAS_ALPHA) + sgy * RUNTIME_BIAS_ALPHA;
    bias[2] = bias[2] * (1.0f - RUNTIME_BIAS_ALPHA) + sgz * RUNTIME_BIAS_ALPHA;
  };
```

- [ ] **Step 3: 루프 내부 IMU 읽기마다 헬퍼 호출**

`pid_task`에서 루프 내부 읽기 바로 다음에:

```cpp
    IMU1.getDataFromRegisters(e1);
    IMU2.getDataFromRegisters(e2);
```

바로 아래에 추가한다(값이 바이어스가 측정된 방식과 일치하도록 IMU2 부호 보정을 적용):

```cpp
    maybe_update_bias(              e1.gyro[0] * GYRO_SCALE,
                                    e1.gyro[1] * GYRO_SCALE,
                                    e1.gyro[2] * GYRO_SCALE,
                                    gyro_bias1);
    maybe_update_bias(IMU2_SIGN_X * e2.gyro[0] * GYRO_SCALE,
                      IMU2_SIGN_Y * e2.gyro[1] * GYRO_SCALE,
                      IMU2_SIGN_Z * e2.gyro[2] * GYRO_SCALE,
                      gyro_bias2);
```

- [ ] **Step 4: 준비 메시지 갱신**

```cpp
  Serial.println("SYSTEM READY (Task 6: runtime bias estimation)");
```

- [ ] **Step 5: 컴파일**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/archive/legacy_flight/dual_imu_pid_pwm/
```

- [ ] **Step 6: 하드웨어 검증 (사용자)**

플래시한다. 부팅 후 드론을 10분간 정지 상태로 둔다(유휴, 시동하지 말 것). 시동 시 바이어스는 여전히 적절한 범위에 있고, 이제 런타임 EMA가 이를 더 정밀하게 다듬는다. 육안으로 확인하려면 5초마다 바이어스 값을 `Serial.printf`로 임시 추가할 수 있다; 이는 선택 사항이다. 기능적 확인: 10분 동안의 `angleZ` 드리프트는 Task 5 단독보다 작아야 한다.

- [ ] **Step 7: 커밋**

```bash
git add firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino
git commit -m "feat: runtime slow gyro bias estimation while idle"
```

---

## Task 7: 최종 호버 비행 테스트 + 튜닝 검증

**목표:** 실제 드론에서 엔드투엔드 동작을 검증한다. 준비 메시지를 최종 형태로 갱신하고 통합을 확인한다. 메시지 외에 코드 변경은 없다.

**파일:**
- 수정: `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino`

- [ ] **Step 1: 준비 메시지를 최종 형태로 갱신**

```cpp
  Serial.println("PWM_TEST_DUAL_IMU_PID READY");
```

- [ ] **Step 2: 컴파일**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/archive/legacy_flight/dual_imu_pid_pwm/
```

- [ ] **Step 3: 하드웨어 검증 (사용자)** — 프로펠러 분리 상태 테스트 우선

a) 전원을 켜고, `[CALIB] OK`와 `PWM_TEST_DUAL_IMU_PID READY`를 확인한다.
b) `scripts/tune_pid.py`를 실행한다. 텔레메트리가 도착하고, `fault_imu=0`이며, 14개 필드가 모두 파싱되는지 확인한다.
c) 튜닝 스크립트로 `start`를 보낸다. 모터가 시동되고(base_throttle=1100), `fault_imu=0`인지 확인한다. `stop`을 보낸다. 시동 해제를 확인한다.
d) PID 튜닝 명령을 테스트한다: `pa 2.0`, 그다음 `pa 2.5`. 스크립트가 갱신된 게인을 보고하는지 확인한다.

- [ ] **Step 4: 하드웨어 검증 (사용자)** — 프로펠러 장착, 테더링 또는 안전한 구역에서

a) 호버링하며 이전의 한쪽으로 드리프트하는 동작이 재현되는지 관찰한다. 예상: 크게 감소.
b) 드리프트가 남아 있으면 텔레메트리 로그에서 `angleX`/`angleY`를 캡처해 보고한다 — alpha 임계값이나 PID 게인을 다시 검토해야 할 수 있다.

- [ ] **Step 5: 커밋**

```bash
git add firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino
git commit -m "chore: finalize PWM_TEST_DUAL_IMU_PID ready message"
```
