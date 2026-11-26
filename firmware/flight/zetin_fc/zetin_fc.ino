// zetin_fc — ZETIN 드론의 유일한 비행 펌웨어(ESP32-S3, 듀얼 ICM42670, BMM350,
// 3901-L0X, PWM ESC). 캐스케이드 PID 자세 제어에 3901-L0X 광류 보정·위치 유지·
// 고도 유지를 더했다. 2026-11-13 옛 주력 dual_imu_cascade_pwm 의 기능을 모두
// 포함하므로 그것은 firmware/archive/legacy_flight 로 보냈다.
// 광류 설계: docs/superpowers/plans/2026-10-29-flow-position-hold.md
// RC 상실 시 거리 기반 자동착륙(range_land.h): 거리가 신선하면 목표 고도를 낮추며
// 위치 유지를 켠 채 내려오고, 접촉을 확인하면 컷한다. 거리를 못 믿으면 기존과 같은
// 시간 기반 하강으로 폴백한다. 설계: docs/superpowers/specs/2026-11-03-range-autoland-design.md
// 실기 검증 전이라 기본 OFF 다(`rangeland 1`, 시동 해제 때만). OFF 면 RC 상실은 늘
// 시간 기반 하강이다.
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ICM42670P.h>
#include <DFRobot_BMM350.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <errno.h>
#include <float.h>

// 프로젝트 헤더: 하드웨어와 무관한 계산 로직만 담아 PC 네이티브 테스트에서도 그대로 쓴다
// 자기계 heading 계산 + yaw 상보필터
#include "logic/mag_yaw_fusion.h"
// yaw 바깥 루프: heading 잠금/스틱 각속도 모드
#include "logic/yaw_command.h"
// RC 끊김 시 자동착륙, 호버 스로틀 추정
#include "logic/failsafe_land.h"
// 3901-L0X 광류/라이다 UART 파서
#include "logic/msp_sensor.h"
// PID 출력 → 모터 4개 분배 (포화 시 우선순위)
#include "logic/control_allocator.h"
// 저스로틀 yaw 권한 부족 감시, 조건부 적분
#include "logic/yaw_authority.h"
#include "logic/flow_calibration.h"
#include "logic/flow_odometry.h"
#include "logic/flow_hold.h"
#include "logic/range_land.h"

#include "fc/wire_types.h"
#include "fc/config.h"
#include "fc/util.h"
#include "fc/hardware.h"
#include "fc/failsafe_state.h"
#include "fc/flow_state.h"
#include "fc/rangeland_state.h"
#include "fc/flow_requests.h"
#include "fc/flight_state.h"
#include "fc/safety_requests.h"
#include "fc/imu_snapshots.h"
#include "fc/magnetometer.h"
#include "fc/motor.h"
#include "fc/imu_calibration.h"
#include "fc/control_support.h"
#include "fc/pid_task.h"
#include "fc/udp_task.h"
// ==========================================================
// 10. setup / loop
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(SPI_CS1, OUTPUT); pinMode(SPI_CS2, OUTPUT);
  digitalWrite(SPI_CS1, HIGH); digitalWrite(SPI_CS2, HIGH);   // float CS 로 인한 버스 경합 방지
  delay(100);

  // 드론이 AP가 된다
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  // 절전 끔 → 패킷 지연 감소
  WiFi.setSleep(false);
  // 최대 송신 출력
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  udp.begin(UDP_PORT);

  // 3901-L0X 수신 전용 UART. TX 핀 -1 로 두어 핀 하나만 점유한다.
  Serial1.begin(MSP_SENSOR_BAUD, SERIAL_8N1, MSP_SENSOR_RX_PIN, -1);

  SPI.begin(12, 13, 11, -1);   // SCK, MISO, MOSI; CS는 각 IMU 객체가 관리

  // 모터 핀 4개에 400Hz/14비트 PWM 연결 → 바로 정지 신호(1000µs) 출력 (ESC가 시동 신호를 받아 대기 상태로)
  bool esc_ok = ledcAttach(pinM1, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM2, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM3, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM4, ESC_FREQ, ESC_RES);
  if (!esc_ok) while (1) { Serial.println("[FAULT] ESC attach FAIL"); delay(1000); }
  stopMotors();

  if (IMU1.begin() < 0) while (1) { Serial.println("[FAULT] IMU1 FAIL"); delay(1000); }
  if (IMU2.begin() < 0) while (1) { Serial.println("[FAULT] IMU2 FAIL"); delay(1000); }
  // IMU 설정: 출력 1600Hz, 가속도 ±16g, 자이로 ±2000dps (GYRO_SCALE/ACCEL_SCALE과 짝) + 하드웨어 LPF
  // 실패 코드를 |=로 모아 하나라도 실패하면 0이 아니게 된다
  // IMUx.begin(): starts communication with the ICM42670
  // 초기화 성공 -> return 0, 실패 -> return < 0
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

  // 부팅 때는 정지 보정을 하지 않는다(지상국 `imucal` 로만). 저장값이 없을 때만 예전처럼.
  // [zetin_fc] 지금은 부팅 때 저장된 IMU 보정값(loadImuCalibration)을 먼저 쓰고, 없을 때만 calibrate_bias()를 돌린다. 아래 두 노트는 그 경우의 이야기
  // [공중에서 재부팅된 경우] 움직임이 감지되어 3번 모두 실패하기 쉽고, 실패하면 calibration_ok=false라 start가 거부된다.
  // pid_task는 이 뒤에 생성되므로 그동안(최소 4.5초) 자세 추정도 제어도 돌지 않는다.
  // 시도 도중 바닥에 멈추면 통과할 수 있으므로, 추락 후에는 기체를 점검하고 전원을 다시 넣는다
  // 약 4초 이상 (IMU 2개 × 2초). 이때 기체를 움직이면 안 된다
  calibration_ok = loadImuCalibration() || calibrate_bias();
  resetFlowHoldForHarness();   // 컴파일 보정값 적용 포함

  // pid_task 전용 태스크 워치독. idle task는 감시하지 않고, timeout 시
  // panic 재부팅으로 마지막 PWM 고정 상태에서 벗어난다.
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = PID_WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK &&
      esp_task_wdt_init(&wdt_config) != ESP_OK) {
    while (1) { Serial.println("[FAULT] TASK WDT INIT FAIL"); delay(1000); }
  }

  // FreeRTOS 태스크를 코어에 고정해 생성: (함수, 이름, 스택 4096B, 인자, 우선순위, 핸들, 코어 번호)
  //  PID = 우선순위 2, Core 1 / UDP = 우선순위 1, Core 0 (WiFi 스택도 Core 0에서 돈다)
  BaseType_t pidTaskOk = xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 2, NULL, 1);
  // 텔레메트리 printf 인자가 97개라 기존 4096 B 에서 늘린다.
  BaseType_t udpTaskOk = xTaskCreatePinnedToCore(udp_task, "UDP", 6144, NULL, 1, NULL, 0);
  if (pidTaskOk != pdPASS || udpTaskOk != pdPASS) {
    safety_lock = true;
    while (1) { Serial.println("[FAULT] TASK CREATE FAIL"); delay(1000); }
  }

  Serial.println("DUAL_IMU_CASCADE_FLOW_HOLD READY");
}

// Arduino 기본 loop(): 실제 일은 위 두 태스크가 하고, 여기서는 지연 디버그 출력만 (디버그 꺼지면 1초씩 잠만 잔다)
void loop() {
#if WIFI_LATENCY_DEBUG
  drainLatRing(udpLatRing);
  drainLatRing(pidLatRing);

  uint32_t nowMs = millis();
  if (nowMs - latLastSummaryMs >= 2000U) {
    latLastSummaryMs = nowMs;
    uint32_t dropped = __atomic_load_n(&udpLatRing.dropped, __ATOMIC_RELAXED)
                     + __atomic_load_n(&pidLatRing.dropped, __ATOMIC_RELAXED);
    Serial.printf("[LATSUM] up=%lu udpMaxUs=%lu pidMaxUs=%lu "
                  "udpLoopOver30=%lu udpLoopOver100=%lu udpLoopOver300=%lu "
                  "pidOver5=%lu dropped=%lu\n",
                  (unsigned long)nowMs,
                  (unsigned long)latUdpMaxUs, (unsigned long)latPidMaxUs,
                  (unsigned long)latUdpLoopOver30,
                  (unsigned long)latUdpLoopOver100,
                  (unsigned long)latUdpLoopOver300,
                  (unsigned long)latPidOver5, (unsigned long)dropped);
  }
  delay(100);
#else
  // UDP 객체는 udp_task 한 곳에서만 접근해 cross-core race를 피한다.
  delay(1000);
#endif
}
