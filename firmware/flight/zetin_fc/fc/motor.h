// zetin_fc/fc/motor.h — 5. 모터 출력.
// zetin_fc.ino 가 정해진 순서로 한 번만 include 한다(단일 번역 단위). 앞 헤더들에 정의된
// 것을 그대로 쓰므로 단독으로 include 하지 않는다. 순서와 모듈 지도는 zetin_fc/README.md.
#pragma once
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

// Pure allocator keeps this compatibility entrypoint for sketch callers and native
// integration tests. It assigns roll/pitch first, then fits yaw in the residual span.
static MotorMix mixAndDesaturate(float roll, float pitch, float yaw,
                                 int throttle, int minMotor, int maxMotor) {
  return allocateControl(roll, pitch, yaw, throttle, minMotor, maxMotor);
}

