#pragma once

#include <Arduino.h>

// [자기계 yaw 융합 정리]
//  자이로만 적분한 yaw(angleZ)는 bias 때문에 시간이 갈수록 조금씩 틀어진다(드리프트)
//  자기계(나침반)는 드리프트는 없지만 노이즈·간섭이 크다 → 둘을 섞는다(상보필터)
//  순서: 기울기 보정한 나침반 heading 계산 → angleZ와의 차이에 아주 작은 gain(K_MAG)을 곱해 조금씩 끌어당김
//  자이로가 단기 반응을, 나침반이 장기 기준을 맡는다

// Header placement keeps Arduino's generated .ino prototypes type-complete.
// Core 0(udp_task)이 읽은 자기계 값을 Core 1(pid_task)에 넘기는 한 묶음. 보정(hard/soft-iron, 축 변환) 후 값
struct MagSnapshot {
  float x;              // 자기장 x (µT, body 축)
  float y;              // 자기장 y (µT)
  float z;              // 자기장 z (µT)
  uint32_t sample_ms;   // 측정 시각 (millis). 오래된 값(MAG_STALE_MS 초과)을 거르는 데 쓴다
};

// 각도를 -180° 이상 +180° 미만으로 감싼다. 두 heading의 차이(오차)를 구할 때 짧은 방향으로 돌게 하려고 쓴다
static inline float wrapDeg(float angle_deg) {
  float wrapped = fmodf(angle_deg + 180.0f, 360.0f);
  if (wrapped < 0.0f) wrapped += 360.0f;   // fmodf는 음수 입력에 음수를 돌려주므로 0~360으로 맞춘다
  return wrapped - 180.0f;
}

// [기울기 보정 heading] 기체가 기울면 자기장 벡터를 그대로 atan2 하면 heading이 틀린다
//  roll/pitch만큼 자기장 벡터를 거꾸로 돌려 "수평면에 투영한" x, y 성분을 구한 뒤 방향각을 계산한다
//  mag_x/y/z           : 보정된 자기장 (µT, body 축)
//  roll_deg, pitch_deg : 현재 자세 추정값 (angleX, angleY)
//  반환                : 자북 기준 heading (°, -180~180)
static inline float tiltCompensatedMagHeadingDeg(
    float mag_x, float mag_y, float mag_z,
    float roll_deg, float pitch_deg) {
  const float roll = roll_deg * PI / 180.0f;     // 도 → 라디안 (sinf/cosf는 라디안을 받는다)
  const float pitch = pitch_deg * PI / 180.0f;
  const float sin_roll = sinf(roll);
  const float cos_roll = cosf(roll);
  const float sin_pitch = sinf(pitch);
  const float cos_pitch = cosf(pitch);

  // 수평면 성분 (기체가 수평이면 roll=pitch=0 → horizontal_x = mag_x, horizontal_y = mag_y)
  const float horizontal_x =
      mag_x * cos_pitch
      + mag_y * sin_roll * sin_pitch
      + mag_z * cos_roll * sin_pitch;
  const float horizontal_y =
      mag_y * cos_roll - mag_z * sin_roll;
  // atan2(-Yh, Xh): 북쪽을 0°, 시계방향을 +로 만드는 표준 나침반 공식. 결과를 라디안 → 도로 변환
  return atan2f(-horizontal_y, horizontal_x) * 180.0f / PI;
}

// [상보필터 보정량] angleZ에 더할 보정량(°)을 돌려준다
//  (나침반 heading - 현재 yaw)를 ±180°로 감싼 오차 × gain
//  gain = K_MAG(0.001), 250Hz로 호출 → 오차를 매번 0.1%씩만 줄임 → 시정수 약 4초
//  나침반 노이즈는 거의 반영되지 않고, 자이로 드리프트처럼 천천히 쌓이는 오차만 잡힌다
static inline float magYawCorrectionDeg(
    float yaw_deg, float mag_heading_deg, float gain) {
  return gain * wrapDeg(mag_heading_deg - yaw_deg);
}
