#pragma once

#include <Arduino.h>

// Header placement keeps Arduino's generated .ino prototypes type-complete.
struct MagSnapshot {
  float x;
  float y;
  float z;
  uint32_t sample_ms;
};

static inline float wrapDeg(float angle_deg) {
  float wrapped = fmodf(angle_deg + 180.0f, 360.0f);
  if (wrapped < 0.0f) wrapped += 360.0f;
  return wrapped - 180.0f;
}

static inline float tiltCompensatedMagHeadingDeg(
    float mag_x, float mag_y, float mag_z,
    float roll_deg, float pitch_deg) {
  const float roll = roll_deg * PI / 180.0f;
  const float pitch = pitch_deg * PI / 180.0f;
  const float sin_roll = sinf(roll);
  const float cos_roll = cosf(roll);
  const float sin_pitch = sinf(pitch);
  const float cos_pitch = cosf(pitch);

  const float horizontal_x =
      mag_x * cos_pitch
      + mag_y * sin_roll * sin_pitch
      + mag_z * cos_roll * sin_pitch;
  const float horizontal_y =
      mag_y * cos_roll - mag_z * sin_roll;
  return atan2f(-horizontal_y, horizontal_x) * 180.0f / PI;
}

static inline float magYawCorrectionDeg(
    float yaw_deg, float mag_heading_deg, float gain) {
  return gain * wrapDeg(mag_heading_deg - yaw_deg);
}
