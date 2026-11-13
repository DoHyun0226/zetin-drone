#pragma once

#include <cmath>

// 하드웨어와 무관한 모터 배분 결과. 모터 순서는 FL, RR, FR, RL.
struct ControlAllocation {
  int motor[4];
  float collective_us;
  float rp_scale;
  float yaw_scale;
  bool scaled;
};

static inline int controlAllocatorClampInt(int value, int low, int high) {
  return value < low ? low : (value > high ? high : value);
}

static inline float controlAllocatorFinite(float value) {
  return std::isfinite(value) ? value : 0.0f;
}

static inline ControlAllocation allocateControl(float roll, float pitch, float yaw,
                                                 int throttle, int minMotor,
                                                 int maxMotor) {
  ControlAllocation out = {};
  minMotor = controlAllocatorClampInt(minMotor, 1000, 2000);
  maxMotor = controlAllocatorClampInt(maxMotor, minMotor, 2000);

  const float rollValueFloat = controlAllocatorFinite(roll);
  const float pitchValueFloat = controlAllocatorFinite(pitch);
  const float yawValueFloat = controlAllocatorFinite(yaw);
  const float availableFloat = static_cast<float>(maxMotor - minMotor);
  const float rpFloat[4] = {
      -pitchValueFloat + rollValueFloat,
      pitchValueFloat - rollValueFloat,
      -pitchValueFloat - rollValueFloat,
      pitchValueFloat + rollValueFloat,
  };
  const float legacyDiff[4] = {
      -pitchValueFloat + rollValueFloat - yawValueFloat,
      pitchValueFloat - rollValueFloat - yawValueFloat,
      -pitchValueFloat - rollValueFloat + yawValueFloat,
      pitchValueFloat + rollValueFloat + yawValueFloat,
  };
  float rpFloatMin = rpFloat[0];
  float rpFloatMax = rpFloat[0];
  float legacyMin = legacyDiff[0];
  float legacyMax = legacyDiff[0];
  for (int index = 1; index < 4; ++index) {
    if (rpFloat[index] < rpFloatMin) rpFloatMin = rpFloat[index];
    if (rpFloat[index] > rpFloatMax) rpFloatMax = rpFloat[index];
    if (legacyDiff[index] < legacyMin) legacyMin = legacyDiff[index];
    if (legacyDiff[index] > legacyMax) legacyMax = legacyDiff[index];
  }

  // RP 와 전체 혼합이 모두 범위 안이면 예전과 똑같은 float 계산과 lroundf 동작을 유지한다.
  // 포화된 입력은 아래 우선순위 풀이로 넘어가며, 그쪽은 유한한 극단값에서도 안전하도록
  // double 중간값을 쓴다.
  const float rpFloatSpan = rpFloatMax - rpFloatMin;
  const float legacySpan = legacyMax - legacyMin;
  if (std::isfinite(rpFloatSpan) && std::isfinite(legacySpan) &&
      rpFloatSpan <= availableFloat && legacySpan <= availableFloat) {
    const float collectiveLo = static_cast<float>(minMotor) - legacyMin;
    const float collectiveHi = static_cast<float>(maxMotor) - legacyMax;
    float collective = static_cast<float>(throttle);
    if (collective < collectiveLo) collective = collectiveLo;
    if (collective > collectiveHi) collective = collectiveHi;

    out.collective_us = collective;
    out.rp_scale = 1.0f;
    out.yaw_scale = 1.0f;
    out.scaled = false;
    for (int index = 0; index < 4; ++index) {
      const int rounded = static_cast<int>(std::lroundf(collective + legacyDiff[index]));
      out.motor[index] = controlAllocatorClampInt(rounded, minMotor, maxMotor);
    }
    return out;
  }

  const double rollValue = rollValueFloat;
  const double pitchValue = pitchValueFloat;
  const double yawValue = yawValueFloat;
  const double available = static_cast<double>(maxMotor - minMotor);

  double rp[4] = {
      -pitchValue + rollValue,
      pitchValue - rollValue,
      -pitchValue - rollValue,
      pitchValue + rollValue,
  };
  const double yawVector[4] = {
      -yawValue, -yawValue, yawValue, yawValue,
  };

  double rpMin = rp[0];
  double rpMax = rp[0];
  for (int index = 1; index < 4; ++index) {
    if (rp[index] < rpMin) rpMin = rp[index];
    if (rp[index] > rpMax) rpMax = rp[index];
  }

  const double rpSpan = rpMax - rpMin;
  double rpScale = 1.0;
  if (rpSpan > available && rpSpan > 0.0) rpScale = available / rpSpan;
  for (int index = 0; index < 4; ++index) rp[index] *= rpScale;

  // λ 는 roll/pitch 가 먼저 몫을 가져간 뒤에도 모든 모터 쌍의 폭이 남은 범위 안에 머무는
  // 가장 큰 yaw 비율이다.
  double yawScale = 1.0;
  for (int first = 0; first < 4; ++first) {
    for (int second = first + 1; second < 4; ++second) {
      const double rpDifference = rp[first] - rp[second];
      const double yawDifference = yawVector[first] - yawVector[second];
      if (yawDifference == 0.0) continue;

      const double boundA = (-available - rpDifference) / yawDifference;
      const double boundB = (available - rpDifference) / yawDifference;
      const double upper = boundA > boundB ? boundA : boundB;
      if (upper < yawScale) yawScale = upper;
    }
  }
  if (yawScale < 0.0) yawScale = 0.0;
  if (yawScale > 1.0) yawScale = 1.0;

  double diff[4] = {};
  double minDiff = 0.0;
  double maxDiff = 0.0;
  for (int index = 0; index < 4; ++index) {
    diff[index] = rp[index] + yawScale * yawVector[index];
    if (index == 0 || diff[index] < minDiff) minDiff = diff[index];
    if (index == 0 || diff[index] > maxDiff) maxDiff = diff[index];
  }

  const double collectiveLo = static_cast<double>(minMotor) - minDiff;
  const double collectiveHi = static_cast<double>(maxMotor) - maxDiff;
  double collective = static_cast<double>(throttle);
  if (collective < collectiveLo) collective = collectiveLo;
  if (collective > collectiveHi) collective = collectiveHi;

  out.collective_us = static_cast<float>(collective);
  out.rp_scale = static_cast<float>(rpScale);
  out.yaw_scale = static_cast<float>(yawScale);
  out.scaled = rpScale < 1.0 || yawScale < 1.0;
  for (int index = 0; index < 4; ++index) {
    const int rounded = static_cast<int>(std::lround(collective + diff[index]));
    out.motor[index] = controlAllocatorClampInt(rounded, minMotor, maxMotor);
  }
  return out;
}
