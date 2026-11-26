#pragma once

#include <cmath>

// [모터 분배기(control allocator) 정리]
//  PID가 낸 roll/pitch/yaw 보정량(µs)과 스로틀(collective)을 모터 4개의 PWM 펄스 폭(µs)으로 바꾼다
//  모터마다 [minMotor, maxMotor] 범위를 넘을 수 없어서, 보정량이 너무 크면 무엇을 먼저 지킬지 정해야 한다
//  우선순위: roll/pitch(자세 유지) > yaw(방향) > collective(스로틀, 고도)
//   1) 다 들어가면: 예전 믹서와 완전히 같은 결과 (float 계산 + lroundf 반올림)
//   2) 안 들어가면: roll/pitch를 먼저 범위 안에 넣고(필요하면 축소), 남은 여유만큼 yaw를 넣고(λ 배로 축소),
//      마지막으로 스로틀을 위아래로 밀어서 4개 모두 범위 안에 들어오게 한다
//  저스로틀에서는 [min, max] 폭이 좁아 yaw가 들어갈 자리가 없다 → yaw_scale이 작아지고 yaw_authority.h가 이를 감시한다
//  하드웨어 의존 코드가 없어서 PC(네이티브 테스트)에서도 그대로 컴파일해 시험한다

// Hardware-independent motor allocation result. Motor order is FL, RR, FR, RL.
struct ControlAllocation {
  int motor[4];          // 최종 모터 출력 (µs). 순서 FL, RR, FR, RL (핀 pinM1~pinM4와 같은 순서)
  float collective_us;   // 실제 적용된 스로틀(µs). 범위를 맞추느라 요청 throttle에서 밀려났을 수 있다
  float rp_scale;        // roll/pitch 보정량에 곱한 배율 (1 = 그대로, <1 = 범위가 모자라 축소)
  float yaw_scale;       // yaw 보정량에 곱한 배율 λ (1 = 그대로, 0 = yaw 완전히 포기)
  bool scaled;           // rp_scale 또는 yaw_scale이 1보다 작으면 true (텔레메트리 "포화" 표시)
};

// value를 [low, high]로 자른다 (정수 클램프)
static inline int controlAllocatorClampInt(int value, int low, int high) {
  return value < low ? low : (value > high ? high : value);
}

// NaN/무한대가 들어오면 0으로 바꾼다. PID가 고장 나도 모터에 이상한 값이 나가지 않게 하는 방어
static inline float controlAllocatorFinite(float value) {
  return std::isfinite(value) ? value : 0.0f;
}

static inline ControlAllocation allocateControl(float roll, float pitch, float yaw,
                                                 int throttle, int minMotor,
                                                 int maxMotor) {
  // roll, pitch, yaw : 각 축 PID 출력 (µs). 모터 간 차이를 만드는 양
  // throttle         : 요청 스로틀 = 4개 모터의 공통 출력 (µs, base_throttle)
  // minMotor/maxMotor: 각 모터 출력 하한/상한 (µs, min_throttle/max_throttle)
  ControlAllocation out = {};
  // 입력 범위 방어: 1000~2000µs 밖은 ESC 신호로 의미가 없고, max < min이 되지 않게 한다
  minMotor = controlAllocatorClampInt(minMotor, 1000, 2000);
  maxMotor = controlAllocatorClampInt(maxMotor, minMotor, 2000);

  const float rollValueFloat = controlAllocatorFinite(roll);
  const float pitchValueFloat = controlAllocatorFinite(pitch);
  const float yawValueFloat = controlAllocatorFinite(yaw);
  const float availableFloat = static_cast<float>(maxMotor - minMotor);  // 모터 간 차이로 쓸 수 있는 최대 폭 (µs)
  // [믹서 부호] 순서 FL, RR, FR, RL
  //  roll>0 : 왼쪽(FL, RL) 올리고 오른쪽(RR, FR) 내림
  //  pitch>0: 뒤쪽(RR, RL) 올리고 앞쪽(FL, FR) 내림
  //  yaw>0  : FR, RL 올리고 FL, RR 내림 (대각선 쌍끼리 같은 방향으로 돈다 → 반토크 차이로 요 회전)
  const float rpFloat[4] = {   // roll/pitch만 섞은 모터별 차이
      -pitchValueFloat + rollValueFloat,
      pitchValueFloat - rollValueFloat,
      -pitchValueFloat - rollValueFloat,
      pitchValueFloat + rollValueFloat,
  };
  const float legacyDiff[4] = {   // roll/pitch/yaw를 모두 섞은 모터별 차이 (예전 믹서와 같은 식)
      -pitchValueFloat + rollValueFloat - yawValueFloat,
      pitchValueFloat - rollValueFloat - yawValueFloat,
      -pitchValueFloat - rollValueFloat + yawValueFloat,
      pitchValueFloat + rollValueFloat + yawValueFloat,
  };
  // 각 차이의 최솟값/최댓값을 구한다 → (최대 - 최소) = 필요한 폭(span)
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

  // Keep the exact legacy float composition and lroundf behavior whenever RP
  // and the complete mix both fit. Saturated inputs continue to the priority
  // solver below, which uses double intermediates to keep finite extremes safe.
  // [1) 포화 없음] 필요한 폭이 가용 폭 안이면 축소 없이 그대로 쓴다 (예전 믹서와 비트 단위로 같은 결과)
  const float rpFloatSpan = rpFloatMax - rpFloatMin;
  const float legacySpan = legacyMax - legacyMin;
  if (std::isfinite(rpFloatSpan) && std::isfinite(legacySpan) &&
      rpFloatSpan <= availableFloat && legacySpan <= availableFloat) {
    // 스로틀만 위아래로 밀어서 가장 낮은 모터 ≥ minMotor, 가장 높은 모터 ≤ maxMotor가 되게 한다
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
      out.motor[index] = controlAllocatorClampInt(rounded, minMotor, maxMotor);  // 반올림 오차 대비 마지막 클램프
    }
    return out;
  }

  // [2) 포화] 여기부터는 우선순위 분배. 극단값에서 오버플로/정밀도 손실을 피하려고 double로 계산한다
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

  // 2-1) roll/pitch 우선: roll/pitch만으로도 폭을 넘으면 같은 비율로 줄여 딱 맞춘다 (방향은 유지, 크기만 축소)
  const double rpSpan = rpMax - rpMin;
  double rpScale = 1.0;
  if (rpSpan > available && rpSpan > 0.0) rpScale = available / rpSpan;
  for (int index = 0; index < 4; ++index) rp[index] *= rpScale;

  // λ is the largest yaw fraction for which every motor-pair span remains
  // inside the residual range after roll/pitch has claimed its priority.
  // 2-2) 남은 여유로 yaw: 모든 모터 쌍 (i, j)에 대해
  //   -available ≤ (rp_i - rp_j) + λ·(yaw_i - yaw_j) ≤ available
  //  을 만족하는 가장 큰 λ(0~1)를 찾는다. 모든 쌍의 차이가 폭 안이면 (최대 - 최소)도 폭 안이다
  //  각 쌍의 부등식을 λ에 대해 풀면 두 경계(boundA, boundB)가 나오고, 그중 큰 쪽이 λ의 상한이다
  double yawScale = 1.0;
  for (int first = 0; first < 4; ++first) {
    for (int second = first + 1; second < 4; ++second) {
      const double rpDifference = rp[first] - rp[second];
      const double yawDifference = yawVector[first] - yawVector[second];
      if (yawDifference == 0.0) continue;  // 같은 회전 방향 쌍은 yaw가 차이를 만들지 않으므로 제약 없음

      const double boundA = (-available - rpDifference) / yawDifference;
      const double boundB = (available - rpDifference) / yawDifference;
      const double upper = boundA > boundB ? boundA : boundB;
      if (upper < yawScale) yawScale = upper;  // 가장 빡빡한 쌍이 λ를 결정한다
    }
  }
  if (yawScale < 0.0) yawScale = 0.0;
  if (yawScale > 1.0) yawScale = 1.0;

  // 축소된 roll/pitch + λ배 yaw = 최종 모터별 차이
  double diff[4] = {};
  double minDiff = 0.0;
  double maxDiff = 0.0;
  for (int index = 0; index < 4; ++index) {
    diff[index] = rp[index] + yawScale * yawVector[index];
    if (index == 0 || diff[index] < minDiff) minDiff = diff[index];
    if (index == 0 || diff[index] > maxDiff) maxDiff = diff[index];
  }

  // 2-3) 스로틀은 마지막: 차이는 그대로 두고 공통값만 밀어서 범위 안에 넣는다 (고도보다 자세가 우선)
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
