#pragma once

#include <Arduino.h>

#include "mag_yaw_fusion.h"   // wrapDeg

// [static inline] 이 헤더는 함수 본문까지 담고 있고 .ino와 네이티브 테스트 양쪽에서 include된다.
//  - inline: 같은 정의가 여러 파일에 들어가도 링크 에러(multiple definition)가 안 나게 하고,
//            작은 함수를 호출 자리에 펼치도록 유도한다(1kHz pid_task 호출 비용 감소). 단 펼칠지는 컴파일러가 결정
//  - static: include한 파일 안에서만 보이게 해 파일 간 이름 충돌을 막는다

// 바깥 yaw 루프의 모드와 heading setpoint.
// hold=false 인 동안 setpoint를 현재 heading으로 계속 슬레이빙하므로
// setpoint가 stale해질 수 없고, 어떤 전환도 정의상 bumpless다.
struct YawOuter {
  float target_angle_deg;
  bool  hold;
};

static inline YawOuter updateYawOuter(
    float target_yaw_rate_dps, float body_gz_dps, float angle_z_deg,
    float prev_target_angle_deg, bool prev_hold, bool override_hold,
    float rate_deadzone_dps, float settle_dps)
    // target_yaw_rate_dps   : 스틱(rcr)이 요구하는 요 각속도 (°/s). 원하는 회전 속도
    // body_gz_dps           : 자이로로 측정한 실제 요 각속도 (°/s). 회전이 멈췄는지(settled) 판단용
    // angle_z_deg           : 현재 추정 heading (°). 잠금이 아니면 목표를 이 값으로 슬레이빙
    // prev_target_angle_deg : 직전 tick의 목표 heading (°). 잠금 중이면 그대로 유지
    // prev_hold             : 직전 tick의 잠금 여부. 한번 잠기면 스틱 중립인 동안 계속 유지하는 데 사용
    // override_hold         : "yaw 1" 오버라이드(yaw_hold_override). true면 무조건 잠금
    // rate_deadzone_dps     : |스틱 각속도|가 이보다 작으면 스틱 중립으로 본다 (YAW_RATE_DEADZONE)
    // settle_dps            : |실제 각속도|가 이보다 작아야 새로 잠글 수 있다 (YAW_HOLD_SETTLE_DPS)
{
  // fabsf: float 값의 절댓값을 구하는 C 표준 수학 함수(<math.h>). Arduino 환경에서는 따로 include하지 않아도 쓸 수 있다
  const bool stick_centered = fabsf(target_yaw_rate_dps) < rate_deadzone_dps; // 스틱이 중립인지 판단. 스틱이 중립이면 잠금 유지/새로 잠금, 아니면 무조건 풀기
  const bool settled = fabsf(body_gz_dps) < settle_dps; // 실제 회전이 멈췄는지 판단. 새로 잠글 때만 사용. 이미 잠긴 상태에서는 무시
  YawOuter out;
  // 스틱을 밀면 항상 rate 모드로 풀고, 중립에서는 기존 잠금을 유지하거나
  // 정착한 뒤 새로 잠근다. settled는 진입 게이트일 뿐 해제 조건이 아니다.
  out.hold =
      override_hold || (stick_centered && (prev_hold || settled)); 
      // override_hold가 true면 무조건 잠금, 아니면 스틱 중립이면서 직전 잠금이거나 정착한 경우에만 잠금
      // prev_hold가 필요한 이유 (없이 stick_centered && settled 만 쓰면):
      //  1) 잠긴 뒤 돌풍으로 10dps 넘게 밀리면 settled=false → 잠금 해제 → 슬레이빙으로 밀려난 방위를
      //     새 목표로 받아들여 원래 방위로 복원하지 않는다
      //  2) 회전 속도가 settle_dps 경계(9~11dps)를 오가면 잠금/해제가 tick마다 깜빡이고(채터링),
      //     풀릴 때마다 목표가 현재값으로 리셋돼 잠금이 제 역할을 못 한다
      // → 처음 잠글 때만 settled를 확인하고, 이후엔 prev_hold가 잠금을 유지. 해제는 스틱을 밀 때뿐
  out.target_angle_deg = out.hold ? prev_target_angle_deg : angle_z_deg; // 잠금 중이면 직전 목표를 그대로 유지, 아니면 현재 heading으로 슬레이빙
  return out;
}

static inline float yawTargetRateDps(
    const YawOuter &state, float target_yaw_rate_dps, float angle_z_deg,
    float kp_angle_yaw, float max_rate_dps)
    // state               : 이번 tick updateYawOuter()의 결과(hold, target_angle_deg). const 참조라 복사 없이 읽기만 한다
    // target_yaw_rate_dps : 스틱(rcr)이 요구하는 요 각속도 (°/s). 잠금이 아니면 이 값을 그대로 출력
    // angle_z_deg         : 현재 추정 heading (°). 잠금이면 목표와의 차이(오차) 계산에 사용
    // kp_angle_yaw        : 잠금 시 각도 오차 → 목표 각속도 비례 게인 (Kp_Angle_Yaw)
    // max_rate_dps        : 출력 요 목표 각속도 상한 (°/s). ±이 값으로 클램프 (MAX_TARGET_RATE_YAW)
{
  // raw: 제한(클램프)을 걸기 전의 요 목표 각속도 (°/s)
  //  - 잠금(hold): (목표 heading - 현재 heading) × Kp → 밀려난 만큼 되돌리는 복원 명령
  //    wrapDeg로 오차를 ±180°로 감싸 짧은 방향으로 돈다 (예: 목표 10°, 현재 350° → +20°, -340° 아님)
  //  - 해제: 스틱 각속도를 그대로 사용
  // 아래에서 ±max_rate_dps로 잘라낸 값이 최종 출력(targetRateYaw)이 된다
  //
  // [잠금 복원은 P 제어만 사용] 오차에 비례해 되돌림: 5° → 15dps, 1° → 3dps (Kp=3), 60° 이상 → 180dps 클램프
  //  출력은 모터가 아니라 목표 각속도 → 안쪽 각속도 PID(Kp 1.5 / Ki 0.05 / Kd 0)가 모터 출력으로 바꾼다
  //  바깥 루프에 I항이 없어도 정상상태 오차가 남지 않는 이유: 정지 상태면 실제 각속도 = 0이고,
  //  안쪽 I항은 각속도 오차가 0이 될 때까지 쌓이므로 목표 각속도(= 각도 오차 × Kp)도 0 → 각도 오차도 0.
  //  즉 일정한 외란을 버티는 힘은 안쪽 I항(Ki_Rate_Yaw)이 만든다. 단 I항이 ±50µs 한도에 닿으면 오차가 남을 수 있다
  const float raw = state.hold
      ? wrapDeg(state.target_angle_deg - angle_z_deg) * kp_angle_yaw
      : target_yaw_rate_dps;
  if (raw > max_rate_dps) return max_rate_dps; // raw값이 max_rate_dps보다 크면 max_rate_dps로 클램프
  if (raw < -max_rate_dps) return -max_rate_dps;  // raw값이 -max_rate_dps보다 작으면 -max_rate_dps로 클램프
  return raw;
}
