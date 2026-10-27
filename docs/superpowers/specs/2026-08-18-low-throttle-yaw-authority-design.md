# 저스로틀 수동 착륙 yaw 권한 관리 설계

작성일: 2026-08-18
대상: `firmware/flight/dual_imu_cascade_pwm`, `scripts/`, `tools/`

## 1. 목적과 관측 근거

고정된 테더를 사용한 최근 수동 비행에서 1300 µs 호버는 대체로 안정했지만,
착륙 말단의 1270 µs 부근에서 yaw 외란이 제어 권한을 넘었다. 기체가 실제로는
양의 yaw로 가속하는 동안 제어기는 음의 yaw를 요구했고 목표 yaw 각속도는
−180°/s에 닿았다. 기존 믹서는 roll, pitch, yaw 차동을 한 비율로 축소하므로 yaw
요구가 커질수록 수평 자세를 복원할 roll·pitch 권한도 함께 줄었다. 마지막에는
pitch가 약 +27°까지 증가했다.

RC 링크, 1 kHz PID 루프, 두 IMU의 일치, 자기계 데이터에는 이 사건을 설명할
고장이 없었다. 그러므로 이번 변경은 센서 고장이나 yaw 부호 반전을 전제로 하지
않는다. 테더 위치·무게·장력은 변경 불가능한 고정 플랜트 외란으로 취급한다.

이번 단계의 성공은 절대 heading 유지가 아니다. 사용할 수 있는 yaw 토크를 넘은
외란에서 heading을 양보하더라도 roll·pitch 제어 권한을 보존하여 수동 착륙 중
전복으로 이어지는 자세 붕괴를 막는 것이다.

## 2. 범위

### 포함

- 포화되지 않은 영역의 기존 모터 출력을 보존하는 축 우선순위 allocator
- roll·pitch 우선, 남은 모터 범위에 yaw를 배정하는 desaturation
- 실제 전달 권한에 따른 축별 conditional-integration anti-windup
- 수동 비행 중 지속적인 yaw 권한 부족을 다루는 상태기
- allocator, PID, 적분기, yaw 권한 상태의 append-only 텔레메트리
- 저스로틀 고정 외란 SIL 회귀 시험
- 지상국 표시, CSV 스키마, 오프라인 로그 요약

### 제외

- PID 게인 변경
- `CTRL_MARGIN`, 모터 PWM 하한·상한 변경
- 동적 idle 또는 PWM–추력 선형화
- optical-flow, range 또는 고도 기반 착륙 감지
- 테더 위치·무게·장력 변경
- 위치 유지
- failsafe 시간, 하강 스로틀, 컷 조건 변경
- 보드 업로드와 실기 비행 자체

## 3. Control allocator

### 3.1 인터페이스

`control_allocator.h`에 Arduino 하드웨어에 의존하지 않는 순수 함수를 둔다.

입력:

- roll, pitch, yaw PID 출력: 모터 PWM 차동 기여 µs
- collective 요청: µs
- 모터 출력 하한·상한: µs

출력:

- 모터 4개 정수 PWM
- 실제 적용 collective
- `rp_scale`: roll·pitch 합성 차동 전달 비율
- `yaw_scale`: yaw 차동 전달 비율
- `scaled`: 두 scale 중 하나라도 1보다 작은지

### 3.2 계산 순서

모터 순서와 부호는 기존 계약을 유지한다.

```text
RP = {
  -pitch + roll,
   pitch - roll,
  -pitch - roll,
   pitch + roll
}

Yaw = {-yaw, -yaw, +yaw, +yaw}
```

1. 하한·상한을 기존과 같은 안전 범위로 정규화한다.
2. RP 벡터의 span이 사용 가능한 모터 범위보다 클 때만 RP 전체를 한 비율로
   줄인다. roll과 pitch 사이의 토크 비율은 보존한다.
3. `scaled_RP + λ·Yaw`의 span이 범위 안에 드는 가장 큰 `λ∈[0,1]`을 구한다.
   이 값이 `yaw_scale`이다.
4. 합성 차동을 보존하도록 가능한 collective 구간을 계산하고, 그 안으로 요청
   collective를 이동한다.
5. 기존과 같은 반올림과 constrain을 거쳐 모터 출력을 만든다.

포화되지 않은 모든 입력에서는 `rp_scale=1`, `yaw_scale=1`이고 기존
`mixAndDesaturate()`와 네 모터 정수 출력 및 collective가 같아야 한다.

### 3.3 축 우선순위 불변식

- yaw 요구만 커진 경우 RP 차동은 변하지 않는다.
- RP 자체가 범위를 넘기 전에는 `rp_scale=1`이다.
- yaw는 RP 배정 뒤 남은 범위만 사용한다.
- 어떤 유한 입력에서도 모터 출력은 정규화된 하한·상한을 넘지 않는다.
- yaw 부호를 반대로 주면 모터 대각쌍과 yaw scale 거동도 대칭이어야 한다.

## 4. 축별 anti-windup

적분은 기존처럼 모터 기여 µs 단위와 `I_TERM_MAX_US` 상한을 유지한다.

- throttle이 1100 µs 이하이면 기존처럼 세 축 적분을 0으로 초기화한다.
- 해당 축의 scale이 1이면 기존 적분을 그대로 수행한다.
- scale이 1보다 작으면 현재 요청 출력을 더 키우는 방향의 적분 변화만 막는다.
- 현재 요청 출력의 절댓값을 줄이는 방향의 적분 변화는 허용한다.
- yaw만 제한되면 roll·pitch 적분은 계속 동작한다.
- RP가 제한되면 roll·pitch 각각 같은 conditional-integration 규칙을 적용한다.

이를 순수 helper로 분리하여 포화 방향 차단, unwind 허용, 상한, 저스로틀 reset을
네이티브 테스트한다.

## 5. Yaw authority 상태기

### 5.1 상태

| 값 | 상태 | 의미 |
|---:|---|---|
| 0 | `YAW_AUTH_NORMAL` | 기존 yaw rate/heading-hold 동작 |
| 1 | `YAW_AUTH_LIMITED` | heading을 양보하고 목표 yaw rate 0으로 감쇠 |
| 2 | `YAW_AUTH_RECOVERING` | 회복을 확인하는 동안 현재 yaw를 계속 추종 |

### 5.2 진입과 회복

초기 검증 상수:

- 진입 scale: 0.5 이하
- 진입 지속시간: 150 ms
- 회복 scale: 0.9 이상
- 회복 yaw 각속도: 기존 `YAW_HOLD_SETTLE_DPS`와 같은 10°/s 미만
- 회복 지속시간: 500 ms

상태기는 무장된 수동 비행, 즉 `Failsafe_Phase=0`에서만 동작한다. 스로틀 값
자체를 진입 조건으로 쓰지 않는다. 실제 yaw 전달 부족이 없으면 정상 호버 동작은
바뀌지 않는다.

- `NORMAL → LIMITED`: yaw 스틱이 중립이고 `yaw_scale≤0.5`가 150 ms 지속
- `LIMITED`: `Yaw_Hold=0`, 목표 heading은 현재 yaw로 슬레이빙, 목표 yaw
  각속도는 0°/s
- `LIMITED → RECOVERING`: `yaw_scale≥0.9`이고 `|Gyro_Z|<10°/s`
- `RECOVERING → NORMAL`: 회복 조건이 500 ms 지속
- 회복 조건이 깨지면 즉시 `LIMITED`로 복귀
- 스틱이 중립을 벗어나면 상태와 타이머를 초기화하고 기존 pilot rate 명령을 즉시
  통과시킨다. 스틱을 놓은 뒤 권한이 계속 부족하면 진입 시간을 다시 센다.
- safety lock, 무장 해제, 새 무장, failsafe 진입은 상태와 타이머를 초기화한다.

`yaw 1` override도 물리적으로 없는 권한을 만들 수 없으므로 LIMITED와
RECOVERING에서는 heading 강제를 우회할 수 없다. allocator는 언제나 RP 우선순위를
지킨다.

### 5.3 Failsafe 경계

새 yaw 상태기는 failsafe에서 비활성화된다. 자동착륙은 기존 `fs_hold_yaw`, 시간,
하강 스로틀, 종료 조건을 그대로 사용한다. allocator와 축별 anti-windup은 공통
모터 안전 경로이므로 failsafe에도 적용하지만, failsafe 상태 전이와 collective
계약은 회귀 시험으로 보존한다.

## 6. 텔레메트리 계약

기존 65개 필드 뒤에 다음 10개를 append한다.

| 필드 | 이름 | 형식 |
|---:|---|---|
| 66 | `Mixer_RP_Scale` | float |
| 67 | `Mixer_Yaw_Scale` | float |
| 68 | `Mixer_Collective_US` | float |
| 69 | `PID_Roll_US` | float |
| 70 | `PID_Pitch_US` | float |
| 71 | `PID_Yaw_US` | float |
| 72 | `I_Roll_US` | float |
| 73 | `I_Pitch_US` | float |
| 74 | `I_Yaw_US` | float |
| 75 | `Yaw_Authority_State` | int |

기존 `Mixer_Scaled`는 삭제하거나 의미를 바꾸지 않는다. `rp_scale` 또는
`yaw_scale`이 1보다 작으면 1이다. 구형 패킷은 새 필드를 `None`으로 파싱하고,
CSV는 수신 시각을 포함해 총 76열이 된다.

지상국은 무장 중 진단 줄에 RP/Yaw scale과 yaw authority 상태를 표시한다. 로그
분석기는 각 scale의 최솟값, 하위 백분위, 제한 상태 진입 횟수와 누적시간, PID와
적분기 포화 요약을 제공한다.

## 7. 자동 검증

### 7.1 Allocator 네이티브 시험

- zero 및 포화되지 않은 roll/pitch/yaw 명령의 기존 출력 동등성
- collective 이동의 기존 출력 동등성
- yaw-only 포화에서 `rp_scale=1`, `yaw_scale<1`, RP 차동 보존
- RP-only 포화에서만 `rp_scale<1`
- 양·음 yaw 대칭
- 극단값과 역전된 모터 범위의 안전 constrain
- 균등 축소로 되돌린 mutation이 yaw 우선순위 시험을 실패시키는 negative proof

### 7.2 상태기와 anti-windup 네이티브 시험

- 150 ms 이전에는 NORMAL, 경계에서 LIMITED
- 일시적인 scale dip은 진입하지 않음
- 500 ms 회복 히스테리시스와 재악화
- pilot rate 즉시 우선
- failsafe, disarm, safety lock reset
- yaw 제한이 RP 적분을 막지 않음
- 포화를 키우는 적분은 차단하고 줄이는 적분은 허용

### 7.3 SIL

1300 µs 정상 구간 뒤 1270 µs로 내리고, 제어 가능한 범위를 넘는 고정 yaw 토크와
작은 pitch 토크를 함께 주입한다.

- yaw가 먼저 제한되고 `rp_scale`은 RP 자체 한계 전까지 1을 유지
- LIMITED 뒤 목표 yaw 각속도가 ±180°/s에 계속 머물지 않음
- 같은 pitch 외란에서 균등 축소 mutation보다 최대 pitch 오차가 작음
- 정상 1300 µs 시나리오와 기존 SIL 회귀 기준은 악화되지 않음
- PID loop, motor bound, safety/failsafe 불변식 유지

## 8. 실기 검증 게이트

자동 검증과 ESP32-S3 compile이 모두 통과한 뒤 별도 승인으로만 업로드한다.

1. 프롭 제거: 모터 범위, 부호, safety cut, RC timeout
2. 손 파지·저출력: 1200→1270→1300 µs, 수동 yaw, 강제 yaw 외란
3. 테더 호버: 1300 µs 30초, 같은 날 기준 대비 roll·pitch RMSE 20% 이내,
   절대 roll RMSE≤5°, pitch RMSE≤4°
4. 지상 이륙→호버→착륙 3회: 착륙 말단 roll·pitch 각각 15° 미만,
   `rp_scale≥0.95` 샘플 99% 이상, LIMITED 뒤 ±180°/s 명령이 0.5초 이상
   지속하지 않음, RC·IMU·disagree·attitude fault 없음

광류가 없으므로 중립 스틱의 수평 위치 표류와 이를 상쇄하는 조종자 입력은 실패로
보지 않는다. yaw heading도 RP 자세 보존보다 낮은 우선순위다.

## 9. 실패 시 다음 단계

allocator가 RP 자세는 보존하지만 yaw 회전 자체가 착륙을 계속 방해한다면, 이번
변경에서 게인을 임의로 조정하지 않는다. 별도 단계로 모터별 PWM–추력 벤치 측정,
동적 idle, 추력 선형화의 필요성을 판단한다. 정상 호버 회귀나 failsafe 계약이
깨지면 실기 진행 없이 해당 변경을 되돌리고 원인을 수정한다.
