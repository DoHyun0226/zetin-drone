# zetin_fc — 비행 펌웨어

ZETIN 드론의 유일한 비행 펌웨어다. ESP32-S3, 듀얼 ICM42670, BMM350, 3901-L0X(광류·거리),
4채널 PWM ESC. 캐스케이드 PID 자세 제어에 광류 위치 유지·고도 유지와 RC 상실 자동착륙(시간
기반, 그리고 기본 OFF 인 거리 기반)을 더했다. 빌드·업로드·안전 수칙은
[펌웨어 안내](../../README.md), 명령·텔레메트리 규격은 [UDP 프로토콜](../../../docs/udp_protocol.md).

2026-11-13 옛 주력 `dual_imu_cascade_pwm`(이 펌웨어에서 광류 기능만 뺀 복사본)과 보류 후보
`dual_imu_flix_quat_pwm` 은 [`archive/legacy_flight`](../../archive/legacy_flight/)로 보냈다.

## 운용 전에 알아둘 것

- **부팅 때 IMU 정지 보정을 하지 않는다.** NVS 에 저장된 보정값(없으면 `fc/config.h` 의 컴파일
  값)을 쓴다. 다시 재려면 바닥에 가만히 둔 채 시동 해제 상태에서 `imucal` 을 보낸다(2 s, NVS 저장).
- **거리 기반 자동착륙은 기본 OFF** 다. 켜려면 시동 해제 상태에서 `rangeland 1`, 텔레메트리 97번
  (`RangeLand_On`)·대시보드 위치 줄로 확인한다. 끄면 RC 상실은 늘 시간 기반 하강이다.
- 광류 보정(`flowcal`)이 안 됐으면 `poshold` 를 거부한다.

## 파일 지도

단일 번역 단위다. `zetin_fc.ino` 가 아래 순서대로 헤더를 include 하고, 헤더는 앞 헤더에
정의된 것을 그대로 쓴다(단독 include 불가). 순서가 계약이다 — 바꾸면 전역 정적 초기화 순서가
바뀔 수 있다. SIL·native 시험은 `#include "zetin_fc.ino"` 한 줄로 전체를 본다.

### `logic/` — 순수 로직

Arduino·FreeRTOS 에 기대지 않는다. `tools/native_tests` 가 호스트에서 단독으로 시험한다.

| 파일 | 내용 |
|---|---|
| `mag_yaw_fusion.h` | 기울기 보상 자기 heading, yaw 보정량 |
| `yaw_command.h` | yaw 각속도 명령, heading 자동 잠금 |
| `failsafe_land.h` | 자동착륙 단계, 하강 스로틀, 프로브 착지 감지(기록용) |
| `msp_sensor.h` | MSPv2 프레임 파서(3901-L0X) |
| `control_allocator.h` | 축 우선순위 믹서·desaturation |
| `yaw_authority.h` | 저스로틀 yaw 권한 상태 |
| `flow_calibration.h` | 광류 보정 행렬·지연 추정 |
| `flow_odometry.h` | 광류 속도·높이 추정, 자이로 이력, 광류 가속도 |
| `flow_hold.h` | 위치 유지·고도 유지 제어기 |
| `range_land.h` | 거리 기반 자동착륙 판정·거리 추적 |

### `fc/` — 펌웨어 모듈 (include 순서)

"쓰기"는 런타임에 값을 바꾸는 코어다. Core 1 = `pid_task`(1 kHz), Core 0 = `udp_task`.

| 순서 | 파일 | 내용 | 쓰기 |
|---|---|---|---|
| 1 | `wire_types.h` | ZIMU/ZCAL 와이어 형식, 텔레메트리 샘플 구조체, raw IMU 링, 지연 이벤트 링 | 링: Core 1 넣기, Core 0 꺼내기 |
| 2 | `config.h` | 튜닝 게인, 핀·통신·센서·failsafe·보정 상수(정적 검사 포함) | 게인: Core 0(명령) |
| 3 | `util.h` | LPF, 링 함수, IMU freeze 감시, `rcTimedOut` | — |
| 4 | `hardware.h` | UDP 소켓·명령 버퍼, IMU 두 개, BMM350 객체 | Core 0(UDP) |
| 5 | `failsafe_state.h` | 무장·해제·resume 요청 우편함, failsafe 단계·프로브 상태 | 단계: Core 1, 요청: Core 0 |
| 6 | `flow_state.h` | 3901-L0X 수신, 광류 스냅숏, 유지·보정 요청, 광류 추정기 | 수신: Core 0, 추정: Core 1 |
| 7 | `rangeland_state.h` | 거리 기반 자동착륙 상태와 리셋 | Core 1 |
| 8 | `flow_requests.h` | 광류 텔레메트리 게시, 부팅·하네스 공용 리셋, 보정·스위치 우편함 | 요청: Core 0, 적용: Core 1 |
| 9 | `flight_state.h` | 호버 추정, 목표·추정 자세, 모터 출력, 적분, IMU 보정값, fault, RC 카운터 | 대부분 Core 1, RC·목표: Core 0 |
| 10 | `safety_requests.h` | 무장·해제·resume 처리, 하강 중 설정 변경 거부 | 요청: Core 0, 적용: Core 1 |
| 11 | `imu_snapshots.h` | 1 kHz IMU 샘플·raw 레지스터 게시 | Core 1 |
| 12 | `magnetometer.h` | 자기계 상태·스냅숏·기준 갱신 요청, 초기화·보정·샘플링 | 샘플: Core 0, 융합: Core 1 |
| 13 | `motor.h` | 모터 PWM 쓰기·정지, 믹서 호출 | Core 1 만 |
| 14 | `imu_calibration.h` | 정지 보정, NVS 저장·불러오기, `imucal` 진행 | Core 1(진행), Core 0(요청) |
| 15 | `control_support.h` | 적응 alpha, 잠금 진입, allocation 텔레메트리 | Core 1 |
| 16 | `pid_task.h` | `pid_task`: 단계 함수 11개(아래) | Core 1 |
| 17 | `udp_task.h` | `udp_task`: 수신·명령 분기·명령별 처리, 센서 폴링·송신 | Core 0 |

`zetin_fc.ino` 에는 라이브러리 include, 위 include 목록, `setup()`/`loop()` 만 있다.

### `pid_task` 한 tick

순서가 곧 동작이다(RC 상실 진입은 이번 tick 에 갱신한 높이·거리 추적·호버 유효를 쓴다).

1. `pidBeginTick` — 지연 보정, 자동착륙 스위치·무장 요청 적용, dt
2. `pidReadImus` — 읽기, freeze·불일치, 융합, body 좌표 변환 (쓸 IMU 없으면 잠그고 tick 끝)
3. `pidEstimateAttitude` — 상보필터, 자기계 yaw 융합
4. `pidSafetyAndResume` — 과도 기울기, resume, 무장 첫 tick 정리
5. `pidFlowUpdateAndHoldCandidates` — 광류·거리 프레임, 거리 기반 착지 판정, 유지 후보
6. `pidHoverAndFailsafe` — 호버 추정, RC 상실 진입, 거리·시간 기반 하강
7. `pidApplyHoldAndPublish` — 유지 적용값 확정, 광류 텔레메트리
8. `pidSafetyGate` — 잠금이면 정리하고 tick 끝
9. `pidOuterLoop` → 10. `pidInnerLoop` → 11. `pidMixAndOutput`

tick 사이에 유지되는 값은 `PidLoopState`, 단계를 건너는 tick 값은 `PidTick` 에 있다. 공유
전역은 구조체로 복사하지 않는다. 단계 함수는 `always_inline` 이라 스택 프레임은 하나다.

## 고치기 전에

- 동작을 바꾸지 않는 정리(코드 이동, 주석, 이름)는 [`tools/prove_same_firmware.py`](../../../tools/prove_same_firmware.py)
  로 ESP32 ELF 가 같음을 보인다.
- 함수 분해처럼 기계어가 달라지는 구조 변경은 [`tools/diff_firmware_behaviour.py`](../../../tools/diff_firmware_behaviour.py)
  (차분 하네스, 시나리오 24개)로 기준 리비전과 출력이 바이트까지 같음을 보인다.
- 상수·명령·텔레메트리를 바꾸면 [UDP 프로토콜](../../../docs/udp_protocol.md)과 지상국
  (`scripts/ground/telemetry_schema.py`)을 같이 고친다.
- 설계와 검증 기록: [모듈화 계획](../../../docs/superpowers/plans/2026-11-13-zetin-fc-modules.md).
