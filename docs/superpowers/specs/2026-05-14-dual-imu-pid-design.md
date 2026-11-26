# PWM_TEST_DUAL_IMU_PID 설계 명세

> 과거 문서: 이 문서는 대체된 듀얼 IMU PID 반복 작업을 설명한다.
> 현행 비행 제어 후보:
> [`dual_imu_cascade_pwm`](../../../firmware/archive/legacy_flight/dual_imu_cascade_pwm/).
> 보관된 결과물:
> [`dual_imu_pid_pwm`](../../../firmware/archive/legacy_flight/dual_imu_pid_pwm/).

**날짜:** 2026-05-14
**상태:** 승인됨, 구현 계획 준비 완료

## 1. 목표

기존 `PWM_TEST_IMU_PID.ino` (단일 IMU)에서 발생하던 **비행 중 한쪽으로 쏠리는 드리프트 문제**를 근본적으로 해결하면서, `DUAL_IMU_RAW_TEST.ino`에서 검증된 듀얼 IMU 하드웨어를 활용한 **새 비행 제어 코드**를 작성한다.

### 비목표
- Kalman 필터 / 가중 평균 등 복잡한 센서 융합 (단순 평균 + 검증으로 충분)
- 기존 UDP 프로토콜 / scripts/tune_pid.py 변경 (호환성 유지)
- 가속도계 바이어스 보정 (1G 기준점이 있어 불필요)
- 자기계(magnetometer) 융합

## 2. 새 폴더와 파일

- 폴더: `firmware/archive/legacy_flight/dual_imu_pid_pwm/`
- 파일: `PWM_TEST_DUAL_IMU_PID.ino`

## 3. 아키텍처 개요

기존 2-task 구조 그대로:
- **Core 1**: `pid_task` (1kHz 메인 제어 루프) + `loop()` (50ms 텔레메트리)
- **Core 0**: `udp_task` (RC/명령 수신)

WiFi/UDP 프로토콜과 텔레메트리 14필드 형식은 100% 호환 → `scripts/tune_pid.py` 무수정 동작.

### 하드웨어 핀
- SPI: SCK=12, MISO=13, MOSI=11
- IMU1 CS=10, IMU2 CS=9
- 모터 PWM: M1=4(FL/CW), M2=5(RR/CW), M3=6(FR/CCW), M4=7(RL/CCW)
- 기존 `LDO_PIN=9`는 **삭제** (IMU2 CS와 충돌). IMU2 라이브러리가 CS 자동 제어.

### IMU 축 정렬
하드웨어 측정 결과: IMU2는 IMU1 대비 **X축 부호 반전, Y축 동일, Z축 부호 반전**.
- `IMU2_SIGN_X = -1.0f`
- `IMU2_SIGN_Y = +1.0f`
- `IMU2_SIGN_Z = -1.0f`

이 부호는 IMU2 raw 값 읽은 직후 곱해 IMU1/drone frame으로 변환. 이후 bias 측정, fusion, 적응 필터 등 모든 downstream 로직은 정렬된 값으로 동작.

## 4. 자이로 바이어스 보정 (드리프트 해결 핵심)

### 4.1 시동 시 보정
`setup()`에서 IMU 두 개 초기화 직후, PID 태스크 생성 **전에** 수행:

1. 2000샘플 (≈2초) 동안 두 IMU의 자이로 raw값 수집
2. 각 IMU별 `gyro_bias[3]` = 평균값 저장
3. 측정 중 자이로 표준편차가 `BIAS_CALIB_MOVEMENT_THRESH` (1.0°/s) 초과하면 "움직임 감지" 시리얼 출력 후 처음부터 재측정 (최대 3회 재시도)
4. 3회 모두 실패 시: "캘리브 실패, 마지막 측정값 사용" 경고 출력 후 마지막 평균값을 바이어스로 사용 (boot은 계속 진행, safety_lock=true 유지)
5. 최종 바이어스 값을 시리얼로 출력

가속도계 바이어스는 측정/보정하지 않음 (1G 기준점 활용).

### 4.2 런타임 느린 추정
PID 루프 안에서 매 사이클:

- **조건 동시 만족**:
  - `base_throttle < 1100` (idle/disarmed)
  - `|raw_gyro_x/y/z| < 2.0°/s` (실제 정지)
- **동작**: `bias[i] = bias[i] * (1 - α) + raw_gyro[i] * α`, α = 0.0005
- 비행 중에는 절대 업데이트하지 않음 (motion 중 업데이트하면 바이어스 오염)

### 4.3 자이로 사용 시점
모든 자이로 사용 직전에 `gx_corrected = raw_gx - gyro_bias[x]` 적용. raw값은 텔레메트리에 보정 후 값으로 전송.

## 5. 듀얼 IMU 융합

### 5.1 매 루프 동작 순서
1. IMU1, IMU2에서 각각 raw accel/gyro 읽기 (SPI 두 번)
2. 각 IMU에 startup bias 보정 적용
3. 고장 검증 (`check_imu_disagree`, `check_imu_frozen`)
4. **정상 시**: 단순 평균 `gx = (gx1 + gx2) / 2`, accel도 동일
5. **한쪽 고장 시**: 멀쩡한 쪽만 사용 (fallback)
6. **둘 다 고장**: `safety_lock = true`
7. 평균/단일값에 LPF 적용 후 PID로 전달

### 5.2 평균화의 이득
- 랜덤 진동 노이즈 √2배 감소
- 한쪽 IMU 갑작스러운 글리치를 절반 흡수

## 6. 적응형 상보 필터

기존 `ALPHA_COMP = 0.995` (고정) → 가속도 크기 기반 동적 α:

```
accel_mag  = sqrt(ax² + ay² + az²)
deviation  = |accel_mag - 1.0|

if      deviation < 0.1G:  alpha = 0.99    (정적, 가속도 신뢰 ↑)
else if deviation < 0.3G:  alpha = 0.995   (보통)
else:                      alpha = 0.999   (격동, 가속도 거의 무시)
```

- 호버링 시: α=0.99 → 가속도로 자세 적극 보정 → 장기 드리프트 제거
- 급기동/진동 시: α=0.999 → 자이로만 사용 → 자세 흔들림 방지

디버깅 편의를 위해 부드러운 보간이 아닌 step 방식 사용.

## 7. 고장 검출

### 7.1 기존 유지
- `check_rc_timeout()`: RC 패킷 500ms 이상 미수신 → safety_lock
- `check_attitude()`: |roll| > 45° 또는 |pitch| > 45° → safety_lock

### 7.2 변경
- `check_imu_frozen()`: 각 IMU 개별 적용
  - 한쪽만 frozen → 해당 IMU `fault_imuN` 세팅, 멀쩡한 쪽으로 fallback, **safety_lock 안 걸음**
  - 둘 다 frozen → safety_lock

### 7.3 신규
- `check_imu_disagree()`:
  - 두 IMU의 자이로 차이 > 30°/s **또는** 가속도 차이 > 0.5G가 100ms 이상 지속
  - 어느 쪽이 맞는지 frozen 검사로 판별 시도
  - 판별 불가 시 **즉시 safety_lock** (안전 우선)

### 7.4 텔레메트리 호환성
기존 14필드 그대로 유지. `fault_imu` 비트는 `fault_imu1 || fault_imu2`로 OR. 개별 IMU 고장은 시리얼 로그로만 출력.

## 8. 데이터 흐름

```
setup()
  ├─ WiFi/UDP init
  ├─ SPI.begin(12,13,11,SPI_CS1)
  ├─ pinMode(CS1/CS2, OUTPUT) + HIGH (버스 충돌 방지)
  ├─ IMU1.begin(), IMU2.begin()
  ├─ IMU1/2.startAccel/startGyro
  ├─ calibrate_bias()            ← 신규
  ├─ ledcAttach (4모터)
  └─ xTaskCreatePinnedToCore(pid_task, udp_task)

pid_task (Core 1, 1kHz)
  ├─ read IMU1, IMU2 (raw)
  ├─ apply bias correction (각 IMU)
  ├─ check_imu_disagree, check_imu_frozen (each)
  ├─ fuse (avg or healthy one)
  ├─ LPF
  ├─ runtime bias update (idle + 정지 시에만)
  ├─ adaptive complementary filter
  ├─ check_rc_timeout, check_attitude
  ├─ PID 계산
  └─ motor mixing + writeMotor x4

udp_task (Core 0): 기존과 동일
loop() (Core 1): 50ms 텔레메트리, 기존 14필드 형식
```

## 9. 상수 요약

기존 PID 게인은 그대로 유지 (Kp_Roll=2.5, Ki_Roll=0.005, Kd_Roll=1.2 등).

신규/변경 상수:
| 이름 | 값 | 의미 |
|---|---|---|
| `SPI_CS1` | 10 | IMU1 CS |
| `SPI_CS2` | 9 | IMU2 CS (기존 LDO_PIN 대체) |
| `BIAS_CALIB_SAMPLES` | 2000 | 캘리브레이션 샘플 수 (2초 @ 1kHz) |
| `BIAS_CALIB_MOVEMENT_THRESH` | 1.0 | 캘리브 중 허용 표준편차 (°/s) |
| `BIAS_CALIB_RETRIES` | 3 | 재시도 횟수 |
| `RUNTIME_BIAS_ALPHA` | 0.0005 | 런타임 바이어스 EMA 계수 |
| `RUNTIME_BIAS_GYRO_LIMIT` | 2.0 | 정지 판단 자이로 임계치 (°/s) |
| `IMU_DISAGREE_GYRO` | 30.0 | 두 IMU 자이로 차이 임계치 (°/s) |
| `IMU_DISAGREE_ACCEL` | 0.5 | 두 IMU 가속도 차이 임계치 (G) |
| `IMU_DISAGREE_MS` | 100 | 불일치 지속 시간 (ms) |
| `ACCEL_DEV_SOFT` | 0.1 | 적응형 alpha soft threshold (G) |
| `ACCEL_DEV_HARD` | 0.3 | 적응형 alpha hard threshold (G) |
| `ALPHA_STATIC` / `ALPHA_NORMAL` / `ALPHA_DYNAMIC` | 0.99 / 0.995 / 0.999 | 적응형 alpha 값 |

`ALPHA_COMP` 상수는 삭제, 대신 동적 계산 함수 사용.

## 10. 테스트 계획

1. **부팅 시리얼 로그**: 두 IMU의 바이어스 3축 값이 합리적 범위 (-2 ~ +2°/s) 인지 확인
2. **정지 상태 yaw 드리프트**: `start` 명령 없이 5분간 angleZ가 얼마나 변하는지 (기존 대비 개선)
3. **한쪽 IMU disconnect 테스트**: CS=10 또는 CS=9 케이블 빼고 시동 → 시리얼에 fault 메시지 + 정상 비행 가능
4. **두 IMU 불일치 시뮬레이션**: 한쪽 IMU만 손으로 흔들기 → safety_lock 동작 확인
5. **실비행**: 호버링 중 드리프트가 기존 대비 줄어드는지 확인
6. **튜닝 호환성**: `scripts/tune_pid.py`로 PID 게인 변경 시 정상 적용되는지 확인

## 11. 미해결 질문 / 리스크

- **SPI 두 번 읽기 시 1kHz 유지 가능한지**: 단일 IMU 시점에 충분히 마진이 있었음. 두 번 읽어도 SPI ≥ 8MHz면 < 200µs 예상. 1kHz 루프 (1000µs) 안에 여유 있을 것. 실측으로 확인 필요.
- **두 IMU 부착 방향**: **하드웨어 측정 결과**, IMU2는 IMU1 대비 X축과 Z축이 부호 반전, Y축은 동일. `IMU2_SIGN_X = -1, IMU2_SIGN_Y = +1, IMU2_SIGN_Z = -1` 상수를 도입해 IMU2 raw 읽기 직후 적용하여 모든 downstream 로직(bias 측정, fusion, 적응 필터)이 IMU1/drone frame에서 동작하도록 함.
- **런타임 바이어스 추정의 위험**: 시동 직전 motor idle 상태에서만 업데이트하므로 propeller wash 영향 없음. 단, `base_throttle < 1100` 임계치는 실제 disarmed 시 보내는 1000 값을 기준으로 안전.
