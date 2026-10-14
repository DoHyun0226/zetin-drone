# dual_imu_flix_quat_pwm

[flix](https://github.com/okalachev/flix) (MIT, Oleg Kalachev) 아키텍처를
ZETIN 하드웨어(ESP32-S3 + 듀얼 ICM42670 + PWM ESC)에 이식한 **보류 후보**다.
주력 [`dual_imu_cascade_pwm`](../dual_imu_cascade_pwm/)과 일부 문자 명령 및
텔레메트리 앞부분을 공유하지만, **현행 지상국 제어와 완전히 호환되지 않는다.**

## 상태와 호환성

2026-07-22에 주력 캐스케이드에 집중하기로 한 보류 결정이 유지되는 경로다.
CI 컴파일 대상에는 들어 있지만, 이 문서에 기록된 플래시·벤치·비행 검증
완료 근거는 없다. 주력의 비행 실적을 이 후보의 실적으로 적용하지 않는다.

최초 이식 이후 주력에 추가된 개선은 자동 전파되지 않았다. 소스에서 확인할
수 있는 차이는 다음과 같다:

- RC 워치독은 부호 있는 시간 차이를 사용하고, `rc` 시퀀스의 부호 문자·
  범위 초과·중복/역순을 거부한다. 이 이식이 주력의 모든 안전 개선을 포함하는
  것은 아니다.
- **`rcr` 미지원.** 이 후보의 `rc <seq> <roll> <pitch> <yaw>`는 각도 명령이며
  yaw도 절대 각도(deg, CCW+)다. 현행 `control_dualsense.py`가 쓰는 `rcr`
  각속도 명령을 처리하지 않으므로 주력용 조종 절차를 그대로 사용할 수 없다.
- **RC 두절 시 모터 컷.** 유효 RC 갱신이 500ms를 넘게 끊기면 잠금 상태로
  전환하여 모터에 1000µs를 출력한다. 주력의 자동착륙 상태기계는 없다.
- **`gains` 응답 없음.** 게인 변경 명령 일부는 있지만 읽기 응답은 없다.
  주력의 `trim`, `mag`/`magcal`, `raw` 명령도 지원하지 않는다.
- **텔레메트리는 22필드.** `Armed`에서 끝나며 주력의 현재 75필드와 다르다.
  공유 [파서](../../../scripts/telemetry_schema.py)는 빠진 필드를 `None`으로,
  CSV에는 빈 셀로 남긴다. 이것은 파싱 호환이며 모터 출력·목표값·자기계·
  자동착륙·거리·광류 관측을 제공한다는 뜻은 아니다.
- **가속도 x/y 매핑 검증 미완료.** 아래 부호 점검이 필요하다. 정지 시
  가속도 크기와 z 방향 검사만으로 x/y 부호까지 검증되지는 않는다.

정확한 명령 분기는 [스케치](dual_imu_flix_quat_pwm.ino)의 `udp_task`,
`handleRcCommand`, `handleGainCommand`, 필드는 `sendTelemetry`를 기준으로
본다. [주력 UDP 문서](../../../docs/udp_protocol.md)의 확장 기능을 이 후보에
자동 적용하지 않는다.

## dual_imu_cascade_pwm과의 차이

| 항목 | dual_imu_cascade_pwm | 이 펌웨어 |
|---|---|---|
| 자세 추정 | 축별 Euler 상보필터 | 쿼터니언 적분 + 착지 시 가속도 보정 + 비행 중 수평 가정 보정 (flix `estimate.ino`) |
| 자세 오차 | 축별 각도 차 | up-벡터 회전벡터 (flix `control.ino`) |
| 내부 단위 | deg, deg/s, µs | rad, rad/s, 정규화 토크 (1.0 = 모터 전 구간 1000µs) |
| outer loop | 250Hz | 1kHz |
| yaw 부호 | 위에서 CW+ | 위에서 CCW+ (오른손 FLU) |
| 믹서 | 자체 부호 | flix 부호 (아래 참조) |
| 조종 명령 | `rcr` 각속도 + 레거시 `rc` 각도 | `rc` 각도만 |
| RC 두절 | 자동착륙 상태기계 | 모터 컷 |
| 텔레메트리 | 75필드 | 22필드 |

듀얼 IMU 평균·한쪽 freeze 시 폴백·불일치 감시·시동 게이트·태스크 워치독을
별도로 구현한다. 일부가 주력에서 유래했어도 현재 두 구현이 같다고 가정하지
않는다. 이 후보는 자이로 121Hz·가속도 25Hz 하드웨어 LPF와 저역 불일치
감시를 사용하고, 이미 무장한 상태의 중복 `start`를 무시한다.
[vector.h](vector.h), [quaternion.h](quaternion.h), [lpf.h](lpf.h)는 flix
유래 수학 코드이며, [pid.h](pid.h)는 명시적 dt·측정값 미분 D항·조건부 적분을
사용하는 프로젝트 변형이다.

## 좌표계와 부호 (첫 비행 전 반드시 벤치 검증)

body frame은 오른손 FLU(x 앞, y 왼쪽, z 위). IMU1 sensor frame 기준
`body = (-sensor_y, +sensor_x, +sensor_z)`.

- roll+ = 오른쪽이 내려감, pitch+ = 기수가 내려감, yaw+ = 위에서 볼 때 CCW.
- 텔레메트리 roll/pitch는 주력과 같은 방향을 의도하며 **yaw 부호는 반대**다.
- 이 후보는 가속도와 자이로에 같은 `sensorToBody()` 변환을 적용한다.
  주력의 가속도 x/y 변환과 부호가 다르다. 시작 시 보정은 가속도 크기와
  z 방향을 검사하여 조건을 벗어나면 시동을 거부하지만, 좌표계가 실기와
  맞는지는 기체를 각 축으로 움직여 별도로 확인해야 한다.

### 벤치 부호 점검 절차 (프로펠러 제거 상태)

1. 부팅 후 시리얼에서 `[CALIB] OK`와 rest accel body가 `(≈0, ≈0, ≈+1)`인지 확인.
2. `monitor_telemetry.py`로 수신하고, 모터는 잠근 상태에서 기체를 손으로 움직여 확인:
   - 오른쪽으로 기울이면 Roll이 +로 증가
   - 기수를 아래로 숙이면 Pitch가 +로 증가
   - 위에서 볼 때 반시계로 돌리면 Yaw가 +로 증가
   하나라도 다르면 모터 시험을 중단하고 매핑을 재검토한다.
3. 모터 시험 전에는 이 후보의 `rc` 각도 명령을 보내는 벤치용 송신 경로를
   준비한다. 유효 RC를 500ms보다 짧은 간격으로 갱신하고, 정지 명령을 확인한다.
   모니터와 별도 송신기를 동시에 켜면 마지막 송신자로 텔레메트리 목적지가
   바뀌므로 한 소켓에서 명령·수신을 담당하도록 한다. `start` + `th 1150`
   상태에서 기체를 오른쪽으로 기울이면 오른쪽 모터
   (M2 RR, M3 FR)가 빨라지는지, 기수를 숙이면 앞 모터(M1 FL, M3 FR)가
   빨라지는지 확인한다.
4. `yaw 0`에서도 목표 각속도 0의 yaw 감쇠는 유지된다. `yaw 1`은 절대
   heading 제어를 추가한다. 해당 모드에서도 보정 방향을 확인한다. 기대한
   모터 회전 방향(FL/RR=CW, FR/RL=CCW)이나 보정 반응과 다르면 시험을 중단하고
   실제 방향·배선·믹서 부호를 함께 재검토한다.

## 믹서

```text
M1 FL(GPIO4, CW)  = t + tx - ty + tz
M2 RR(GPIO5, CW)  = t - tx + ty + tz
M3 FR(GPIO6, CCW) = t - tx - ty - tz
M4 RL(GPIO7, CCW) = t + tx + ty - tz
```

desaturation은 자세 차동 명령을 우선 보존하고 collective를 이동하는
기존 방식을 유지한다.

## 게인 단위와 기본값

| 명령 | 대상 | 단위 | 기본값 |
|---|---|---|---|
| `ap`/`ar`/`at` | 각도 P (roll/pitch) | (rad/s)/rad | 6.0 |
| `ay` | 각도 P (yaw) | (rad/s)/rad | 3.0 |
| `rp`,`pa` / `ri`,`ia` / `rd`,`da` | 각속도 PID (roll+pitch) | 입력 rad/s, 출력 정규화 토크 | 0.05 / 0.2 / 0.001 |
| `yp`,`py` / `yi`,`iy` / `yd`,`dy` | 각속도 PID (yaw) | 입력 rad/s, 출력 정규화 토크 | 0.3 / 0 / 0 |

**주의: `dual_imu_cascade_pwm`의 µs/(deg/s) 게인 값과 호환되지 않는다.**
기본값은 flix의 것으로, 기체 질량·프로펠러가 다르므로 낮은 스로틀 창에서
다시 튜닝해야 한다. 적분항 한계는 토크 0.3, D항 LPF는 약 40Hz다.

## 빌드

[공통 빌드 안내](../../README.md)에 따라 ESP32 코어와 `ICM42670P`를
준비한 뒤 저장소 루트에서 실행한다. 같은 디렉터리의 네 헤더가 함께 필요하다.

```bash
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-dual_imu_flix_quat_pwm \
  firmware/flight/dual_imu_flix_quat_pwm
```
