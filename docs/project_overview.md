# ZETIN Drone 프로젝트 개요

이 문서는 현재 지원하는 경로와 역사 보관 경로를 구분해 처음 보는 사람이
올바른 파일에서 시작하도록 안내한다.

현행 설명은 **저스로틀 yaw 개선 통합 코드와 대조**했다. 과거 실험 기록과
통합 후 남은 검증 범위는 [현재 상태](current_status.md), 실행 순서는
[작업 흐름](workflow.md), 실물 확인이 필요한 구성은
[하드웨어 구성](hardware_configuration.md)에서 확인한다.

## 지원 범위와 성숙도

현행 하드웨어·제어 조합은 **ESP32-S3 + 듀얼 ICM42670 + PWM ESC**다.

| 기능 | 상태 | 시작점 |
|---|---|---|
| 4채널 ESC PWM | 벤치 검증됨 | [`motor_pwm_bench`](../firmware/diagnostics/motor_pwm_bench/) |
| 단일·듀얼 ICM42670 raw 읽기 | 벤치 검증됨 | [`firmware/diagnostics/`](../firmware/diagnostics/) |
| 듀얼 IMU 캐스케이드 제어 | **실비행 검증됨** (2026-08-01, 176초 무장·고장 0건) | [`dual_imu_cascade_pwm`](../firmware/flight/dual_imu_cascade_pwm/) |
| BMM350 yaw 융합 | 벤치 4단계 + 실비행 heading-hold 검증됨 | [`bmm350_yaw_bench_test.md`](bmm350_yaw_bench_test.md) |
| RC 두절 자동착륙 | 실비행 4회 수행. 착지 감지는 **미해결**(시간 기반 하강) | [`failsafe_land_research.md`](failsafe_land_research.md) |
| 3901-L0X 광류·거리계 | 텔레메트리 기록만. **제어에 쓰지 않음** | [`msp_sensor.h`](../firmware/flight/dual_imu_cascade_pwm/msp_sensor.h) |
| 테더 이륙·수동 호버·착륙 반복 | 별도 실험 구성에서 2026-08-19 5회 기록 | [현재 상태와 적용 범위](current_status.md) |
| 자동 고도·위치 유지 | 미구현 | 거리·광류 피드백을 제어에 연결하지 않음 |

로그 한 번이 반복 가능한 비행을 보장하지 않는다. 2026-08-01 비행은 자세
제어와 안전 경로가 공중에서 동작한다는 증거이지, 실내 자유 비행이 가능하다는
뜻이 아니다. **고도와 위치를 측정하는 센서가 폐루프에 들어가 있지 않아, 조종
스틱을 중립에 두면 기체는 계속 흘러간다**(2026-08-18~19 테더 시험 기록에서
조종자가 호버 내내 역방향 조향을 했다고 설명했다). 보관 코드의 과거 성공 기록은 현행 지원
근거로 사용하지 않는다.

## 저장소 구조

```text
firmware/
  flight/                 current flight-controller candidate
  diagnostics/            current ESP32-S3/ICM42670/PWM bench sketches
  archive/                unsupported historical firmware
scripts/
  archive/                deprecated GPS/TCP PC tools
docs/                     maintained, historical, and archived documents
logs/                     generated flight CSV logs
tools/                    repository and telemetry checks
```

초기 PlatformIO `src/`, `lib/`, `include/` 골격은 실제 센서 대신 시뮬레이션
값을 쓰고 옛 TCP 전송을 포함한다. 현재 코드로 사용하지 말고
[`firmware/archive/platformio_skeleton/README.md`](../firmware/archive/platformio_skeleton/README.md)에서
역사 자료로만 확인한다.

## 현행 비행 제어 후보

대표 소스는
[`dual_imu_cascade_pwm.ino`](../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino)다.
ESP32-S3의 FreeRTOS 태스크에서 듀얼 IMU를 읽고, 자세 바깥 루프와 각속도
안쪽 루프를 거쳐 4개 ESC PWM을 계산한다. 통신은 ESP32 SoftAP와 UDP
4210을 쓴다. 자세한 문자열 명령과 필드 순서는
[`udp_protocol.md`](udp_protocol.md)를 따른다.

yaw 스틱은 절대 각도가 아니라 `rcr` 각속도 명령이다. 스틱이 중립이고 실제
yaw 각속도까지 정착하면 회전이 멈춘 heading을 자동으로 잠그며, 정착하지
않으면 강제 잠금 없이 rate 모드에 남는다. BMM350 자기계 융합(`mag 1`)을 켜면
자이로 적분 드리프트를 보정한다. 정확도는 자기계 보정·간섭·장착 조건에
영향받는다. **펌웨어 기본값은 OFF, 지상국의 조종자 선택 기본값은 ON**이다.
`control_dualsense.py`는 시동 직전 선택값을 전송하며 `--no-mag`나
`mag off`로 끈 선택을 유지한다. 실제 적용 상태는 `Mag_Enabled`로 확인한다. hard/soft-iron 보정은 Li & Griffiths 제약 타원체 피팅으로 구하고
(`scripts/magcal_fit.py`), 모터 전류 간섭은 hard/soft-iron·body 변환 후 XYZ의 throttle 보정으로
상쇄한다([`bmm350_yaw_bench_test.md`](bmm350_yaw_bench_test.md)). PID 태스크가
SPI 행업 등으로 정지하면 태스크 워치독(500ms)이 재부팅을 강제해 마지막
PWM으로 모터가 고정되는 것을 막는다.

Matek 3901-L0X(PMW3901 광류 + VL53L0X ToF)는 GPIO16으로 INAV MSPv2 프레임을
받아 텔레메트리 60~64번으로 내보내기만 한다. **아직 어떤 제어 판정에도 쓰지
않는다.** 거리 원값은 작동 범위(80~2000mm) 밖이면 음수이고 너무 가까울 때와
너무 멀 때가 같은 음수이므로, 신선도는 반드시 `*_Quality`의 −1로 판단한다.

RC 입력이 500ms를 초과해 끊기면 호버 추정이 유효하지 않거나 스로틀이
`FS_GROUND_CUT_MAX_US=1150` 이하일 때 즉시 컷한다. 그 외에는 저장된
roll·pitch 트림을 유지하며 시간 기반 자동착륙에 진입한다. 하강값 `FS_DESCENT_DELTA_US=60`은
`CTRL_MARGIN=150`보다 작게 유지한다. **자동착륙은 착지를 감지하지 않고
`FS_MAX_MS=3000`ms 동안 내려간 뒤 백스톱이 자른다.** IMU 기반 후보는
평가·보류 상태이며 착지 판정에 채택되지 않았다. 접지 여부를 확인하지
않으므로 시간 종료 시 공중 컷도 배제할 수 없다. 능동 스로틀 프로브는
실제 스로틀 딥과 기록을 계속하지만 착지 컷을 결정하지 않는다.
RC 타임아웃만 이 경로를 사용하고, IMU 전멸·과도 기울기·`stop`은
즉시 컷한다. 자세한 근거는
[`failsafe_land_research.md`](failsafe_land_research.md).

### 핀 배치

```text
SPI: SCK=12, MISO=13, MOSI=11
IMU1 CS=10, IMU2 CS=9
Motor PWM: M1=4 (FL), M2=5 (RR), M3=6 (FR), M4=7 (RL)
BMM350: I2C 주소 0x14
3901-L0X: 모듈 TX → GPIO16 (수신 전용, 3.3V 직결). 전원은 반드시 5V
```

단일 IMU raw 진단만 별도 배치(SCK/MISO/MOSI/CS = 18/19/23/5)를 쓴다.
각 진단별 핀은 [`firmware/README.md`](../firmware/README.md)에 정리돼 있다.

### 센서축과 기체축

IMU2는 IMU1 기준 X와 Z 부호가 반대이고 Y는 같다. 두 센서를 평균하기
전에 IMU2에 X=-1, Y=+1, Z=-1을 적용한다. 융합 센서축에서 기체축으로의
현재 변환은 다음과 같다.

```text
body roll rate X  =  sensor Y
body pitch rate Y = -sensor X
body yaw rate Z   = -sensor Z
body accel X      =  sensor Y
body accel Y      = -sensor X
body accel Z      =  sensor Z
```

### 모터 믹서

M1/M2/M3/M4는 FL/RR/FR/RL 순서다. `T`를 collective throttle,
`R`, `P`, `Y`를 roll/pitch/yaw 보정량이라 하면 기본 차동 부호는 다음과
같다. 실제 allocator는 roll/pitch를 우선 배분하고 남은 범위에 yaw를 넣는다.
roll/pitch 차동 자체가 모터 허용 폭을 넘을 때만 두 축을 공통 비율로 줄이며,
yaw는 별도 비율로 축소한다. collective는 배분한 차동을 보존하도록 이동한다.

```text
M1 = T - P + R - Y
M2 = T + P - R - Y
M3 = T - P - R + Y
M4 = T + P + R + Y
```

## PC 도구

| 도구 | 역할 |
|---|---|
| [`control_dualsense.py`](../scripts/control_dualsense.py) | **정상 조종·벤치의 유일한 대화형 지상국.** RC 스트리밍·stdin 명령·상태 표시·CSV·raw 기록을 한 프로세스에서 처리 |
| [`tune_pid.py`](../scripts/tune_pid.py) | 게임패드 없이 UDP 명령·게인만 다루는 보조 도구 |
| [`receive_telemetry.py`](../scripts/receive_telemetry.py) | 터미널 수신기 겸 CSV 로거 |
| [`monitor_telemetry.py`](../scripts/monitor_telemetry.py) | 실시간 플롯 겸 CSV 로거 |
| [`analyze_flight_log.py`](../scripts/analyze_flight_log.py) | 20Hz CSV 오프라인 분석 |
| [`analyze_probe_response.py`](../scripts/analyze_probe_response.py) | 자동착륙 프로브 판정·응답 분포 분석 |
| [`decode_imu_raw.py`](../scripts/decode_imu_raw.py) | `ZIMU`/`ZCAL` 바이너리 → CSV 디코더 |
| [`magcal_fit.py`](../scripts/magcal_fit.py) | 캡처 CSV → 제약 타원체 hard/soft-iron 상수 피팅 |
| [`receive_dual_imu_debug.py`](../scripts/receive_dual_imu_debug.py) | 듀얼 IMU 루프 진단 수신기 |
| [`bench_sign_test.py`](../scripts/bench_sign_test.py) · [`bench_yaw_test.py`](../scripts/bench_yaw_test.py) · [`bench_thrust_ramp.py`](../scripts/bench_thrust_ramp.py) | 전원 인가 벤치 절차용 스크립트. 시작 시 `trim 0 0`을 보내 드론 트림을 지운다 |

⚠️ 위 도구 중 둘 이상을 **동시에 실행하면 안 된다.** 펌웨어는 들어온 모든 UDP
패킷의 발신자로 텔레메트리 목적지를 다시 지정하므로, 보조 도구가 명령을 보내는
순간 텔레메트리를 가져간다. 실행 명령과 주의사항은
[`scripts/README.md`](../scripts/README.md)에 있다.

현행 펌웨어는 UDP 패킷마다 75개 텔레메트리 필드를 보낸다.
3901-L0X 상태(60~64)와 `Mag_Cal_Active`(65)에 이어 allocator·축별 PID/I·
yaw 권한 상태(66~75)가 붙는다. PC 수신기는 맨 앞에 수신
`Timestamp`를 추가하므로 CSV는 76개 열이다. 필드는 **append-only**다 — 중간에
끼워 넣거나 순서를 바꾸면 구버전 파서가 전부 깨진다. 공유 파서는 오래된
10·14·21·22·30·31·34·35·38·40·43·55·58·59·64·65필드 패킷도 받아들이며, 없는
값은 빈 CSV 셀로 남긴다. 필드별 의미는
[`udp_protocol.md`](udp_protocol.md)가 원본이다.

## 빌드와 실행

저장소 루트에서 현행 비행 후보를 다음처럼 빌드한다. Arduino 빌드 파일은
저장소가 아닌 `/tmp`에 둔다.

```bash
arduino-cli compile --warnings all \
  --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-flight-build \
  firmware/flight/dual_imu_cascade_pwm
```

PC를 펌웨어의 `Drone_Tuning` SoftAP에 연결한 뒤 필요한 도구를 실행한다.
기본 드론 주소는 `192.168.4.1`, 포트는 4210이다. 실행 명령은
[`scripts/README.md`](../scripts/README.md)에 있다.

## 보관된 기능

다음 항목은 모두 deprecated이며 현재 빌드 대상으로 고치지 않는다.

- 옛 단일·듀얼 PID, 단일 IMU 캐스케이드, Kalman 비행 변형:
  [`firmware/archive/legacy_flight/`](../firmware/archive/legacy_flight/),
  [`firmware/archive/filter_experiments/`](../firmware/archive/filter_experiments/)
- DShot와 MPU6500 실험:
  [`firmware/archive/dshot/`](../firmware/archive/dshot/)
- BMM350, GPS, US100:
  [`firmware/archive/other_sensors/`](../firmware/archive/other_sensors/)
- STM32 F411 UART 실험:
  [`firmware/archive/other_mcus/`](../firmware/archive/other_mcus/)
- 옛 GPS 수신기와 TCP 테스트:
  [`scripts/archive/`](../scripts/archive/)

26개 스케치의 상태는 [`firmware_catalog.md`](firmware_catalog.md), 모든 옛
경로와 새 경로의 대응은 [`migration_map.md`](migration_map.md)에서 찾는다.

## 안전한 확인 순서

1. 프로펠러를 제거한다.
2. [`motor_id_single`](../firmware/diagnostics/motor_id_single/)로 한 모터씩
   번호·방향·PWM 반응을 확인한다. 자동 4모터 램프인 `motor_pwm_bench`와 구분한다.
3. [`icm42670_single_raw`](../firmware/diagnostics/icm42670_single_raw/)과
   [`icm42670_dual_raw`](../firmware/diagnostics/icm42670_dual_raw/)로 센서축과
   두 IMU 부호를 확인한다.
4. [`icm42670_dual_loop_debug`](../firmware/diagnostics/icm42670_dual_loop_debug/)로
   추정각·루프 주기를 확인한다. 이 진단은 PID·믹서가 없으므로 보정 방향은
   주력 비행 펌웨어의 프롭 OFF 벤치 단계에서 확인한다.
5. 전원 극성, 비상 정지, RC timeout 자동착륙, 과도 기울기 정지를 확인한
   뒤에만 제한된 테스트 리그에서 비행 후보를 평가한다.
