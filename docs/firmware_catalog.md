# 펌웨어 수명주기 카탈로그

이 표는 Git에서 추적하는 **26개 스케치**를 빠짐없이 연결한다: 주력 1개,
보류 후보 1개, 진단 7개, 보관 17개다. `현행`은 유지되는 경로를 뜻하며
모든 실기 검증이 끝났다는 표시는 아니다. CI는 주력·보류 후보·진단을
컴파일하고, 보관된 스케치는 원형 보존하며 빌드하지 않는다.
의존성과 빌드 방법은 [펌웨어 안내](../firmware/README.md)를 따른다.
모터 제어 스케치를 실행하기 전에는 프로펠러를 제거한다.

주력 스케치는 단일 `.ino`가 아니라 같은 디렉터리의 헤더와 함께 빌드된다:
`failsafe_land.h`(자동착륙 상태·호버 추정·프로브), `mag_yaw_fusion.h`(BMM350
융합 수학), `yaw_command.h`(yaw 각속도 명령·heading 자동 잠금),
`msp_sensor.h`(3901-L0X INAV MSPv2 파서). 이 헤더들은
[`tools/native_tests/`](../tools/native_tests/)에서 호스트 단위검사와 SIL로
검증한다. 호스트 SIL 결과는 실제 센서·PWM·비행 검증과 구분한다.

| 수명주기 | 대상 MCU | 센서 | 모터 프로토콜 | 안전/빌드 메모 | 링크 |
|---|---|---|---|---|---|
| 현행·**주력** | ESP32-S3 | 듀얼 ICM42670 + BMM350 + 3901-L0X | PWM | 75필드 텔레메트리·RC 감시·자동착륙 상태기계 포함. BMM350 yaw 융합은 기본 OFF, 3901-L0X는 기록 전용. 실비행 기록과 자동착륙 한계는 아래 근거 참조 | [`dual_imu_cascade_pwm`](../firmware/flight/dual_imu_cascade_pwm/) |
| **보류** | ESP32-S3 | 듀얼 ICM42670 | PWM | flix 기반 쿼터니언 후보. 컴파일 대상이지만 실기 검증 미완료. 22필드, `rcr`·`gains`·자동착륙 미지원, 게인 단위·yaw 부호 차이 → README 참조 | [`dual_imu_flix_quat_pwm`](../firmware/flight/dual_imu_flix_quat_pwm/) |
| 현행 | ESP32-S3 | 없음 | PWM | ESP32Servo 50Hz. **부팅 7초 후 네 모터 자동 램프(최대 1300µs), 시리얼 정지 명령 없음** | [`motor_pwm_bench`](../firmware/diagnostics/motor_pwm_bench/) |
| 현행 | ESP32-S3 | 없음 | PWM | 수동 단일 모터 식별, 상한 1250µs, 자동 시작 없음. 벤치 Stage A. **비행 펌웨어와 같은 LEDC 400Hz** 사용. 이 기체에서 기록된 ESP32Servo 채널 묶임 문제를 피하는 경로 | [`motor_id_single`](../firmware/diagnostics/motor_id_single/) |
| 현행 | ESP32-S3 | 없음 | PWM | ESC 설정용 LEDC 50Hz, 기본 GPIO7. **전류 제한 전원·ESC/모터 하나만 연결. `h`는 최대 2000µs를 계속 출력** | [`esc_config_throttle`](../firmware/diagnostics/esc_config_throttle/) |
| 현행 | ESP32-S3 | ICM42670 1개 | 없음 | raw SPI 읽기. 별도 18/19/23/5 핀 배치 사용 | [`icm42670_single_raw`](../firmware/diagnostics/icm42670_single_raw/) |
| 현행 | ESP32-S3 | 듀얼 ICM42670 | 없음 | 듀얼 IMU raw SPI 읽기 | [`icm42670_dual_raw`](../firmware/diagnostics/icm42670_dual_raw/) |
| 현행 | ESP32-S3 | 듀얼 ICM42670 | PWM 최저값 고정 | 구형 추정기 루프 진단. PID·믹서 없음, 전 모터 1000µs. 별도 14필드이며 `receive_dual_imu_debug.py`와 사용 | [`icm42670_dual_loop_debug`](../firmware/diagnostics/icm42670_dual_loop_debug/) |
| 현행 | ESP32-S3 | 듀얼 ICM42670 | 없음 | PCB v1.5.2 센서 브링업 진단 | [`board_v1_5_2_dual_imu`](../firmware/diagnostics/board_v1_5_2_dual_imu/) |
| 보관 | ESP32-S3 | ICM42670 1개 | PWM | 대체된 단일 IMU PID. 미지원 | [`single_imu_pid_pwm`](../firmware/archive/legacy_flight/single_imu_pid_pwm/) |
| 보관 | ESP32-S3 | 듀얼 ICM42670 | PWM | 대체된 듀얼 IMU PID. 단순 평균·시작 시 bias 보정이며 과거 계획의 개별 IMU 폴백·런타임 bias 갱신은 없음. 미지원 | [`dual_imu_pid_pwm`](../firmware/archive/legacy_flight/dual_imu_pid_pwm/) |
| 보관 | ESP32-S3 | ICM42670 1개 | PWM | 대체된 캐스케이드 실험. 미지원 | [`single_imu_cascade_pwm`](../firmware/archive/legacy_flight/single_imu_cascade_pwm/) |
| 보관 | ESP32-S3 | ICM42670 1개 | PWM | Kalman/PID 실험. **과도 기울기 분기에서 잠금만 설정하고 모터 출력을 즉시 갱신하지 않음**. 미지원 | [`single_imu_kalman_pid_pwm`](../firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/) |
| 보관 | ESP32-S3 | ICM42670 1개 | 없음 | Kalman 자세 추정 실험. 미지원 | [`icm42670_kalman_attitude`](../firmware/archive/filter_experiments/icm42670_kalman_attitude/) |
| 보관 | ESP32 계열 | 없음 | DShot600 | 고정 단일 모터 출력. 미지원 | [`single_motor_fixed_throttle`](../firmware/archive/dshot/single_motor_fixed_throttle/) |
| 보관 | ESP32 계열 | 없음 | DShot600 | 시리얼 제어 단일 모터 실험. 미지원 | [`single_motor_serial_control`](../firmware/archive/dshot/single_motor_serial_control/) |
| 보관 | ESP32 계열 | MPU6500 | DShot600 | 기울기-스로틀 실험. 미지원 | [`mpu6500_tilt_throttle`](../firmware/archive/dshot/mpu6500_tilt_throttle/) |
| 보관 | ESP32 계열 | MPU6500 | DShot600 | 이름과 달리 PID 계산 없이 기울기로 출력 선택. **센서 초기화 실패에도 진행하며 0 명령도 최종 출력에서 최소 5%로 바꿈**. 실행용 벤치 예제 아님 | [`mpu6500_pid_bench`](../firmware/archive/dshot/mpu6500_pid_bench/) |
| 보관 | ESP32 계열 | 없음 | DShot600 | **위험: 4개 모터를 최대 출력 근처로 구동함**. 벤치 테스트로 실행 금지 | [`four_motor_full_throttle_unsafe`](../firmware/archive/dshot/four_motor_full_throttle_unsafe/) |
| 보관 | ESP32-S3 | 시뮬레이션 입력 | DShot600 | 미완성 듀얼코어 모터 할당기. 미지원 | [`motor_allocator_dual_core`](../firmware/archive/dshot/motor_allocator_dual_core/) |
| 보관 | ESP32-S3 | 시뮬레이션 입력 | DShot600 | 오래된 PlatformIO 골격에 의존. 미지원 | [`motor_allocator_dshot`](../firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/) |
| 보관 | ESP32-S3 | BMM350 | 없음 | 단독 자기계 읽기. 미지원 | [`bmm350_read`](../firmware/archive/other_sensors/bmm350_read/) |
| 보관 | ESP32-S3 | BMM350 | 없음 | PCB v1.5.2 자기계 진단. 미지원 | [`board_v1_5_2_bmm350`](../firmware/archive/other_sensors/board_v1_5_2_bmm350/) |
| 보관 | ESP32-S3 | GPS UART | 없음 | raw NMEA 패스스루. 미지원 | [`gps_uart_passthrough`](../firmware/archive/other_sensors/gps_uart_passthrough/) |
| 보관 | Arduino 호환 | US100 | 없음 | 오래된 의존성 기반 거리 측정 실험. 미지원 | [`us100_distance`](../firmware/archive/other_sensors/us100_distance/) |
| 보관 | STM32 F411 | UART | 없음 | 대체 MCU 수신 실험. 미지원 | [`stm32_f411_uart_rx`](../firmware/archive/other_mcus/stm32_f411_uart_rx/) |

## 검증 기록과 과거 문서 읽는 법

- [전원 인가·벤치 절차](power_on_bench_procedure.md)의 2026-08-01 기록에는
  176.2초 연속 무장 구간과 고장 플래그 0건이 있다. 무장 시간 전체가 공중
  체공 시간이라는 뜻은 아니며, 현재 리비전의 반복 비행 보장으로 확대하지 않는다.
- 자동착륙 상태기계는 구현되어 있지만, IMU만으로 접지를 판별하는 데 한계가
  있다. [자동착륙 연구·정정 기록](failsafe_land_research.md)을 함께 읽는다.
- [마이그레이션 맵](migration_map.md)의 23개 원본 스케치는 당시 이동 대상이다.
  이후 추가된 flix 후보·`motor_id_single`·`esc_config_throttle`까지 포함한
  현재 전체 목록은 위 26개다.
- [과거 PID 설계](history/2026-05-14-dual-imu-pid-design.md)와
  [구현 계획](history/2026-05-14-dual-imu-pid-implementation-plan.md),
  [정리 설계](design/2026-07-13-repository-cleanup-design.md)와
  [정리 계획](plans/2026-07-13-repository-cleanup.md)은 당시의 요구·예상·명령을
  보존한다. 체크리스트나 예상 결과를 구현·실행 완료 증거로 사용하지 않는다.
