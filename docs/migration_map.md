# 저장소 마이그레이션 맵

이 표는 이전 저장소 구조에서 정리 후 구조로의 대응을 정의하는 기준 표다.
모든 새 경로는 이 리비전에 존재하며, 모든 이전 경로는 제거되거나 이름이
바뀌었다.

| 이전 경로 | 새 경로 | 수명주기 |
|---|---|---|
| `firmware/examples/DUAL_IMU_CASCADE/DUAL_IMU_CASCADE.ino` | `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino` | 현행 |
| `firmware/examples/PWM_TEST/PWM_TEST.ino` | `firmware/diagnostics/motor_pwm_bench/motor_pwm_bench.ino` | 현행 |
| `firmware/examples/IMU_TEST_RAW/IMU_TEST_RAW.ino` | `firmware/diagnostics/icm42670_single_raw/icm42670_single_raw.ino` | 현행 |
| `firmware/examples/DUAL_IMU_RAW_TEST/DUAL_IMU_RAW_TEST.ino` | `firmware/diagnostics/icm42670_dual_raw/icm42670_dual_raw.ino` | 현행 |
| `firmware/examples/DUAL_IMU_PID_DEBUG/DUAL_IMU_PID_DEBUG.ino` | `firmware/diagnostics/icm42670_dual_loop_debug/icm42670_dual_loop_debug.ino` | 현행 |
| `firmware/examples/PCB_1.5.2_TEST/PCB_1.5.2_TEST.ino` | `firmware/diagnostics/board_v1_5_2_dual_imu/board_v1_5_2_dual_imu.ino` | 현행 |
| `firmware/examples/PWM_TEST_IMU_PID/PWM_TEST_IMU_PID.ino` | `firmware/archive/legacy_flight/single_imu_pid_pwm/single_imu_pid_pwm.ino` | 보관 |
| `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino` | `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino` | 보관 |
| `firmware/examples/DUAL_LOOP_TEST/DUAL_LOOP_TEST.ino` | `firmware/archive/legacy_flight/single_imu_cascade_pwm/single_imu_cascade_pwm.ino` | 보관 |
| `firmware/examples/KALMAN_FLIGHT_TEST/KALMAN_FLIGHT_TEST.ino` | `firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/single_imu_kalman_pid_pwm.ino` | 보관 |
| `firmware/examples/KALMAN_TEST/KALMAN_TEST.ino` | `firmware/archive/filter_experiments/icm42670_kalman_attitude/icm42670_kalman_attitude.ino` | 보관 |
| `firmware/examples/DSHOT_TEST/DSHOT_TEST.ino` | `firmware/archive/dshot/single_motor_fixed_throttle/single_motor_fixed_throttle.ino` | 보관 |
| `firmware/examples/DSHOT_TEST_PERCENT/DSHOT_TEST_PERCENT.ino` | `firmware/archive/dshot/single_motor_serial_control/single_motor_serial_control.ino` | 보관 |
| `firmware/examples/DSHOT_TEST_IMU/DSHOT_TEST_IMU.ino` | `firmware/archive/dshot/mpu6500_tilt_throttle/mpu6500_tilt_throttle.ino` | 보관 |
| `firmware/examples/DSHOT_TEST_IMU_PID/DSHOT_TEST_IMU_PID.ino` | `firmware/archive/dshot/mpu6500_pid_bench/mpu6500_pid_bench.ino` | 보관 |
| `firmware/examples/RMT_TEST/RMT_TEST.ino` | `firmware/archive/dshot/four_motor_full_throttle_unsafe/four_motor_full_throttle_unsafe.ino` | 보관 |
| `firmware/examples/ZETIN_Drone_Standalone_Test/ZETIN_Drone_Standalone_Test.ino` | `firmware/archive/dshot/motor_allocator_dual_core/motor_allocator_dual_core.ino` | 보관 |
| `firmware/examples/MOTOR_ALGO_TEST/MOTOR_ALGO_TEST.ino` | `firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/motor_allocator_dshot.ino` | 보관 |
| `firmware/examples/BMM_TEST/BMM_TEST.ino` | `firmware/archive/other_sensors/bmm350_read/bmm350_read.ino` | 보관 |
| `firmware/examples/PCB_1.5.2_BMM_TEST/PCB_1.5.2_BMM_TEST.ino` | `firmware/archive/other_sensors/board_v1_5_2_bmm350/board_v1_5_2_bmm350.ino` | 보관 |
| `firmware/examples/GPS_TEST_RAW/GPS_TEST_RAW.ino` | `firmware/archive/other_sensors/gps_uart_passthrough/gps_uart_passthrough.ino` | 보관 |
| `firmware/examples/US100_TEST/US100_TEST.ino` | `firmware/archive/other_sensors/us100_distance/us100_distance.ino` | 보관 |
| `firmware/examples/F411_test/F411_test.ino` | `firmware/archive/other_mcus/stm32_f411_uart_rx/stm32_f411_uart_rx.ino` | 보관 |
| `firmware/src` | `firmware/archive/platformio_skeleton/src` | 보관 |
| `firmware/lib` | `firmware/archive/platformio_skeleton/lib` | 보관 |
| `firmware/include` | `firmware/archive/platformio_skeleton/include` | 보관 |
| `firmware/platformio_config/platformio.ini` | `firmware/archive/platformio_skeleton/platformio.ini` | 보관 |
| `test/README` | `firmware/archive/platformio_skeleton/test/README` | 보관 |
| `scripts/Drone_Control_Dualsense.py` | `scripts/ground/control_station.py` | 현행 |
| `scripts/Drone_Tuning.py` | `scripts/drone_control.py` | 현행 |
| `scripts/Drone_Reciever.py` | `scripts/drone_control.py` | 현행 |
| `scripts/Drone_Monitor.py` | `scripts/monitor_telemetry.py` | 현행 |
| `scripts/Drone_Analasys.py` | `scripts/ground/flight_log.py` | 현행 |
| `scripts/drone_telemetry.py` | `scripts/ground/telemetry_schema.py` | 현행 |
| `scripts/dual_imu_pid_debug_receiver.py` | `scripts/receive_dual_imu_debug.py` | 현행 |
| `scripts/Controller_test.py` | `scripts/test_dualsense_input.py` | 현행 |
| `scripts/GPS_Reciever.py` | `scripts/archive/receive_gps_udp_legacy.py` | 보관 |
| `test/tcp_test.py` | `scripts/archive/test_tcp_legacy.py` | 보관 |
| `docs/ONBOARDING.md` | `docs/project_overview.md` | 현행 |
| `docs/superpowers/specs/2026-05-14-dual-imu-pid-design.md` | `docs/history/2026-05-14-dual-imu-pid-design.md` | 과거 |
| `docs/superpowers/plans/2026-05-14-dual-imu-pid.md` | `docs/history/2026-05-14-dual-imu-pid-implementation-plan.md` | 과거 |
| `docs/presentation.c` | `docs/archive/pid_dshot_presentation_snippet.c` | 보관 |
| `docs/commit_message.txt` | `docs/archive/commit_message_snapshot.txt` | 보관 |
| `test/README.md` | `scripts/archive/README.md` | 보관 메모로 통합 |

머신 로컬 파일인 `.gemini/settings.json`, `.vscode/extensions.json`,
`.vscode/settings.json`은 버전 관리에서 제거됐다. 이 디렉터리들은 `.codex/`,
`.claude/`, `AGENTS.md`와 함께 의도적으로 무시된다.
