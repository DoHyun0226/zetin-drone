# 저장소 정리 설계

**날짜:** 2026-07-13  
**상태:** 구현 계획 승인됨

## 1. 목표

독자가 현행 ESP32-S3 + ICM42670 + PWM 경로를 폐기된 실험들과 즉시
구분할 수 있도록 저장소를 재구성하되, 후자는 명시적인 보관소 아래에
보존한다. 마이그레이션 이후에는 유지되는 모든 문서와 명령이 실제로
존재하는 파일이나 디렉터리를 가리켜야 한다.

## 2. 범위와 결정 사항

- 지원되는 유일한 하드웨어/제어 스택은 ESP32-S3 + ICM42670 + PWM이다.
- `DUAL_IMU_CASCADE`는 현행 비행 제어 후보다. 하드웨어 비행 검증으로
  달리 입증되기 전까지는 실험적 상태로 남는다.
- 현행 진단은 PWM 모터 벤치, ICM42670 원시 데이터 리더, 듀얼 IMU 루프
  진단, 그리고 PCB v1.5.2 듀얼 IMU 진단이다.
- DShot, MPU6500, STM32 F411, 칼만 실험, BMM350, GPS, US100, 구형
  PlatformIO 골격, TCP 테스트, 대체된 PID 변형들은 폐기된다.
- 폐기된 코드는 `git mv`로 옮겨 `archive/` 아래에 보존한다. 이 코드는
  수리·현대화하거나 지원 빌드 매트릭스에 포함하지 않는다.
- Arduino CLI가 지원되는 펌웨어 빌드 경로다. 낡은 PlatformIO
  프로젝트는 복원하지 않고 역사적 자료로 보관한다.
- Python 도구는 명령이나 텔레메트리 동작을 바꾸지 않은 채 서술적인
  `lower_snake_case` 이름으로 개명한다.
- UDP 명령 프로토콜과 10/14/21 필드 텔레메트리 호환성은 그대로
  유지된다.
- 구형 이름으로 된 호환성 심링크나 중복 래퍼 스크립트는 남기지
  않는다. `docs/migration_map.md`가 이전-신규 조회의 권위 있는 출처다.
- 로컬 AI 에이전트 및 머신 특정 에디터 파일은 추적에서 제거하고
  저장소 검사에서 제외한다.

## 3. 명명 규칙

1. 디렉터리와 소스 파일은 `lower_snake_case`를 사용한다.
2. Arduino 스케치 디렉터리와 그 주 `.ino` 기본 이름은 일치해야 한다.
3. 이름은 시간 순서가 아니라 대상과 목적을 서술한다: `NEW_DUAL_TEST`가
   아니라 `icm42670_dual_raw`를 사용한다.
4. 버전 구두점은 밑줄로 바꾼다: `board_v1_5_2_dual_imu`를 사용한다.
5. 수명주기 상태는 `_TEST` 같은 모호한 접미사가 아니라 디렉터리 배치
   (`flight/`, `diagnostics/`, `archive/`)로 표현한다.
6. 보관된 안전하지 않은 코드는 명시적인 이름과 보관 경고를 사용한다.
   실행 가능한 빠른 시작 예제로 절대 제시하지 않는다.

## 4. 목표 저장소 구조

```text
zetin-drone/
├── README.md
├── firmware/
│   ├── README.md
│   ├── flight/
│   │   └── dual_imu_cascade_pwm/
│   │       └── dual_imu_cascade_pwm.ino
│   ├── diagnostics/
│   │   ├── motor_pwm_bench/
│   │   ├── icm42670_single_raw/
│   │   ├── icm42670_dual_raw/
│   │   ├── icm42670_dual_loop_debug/
│   │   └── board_v1_5_2_dual_imu/
│   └── archive/
│       ├── README.md
│       ├── legacy_flight/
│       ├── filter_experiments/
│       ├── dshot/
│       ├── other_sensors/
│       ├── other_mcus/
│       └── platformio_skeleton/
├── scripts/
│   ├── README.md
│   ├── control_dualsense.py
│   ├── tune_pid.py
│   ├── receive_telemetry.py
│   ├── monitor_telemetry.py
│   ├── analyze_flight_log.py
│   ├── telemetry_schema.py
│   ├── receive_dual_imu_debug.py
│   ├── test_dualsense_input.py
│   └── archive/
├── logs/
│   └── README.md
├── docs/
│   ├── README.md
│   ├── project_overview.md
│   ├── firmware_catalog.md
│   ├── udp_protocol.md
│   ├── migration_map.md
│   ├── design/
│   ├── history/
│   └── archive/
└── tools/
    └── check_repo_layout.py
```

## 5. 펌웨어 마이그레이션 맵

### 5.1 현행 비행 및 진단

| 이전 경로 | 새 경로 | 역할 |
|---|---|---|
| `firmware/examples/DUAL_IMU_CASCADE/DUAL_IMU_CASCADE.ino` | `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino` | 현행 비행 제어 후보 |
| `firmware/examples/PWM_TEST/PWM_TEST.ino` | `firmware/diagnostics/motor_pwm_bench/motor_pwm_bench.ino` | 4모터 PWM 벤치 테스트 |
| `firmware/examples/IMU_TEST_RAW/IMU_TEST_RAW.ino` | `firmware/diagnostics/icm42670_single_raw/icm42670_single_raw.ino` | 단일 ICM42670 원시 SPI 리더; 핀 배치 명시 문서화 |
| `firmware/examples/DUAL_IMU_RAW_TEST/DUAL_IMU_RAW_TEST.ino` | `firmware/diagnostics/icm42670_dual_raw/icm42670_dual_raw.ino` | 듀얼 ICM42670 축/부호 리더 |
| `firmware/examples/DUAL_IMU_PID_DEBUG/DUAL_IMU_PID_DEBUG.ino` | `firmware/diagnostics/icm42670_dual_loop_debug/icm42670_dual_loop_debug.ino` | 듀얼 IMU 루프 타이밍/필터 진단 |
| `firmware/examples/PCB_1.5.2_TEST/PCB_1.5.2_TEST.ino` | `firmware/diagnostics/board_v1_5_2_dual_imu/board_v1_5_2_dual_imu.ino` | PCB v1.5.2 듀얼 IMU 스모크 테스트 |

### 5.2 폐기된 비행 및 필터 실험

| 이전 경로 | 보관 경로 |
|---|---|
| `firmware/examples/PWM_TEST_IMU_PID/PWM_TEST_IMU_PID.ino` | `firmware/archive/legacy_flight/single_imu_pid_pwm/single_imu_pid_pwm.ino` |
| `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino` | `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino` |
| `firmware/examples/DUAL_LOOP_TEST/DUAL_LOOP_TEST.ino` | `firmware/archive/legacy_flight/single_imu_cascade_pwm/single_imu_cascade_pwm.ino` |
| `firmware/examples/KALMAN_FLIGHT_TEST/KALMAN_FLIGHT_TEST.ino` | `firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/single_imu_kalman_pid_pwm.ino` |
| `firmware/examples/KALMAN_TEST/KALMAN_TEST.ino` | `firmware/archive/filter_experiments/icm42670_kalman_attitude/icm42670_kalman_attitude.ino` |

### 5.3 폐기된 DShot 및 모터 할당 실험

| 이전 경로 | 보관 경로 |
|---|---|
| `firmware/examples/DSHOT_TEST/DSHOT_TEST.ino` | `firmware/archive/dshot/single_motor_fixed_throttle/single_motor_fixed_throttle.ino` |
| `firmware/examples/DSHOT_TEST_PERCENT/DSHOT_TEST_PERCENT.ino` | `firmware/archive/dshot/single_motor_serial_control/single_motor_serial_control.ino` |
| `firmware/examples/DSHOT_TEST_IMU/DSHOT_TEST_IMU.ino` | `firmware/archive/dshot/mpu6500_tilt_throttle/mpu6500_tilt_throttle.ino` |
| `firmware/examples/DSHOT_TEST_IMU_PID/DSHOT_TEST_IMU_PID.ino` | `firmware/archive/dshot/mpu6500_pid_bench/mpu6500_pid_bench.ino` |
| `firmware/examples/RMT_TEST/RMT_TEST.ino` | `firmware/archive/dshot/four_motor_full_throttle_unsafe/four_motor_full_throttle_unsafe.ino` |
| `firmware/examples/ZETIN_Drone_Standalone_Test/ZETIN_Drone_Standalone_Test.ino` | `firmware/archive/dshot/motor_allocator_dual_core/motor_allocator_dual_core.ino` |
| `firmware/examples/MOTOR_ALGO_TEST/MOTOR_ALGO_TEST.ino` | `firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/motor_allocator_dshot.ino` |

DShot 보관 README는 누락된 의존성과 안전하지 않은 최대 스로틀
스케치를 반드시 명시해야 한다. 이 스케치들 중 어느 것도 검증의
일부가 아니다.

### 5.4 폐기된 센서 및 MCU 대상

| 이전 경로 | 보관 경로 |
|---|---|
| `firmware/examples/BMM_TEST/BMM_TEST.ino` | `firmware/archive/other_sensors/bmm350_read/bmm350_read.ino` |
| `firmware/examples/PCB_1.5.2_BMM_TEST/PCB_1.5.2_BMM_TEST.ino` | `firmware/archive/other_sensors/board_v1_5_2_bmm350/board_v1_5_2_bmm350.ino` |
| `firmware/examples/GPS_TEST_RAW/GPS_TEST_RAW.ino` | `firmware/archive/other_sensors/gps_uart_passthrough/gps_uart_passthrough.ino` |
| `firmware/examples/US100_TEST/US100_TEST.ino` | `firmware/archive/other_sensors/us100_distance/us100_distance.ino` |
| `firmware/examples/F411_test/F411_test.ino` | `firmware/archive/other_mcus/stm32_f411_uart_rx/stm32_f411_uart_rx.ino` |

### 5.5 폐기된 PlatformIO 골격

다음 항목들은 역사적 관계가 보존되도록
`firmware/archive/platformio_skeleton/` 아래로 함께 옮긴다:

- `firmware/src/`
- `firmware/lib/`
- `firmware/include/`
- `firmware/platformio_config/platformio.ini`
- 생성된 PlatformIO `test/README`

보관 README는 이 프로젝트가 시뮬레이션된 센서 데이터, 취소된 모터
할당기, 구형 TCP 전송, 낡은 의존성을 담고 있으며 지원되는 빌드
보장이 없음을 명시한다.

## 6. Python 마이그레이션 맵

| 이전 경로 | 새 경로 | 필요한 참조 업데이트 |
|---|---|---|
| `scripts/Drone_Control_Dualsense.py` | `scripts/control_dualsense.py` | README 명령과 펌웨어 주석 |
| `scripts/Drone_Tuning.py` | `scripts/tune_pid.py` | README 명령, 펌웨어 주석, 역사 문서 |
| `scripts/Drone_Reciever.py` | `scripts/receive_telemetry.py` | README 명령과 텔레메트리 import |
| `scripts/Drone_Monitor.py` | `scripts/monitor_telemetry.py` | README 명령과 텔레메트리 import |
| `scripts/Drone_Analasys.py` | `scripts/analyze_flight_log.py` | README와 로그 문서 |
| `scripts/drone_telemetry.py` | `scripts/telemetry_schema.py` | 리시버와 모니터의 import |
| `scripts/dual_imu_pid_debug_receiver.py` | `scripts/receive_dual_imu_debug.py` | 독스트링과 카탈로그의 짝 펌웨어 경로 |
| `scripts/Controller_test.py` | `scripts/test_dualsense_input.py` | 스크립트 카탈로그 |
| `scripts/GPS_Reciever.py` | `scripts/archive/receive_gps_udp_legacy.py` | 보관 카탈로그만 |
| `test/tcp_test.py` | `scripts/archive/test_tcp_legacy.py` | 보관 카탈로그만 |

개명 과정에서 `Reciever`는 `receive`로, `Analasys`는 `analyze`로
바로잡는다. 오탈자는 새 이름에 보존하지 않는다.

## 7. 문서 아키텍처

### 7.1 유지되는 문서

- `README.md`: 짧은 현행 상태 진입점이자 빠른 시작. 지원되는 비행,
  진단, 도구, 그리고 문서 색인으로만 링크한다.
- `docs/README.md`: 유지되는 것, 역사적인 것, 보관된 것으로 묶은 완전한
  문서 색인.
- `docs/project_overview.md`: 개명하고 바로잡은 온보딩 문서. 검증된
  것과 실험적인 것 사이의 경계를 보존한다.
- `docs/firmware_catalog.md`: 현행 및 보관된 스케치마다 한 행씩, 대상
  MCU, 센서, 모터 프로토콜, 수명주기, 안전 유의 사항, 정확한 링크를
  담는다.
- `docs/udp_protocol.md`: 권위 있는 명령 문법, UDP 포트, 핸드셰이크,
  그리고 10/14/21 필드 텔레메트리 스키마.
- `docs/migration_map.md`: 펌웨어, 스크립트, 테스트, 문서에 대한 완전한
  이전 경로-신규 경로 매핑.
- `firmware/README.md`: Arduino CLI 빌드 지침과 현행 스케치. 보관된
  PlatformIO 골격을 현행으로 제시해서는 안 된다.
- `scripts/README.md`: 의존성과 개명된 스크립트를 사용한 정확한 명령.
- `logs/README.md`: 실제 `flight_log_YYYY-MM-DD_HHMMSS.csv` 규칙과
  Timestamp를 포함한 실제 22열 CSV 스키마.

### 7.2 역사 및 보관 문서

- 2026-05-14 듀얼 IMU PID 설계 및 구현 계획을 `docs/superpowers/`에서
  `docs/history/`로 옮기고, 정확한 파일 링크를 업데이트하며, 결과 PID
  펌웨어를 현행 캐스케이드 경로로 대체된 것으로 표시한다.
- `docs/presentation.c`를 `docs/archive/pid_dshot_presentation_snippet.c`로
  옮기고 실행 불가로 표기한다.
- `docs/commit_message.txt`를 `docs/archive/commit_message_snapshot.txt`로
  옮긴다. 이것은 커밋 규약이 아니다.
- `test/README.md`를 `scripts/archive/README.md`로 합친 뒤, 비어 있는
  최상위 `test/` 디렉터리를 제거한다.

### 7.3 참조 규칙

- 유지되는 문서 내의 정확한 저장소 경로는 해석 가능한 Markdown 링크를
  사용한다.
- 코드 예제는 백틱을 사용해도 되지만, 탐색을 위한 경로는 링크도
  가져야 한다.
- 구형 경로는 `docs/migration_map.md`와 명확히 역사적이라고 표기된
  설명에서만 허용된다.
- Python 도구를 지칭하는 펌웨어 소스 주석은 새 경로로 업데이트한다.
- 짝 펌웨어를 지칭하는 Python 독스트링은 새 경로로 업데이트한다.
- 머신 로컬 `.codex`, `.claude`, `.gemini`, `.vscode`, `AGENTS.md`
  내용은 문서 검사 범위 밖이다.

## 8. 로컬 및 생성 파일

추적되는 `.gemini/`와 `.vscode/` 설정을 제거한다. 후자는 머신 특정
Windows 경로와 구형 `ZETIN_Drone` 저장소 이름을 담고 있다. `.gitignore`를
확장해 다음을 포함한다:

- `.codex/`, `.claude/`, `.gemini/`
- 저장소 어느 깊이에 있든 `AGENTS.md`
- `.vscode/`
- `__pycache__/`, `*.pyc`
- `*.swp`와 `*.kate-swp`를 포함한 에디터 스왑 파일
- 기존 PlatformIO 및 CSV 출력 패턴

이 경로들은 마이그레이션되지도, 저장소 검증기에 의해 검사되지도
않는다.

## 9. 저장소 검증

`tools/check_repo_layout.py`를 네 가지 결정론적 검사와 함께 만든다:

1. `README.md`, `docs/`, `firmware/README.md`, `scripts/README.md`,
   `logs/README.md`의 로컬 Markdown 링크를 각 소스 파일 기준으로
   해석한다.
2. 보관된 스케치를 포함해 모든 Arduino 스케치 디렉터리가 그 `.ino`
   기본 이름과 일치하는지 확인한다.
3. `docs/migration_map.md`의 모든 이전/신규 쌍이 마이그레이션 이후
   누락된 이전 경로 하나와 존재하는 새 경로 하나를 갖는지 확인한다.
4. 유지되는 문서와 현행 소스에서 구형 경로/이름 토큰을 거부하되,
   마이그레이션 맵과 명시적으로 역사적인 문서에서는 허용한다.

모든 이동 후의 검증:

```bash
/home/light/anaconda3/bin/python tools/check_repo_layout.py
/home/light/anaconda3/bin/python -m py_compile scripts/*.py
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-flight-build \
  firmware/flight/dual_imu_cascade_pwm
```

`firmware/diagnostics/` 아래의 각 디렉터리를 별도의 `/tmp` 빌드
경로로 컴파일한다. 정확히 10, 14, 21개 입력 필드에 대한 텔레메트리
파서 케이스를 실행하고 Timestamp를 포함한 22열 CSV 행을 검증한다.
`firmware/archive/` 아래의 어떤 것도 컴파일하지 않는다.

최종 저장소 검사:

- `git status --short`에 추적되지 않는 AI/에디터/생성 파일이 없다.
- `git diff --check`가 공백 오류를 보고하지 않는다.
- 로컬 `main`과 `origin/main`은 정리 커밋이 완료되고 푸시가 명시적으로
  승인된 후에만 비교한다.

## 10. 커밋 전략

1. `chore: ignore local agent and editor files`
   - 추적되는 `.gemini/`와 `.vscode/`를 제거하고 `.gitignore`를
     업데이트한다.
2. `refactor: separate current and archived firmware`
   - 모든 스케치와 PlatformIO 골격을 옮긴다. 경로가 설명 가능하도록
     같은 커밋에서 펌웨어 보관 및 카탈로그 문서를 추가한다.
3. `refactor: rename drone utility scripts`
   - 현행 도구를 개명하고, Python import와 짝 펌웨어 주석을
     업데이트하며, GPS/TCP 도구를 보관한다.
4. `docs: align project documentation with repository layout`
   - 유지되는 모든 문서를 업데이트하고, 역사적 자료를 옮기며, 완전한
     마이그레이션 맵을 추가한다.
5. `test: add repository layout validation`
   - 링크/레이아웃 검사기와 파서 케이스를 추가한 뒤, 지원되는 전체
     검증 세트를 실행한다.

어떤 정리 커밋도 컨트롤러 게인, 핀 배치, 모터 믹싱, 텔레메트리 필드
순서, UDP 명령, 보관된 구현 로직을 변경하지 않는다.

## 11. 인수 기준

- 루트 README에서 현행 비행 스케치에 두 번의 링크 이내로 도달할 수
  있으며 실험적인 것으로 식별된다.
- 지원되는 모든 스크립트 명령과 펌웨어 빌드 명령이 존재하는 경로를
  지칭한다.
- 유지되는 모든 로컬 Markdown 링크가 해석된다.
- 23개의 원본 스케치 전부가 마이그레이션 맵에 정확히 한 번, 정확히
  한 목적지에 나타난다.
- 현행 펌웨어는 `firmware/flight/`와 `firmware/diagnostics/`에만
  존재하고, 폐기된 펌웨어는 `firmware/archive/`에만 존재한다.
- 보관 README는 보관된 코드가 지원되지 않으며 빌드 검증되지 않았음을
  명시한다.
- 현행 캐스케이드 스케치와 모든 현행 진단이 의도된 ESP32-S3 대상으로
  컴파일된다.
- 현행 Python 스크립트가 컴파일되고 개명된 import가 해석된다.
- 텔레메트리 파싱이 10, 14, 21 필드 패킷과 호환성을 유지한다.
- CSV 로거가 여전히 Timestamp와 21개의 텔레메트리 필드를 기록한다.
- 마이그레이션 맵 밖의 유지되는 문서에 구형 경로가 남지 않는다.
- AI 에이전트, 에디터, 캐시, 스왑, 생성된 로그 파일이 Git 밖에
  머문다.
