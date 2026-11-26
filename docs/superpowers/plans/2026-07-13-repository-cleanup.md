# 저장소 정리 구현 계획

> **에이전트 작업자용:** 필수 하위 스킬: 이 계획을 작업 단위로 구현하려면
> `superpowers:subagent-driven-development`(권장) 또는
> `superpowers:executing-plans`를 사용한다. 단계는 추적을 위해
> 체크박스(`- [ ]`) 구문을 사용한다.

**목표:** 지원되는 ESP32-S3 + ICM42670 + PWM 경로를 명확하게 드러내고, 그 외
모든 펌웨어와 도구는 명확히 지원되지 않는 보관 영역 아래에 보존하며, 유지되는
모든 문서가 실제 존재하는 경로를 가리키도록 보장한다.

**아키텍처:** 현행 캐스케이드 컨트롤러와 하드웨어 진단을 `firmware/flight/`와
`firmware/diagnostics/`로 승격한다. 그 외 모든 스케치와 오래된 PlatformIO
프로젝트는 수리하지 않고 `firmware/archive/`로 옮긴다. 현행 Python 도구를
제자리에서 이름을 바꾸고, 레거시 네트워크 도구를 보관하며, 마이그레이션 맵을
중심으로 문서를 재구성하고, 그 결과를 로컬 저장소 레이아웃 검증기로
강제한다.

**기술 스택:** Git, Arduino CLI, ESP32-S3 Arduino 코어, ICM42670P,
ESP32Servo/LEDC PWM, Python 3 표준 라이브러리, pandas, matplotlib, pygame.

**설계:**
[`docs/design/2026-07-13-repository-cleanup-design.md`](../specs/2026-07-13-repository-cleanup-design.md)

## 전역 제약

- 지원 스택: ESP32-S3 + ICM42670 + PWM 전용.
- `dual_imu_cascade_pwm`는 현행 비행 제어 후보이며, 안정적인 비행이나 호버를
  보장하는 것은 아니다.
- 검증된 핀 배치, 모터 믹서 부호, 기체축 매핑, UDP 명령 문법, 텔레메트리 필드
  순서를 보존한다.
- 10, 14, 21 필드 텔레메트리 패킷과의 호환성을 보존한다.
- `firmware/archive/` 또는 `scripts/archive/` 아래의 어떤 것도 수리, 현대화,
  컴파일하거나 지원을 주장하지 않는다.
- 이력을 복구 가능하게 유지하도록 추적된 이동에는 `git mv`를 사용한다.
- Arduino 스케치 디렉토리 이름은 해당 `.ino` 기본 이름과 정확히 일치해야 한다.
- 현행 소스와 유지되는 문서는 `lower_snake_case` 경로를 사용한다.
- `.codex`, `.claude`, `.gemini`, `.vscode`, `AGENTS.md`, Python 캐시, 에디터
  스왑 파일, 생성된 CSV 로그를 커밋하지 않는다.
- 중간 커밋을 푸시하지 않는다. 완료된 정리가 검토되고 사용자가 명시적으로
  요청한 후에만 푸시한다.

---

### 작업 1: 로컬 에이전트, 에디터, 생성된 파일 제외

**파일:**

- 수정: `.gitignore`
- Git에서 삭제: `.gemini/settings.json`
- Git에서 삭제: `.vscode/extensions.json`
- Git에서 삭제: `.vscode/settings.json`

**인터페이스:**

- 입력: 현행 저장소 루트.
- 출력: Git이 프로젝트 파일은 유지하면서 로컬 에이전트/에디터 상태를 무시한다.

- [ ] **단계 1: `.gitignore` 변경 전 무시 어서션 실행**

실행:

```bash
for candidate in \
  .codex/probe \
  .claude/probe \
  .gemini/settings.json \
  .vscode/settings.json \
  AGENTS.md \
  firmware/examples/AGENTS.md \
  scripts/__pycache__/probe.pyc \
  scratch.swp \
  scratch.kate-swp; do
  git check-ignore --no-index -q "$candidate" || {
    echo "NOT_IGNORED $candidate"
    exit 1
  }
done
```

예상: 최소한 `.codex/probe`에서 FAIL; 이는 현행 무시 파일이 정리 요구사항을
충족하지 못함을 증명한다.

- [ ] **단계 2: 정확한 로컬/생성 패턴으로 `.gitignore` 확장**

`apply_patch`를 사용해 다음 블록을 추가한다:

```gitignore

# Local AI agents and project instructions
.codex/
.claude/
.gemini/
AGENTS.md

# Machine-specific editor state
.vscode/
*.swp
*.kate-swp

# Python generated files
__pycache__/
*.py[cod]
```

기존 `.pio` 및 `*.csv` 규칙은 유지한다.

- [ ] **단계 3: 추적된 로컬 설정 제거**

```bash
git rm -r .gemini .vscode
```

예상: `.gemini/settings.json`, `.vscode/extensions.json`,
`.vscode/settings.json`가 삭제 대상으로 스테이징된다.

- [ ] **단계 4: 단계 1 무시 어서션 재실행**

예상: `NOT_IGNORED` 출력 없이 exit 0. 그런 다음 실행:

```bash
git status --short --ignored | rg '!! (\.codex/|AGENTS\.md|firmware/examples/\.codex/|firmware/examples/AGENTS\.md)'
```

예상: 해당 로컬 경로가 체크아웃에 존재한다면 오직 `!!`로만 나타나며, 절대
`??`로는 나타나지 않는다. 로컬 전용 경로가 없는 격리된 워크트리에서는 출력이
없는 것도 유효하다; 위의 가상 경로 어서션이 여전히 권위 있는 검사로
남는다.

- [ ] **단계 5: 로컬 상태 정리 커밋**

```bash
git add .gitignore
git commit -m "chore: ignore local agent and editor files"
```

### 작업 2: 현행 펌웨어를 보관 영역과 분리

**파일:**

- 이동: 현재 `firmware/examples/` 아래의 23개 디렉토리 전부
- 이동: `firmware/src/`, `firmware/lib/`, `firmware/include/`
- 이동: `firmware/platformio_config/platformio.ini`
- 이동: `test/README`
- 수정: `firmware/README.md`
- 생성: `firmware/archive/README.md`
- 생성: `firmware/archive/dshot/README.md`
- 생성: `firmware/archive/platformio_skeleton/README.md`

**인터페이스:**

- 현행 비행 스케치 출력:
  `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`.
- `firmware/diagnostics/` 아래에 다섯 개의 진단 스케치 루트를 출력한다.
- 지원되지 않는 역사적 자료는 오직 `firmware/archive/` 아래에만 출력한다.
- 이후 작업은 소스 주석과 문서를 위해 이 정확한 대상 경로에 의존한다.

- [ ] **단계 1: 마이그레이션 전 펌웨어 목록 기록**

```bash
git ls-files 'firmware/examples/**/*.ino' | sort > /tmp/zetin-firmware-before.txt
test "$(wc -l < /tmp/zetin-firmware-before.txt)" -eq 23
```

예상: exit 0과 23개의 스케치 경로.

- [ ] **단계 2: 수명주기/범주 상위 디렉토리 생성**

```bash
mkdir -p \
  firmware/flight \
  firmware/diagnostics \
  firmware/archive/legacy_flight \
  firmware/archive/filter_experiments \
  firmware/archive/dshot \
  firmware/archive/other_sensors \
  firmware/archive/other_mcus \
  firmware/archive/platformio_skeleton/examples \
  firmware/archive/platformio_skeleton/test
```

- [ ] **단계 3: 현행 스케치 6개 이동 및 이름 변경**

```bash
git mv firmware/examples/DUAL_IMU_CASCADE firmware/flight/dual_imu_cascade_pwm
git mv firmware/flight/dual_imu_cascade_pwm/DUAL_IMU_CASCADE.ino \
  firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino

git mv firmware/examples/PWM_TEST firmware/diagnostics/motor_pwm_bench
git mv firmware/diagnostics/motor_pwm_bench/PWM_TEST.ino \
  firmware/diagnostics/motor_pwm_bench/motor_pwm_bench.ino

git mv firmware/examples/IMU_TEST_RAW firmware/diagnostics/icm42670_single_raw
git mv firmware/diagnostics/icm42670_single_raw/IMU_TEST_RAW.ino \
  firmware/diagnostics/icm42670_single_raw/icm42670_single_raw.ino

git mv firmware/examples/DUAL_IMU_RAW_TEST firmware/diagnostics/icm42670_dual_raw
git mv firmware/diagnostics/icm42670_dual_raw/DUAL_IMU_RAW_TEST.ino \
  firmware/diagnostics/icm42670_dual_raw/icm42670_dual_raw.ino

git mv firmware/examples/DUAL_IMU_PID_DEBUG firmware/diagnostics/icm42670_dual_loop_debug
git mv firmware/diagnostics/icm42670_dual_loop_debug/DUAL_IMU_PID_DEBUG.ino \
  firmware/diagnostics/icm42670_dual_loop_debug/icm42670_dual_loop_debug.ino

git mv firmware/examples/PCB_1.5.2_TEST firmware/diagnostics/board_v1_5_2_dual_imu
git mv firmware/diagnostics/board_v1_5_2_dual_imu/PCB_1.5.2_TEST.ino \
  firmware/diagnostics/board_v1_5_2_dual_imu/board_v1_5_2_dual_imu.ino
```

- [ ] **단계 4: 대체된 비행/필터 스케치 보관**

```bash
git mv firmware/examples/PWM_TEST_IMU_PID \
  firmware/archive/legacy_flight/single_imu_pid_pwm
git mv firmware/archive/legacy_flight/single_imu_pid_pwm/PWM_TEST_IMU_PID.ino \
  firmware/archive/legacy_flight/single_imu_pid_pwm/single_imu_pid_pwm.ino

git mv firmware/examples/PWM_TEST_DUAL_IMU_PID \
  firmware/archive/legacy_flight/dual_imu_pid_pwm
git mv firmware/archive/legacy_flight/dual_imu_pid_pwm/PWM_TEST_DUAL_IMU_PID.ino \
  firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino

git mv firmware/examples/DUAL_LOOP_TEST \
  firmware/archive/legacy_flight/single_imu_cascade_pwm
git mv firmware/archive/legacy_flight/single_imu_cascade_pwm/DUAL_LOOP_TEST.ino \
  firmware/archive/legacy_flight/single_imu_cascade_pwm/single_imu_cascade_pwm.ino

git mv firmware/examples/KALMAN_FLIGHT_TEST \
  firmware/archive/legacy_flight/single_imu_kalman_pid_pwm
git mv firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/KALMAN_FLIGHT_TEST.ino \
  firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/single_imu_kalman_pid_pwm.ino

git mv firmware/examples/KALMAN_TEST \
  firmware/archive/filter_experiments/icm42670_kalman_attitude
git mv firmware/archive/filter_experiments/icm42670_kalman_attitude/KALMAN_TEST.ino \
  firmware/archive/filter_experiments/icm42670_kalman_attitude/icm42670_kalman_attitude.ino
```

- [ ] **단계 5: DShot 및 모터 할당 스케치 보관**

```bash
git mv firmware/examples/DSHOT_TEST \
  firmware/archive/dshot/single_motor_fixed_throttle
git mv firmware/archive/dshot/single_motor_fixed_throttle/DSHOT_TEST.ino \
  firmware/archive/dshot/single_motor_fixed_throttle/single_motor_fixed_throttle.ino

git mv firmware/examples/DSHOT_TEST_PERCENT \
  firmware/archive/dshot/single_motor_serial_control
git mv firmware/archive/dshot/single_motor_serial_control/DSHOT_TEST_PERCENT.ino \
  firmware/archive/dshot/single_motor_serial_control/single_motor_serial_control.ino

git mv firmware/examples/DSHOT_TEST_IMU \
  firmware/archive/dshot/mpu6500_tilt_throttle
git mv firmware/archive/dshot/mpu6500_tilt_throttle/DSHOT_TEST_IMU.ino \
  firmware/archive/dshot/mpu6500_tilt_throttle/mpu6500_tilt_throttle.ino

git mv firmware/examples/DSHOT_TEST_IMU_PID \
  firmware/archive/dshot/mpu6500_pid_bench
git mv firmware/archive/dshot/mpu6500_pid_bench/DSHOT_TEST_IMU_PID.ino \
  firmware/archive/dshot/mpu6500_pid_bench/mpu6500_pid_bench.ino

git mv firmware/examples/RMT_TEST \
  firmware/archive/dshot/four_motor_full_throttle_unsafe
git mv firmware/archive/dshot/four_motor_full_throttle_unsafe/RMT_TEST.ino \
  firmware/archive/dshot/four_motor_full_throttle_unsafe/four_motor_full_throttle_unsafe.ino

git mv firmware/examples/ZETIN_Drone_Standalone_Test \
  firmware/archive/dshot/motor_allocator_dual_core
git mv firmware/archive/dshot/motor_allocator_dual_core/ZETIN_Drone_Standalone_Test.ino \
  firmware/archive/dshot/motor_allocator_dual_core/motor_allocator_dual_core.ino

git mv firmware/examples/MOTOR_ALGO_TEST \
  firmware/archive/platformio_skeleton/examples/motor_allocator_dshot
git mv firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/MOTOR_ALGO_TEST.ino \
  firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/motor_allocator_dshot.ino
```

- [ ] **단계 6: 센서 및 대체 MCU 스케치 보관**

```bash
git mv firmware/examples/BMM_TEST \
  firmware/archive/other_sensors/bmm350_read
git mv firmware/archive/other_sensors/bmm350_read/BMM_TEST.ino \
  firmware/archive/other_sensors/bmm350_read/bmm350_read.ino

git mv firmware/examples/PCB_1.5.2_BMM_TEST \
  firmware/archive/other_sensors/board_v1_5_2_bmm350
git mv firmware/archive/other_sensors/board_v1_5_2_bmm350/PCB_1.5.2_BMM_TEST.ino \
  firmware/archive/other_sensors/board_v1_5_2_bmm350/board_v1_5_2_bmm350.ino

git mv firmware/examples/GPS_TEST_RAW \
  firmware/archive/other_sensors/gps_uart_passthrough
git mv firmware/archive/other_sensors/gps_uart_passthrough/GPS_TEST_RAW.ino \
  firmware/archive/other_sensors/gps_uart_passthrough/gps_uart_passthrough.ino

git mv firmware/examples/US100_TEST \
  firmware/archive/other_sensors/us100_distance
git mv firmware/archive/other_sensors/us100_distance/US100_TEST.ino \
  firmware/archive/other_sensors/us100_distance/us100_distance.ino

git mv firmware/examples/F411_test \
  firmware/archive/other_mcus/stm32_f411_uart_rx
git mv firmware/archive/other_mcus/stm32_f411_uart_rx/F411_test.ino \
  firmware/archive/other_mcus/stm32_f411_uart_rx/stm32_f411_uart_rx.ino
```

로컬 `firmware/examples/` 디렉토리가 여전히 무시된 `.codex/` 또는 `AGENTS.md`
파일을 포함하고 있다면 제거하지 않는다. 이동 후 Git은 추적된
`firmware/examples/` 콘텐츠를 기록하지 않는다.

- [ ] **단계 7: 오래된 PlatformIO 스켈레톤을 하나의 역사적 단위로 이동**

```bash
git mv firmware/src firmware/archive/platformio_skeleton/src
git mv firmware/lib firmware/archive/platformio_skeleton/lib
git mv firmware/include firmware/archive/platformio_skeleton/include
git mv firmware/platformio_config/platformio.ini \
  firmware/archive/platformio_skeleton/platformio.ini
rmdir firmware/platformio_config
git mv test/README firmware/archive/platformio_skeleton/test/README
```

그 소스, 인클루드, 의존성 목록을 편집하지 않는다.

- [ ] **단계 8: 보관 경고 작성 및 `firmware/README.md` 교체**

`firmware/archive/README.md` 생성:

```markdown
# Archived firmware

Everything below this directory is preserved for history only. It is not part
of the supported ESP32-S3 + ICM42670 + PWM build path, is not compile-verified,
and may contain missing dependencies, obsolete pinouts, or unsafe behavior.

Do not flash archived motor-control code without reviewing it completely and
removing propellers. Current firmware is linked from [`../README.md`](../README.md).
```

`firmware/archive/dshot/README.md` 생성:

```markdown
# Archived DShot experiments

These sketches depend on old or absent DShot/MPU6500 libraries. They are not
maintained. `four_motor_full_throttle_unsafe` commands all four motors near
full output and must never be treated as a normal bench test.
```

`firmware/archive/platformio_skeleton/README.md` 생성:

```markdown
# Archived PlatformIO skeleton

This project contains simulated sensor data, a canceled motor allocator, an
old TCP transport, and stale dependencies. It is retained only for history.
Supported firmware is built with Arduino CLI from `firmware/flight/` and
`firmware/diagnostics/`.
```

`firmware/README.md`를 현행 전용 가이드로 교체한다. 하나의 비행 및 다섯 개의
진단 디렉토리로의 링크, `/tmp` Arduino CLI 빌드 패턴, 보관 경고, 기존
프로펠러/극성 안전 규칙을 포함한다. 각 진단에 대한 실제 SPI/CS 핀 배치를
나열하며, 특히 핀이 PCB v1.5.2 진단과 다른 단일 IMU raw 스케치를
명시한다.

- [ ] **단계 9: 구조 검증 및 현행 펌웨어만 컴파일**

```bash
test "$(find firmware -type f -name '*.ino' | wc -l)" -eq 23

find firmware -type f -name '*.ino' -print0 | while IFS= read -r -d '' ino; do
  test "$(basename "$(dirname "$ino")")" = "$(basename "$ino" .ino)" || {
    echo "SKETCH_NAME_MISMATCH $ino"
    exit 1
  }
done

for sketch in \
  firmware/flight/dual_imu_cascade_pwm \
  firmware/diagnostics/motor_pwm_bench \
  firmware/diagnostics/icm42670_single_raw \
  firmware/diagnostics/icm42670_dual_raw \
  firmware/diagnostics/icm42670_dual_loop_debug \
  firmware/diagnostics/board_v1_5_2_dual_imu; do
  name=$(basename "$sketch")
  arduino-cli compile --warnings all \
    --fqbn esp32:esp32:esp32s3 \
    --build-path "/tmp/zetin-cleanup-$name" \
    "$sketch" || exit 1
done
```

예상: 23개의 이름 검사와 여섯 개의 현행 ESP32-S3 빌드가 통과한다. 서드파티
라이브러리 경고는 남아 있을 수 있으나, 소스 컴파일 오류는 남으면 안 된다.
`firmware/archive/` 아래의 어떤 것도 컴파일하지 않는다.

- [ ] **단계 10: 펌웨어 수명주기 분리 커밋**

```bash
git add firmware
git commit -m "refactor: separate current and archived firmware"
```

---

### 작업 3: 현행 Python 도구 이름 변경 및 레거시 네트워크 스크립트 보관

**파일:**

- 이름 변경: `scripts/` 아래의 현행 파일 8개
- 이동: `scripts/GPS_Reciever.py`
- 이동: `test/tcp_test.py`
- 콘텐츠를 합친 후 제거: `test/README.md`
- 생성: `scripts/archive/README.md`
- 임포트 수정 대상: `scripts/receive_telemetry.py`,
  `scripts/monitor_telemetry.py`
- 짝 경로 독스트링 수정 대상: `scripts/receive_dual_imu_debug.py`
- 지상 도구를 명명하는 현행 비행 펌웨어 주석 수정
- 교체: `scripts/README.md`

**인터페이스:**

- `TELEMETRY_FIELDS`, `CSV_FIELDS`, `parse_telemetry_packet`,
  `sample_to_csv_row`, `active_fault_names`를 변경 없이 내보내는
  `telemetry_schema` 모듈을 출력한다.
- 유지되는 모든 문서가 사용하는 실행 파일 경로를 출력한다.

- [ ] **단계 1: 새 이름이 아직 존재하지 않음을 증명**

```bash
test ! -e scripts/control_dualsense.py
test ! -e scripts/telemetry_schema.py
```

예상: exit 0.

- [ ] **단계 2: 현행 스크립트 이름 변경 및 레거시 스크립트 보관**

```bash
git mv scripts/Drone_Control_Dualsense.py scripts/control_dualsense.py
git mv scripts/Drone_Tuning.py scripts/tune_pid.py
git mv scripts/Drone_Reciever.py scripts/receive_telemetry.py
git mv scripts/Drone_Monitor.py scripts/monitor_telemetry.py
git mv scripts/Drone_Analasys.py scripts/analyze_flight_log.py
git mv scripts/drone_telemetry.py scripts/telemetry_schema.py
git mv scripts/dual_imu_pid_debug_receiver.py scripts/receive_dual_imu_debug.py
git mv scripts/Controller_test.py scripts/test_dualsense_input.py

mkdir -p scripts/archive
git mv scripts/GPS_Reciever.py scripts/archive/receive_gps_udp_legacy.py
git mv test/tcp_test.py scripts/archive/test_tcp_legacy.py
```

- [ ] **단계 3: `apply_patch`를 사용해 임포트와 짝 경로 갱신**

`scripts/receive_telemetry.py`와
`scripts/monitor_telemetry.py` 양쪽에서 다음을 교체한다:

```python
from drone_telemetry import (
```

다음으로:

```python
from telemetry_schema import (
```

`scripts/receive_dual_imu_debug.py`에서 짝 펌웨어 줄을 다음으로 교체한다:

```python
Pairs with
firmware/diagnostics/icm42670_dual_loop_debug/icm42670_dual_loop_debug.ino.
```

`firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`에서 두 개의
지상 도구 주석을 다음으로 교체한다:

```cpp
// scripts/control_dualsense.py 프로토콜 호환용
// scripts/tune_pid.py 명령 호환: P/I/D는 inner rate PID에 대응
```

소켓 상수, 패킷 포맷, 필드 순서, 게인, 핀 배치, 모터 믹서, 컨트롤러 동작을
변경하지 않는다.

- [ ] **단계 4: 스크립트 보관 경고 생성 및 `test/` 제거**

`scripts/archive/README.md` 생성:

```markdown
# Archived PC tools

These scripts target deprecated GPS/TCP experiments and are retained only for
history. They are not part of the current UDP telemetry/control workflow and
are not included in current Python verification.
```

`test/README.md`의 고유한 사실 기반 TCP 설명을 보관 README에 합친 다음,
실행한다:

```bash
git rm test/README.md
rmdir test
```

- [ ] **단계 5: `scripts/README.md`를 정확한 현행 명령으로 교체**

저장소 루트에서 다음 명령을 나열한다:

```bash
python scripts/control_dualsense.py
python scripts/tune_pid.py
python scripts/receive_telemetry.py
python scripts/monitor_telemetry.py
python scripts/analyze_flight_log.py [optional-log.csv]
python scripts/receive_dual_imu_debug.py
python scripts/test_dualsense_input.py
```

리시버와 모니터가 모두 `logs/`에 기록하고, UDP 4210을 사용하며,
`telemetry_schema.py`를 공유한다고 명시한다. 레거시 스크립트는 오직
`archive/README.md`를 통해서만 링크한다.

- [ ] **단계 6: 임포트 및 구문 검증**

```bash
/home/light/anaconda3/bin/python -m py_compile scripts/*.py
PYTHONPATH=scripts /home/light/anaconda3/bin/python -c \
  'from telemetry_schema import CSV_FIELDS, parse_telemetry_packet; assert len(CSV_FIELDS) == 22; assert parse_telemetry_packet(",".join(["0"] * 10))["Throttle"] == 0'
```

예상: 두 명령 모두 exit 0. `scripts/archive/*.py`를 현행 지원 검사에 포함하지
않는다.

- [ ] **단계 7: 스크립트 이름 변경 커밋**

```bash
git add scripts firmware/flight
git commit -m "refactor: rename drone utility scripts"
```

---

### 작업 4: 유지되는 문서 및 마이그레이션 이력 재구성

**파일:**

- 수정: `README.md`
- 이동: `docs/ONBOARDING.md` → `docs/project_overview.md`
- 교체: `docs/README.md`
- 생성: `docs/firmware_catalog.md`
- 생성: `docs/udp_protocol.md`
- 생성: `docs/migration_map.md`
- 교체: `logs/README.md`
- `docs/superpowers/` 파일 두 개를 `docs/history/`로 이동
- 보관: `docs/presentation.c`, `docs/commit_message.txt`
- 역사적 문서 배너와 정확한 경로 수정

**인터페이스:**

- 작업 5가 검증하는 유지되는 내비게이션 표면을 출력한다.
- `check_repo_layout.py::check_migration_map`가 사용하는 44개의 마이그레이션
  행을 출력한다.

- [ ] **단계 1: 문서를 유지/역사/보관 경로로 이동**

```bash
mkdir -p docs/history docs/archive
git mv docs/ONBOARDING.md docs/project_overview.md
git mv docs/superpowers/specs/2026-05-14-dual-imu-pid-design.md \
  docs/history/2026-05-14-dual-imu-pid-design.md
git mv docs/superpowers/plans/2026-05-14-dual-imu-pid.md \
  docs/history/2026-05-14-dual-imu-pid-implementation-plan.md
rmdir docs/superpowers/specs docs/superpowers/plans docs/superpowers
git mv docs/presentation.c docs/archive/pid_dshot_presentation_snippet.c
git mv docs/commit_message.txt docs/archive/commit_message_snapshot.txt
```

- [ ] **단계 2: 루트 README를 현행 전용 진입점으로 교체**

다음의 정확한 구조와 링크 집합을 사용한다:

```markdown
# ZETIN Drone

ESP32-S3, dual ICM42670 IMUs, and PWM ESC control are the current development
stack. Motor PWM and raw IMU acquisition are bench-verified; closed-loop
stabilization remains experimental.

## Start here

- [Project overview](docs/project_overview.md)
- [Current flight-controller candidate](firmware/flight/dual_imu_cascade_pwm/)
- [Firmware and diagnostic guide](firmware/README.md)
- [PC control and telemetry tools](scripts/README.md)
- [UDP protocol and telemetry schema](docs/udp_protocol.md)
- [Firmware lifecycle catalog](docs/firmware_catalog.md)

## Quick verification

```bash
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-flight-build \
  firmware/flight/dual_imu_cascade_pwm

/home/light/anaconda3/bin/python -m py_compile scripts/*.py
/home/light/anaconda3/bin/python tools/check_repo_layout.py
```

## Safety

Remove propellers for bench tests. Verify power polarity, pin assignments,
motor order, and correction direction before any restrained flight test.
Archived experiments are unsupported and may be unsafe.
```

- [ ] **단계 3: `docs/README.md`를 문서 색인으로 재작성**

```markdown
# Documentation

## Maintained
- [Project overview](project_overview.md)
- [Firmware catalog](firmware_catalog.md)
- [UDP protocol](udp_protocol.md)
- [Repository migration map](migration_map.md)
- [Repository cleanup design](design/2026-07-13-repository-cleanup-design.md)
- [Repository cleanup implementation plan](plans/2026-07-13-repository-cleanup.md)

## Historical
- [2026-05-14 dual-IMU PID design](history/2026-05-14-dual-imu-pid-design.md)
- [2026-05-14 dual-IMU PID implementation plan](history/2026-05-14-dual-imu-pid-implementation-plan.md)

## Archived
- [PID/DShot presentation snippet](archive/pid_dshot_presentation_snippet.c)
- [Old commit-message snapshot](archive/commit_message_snapshot.txt)
```

- [ ] **단계 4: `docs/project_overview.md`의 사실과 링크 갱신**

모든 변경을 하나의 `apply_patch` 검토로 적용한다:

1. 오래된 `firmware/examples/` 맵을 `flight/`, `diagnostics/`, `archive/`로
   교체한다.
2. 대표 컨트롤러를
   `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`로 가리킨다.
3. 모든 Python 도구 이름을 작업 3의 이름으로 교체한다.
4. Timestamp를 포함해 21개 UDP 필드와 22개 CSV 열을 설명한다; 명시적인
   10/14 필드 호환성 참고를 보존한다.
5. 오래된 `src/` 경고를 `firmware/archive/platformio_skeleton/README.md`로의
   링크로 교체한다.
6. BMM350, GPS, US100, DShot, Kalman, STM32, TCP, 오래된 PID 변형을 명시적인
   폐기/보관 섹션에 배치한다.
7. 성숙도 경계를 유지한다: PWM과 raw IMU는 벤치 검증되었으나, 안정적인
   폐루프 비행은 확립되지 않았다.
8. 현행 핀, 믹서, 기체축 사실을 변경 없이 유지한다.
9. 모든 내비게이션 코드 스팬 경로를 상대 Markdown 링크로 변환한다.

다음의 현행 도구 표를 정확히 사용한다:

```markdown
| Tool | Role |
|---|---|
| [`control_dualsense.py`](../scripts/control_dualsense.py) | DualSense command sender |
| [`tune_pid.py`](../scripts/tune_pid.py) | Manual UDP command and gain tuning |
| [`receive_telemetry.py`](../scripts/receive_telemetry.py) | Terminal receiver and CSV logger |
| [`monitor_telemetry.py`](../scripts/monitor_telemetry.py) | Live plots and CSV logger |
| [`analyze_flight_log.py`](../scripts/analyze_flight_log.py) | Offline CSV analysis |
| [`receive_dual_imu_debug.py`](../scripts/receive_dual_imu_debug.py) | Paired loop diagnostic receiver |
```

- [ ] **단계 5: 권위 있는 UDP 프로토콜 문서 생성**

`docs/udp_protocol.md`는 다음을 정의해야 한다:

```text
Transport: UDP
Drone address: 192.168.4.1
Port: 4210
Registration: any incoming packet identifies the ground-station endpoint;
              current receivers send "connect" periodically.
```

다음 명령을 정확히 문서화한다:

```text
start
stop
rc <seq> <roll> <pitch> <yaw>
th <microseconds>
yaw <0|1>
pa|ia|da <value>
pr|ir|dr <value>
pp|ip|dp <value>
py|iy|dy <value>
```

다음의 정확한 순서로 21개 텔레메트리 필드를 문서화한다:

```text
Roll, Pitch, Yaw,
Gyro_X, Gyro_Y, Gyro_Z,
Accel_X, Accel_Y, Accel_Z,
Throttle,
Fault_RC, Fault_Critical,
RC_Total_Pkts, RC_Dropped_Pkts,
Fault_IMU1, Fault_IMU2, Fault_Disagree,
Active_IMUs, Mixer_Scaled, Fault_Attitude, Calibration_OK
```

10 필드 패킷은 `Throttle`에서 끝나고, 14 필드 패킷은 `RC_Dropped_Pkts`에서
끝나며, 알 수 없는 레거시 값은 빈 CSV 셀이 되고, Timestamp는 지상 도구에서만
추가된다고 명시한다.

- [ ] **단계 6: 펌웨어 카탈로그 및 완전한 마이그레이션 맵 생성**

`docs/firmware_catalog.md`는 다음 열을 가진 정확히 23개의 행을 포함해야 한다:

```text
Lifecycle | Target MCU | Sensor | Motor protocol | Safety/build note | Link
```

작업 2 단계 3의 여섯 개 현행 대상을 `current`로, 작업 2 단계 4-6의 17개
대상을 `archived`로 사용한다. `four_motor_full_throttle_unsafe`를 명시적으로
안전하지 않음으로 표시한다. 보관된 스케치는 수리되거나 빌드 검증되지 않았다고
명시한다.

`docs/migration_map.md`는 첫 두 개의 테이블 열로 `Old path`와 `New path`를
사용해야 하며 정확히 다음을 포함해야 한다:

- 작업 2 단계 3-6의 23개 스케치 이동;
- 작업 2 단계 7의 다섯 개 PlatformIO/템플릿 이동;
- 작업 3 단계 2의 10개 Python/test 이동;
- 작업 4 단계 1의 다섯 개 문서 이동에 더해 합쳐진
  `test/README.md` → `scripts/archive/README.md` 관계;

예상 합계: 고유한 옛 경로 44개와 고유한 새 경로 44개. 제거된 `.gemini`/`.vscode`
파일은 `Old path`/`New path` 테이블 구문 없이 별도로 나열한다.

- [ ] **단계 7: 로그와 역사적 문서 수정**

`logs/README.md`를 실제 `flight_log_YYYY-MM-DD_HHMMSS.csv` 규칙, 두 개의
기록기와 분석기로의 링크, Timestamp에 더해 단계 5의 정확한 21개 필드, 그리고
10/14 필드 레거시 동작으로 교체한다. 배터리, 모터 출력, PID 출력 열에 대한
주장을 제거한다.

`docs/history/` 파일 두 개의 최상단에 다음을 삽입한다:

```markdown
> Historical document: this describes the superseded dual-IMU PID iteration.
> Current flight-controller candidate:
> [`dual_imu_cascade_pwm`](../../firmware/flight/dual_imu_cascade_pwm/).
> Archived result:
> [`dual_imu_pid_pwm`](../../firmware/archive/legacy_flight/dual_imu_pid_pwm/).
```

해당 역사적 파일 안의 모든 발생을 교체한다:

```text
firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino
→ firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino

firmware/examples/PWM_TEST_IMU_PID/PWM_TEST_IMU_PID.ino
→ firmware/archive/legacy_flight/single_imu_pid_pwm/single_imu_pid_pwm.ino

Drone_Tuning.py → scripts/tune_pid.py
Drone_Control_Dualsense.py → scripts/control_dualsense.py
```

역사적 컴파일 명령을 지원되지 않음으로 표시하고 실행하지 않는다.

- [ ] **단계 8: 검증기 실행 전 오래된 참조 감사 수행**

```bash
rg -n \
  'firmware/examples/|firmware/src/|firmware/platformio_config/|Drone_(Reciever|Analasys|Monitor|Tuning|Control_Dualsense)\.py|Controller_test\.py|GPS_Reciever\.py|ZETIN_Drone' \
  README.md docs/project_overview.md docs/README.md docs/firmware_catalog.md \
  docs/udp_protocol.md firmware/README.md scripts/README.md logs/README.md \
  firmware/flight firmware/diagnostics scripts \
  --glob '!scripts/archive/**'
```

예상: 출력 없음. 옛 이름은 오직 `docs/migration_map.md`, `docs/design/`,
`docs/plans/`, `docs/history/`, 보관된 소스에만 남는다.

- [ ] **단계 9: 문서 마이그레이션 커밋**

```bash
git add README.md docs firmware/README.md scripts/README.md logs/README.md
git commit -m "docs: align project documentation with repository layout"
```

---

### 작업 5: 결정론적 저장소 레이아웃 및 텔레메트리 검사 추가

**파일:**

- 생성: `tools/test_repo_layout.py`
- 생성: `tools/check_repo_layout.py`
- 생성: `tools/test_telemetry_schema.py`

**인터페이스:**

- `check_markdown_links(repo: Path, files: Iterable[Path]) -> list[str]`
- `check_sketch_names(repo: Path) -> list[str]`
- `check_migration_map(repo: Path, map_path: Path) -> list[str]`
- `check_stale_tokens(repo: Path, files: Iterable[Path]) -> list[str]`
- `main() -> int`

- [ ] **단계 1: 저장소 검증기용 실패 테스트 작성**

`tools/test_repo_layout.py` 생성:

```python
import tempfile
import unittest
from pathlib import Path

from tools.check_repo_layout import (
    check_markdown_links,
    check_migration_map,
    check_sketch_names,
    check_stale_tokens,
)


class RepoLayoutChecksTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.repo = Path(self.tempdir.name)

    def tearDown(self):
        self.tempdir.cleanup()

    def write(self, relative, content=""):
        path = self.repo / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        return path

    def test_markdown_links_report_only_missing_local_target(self):
        self.write("docs/good.md", "ok")
        readme = self.write(
            "README.md",
            "[good](docs/good.md) [bad](docs/missing.md) "
            "[web](https://example.com)",
        )
        errors = check_markdown_links(self.repo, [readme])
        self.assertEqual(1, len(errors))
        self.assertIn("docs/missing.md", errors[0])

    def test_sketch_directory_must_match_ino_basename(self):
        self.write("firmware/flight/right/right.ino")
        self.write("firmware/flight/wrong/not_wrong.ino")
        errors = check_sketch_names(self.repo)
        self.assertEqual(1, len(errors))
        self.assertIn("not_wrong.ino", errors[0])

    def test_migration_requires_missing_old_and_existing_new(self):
        mapping = self.write(
            "docs/migration_map.md",
            "| Old path | New path |\n|---|---|\n"
            "| `old/file.txt` | `new/file.txt` |\n",
        )
        self.write("new/file.txt")
        self.assertEqual([], check_migration_map(self.repo, mapping, 1))
        self.write("old/file.txt")
        errors = check_migration_map(self.repo, mapping, 1)
        self.assertTrue(any("old path still exists" in error for error in errors))

    def test_stale_token_is_rejected(self):
        readme = self.write("README.md", "Run Drone_Reciever.py")
        errors = check_stale_tokens(self.repo, [readme])
        self.assertEqual(1, len(errors))
        self.assertIn("Drone_Reciever.py", errors[0])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **단계 2: 테스트 실행 및 예상된 red 상태 확인**

```bash
/home/light/anaconda3/bin/python -m unittest tools/test_repo_layout.py -v
```

예상: `ModuleNotFoundError: No module named
'tools.check_repo_layout'`로 FAIL.

- [ ] **단계 3: `tools/check_repo_layout.py` 구현**

```python
#!/usr/bin/env python3
import re
import sys
from pathlib import Path
from urllib.parse import unquote


REPO_ROOT = Path(__file__).resolve().parents[1]
LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
MAP_RE = re.compile(r"^\|\s*`([^`]+)`\s*\|\s*`([^`]+)`\s*\|")

LEGACY_TOKENS = (
    "firmware/examples/",
    "firmware/src/",
    "firmware/platformio_config/",
    "Drone_Control_Dualsense.py",
    "Drone_Tuning.py",
    "Drone_Reciever.py",
    "Drone_Monitor.py",
    "Drone_Analasys.py",
    "drone_telemetry.py",
    "dual_imu_pid_debug_receiver.py",
    "Controller_test.py",
    "GPS_Reciever.py",
    "ZETIN_Drone",
)


def _link_target(raw):
    raw = raw.strip()
    if raw.startswith("<") and ">" in raw:
        raw = raw[1 : raw.index(">")]
    else:
        raw = raw.split(maxsplit=1)[0]
    return unquote(raw)


def check_markdown_links(repo, files):
    errors = []
    for source in files:
        text = source.read_text(encoding="utf-8")
        for match in LINK_RE.finditer(text):
            target = _link_target(match.group(1))
            if not target or target.startswith("#"):
                continue
            if re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*:", target):
                continue
            target_path = target.split("#", 1)[0]
            if not target_path:
                continue
            if target_path.startswith("/"):
                errors.append(f"{source.relative_to(repo)}: absolute link {target}")
                continue
            resolved = (source.parent / target_path).resolve()
            if not resolved.exists():
                errors.append(
                    f"{source.relative_to(repo)}: missing link target {target_path}"
                )
    return errors


def check_sketch_names(repo):
    errors = []
    for ino in sorted((repo / "firmware").rglob("*.ino")):
        if ino.parent.name != ino.stem:
            errors.append(
                f"sketch name mismatch: {ino.relative_to(repo)} "
                f"(directory {ino.parent.name!r}, file {ino.stem!r})"
            )
    return errors


def check_migration_map(repo, map_path, expected_rows=44):
    errors = []
    rows = []
    for line_number, line in enumerate(
        map_path.read_text(encoding="utf-8").splitlines(), 1
    ):
        match = MAP_RE.match(line)
        if match and match.group(1) != "Old path":
            rows.append((line_number, match.group(1), match.group(2)))

    if len(rows) != expected_rows:
        errors.append(
            f"migration map has {len(rows)} rows; expected {expected_rows}"
        )

    old_seen = set()
    new_seen = set()
    for line_number, old, new in rows:
        if old in old_seen:
            errors.append(f"migration map line {line_number}: duplicate old path {old}")
        if new in new_seen:
            errors.append(f"migration map line {line_number}: duplicate new path {new}")
        old_seen.add(old)
        new_seen.add(new)
        if (repo / old).exists():
            errors.append(f"migration map line {line_number}: old path still exists {old}")
        if not (repo / new).exists():
            errors.append(f"migration map line {line_number}: new path missing {new}")
    return errors


def check_stale_tokens(repo, files):
    errors = []
    for source in files:
        text = source.read_text(encoding="utf-8")
        for token in LEGACY_TOKENS:
            if token in text:
                errors.append(f"{source.relative_to(repo)}: stale token {token}")
    return errors


def maintained_markdown_files(repo):
    explicit = [
        repo / "README.md",
        repo / "docs/README.md",
        repo / "docs/project_overview.md",
        repo / "docs/firmware_catalog.md",
        repo / "docs/udp_protocol.md",
        repo / "docs/migration_map.md",
        repo / "firmware/README.md",
        repo / "scripts/README.md",
        repo / "logs/README.md",
        repo / "firmware/archive/README.md",
        repo / "firmware/archive/dshot/README.md",
        repo / "firmware/archive/platformio_skeleton/README.md",
        repo / "scripts/archive/README.md",
    ]
    return [path for path in explicit if path.exists()]


def stale_scan_files(repo):
    # The migration map intentionally contains every legacy token, so it is
    # validated structurally but excluded from stale-reference rejection.
    files = [
        path
        for path in maintained_markdown_files(repo)
        if path != repo / "docs/migration_map.md"
    ]
    files.extend(sorted((repo / "firmware/flight").rglob("*.ino")))
    files.extend(sorted((repo / "firmware/diagnostics").rglob("*.ino")))
    files.extend(sorted((repo / "scripts").glob("*.py")))
    return files


def main():
    errors = []
    markdown = maintained_markdown_files(REPO_ROOT)
    errors.extend(check_markdown_links(REPO_ROOT, markdown))
    errors.extend(check_sketch_names(REPO_ROOT))
    errors.extend(
        check_migration_map(REPO_ROOT, REPO_ROOT / "docs/migration_map.md")
    )
    errors.extend(check_stale_tokens(REPO_ROOT, stale_scan_files(REPO_ROOT)))

    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print("repository layout checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **단계 4: 검증기 테스트를 green이 될 때까지 실행**

```bash
/home/light/anaconda3/bin/python -m unittest tools/test_repo_layout.py -v
```

예상: 네 개의 테스트 통과.

- [ ] **단계 5: 텔레메트리 호환성 특성화 테스트 추가**

`tools/test_telemetry_schema.py` 생성:

```python
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from telemetry_schema import (  # noqa: E402
    CSV_FIELDS,
    TELEMETRY_FIELDS,
    parse_telemetry_packet,
    sample_to_csv_row,
)


def packet(field_count):
    values = [str(index + 0.5) for index in range(9)]
    values.extend(str(index) for index in range(9, field_count))
    return ",".join(values)


class TelemetryCompatibilityTest(unittest.TestCase):
    def test_10_field_packet_keeps_extended_values_unknown(self):
        sample = parse_telemetry_packet(packet(10))
        self.assertEqual(9, sample["Throttle"])
        self.assertIsNone(sample["Fault_RC"])
        self.assertEqual(22, len(sample_to_csv_row("00:00:00.000", sample)))

    def test_14_field_packet_populates_legacy_fault_and_rc_fields(self):
        sample = parse_telemetry_packet(packet(14))
        self.assertEqual(13, sample["RC_Dropped_Pkts"])
        self.assertIsNone(sample["Fault_IMU1"])

    def test_21_field_packet_populates_cascade_diagnostics(self):
        sample = parse_telemetry_packet(packet(21))
        self.assertEqual(20, sample["Calibration_OK"])
        self.assertEqual(21, len(TELEMETRY_FIELDS))
        self.assertEqual(22, len(CSV_FIELDS))

    def test_short_packet_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_telemetry_packet(packet(9))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **단계 6: 모든 저장소 및 텔레메트리 테스트 실행**

```bash
/home/light/anaconda3/bin/python -m unittest \
  tools/test_repo_layout.py \
  tools/test_telemetry_schema.py -v
/home/light/anaconda3/bin/python tools/check_repo_layout.py
```

예상: 여덟 개의 단위 테스트가 통과하고 검증기가
`repository layout checks passed`를 출력한다.

- [ ] **단계 7: 검증 도구 커밋**

```bash
git add tools
git commit -m "test: validate repository layout and telemetry compatibility"
```

---

### 작업 6: 최종 정리 검증 실행 및 검토 준비

**파일:** 검증 실패가 작업 1-5가 이미 소유한 파일의 결함을 드러내지 않는 한
새 파일은 없다.

**인터페이스:** 깨끗하고 로컬에서 검증된 브랜치를 출력한다. 푸시하지 않는다.

- [ ] **단계 1: 완전한 Python 및 저장소 검사를 새로 실행**

```bash
/home/light/anaconda3/bin/python -m unittest \
  tools/test_repo_layout.py \
  tools/test_telemetry_schema.py -v
/home/light/anaconda3/bin/python tools/check_repo_layout.py
/home/light/anaconda3/bin/python -m py_compile scripts/*.py tools/*.py
```

예상: 여덟 개의 테스트가 통과하고, 저장소 검증기가 통과하며, py_compile이
exit 0.

- [ ] **단계 2: 지원되는 여섯 개 펌웨어 스케치 전부 새로 컴파일**

```bash
for sketch in \
  firmware/flight/dual_imu_cascade_pwm \
  firmware/diagnostics/motor_pwm_bench \
  firmware/diagnostics/icm42670_single_raw \
  firmware/diagnostics/icm42670_dual_raw \
  firmware/diagnostics/icm42670_dual_loop_debug \
  firmware/diagnostics/board_v1_5_2_dual_imu; do
  name=$(basename "$sketch")
  arduino-cli compile --warnings all \
    --fqbn esp32:esp32:esp32s3 \
    --build-path "/tmp/zetin-final-$name" \
    "$sketch" || exit 1
done
```

예상: exit 0 빌드 여섯 개. 보관 빌드로 대체하지 않는다.

- [ ] **단계 3: 마이그레이션 집계 및 작업 트리 품질 검증**

```bash
test "$(find firmware -type f -name '*.ino' | wc -l)" -eq 23
test "$(find firmware/flight firmware/diagnostics -type f -name '*.ino' | wc -l)" -eq 6
test "$(find firmware/archive -type f -name '*.ino' | wc -l)" -eq 17
git diff --check origin/main...HEAD
git status --short --branch
git log --oneline --decorate origin/main..HEAD
```

예상:

- 총 23개, 현행 6개, 보관 17개 스케치;
- 공백 오류 없음;
- 스테이징되거나 스테이징되지 않은 프로젝트 파일 없음;
- 로컬 에이전트/에디터 경로는 무시되므로 나타나지 않음;
- 로컬 브랜치는 정리 문서 및 구현 커밋만큼만 앞서
  있음.

- [ ] **단계 4: 최종 diff에서 금지된 동작 변경 검토**

```bash
git diff --find-renames --stat origin/main...HEAD
git diff --find-renames origin/main...HEAD -- \
  firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
  scripts/control_dualsense.py \
  scripts/tune_pid.py \
  scripts/telemetry_schema.py
```

예상: 현행 펌웨어/스크립트 diff는 경로/주석/임포트 변경만 포함한다. 게인, 핀,
모터 믹서, 패킷 순서, 소켓 주소, 명령 파싱 변경은
없어야 한다.

- [ ] **단계 5: 푸시 없이 사용자 검토로 인계**

보고:

- 최종 커밋 목록;
- 현행 6개 및 보관 17개 스케치 개수;
- 펌웨어 컴파일 결과;
- Python, 단위 테스트, 검증기 결과;
- 정확한 설계 및 구현 계획 링크;
- 보관 코드가 수리되거나 빌드 주장되지 않았다는 확인;
- 브랜치가 푸시되지 않았다는 확인.
