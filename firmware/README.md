# Firmware

주력은 **ESP32-S3 + 듀얼 ICM42670 + PWM 제어**다. BMM350 자기계는 선택적
yaw 융합에, 3901-L0X 거리·광류 센서는 기록에 사용한다. 전체 스케치의
지원 상태는 [펌웨어 카탈로그](../docs/firmware_catalog.md)에서 확인한다.
[`archive/`](archive/README.md)는 역사 보존 영역이며 빌드 지원 대상이 아니다.

## 비행 펌웨어 선택

- [`flight/dual_imu_cascade_pwm/`](flight/dual_imu_cascade_pwm/): 듀얼
  ICM42670, 바깥쪽 자세 루프와 안쪽 각속도 루프, 4채널 PWM을 사용하는
  주력 비행 제어 경로. 실비행 기록은 있지만 반복 가능한 안정 비행과
  자동착륙의 실기 검증 완료를 뜻하지 않는다. 운용은
  [전원 인가·벤치 절차](../docs/power_on_bench_procedure.md)를 따른다.
- [`flight/dual_imu_flix_quat_pwm/`](flight/dual_imu_flix_quat_pwm/):
  [flix](https://github.com/okalachev/flix) 아키텍처 기반 쿼터니언 자세
  추정·SI 단위 캐스케이드 제어 **보류 후보**다. 빌드 매트릭스에는 포함되지만
  현행 지상국의 `rcr` 명령·자동착륙·확장 텔레메트리를 지원하지 않는다.
  게인 단위와 yaw 부호도 다르다. 호환성 범위와 벤치 검증 절차는
  [해당 README](flight/dual_imu_flix_quat_pwm/README.md) 참조.

핀 배치: 모터 FL/RR/FR/RL = GPIO 4/5/6/7, SPI SCK/MISO/MOSI =
GPIO 12/13/11, IMU1/IMU2 CS = GPIO 10/9.

## 현행 진단 스케치

- [`diagnostics/motor_id_single/`](diagnostics/motor_id_single/):
  모터 식별을 시작할 때 쓰는 수동 진단. 비행 펌웨어와 같은 LEDC 400Hz로
  한 번에 한 모터만 구동한다. 부팅 시 자동 시작이 없으며 시리얼 `1`~`4`로
  선택하고 `0`/`s`로 정지한다. 상한은 1250µs다.
- [`diagnostics/motor_pwm_bench/`](diagnostics/motor_pwm_bench/): GPIO
  4/5/6/7의 4채널 자동 램프 시험. **부팅 7초 뒤 네 모터가 자동으로 돌고
  1000~1300µs 램프를 반복한다.** ESP32Servo 50Hz를 사용하므로 위 LEDC
  모터 식별 시험과 다르며, 시리얼 정지 명령은 없다.
- [`diagnostics/esc_config_throttle/`](diagnostics/esc_config_throttle/):
  ESC 설정용 단일 채널 LEDC 50Hz 출력. 기본 GPIO7(M4/RL), 부팅 시 1000µs.
  **프로펠러 제거·전류 제한 전원·설정할 ESC/모터 하나만 연결**해야 한다.
  `h`는 최대 2000µs를 계속 출력하므로 일반 저속 모터 시험으로 쓰지 않는다.
- [`diagnostics/icm42670_single_raw/`](diagnostics/icm42670_single_raw/):
  단일 ICM42670의 가속도·각속도 6축 출력(루프당 100ms 대기).
  SPI SCK/MISO/MOSI/CS = GPIO 18/19/23/5.
  이 배치는 PCB v1.5.2 진단과 다르다.
- [`diagnostics/icm42670_dual_raw/`](diagnostics/icm42670_dual_raw/): 듀얼
  ICM42670 원시값 출력. SPI SCK/MISO/MOSI = GPIO 12/13/11, CS =
  GPIO 10/9.
- [`diagnostics/icm42670_dual_loop_debug/`](diagnostics/icm42670_dual_loop_debug/):
  구형 듀얼 IMU 추정기의 루프 주기·필터 진단. **PID·믹서 없이 모터는
  1000µs에 고정**된다. 별도 14필드 UDP 형식이므로
  [`receive_dual_imu_debug.py`](../scripts/receive_dual_imu_debug.py)와 짝지어
  사용한다. 모터 = GPIO 4/5/6/7, SPI = GPIO 12/13/11, CS = GPIO 10/9.
- [`diagnostics/board_v1_5_2_dual_imu/`](diagnostics/board_v1_5_2_dual_imu/):
  PCB v1.5.2 듀얼 ICM42670의 자이로 Z·가속도 Z 비교 출력.
  SPI SCK/MISO/MOSI = GPIO 12/13/11,
  CS = GPIO 10/9.

## 빌드

Arduino CLI를 사용한다. 의존성과 버전의 기준은
[CI 워크플로](../.github/workflows/ci.yml)의 `firmware` 잡이다.
`ICM42670P`는 비행·IMU 진단에, `ESP32Servo`는 `motor_pwm_bench`에 필요하다.
주력 스케치는 자기계 융합을 끄더라도 `DFRobot_BMM350`을 빌드 의존성으로
요구한다. CI와 같은 설치 절차는 다음과 같다:

```bash
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.3.10
arduino-cli lib install ICM42670P@1.0.9 ESP32Servo@3.2.1
arduino-cli config set library.enable_unsafe_install true
arduino-cli lib install --git-url https://github.com/DFRobot/DFRobot_BMM350.git
```

저장소 루트에서 빌드한다. 빌드 산출물은 저장소 밖 `/tmp`에 둔다:

```bash
sketch=firmware/flight/dual_imu_cascade_pwm
name=$(basename "$sketch")
arduino-cli compile --warnings all \
  --fqbn esp32:esp32:esp32s3 \
  --build-path "/tmp/zetin-$name" \
  "$sketch"
```

진단과 보류 후보도 `sketch` 경로를 바꿔 같은 방식으로 빌드한다. CI는
`flight/`와 `diagnostics/`를 컴파일하고 `archive/`는 제외한다. 컴파일 성공은
핀 연결, ESC 동작, 통신, 비행을 검증한 결과와 구분한다.

## 안전

- 모터를 구동하는 모든 벤치 시험과 펌웨어 업로드는 프로펠러를 제거한
  상태에서 진행한다.
- 배터리 연결 전에 전압과 극성을 다시 확인한다.
- 모터 번호, 회전 방향, PWM 최소값과 비상 정지 동작을 확인하기 전에는
  기체를 띄우지 않는다.
- [`archive/`](archive/README.md)의 모터 제어 코드는 특히 안전하다고
  가정하면 안 된다.
