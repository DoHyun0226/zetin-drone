# ZETIN Drone

센서 읽기·자세 추정·PID·모터 제어·안전장치와 PC 지상국을 직접 만드는
쿼드콥터 프로젝트다. 목표는 실내에서 안정적으로 조종되는 기체다.
주력 구성은 **ESP32-S3 + 듀얼 ICM42670 IMU + BMM350 자기계 + PWM ESC**이며,
USB 게임패드를 연결한 지상국과 WiFi SoftAP UDP로 통신한다.

[노션: 프로젝트 현황·작업 흐름](https://app.notion.com/p/3142d27b08d3804eb85ced1ca13d8998) ·
[문서 목록](docs/README.md) ·
[발표자료](https://lightminn.github.io/zetin-drone-presentations/)

## 현재 상태

**2026-10-15 대조 기준. 저스로틀 yaw 개선은 2026-10-14 `main`에 통합됐다(`1dc8582`).**

| 항목 | 확인된 상태 | 다음 확인 |
|---|---|---|
| 자세 제어 | 캐스케이드 PID·yaw 자동 잠금·자기계 보정 구현. 8월 1일 실비행 기록 | 시험 구성과 실제 사용 게인을 기록하고 반복 검증 |
| 최근 시험 | 8월 19일 테더 이륙·수동 호버·착륙 5회 기록 | 통합 코드의 무장 거부·확인 시간 초과·failsafe 경로 실기 검증 |
| RC 상실 대응 | 조건에 따라 즉시 컷 또는 시간 기반 하강. 착지 감지는 미구현 | 호버 추정의 지상 오염과 접지 후 동작 확인 |
| 거리·광류 | 3901-L0X 수신·기록 코드 구현. 최근 시험의 신선한 값 미확보 | 배선·프레임·품질·지연부터 측정 |
| 자동 고도·위치 유지 | 미구현. 최근 테더 시험의 호버는 조종자가 유지 | 유효한 센서 관측 확보 후 설계 |

8월 1일의 **176.2초는 연속 무장 시간**이며, 전체 체공 시간으로 해석하지
않는다. 같은 날의 자동착륙 시험은 별도 기록이다. 8월 19일 결과는 테더와
조종자가 포함된 구성의 기록이다. 그때 비행한 roll/pitch 우선·잔여 범위 yaw 배분과
펌웨어 무장 확인 절차는 2026-10-14 기본 코드에 통합됐고, 통합 펌웨어는 비행한
코드와 주석만 다르다. 무장 거부·확인 시간 초과와 failsafe 경로는 실기 검증이 남아 있다.
커밋·시험 조건·남은 작업의 완료 기준은 [현재 상태](docs/current_status.md)에 정리했다.

## 여기서 시작

| 하고 싶은 일 | 읽을 곳 |
|---|---|
| 처음 참여하고 전체 흐름 파악 | [프로젝트 개요](docs/project_overview.md) → [작업 순서·인수인계](docs/workflow.md) |
| 기체 구성과 작업할 코드 선택 | [하드웨어 확인표](docs/hardware_configuration.md) → [펌웨어·진단 가이드](firmware/README.md) |
| 모터·센서 벤치 시험 | [전원 인가 절차](docs/power_on_bench_procedure.md) — 프로펠러 제거 단계부터 |
| 조종·텔레메트리·로그 분석 | [PC 도구](scripts/README.md) → [로그 안내](logs/README.md) |
| 명령·필드·바이너리 규격 변경 | [UDP 프로토콜](docs/udp_protocol.md) |
| 다음 실험과 착지 감지 연구 | [현재 상태](docs/current_status.md) → [착지 감지 연구](docs/failsafe_land_research.md) |

주력 소스는 [`dual_imu_cascade_pwm`](firmware/flight/dual_imu_cascade_pwm/)다.
[`dual_imu_flix_quat_pwm`](firmware/flight/dual_imu_flix_quat_pwm/)는 보류 후보이며
주력 지상국의 `rcr` 조종을 지원하지 않는다. 진단·보관 코드까지 포함한
26개 스케치의 용도와 실행 범위는 [펌웨어 카탈로그](docs/firmware_catalog.md)를 따른다.

## 빠른 확인

저장소 루트에서 실행한다. 의존성은 [펌웨어 가이드](firmware/README.md)와
[지상국 가이드](scripts/README.md)에 따라 준비한다. 아래 명령은 PC에서
코드를 검사·컴파일하며 기체에 업로드하지 않는다.

```bash
python -m py_compile scripts/*.py
python tools/check_repo_layout.py
python -m unittest discover -s tools -p 'test_*.py' -v

# 주력·보류 후보·진단 스케치 9개. 실패하면 이 빌드 묶음을 중단한다.
(
  set -e
  for sketch in firmware/flight/*/ firmware/diagnostics/*/; do
    arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
      --build-path "/tmp/zetin-$(basename "$sketch")" "$sketch"
  done
)
```

2026-10-14 기준 로컬 `unittest` **200개가 skip 없이 통과**했고, `main`
CI(ESP32 코어 3.3.10)에서 **스케치 9개 컴파일**도 통과했다.
이전 검사 범위와 실기에서 남은 확인은 [2026-09-07 정합성 점검 기록](docs/reports/2026-09-07-documentation-audit.md)에 있다.

## 안전

- **모터 벤치와 업로드는 프로펠러를 제거한 상태에서 한다.** 전원 극성·핀·모터
  번호·회전 방향·보정 부호·정지 경로를 먼저 확인한다. 단일 모터 식별은
  `motor_id_single`로 시작하고, 부팅 후 자동 구동하는 진단 스케치와 구분한다.
- **게임패드는 USB, 드론에 UDP를 보내는 프로그램은 하나만 사용한다.** 과거
  WiFi/BT 공유 라디오 장비에서 블루투스 사용과 RC 정체가 함께 관측됐다.
  장비 조건과 진단 근거는 [링크 문서](docs/ground_station_link.md)에 있다.
- **RC 단절이 안전한 착지를 보장하지 않는다.** 호버 추정·스로틀 조건에 따라
  즉시 컷하거나 시간 기반으로 하강한다. 하강 중에는 접지 후에도 모터가 돌 수
  있고, 시간이 끝날 때 아직 공중일 수 있다. 실제 프로브 딥도 계속 적용된다.
- **명령 전송과 실제 무장·정지를 구분한다.** 정지만 요청할 때는 stdin `stop`을
  사용한다. X는 로컬 무장 추정에 따른 토글이며, 링크 복귀만으로 조종이 재개되지는 않는다.

실기 시험은 [벤치 절차](docs/power_on_bench_procedure.md)의 단계와 조건을 따른다.
