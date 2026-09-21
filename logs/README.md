# 비행·센서 로그

조종과 함께 기록할 때는 [`control_dualsense.py`](../scripts/control_dualsense.py)를
사용한다. 단독 수신기 `receive_telemetry.py`·`monitor_telemetry.py`도 비행 CSV를
만들지만 raw 바이너리는 저장하지 않는다. UDP 수신 도구를 동시에 실행하면
텔레메트리 목적지가 바뀌므로 [도구별 사용 조건](../scripts/README.md)을 따른다.

## 어떤 파일인가

| 파일 | 내용 | 시간·좌표계 |
|---|---|---|
| `flight_log_YYYY-MM-DD_HHMMSS.csv` | 수신한 ASCII 텔레메트리 | `Timestamp`는 PC 수신 벽시계. IMU 값은 펌웨어 body frame |
| `imuraw_YYYY-MM-DD_HHMMSS.bin` | 같은 지상국 세션의 `ZIMU`·`ZCAL` 데이터그램 원본 | 드론 `micros()` 기반. int16 센서 레지스터 원값 |
| `imuraw_YYYY-MM-DD_HHMMSS.csv` | raw 바이너리 디코딩 결과 | `t_us`와 `sample_idx`. 물리량 열은 IMU1 센서 프레임 |

같은 파일명 시각은 세션을 연결하는 표식이다. PC 수신시각과 드론 `micros()`를
샘플 단위로 동기화한 공통 키가 아니다. 파일이 있다는 사실도 실비행·착지·시험
통과를 증명하지 않는다. 당시 빌드·실험 조건·운영자 기록과 함께 확인한다.
과거 CSV·바이너리는 원본으로 보존하고 재분석 결과는 별도 출력으로 남긴다.

## 비행 CSV 읽기

현행 수집기는 `Timestamp` 다음에 공유 스키마의 텔레메트리를 기록한다.
**저스로틀 yaw 개선 통합 코드 기준 75필드 + PC 시각 = 76열**이다. 정확한 필드 순서·단위·
레거시 형식은 [UDP 프로토콜](../docs/udp_protocol.md#드론--지상국), 구현은
[`telemetry_schema.py`](../scripts/telemetry_schema.py)가 정본이다.

현행 수집기로 구형 패킷을 받으면 없는 필드는 빈 셀로 채워진다. 과거 도구로
생성한 파일은 헤더 자체가 짧을 수 있으므로 실제 헤더를 먼저 읽는다.
빈 셀은 “알 수 없음”이며 `Fault=0`, `Armed=0`, `Failsafe_Phase=0`을 뜻하지 않는다.
`Timestamp`는 패킷에 포함되지 않으며 자정·PC 시계 변경도 고려해야 한다.

- `Armed`는 드론 safety lock 상태이며 콘솔의 `ARMED` 요청 배너와 구분한다.
- `Failsafe_Phase=1`은 하강 중, 3은 시간 기반 컷, 4는 중단컷이다. 현행
  비행 코드에서 2는 도달 불가지만 과거 기록과 도구에는 이름이 남아 있다.
  GUI의 `AUTO-LAND ACTIVE` 경고는 종료 phase에도 남으므로 숫자와 `Armed`를
  함께 읽는다. `1 → 0`은 명시적 복구일 수 있으며 접지 시각을 뜻하지 않는다.
- `Mag_Cal_Active=1`인 행의 `Mag_X/Y/Z`는 보정 전 raw µT다. 0인 행은
  hard/soft-iron·body 부호·스로틀 간섭 보정 후 도메인이다. mag가 꺼져 있으면
  값이 갱신되지 않을 수 있으므로 0을 신선도 보장으로 해석하지 않는다.
- `RC_Total_Pkts`·`RC_Dropped_Pkts`는 부팅 이후 누적 카운터다.
  `analyze_flight_log.py`의 드롭 비율 출력은 마지막 누적값의 비율이며 그 파일
  구간의 무선 유실률이 아니다. [링크 진단](../docs/ground_station_link.md)을 참고한다.
- 목표 roll/pitch 각도에는 트림이 포함된다. `Motor_M1`~`Motor_M4`는 모터
  PWM 명령이며 RPM·추력 측정값이 아니다. 배터리 전압·PID 항별 기여 열은 없다.

```bash
python scripts/analyze_flight_log.py logs/flight_log_YYYY-MM-DD_HHMMSS.csv --no-plot
```

파일을 생략하면 `logs/*.csv` 중 수정시각이 가장 최근인 것을 고른다. raw
CSV를 비행 CSV로 오인하지 않도록 경로를 지정한다. 위 도구는 읽기 분석만 하며
그래프 저장 기능은 없다. `--no-plot`을 빼면 화면에 그래프를 띄운다.

## raw 로그 읽기

지상국 기본 모드는 첫 raw 데이터그램 수신 때 `.bin`을 만들고, 정상 종료
정리에서 같은 이름의 `.csv`로 변환한다. raw를 받지 않았거나 `--no-raw`로
시작한 세션에는 두 raw 파일이 생기지 않는다. 변환 실패·비정상 종료로 `.csv`가
없으면 보존된 바이너리를 다음처럼 다시 디코딩한다.

```bash
python scripts/decode_imu_raw.py logs/imuraw_YYYY-MM-DD_HHMMSS.bin /tmp/imu-decoded.csv
```

출력 경로를 생략하면 입력 옆 `.csv`를 생성/덮어쓴다. `ZIMU` v2는
`failsafe_phase`가 있고 v1은 그 열을 비워둔다. v1 로그에는 소프트웨어를
업데이트해도 과거 phase가 소급해서 생기지 않는다. `ZCAL`이 있으면 bias·scale·
IMU2 부호를 적용한 dps/g 열이 추가된다. **body-frame 변환과 펌웨어 gyro
소프트웨어 LPF는 디코더에서 적용하지 않는다.** 두 종류 CSV의 같은 이름 축을
바로 빼거나 평균내지 않는다.

stderr의 `wireless_lost_batches`는 배치 시퀀스 틈으로 추정한 손실,
`producer_dropped`는 마지막 헤더의 링 포화 누적 샘플 수다. 중복·순서 뒤바뀜·
재부팅이 있으면 단순 유실 해석에 주의한다. `average_sample_rate_hz`는 수신한
샘플의 기록된 `dt_us`로 계산하며, 무선에서 사라진 모든 샘플까지 반영한 파일
완전성을 보장하지 않는다. `t_us`는 uint32 래핑을 그대로 유지한다.

텔레메트리는 약 20Hz 스냅샷이라 빠른 프롭 진동이 앨리어싱된다. raw의 명목
1kHz 기록도 하드웨어 LPF(gyro 121Hz, accel 25Hz)를 거친 센서 값이다.
대역 밖은 감쇠하므로 이 파일만으로 원래 진동 스펙트럼을 복원했다고 주장하지
않는다. 기록 성공·오프라인 계산·실제 제어/비행 검증은 서로 구분한다.
