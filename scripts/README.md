# PC 지상국·분석 도구

현행 `dual_imu_cascade_pwm`용 조종·텔레메트리 수집·오프라인 분석 도구다.
처음 시작하면 [전원 인가 벤치 절차](../docs/power_on_bench_procedure.md)를
먼저 읽고, USB로 연결한 DualSense와 `drone_control.py`를 사용한다.
실행 도구는 세 개(`drone_control.py`·`drone_video.py`·`drone_logs.py`)뿐이고,
나머지 코드는 `ground/` 패키지, 벤치·특수 펌웨어용 스크립트는 [archive/](archive/README.md)에 있다.
이 문서는 저스로틀 yaw 개선이 통합된 저장소 코드 기준이며 현재 기체에 올라간 빌드나 실기
통과 여부를 뜻하지 않는다. 명령·필드 순서·단위의 정본은
[UDP 프로토콜](../docs/udp_protocol.md)이다.

## 무엇을 실행할까

아래 명령은 저장소 루트에서, 필요한 패키지가 설치된 Python 환경으로 실행한다.
의존성은 [requirements.txt](../requirements.txt)에 있다. 전체 화면 대시보드는
`prompt_toolkit`이 설치되어 있어야 하며, 없으면 `--plain`과 같은 줄 출력으로 시작한다.

| 목적 | 명령 | 실행 조건과 출력 |
|---|---|---|
| 조종·stdin 명령·CSV·raw 기록 | `drone_control.py` | 현행 기본 지상국(대시보드). 게임패드 USB 연결, `Drone_Tuning` 접속. `--plain`은 대시보드 없이 줄 출력 |
| 축·버튼 번호 확인 | `drone_control.py --pad-check` | 드론에 아무것도 보내지 않음(소켓 없음). 처음 0.7초 손을 떼고 각 입력을 하나씩 확인 |
| 입력값 연속 표시 | `drone_control.py --pad-check --live` | 로컬 입력 진단. 한 줄에 모든 축·버튼·햇 값을 표시. 번호는 실측 |
| 게임패드 없는 명령 콘솔 | `drone_control.py --keyboard` | 설정·읽기용. RC 송신 없음, `start`·`resume` 거부, 종료 시 자동 송신 없음(`stop`은 직접 입력), 로그 파일 없음 |
| 수신·표시·CSV·raw 기록 | `drone_control.py --listen` | 수신 전용. `connect` 외에는 보내지 않음(입력줄 명령·트림 재동기화·종료 시 disarm 없음) |
| 실시간 그래프·CSV 기록 | `drone_control.py --listen --plot` | GUI(matplotlib) 필요. `connect` 외에는 보내지 않음 |
| 카메라 녹화 | `drone_video.py` (= `record`) | 비행 UDP 포트를 쓰지 않음. `Drone_Tuning` 접속, 조종 지상국과 다른 터미널 |
| 카메라 녹화 → 정렬 CSV·mp4 | `drone_video.py mp4 <csv> [--no-mp4]` | 오프라인. 입력 `logs/camera_*.csv` |
| 비행 CSV 요약·그래프 | `drone_logs.py flight [csv] [--no-plot]` | 오프라인. 생략하면 `logs/flight_log_*.csv` 중 가장 최근 파일 |
| 프로브 분포 분석 | `drone_logs.py probe` | 오프라인 연구용. 결과가 현행 착지컷에 연결되지는 않음 |
| raw 바이너리 → CSV | `drone_logs.py imuraw <bin> [csv]` | 오프라인. 원본 `.bin` 보존, 출력 CSV 생성/덮어쓰기 |
| 자기계 보정 상수 계산 | `drone_logs.py magcal <csv>` | 보정용 raw CSV에서 상수를 출력. 펌웨어를 자동 수정하거나 업로드하지 않음 |

공통 옵션은 `--no-raw`(raw 스트림·기록 끔)·`--no-mag`(자기계 융합 OFF 선택)·`--plain`이다.
벤치·특수 펌웨어용 도구(`motor_serial.py`·`bench_*.py`·`receive_dual_imu_debug.py`)는 현행 절차가 아니며
[archive/README.md](archive/README.md)에서 용도와 전제를 확인한다.

**드론에 UDP를 보내는 도구는 한 번에 하나만 실행한다.** 펌웨어는 마지막
발신자의 IP/포트를 텔레메트리 목적지로 삼는다. 포트 충돌을 피했더라도
`--keyboard`의 명령이나 `--listen`·수신기의 `connect`가 조종 지상국의 데이터를 가져갈 수 있다.
DualSense 조종 중 상태 확인과 로그 기록도 같은 `drone_control.py`에서 한다.

`--listen --plot`(`ground/monitor_plot.py`)는 현재 raw 바이너리 분기가
없다. 펌웨어 raw가 켜져 있으면 `ZIMU`·`ZCAL`을 불량 텔레메트리로 집계하며
저장하지 않는다. raw까지 받을 세션은 `--plot` 없는 `drone_control.py`(`--listen` 포함)를 사용한다.
독립 수신기만 사용할 벤치에서는 모터를 정지한 채 기존 콘솔에서 `raw off`를
보내고 그 콘솔을 종료한 뒤 수신기를 실행한다.
이 설정은 드론 재부팅 후 유지되지 않는다.

## DualSense 시작·조종·정지

```bash
python scripts/drone_control.py --pad-check
python scripts/drone_control.py
```

첫 명령을 종료한 다음 두 번째 명령을 실행한다. 실행만으로 시동하지 않는다.
`[STATUS]`의 자세·`Armed`·`Faults`를 확인하고 벤치 절차의 시동 조건을 따른다.
게임패드 없이 시작하면 조종 스레드는 종료하지만 stdin·수신은 남는다. 이때
stdin `start`만으로 주기적인 RC가 생기지는 않으므로 비행 조종에 사용하지 않는다.

| 입력 | 현행 동작 |
|---|---|
| X | `ARMED`이거나 `ARMING`(확인 대기)이면 `stop` 후 대기 취소, 아니면 `start` 요청 |
| △ | 자동착륙 하강 중 명시적 `resume` 시도 |
| 왼쪽 스틱 | roll/pitch 목표 각도, 최대 ±15° |
| 오른쪽 스틱 좌우 | yaw 각속도, 최대 ±90°/s. 상하는 사용하지 않음 |
| R2 / L2 | 눌림으로 판정된 동안 스로틀 증가/감소, 일정한 200µs/s |
| R1 / L1 | 누름 엣지마다 스로틀 +1 / −1µs |
| D-pad | roll/pitch 트림 0.2°씩 변경, 각 축 ±10° 제한 |
| PS | 트림을 0으로 초기화 |
| stdin `stop` | 지상국의 무장 추정과 무관하게 `stop` 반복 전송 |
| Ctrl+C | `ARMED`이거나 `ARMING`이면 `stop` 후 종료·로그 닫기 |

RC `rcr` 송신 목표는 **20Hz**(`CTRL_LOOP_HZ=20`)다. 스로틀과 트림 입력은
RC 송신 중에만 처리된다. 버튼 번호는 OS/연결 방식에 따라 확인해야 하며,
현재 코드는 X=b0, △=b3, PS=b12, R1=b5, L1=b4, R2=a5, L2=a2를 쓴다.

stdin에는 [프로토콜의 명령](../docs/udp_protocol.md#지상국--드론)을 입력한다.
`start`·`stop`·`resume`·`th`는 지상국 상태도 함께 갱신하는 별도 경로다.
자기계와 raw 토글은 이 콘솔에서 **`mag on/off`, `raw on/off`**로 입력한다.
그 외 PID 게인, `gains`, `yaw`, `magcal`, `magc` 등은 UDP로 전달된다.
무장 요청은 `ARMING`으로 시작하며, 펌웨어 `Armed=1`을 수신한 뒤에만
`ARMED 확인`을 출력하고 RC 송신을 시작한다. 거부·확인 시간 초과는 성공이
아니다. 다른 명령도 전송 메시지만으로 적용을 판단하지 말고 `Mag_Enabled`,
`Trim_*`, `GAINS` 응답 등 해당 텔레메트리로 확인한다.

## 링크가 끊겼을 때

게임패드 분리, 유효 텔레메트리 1.5초 미수신, `Fault_RC` 상승 엣지는
**RC 송신만 중단**한다. 로컬 스로틀은 초기화하지만 `th 1000`이나 `stop`을
자동으로 보내지 않는다. 펌웨어는 자체 RC 타임아웃 조건에 따라 즉시 컷 또는
시간 기반 하강을 수행한다. 하강은 착지를 감지하지 않으며 자세한 조건과
시간 예산은 [프로토콜](../docs/udp_protocol.md)에 있다.

수신·패드 연결이 돌아와도 RC가 자동 재개되지는 않는다. 같은 지상국 세션이
드론을 무장 상태로 보고 있고, 최신 `Failsafe_Phase=1`과 유효한 `Hover_Est`가
있을 때 △ 또는 stdin `resume`으로 복구를 요청한다. 지상국은 RC 송신을 먼저
재개하고, 카운터로 RC 수락을 확인한 뒤 `resume`을 보낸다. 이후 추가 RC
수락과 `Failsafe_Phase=0`을 확인해야 성공을 출력하고 로컬 스로틀을 호버
추정치에 맞춘다. **복귀는 고도·하강속도 복구를 보장하지 않는다.**

확인 시간 초과 메시지는 성공을 뜻하지 않는다. 해당 실패 처리만으로는
재개했던 RC 송신이 꺼지지 않을 수 있으며, 실제 phase를 다시 봐야 한다.
킬이나 재상실은 진행 중 복구를 취소한다. 펌웨어가 이미 컷한 뒤에는
`resume`할 수 없다. 지상국을 재시작하면 로컬 무장 추정도 초기화되므로
비행 중 프로세스 재시작을 복구 절차로 사용하지 않는다.

수동 킬은 stdin `stop`·X·Ctrl+C 경로다. 단 X는 로컬 무장 추정에 따른
토글이므로 **항상 정지만 요청하려면 stdin `stop`**을 사용한다.
드론의 `Armed=0` 확인(시동 직후 유예 제외)이나 `Fault_Critical` 상승 엣지도
지상국 무장 추정이 남아 있으면 `stop`을 보낸다. 자세한 링크 진단은
[지상국 링크 문서](../docs/ground_station_link.md)를 따른다.

## 상태·트림·자기계 읽기

`[STATUS]`는 수신된 상태를 약 1초마다, 무장 중 `[DIAG]`는 약 5Hz로 표시한다.
하강 중 `[AUTO-LAND]`는 매 텔레메트리 샘플을 표시하고 종료 phase는 전이 때
한 번 출력한다. 없는 구형 필드는 `-` 또는 `legacy/unknown`이며 정상값 0이 아니다.

- `Mag=`는 지상국 선택이 아닌 실제 `Mag_Enabled`다. 펌웨어 부팅 기본은 OFF,
  지상국 선택 기본은 ON이다. 시동 요청 전에 선택값을 다시 보낸다.
  OFF 선택으로 시작하려면 `--no-mag`를 쓴다. 기존 벤치 결과와 현재 보정 후
  재검증 범위는 [BMM350 문서](../docs/bmm350_yaw_bench_test.md)에서 확인한다.
- `AGL=`은 거리계 표시다. 신선도 없음은 `-`, 음수 거리 원값은 `OOR/q<품질>`,
  그 외는 `1.23m/q200`처럼 표시한다. 거리·광류는 현재 기록용이며 자동 고도
  제어·착지 판정에 사용하지 않는다. optical-flow 수평 위치 폐루프도 없으며,
  `Flow_*`는 수신·기록용이다. 중립 스틱에서 생기는 드리프트는 조종자가 수동 보정한다.
- `eR/eP`는 목표 각도에서 측정 각도를 뺀 값이다. 목표에는 트림이 이미 포함된다.
  `dG`는 두 IMU의 축별 gyro 차이 중 최댓값이다. 이 20Hz 스냅샷으로 빠른
  프롭 진동·순간 센서 고장을 확정할 수 없다.
- `[DIAG]`의 `RP_Scale`·`Yaw_Scale`·`Yaw_Authority`는 펌웨어 `Armed=1`일 때만
  수신값을 표시한다. 상태는 `NORMAL(0)`·`LIMITED(1)`·`RECOVERING(2)`이며,
  해당 값이 없는 구형 패킷이나 미시동 패킷은 `-`로 표시한다.
- `Trim=`은 드론이 보내온 값이다. 로컬 값과 다르면 `!`가 붙고 0.2초 이상
  간격으로 재전송한다. 유실·거부가 계속되면 표시도 남는다. 첫 완전한 트림
  텔레메트리를 한 번 채택한 뒤에는 로컬 값을 유지하므로 **stdin `trim`이나
  드론 재부팅으로 바뀐 값을 다시 덮어쓸 수 있다.** 같은 세션의 트림 변경은
  D-pad·PS를 사용한다. `start`는 트림을 따로 보내지 않는다.

## 로그 저장과 오프라인 분석

공유 `ground/telemetry_schema.py`의 현행 패킷은 **75필드**, PC 수신 시각
`Timestamp`를 앞에 붙인 CSV는 **76열**이다. 수신·모니터 도구도 같은 파서를
쓴다. 10~74필드의 레거시 패킷은 없는 뒤쪽 값을 빈 셀로 기록하고, 75필드를
넘는 미래 패킷은 알려진 앞쪽만 읽는다. 첫 10필드는 비어 있지 않은 유한수,
정수 필드는 정수값이어야 한다. 역사적 확장 경계와 단위는
[UDP 프로토콜](../docs/udp_protocol.md#드론--지상국)을 따른다.

추가 진단 필드의 순서는 다음과 같다.

| 번호 | 필드 | 의미 |
|---|---|---|
| 66 | `Mixer_RP_Scale` | roll·pitch 전달 비율 |
| 67 | `Mixer_Yaw_Scale` | yaw 전달 비율 |
| 68 | `Mixer_Collective_US` | 실제 배분한 collective(µs) |
| 69~71 | `PID_Roll_US`, `PID_Pitch_US`, `PID_Yaw_US` | 축별 PID 기여(µs) |
| 72~74 | `I_Roll_US`, `I_Pitch_US`, `I_Yaw_US` | 축별 적분 기여(µs) |
| 75 | `Yaw_Authority_State` | yaw 권한 상태 |

기본 지상국은 `logs/flight_log_<timestamp>.csv`를 기록한다. raw를 처음
받았을 때 같은 타임스탬프의 `imuraw_<timestamp>.bin`을 열고 데이터그램을
그대로 이어 쓴다. 종료 시 `.bin`을 닫고 같은 이름의 `.csv`로 변환한다.
변환 실패는 경고하며 원본 `.bin`은 남는다. 시간은 로그 크기와 실행 환경에
따라 달라진다. 코드의 “10분 세션 약 12초”는 안내값이다.

```bash
python scripts/drone_control.py --no-raw --no-mag
python scripts/drone_logs.py flight logs/flight_log_YYYY-MM-DD_HHMMSS.csv --no-plot
python scripts/drone_logs.py imuraw logs/imuraw_YYYY-MM-DD_HHMMSS.bin /tmp/imu.csv
python scripts/drone_logs.py probe ground.csv air.csv --label ground --label air
python scripts/drone_logs.py magcal capture.csv
```

각 명령은 서로 별도 용도다. `--no-raw`는 시작 시 `raw 0`을 반복 전송하며
세션 내 raw 파일 기록을 막는다. 이후 `raw on`은 펌웨어 게이트만 켜므로
`--no-raw`로 시작한 세션의 기록을 다시 켜지는 않는다. 기본 실행도 `raw 1`을
자동 재전송하지 않으므로 이전 세션이 껐다면 `raw on`을 직접 입력한다.

`drone_logs.py flight`의 `--no-plot`은 창을 생략하지만 matplotlib은 여전히
필요하다. 파일을 생략하면 `logs/flight_log_*.csv` 중 수정시각이 가장 최근인
파일을 선택한다. 요약에는 RP/Yaw scale의
최솟값·p05(5백분위수), LIMITED 진입 횟수·누적 시간, roll/pitch/yaw별
PID·I 기여의 절댓값 최댓값이 포함된다. 필요한 진단값이 없는 레거시 CSV는
해당 항목을 `legacy/unknown`으로 표시한다. LIMITED 시간은 `Timestamp`로
계산하며, 시간 자료가 없으면 `duration unavailable`, 상태·시간 결측이 있으면
`incomplete: unknown state/time gap`을 표시한다.

프로브 분석은 실행 시 현재 `.ino`에서 임계를 읽는다. 과거 비행 당시 임계와
같다는 뜻은 아니다. 20Hz에서 관측된 상태 전이 또는 응답값 변화로 이벤트를
추출하므로 일부 이벤트를 놓칠 수 있다. `BLOCKED`·0 응답을 제외하고,
표본 부족·`Hover_Est` 혼합은 판정을 보류한다. 종료코드 0은 분석 실행 성공이며
분포 “통과”를 뜻하지 않는다. `--plot`일 때만 matplotlib을 불러온다.

`drone_logs.py magcal`은 기본적으로 `Mag_Cal_Active=1`인 raw 행만 사용한다.
`--all`은 보정 전 raw만 따로 추린 파일임을 확인했을 때만 사용한다. 출력의
`95p |B| radial residual`은 자기장 크기 피팅 잔차로, heading 정확도가 아니다.
스로틀 간섭 계수 환산은 입력 좌표계가 맞아야 하며 새 보정 후 모터 벤치로
재검증한다. 로그 좌표계·시간축·결측 해석은 [logs/README.md](../logs/README.md).

## 카메라 녹화

기체의 ESP32-CAM 보드([`firmware/camera`](../firmware/camera/README.md))가 보내는 영상을
노트북에서 녹화한다. 조종 링크와는 별개이므로 `drone_control.py`와 **다른 터미널에서
동시에** 실행한다. 녹화기는 비행 UDP 포트를 쓰지 않는다.

사전 점검:

- 노트북이 `Drone_Tuning`에 붙어 있어야 한다. 유선 LAN이나 USB 테더링이 192.168.4.0/24로
  가는 경로를 가로채지 않는지 확인한다. 녹화기는 접속한 로컬 주소가 192.168.4.x가 아니면
  `[WARN]`을 찍는다.
- WiFi 절전을 끈다([링크 문서](../docs/ground_station_link.md)의 `powersave` 항목). 연결 프로필에
  절전을 꺼 두어도 커널에서 켜져 있던 적이 있다. 녹화 직전에 `iw dev wlan0 get power_save`로
  `off`인지 확인한다.

```bash
python scripts/drone_video.py                 # 미리보기 창 + 녹화 (q로 종료)
python scripts/drone_video.py --no-preview    # 창 없이 녹화 (Ctrl+C로 종료)
python scripts/drone_video.py mp4 logs/camera_YYYY-MM-DD_HHMMSS.csv
```

출력:

- `logs/camera_<시각>.mjpeg`: 받은 JPEG를 다시 인코딩하지 않고 그대로 이어 붙인 파일
- `logs/camera_<시각>.csv`: 프레임마다 한 행. 행에는 다음이 들어간다.
  - 카메라 일련번호와 캡처 시각
  - 노트북 수신 시각. 비행 로그와 같은 `HH:MM:SS.mmm` 형식이고, 헤더가 도착한 시각이다.
  - 접속 구간 번호
- 후처리 결과(`drone_video.py mp4`)
  - `_aligned.csv`: 프레임별 캡처 시각을 노트북 시계로 옮긴 표. 비행 로그 `Timestamp`와 맞춰 보는 데 쓴다.
  - `.mp4`: 일반 재생용 가변 프레임레이트 영상

카메라와 비행 로그의 시각은 사건 단위(약 ±100 ms) 대조용이다. 오프셋은 구간마다 '최소
전달 지연'으로 추정하므로, 정밀 운동 분석에 쓸 동기화는 아니다(한계는
[설계 문서 5.3](../docs/superpowers/specs/2026-11-02-camera-streaming-design.md)).
종료 요약의 `seq gaps`가 0이 아니면 카메라가 보낸 프레임이 녹화에서 빠진 것이다.

## 보관된 벤치 스크립트

`bench_sign_test.py`·`bench_yaw_test.py`·`bench_thrust_ramp.py`·`motor_serial.py`·
`receive_dual_imu_debug.py`는 [archive/](archive/README.md)로 옮겼다. 일부는 WiFi 전환·`start`·
주기적 RC를 자동으로 보내므로, 실행 전에 그 문서의 경고와 전제를 읽는다.

공유 모듈은 `ground/` 패키지의 `telemetry_schema.py`(파싱·CSV 정규화)와
`failsafe_telemetry.py`(phase·probe 이름과 표시)다. 이들은 실행 도구가 아니다.
[archive/](archive/README.md)의 GPS/TCP 도구는 현행 비행 펌웨어용이 아니다.
