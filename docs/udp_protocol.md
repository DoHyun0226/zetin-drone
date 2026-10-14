# UDP 제어·텔레메트리 프로토콜

현행 `dual_imu_cascade_pwm`와 지상 도구의 명령·필드 정본이다.
저스로틀 yaw 개선 통합 코드 기준이며, 실제 기체의 업로드 빌드는 별도 확인한다.
운영 순서는 [PC 도구 안내](../scripts/README.md), 링크 상실 진단은
[지상국 링크 문서](ground_station_link.md)를 먼저 참고한다.

```text
전송: UDP
드론 주소: 192.168.4.1
포트: 4210
등록: 수신되는 모든 패킷이 지상국 엔드포인트를 식별한다.
      현행 수신기는 주기적으로 "connect"를 보낸다.
```

펌웨어는 SSID `Drone_Tuning`으로 SoftAP를 운영한다. 주소와 포트는
[`dual_imu_cascade_pwm.ino`](../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino)에
정의돼 있다.
RC 송신 목표는 `control_dualsense.py`의 20Hz, ASCII 텔레메트리는 약 20Hz,
raw IMU 생산은 명목 1kHz다. 서로 다른 주기이므로 혼동하지 않는다.
텔레메트리 수신과 `connect` 응답 목적지 등록은 시동·RC 수락 ACK가 아니다.
펌웨어는 마지막 UDP 발신자의 IP/포트 하나로 회신하므로 제어·수신 도구를
동시에 실행하지 않는다.
보류 트랙인
[`dual_imu_flix_quat_pwm`](../firmware/flight/dual_imu_flix_quat_pwm/)은 같은
엔드포인트와 일부 `rc` 절대각 명령을 공유하지만 `rcr`은 지원하지 않는다.
RC 두절 시 자동착륙 없이 즉시 컷하며 **주력 지상국으로 조종할 수 없다.**
게인 명령의 단위가 SI(rad 기반)이고 yaw 각도 부호가 반대(CCW+)다.
`gains`·`mag`·`magcal`을 구현하지 않고 텔레메트리도 `Armed`까지 22필드만
보낸다. [해당 README](../firmware/flight/dual_imu_flix_quat_pwm/README.md)를
참조한다.

## 지상국 → 드론

명령은 UTF-8/ASCII 텍스트 데이터그램이다. 인자는 공백으로 구분한다.
다음은 와이어 명령이며 `control_dualsense.py` stdin에서는 `mag on/off`,
`raw on/off`를 각각 `mag 1/0`, `raw 1/0`으로 바꿔 보낸다. 그 콘솔의
`start`·`stop`·`resume`·`th`는 로컬 상태도 갱신하는 전용 경로다.
일반 명령에는 UDP ACK가 없고, `gains`만 별도 응답을 보낸다. 시리얼의
거부 메시지와 상태 텔레메트리를 확인해야 실제 적용을 알 수 있다.

```text
connect                            # 텔레메트리 목적지 등록용. RC 워치독 갱신 아님
start
stop
resume
rc <seq> <roll> <pitch> <yaw>
rc <roll> <pitch> [yaw]             # 무시퀀스 벤치 호환형. 조종용으로 쓰지 않음
rcr <seq> <roll> <pitch> <yaw_rate>  # yaw_rate는 dps
th <microseconds>
trim <roll_deg> <pitch_deg>          # 절대 트림(도), 각 축 ±10°로 클램프
yaw <0|1>
mag <0|1>                # 자기계 yaw 드리프트 보정 ON/OFF (기본 OFF)
magcal <0|1>             # raw 캡처 시작/종료. 시작은 시동 해제 상태에서만
magc <x> <y> <z>         # 모터 전류 간섭 보정 계수(µT/µs) 3축. "0 0 0" = 보정 off
raw <0|1>                # 1kHz dual-IMU 원시 배치 스트림 ON/OFF (기본 ON)
gains                    # 현재 PID 게인 12개를 1회 응답

# 안쪽 각속도 PID 게인
pa|ia|da <value>      # roll+pitch 공통 P/I/D
pr|ir|dr <value>      # roll P/I/D
pp|ip|dp <value>      # pitch P/I/D
py|iy|dy <value>      # yaw P/I/D
rp|ri|rd <value>      # roll+pitch 공통 P/I/D (pa|ia|da와 동일)
yp|yi|yd <value>      # yaw P/I/D (py|iy|dy와 동일)

# 바깥 각도 P 게인
ap <value>            # roll+pitch 공통
ar|at|ay <value>      # roll / pitch / yaw
```

- `start`는 캘리브레이션 성공, 기울기 정상, 사용 가능한 IMU 존재, IMU
  일치, `magcal` 미진행 조건을 모두 통과한 뒤에만 시동하며, latch된 fault를
  해제하고 스로틀 창을 기본값(base 1100, min 1050, max 1250)으로 리셋한다.
  이때 yaw 추정(`angleZ`)을 0으로 초기화하고 mag 기준 heading도 다시 잡는다.
  이미 시동됐거나 Core 1의 arm 적용을 기다리는 상태에서 도착한 중복 `start`는
  무시되므로, 지연 도착한 재전송이 fault latch나 스로틀 창을 되돌리지 않는다.
  자동착륙 하강 중(`Failsafe_Phase=1`)인 `start`는 `safety_lock` 값과 관계없이
  거부한다. 따라서 `stop` 적용과 phase 정리 사이에도 재무장하지 않는다.
  `Failsafe_Phase=2`(착지컷), 3(타임아웃컷), 4(중단컷)에서는 정상 재시동
  경로를 계속 허용한다. `stop`은 즉시 시동을 해제한다.
- 자동착륙 하강 중에는 설정 변경 명령 `trim`, `magc`,
  `pa`, `ia`, `da`, `pr`, `ir`, `dr`, `pp`, `ip`, `dp`, `py`, `iy`, `dy`,
  `rp`, `ri`, `rd`, `yp`, `yi`, `yd`, `ap`, `ar`, `at`, `ay`를 거부하고
  수신 패킷 처리 시 거부 로그를 한 번 남긴다. 읽기 전용 `gains` 조회는
  하강 중에도 계속 허용한다.
- `resume`은 자동착륙 하강 중(`Failsafe_Phase=1`)에만 조종권 복귀를
  요청한다. RC 패킷이 최근 500ms 안에 수신됐고, 과도 기울기가 아니고, 사용
  가능한 IMU가 하나 이상이며 두 IMU 동시 freeze·현재 disagree가 없고,
  `Hover_Valid=1`일 때만 수락한다. 수락하면
  `Failsafe_Phase=0`, `Fault_RC=0`, `base_throttle=round(Hover_Est)`로
  복원하고 스로틀 창도 ±150µs로 다시 잡는다. 조건이 하나라도 맞지 않으면
  시리얼에 `RESUME REFUSED <phase|rc|tilt|imu|hover|cumulative>`를 남기고
  하강을 계속한다. 한 비행의 최초 자동착륙 진입부터
  `FS_RESUME_MAX_MS = 3 × FS_MAX_MS`(현재 9초)가 지나면 `cumulative`로
  거부한다. 이 시각은 앞선 `resume`이 지우지 않으며, 거부 자체는 현재
  자동착륙을 강제 종료하지 않는다. 링크가 돌아온 것만으로 자동 복귀하지
  않으며 명시적 `resume`이 반드시 필요하다.
- **`resume`은 고도를 되찾지 못한다.** `Hover_Est` 복원은 추가 하강 가속도를
  0 근처로 줄일 뿐, 이미 쌓인 하강속도는 없애지 않는다. 링크가 돌아오면 즉시
  `resume`하고, 곧바로 스로틀을 올려 고도를 회복해야 한다. 늦게 보낼수록
  잔류 하강속도와 이후 고도 손실이 커진다.
- RC 스트림이 `RC_TIMEOUT_MS`(500ms)를 초과해 끊기면 `Fault_RC`를 세운다.
  `connect`·`th`·PID 명령은 이 워치독을 갱신하지 않는다.
  - 아직 `Hover_Valid=0`이면 사용할 호버 추정치가 없어 **즉시 컷**한다.
    `Failsafe_Phase`는 0에 머문다. 이 분기는 지상 여부를 직접 측정하지 않는다.
  - `Hover_Valid=1`이어도 `base_throttle ≤ FS_GROUND_CUT_MAX_US`(1150µs)일
    때만 지상 스로틀로 보고 즉시 컷한다. 지상 즉시컷 경계에는 `Hover_Est`를
    쓰지 않으므로, 등속 상승을 잘못 학습한 높은 추정치가 공중 즉시컷을 만들지
    못한다.
  - 그 외에는 **자동착륙**에 진입한다(`Failsafe_Phase=1`). 자세 목표는 적용
    중인 트림, yaw 각도는 진입 시점 heading 스냅샷, yaw 각속도 목표는 0이며,
    `Hover_Est - FS_DESCENT_DELTA_US`(초기 60µs)를 하강 스로틀로 쓴다.
    조종자가 상승 중이었어도 진입 스로틀을 기준으로 삼지 않는다. 하강 중에는
    정상 하강 collective 기준 스로틀 창(`min_throttle`/`max_throttle`)도 매
    tick 고정되므로 뒤늦게 도착한 `th`가 하강을 뒤집지 못한다.
- 호버 추정기는 무장·`FS_NONE` 상태에서 roll/pitch가 각각 ±10° 안이고,
  `|accel|`이 1g±0.05g이며, 스로틀이 1150µs를 넘는 샘플만 3초 시정수 LPF로
  추적한다. 해당 샘플 시간이 누적 1.5초가 되면 `Hover_Valid=1`이다. 이
  임계·시간 값은 모두 Stage E-0 벤치 조정 대상이다.
- **자동착륙은 착지를 감지하지 않는다. 시간 기반 하강이다.**
  `FS_MAX_MS = 3000`ms 동안 하강한 뒤 `Phase=3` 백스톱이 자른다. 즉 정상
  종료 위상은 항상 3이며, `Phase=2`(착지컷)는 **현재 펌웨어에서 도달하지
  않는다**(위상 번호는 wire contract라 값 자체는 유지한다).
  2026-08-01 실기에서 라벨된 하강 4건을 1kHz로 분석해 IMU 기반 후보 6종을
  평가했다. 원 `|accel|` 프로브와 HF/LF 비율은 판정 보류, 자이로 HF는
  조건부이며, 음성 대조군 검증이 부족해 현재 착지 컷에는 채택하지 않는다.
  원래 쓰던 프로브는 접지 상태에서도 응답이 임계 위로 나와 착지를 확정하지
  못했다. 공중 오판 컷의 위험을 0으로
  만드는 쪽을 택했고, 대가는 접지 후 남은 시간만큼 프롭이 도는 것이다.
  근거와 배제 과정은 [`failsafe_land_research.md`](failsafe_land_research.md).
- `FS_MAX_MS`는 **하강 예산 그 자체**다. 2026-08-01 접지 충격 임펄스로 역산한
  하강 가속도 평균 1.24 m/s²에서 3.0초는 5.60m를 덮으며, 이는 실측 최고
  시험 고도(1.86m)의 3배다. 컴파일 타임 `static_assert` 하한이 2720ms라
  2.5초는 빌드되지 않는다. 고도 센서로 착지를 **측정**할 수 있게 되기 전에는
  이 값을 늘리는 것이 곧 접지 후 프롭 회전 시간을 늘리는 것이다.
  **2026-09-07 해석 보완:** 위 가속도·고도·시간 수치는 당시 시험 조건의
  기록이다. 앞의 “공중 오판 컷의 위험을 0”은 프로브의 착지 오판 경로를 없앤
  범위로 한정한다. 시간 기반 컷이 모든 초기 고도·배터리·추정 오차에서 접지 이후에
  일어남을 보장하지 않는다. 거리계 필드는 추가됐지만 현재 착지컷에는 쓰지 않는다.
- 능동 스로틀 프로브는 **판정에서 빠졌을 뿐 계속 돈다.** 다음 판별식을
  오프라인 데이터로 설계하려면 그 기록이 필요하기 때문이며, 결과는 텔레메트리
  41~43번으로만 나간다(`landed`는 코드에서 상수 `false`다).
  `FS_MIN_DESCEND_MS`(1초) 뒤부터 400ms마다 정상 하강 collective를
  `round((Hover_Est − 1000) × FS_PROBE_DIP_FRAC)`만큼 낮춰 120ms 유지한다.
  `FS_PROBE_DIP_FRAC=0.118`이며 결과는 20µs 이상,
  `CTRL_MARGIN`(150µs) 미만으로 런타임 clamp된다. 명목
  `Hover_Est=1340µs`에서는 40µs이고, clamp가 걸리면 시리얼에
  `AUTO-LAND PROBE DIP CLAMPED`가 남는다. 딥 직전과 첫 30ms를 제외한 딥 중
  5Hz LPF `|accel|` 최솟값의 차분이 `FS_PROBE_RESPONSE_G`(0.03g)를 넘으면
  공중 반응으로 보고 연속 무반응 카운트를 지운다.
  `FS_PROBE_CONFIRM_N`(4)과 `FS_LAND_ACCEL_TOL_G`(0.10g)는 프로브 상태를
  계산할 때만 쓰이고 컷을 만들지 않는다. 딥 tick에는 collective 하한만 요청
  딥만큼 넓혀 자세 차동이 딥을 자르지 않게 한다. 그래도 표본 구간의 어느
  tick에서든 믹서 적용 collective가 요청 딥의 80% 미만이면 프로브 전체를
  `BLOCKED`(4)로 폐기하고, 시리얼 경고는 하강당 한 번으로 제한한다.
  적용 딥 뒤 collective가 1000µs 미만이면 유효한 프로브를 만들 수 없어
  상태를 `UNAVAILABLE`(3)로 남긴다. 어느 쪽이든 하강 종료는 백스톱이 한다.
- IMU 전멸·과도 기울기·명시적 `stop`은 계속 즉시 컷이다. 하강 중 이들이
  걸리면 `Phase=4`(중단컷)로 끝난다.
- `rcr`은 패킷 시퀀스와 목표 roll·pitch 각도, yaw 각속도(dps)를 담는 현행
  조종 명령이다. roll·pitch 목표는 ±30°, yaw 각속도는 펌웨어에서 ±180dps로
  제한된다. `control_dualsense.py`는 이 펌웨어 한도와 별개인
  `YAW_RATE_MAX_DPS = 90.0`으로 최대 스틱 명령을 ±90dps로 제한한다.
  시퀀스·드롭 계수·RC 워치독 상태는 `rc`와 공유한다.
- `rc`는 기존 벤치 도구 호환을 위해 그대로 유지되는 roll·pitch·yaw **각도**
  명령이다. roll·pitch 목표는 ±30°로 제한된다. yaw 필드는 벤치 지향
  `yaw 1` heading-hold 오버라이드에서만 사용하는 지원 인터페이스이며, 자동
  모드 조종에는 `rcr`을 사용한다. 시퀀스의 uint32 정방향 진행만 수락한다
  (정상 래핑 허용). 지연·중복은 아래 카운터 규칙대로 폐기된다.
- `th`는 기본 ESC 펄스 폭을 마이크로초 단위로 설정한다. 1000~1900으로
  제한되며, min/max 스로틀 창을 기본값 ±150 마진으로 함께 재설정한다.
- `trim`은 roll·pitch 트림의 절대값(도)을 설정한다. 각 축은 ±10°로
  클램프되며 자동착륙 하강 중이 아니면 시동 중에도 수락된다. 기체
  속성이므로 `start`와 `stop`은 지우지 않는다. roll·pitch 조종 목표에는
  트림을 더한 뒤 총합 ±30° 제한을 적용한다.
- `yaw 0`은 항상 수락되며 자동 모드로 돌아간다. 자동 모드가 기본값이다.
  스틱이 편향되면 `rcr`의 yaw 각속도를 추종하고, 스틱이 중립이면서 실제 yaw
  각속도까지 정착하면 회전이 멈춘 그 heading을 자동으로 잠근다. 실제 각속도가
  정착하지 않으면 강제로 잠그지 않고 rate 모드에 남는다.
- `yaw 1`은 heading을 고정하고 현재 heading으로 setpoint를 다시 슬레이빙하지
  않는 수동 오버라이드다. 단, 이 보장은 `Yaw_Authority_State=0`(NORMAL)일 때만
  성립한다. 1(LIMITED)·2(RECOVERING)에서는 override와 관계없이 `Yaw_Hold=0`,
  `TgtRate_Yaw=0`이며 목표 heading은 현재 heading을 따라간다. **시동 해제 상태에서만 수락**되고 무장 해제
  엣지에서 자동으로 지워지므로 비행마다 다시 켜야 하는 벤치 지향 opt-in이다.
  자동 잠금 판정 상수 `YAW_RATE_DEADZONE = 3dps`와
  `YAW_HOLD_SETTLE_DPS = 10dps`는 실기 벤치에서 조정한다.
- `mag`는 BMM350 자기계 기반 yaw 드리프트 보정을 켜거나 끈다. **기본값은
  OFF**이며, OFF일 때 yaw는 자이로 적분만으로 추정한다. 켜는 순간 현재 추정
  yaw를 기준으로 상대 heading 오프셋을 잡으므로 기수가 자북으로 돌아가지
  않는다(상대 heading 유지 전용, 자북·편각·항법 아님). 보정은 250Hz 바깥
  루프에서 시정수 약 4초로 천천히 끌어당긴다. `mag 1`은 `magcal` 진행 중과
  무장 상태의 최초 센서 초기화가 필요한 경우 거부된다. `mag 0`은 수락된다.
  `control_dualsense.py` stdin에서는 `mag on`/`mag off`로 선택하며, 다음
  `start` 직전에 선택값을 `mag 1`/`mag 0`으로 다시 전송한다. 지상국 선택
  기본은 ON이므로 펌웨어 부팅 기본 OFF와 구분한다.
- `magcal 1`은 hard/soft-iron 캘리브레이션용 raw 캡처를 시작한다(시동
  상태에서는 거부). 첫 유효 raw 샘플부터 `Mag_Cal_Active=1`과 보정 전
  BMM350 raw `Mag_X/Y/Z`를 같은 스냅샷으로 보내므로 지상국 CSV에 회전 전체가
  남는다. 모든 방향으로 회전한 뒤
  `magcal 0`을 보내면 시리얼에는 샘플 수와 축별 span만 출력된다. CSV에
  `scripts/magcal_fit.py`를 실행해 Li & Griffiths 제약 타원체 + MAD 클리핑으로
  `MAG_HARD_IRON`/`MAG_SOFT_IRON`과 재환산 `mag_comp`를 얻는다. 전체 절차와
  통과 기준은
  [`bmm350_yaw_bench_test.md`](bmm350_yaw_bench_test.md)를 따른다.
  `magcal 1`은 yaw 융합을 끄며, `magcal 0`이 자동으로 다시 켜지는 않는다.
  종료는 무장 여부와 무관하게 처리하고 보정 상수를 자동 저장하지 않는다.
- `magc x y z`는 모터 전류가 만드는 body-frame 자기 간섭을 보정하는 3축
  계수(µT/µs)를 런타임 설정한다. `mag.{x,y,z} -= k·(base_throttle − 1000)`을
  **hard/soft-iron 행렬·body 부호 변환 이후, 틸트보정 이전**에 적용한다.
  즉 현재 계수는 보정 후 XYZ(µT) 도메인이고 BMM350 raw 계수와 같지 않다.
  펌웨어 기본 `mag_comp_x/y/z`는 과거 벤치 계수를 행렬로 환산한 값이다.
  2026-08-04 보정 후 모터 간섭 재검증 기록과 그 수치의 재분석 범위는
  [`bmm350_yaw_bench_test.md`](bmm350_yaw_bench_test.md)에 남겼다.
  `magc 0 0 0`은 보정을 끄며, 하강 중 계수 변경은 거부한다.
  `magcal_fit.py`의 `--mag-comp`는 입력 벡터에 새 행렬을 곱할 뿐 기존 행렬을 역변환하지
  않는다. 이미 보정된 현행 계수를 그대로 넣어 이중 변환하지 말고, 입력의
  좌표계를 확인한 뒤 새 보정 기준에서 모터 간섭을 다시 측정한다.
- `raw 1`은 1kHz 제어 루프에서 읽은 두 IMU의 int16 레지스터 원값을
  `ZIMU` 배치로 보내며 `raw 0`은 중지한다. 기본값은 **ON**이다. 생산자는
  `raw_stream_enabled && connectionEstablished`일 때만 링에 넣으므로, 지상국
  연결 전에는 링을 채우거나 `dropped`를 올리지 않는다. 따라서 `dropped`는
  실제 스트리밍 중 소비자가 생산자를 따라가지 못한 경우만 뜻한다. 부호,
  bias/scale, 소프트웨어 LPF, body-frame 변환은 이 스트림에 적용하지 않는다.
  `control_dualsense.py` stdin에서는 각각 `raw on`, `raw off`로 보낸다.
- `gains`는 현재 cascade PID 게인 12개를 `GAINS,<...>` 데이터그램 한 개로
  요청을 보낸 지상국에 응답한다. 값은 소수점 아래 4자리로 전송한다.
- 게인 값은 0~100 범위의 유한한 수만 수락하며, 범위를 벗어나거나 파싱에
  실패한 명령은 무시된다. 자동착륙 하강 중에는 유효한 값도 변경하지 않는다.

수신된 데이터그램은 그 출발지 주소를 텔레메트리 목적지로도 등록한다.
[`receive_telemetry.py`](../scripts/receive_telemetry.py)와
[`monitor_telemetry.py`](../scripts/monitor_telemetry.py)는 이를 위해 주기적으로
`connect`를 보낸다.

### RC 순서와 카운터

시퀀스형 `rc`와 `rcr`은 `lastRcSeq`·워치독·카운터를 공유한다. `seq`는 부호
없는 0~4294967295 정수이며, 정방향 uint32 래핑을 수락한다. 정상 `start`는
시퀀스 비교 기준과 RC 시각을 초기화하지만 아래 누적 카운터는 지우지 않는다.

| 필드 | 증가 조건 |
|---|---|
| `RC_Total_Pkts` | 시퀀스·값 파싱이 유효한 시퀀스형 RC 데이터그램마다 +1. 순서 검사 전에 증가 |
| `RC_Dropped_Pkts` | 중복/오래된 패킷이면 +1 후 폐기. 정방향 시퀀스 틈이 있으면 빠진 번호 수만큼 추가 |

무시퀀스 벤치형 `rc`는 유효 목표와 워치독을 갱신하지만 위 카운터·시퀀스 기준을
갱신하지 않는다. 하강 중 수락된 RC도 워치독은 갱신하지만 조종 목표를 바꾸지
않는다. `resume`은 별도로 필요하다. 카운터의 `Dropped / Total`은 정확한
무선 유실률이 아니며 100%를 넘을 수도 있다.

게인·트림·mag/raw 선택 등의 런타임 변경은 재부팅 시 펌웨어 기본값으로 돌아간다.

## 드론 → 지상국

텔레메트리는 다음 75개 필드를 정확한 순서로 담은 쉼표 구분 데이터그램이다.

```text
Roll, Pitch, Yaw,
Gyro_X, Gyro_Y, Gyro_Z,
Accel_X, Accel_Y, Accel_Z,
Throttle,
Fault_RC, Fault_Critical,
RC_Total_Pkts, RC_Dropped_Pkts,
Fault_IMU1, Fault_IMU2, Fault_Disagree,
Active_IMUs, Mixer_Scaled, Fault_Attitude, Calibration_OK,
Armed,
Motor_M1, Motor_M2, Motor_M3, Motor_M4, PID_Loop_Hz,
TgtRate_Roll, TgtRate_Pitch, TgtRate_Yaw,
MagHeading, Mag_X, Mag_Y, Mag_Z,
Yaw_Hold,
Failsafe_Phase, Trim_Roll, Trim_Pitch,
Hover_Est, Hover_Valid,
Failsafe_Probe_State, Failsafe_Probe_NoResponse,
Failsafe_Probe_Response_G,
IMU1_Gyro_X, IMU1_Gyro_Y, IMU1_Gyro_Z,
IMU1_Accel_X, IMU1_Accel_Y, IMU1_Accel_Z,
IMU2_Gyro_X, IMU2_Gyro_Y, IMU2_Gyro_Z,
IMU2_Accel_X, IMU2_Accel_Y, IMU2_Accel_Z,
TgtAngle_Roll, TgtAngle_Pitch, TgtAngle_Yaw,
Mag_Enabled,
Range_MM, Range_Quality, Flow_X, Flow_Y, Flow_Quality,
Mag_Cal_Active,
Mixer_RP_Scale, Mixer_Yaw_Scale, Mixer_Collective_US,
PID_Roll_US, PID_Pitch_US, PID_Yaw_US,
I_Roll_US, I_Pitch_US, I_Yaw_US,
Yaw_Authority_State
```

기존 21개 필드 뒤에 `Armed`(22), Tier 1 관측 필드(23~30), `MagHeading`(31),
`Mag_X`/`Mag_Y`/`Mag_Z`(32~34), `Yaw_Hold`(35), `Failsafe_Phase`(36),
`Trim_Roll`/`Trim_Pitch`(37~38), `Hover_Est`(39), `Hover_Valid`(40),
프로브 진단(41~43), IMU1 gyro/accel(44~49), IMU2 gyro/accel(50~55)을
차례로 append하고 목표 roll/pitch/yaw 각도(56~58), 실제 mag 융합 상태(59),
3901-L0X 상태(60~64), 캘리브레이션 활성 상태(65), allocator·PID·적분기·
yaw 권한 상태(66~75)를 순서대로 append한다.

- `Armed`는 펌웨어 safety lock의 반전값이다. `start`가 거부되거나
  펌웨어가 스스로 시동을 해제한 것을 지상국이 이 필드로 감지한다.

| 순서 | 필드 | 타입 | 의미 |
|---|---|---|---|
| 1~3 | `Roll`, `Pitch`, `Yaw` | float | 추정 자세(도). yaw는 시동 때 0으로 초기화하는 상대 방위 |
| 4~6 | `Gyro_X`, `Gyro_Y`, `Gyro_Z` | float | 융합 body-frame 각속도(dps) |
| 7~9 | `Accel_X`, `Accel_Y`, `Accel_Z` | float | 융합 body-frame 가속도(g), 중력 성분 포함 |
| 10 | `Throttle` | int | 기본 collective 스로틀(µs). 개별 모터 출력과 다름 |
| 11 | `Fault_RC` | int | RC 타임아웃 latch. 정상 `start` 또는 수락된 `resume`에서 해제 |
| 12 | `Fault_Critical` | int | 사용 가능 IMU 0개·과도 기울기 latch·보정 실패 중 하나이면 1 |
| 13~14 | `RC_Total_Pkts`, `RC_Dropped_Pkts` | int | [RC 카운터](#rc-순서와-카운터). 부팅 후 uint32 누적 |
| 15~17 | `Fault_IMU1`, `Fault_IMU2`, `Fault_Disagree` | int | 개별 IMU 고장·불일치 이력 latch. 현재 사용 가능 개수와 구분 |
| 18 | `Active_IMUs` | int | 현재 융합에 사용하는 IMU 개수 |
| 19 | `Mixer_Scaled` | int | 믹서에서 자세 차동 출력을 축소한 상태 |
| 20 | `Fault_Attitude` | int | 과도 기울기 fault latch |
| 21 | `Calibration_OK` | int | 부팅 캘리브레이션 성공 여부 |
| 22 | `Armed` | int | safety lock이 풀려 있으면 1. 회전수·추력·공중 상태 측정이 아님 |
| 23~26 | `Motor_M1`~`Motor_M4` | int | 실제 모터 PWM 출력(µs), 시동 해제 시 1000 |
| 27 | `PID_Loop_Hz` | int | `pid_task`의 실측 루프 주파수(Hz) |
| 28~30 | `TgtRate_Roll`, `TgtRate_Pitch`, `TgtRate_Yaw` | float | 안쪽 PID가 추종할 목표 각속도(dps). yaw rate 모드는 각도 루프를 우회 |
| 31 | `MagHeading` | float | BMM350 틸트보정 heading(deg). throttle 간섭 보정 적용값. `mag 0`이면 갱신되지 않는다 |
| 32~34 | `Mag_X`, `Mag_Y`, `Mag_Z` | float | 같은 패킷의 `Mag_Cal_Active=0`이면 hard/soft-iron·body 부호·현재 throttle 간섭 보정 후 자기장 성분(µT), 1이면 보정 전 BMM350 raw µT |
| 35 | `Yaw_Hold` | int | 0=각속도 모드, 1=heading 잠금 |
| 36 | `Failsafe_Phase` | int | 0=정상, 1=하강 중, 2=착지컷(**현재 도달 불가**), 3=백스톱컷(시간 기반 하강의 정상 종료), 4=중단컷 |
| 37~38 | `Trim_Roll`, `Trim_Pitch` | float | 드론이 적용 중인 roll·pitch 트림(도) |
| 39 | `Hover_Est` | float | 유효한 호버 후보에서 LPF로 추정한 collective 스로틀(µs) |
| 40 | `Hover_Valid` | int | 0=미확정, 1=호버 후보 시간이 유효 기준을 충족 |
| 41 | `Failsafe_Probe_State` | int | 0=WAIT, 1=DIP, 2=EVALUATE, 3=UNAVAILABLE(1000µs 아래 딥 가드), 4=BLOCKED(표본 구간 중 요청 딥의 80% 미만 전달). **기록 전용 — 컷을 만들지 않는다** |
| 42 | `Failsafe_Probe_NoResponse` | int | 연속 무반응 프로브 수. 공중 반응이면 0으로 초기화. **기록 전용** |
| 43 | `Failsafe_Probe_Response_G` | float | 최근 프로브의 `딥 직전 LPF − 딥 중 LPF 최솟값`(g). **기록 전용** |
| 44 | `IMU1_Gyro_X` | float | IMU1 body-frame X 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 45 | `IMU1_Gyro_Y` | float | IMU1 body-frame Y 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 46 | `IMU1_Gyro_Z` | float | IMU1 body-frame Z 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 47 | `IMU1_Accel_X` | float | IMU1 body-frame X 가속도(g), 소프트웨어 LPF 없음 |
| 48 | `IMU1_Accel_Y` | float | IMU1 body-frame Y 가속도(g), 소프트웨어 LPF 없음 |
| 49 | `IMU1_Accel_Z` | float | IMU1 body-frame Z 가속도(g), 소프트웨어 LPF 없음 |
| 50 | `IMU2_Gyro_X` | float | IMU2 body-frame X 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 51 | `IMU2_Gyro_Y` | float | IMU2 body-frame Y 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 52 | `IMU2_Gyro_Z` | float | IMU2 body-frame Z 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 53 | `IMU2_Accel_X` | float | IMU2 body-frame X 가속도(g), 소프트웨어 LPF 없음 |
| 54 | `IMU2_Accel_Y` | float | IMU2 body-frame Y 가속도(g), 소프트웨어 LPF 없음 |
| 55 | `IMU2_Accel_Z` | float | IMU2 body-frame Z 가속도(g), 소프트웨어 LPF 없음 |
| 56 | `TgtAngle_Roll` | float | 펌웨어가 사용하는 roll 목표 각도(deg) |
| 57 | `TgtAngle_Pitch` | float | 펌웨어가 사용하는 pitch 목표 각도(deg) |
| 58 | `TgtAngle_Yaw` | float | yaw outer loop의 목표 방위(deg) |
| 59 | `Mag_Enabled` | int | 펌웨어가 실제 적용 중인 BMM350 yaw 융합 상태. 0=OFF, 1=ON |
| 60 | `Range_MM` | int | Matek 3901-L0X 거리계 **원값**(mm) |
| 61 | `Range_Quality` | int | 0~255, **−1 = 신선한 프레임 없음** |
| 62 | `Flow_X` | int | 광류 `motionX` |
| 63 | `Flow_Y` | int | 광류 `motionY` |
| 64 | `Flow_Quality` | int | 0~255, **−1 = 신선한 프레임 없음** |
| 65 | `Mag_Cal_Active` | int | 같은 패킷의 32~34번 도메인 식별자. 0=보정 후, 1=보정 전 raw. 시작 시 첫 raw와 함께 1이 되고 종료 시 보정값 복원과 함께 0이 된다 |
| 66 | `Mixer_RP_Scale` | float | 합성 roll·pitch 차동의 실제 전달 비율. RP 자체가 모터 span을 넘을 때만 1보다 작다 |
| 67 | `Mixer_Yaw_Scale` | float | RP 배정 뒤 남은 motor span에 들어간 yaw 차동 전달 비율 |
| 68 | `Mixer_Collective_US` | float | 차동을 보존하도록 allocator가 실제 적용한 collective(µs) |
| 69~71 | `PID_Roll_US`, `PID_Pitch_US`, `PID_Yaw_US` | float | 해당 1kHz tick에서 allocator에 입력한 축별 PID 모터 기여(µs) |
| 72~74 | `I_Roll_US`, `I_Pitch_US`, `I_Yaw_US` | float | 같은 tick PID 계산에 사용한, conditional update 이전 축별 적분 기여(µs) |
| 75 | `Yaw_Authority_State` | int | 0=NORMAL, 1=LIMITED, 2=RECOVERING |

필드 66~75는 한 PID tick의 같은 스냅샷이다. 잠금·시동 해제·IMU 전멸
경로에서는 scale 1, collective 1000µs, PID/I 0, 상태 NORMAL로 초기화된다.
`Mixer_Yaw_Scale≤0.5`가 150ms 이어지면 수동 비행의 상태가 LIMITED로
들어가 heading을 현재 yaw에 양보하고 `TgtRate_Yaw=0`을 명령한다. scale이
0.9 이상이고 `|Gyro_Z|<10dps`인 회복 조건을 만족하면 RECOVERING으로 들어가며,
500ms 연속 회복 뒤 NORMAL로 돌아간다. yaw 스틱 입력, failsafe, 잠금 또는 새
무장은 상태와 타이머를 초기화한다.

### 3901-L0X 필드 해석 (필드 60~64)

`Range_MM`은 모듈이 보낸 값 그대로다. **작동 범위가 80~2000mm 이고 그 밖이면
음수**인데, iNav 구조체 주석이 `Negative value for out of range` 하나뿐이라
**너무 가까울 때와 너무 멀 때가 같은 음수**다. 부호만으로 위/아래를 구분할 수
없다. 현재 이 센서는 기록 전용이며 접지 판정을 수행하지 않는다. 향후 판별식을
설계하려면 직전 유효 거리·품질·신선도와 독립 접지 근거를 함께 검증해야 한다.

`*_Quality`가 **−1이면 신선한 프레임이 없다**는 뜻이다(펌웨어
`MSP_SENSOR_STALE_MS`). 거리 값 자체를 센티넬로 못 쓰기 때문에 신선도는 반드시
quality로 판단한다 — 얼어붙은 센서가 마지막 값으로 유효해 보이면 안 된다.

필드 수신 코드와 텔레메트리가 있어도 현행 비행 제어에는 사용 가능한
**optical-flow 수평 위치 폐루프가 없다.** `Flow_X/Y`는 기록값일 뿐 제어 입력이
아니며, 수평 드리프트는 조종자가 수동으로 보정해야 한다.

프로토콜은 INAV MSPv2다. `'$' 'X' <dir>` 뒤에 `flags(u8) cmd(u16 LE)
size(u16 LE) payload crc8`이 오고, **CRC(crc8_dvb_s2, 다항식 0xD5, 초기값 0)는
방향 바이트 다음부터** 적용된다. `MSP2_SENSOR_RANGEFINDER=0x1F01`
(`u8 quality; i32 distanceMm`), `MSP2_SENSOR_OPTIC_FLOW=0x1F02`
(`u8 quality; i32 motionX; i32 motionY`). 배선은 모듈 TX → ESP32 GPIO16
(수신 전용, 3.3V 직결), 전원은 **반드시 5V**다(3.3V로 주면 측정거리가 줄어든다).

목표 각도 필드는 다음처럼 해석해야 한다.

- 일반 비행의 `TgtAngle_Roll`/`TgtAngle_Pitch`는
  `constrain(command + trim, ...)` 결과이므로 트림이 이미 합산된 setpoint다.
  `Trim_Roll`/`Trim_Pitch`를 다시 더하면 이중 계산이다.
- 자동착륙 하강 중(`Failsafe_Phase == 1`)에는 `trim_roll`, `trim_pitch`,
  `fs_hold_yaw`가 들어간다. 즉 하강 중 수평 setpoint와 유지 방위를 나타낸다.
- `TgtAngle_Yaw`는 `yawOuter.target_angle_deg`다. `Yaw_Hold=1`이면 유지 중인
  방위이고, `Yaw_Hold=0`이면 현재 heading에 슬레이빙된 값이다.

`MagHeading`(31번)은 mag를 꺼도 마지막 값이 남으므로 융합 활성 여부를
판정할 수 없다. 실제 ON/OFF 상태의 유일한 근거는 `Mag_Enabled`(59번)다.
`Yaw`(3번)와 `MagHeading`의 차이는 융합 거동 분석에만 사용한다.
`Mag_X`~`Mag_Z`는 모터 전류 간섭 특성화/검증에 쓴다. mag가 꺼져 있으면
정상 도메인 값도 갱신되지 않을 수 있으므로, 스로틀 램프 중 평평한 값만으로
보정 성공을 판정하지 않는다. 같은 자세·센서 갱신·융합 활성 상태를 확인한다.

IMU별 필드는 융합 `Gyro_*`/`Accel_*`와 동일한 body frame이다.
동일 제어 tick에서 `Active_IMUs == 2`이고 `Fault_Disagree == 0`이면
융합 계산은 다음과 같다.

```text
Gyro_axis  = (IMU1_Gyro_axis  + IMU2_Gyro_axis)  / 2
Accel_axis = (IMU1_Accel_axis + IMU2_Accel_axis) / 2
```

IMU별 12값 묶음은 별도 스냅샷이지만 ASCII 패킷 전체가 한 tick의 원자적
스냅샷은 아니다. 다른 필드와 시점이 어긋날 수 있으므로 CSV 행마다 위 식을
엄격한 등식으로 강제하거나 작은 차이를 센서 고장으로 단정하지 않는다.

이 필드는 1kHz 센서 루프의 원본 스트림이 아니라 20Hz 텔레메트리
스냅샷이다. Nyquist 주파수는 10Hz이므로 프롭 진동은 앨리어싱되며, 이
필드만으로 진동·순간 disagree·freeze를 판정할 수 없다. 용도는 IMU별 느린
bias/scale 드리프트와 정상상태 오프셋 관찰로 한정한다. accel은 소프트웨어
LPF가 없으므로 20Hz 로그에서도 gyro보다 시끄러운 것이 정상이다.

`gains` 명령의 one-shot 응답은 텔레메트리와 별도인 다음 형식이다.

```text
GAINS,
Kp_Angle_Roll, Kp_Angle_Pitch, Kp_Angle_Yaw,
Kp_Rate_Roll, Kp_Rate_Pitch, Kp_Rate_Yaw,
Ki_Rate_Roll, Ki_Rate_Pitch, Ki_Rate_Yaw,
Kd_Rate_Roll, Kd_Rate_Pitch, Kd_Rate_Yaw
```

위 이름은 설명을 위한 것이며 실제 패킷에는 같은 순서의 숫자 12개만 들어간다.
수신기는 `GAINS,` 접두사를 먼저 분리하므로 이 응답을 텔레메트리 불량으로
계수하지 않는다. `tune_pid.py`는 각 이름과 값을 한 줄로 출력한다.


`telemetry_schema.py`는 과거 패킷도 받아들인다. 아래 길이는 역사적 확장
경계이며 허용 길이의 화이트리스트가 아니다. 실제 파서는 **10필드 이상**이면
알고 있는 앞쪽 75필드를 읽고 더 뒤의 미래 필드는 무시한다. 첫 10필드는
빈 값 없이 유한수여야 하고 정수 필드는 정수값이어야 한다. 이후 빈 값·없는
필드는 `None`으로 유지한다. 따라서 파싱 성공만으로 정확한 펌웨어 버전을
식별할 수 없다.

- 10필드 패킷은 `Throttle`에서 끝난다.
- 14필드 패킷은 `RC_Dropped_Pkts`에서 끝난다.
- 21필드 패킷은 `Calibration_OK`에서 끝난다 (`Armed` 도입 이전 펌웨어).
- 22필드 패킷은 `Armed`에서 끝난다 (Tier 1 관측 도입 이전 펌웨어).
- 30필드 패킷은 `TgtRate_Yaw`에서 끝난다 (BMM350 mag 융합 도입 이전 펌웨어).
- 31필드 패킷은 `MagHeading`에서 끝난다 (Mag XYZ 텔레메트리 도입 이전 펌웨어).
- 34필드 패킷은 `Mag_Z`에서 끝난다 (`Yaw_Hold` 도입 이전 펌웨어).
- 35필드 패킷은 `Yaw_Hold`에서 끝난다 (자동착륙·기체 트림 도입 이전 펌웨어).
- 38필드 패킷은 `Trim_Pitch`에서 끝난다 (호버 추정 텔레메트리 도입 이전 펌웨어).
- 40필드 패킷은 `Hover_Valid`에서 끝난다 (능동 프로브 진단 도입 이전 펌웨어).
- 43필드 패킷은 `Failsafe_Probe_Response_G`에서 끝난다 (IMU별 텔레메트리 도입 이전 펌웨어).
- 55필드 패킷은 `IMU2_Accel_Z`에서 끝난다 (목표 각도 텔레메트리 도입 이전 펌웨어).
- 58필드 패킷은 `TgtAngle_Yaw`에서 끝난다 (mag 상태 텔레메트리 도입 이전 펌웨어).
- 59필드 패킷은 `Mag_Enabled`에서 끝난다 (3901-L0X 텔레메트리 도입 이전 펌웨어).
- 64필드 패킷은 `Flow_Quality`에서 끝난다 (`Mag_Cal_Active` 도입 이전 펌웨어).
- 65필드 패킷은 `Mag_Cal_Active`에서 끝난다 (allocator 진단 도입 이전 펌웨어).
- 과거 패킷에 없는 값은 정규화된 CSV에서 빈 셀이 된다.
- `Timestamp`는 드론이 보내지 않는다. 지상 도구가 CSV의 첫 열로 추가하므로
  현행 CSV는 76개 열이 된다.

공유 구현은
[`telemetry_schema.py`](../scripts/telemetry_schema.py)에 있다.

### 1kHz IMU 원시 배치 (`ZIMU`)

raw 게이트가 ON이고 `connectionEstablished`일 때 전송하는 리틀엔디언
바이너리 데이터그램이다. 게이트 기본값은 ON이지만 첫 지상국 데이터그램을
받기 전에는 생산도 시작하지 않는다. 1kHz 샘플을 50개씩 묶어 정상상태
패킷률을 20pkt/s로 제한한다. 소비자는 10ms마다 확인해 50개가 모이면 보내고,
마지막 `ZIMU` 전송 후 50ms가 지나면 1~49개 부분 배치를 보낸다. 한
`udp_task` 반복에서 추가 raw 데이터그램은 최대 한 개다.

헤더는 20바이트다.

| 오프셋 | 필드 | 타입 | 크기 | 의미 |
|---:|---|---|---:|---|
| 0 | `magic` | `char[4]` | 4 | ASCII `ZIMU` |
| 4 | `version` | `uint8` | 1 | `2` (v1 = 26바이트 샘플, v2 = 27바이트). 디코더는 둘 다 읽는다 |
| 5 | `n_samples` | `uint8` | 1 | 1~50 |
| 6 | `reserved` | `uint16` | 2 | `0` |
| 8 | `batch_seq` | `uint32` | 4 | 배치마다 단조 증가, uint32 래핑 허용 |
| 12 | `base_t_us` | `uint32` | 4 | 첫 샘플의 `micros()` |
| 16 | `dropped` | `uint32` | 4 | 생산자 링 포화로 버린 신규 샘플 누적 수 |

헤더 뒤에는 다음 27바이트 샘플이 `n_samples`개 이어진다(v1은 26바이트로
`failsafe_phase`가 없다).

| 샘플 내 오프셋 | 필드 | 타입 | 크기 | 의미 |
|---:|---|---|---:|---|
| 0 | `dt_us` | `uint16` | 2 | 직전 기록 샘플과의 시간차. 배치 첫 샘플도 이전 배치 마지막 샘플 기준 |
| 2 | `imu1_gyro` | `int16[3]` | 6 | `e1.gyro[0..2]` 레지스터 원값 |
| 8 | `imu1_accel` | `int16[3]` | 6 | `e1.accel[0..2]` 레지스터 원값 |
| 14 | `imu2_gyro` | `int16[3]` | 6 | `e2.gyro[0..2]` 레지스터 원값 |
| 20 | `imu2_accel` | `int16[3]` | 6 | `e2.accel[0..2]` 레지스터 원값 |
| 26 | `failsafe_phase` | `uint8` | 1 | **v2부터.** 기록 시점의 `Failsafe_Phase`. 직전 tick 값이라 최대 1ms stale |

최대 크기는 `20 + 50 × 27 = 1370B`로 1472바이트 UDP payload 한도보다
작다.

`failsafe_phase`가 v2에 추가된 이유: `.bin`은 펌웨어 `micros()`, 20Hz CSV는
PC 벽시계라 **공통 키가 없어 1kHz 로그를 자동착륙 구간으로 자를 수 없었다.**
2026-08-01 착지 판별식 분석이 정확히 여기서 막혔다. 이 바이트 하나로 위상별
구간 추출이 가능해지고, 이미 확보한 라벨된 비행들이 오프라인 평가셋이 된다
(→ `failsafe_land_research.md`). 링이 가득 차면 기존 샘플을 보존하고 새 샘플만 버린다. 따라서
`dropped` 증가는 생산자/CPU/링 부족이고, `batch_seq` 불연속은 별도의 무선
손실이다. 분석 시 두 결측 원인을 따로 보고해야 한다.

### 보정상수 (`ZCAL`)

raw 스트림이 켜져 있는 동안 1초에 한 번 보내는 60바이트 리틀엔디언
데이터그램이다.

| 오프셋 | 필드 | 타입 | 크기 | 의미 |
|---:|---|---|---:|---|
| 0 | `magic` | `char[4]` | 4 | ASCII `ZCAL` |
| 4 | `version` | `uint8` | 1 | 현재 `2`. v1·v2 모두 동일한 60바이트 레이아웃이며 디코더가 둘 다 지원 |
| 5 | `reserved` | `uint8[3]` | 3 | 모두 `0` |
| 8 | `gyro_bias1` | `float[3]` | 12 | IMU1 부팅 보정 bias(dps) |
| 20 | `gyro_bias2` | `float[3]` | 12 | IMU2 부팅 보정 bias(dps), IMU1 센서축 기준 |
| 32 | `accel_scale1` | `float` | 4 | IMU1 부팅 accel 보정 배율 |
| 36 | `accel_scale2` | `float` | 4 | IMU2 부팅 accel 보정 배율 |
| 40 | `gyro_scale` | `float` | 4 | `GYRO_SCALE` |
| 44 | `accel_scale` | `float` | 4 | `ACCEL_SCALE` |
| 48 | `imu2_sign` | `float[3]` | 12 | `IMU2_SIGN_X/Y/Z` |

`ZIMU`, `ZCAL`, 현행 75필드 ASCII 텔레메트리는 모두 **같은 UDP 포트
4210, 같은 `laptopIP`/`laptopPort` 목적지**로 온다. 지상국은 UTF-8
디코드보다 먼저 첫 4바이트를 검사해 `ZIMU`/`ZCAL`을 바이너리 로그로
분기해야 한다. 그 외 데이터그램만 ASCII/`GAINS` 파서로 넘긴다.

하드웨어 LPF는 gyro 121Hz, accel 25Hz로 설정돼 있다
(`inv_imu_set_gyro_ln_bw`, `inv_imu_set_accel_ln_bw`). 대역 밖 성분은 감쇠하며
1kHz로 기록한다고 소실·감쇠된 원래 진동 스펙트럼이 복원되지는 않는다.
기록 주파수를 올리는 것은 센서 ODR이나 하드웨어 LPF 대역을 바꾸지 않는다.
