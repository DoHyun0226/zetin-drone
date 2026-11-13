# 주력 비행 펌웨어 일원화와 모듈화 계획 (2026-11-13)

## 목적

비행 펌웨어를 하나(`zetin_fc`)로 모으고, 3,503줄 단일 `.ino` 를 사람이 읽고 나눠 맡을 수 있는
모듈로 나눈다. **비행 동작은 한 비트도 바꾸지 않는다.** 기준은 main `a41a6f6`
(`Merge feat/range-autoland`)의 `firmware/flight/dual_imu_cascade_flow_hold/` 디렉터리 전체다.

## 사용자가 정한 것 (2026-11-13)

- flow-hold 스케치를 유일한 주력으로. `dual_imu_cascade_pwm` 은 archive.
- range-autoland 를 기본 OFF 스위치로 먼저 main 에 머지(완료, `a41a6f6`, CI firmware·python 성공).
- 범위: 파일 분할 + `pid_task`·`udp_task` 함수 분해.
- 폴더 이름 `firmware/flight/zetin_fc/zetin_fc.ino`.
- 보류 후보 `dual_imu_flix_quat_pwm` 도 archive.

## 반목표 — 이렇게 되면 실패다

- 기체 밖으로 나가는 것(모터 PWM 쓰기, UDP 패킷 전부, 시리얼 로그, NVS 쓰기) 중 하나라도 기준과 달라진다.
- 시험이 약해진다: 기존 단정을 지우거나 느슨하게 해서 통과시킨다. 사실이 바뀐 기대값(필드 수 등)만
  바꾸고, 바꾼 줄마다 이유를 커밋에 적는다. 이식한 뮤테이션 시험은 여전히 단정 실패를 일으켜야 한다.
- Core 0/Core 1 공유 변수의 소유·임계구역·읽는 횟수와 시점이 바뀐다.
- 전역 정적 초기화 순서가 바뀐다(`flowOdomCfg = flowOdomDefaultConfig()`, `posHoldGains`,
  `rlAltHoldGains`, `IMU1/IMU2/bmm` 생성자 등).
- Arduino 빌드가 깨지거나, 호스트 SIL 만 같고 ESP32 빌드는 다르다.
- CI 의 "unittest skip 0건" 조건이 깨진다.

## 현재 구조 (a41a6f6, 직접 확인)

- `dual_imu_cascade_flow_hold.ino` 3,503줄. 구획: 와이어 형식·raw 링·지연 링(34–230, 189–230 은
  `#if WIFI_LATENCY_DEBUG`), 튜닝 파라미터(231–272), 시스템 상수(274–533), 유틸(535–727), 시스템 변수·
  요청 우편함·자기계(728–1370), 모터(1371–1390), IMU 보정(1391–1610), 자세 alpha(1611–1663),
  `pid_task`(1665–2600, 936줄), `udp_task`(2602–3409), `setup`/`loop`(3411–3503).
- `pid_task` 의 실제 실행 순서(단계가 교차한다): 센서 읽기(1743–1808) → IMU 융합(1809–1864, **1863
  `continue`**: 사용 가능 IMU 0) → 자세 추정(1869–1938) → 과기울기·resume·인계·무장 에지(1939–2005) →
  거리·광류 갱신, `rangeLandStep`, 위치·고도 유지 후보(2007–2190) → hover 추정, RC 상실 진입(이번 tick 의
  `heightFresh`·거리 추적·`hover_valid` 사용), 거리 기반·시간 기반 하강(2192–2406) → hold 적용 확정·
  `publishFlowHoldTelemetry`(2407–2456) → `safety_lock` 처리(**2476 `continue`**) → outer(250 Hz) →
  inner(1 kHz) → desaturation·anti-windup·모터 출력(**2589 `continue`**)·텔레메트리 게시.
  루프 밖 automatic 지역 변수(LPF, freeze 감시기 `fm1/fm2`, 분주 카운터, 이전 gyro, `landDet`,
  `e1/e2` 등 약 26개)는 tick 사이에 값을 유지한다. 루프 안 지역 변수(약 35개)는 매 tick 새로 시작한다.
  `pid_task` 안에 `static` 지역 선언은 없다. 태스크 스택 4096 B(3467행).
- `udp_task` 의 루프 밖 지역 `lastSend`·`lastRawCheck` 는 태스크 수명 동안 유지된다(3062행).
- 순수 로직 헤더 10개(`control_allocator.h` `failsafe_land.h` `flow_calibration.h` `flow_hold.h`
  `flow_odometry.h` `mag_yaw_fusion.h` `msp_sensor.h` `range_land.h` `yaw_authority.h`
  `yaw_command.h`). 6개는 `dual_imu_cascade_pwm` 과 바이트 동일 강제(`tools/test_flow_hold_sketch.py`).
- 스케치 소스를 쓰는 곳:
  - `.ino` 통째 include: `test_sil_flow_hold.cpp`(flow-hold), `test_control_math.cpp`·
    `test_sil_attitude.cpp`·`test_mag_yaw_fusion.cpp`(cascade_pwm).
  - 헤더 `-I SKETCH_DIR`: `test_control_allocator.py` `test_yaw_authority.py` `test_yaw_command.py`
    `test_msp_sensor.py` `test_mag_yaw_fusion.py`(+뮤테이션 대상 경로) `test_failsafe_land.py` 등.
  - `.ino` 텍스트를 정규식으로 읽거나 고침: `tools/test_failsafe_land.py`(상수·문장 검사),
    `tools/test_native_control_math.py`(`FS_DESCENT_DELTA_US` 뮤테이션 1종 → 컴파일 실패 기대),
    **`scripts/ground/probe_response.py`**(`FIRMWARE_PATH` 하드코딩, `read_firmware_float_constant`) 와
    그 시험 `tools/test_analyze_probe_response.py`.
  - `tools/check_repo_layout.py`: stale 검사는 `firmware` 아래 `.ino` 만 본다(147행), migration map 의
    "새 경로" 존재 검사(`docs/migration_map.md` 9행이 cascade_pwm).
- 2026-11-13 이식 탐색: cascade_pwm 용 native 3개를 flow-hold `.ino` 로 바꿔 끼우면
  `test_control_math` 는 `publishAllocationTelemetry` 인자 수(flow-hold 9개 vs 시험 호출 8개,
  `requestedUs`) 컴파일 오류, `test_sil_attitude`·`test_mag_yaw_fusion` 은 필드 수 단정 실패. 두 스케치의
  기본값 차이도 시험에 영향: `trim_roll/pitch`(0 vs −2.2/−1.8), `Ki_Rate_Roll/Pitch`(0.05 vs 0.20),
  `pollMspSensor` 뒤 flow 상태 오염(이식 시험 리셋이 `resetFlowHoldForHarness()` 를 부르지 않음).
  `test_sil_attitude.cpp:1582` 는 75필드의 끝을 패킷 전체의 접미사로 검사한다.
- 2026-11-13 Arduino 실험(로컬 `arduino-cli` + `esp32:esp32@3.3.10`, CI 와 같은 라이브러리):
  순수 로직 헤더를 `logic/` 하위 폴더로 옮기고 `#include "logic/x.h"` 로 바꾼 사본이 컴파일되며, 기준과
  플래시 991,468 B·RAM 81,628 B 가 같고 `objdump -d` 디스어셈블리가 첫 줄(파일 이름)만 다르다. `.bin` 은
  앱 설명자(프로젝트 이름·빌드 시각·해시)에서 68바이트 다르다.

## 설계

### 단일 번역 단위 유지

Arduino 는 루트의 `.ino` 를 하나로 이어 붙이고, 루트와 `src/` 의 `.cpp` 는 따로 컴파일한다. SIL 은
`.ino` 를 C++ 한 파일로 include 해 내부 전역을 직접 읽고 쓴다. 따라서 **모듈은 `.ino` 가 차례로
include 하는 헤더**로 만든다. 이유: 전역 정적 초기화 순서 보존, `static` 함수·우편함 유지, 시험이 한
줄 include 로 전체를 계속 봄, 아래 A 단계의 ELF 동일 증명이 가능. 대가: 헤더는 독립 include 불가,
순서가 계약 — 각 `fc/*.h` 머리에 "zetin_fc.ino 에서 이 순서로만 include, 앞에 필요한 것"을 적는다.

### 폴더

```
firmware/flight/zetin_fc/
  zetin_fc.ino        include 순서 + setup()/loop()
  logic/              순수 로직(호스트 단위 시험 대상, Arduino 의존 없음) — 기존 10개 헤더
  fc/                 펌웨어 모듈(Arduino·FreeRTOS 의존, .ino 에서만 include)
  README.md           모듈 지도: 파일별 내용, 어느 코어가 쓰는지, include 순서
```

하위 폴더 헤더 include 는 위 실험과 Codex 가 확인한 arduino-cli 파일 수집 규칙으로 동작한다.

### 스케치 소스 읽기 공용 도우미

`scripts/ground/firmware_source.py`(지상국 분석 도구가 먼저 쓰고, 시험은 `scripts` 를 path 에 넣어
import): `sketch_dir()`, `sketch_files(dir)`(`.ino` + `fc/*.h` + `logic/*.h`, 정해진 순서),
`find_unique(pattern)`(모든 파일에서 정확히 한 번 — 0 또는 2 이상이면 오류), `mutate_unique(dir, old, new)`
(정확히 한 파일·한 곳을 바꾸고 그 파일 경로를 돌려줌). `probe_response.py`·`test_failsafe_land.py`·
`test_native_control_math.py`·`test_mag_yaw_fusion.py` 와 `check_repo_layout.py`(stale 검사에 `.h` 포함)가
이것을 쓴다.

### A 단계 — 기계적 분할

원본 순서 그대로 연속 구간을 잘라 `fc/*.h` 로 옮기고 `.ino` 가 같은 순서로 include 한다. 문장·주석은
고치지 않는다(파일 머리 주석과 `#pragma once` 만 더함). 구간(줄 번호는 a41a6f6, 조건부 컴파일 블록
189–230, 637–687, 1733–1738, 3065–3084, 3395–3404, 3479–3502 는 모두 한 구간 안):

| 파일 | 원본 줄 | 내용 |
|---|---|---|
| `fc/wire_types.h` | 34–230 | ZIMU/ZCAL 형식, 텔레메트리 샘플 구조체, raw 링, 지연 이벤트 링 |
| `fc/config.h` | 231–533 | 1. 튜닝 파라미터, 2. 시스템 상수 |
| `fc/util.h` | 535–727 | 3. 유틸(LPF, 링 함수, freeze, `rcTimedOut`) |
| `fc/state_core.h` | 728–781 | 4. 네트워크·IMU 객체·안전·failsafe·MSP 상태 |
| `fc/state_flow.h` | 782–827 | 광류 위치·고도 유지 상태 |
| `fc/state_rangeland.h` | 828–998 | 거리 기반 자동착륙 상태, 우편함, 하네스 리셋 |
| `fc/state_attitude.h` | 999–1076 | 호버 추정·목표·자세·자기계·fault 상태 |
| `fc/requests.h` | 1077–1250 | 안전·resume·자기계·IMU 스냅숏 요청/게시 |
| `fc/magnetometer.h` | 1251–1370 | BMM350 초기화·보정·샘플링 |
| `fc/motor.h` | 1371–1390 | 5. 모터 |
| `fc/imu_calibration.h` | 1391–1610 | 6. gyro bias 보정·NVS |
| `fc/attitude_alpha.h` | 1611–1663 | 7. 적응 alpha |
| `fc/pid_task.h` | 1664–2600 | 8. `pid_task` |
| `fc/udp_task.h` | 2601–3409 | 9. `udp_task` |
| `zetin_fc.ino` | 1–33, 3410–3503 | include, 10. setup/loop |

증명(둘 다 통과해야 한다):
1. **ESP32 ELF 동일(주 증명).** 기준(a41a6f6 스케치 디렉터리 전체를 `git archive` 로 임시 폴더에 고정,
   같은 이름 `zetin_fc` 로)과 새 스케치를 같은 `arduino-cli`·코어·라이브러리·FQBN·옵션으로 빌드하고 `.elf`
   를 비교한다(`.bin` 은 체크섬·SHA 가 연쇄로 바뀌어 차이 위치를 판정할 수 없으므로 쓰지 않는다): ELF entry,
   모든 `SHF_ALLOC` 섹션의 이름·주소·크기·정렬·flags(`.bss` 등 `NOBITS` 포함, `readelf -S`), 내용이 있는
   할당 섹션의 바이트(`objdump -s`), 디스어셈블리(`objdump -d`), 초기화 배열. 앱 설명자
   (`esp_app_desc_t`)는 통째로 빼지 않고 `time`·`date`·`app_elf_sha256` 필드만 마스킹한다(`secure_version`·
   eFuse revision·MMU page size 등은 비교). `WIFI_LATENCY_DEBUG=0`(기본)과 `=1`(`--build-property compiler.cpp.extra_flags=
   -DWIFI_LATENCY_DEBUG=1`) 두 번. Arduino 가 만드는 함수 원형은 생성된 `.ino.cpp` 를 비교해 차이를 기록한다.
   이름이 같아야 `__FILE__` 류 문자열이 같으므로 두 빌드 모두 `zetin_fc` 이름·같은 깊이의 경로로 둔다.
2. **호스트 전처리 동일.** `g++ -E -P -x c++` + native shim 으로 두 `.ino` 를 전처리해, **출력이 비어 있지
   않음**을 확인하고 빈 줄만 뺀 텍스트가 문자 단위로 같아야 한다(문자열 리터럴 안 공백도 그대로 비교).
   SIL·native 시험이 보는 프로그램이 같다는 증명이다.

### B 단계 — 함수 분해와 재배치

규칙(어기면 차분 결과와 무관하게 실패로 본다):
- **블록 순서를 바꾸지 않는다.** 단계 함수의 경계는 위 "실제 실행 순서"를 따른다:
  `pidReadSensors` → `pidFuseImus`(→ `TickFlow::Skip` 반환 가능) → `pidEstimateAttitude` →
  `pidSafetyAndResume` → `pidFlowUpdateAndHoldCandidates` → `pidHoverAndFailsafe` →
  `pidApplyHoldAndPublish` → `pidSafetyGate`(→ Skip) → `pidOuterLoop` → `pidInnerLoop` →
  `pidMixAndOutput`(→ Skip). 호출부는 Skip 이면 그 tick 의 남은 단계를 건너뛴다(원래 `continue` 위치와 같은
  지점). 조건식의 평가 순서·단락 평가를 바꾸지 않는다.
- **상태 수명을 나눈다.** 루프 밖 automatic 지역 → `PidLoopState`(`pid_task` 진입 때 스택에 한 번 생성,
  원래 초기값 그대로). 루프 안 지역 → `PidTick`(매 tick 루프 몸통 첫 줄에서 값 초기화). 둘 다 `static`·
  전역으로 올리지 않는다(시험이 `runTicks` 마다 `pid_task` 를 새로 시작하는 동작을 보존).
- **공유 전역은 구조체에 복사하지 않는다.** `volatile`·mux 로 보호되는 전역은 원래 읽던 자리에서 원래 횟수만큼
  읽는다(ESP32 의 `constrain` 은 인자를 여러 번 평가하는 매크로다 — 식을 그대로 둔다). 임계구역 범위·
  `__atomic_*` 순서를 그대로 옮긴다.
- `pid_task` 단계 함수와 `udp_task` 명령 처리 함수는 `static inline __attribute__((always_inline))` — 스택
  프레임을 하나로 유지해 태스크 스택(PID 4096 B, UDP 6144 B) 사용이 늘지 않게 한다. 검증: `-fstack-usage` 로
  `pid_task`·`udp_task` 프레임을 기준과 비교하고, 디스어셈블리에서 두 태스크가 새로 생긴 함수를 호출하지
  않는지(잔존 `call` 대상 목록이 기준과 같은지) 확인한다. `dynamic`·비한정 표시가 새로 생기면 실패로 본다.
- 람다 `startTimedDescent` 는 같은 캡처 의미의 always_inline 함수로.
- `udp_task` 는 명령 `if/else` 사슬을 명령별 처리 함수로 나누되 비교 순서(접두사 겹침 `rcr`/`rc`,
  `magcal`/`magc`/`mag`, `raw`/`rangeland`)를 그대로 둔다. 루프 밖 지역(`lastSend` 등)은 `udp_task` 의
  지역으로 남긴다. 50 ms 주기 블록의 호출 순서를 바꾸지 않는다.
- 상태 재배치(B-3): A 단계의 `state_*.h`·`requests.h` 의 선언을 쓰는 모듈 곁으로 옮긴다. 전역 이름은
  바꾸지 않는다. 정적 초기화가 있는 전역끼리의 상대 순서를 유지한다(옮기기 전후 초기화자가 있는 전역 목록을
  순서대로 뽑아 비교).

증명:
1. **차분 하네스**(`tools/native_tests/diff_harness/`, Task 4 에서 새로 만든다):
   - 기준은 a41a6f6 스케치 디렉터리 전체(헤더 포함)를 임시 폴더에 고정해 쓰고, 양쪽 include 경로가 섞이지
     않는지 빌드 로그로 확인한다.
   - **두 태스크와 `loop()` 를 시나리오 내내 이어서 돌린다.** 차분 하네스 전용 shim 에 lockstep 스케줄러를
     둔다: 중앙 가상 시계(µs), 태스크별 다음 깨어날 시각, `pid_task`·`udp_task` 는 각자 스레드에서 한 번만
     시작하고 `loop()` 는 controller 가 가상 시간에 맞춰 부른다. 차례 넘김은 `vTaskDelayUntil`·`vTaskDelay`·
     `delay`·`delayMicroseconds` **안에서 기다렸다가 반환**하는 방식이다(예외로 넘기지 않는다 — 스택이 풀려
     지속 실행이 깨진다). `vTaskDelayUntil` 은 `wake` 를 실제처럼 갱신하고 `xTaskGetTickCount` 를 가상
     시계에서 낸다. `delay()` 는 가상 시간을 그만큼 진행시킨다(자기계 초기화의 100 ms 등). 관측은 정해진
     barrier(모든 태스크가 대기 중일 때)에서만 한다. `TaskDelayExit` 는 시나리오 끝에만 던지고 각 worker
     스레드 안에서 잡는다. 명령은 `sendUdp()` 로 태스크를 다시 시작하지 않고 수신 큐에 넣는다.
   - 기록(순서 보존, float 은 비트 그대로 16진수): 모든 `ledcWrite` 호출, 송신한 모든 UDP 패킷(텍스트·
     바이너리 ZIMU/ZCAL·`GAINS`·`FLOWCAL`·`IMUCAL` 응답), 시리얼 출력, NVS 쓰기, tick 마다 상태(모터,
     스로틀 창, `fs_phase`, `safety_lock`, 프로브, `hover_est/valid`, `rl_mode/stage`, hold 상태), 시나리오
     끝 raw 링 대기 샘플·drop 수.
   - 시나리오(각각 도달 단정 포함 — 의도한 분기에 실제로 갔는지): 위치·고도 유지 RC 상실→시간 하강→컷;
     같은 비행에서 RC 복귀·resume→재상실→stop→재무장; resume 누적 제한 거부; 수동 RC 상실; `rangeland 1`
     거리 기반 착지(접촉 컷), 폴백, 바닥 근접 철회; 진입 때 거리 상실; 과도 기울기; IMU 하나 freeze,
     둘 다 freeze(`active_imus == 0` → 1863 `continue`), IMU 불일치; IMU 읽기 오류; yaw authority 전이;
     `gains`·`trim`·`yaw 1`·`raw 0/1`·`th`·`pa ..` 게인·잘못된 명령; `flowcal` 성공·실패; `imucal` 성공·
     움직임 거부(NVS 쓰기 포함); `mag 1` 융합(기존 합성 자기장 주입 `test_mag_yaw_fusion.cpp:298` 방식)과
     magcal; 시간 wrap(micros·millis); `WIFI_LATENCY_DEBUG=1` 빌드로 한 번 더(`loop()` 의 지연 링 drain·
     `[LATSUM]` 출력 포함, 지연 이벤트가 실제로 생성·소비됐다는 도달 단정).
   - IMU 읽기 오류는 shim 에 "실패 시 출력 인자를 건드리지 않는" 주입 모드를 더해 모델링한다(지금 shim 은
     실패 코드와 함께 event 를 덮어써 펌웨어가 전제하는 이전 event 유지를 가린다). 정상 샘플 → 연속 오류 →
     복구에서 이전 raw 값 유지와 freeze 전이를 단정한다.
   - **채널별 민감도 대조**: 기준에 일부러 넣은 변형마다 해당 채널에서 갈라져야 한다 — 하강 델타(모터),
     `GAINS` 응답 서식(UDP), 텔레메트리 필드 하나(UDP), NVS 키(NVS), 시리얼 문구(시리얼), raw 배치 크기(ZIMU).
   - 시나리오마다 기준과 바이트 동일.
2. **공유 변수 접근 목록 비교.** 옮기기 전후로 `volatile` 전역·`portENTER/EXIT_CRITICAL`·`__atomic_*` 사용
   지점을 (함수, 순서) 목록으로 뽑아 비교하고, 차이를 Codex 가 구조 검토한다(순차 SIL 이 못 보는 코어 간
   순서·임계구역 범위).
3. **ESP32 빌드**: 컴파일 성공, `-fstack-usage` 프레임 크기 기준 이하, 플래시·RAM 크기 기록. 단계 함수가
   always_inline 이므로 `pid_task` 디스어셈블리가 기준과 같으면 그것을 추가 증거로 적는다(같지 않아도 실패는
   아니다 — 그때는 차분 하네스가 증명).

### 시험 이식 (Task 2)

- cascade_pwm 을 쓰던 native 3개를 `zetin_fc.ino` 로. 리셋 함수에 시험이 전제하던 값을 명시한다
  (`trim_roll = trim_pitch = 0`, 시험이 기대하는 Ki, `resetFlowHoldForHarness()`). 바뀐 기대값마다 이유를
  주석으로 단다. `publishAllocationTelemetry` 호출에 `requestedUs` 인자.
- `test_sil_attitude.cpp:1582` 의 75필드 접미사 검사는 "필드 38–74 의 위치·내용"을 그대로 검사하도록
  바꾸고, 75–96 기본값 검사를 따로 더한다(기존 구간 검증을 잃지 않는다). 필드 수 단정은 97.
- 이식한 뮤테이션 시험(`test_native_control_math.py` 의 `FS_DESCENT_DELTA_US` 1종, `test_mag_yaw_fusion.py` 의
  `BMM350_SIL_MUTATION` 변형들)은 이식 뒤에도
  변형을 넣으면 실제 단정 실패를 일으키는지 한 번씩 확인한다.
- 헤더 경로 `-I` 를 `zetin_fc/logic` 으로. `test_flow_hold_sketch.py`(두 스케치 헤더 동일 강제)는 대상이
  사라지므로 삭제하고, 그 안의 다른 단정은 옮긴다.
- 텔레메트리 스키마의 75필드 경로는 옛 로그 읽기용으로 남긴다.

## 태스크

각 태스크 끝: `python -m unittest discover -s tools -p 'test_*.py'`(skip 0건 확인),
`tools/check_repo_layout.py`, `git diff --check`, 로컬 `arduino-cli` 로 `firmware/flight/*/`·
`firmware/diagnostics/*/` 컴파일. 커밋·push 는 사용자 확인.

1. **archive 와 이름 변경 + 경로가 걸린 검사.** `git mv` 로 `flight/dual_imu_cascade_pwm`·
   `flight/dual_imu_flix_quat_pwm` → `archive/legacy_flight/`, `flight/dual_imu_cascade_flow_hold` →
   `flight/zetin_fc`(`.ino` 이름도). 같은 커밋에서 검사가 걸린 경로를 고친다: 시험·CI·`check_repo_layout.py`·
   `docs/migration_map.md`(행 수 유지, 9행 새 경로·수명주기 갱신)·`check_repo_layout.py` 의
   `maintained_markdown_files` 전체의 링크·`probe_response.py`. cascade_pwm 을 쓰던 native 시험은 이 태스크에서는
   **archive 경로를 계속 가리키게** 경로만 고친다(zetin_fc 로 옮기는 것은 인자·단정 변경과 함께 Task 2).
   펌웨어 파일 내용은 바꾸지 않는다(`git diff -M` 100 %, `.ino` 머리 주석의 스케치 이름 줄은 따로 커밋).
2. **시험 이식 + 공용 도우미.** 위 "시험 이식"과 `firmware_source.py`. 순수 로직 헤더를 `logic/` 로.
   끝나면 cascade_pwm 이 하던 native 시험 전체가 zetin_fc 에서 돈다.
3. **A 단계 분할.** ELF 동일(디버그 0/1)과 호스트 전처리 동일을 커밋 메시지와 이 계획 추가 절에 남긴다.
4. **차분 하네스.** lockstep 스케줄러, 기록 채널, 시나리오, 채널별 민감도 대조. 기준 대 A 단계 결과로 먼저
   돌려 동일함을 확인한다.
5. **B 단계 1–2(함수 분해).** 차분 동일 + 공유 변수 접근 목록 + 스택 + 전체 시험 + Codex 리뷰.
6. **B 단계 3(상태 재배치).** 차분 동일 + 정적 초기화 순서 비교 + 전체 시험 + ESP32 빌드 + Codex 리뷰.
7. **문서.** README·`firmware/README.md`·카탈로그·`current_status`·`project_overview`·`workflow`·프로토콜의
   "주력" 설명, `zetin_fc/README.md` 모듈 지도. 날짜 박힌 기록 문서는 고치지 않는다.
8. **노션.** `docs/notion_doc_convention.md` 에 따라 펌웨어 설명형 페이지 갱신(사용자 확인 뒤).

## 범위 밖

- 제어 로직·상수·텔레메트리 형식 변경, 새 기능.
- `.cpp` 다중 번역 단위화, 클래스 기반 재설계.
- 진단 스케치·카메라 스케치·지상국 구조 변경(경로·상수 탐색 수정만).
- 순차 SIL 로 재현할 수 없는 코어 간 타이밍·실행 지연(구조 검토와 실기로만 확인 가능 — 실기 비행 전
  벤치에서 텔레메트리 loop Hz·지연 카운터를 기준과 비교하는 것을 권고로 남긴다).

## 자체 검토

1. **직접 재계산(Claude, 2026-11-13).** 구간 경계 38곳을 원본에서 다시 뽑았다. 결함 2건 수정:
   `#if WIFI_LATENCY_DEBUG` 가 189 에서 열려 230 `#endif` 로 닫히므로 `wire_types.h` 는 34–230(처음 34–229 로
   적어 블록을 갈랐다). `publishAllocationTelemetry` 인자 수를 10 vs 9 로 잘못 적음 → 9 vs 8. 나머지 경계는
   빈 줄·구획 머리 주석에 정확히 걸린다. `pid_task` 1665–2600(936줄), 조건부 블록 6개 모두 단일 구간 안.
   Arduino 실험으로 하위 폴더 include 와 "같은 소스면 같은 ELF 코드"를 확인(위 현재 구조 마지막 항목).
2. **Codex 적대적 검토(P1 1·P2 9·P3 1).** 단계 순서가 실제 실행 순서와 충돌(P1) → 실제 순서대로 단계 경계 재정의.
   호스트 토큰 동일이 ESP32 동일을 함의하지 않음(P2) → ESP32 ELF 동일을 주 증명으로, 디버그 0/1 두 번.
   `g++ -E -P file.ino` 는 출력 0바이트로 통과(P2, Codex 재현) → `-x c++` + 비어 있지 않음 + 문자 단위 비교.
   `PidTick` 만으로는 수명·조기 탈출 계약 부족(P2) → `PidLoopState`/`PidTick` 분리, Skip 반환. volatile 읽기
   시점·`constrain` 매크로·임계구역(P2) → 공유 전역 복사 금지 규칙, 접근 목록 비교, 구조 검토. 차분 관측 부족
   (P2: UDP shim 마지막 패킷만, `ledcWrite`·NVS·응답 패킷·raw 링 미관측, float 자릿수) → 기록 채널 확장·비트
   직렬화·채널별 민감도 대조. 태스크 재시작(P2) → lockstep 스케줄러로 지속 실행. 시험 이식 범위(P2: 접미사,
   뮤테이션 경로) → 시험 이식 절. `probe_response.py`(P2) → 공용 도우미. 기준은 디렉터리 전체 고정(P2).
   stale 검사 `.h`·migration map 시점·skip 0건(P3) → Task 1·도우미·태스크 공통 검사에 반영.
   Codex 는 하위 폴더 include(④)와 단일 번역 단위(①)에 동의.
3. **agy(Gemini 4 Argon) 독립 검토(①②를 보지 않음, "적히지 않은 전제").** 이식 시 기본값 차이(trim·Ki)와
   flow 상태 오염, 접미사 단정 → 시험 이식 절. 텍스트 파싱 3곳(특히 `probe_response.py`) → 공용 도우미.
   migration map 이 Task 1 에서 깨짐 → Task 1. `pid_task` 단계 교차와 `continue` 3개(1863·2476·2589) →
   단계 경계·Skip. 루프 밖/안 지역 수명, 스택 4096 B → `PidLoopState`/`PidTick`, always_inline·
   `-fstack-usage`. Arduino 원형 생성 → ELF 비교. `sendUdp` 가 `udp_task` 지역을 매번 초기화 → lockstep.
   시나리오에 거리 기반 진입·IMU 고장 경로 → 시나리오 목록. **정정:** agy 가 든 뮤테이션 대상
   `FS_ENTER_CONFIRM_MS` 는 저장소에 없다(Claude 가 확인 없이 옮겨 적었고 Codex 재검토가 잡음 → 실제
   `FS_DESCENT_DELTA_US` 1종으로 고침). 나머지는 소스로 확인했다.
2-1. **Codex 재검토(이전 P1 해결, 새 P1 없음, P2 5·P3 2).** ELF 비교 제외 범위가 넓고 배치 검사 없음(P2) →
   `SHF_ALLOC` 섹션 배치·`.bss`·entry·초기화 배열, 앱 설명자는 시각·날짜·SHA 필드만 마스킹, `.bin` 미사용.
   lockstep 가상 시간·예외 계약(P2: shim 의 `wake`·tick·`delay()` 무동작) → 중앙 가상 시계, delay 안 대기,
   barrier 관측, 예외는 종료에만·worker 안에서. debug=1 에 `loop()` 미실행(P2) → `loop()` 도 가상 시간으로
   실행·지연 이벤트 도달 단정. ICM shim 오류 모델(P2) → 출력 보존 주입 모드. 태스크 프레임만 비교(P2) →
   UDP 처리 함수도 always_inline, 잔존 호출 대상 비교, `dynamic` 금지. Task 1 중간 상태·링크 범위(P3) →
   Task 1 문구. 존재하지 않는 뮤테이션(P3) → 정정.
2-2. **Codex 수렴(2026-11-13):** "계획 수준에서 남아 있거나 새로 발견한 P1/P2 없음". 구현·하네스·ELF 비교기의
   정확성은 구현 단계의 검토로 확인한다.
