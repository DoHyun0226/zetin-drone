# 지상국 스크립트 정리 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 실행 스크립트를 조종·영상·로그 분석 세 진입점으로 줄이고, 조종 화면을 고정 대시보드로 바꾸되 기존 기능과 안전 흐름은 그대로 둔다.

**Architecture:** 공유 모듈과 각 도구의 본체를 `scripts/ground/` 패키지로 옮기고, `scripts/drone_*.py` 세 파일은 인자만 나눠 본체의 `main(argv)`를 부르는 얇은 진입점으로 둔다. 조종 모듈은 import 시점 전역 구조(소켓·로그·상태)를 유지하고, 맨 아래 실행부만 `main()`으로 감싸며, 모든 출력을 `ui` 객체로 보낸다. 기본 UI는 지금과 같은 출력을 내는 `ConsoleUI`, 터미널에서는 `prompt_toolkit` 대시보드다.

**Tech Stack:** Python 3.12(CI)·3.11(로컬 conda base), unittest, pygame, prompt_toolkit ≥ 3.0(로컬 3.0.52 확인), matplotlib, OpenCV.

**Spec:** [docs/superpowers/specs/2026-11-02-ground-station-cleanup-design.md](../specs/2026-11-02-ground-station-cleanup-design.md)

## Global Constraints

- 펌웨어·UDP 명령 문법·텔레메트리 필드·CSV 열 순서·raw `.bin` 형식을 바꾸지 않는다.
- 기존 시험은 옮기거나 경로를 고칠 수 있지만, 검증하던 행동을 빼거나 약하게 만들지 않는다. 시험 수가 줄면 이유를 원장에 적는다.
- 안전 흐름(시동 확인·재시도·거부, `resume` 확인, 종료 시 조건부 disarm, 텔레메트리 끊김 → `stop_streaming_only`, RC 타임아웃·치명 고장 처리)의 코드 경로를 그대로 둔다. 이 계획이 일부러 바꾸는 안전 동작은 아래 세 가지뿐이고, 각각 시험으로 고정한다.
  1. 수신·조종 스레드가 예외로 죽으면 텔레메트리 끊김과 같이 `stop_streaming_only`로 RC를 멈추고 드론 자동착륙에 맡긴다(지금은 RC가 계속 나갈 수 있다). 이 오류는 래치되어, 지상국을 다시 시작하기 전에는 시동·복구·RC 송신을 막는다.
  2. Ctrl+D(EOF)도 Ctrl+C와 같이 무장·시동 중이면 disarm한다(지금은 EOF에서 disarm이 없다).
  3. 같은 포트(4210)를 쓰는 지상국이 이미 떠 있으면 시작을 거부하고 이유를 출력한다.
- 사건 메시지 문구(`[ARM]`·`[RESUME]`·`[FAULT]`·`[WARN]`·`[TRIM]`·`[TH]`·`[STDIN]`·`[LOG]`·`[OK]`·`[ERR]`·`[RAW]`·`[GAINS]`·`[FLOWCAL]`·`[IMUCAL]`)는 지금 문자열을 유지한다. `ConsoleUI`의 출력(주기 줄 `[STATUS]`·`[DIAG]`·`[AUTO-LAND]`·`[AXIS]` 포함)은 지금과 글자 하나까지 같다.
- 화면 그리기는 최대 10 Hz(`min_redraw_interval=0.1`), 수신이 없어도 10 Hz로 갱신(`refresh_interval=0.1`). 텔레메트리·RC 스레드는 그리기를 기다리지 않는다.
- UI 호출은 안전 경로를 막지 않는다. UI 메서드는 예외를 밖으로 내지 않고, 안전 함수는 상태 변경·송신 뒤에 UI를 부른다.
- 로그·펌웨어 경로는 파일 위치가 아니라 `ground/paths.py`의 `REPO_ROOT`·`LOG_DIR` 기준이다(로그는 계속 `<repo>/logs`).
- 옛 스크립트 이름의 호환 껍데기를 두지 않는다. 파일을 옮기는 커밋에서 그 파일을 가리키는 레포 문서 링크·`docs/migration_map.md`·CI 단계를 함께 고친다(각 커밋에서 `tools/check_repo_layout.py` 통과).
- CI python 작업은 skip을 실패로 본다. `prompt_toolkit>=3.0`은 `requirements.txt`에 넣는다. 설치되지 않은 환경에서는 대시보드 대신 `ConsoleUI`로 뜬다(지연 import).
- 조종은 패드로만 한다. `--keyboard`는 `start`·`resume`을 보내지 않고, 종료 시 자동 `stop`도 보내지 않는다(설계 3절 정정 — 자동착륙 하강 중 `stop`이 나가면 모터가 꺼진다). `stop`은 입력줄에서 직접 보낼 수 있다.
- `--listen`은 `connect` 외에는 아무것도 보내지 않는다(트림 동기 재전송·`raw 0`·종료 시 disarm 포함).
- RSSI는 표시하지 않는다.

## Review Focus

1. **사건 폭주 중 수신 지연** — DESCENDING 매 패킷 갱신과 경고가 몰려도 `ui.event`·`ui.update`가 막히지 않고, 그리기는 10 Hz를 넘지 않는다. Task 6 `test_redraw_rate_is_capped`, `test_ui_calls_do_not_block_while_app_runs`.
2. **비정상 종료** — 워커 예외·Ctrl+C·Ctrl+D 뒤 안전 경로가 돌고, 터미널이 복구되며, 로그 저장·raw 변환 사건이 화면 뒤에 갇히지 않는다. Task 4 `test_worker_exception_stops_rc_streaming`, `test_worker_fault_latch_blocks_resume_and_rc`, `test_ui_failure_does_not_skip_safety`, `test_eof_disarms_when_armed`, Task 6 `test_events_after_run_go_to_stdout`.
3. **한글 폭** — 패널 줄이 터미널 폭을 넘지 않는다. Task 5 `test_lines_fit_width_with_korean_labels`.
4. **터미널이 아닌 실행** — 파이프·CI·`--plain`·prompt_toolkit 미설치에서는 지금과 같은 줄 출력. Task 6 `test_non_tty_uses_console_ui`, `test_missing_prompt_toolkit_falls_back`.
5. **필드가 다른 펌웨어** — 75필드(main 비행 펌웨어)와 91필드(flow-hold)에서 패널이 각각 맞게 보인다. Task 5 `test_flow_panel_only_with_flow_fields`.

---

### Task 0: 시작 조건 확인

**Files:** `docs/superpowers/specs/2026-11-02-ground-station-cleanup-design.md`

- [ ] **Step 1:** `feat/flow-position-hold`가 main에 머지됐는지 확인한다(`git merge-base --is-ancestor <flow-hold tip> origin/main`). 안 됐으면 멈추고 사용자에게 알린다.
- [ ] **Step 2:** `feat/ground-station-cleanup`을 최신 main 위로 다시 올린다(설계·계획 커밋만 있다).
- [ ] **Step 3: 기준선.** CI의 Unit tests 단계와 같은 코드로 전체 시험을 돌려 통과 수를 원장에 적는다. Expected: 실패 0, skip 0. `python tools/check_repo_layout.py` 통과.
- [ ] **Step 4:** flow-hold로 생긴 기능(`[FLOWCAL]`·`[IMUCAL]` 사건, `flowcal`·`poshold`·`althold`·`hg`·`imucal` 명령, `PosHold`/`AltHold`/`Height_M` 상태)을 설계 7절 표에 추가하고, 이 계획의 정정(키보드 모드 자동 `stop` 없음, `--listen` 송신 범위, 세 가지 안전 동작 변경)을 설계에 반영해 커밋한다.

### Task 1: 공유 모듈과 경로 기준을 `scripts/ground/`로 (동작 변화 없음)

**Files:**
- Create: `scripts/ground/__init__.py`, `scripts/ground/paths.py`
- Move: `telemetry_schema.py`, `failsafe_telemetry.py`, `camera_stream.py` → `scripts/ground/`
- Modify: 이 셋을 import하는 `scripts/*.py`·`tools/test_*.py`(`tools/test_failsafe_telemetry.py`의 `from scripts import failsafe_telemetry` 포함), `scripts/bench_*.py`의 import, `.github/workflows/ci.yml`(`python -m py_compile scripts/*.py` → `python -m compileall -q scripts`), `tools/check_repo_layout.py`(`stale_scan_files`가 `scripts/**/*.py`를 보게, `check_migration_map`이 통합 때문에 같은 새 경로를 여러 옛 경로가 가리키는 것을 허용하고 행 수 기대값을 갱신), `tools/test_repo_layout.py`, 옮긴 파일을 가리키는 문서 링크(`docs/udp_protocol.md`, `logs/README.md` 등 `git grep`으로 전부), `docs/migration_map.md`의 해당 행
- Test: `tools/test_ground_layout.py`

**Interfaces:**
- Produces: `ground.paths.REPO_ROOT`(저장소 루트), `ground.paths.LOG_DIR`(`REPO_ROOT / "logs"`). `from ground.telemetry_schema import …` 등. `scripts/`가 `sys.path`에 있다는 기존 전제를 유지한다.

- [ ] **Step 1: 실패하는 시험**

```python
def test_shared_modules_live_in_ground_package(self):
    for name in ("telemetry_schema", "failsafe_telemetry", "camera_stream", "paths"):
        self.assertTrue((SCRIPTS / "ground" / f"{name}.py").exists(), name)
    for name in ("telemetry_schema", "failsafe_telemetry", "camera_stream"):
        self.assertFalse((SCRIPTS / f"{name}.py").exists(), name)

def test_paths_point_at_repo_logs(self):
    from ground import paths
    self.assertEqual(paths.REPO_ROOT, REPO_ROOT)
    self.assertEqual(paths.LOG_DIR, REPO_ROOT / "logs")

def test_migration_map_allows_merged_destinations(self):
    # 두 옛 경로가 같은 새 경로를 가리켜도 오류가 아니다. 옛 경로 중복·새 경로 부재는 여전히 오류.
    ...
```

- [ ] **Step 2:** `python -m unittest tools.test_ground_layout -v` → FAIL
- [ ] **Step 3:** `git mv`, import·링크·map·CI·검사기 수정.
- [ ] **Step 4:** 새 시험 PASS, 전체 시험이 기준선 수 이상으로 통과, `check_repo_layout.py` 통과.
- [ ] **Step 5: 커밋** — `refactor(ground): move shared telemetry and camera modules into scripts/ground`

### Task 2: `drone_logs.py` (로그 분석 진입점)

**Files:**
- Move: `analyze_flight_log.py` → `ground/flight_log.py`, `decode_imu_raw.py` → `ground/imu_raw.py`, `analyze_probe_response.py` → `ground/probe_response.py`, `magcal_fit.py` → `ground/magcal.py`
- Create: `scripts/drone_logs.py`
- Modify: `scripts/control_dualsense.py`(`decode_file` import), 네 분석기 시험(subprocess로 경로를 실행하던 시험은 `drone_logs.py <하위 명령>`을 실행), `tools/test_failsafe_telemetry.py`의 `ANALYZE_SCRIPT`, 각 분석기의 `SCRIPT_DIR`·`LOG_DIR`·`REPO_ROOT` 계산을 `ground.paths`로, 문서 링크·migration map
- Test: `tools/test_drone_logs.py`

**Interfaces:**
- Consumes: Task 1의 `ground.*`, `ground.paths`
- Produces: `ground.flight_log.main(argv=None) -> int`(지금 모듈 최상위 코드와 `sys.argv` 직접 읽기를 `argv`로 감쌈, `--no-plot`·위치 인자 의미 유지), `ground.imu_raw.main`·`ground.probe_response.main`·`ground.magcal.main`(기존). `drone_logs.py <flight|imuraw|probe|magcal> [인자…]` — 나머지 인자를 그대로 넘기고 반환값으로 종료.
- 경로를 생략한 `flight`는 `LOG_DIR`의 **`flight_log_*.csv` 중** 최신 파일을 고른다(지금은 `*.csv` 전체라 카메라 색인 CSV를 고를 수 있다).

- [ ] **Step 1: 실패하는 시험**

```python
def test_help_lists_four_subcommands(self):
    out = run([sys.executable, DRONE_LOGS, "--help"])
    for sub in ("flight", "imuraw", "probe", "magcal"):
        self.assertIn(sub, out.stdout)

def test_imuraw_matches_decoder(self):
    # tools/test_decode_imu_raw.py 의 _zimu()/_zcal() 데이터그램 생성기로 합성 캡처를 만든다
    bin_path = tmp / "cap.bin"; bin_path.write_bytes(_zcal(...) + _zimu(0, 0, samples))
    run([sys.executable, DRONE_LOGS, "imuraw", bin_path, tmp / "a.csv"], check=True)
    imu_raw.decode_file(bin_path, tmp / "b.csv")
    self.assertEqual((tmp / "a.csv").read_bytes(), (tmp / "b.csv").read_bytes())

def test_flight_no_plot_runs_on_fixture(self):
    # tools/test_analyze_flight_log.py 의 _run_analyzer 와 같은 방식으로 CSV 를 쓰고 새 진입점으로 실행
    out = run([sys.executable, DRONE_LOGS, "flight", csv_path, "--no-plot"], check=True)
    self.assertIn("Allocator scale: legacy/unknown", out.stdout)

def test_flight_default_picks_latest_flight_log_not_camera_csv(self):
    # LOG_DIR 를 임시 폴더로 바꾸고 flight_log_A.csv(오래됨) + camera_B.csv(최신) → A 선택
    ...

def test_probe_analyzer_finds_firmware_constants_after_move(self):
    self.assertTrue(probe_response.FIRMWARE_PATH.exists())

def test_unknown_subcommand_fails(self):
    self.assertNotEqual(run([sys.executable, DRONE_LOGS, "nope"]).returncode, 0)
```

- [ ] **Step 2:** FAIL → **Step 3:** 구현 → **Step 4:** PASS, 기준선 이상, `check_repo_layout.py` 통과
- [ ] **Step 5: 커밋** — `refactor(ground): add drone_logs.py and move the log analyzers under it`

### Task 3: `drone_video.py` (영상 진입점)

**Files:**
- Move: `record_camera.py` → `ground/camera_record.py`, `camera_to_mp4.py` → `ground/camera_mp4.py`(둘 다 `LOG_DIR`을 `ground.paths`에서)
- Create: `scripts/drone_video.py`
- Modify: `tools/test_record_camera.py`, `tools/test_camera_to_mp4.py`, `tools/test_camera_sketch.py`(`CAMERA_SCRIPTS`), 문서 링크·migration map
- Test: `tools/test_drone_video.py`

**Interfaces:**
- Produces: `drone_video.main(argv) -> int`. `drone_video.py [record] [--url --out-dir --no-preview --timeout]`(하위 명령 생략 시 `record`), `drone_video.py mp4 <csv> [--no-mp4]`.

- [ ] **Step 1: 실패하는 시험**

```python
def test_default_subcommand_is_record(self):
    with mock.patch.object(camera_record, "main", return_value=0) as m:
        self.assertEqual(drone_video.main(["--no-preview"]), 0)
    m.assert_called_once_with(["--no-preview"])

def test_mp4_no_mp4_writes_aligned_csv(self):
    csv_path = write_fixture_recording(tmp)   # tools/test_camera_to_mp4.py 의 픽스처 작성 방식 재사용
    self.assertEqual(drone_video.main(["mp4", str(csv_path), "--no-mp4"]), 0)
    self.assertTrue(aligned_csv_for(csv_path).exists())

def test_record_default_out_dir_is_repo_logs(self):
    self.assertEqual(camera_record.LOG_DIR, REPO_ROOT / "logs")
```

- [ ] **Step 2:** FAIL → **Step 3:** 구현 → **Step 4:** PASS, 기준선 이상
- [ ] **Step 5: 커밋** — `refactor(ground): add drone_video.py for camera recording and mp4 export`

### Task 4: 조종 모듈 이동, 출력 통로, 안전 보강

**Files:**
- Move: `scripts/control_dualsense.py` → `scripts/ground/control_station.py`(`LOG_DIR`은 `ground.paths`)
- Create: `scripts/ground/station_ui.py`(`ConsoleUI`)
- Modify: `tools/test_control_dualsense.py`(`CONTROL_PATH`; `_load_module`은 같은 mock 범위 안에서 `exec_module` 뒤 `module.main()`을 호출), 문서 링크·migration map
- Test: `tools/test_station_ui.py`, `tools/test_control_dualsense.py`에 추가

**Interfaces:**
- Produces:
  - `ConsoleUI`: `event(text)`(지금 `print`와 같은 줄), `periodic(text)`(지금 주기 줄과 같은 print — `[STATUS]` 1 Hz, `[DIAG]` ~5 Hz, `[AUTO-LAND]`, `[AXIS]` 0.5 s, 아날로그 스로틀 중 `[TH]`), `transition(text)`(아무것도 안 함 — 대시보드 전용 상태 전이 사건), `update(snap)`(아무것도 안 함), `alert(text)`(= `event`), `run(input_handler, on_interrupt, on_eof)`(지금의 `input()` 루프), `after_run()`(아무것도 안 함).
  - `control_station.ui`(기본 `ConsoleUI()`), `control_station.main(argv=None, ui=None)` — 맨 아래 실행부(스레드 시작·입력 루프·`finally` 정리). 소켓·로그 파일·전역 상태 초기화는 import 시점 그대로.
  - `control_station.snapshot() -> dict` — `arm_state`, `is_armed`, `is_streaming`, `current_throttle`, `trim_roll/pitch`, `telem_total_pkts`, `telem_dropped_pkts`, `bad_packet_count`, `last_telem_time`, `log_path`, raw 상태, 마지막 샘플, `mode`, `worker_fault`.
- 모든 `print(` 70곳을 분류한다: 주기 줄은 `ui.periodic`, 나머지는 `ui.event`. 분류 목록(줄 번호·태그·주기)을 원장에 남긴다.
- `Failsafe_Phase`·`Failsafe_Probe_State`·`Yaw_Authority_State`·`PosHold_State`·`AltHold_State`가 바뀌는 패킷에서 `ui.transition("[STATE] <필드> <이전> → <다음>")`.
- **워커 예외:** `telemetry_thread`·`controller_thread`를 감싸 예상 못 한 예외가 나면 **먼저** 안전 상태를 바꾼다 — `worker_fault`(래치, 재시작 전에는 풀리지 않음)를 기록하고, 진행 중인 시동·복구 시도를 취소하고, `is_streaming`이면 `stop_streaming_only("<스레드> 스레드 오류")`. 그 **다음** `ui.alert("[ERR] <스레드> 스레드 오류: <예외> - rc 중단, 드론 자동착륙에 맡김. 지상국을 다시 시작하세요")`. 프로그램은 계속 떠 있어 운영자가 상황을 본다.
- **워커 오류 래치:** `worker_fault`가 있으면 `arm()`·`request_resume()`은 거부 사건을 띄우고 아무것도 보내지 않으며, `advance_arm_attempt`는 `Armed=1`을 받아도 무장·스트리밍으로 넘어가지 않고 `start` 재시도도 하지 않는다. RC 송신(`rc`·`rcr`)도 래치 중에는 나가지 않는다.
- **UI는 안전 경로를 막지 않는다:** `ConsoleUI`·`DashboardUI`의 모든 메서드는 예외를 밖으로 내지 않는다(내부에서 삼키고, 가능하면 stderr에 한 줄). 안전 함수(`disarm`·`stop_streaming_only`·`cancel_resume_attempt` 등)는 상태 변경과 송신을 끝낸 뒤에 `ui`를 부른다.
- **EOF:** `run`이 EOF를 받으면 `on_eof()` — KeyboardInterrupt와 같은 조건부 disarm(`is_armed or arm_state == "arming"`).
- **포트 점유:** `sock.bind`가 `OSError`면 `[ERR] 포트 4210을 다른 지상국이 쓰고 있음 - 먼저 종료하세요`를 출력하고 종료 코드 1(로그 파일을 만들기 전에).

- [ ] **Step 1: 실패하는 시험**

```python
def test_control_station_has_no_bare_print(self):
    src = (SCRIPTS / "ground" / "control_station.py").read_text(encoding="utf-8")
    self.assertNotRegex(src, r"(?m)^\s*print\(")

def test_console_ui_event_prints_same_text(self):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        ConsoleUI().event("\n>>> [SYSTEM] DISARMED (수동)")
    self.assertEqual(out.getvalue(), "\n>>> [SYSTEM] DISARMED (수동)\n")

def test_worker_exception_stops_rc_streaming(self):
    # 무장·스트리밍 상태에서 csv_writer.writerow 가 OSError → 수신 스레드 종료
    self.module.is_armed = True; self.module.is_streaming = True
    self.module.csv_writer = types.SimpleNamespace(writerow=mock.Mock(side_effect=OSError("disk")))
    out = self._run_telemetry_wrapped([sample])
    self.assertFalse(self.module.is_streaming)
    self.assertIn("수신 스레드 오류", out)
    self.assertEqual(self.module.snapshot()["worker_fault"], "telemetry")

def test_controller_exception_stops_rc_streaming(self): ...

def test_worker_fault_latch_blocks_resume_and_rc(self):
    # 수신 워커 오류 → stop_streaming_only → request_resume("패드") 는 False, 이후 조종 tick 에서 rc/rcr 미송신
    ...

def test_worker_fault_during_arming_blocks_arm_confirmation(self):
    # arm_state == "arming" 에서 조종 워커 오류 → 다음 Armed=1 샘플에도 is_armed/is_streaming False, start 재전송 없음
    ...

def test_ui_failure_does_not_skip_safety(self):
    # ui.alert/ui.event 가 BrokenPipeError 를 던지는 UI 로 워커 오류 재생 → is_streaming False, worker_fault 기록
    ...

def test_ui_methods_never_raise(self):
    # 닫힌 stdout 에서 ConsoleUI 의 모든 메서드 호출이 예외 없이 반환
    ...

def test_eof_disarms_when_armed(self):
    m, sock = self._load_module_armed(input_side_effect=EOFError)
    self.assertTrue(any(d == b"stop" for d, _ in sock.sent))

def test_bind_failure_exits_before_creating_log(self):
    # socket.bind 가 OSError → SystemExit(1), Path.open 호출 없음, 메시지에 "포트 4210"
    ...

def test_state_transitions_reach_ui(self):
    # Failsafe_Phase 0 → 1 → 1 → 0 샘플 → transition 2회
    ...
```

- [ ] **Step 2:** FAIL → **Step 3:** 구현 → **Step 4:** 조종 시험 108개 전부 PASS(출력 비교 포함), 전체 기준선 이상
- [ ] **Step 5: 커밋** — `refactor(ground): route control station output through a UI object and stop RC when a worker dies`

### Task 5: 대시보드 화면 모델 (순수 함수)

**Files:**
- Create: `scripts/ground/dashboard_view.py`
- Test: `tools/test_dashboard_view.py`

**Interfaces:**
- Consumes: Task 4의 `snapshot()`, `ground.telemetry_schema`·`ground.failsafe_telemetry`의 이름 표와 `active_fault_names`
- Produces: `render(snap: dict, width: int, height: int, now: float) -> list[list[tuple[str, str]]]` — 줄마다 `(style, text)` 조각 목록. 스타일: `class:armed`, `class:disarmed`, `class:alert`, `class:warn`, `class:dim`, `class:ok`, `class:value`. 위 상태줄에는 시동 상태·수신율·드롭률·불량 패킷 수·비행 시간·로그 이름·경고(RC 타임아웃, 치명 고장, 시동 불일치, 텔레메트리 끊김, 워커 오류).

- [ ] **Step 1: 실패하는 시험** (샘플은 `tools/test_telemetry_schema.py`의 `packet(field_count)`로 75·91필드 패킷 문자열을 만든 뒤 `parse_telemetry_packet`으로 사전을 얻는다. `sample75()`·`sample91(**overrides)`는 그 두 줄을 감싼 시험 내부 도우미다)

```python
def test_all_panels_render_for_flight_firmware_sample(self):
    text = flatten(render(snap(sample75()), 100, 30, now))
    for label in ("자세", "목표", "고도·추력", "모터", "믹서", "트림"):
        self.assertIn(label, text)
    self.assertIn("M1 FL", text)

def test_missing_fields_show_dash(self):
    s = sample75(); s["Hover_Est"] = None
    self.assertRegex(flatten(render(snap(s), 100, 30, now)), r"Hover_Est\s+-")

def test_flow_panel_only_with_flow_fields(self):
    self.assertNotIn("PosHold", flatten(render(snap(sample75()), 100, 30, now)))
    t = flatten(render(snap(sample91(PosHold_State=1, Height_M=0.42)), 100, 30, now))
    self.assertIn("PosHold ACTIVE", t); self.assertIn("0.42", t)

def test_probe_fields_hidden_when_absent(self):
    s = sample75(); s["Failsafe_Probe_State"] = None
    self.assertNotIn("Probe", flatten(render(snap(s), 100, 30, now)))

def test_descending_failsafe_is_alert_style(self):
    lines = render(snap(sample75(Failsafe_Phase=1)), 100, 30, now)
    self.assertTrue(any(st == "class:alert" and "DESCENDING" in tx for ln in lines for st, tx in ln))

def test_stale_link_dims_values_and_shows_age(self):
    lines = render(snap(sample75(), last_telem_time=now - 3.0), 100, 30, now)
    self.assertIn("수신 없음 3.0s", flatten(lines[:1]))

def test_top_bar_alerts(self):
    top = flatten(render(snap(sample75(Fault_RC=1, Fault_Critical=1), worker_fault="telemetry"), 100, 30, now)[:1])
    for word in ("RC", "치명", "스레드"):
        self.assertIn(word, top)

def test_bad_packet_count_in_top_bar(self): ...

def test_small_terminal_collapses_motor_and_mixer_rows(self):
    t = flatten(render(snap(sample75()), 79, 23, now))
    self.assertNotIn("M1 FL", t); self.assertIn("자세", t)

def test_lines_fit_width_with_korean_labels(self):
    for w in (60, 80, 120):
        for line in render(snap(sample91()), w, 30, now):
            self.assertLessEqual(get_cwidth("".join(t for _, t in line)), w)
```

- [ ] **Step 2:** FAIL → **Step 3:** 구현. 값 서식은 `control_station`의 `_format_*`와 같은 자릿수(필요하면 그 함수들을 `dashboard_view`로 옮기고 `control_station`이 import). 고도 칸은 `Height_M`(유효할 때)과 지금의 `_format_agl`(3901-L0X `Range_MM`·`Range_Quality`).
- [ ] **Step 4:** PASS, 기준선 이상
- [ ] **Step 5: 커밋** — `feat(ground): add the dashboard view model`

### Task 6: `prompt_toolkit` 대시보드 UI

**Files:**
- Modify: `scripts/ground/station_ui.py`(`DashboardUI`, `make_ui(args)`), `scripts/ground/control_station.py`(`--plain`, `main`에서 `make_ui`), `requirements.txt`
- Test: `tools/test_station_ui.py`

**Interfaces:**
- `DashboardUI`: `ConsoleUI`와 같은 메서드. `event`·`transition`·`alert`는 시각을 붙여 사건 deque(최근 500개)에 넣고 그리기 요청, `periodic`은 무시(패널이 대신), `update`는 잠금 안에서 스냅숏 교체 후 그리기 요청. 그리기 요청은 `Application(full_screen=True, min_redraw_interval=0.1, refresh_interval=0.1)`이 묶는다(`min_redraw_interval`은 요청 폭주를 10 Hz로 묶고, `refresh_interval`은 수신이 없어도 경과 시간 등이 갱신되게 한다). `run(input_handler, on_interrupt, on_eof)`: 레이아웃 위 상태줄 / 패널 / 사건 / 입력줄(`TextArea`; Enter → `input_handler(text)`, `?` → 시작 도움말을 사건으로, Esc → 입력줄 비움). Ctrl+C → `on_interrupt()`, Ctrl+D → `on_eof()` 뒤 앱 종료. 입력 처리 중 예외 → 사건 기록 후 계속.
- `after_run()`: 앱이 끝난 뒤 쌓인 사건 목록을 일반 출력으로 내보내고, 그 뒤의 `event`는 바로 `print`(로그 저장·raw 변환 사건이 일반 출력으로 보이게). `main()`은 `ui.run(...)` → `ui.after_run()` → `shutdown_workers_and_close_log` 순서.
- `make_ui(args)`: `args.plain`, `not sys.stdout.isatty()`, `prompt_toolkit` import 실패 중 하나면 `ConsoleUI`(마지막 경우 `[WARN] prompt_toolkit 없음 - 줄 출력으로 실행` 사건).

- [ ] **Step 1: 실패하는 시험** (`create_pipe_input()`과 `DummyOutput`으로 실제 터미널 없이)

```python
def test_enter_sends_command_to_handler(self): ...       # "pr 0.9\r" → handler("pr 0.9")
def test_question_mark_shows_help_in_events(self): ...   # "PID:" 도움말 줄이 사건에
def test_escape_clears_input_line(self): ...
def test_ctrl_c_calls_on_interrupt_and_returns(self): ...
def test_ctrl_d_calls_on_eof_and_returns(self): ...
def test_handler_exception_is_logged_and_app_keeps_running(self): ...
def test_event_from_other_thread_appears(self): ...
def test_ui_calls_do_not_block_while_app_runs(self):
    t0 = time.perf_counter()
    for i in range(2000): ui.event(f"[WARN] {i}"); ui.update(fake_snapshot)
    self.assertLess(time.perf_counter() - t0, 0.5)
def test_redraw_rate_is_capped(self):
    # 0.5 s 동안 사건 2000개 → 렌더 횟수(Application.render_counter) ≤ 0.5/0.1 + 2
    ...
def test_stale_age_keeps_updating_without_packets(self):
    # 마지막 update 뒤 0.35 s 동안 아무 요청 없이도 렌더 ≥ 3회, 상태줄의 "수신 없음 N.Ns" 값이 증가
    ...
def test_events_after_run_go_to_stdout(self):
    # run() 종료 → after_run() → ui.event("[LOG] 저장 완료: …") 가 stdout 에 보인다
    ...
def test_non_tty_uses_console_ui(self):
    with mock.patch.object(sys.stdout, "isatty", return_value=False):
        self.assertIsInstance(make_ui(ns(plain=False)), ConsoleUI)
def test_plain_flag_uses_console_ui(self): ...
def test_missing_prompt_toolkit_falls_back(self):
    with mock.patch.dict(sys.modules, {"prompt_toolkit": None}):
        self.assertIsInstance(make_ui(ns(plain=False), isatty=lambda: True), ConsoleUI)
```

- [ ] **Step 2:** FAIL → **Step 3:** 구현 → **Step 4:** PASS, 기준선 이상(조종 시험은 비TTY라 `ConsoleUI`로 돈다)
- [ ] **Step 5: 커밋** — `feat(ground): show the control station as a fixed dashboard`

### Task 7: `drone_control.py`, `--keyboard`, `--listen`, 불량 패킷 집계

**Files:**
- Create: `scripts/drone_control.py`
- Modify: `scripts/ground/control_station.py`(인자 `--keyboard`·`--listen`, 모드별 스레드·송신 제한, 불량 패킷 집계), 문서 링크·migration map
- Delete: `scripts/tune_pid.py`, `scripts/receive_telemetry.py`
- Test: `tools/test_drone_control.py`, `tools/test_control_dualsense.py`에 추가

**Interfaces:**
- `drone_control.py [--keyboard | --listen [--plot] | --pad-check] [--no-raw] [--no-mag] [--plain]`. 모드 플래그는 서로 배타. 이 Task에서는 `--keyboard`·`--listen`·기본 모드만 `control_station.main(argv)`로 넘긴다. `--pad-check`·`--plot`은 Task 8.
- `--keyboard`: 컨트롤러 스레드를 띄우지 않는다. `start`·`resume`은 보내지 않고 `[STDIN] 키보드 모드: 시동·복구는 패드로만 합니다` 사건. 종료 시 자동 송신 없음. CSV·raw 파일을 만들지 않는다(`tune_pid.py`와 같다 — 빈 `flight_log_*.csv`가 분석 기본값을 가리지 않게).
- `--listen`: 컨트롤러 스레드 없음. 송신 관문 `send_cmd`·`reliable_send`가 `connect`만 통과시킨다(트림 동기 재전송, `--no-raw`의 `raw 0`, 입력줄 명령, 종료 시 disarm 모두 막힘; 막힌 입력줄 명령은 `[STDIN] 수신 전용 모드` 사건). CSV·raw 기록은 한다.
- 불량 패킷: 모든 모드에서 텔레메트리 디코드·파싱 실패를 `bad_packet_count`로 세고(지금은 조용히 버린다), 스냅숏·상태줄에 보이며, 종료 시 `[LOG] 저장 {packet_count}개, 잘못된 패킷 {bad_packet_count}개`를 `event`로 남긴다.

- [ ] **Step 1: 실패하는 시험**

```python
def test_keyboard_mode_refuses_start_and_resume(self):
    m, sock = self._load_module(argv=("--keyboard",))
    for cmd in ("start", "resume"):
        m.handle_stdin_command(cmd)
    self.assertFalse(any(d in (b"start", b"resume") for d, _ in sock.sent))

def test_keyboard_mode_sends_nothing_on_exit(self): ...
def test_keyboard_mode_does_not_start_controller_thread(self): ...
def test_keyboard_mode_creates_no_log_files(self): ...
def test_listen_mode_sends_only_connect(self):
    # trim (0,0) → (1,2) 두 패킷 재생 + 입력줄 "pr 0.9" + --no-raw 시작 + 종료
    m, sock = self._load_module(argv=("--listen", "--no-raw"))
    ...
    self.assertEqual({d for d, _ in sock.sent}, {b"connect"})
def test_bad_packets_are_counted(self):
    out = self._run_datagrams([b"\xff\xfe", b"not,a,packet", good], rows)
    self.assertEqual(self.module.bad_packet_count, 2)
def test_modes_are_mutually_exclusive(self):
    self.assertNotEqual(run([sys.executable, DRONE_CONTROL, "--keyboard", "--listen"]).returncode, 0)
def test_help_does_not_open_socket_or_log(self): ...  # drone_control.py --help → control_station 미import
```

- [ ] **Step 2:** FAIL → **Step 3:** 구현 → **Step 4:** PASS, 기준선 이상
- [ ] **Step 5: 커밋** — `feat(ground): add drone_control.py with keyboard and listen modes`

### Task 8: `--listen --plot`과 `--pad-check`

**Files:**
- Move: `monitor_telemetry.py` → `ground/monitor_plot.py`(최상위 코드를 `main(argv)`로, import 시 부작용 없음, `LOG_DIR`은 `ground.paths`)
- Create: `ground/pad_check.py` — `gamepad_probe.py`의 매핑 진단과 `test_dualsense_input.py`의 연속 표시를 합친 `main(argv)`
- Modify: `scripts/drone_control.py`(`--pad-check`·`--listen --plot`을 `control_station` import 전에 처리), 문서 링크·migration map
- Delete: `scripts/gamepad_probe.py`, `scripts/test_dualsense_input.py`
- Test: `tools/test_monitor_plot.py`, `tools/test_pad_check.py`, `tools/test_drone_control.py`에 추가

- [ ] **Step 1: 실패하는 시험**

```python
def test_monitor_plot_import_has_no_side_effects(self):
    with mock.patch.object(socket, "socket") as s:
        importlib.reload(monitor_plot)
    s.assert_not_called()

def test_monitor_plot_update_consumes_sample(self): ...  # Agg 백엔드, 가짜 소켓 패킷 1개 → 데이터 길이 +1, CSV 1줄, 불량 패킷 집계 유지
def test_pad_check_reports_axis_button_hat(self): ...    # 가짜 pygame 이벤트 → 축 번호·값, 버튼·햇
def test_pad_check_never_creates_socket(self): ...
def test_pad_check_mode_does_not_import_control_station(self):
    with mock.patch.object(socket, "socket") as s, mock.patch.object(pad_check, "main", return_value=0):
        self.assertEqual(drone_control.main(["--pad-check"]), 0)
    s.assert_not_called(); self.assertNotIn("ground.control_station", sys.modules)
def test_listen_plot_mode_runs_monitor_plot(self): ...
```

- [ ] **Step 2:** FAIL → **Step 3:** 구현 → **Step 4:** PASS, 기준선 이상
- [ ] **Step 5: 커밋** — `refactor(ground): fold the telemetry plot and gamepad check into drone_control`

### Task 9: 보관과 문서 마무리

**Files:**
- Move: `bench_sign_test.py`, `bench_yaw_test.py`, `bench_thrust_ramp.py`, `motor_serial.py`, `receive_dual_imu_debug.py` → `scripts/archive/`(각 스크립트의 `sys.path`를 `scripts/`로, import를 `ground.*`로)
- Modify: `scripts/archive/README.md`(용도·전제, `scripts/README.md`에 있던 벤치 스크립트 경고를 옮김), `scripts/README.md`(세 도구 중심으로 다시 씀; UDP 단일 발신자 경고 유지), `tools/test_control_dualsense.py`의 `BenchTrimResetTests._load_bench`(경로 `scripts/archive/`), `tools/check_repo_layout.py`의 `LEGACY_TOKENS`에 옛 실행 스크립트 이름 추가, 남은 문서 링크·migration map(날짜 박힌 기록 문서는 고치지 않는다)
- Test: `tools/test_ground_layout.py`에 추가

- [ ] **Step 1: 실패하는 시험**

```python
def test_only_three_entry_points_at_top_level(self):
    self.assertEqual(sorted(p.name for p in SCRIPTS.glob("*.py")),
                     ["drone_control.py", "drone_logs.py", "drone_video.py"])

def test_old_script_names_are_legacy_tokens(self):
    for old in ("control_dualsense.py", "tune_pid.py", "receive_telemetry.py", "monitor_telemetry.py",
                "gamepad_probe.py", "test_dualsense_input.py", "record_camera.py", "camera_to_mp4.py",
                "analyze_flight_log.py", "decode_imu_raw.py", "analyze_probe_response.py", "magcal_fit.py"):
        self.assertIn(old, check_repo_layout.LEGACY_TOKENS)

def test_archived_scripts_resolve_ground_imports(self):
    for p in (SCRIPTS / "archive").glob("*.py"):
        src = p.read_text(encoding="utf-8")
        self.assertNotRegex(src, r"(?m)^from (telemetry_schema|failsafe_telemetry) import")
```

- [ ] **Step 2:** FAIL → **Step 3:** 구현 → **Step 4:** PASS, `check_repo_layout.py` 통과, 기준선 이상. `LEGACY_TOKENS`가 날짜 박힌 기록 문서를 잡으면 그 문서는 `maintained_markdown_files`의 검사 대상에서 빠져 있는지 확인하고, 아니면 원장에 판단을 남긴다.
- [ ] **Step 5: 커밋** — `docs(ground): describe the three ground tools and archive the bench scripts`

### Task 10: 실기 확인 (프로펠러 제거 벤치)

**Files:** 없음. 결과는 원장과 최종 보고에 적는다. 사용자와 함께 한다.

설계 7절 표의 모든 줄을 실제로 한 번씩 확인한다.

- [ ] `drone_control.py`: 대시보드 표시 → X 시동(Armed=1 확인 사건) → 스로틀 R2/L2·R1/L1 → 트림 DPAD·PS 리셋 → 입력줄 `gains`·`pr <값>`·Esc → 컨트롤러 분리/재연결 → Ctrl+C 종료 시 disarm·로그 저장·raw CSV 변환 사건이 일반 출력으로 남음. Ctrl+D 종료도 한 번.
- [ ] pygame이 켜진 상태에서 Ctrl+C가 대시보드에 전달되는지(SDL 시그널 처리와 충돌 없음)
- [ ] 자동착륙 표시: 패드를 끄고 RC 타임아웃을 일으켜 failsafe 패널 강조·상태줄 경고·`[STATE]` 전이 사건 확인
- [ ] 두 번째 지상국 실행 → 포트 점유 메시지로 거부
- [ ] `--keyboard`: `start` 거부 사건, 게인 명령 동작, 종료 시 아무것도 안 보냄, 로그 파일 없음
- [ ] `--listen`(+`--plot`): 수신·CSV, `connect` 외 미송신. `--pad-check`: 축·버튼 표시
- [ ] `drone_video.py`: 녹화·미리보기, `mp4` 변환. `drone_logs.py`: 하위 명령 넷을 오늘 로그로 한 번씩, `flight` 경로 생략 시 최신 비행 로그 선택
- [ ] 80×24 미만 터미널에서 접힌 배치, `--plain`에서 줄 출력

## 자체 검토

- 설계 대응: 5.1 표 → Task 2·3·7·8, 5.2 → Task 1·9, 5.3 → Task 5, 6절 → Task 5·6, 7절 → Task 10과 각 시험, 8절 → Task 0·각 단계의 기준선 유지.
- 독립 검토 기록:
  1. **직접 재계산(Claude).** 텔레메트리 필드 75(main)·91(flow-hold), failsafe phase·probe 이름 표, 조종 모듈 `print` 70곳, 조종 시험 108개, 텔레메트리 스레드의 1 s `connect`, 텔레메트리 끊김 → `stop_streaming_only`, 종료 시 조건부 disarm을 소스에서 확인했다. 초안이 지어낸 시험 도우미 이름(`sample75` 생성기, `write_synthetic_capture`)을 실제 이름(`packet(field_count)`, `_zimu`/`_zcal`, `_run_analyzer`)으로 고쳤다.
  2. **Codex 적대적 검토(P1 1·P2 9).** 워커 예외 시 RC 지속(P1) → Task 4 워커 감싸기·`stop_streaming_only`. EOF 미disarm → Task 4 `on_eof`. `--listen`의 트림 재전송·`raw 0` → Task 7 송신 관문. 이동 후 `__file__` 기준 경로 → Task 1 `ground/paths.py`. 문서·migration map·검사기를 Task 9로 미루면 Task 1부터 CI 실패 → 이동하는 커밋마다 고침, 검사기의 새 경로 중복 허용. 벤치 시험 로더·벤치 `sys.path`·분석기 subprocess 경로 → Task 2·9. Task 7이 Task 8 산출물에 의존 → `--pad-check` 시험을 Task 8로. `refresh_interval`은 상한이 아님 → `min_redraw_interval` + 렌더 횟수 시험. 종료 사건이 화면 뒤에 갇힘 → `after_run()` 순서. 불량 패킷 집계 누락 → Task 7.
  2-1. **Codex 재검토(P1 2·P2 1).** 워커 오류 뒤 `resume`·시동 확정으로 RC가 다시 열림 → 오류 래치. `ui.alert` 실패 시 RC 차단 생략 → 안전 상태 변경을 먼저, UI 메서드는 예외를 내지 않음. `min_redraw_interval`만으로는 무수신 중 갱신이 멈춤 → `refresh_interval=0.1` 병행과 시험.
  3. **agy(Gemini 4 Argon) 독립 검토(결론 비공개).** 키보드 모드 종료 시 자동 `stop`이 자동착륙 하강 중 모터를 끔 → 자동 `stop` 삭제(설계 정정, 사용자 확인 필요). 키보드 모드가 빈 비행 로그를 만들어 분석 기본값을 가림 → 키보드 모드 로그 없음 + `flight` 기본값을 `flight_log_*.csv`로. 2 Hz `[AXIS]`가 사건 영역을 도배 → `periodic` 분류. 프로브 딥 같은 짧은 전이가 10 Hz 패널에서 사라짐 → `[STATE]` 전이 사건. prompt_toolkit 미설치 시 조종기가 안 뜸 → 지연 import 폴백. 종료 시 raw 변환 출력이 화면 뒤에 갇힘 → `after_run()`. migration map 44행·중복 규칙, 벤치 import 깨짐 → Task 1·9. 입력줄 오염(`pr 0.9stop`) → Esc로 비움(지금 `input()`과 같은 위험, 패드 X로 disarm 가능). pygame/SDL의 SIGINT 처리 → 대시보드는 Ctrl+C를 키 입력으로 받지만 Task 10에서 실기 확인.
