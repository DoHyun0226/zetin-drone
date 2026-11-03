# 거리계 기반 자동착륙 구현 계획 (flow-hold 스케치)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** flow-hold 스케치에서 RC가 끊기면 거리계로 고도를 재며 위치 유지를 켠 채 천천히 내려오고, 착지를 확인하면 컷한다. 거리값을 못 믿으면 지금의 시간 기반 하강/즉시 컷으로 폴백한다.

**Architecture:** 하강 목표·착지 판정·폴백 판정은 flow-hold 전용 새 헤더 `range_land.h` 의 순수 함수로 만들고 네이티브 시험한다. 고도 제어는 기존 `AltHold` 에 외부 목표(고도·수직속도)와 하강 전용 이득을 더해 재사용한다. `.ino` 의 RC 타임아웃 블록이 진입 시 모드를 고르고 매 tick 이 함수들을 부른다. 단계 번호는 그대로(`FS_DESCENDING`), 모드·단계·목표 고도는 flow-hold 텔레메트리 뒤에 덧붙인다.

**Tech Stack:** Arduino(ESP32-S3), C++17 네이티브 시험(g++ -m32, `tools/test_flow_hold_native.py`), SIL(`tools/native_tests/test_sil_flow_hold.cpp` 가 `.ino` 를 include), Python 지상국(`scripts/ground/`).

**Spec:** [docs/superpowers/specs/2026-11-03-range-autoland-design.md](../specs/2026-11-03-range-autoland-design.md)

## Global Constraints

- 주력 스케치 `firmware/flight/dual_imu_cascade_pwm/` 는 건드리지 않는다. 공유 헤더 6개(`control_allocator.h`, `failsafe_land.h`, `mag_yaw_fusion.h`, `msp_sensor.h`, `yaw_authority.h`, `yaw_command.h`)도 바꾸지 않는다(`tools/test_flow_hold_sketch.py`).
- `FailsafePhase` 번호·의미 그대로. 거리 기반·시간 기반 하강 모두 `FS_DESCENDING`, 착지 컷 `FS_CUT_LANDED`, 백스톱 `FS_CUT_TIMEOUT`.
- 텔레메트리는 기존 91필드 뒤에만 덧붙인다. 주력 펌웨어 패킷(75필드)은 새 필드를 `None` 으로 둔다.
- 진입 조건 그대로: 호버 추정 무효 → 즉시 컷, `base_throttle <= FS_GROUND_CUT_MAX_US`(1150) → 지상 컷, 그 밖 → 자동착륙. 거리 기반 모드는 그 안에서 진입 순간 높이 추정이 신선할 때만.
- 시간 기반 하강(`FS_MAX_MS` 3000, 프로브 딥, `FS_DESCENT_DELTA_US` 60) 자체는 바꾸지 않는다. 폴백은 바닥 근접 전에만 그 하강을 새로 시작하고(한 번의 예산 3 s 그대로), 폴백 에피소드 내내 기준 스로틀은 폴백 직전 실제 collective 를 넘지 않는다(`fallback_cap`). 시간 상한은 **연속된 한 failsafe 에피소드**마다 `RL_MAX_MS + FS_MAX_MS` 이고, resume 으로 끝난 뒤 다시 RC 를 잃으면 새 에피소드다(반복 사이클은 기존 9 s 누적 resume 규칙이 막는다).
- `FS_RESUME_MAX_MS`(첫 진입부터 누적 9000 ms) 규칙은 그대로 — 긴 거리 기반 하강 끝 무렵 resume 은 거부될 수 있다(착륙은 계속). failsafe 진입 시 위치·고도 유지 요청을 끄는 지금 규칙도 그대로(resume 뒤 조종자가 다시 켠다).
- 1 kHz 루프 안 `Serial` 출력은 모드·단계 전이 때 한 번씩만.
- 착지 판정의 거리 기준은 원값(`range_mm`). 실측 착지 원값 18–37 mm, 중앙값 29–31 mm, 품질 255, 범위 밖 −1(2026-11-03). `range_timing_ok=false` 프레임(Core 0 지연 뒤 꺼내 시각을 못 믿는 프레임)은 근접·접촉 증거로도, 무효 증거로도 쓰지 않는다. 신선도는 마지막 신뢰 프레임 기준. 높이 추정 갱신 규칙은 그대로.
- SIL 시험은 사건(링크 끊김·거리 끊김·외란 변경·resume)을 **한 번의 `runTicks` 안에서 `on_tick` hook 으로** 넣는다. `runTicks` 를 나눠 부르면 `pid_task` 의 지역 상태가 다시 만들어져 진행 중인 failsafe 가 재시작된다.
- 아래 상수는 첫 값이다. SIL 결과로 바꾸면 근거를 원장에 남긴다.

| 상수 (`range_land.h`) | 첫 값 | 근거 |
|---|---|---|
| `RL_DESCENT_FAST_MPS` / `RL_DESCENT_SLOW_MPS` | 0.30 / 0.12 | 1.8 m에서 약 5 s, 접근 구간 감속 |
| `RL_APPROACH_M` | 0.30 | 이 높이 아래에서 느리게 |
| `RL_REF_LEAD_M` | 0.15 | 목표가 실제보다 이만큼 넘게 아래로 가지 않음 |
| `RL_NEAR_GROUND_MM` | 100 | `FLOW_RANGE_MIN_M` 0.10 m. 이 아래 신선한 양수 원값은 바닥 근접 |
| `RL_LANDED_MIN_MM` / `RL_LANDED_MAX_MM` | 10 / 60 | 실측 18–37 mm에 여유 |
| `RL_CONTINUITY_MS` | 1000 | 바닥 근접 진입 직전 이 시간 안에 신선한 높이 ≤ `RL_APPROACH_M` 이 있어야 함 |
| `RL_MAX_STEP_M` | 0.05 | 예측 원값과의 허용 차이 고정분 |
| `RL_MAX_VZ_MPS` | 0.6 | 허용 차이 공백분: `tol = RL_MAX_STEP_M + RL_MAX_VZ_MPS·gap`, `gap` = 마지막 채택 뒤 **첫 신뢰 프레임이 도착하기까지** 걸린 시간(그 순간 고정, 거부가 이어져도 더 커지지 않음). 35 ms 주기면 0.071 m, 결손 300 ms 뒤면 0.23 m |
| `RL_RESUME_SLEW_US_S` | 300 | resume 때 collective 를 호버로 넘기는 속도(`ALTHOLD_SLEW_US_S` 와 같음) |
| `RL_CONTACT_DROP_US` | 40 | 착지 확정에 필요한 호버 대비 최소 스로틀 감소(SIL 1 µs ≈ 0.029 m/s², 40 µs ≈ 1.2 m/s² 부족) |
| `RL_LANDED_HOLD_MS` | 400 | 접촉 감소 도달 뒤 착지 대역 유지 시간 |
| `RL_TOUCHDOWN_RAMP_US_S` | 100 | 바닥 근접에서 기준 스로틀 감소 속도 |
| `RL_TOUCHDOWN_MAX_MS` | 3000 | 바닥 근접 진입 뒤 착지 미확정이면 그 자리 컷 |
| `RL_INVALID_GRACE_MS` | 300 | `ALTHOLD_SUSPEND_S` 와 같은 성격 |
| `RL_MAX_MS` | 12000 | (1.8−0.3)/0.30 + 0.3/0.12 + 접촉 확인 여유 |

## Review Focus

1. **공중 착지 오판** — 높은 곳에서 근접 물체(손·풀·테더 줄)가 원값을 착지 대역으로 만들면 착지가 아니라 이상값(→ 폴백). Task 1 `jump_from_height_to_landed_band_is_invalid`, `two_step_jump_does_not_pass_continuity`, Task 4 `sil_near_object_at_1_5m_does_not_cut_landed`, `sil_two_step_near_object_does_not_cut`.
2. **하강 권한·호버 편향** — 호버 추정이 ±30 µs 틀리고 배터리가 처져도 정해진 시간 안에 바닥 근접까지 내려간다. Task 2·4 `sil_hover_bias_*`, `sil_battery_sag_*`.
3. **바닥 근처 무효·데드존** — 60–100 mm 통과는 폴백이 아니고, 바닥 근접 뒤 −1 은 3 s 폴백이 아니라 컷. Task 1 `near_ground_60_to_100mm_is_not_invalid`, `loss_after_near_ground_cuts`.
4. **스로틀 상승** — 착지 접근·폴백 전환에서 실제 collective 가 오르지 않는다. Task 4 `sil_no_collective_rise_after_near_ground`, `sil_fallback_does_not_raise_collective`.
5. **Core 0 지연** — 거리 프레임 시각이 의심스러워도 원값 판정 때문에 폴백하지 않는다. Task 4 `sil_core0_stall_does_not_force_fallback`, `sil_backlogged_near_frame_is_not_contact_evidence`.

---

### Task 1: `range_land.h` 순수 로직

**Files:** Create `firmware/flight/dual_imu_cascade_flow_hold/range_land.h`, `tools/native_tests/test_range_land.cpp`; Modify `tools/test_flow_hold_native.py`(`test_range_land`).

**Interfaces (Produces):**
- `enum RangeLandMode : uint8_t { RL_MODE_NONE=0, RL_MODE_RANGE=1, RL_MODE_TIMED=2, RL_MODE_TIMED_FALLBACK=3 };` (텔레메트리, append-only)
- `enum RangeLandStage : uint8_t { RL_STAGE_IDLE=0, RL_STAGE_DESCENT=1, RL_STAGE_APPROACH=2, RL_STAGE_NEAR_GROUND=3, RL_STAGE_CONTACT=4 };`
- `struct RangeLandState { uint8_t stage; float h_ref; uint32_t enter_ms, last_fresh_ms, last_low_fresh_ms, near_ground_since_ms, landed_since_ms, invalid_since_ms, ref_ms; bool invalid_timing, landed_timing, tol_fixed; int16_t ref_mm, tol_mm, pred_mm; float drop_us; };`
- `enum RangeLandCutReason : uint8_t { RL_CUT_NONE=0, RL_CUT_CONTACT=1, RL_CUT_NEAR_GROUND_LOSS=2, RL_CUT_NEAR_GROUND_STUCK=3, RL_CUT_BACKSTOP=4 };` (텔레메트리, append-only)
- `void rangeLandStart(RangeLandState&, float h_now, uint32_t now_ms);`
- `struct RangeLandInput { bool height_fresh; float h_m; int32_t range_mm; bool range_fresh; bool range_trusted_new; float hover_us; float delivered_collective_us; float dt_s; uint32_t now_ms; };` — `delivered_collective_us` 는 직전 tick allocator 출력 `collective_us`.
- `enum RangeLandAction : uint8_t { RL_CONTINUE=0, RL_LANDED, RL_TIMEOUT, RL_FALLBACK, RL_CUT_NEAR_GROUND };` — 출력에 `cut_reason` 포함.
- `struct RangeLandOutput { uint8_t stage; RangeLandAction action; float h_ref; float vz_ref; float drop_us; float hold_scale; };`
  - `vz_ref`: DESCENT −0.30, APPROACH −0.12, NEAR_GROUND/CONTACT 0(기준 스로틀 감소로 내려감). `hold_scale`: 위치 유지 적용 비율, DESCENT·APPROACH 1, NEAR_GROUND 이후 0 쪽으로 감쇠.
- `RangeLandOutput rangeLandStep(RangeLandState&, const RangeLandInput&);` 규칙:
  - 입력의 `range_fresh` 는 **신뢰 프레임**(`range_timing_ok`)의 나이로 계산된 값이고, `range_trusted_new` 는 이번 tick 에 새 신뢰 프레임이 왔는지다.
  - 추적 기준(`ref_mm`, `ref_ms`, `tol_mm`): 시작 때 진입 원값. 예측 `pred = ref_mm − |vz_ref|·(t_pred − ref_ms)`, `t_pred` = 마지막 채택 뒤 첫 신뢰 프레임 도착 시각(`tol_mm` 과 같은 순간 고정). 거부가 이어지는 동안 허용폭도 예측 중심도 움직이지 않는다 — 실제로 내려가지 않았는데 명령 기반 예측이 이상값 쪽으로 다가가 연속성을 만들어 주는 일을 막는다. 새 신뢰 프레임은 `raw > 0 && |raw − pred| ≤ tol_mm` 일 때만 **채택**해 `ref_mm=raw`, `ref_ms=now`, `tol_mm` 재고정 대기. 그 밖(음수·−1 포함)은 **거부** — 거부 프레임은 기준·`tol_mm`·무효 타이머 어느 것도 바꾸지 않고, 거부가 이어지는 동안 무효 타이머가 돈다. 무효 타이머는 **채택 프레임으로만** 지운다(다른 무효 조건이 남아 있으면 지우지 않음). 그래서 지속된 점프는 유예 300 ms 뒤 폴백/컷이지 새 추적이 아니다. 아래 판정은 **채택 프레임**만 본다.
  - 신선한 높이: `h_ref -= |vz_ref|·dt`, `h_ref = max(h_ref, h_m − RL_REF_LEAD_M)`, `h_m <= RL_APPROACH_M` 이면 APPROACH, `last_low_fresh_ms` 갱신.
  - 바닥 근접: `range_fresh && 채택 && 0 < range_mm < RL_NEAR_GROUND_MM` 이고 `now − last_low_fresh_ms <= RL_CONTINUITY_MS` → NEAR_GROUND(진입 시각 기록), `drop_us += RL_TOUCHDOWN_RAMP_US_S·dt`. 연속성이 없으면 **이상값 → 무효**로 센다.
  - CONTACT: NEAR_GROUND 에서 `drop_us >= RL_CONTACT_DROP_US` **이고** `delivered_collective_us <= hover_us − RL_CONTACT_DROP_US`(allocator 가 자세 권한 때문에 실제 collective 를 올려 둔 동안은 추력 부족 근거가 없으므로 접촉 타이머를 돌리지 않고 리셋). `range_fresh && RL_LANDED_MIN_MM <= range_mm <= RL_LANDED_MAX_MM` 인 동안 착지 타이머, `RL_LANDED_HOLD_MS` 이면 `RL_LANDED`. 신선하지 않거나 대역 밖이면 타이머 리셋.
  - 무효: `!range_fresh || range_mm <= 0 || 이상값 || (!height_fresh && range_mm >= RL_NEAR_GROUND_MM)` 이 `RL_INVALID_GRACE_MS` 넘게 → NEAR_GROUND 전이면 `RL_FALLBACK`, NEAR_GROUND 이후면 `RL_CUT_NEAR_GROUND`.
  - NEAR_GROUND 진입 뒤 `RL_TOUCHDOWN_MAX_MS` 안에 `RL_LANDED` 가 없으면 `RL_CUT_NEAR_GROUND`.
  - `now − enter_ms >= RL_MAX_MS` → `RL_TIMEOUT`. 우선순위 TIMEOUT > LANDED > CUT_NEAR_GROUND > FALLBACK.

- [ ] **Step 1: 실패하는 시험** (`test_range_land.cpp`)

```cpp
runCase("ref_descends_fast_then_slow_and_reports_vz_ref", ...);
runCase("ref_never_leads_measured_height_by_more_than_lead", ...);
runCase("short_positive_range_is_not_invalid_after_continuous_approach", ...);
runCase("near_ground_60_to_100mm_is_not_invalid", ...);              // 80 mm 1 s → FALLBACK 없음, drop 증가
runCase("jump_from_height_to_landed_band_is_invalid", ...);          // 1.5 m 신선 → 다음 프레임 30 mm → 이상값, 300 ms 뒤 FALLBACK
runCase("two_step_jump_does_not_pass_continuity", ...);              // 1.5 m → 200 mm → 30 mm → 둘 다 거부, NEAR_GROUND 없음
runCase("held_jump_never_becomes_reference", ...);                   // 1500 → 200 mm 600 ms 유지 → 130 → 60 mm: 거부 지속 300 ms 뒤 FALLBACK, LANDED 없음
runCase("tolerance_does_not_grow_during_rejection", ...);            // 채택 300 mm 뒤 90 mm 를 35 ms 마다: 280 ms 에도 거부, 첫 거부 + 300 ms 에 FALLBACK
runCase("staircase_rejections_do_not_walk_prediction", ...);       // APPROACH 0.12 m/s, 채택 300 mm 뒤 200 mm 를 210 ms, 100 mm 를 245 ms, 40 mm: 모두 거부, 첫 거부 + 300 ms 에 FALLBACK
runCase("negative_raw_is_never_accepted", ...);                      // 채택 150 mm 뒤 −1 반복: 채택 0건, 300 ms 뒤 FALLBACK
runCase("contact_needs_delivered_collective_drop", ...);             // drop_us 40 이지만 delivered = hover − 5 → 착지 타이머 안 돎; delivered = hover − 40 → 400 ms 뒤 LANDED
runCase("normal_descent_frames_are_all_accepted", ...);              // 0.30 m/s 하강, 35 ms 주기, 6 % 프레임 결손 → 거부 0건
runCase("accepted_frame_resets_rejection_timer", ...);               // 거부 200 ms → 채택 1건 → 무효 타이머 리셋
runCase("untrusted_frames_are_neither_evidence_nor_invalid", ...);  // range_trusted_new=false 30 mm 프레임 → NEAR_GROUND 없음, 무효 타이머는 신뢰 프레임 나이로만
runCase("landed_requires_contact_drop", ...);                       // 30 mm 유지라도 drop < 40 µs 면 LANDED 없음
runCase("landed_after_hold_once_drop_reached", ...);                // drop ≥ 40 후 399 ms false, 400 ms LANDED
runCase("landed_timer_stops_on_stale_frame", ...);                  // 대역 300 ms 후 stale → 타이머 리셋, 400 ms 에 LANDED 없음
runCase("fallback_after_grace_before_near_ground", ...);            // 299 ms false, 300 ms FALLBACK
runCase("loss_after_near_ground_cuts", ...);                        // NEAR_GROUND 뒤 −1 300 ms → CUT_NEAR_GROUND
runCase("stuck_in_near_ground_cuts_after_max", ...);                // 80 mm 고정 2999 ms false, 3000 ms CUT_NEAR_GROUND
runCase("timeout_backstop_and_priority", ...);
runCase("hold_scale_fades_after_near_ground", ...);
```

- [ ] **Step 2:** `python -m unittest tools.test_flow_hold_native` → FAIL(헤더 없음)
- [ ] **Step 3:** 구현 → **Step 4:** PASS, 전체 시험 기준선(467) 이상, layout 통과
- [ ] **Step 5: 커밋** — `feat(flow): add the range-based landing logic as pure functions`

### Task 2: `AltHold` 외부 목표(고도·수직속도)와 하강 이득

**Files:** Modify `firmware/flight/dual_imu_cascade_flow_hold/flow_hold.h`; Test `tools/native_tests/test_flow_hold.cpp`.

**Interfaces:**
- `AltHoldInput` 에 `bool external_ref = false; float h_ref_ext = 0.0f; float vz_ref_ext = 0.0f; bool no_positive = false;`. `external_ref` 면 `h_ref = h_ref_ext`, 감쇠항은 `k_vz·(vz_ref_ext − vz)`(목표 속도를 따라감), ADJUST 판정 없음. `no_positive` 면 최종 보정을 0 이하로 자른다.
- `AltHoldGains altHoldDescentGains()` — 하강 전용 이득(첫 값: `out_max_us` 100, `i_max_us` 60, `k_i` 40, `k_h`·`k_vz` 기본과 같음; `CTRL_MARGIN` 150 안). SIL(Task 4)의 편향·처짐 시험이 통과하는 값으로 확정하고 근거를 원장에.
- 기존 호출(`external_ref=false`)의 출력은 기존 시퀀스와 바이트 단위로 같아야 한다.

- [ ] **Step 1: 실패하는 시험**

```cpp
runCase("althold_external_ref_tracks_target_velocity", ...);   // vz_ref −0.3, vz −0.3, h=h_ref 에서 감쇠항 0
runCase("althold_external_ref_ignores_base_changes", ...);
runCase("althold_no_positive_clamps_output", ...);
runCase("althold_default_path_unchanged", ...);               // 변경 전 실행으로 기록한 기대 출력과 일치
```

- [ ] **Step 2–4:** RED → 구현 → GREEN, 기존 flow_hold 시험 전부 통과
- [ ] **Step 5: 커밋** — `feat(flow): let altitude hold track an external height and descent rate`

### Task 3: SIL 센서·기체 모델 보강 (기존 시험 불변)

**Files:** Modify `tools/native_tests/test_sil_flow_hold.cpp`.

**Interfaces:**
- `SensorModel`: `int ground_offset_mm = 0;`, `bool report_short = false;`(80 mm 미만도 보고), `int noise_mm = 0;`(결정적 의사난수), `std::function<int32_t(uint32_t, int32_t)> range_override;`(근접 물체·끊김 주입). 기본값이면 지금 출력과 같다.
- `Params`/`Run`: `double thrust_scale = 1.0;`(배터리 처짐, 시간 함수 `std::function<double(uint32_t)> thrust_scale_at`), `std::function<double(uint32_t)> tether_floor;`, `double ground_effect_gain = 0;`(바닥 근처 추력 증가).
- `hover_est` 를 강제로 편향시키는 하네스 도우미 `forceHoverEstimate(float us)`(진입 직전 hook 에서).

- [ ] **Step 1:** 새 케이스 `sil_sensor_reports_landed_range`(지면에서 18–38 mm) 실패 확인
- [ ] **Step 2–4:** 구현, 기존 SIL 27개 케이스 그대로 통과
- [ ] **Step 5: 커밋** — `test(flow): model landed range, near objects, sag, ground effect and a tether in the SIL`

### Task 4: `.ino` 연결

**Files:** Modify `firmware/flight/dual_imu_cascade_flow_hold/dual_imu_cascade_flow_hold.ino`, `tools/native_tests/test_sil_flow_hold.cpp`.

**Interfaces:**
- 전역 `RangeLandState rangeLand; uint8_t rl_mode = RL_MODE_NONE; uint8_t rl_stage, rl_cut_reason; int rl_collective_cap, rl_fallback_cap;`. 원값 신뢰·신선도: `range_timing_ok` 인 새 프레임만 신뢰, 신선 = 마지막 신뢰 프레임 나이 ≤ `FLOW_HEIGHT_MAX_AGE_US`.
- RC 타임아웃 자동착륙 분기: `heightFresh` 면 `RL_MODE_RANGE` + `rangeLandStart` + `[FAULT] RC TIMEOUT -> AUTO-LAND (RANGE)`, 아니면 `RL_MODE_TIMED`(지금 경로).
- 위치 유지(거리 기반 모드): 조종 입력은 0 으로 보고(`OVERRIDE` 되지 않게), 기준점은 진입 위치로 옮기되 적분·출력은 리셋하지 않고 이어받는다. 적용값 × `hold_scale`.
- 고도: `altHoldStep(external_ref=true, h_ref_ext, vz_ref_ext, no_positive=(stage ≥ NEAR_GROUND))` with `altHoldDescentGains()`. 거리 기반 모드에서는 지상 컷 경계 기반 `inhibit`/`hard_zero` 를 쓰지 않는다(그 경계는 진입 판정용).
- 기준 스로틀 `= round(hover_est) − drop_us`. `min_throttle`/`max_throttle` 창은 기존 자동착륙과 같은 규칙으로 기준 주변.
- **collective 래칫**: NEAR_GROUND 이후 매 tick `기준 + 고도 보정` 합계를 `rl_collective_cap = min(rl_collective_cap, 합계)` 로 묶는다(감쇠 중인 음수 보정이 0 쪽으로 돌아와도 합계는 오르지 않음). 래칫과 `fallback_cap` 은 **allocator 에 넣는 요청 collective** 에 대한 보장이다. 공유 `control_allocator.h` 가 모터 하한에서 자세 권한을 지키려 실제 collective 를 올리는 몫은 기존 우선순위(1dc8582, 자세 > collective)대로 둔다 — 바꾸면 접지 직전 자세 권한을 잃는다. 그 몫은 텔레메트리(실제 모터 평균 − 요청)로 보이고 SIL 로 한정한다.
- **resume 인계**: 거리 기반 모드에서 resume 이 수락되면 기준 스로틀을 그 순간 collective 에서 시작해 `RL_RESUME_SLEW_US_S` 로 호버까지 올린다(계단 없음). 인계 중(`rl_resume_handover`)에 RC 를 다시 잃으면 `base_throttle ≤ FS_GROUND_CUT_MAX_US` 지상 컷 분기를 쓰지 않는다 — 그 낮은 base 는 지상이라서가 아니라 인계 중이라서다. 진입 판정은 `max(base_throttle, hover_est)` 로 하고, 호버 추정 무효면 지금처럼 즉시 컷. 인계가 호버에 닿으면 플래그를 내리고 기존 판정으로 돌아간다.
- 액션: `RL_LANDED` → `FS_CUT_LANDED`(사유 CONTACT), `RL_TIMEOUT` → `FS_CUT_TIMEOUT`(BACKSTOP), `RL_CUT_NEAR_GROUND` → `FS_CUT_TIMEOUT`(NEAR_GROUND_LOSS/STUCK — 접촉이 확정되지 않았으므로 `CUT_LANDED` 가 아니다), `RL_FALLBACK` → `RL_MODE_TIMED_FALLBACK` 로 시간 기반 하강을 진입 때와 같은 초기화로 시작하고 `rl_fallback_cap = 직전 실제 collective`, 에피소드 내내 시간 기반 기준 스로틀을 `min(…, rl_fallback_cap)` 로 묶는다.
- 모드 전환 tick: 기존 `forceOuterRecompute`. 단계·모드 전이 때만 Serial 한 줄. resume 성공 시 `rl_mode = RL_MODE_NONE`.

- [ ] **Step 1: 실패하는 SIL 시험** (`ground_offset_mm=30, report_short=true, noise_mm=8`, 사건은 한 번의 `runTicks` 안 hook)

```cpp
runCase("sil_lands_from_0_6m_with_cut_landed", ...);        // CUT_LANDED, 컷 시점 s.h < 0.02
runCase("sil_lands_from_1_2m_with_cut_landed", ...);
runCase("sil_lands_from_1_7m_within_backstop", ...);        // 경과 < RL_MAX_MS
runCase("sil_hover_bias_plus_30us_still_lands", ...);       // hover_est +30 µs
runCase("sil_hover_bias_minus_30us_lands_without_hard_hit", ...);  // 접지 직전 |vh| < 0.5 m/s
runCase("sil_battery_sag_10pct_still_lands", ...);          // thrust_scale 1.0→0.9 선형
runCase("sil_ground_effect_still_reaches_near_ground", ...);
runCase("sil_poshold_applied_during_range_descent", ...);   // 3° 외란, 수평 이탈 < 0.5 m, 접지 순간 |applied| ≈ 0
runCase("sil_no_collective_rise_after_near_ground", ...);   // NEAR_GROUND 이후 allocator 전달 collective 단조 비증가
runCase("sil_dropout_mid_descent_falls_back_to_timed", ...); // 0.8 m 끊김 → TIMED_FALLBACK, 3 s 규칙대로 종료
runCase("sil_fallback_does_not_raise_collective", ...);   // 전환 tick 과 그 뒤 1 s 동안 collective 상승 없음
runCase("sil_two_step_near_object_does_not_cut", ...);    // 1.5 m에서 200 mm → 30 mm 주입 → 착지 컷 없음
runCase("sil_backlogged_near_frame_is_not_contact_evidence", ...);  // stall 뒤 꺼낸 근접 프레임으로 NEAR_GROUND 진입 없음
runCase("sil_resume_near_ground_has_no_step", ...);       // NEAR_GROUND 에서 resume → collective 가 300 µs/s 이하로만 오름
runCase("sil_rc_loss_during_resume_handover_reenters", ...);  // 0.8 m, collective 1140 에서 resume → 2 ms 뒤 RC 재상실 → FS_DESCENDING(지상 컷 아님)
runCase("sil_rc_loss_after_handover_uses_ground_rule", ...);  // 인계 완료 뒤 지상(base ≤ 1150)에서 RC 상실 → 기존 지상 컷
runCase("sil_near_ground_roll_disturbance_bounded", ...);   // NEAR_GROUND 에서 roll 외란: 요청 collective 단조 비증가, 기체 높이가 NEAR_GROUND 중 최저점 + 0.02 m 를 넘지 않고 상승 속도 ≤ 0.05 m/s, 기체 높이 > 0.01 m 인 동안 LANDED 없음, RL_TOUCHDOWN_MAX_MS 안에 CUT_LANDED 또는 STUCK 컷
runCase("sil_resume_reentry_starts_a_new_episode", ...);  // resume 뒤 재상실 → 새 에피소드, 9 s 누적 규칙은 resume 만 제한
runCase("sil_loss_after_near_ground_cuts_not_timed", ...);  // 바닥 근접 뒤 −1 → 300 ms 뒤 CUT_TIMEOUT, 사유 NEAR_GROUND_LOSS
runCase("sil_near_object_at_1_5m_does_not_cut_landed", ...); // 1.5 m에서 원값 30 mm 1 s 주입 → CUT_LANDED 없음
runCase("sil_tether_floor_5cm_cuts_near_ground", ...);      // 높이 0.05 m(원값 약 80 mm, 오프셋 30 mm)에 걸림 → RL_TOUCHDOWN_MAX_MS 뒤 CUT_TIMEOUT, 사유 STUCK
runCase("sil_tether_floor_2cm_cut_is_harmless", ...);       // 2 cm 걸림 컷 → s.h ≤ 0.02 에서 컷
runCase("sil_core0_stall_does_not_force_fallback", ...);    // stall_from/to 100 ms 하강 중 → FALLBACK 없음
runCase("sil_height_invalid_at_entry_uses_timed", ...);
runCase("sil_hover_invalid_still_cuts_immediately", ...);
runCase("sil_resume_during_range_descent", ...);            // 진입 4 s 뒤 RC 복귀+resume → FS_NONE
runCase("sil_mode_switch_is_smooth", ...);
```

- 기존 케이스 `RC loss: applied hold is zero from the failsafe tick on` 은 **의도적으로 범위를 좁힌다**: 시간 기반 모드(진입 때 거리 끊김)에서 그대로 검증하고, 거리 기반 모드의 위치 유지 적용은 새 케이스가 검증한다. 변경 사유를 원장에.

- [ ] **Step 2:** RED → **Step 3:** 구현 → **Step 4:** SIL·네이티브·전체 시험, 공유 헤더 시험, `arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/flight/dual_imu_cascade_flow_hold` 통과. 상수 변경 근거를 원장에.
- [ ] **Step 5: 커밋** — `feat(flow): land on the range sensor after RC loss and fall back to the timed descent`

### Task 5: 텔레메트리·지상국·규격 문서

**Files:** `.ino`(`FlowHoldTelemetry` + 직렬화: 92 `Land_Mode`, 93 `Land_Stage`, 94 `Land_Ref_M`, 95 `Land_Cut_Reason`), `scripts/ground/telemetry_schema.py`, `scripts/ground/failsafe_telemetry.py`(이름 표), `scripts/ground/dashboard_view.py`(failsafe 줄, 필드 있을 때만), `scripts/ground/control_station.py`(전이 사건에 `Land_Mode`·`Land_Stage`), `docs/udp_protocol.md`. Test: `tools/test_telemetry_schema.py`, `tools/test_dashboard_view.py`, `tools/test_control_dualsense.py`, SIL.

- [ ] **Step 1:** `test_flow_hold_95_field_packet_parses_land_fields`, `test_91_and_75_field_packets_leave_land_fields_none`, `test_dashboard_shows_range_landing_mode_and_ref`, `test_land_stage_transition_is_an_event` 실패 확인
- [ ] **Step 2–4:** 구현 → 전체 시험·layout·compileall·Arduino 컴파일
- [ ] **Step 5: 커밋** — `feat(flow): report the range landing mode, stage and target height`

### Task 6: 문서

`docs/firmware_catalog.md`(flow-hold 줄), 스케치 머리 주석, `docs/power_on_bench_procedure.md` 에 거리 기반 자동착륙 벤치·테더 절 추가:
- 호버 추정을 유효하게 만드는 절차(프롭 제거·무장 뒤 기체를 수평으로 들고 스로틀 > 1150 µs 를 1.5 s 넘게 흔들림 없이 유지 → 대시보드 `Hover_Valid=1` 확인 뒤 패드 분리).
- 폴백 시험은 센서를 손으로 가리지 않는다(가까운 손은 착지 대역으로 읽힌다) — 기체를 수평으로 유지한 채 1.9 m 넘게 들어 올린다(기울이면 오버틸트 컷).
- 테더는 착지 높이까지 느슨하게.
- [ ] 문서 → layout 통과 → 커밋 `docs(flow): describe the range-based auto-land and its bench steps`

### Task 7: 실기 확인 (사용자와 함께)

- [ ] 프롭 제거·무장, 위 절차로 `Hover_Valid=1` → 0.6–1.0 m로 들고 패드 분리 → `Land_Mode=거리`, 목표 고도 하강, 천천히 내려 바닥에 놓으면 접촉 감소 뒤 `CUT_LANDED`
- [ ] 들고 있는 중 기체를 수평으로 유지한 채 1.9 m 넘게 들어 올려 → `TIMED_FALLBACK`(센서를 기울여 돌리면 60° 오버틸트 컷이 먼저 난다)
- [ ] 바닥 근접 원값 61–99 mm(예: 80 mm)에서 멈춰 들고 있으면 `RL_TOUCHDOWN_MAX_MS` 뒤 그 자리 컷(프롭 제거 상태). 10–60 mm 는 착지 대역이라 정체 시험이 아니다
- [ ] 테더 비행: 0.5 m 안팎에서 RC 상실 → 착지 컷, 줄은 착지 높이까지 느슨하게
- [ ] 결과·로그를 원장과 노션 기록 페이지에, 주력 머지 판단은 사용자에게

## 자체 검토

- 설계 대응: 4.1 → Task 4 진입, 4.2 → Task 1·2·4, 4.3 → Task 1·4, 4.4 → Task 1·4, 4.5 → Task 4, 5 → Task 5, 6 → Global Constraints, 7 → Task 1·3·4·7.
- 독립 검토 기록:
  1. **직접 재계산(Claude).** 상수(`FLOW_RANGE_MIN_M` 0.10/`MAX` 1.80, `ALTHOLD_SUSPEND_S` 0.30, `out_max_us` 60, `ALTHOLD_SLEW_US_S` 300, `FS_MAX_MS` 3000, `FS_RESUME_MAX_MS` 9000, `FLOW_HEIGHT_MAX_AGE_US` 150 ms, 91필드)를 소스에서 확인. 결함: 원값 60–100 mm 구간이 착지 직전 폴백을 일으킴 → 바닥 근접 정의.
  2. **Codex 적대적 검토(P1 3·P2 5).** 원값 대역만으로는 접촉 구분 불가(P1) → 연속성 + 접촉 감소 + 신선도 조건. 착지 타이머가 오래된 값으로 완주(P1) → 신선도 상실 시 리셋. 하강 권한 부족(P1: 감쇠항 +18 µs vs 위치항 −9 µs, 호버 +20 µs 에서 12 s 뒤 0.94 m) → 목표 속도 추종 + 하강 이득 + 편향·처짐 SIL. 지상 컷 경계 inhibit(P2) → 거리 모드에서 미적용. 폴백·resume 스로틀 상승(P2) → collective 상한·collective 기준 시험. PosHold OVERRIDE·리셋(P2) → 조종 입력 0·적분 인계. 폴백 시간 예산(P2) → 스펙 정정(한 번의 예산 3 s, 총 상한 명시). SIL 재시작(P2) → hook 주입 규칙.
  2-1. **Codex 재검토(P1 2·P2 5).** 중간값 하나로 연속성 우회(P1) → 프레임 간 높이 변화 한도·추적 유지 시간. 밀린 FIFO 프레임을 신선하게 취급(P1) → `range_timing_ok=false` 프레임은 증거·무효 모두에서 제외. collective 단조성·resume 계단(P2) → collective 래칫, resume 300 µs/s 인계. 폴백 기준값이 다음 tick 에 복귀(P2) → 에피소드 내내 `fallback_cap`. resume 재진입 시 총 상한(P2) → 연속 에피소드 기준 명시·시험. 근접 컷을 CUT_LANDED 로 보냄(P2) → CUT_TIMEOUT + `Land_Cut_Reason` 필드. 8 cm 테더 시험 좌표(P2) → `tether_floor=0.05` m.
  2-2. **Codex 재검토(P1 1·P2 2).** 지속된 점프 값이 새 추적 기준이 됨(P1: 1500→200 mm 600 ms 유지→130→60 mm 로 LANDED) → 거부된 프레임은 기준이 되지 않고 거부 지속이 무효 타이머, 허용 차이는 시간에 비례. 래칫이 allocator 앞(P2) → 보장 대상을 요청 collective 로 명시, allocator 자세 몫은 기존 우선순위 유지(Claude 결정), 외란 SIL 로 한정. resume 인계 중 재상실이 지상 컷(P2) → 인계 중엔 `max(base, hover_est)` 로 진입 판정.
  2-3. **Codex 재검토(P1 1·P2 2, allocator 자세 우선 유지엔 동의).** 시간에 따라 커지는 허용폭이 거부 값을 결국 채택(P1: 300 mm 뒤 90 mm 반복이 280 ms 에 채택) → 허용폭은 첫 도착 공백으로 고정, 기준은 명령 하강 속도로 예측. 음수 원값이 공간 조건 통과(P2) → `raw > 0` 필수, 무효 타이머는 채택으로만 해제. 접촉 증거가 base 만 봄(P2: 요청 1260 vs 실제 1335) → allocator 실제 전달 collective 감소 필요, 외란 시험에 물리 합격 기준.
  2-4. **Codex 재검토(P1 1).** 고정 허용폭이어도 예측 중심이 명령 하강으로 계속 내려가 계단형 이상값(200→100→40 mm)을 채택 → 거부 중 예측 중심도 첫 도착 시각에 고정, 계단 시험 추가.
  2-5. **Codex 수렴(2026-11-03):** "남은 P1/P2 및 새 P1/P2 없음" — 조건식 재생에서 계단형·지속 점프·음수 입력 거부 후 300 ms 폴백, 결손·잡음 정상 하강 거부 0건. 범위는 스펙·계획 검토와 조건식 재생이며 구현·SIL·컴파일·실기는 미검증.
  3. **agy(Gemini 4 Argon) 독립 검토(결론 비공개).** 근접 물체·테더 줄 공중 오판 → 연속성. 데드존 −1 로 바닥에서 3 s 폴백 → 바닥 근접 뒤 컷. 호버 편향·배터리 처짐·지면 효과 → SIL 케이스. 접지 순간 위치 유지 기울기 → `hold_scale` 감쇠. Core 0 지연으로 거짓 폴백 → 원값 신선도는 도착 시각. 61–99 mm 걸림 → `RL_TOUCHDOWN_MAX_MS` 컷. 9 s resume 누적 제한·요청 해제는 기존 규칙 유지(명시). 벤치 절차(호버 유효 조건, 손 가림 금지) → Task 6·7.
