# Low-Throttle Yaw Authority Implementation Plan

> **For implementers:** REQUIRED SUB-SKILL: Follow strict RED→GREEN TDD. Run the
> named test before production edits and record the expected failure, then make
> the minimum implementation pass. Do not flash hardware, push, or modify files
> outside this worktree.

**Goal:** Preserve roll/pitch control authority when a fixed tether yaw disturbance
exceeds available motor torque during manual low-throttle landing, without changing
normal unsaturated mixer output.

**Architecture:** Replace the uniform attitude scaler with a pure two-stage control
allocator that gives the combined roll/pitch vector first claim on motor span and
fits yaw into the residual span. Feed its per-axis scales into conditional integrator
anti-windup and a manual-flight yaw authority state machine. Append observability to
the existing telemetry contract and prove the failure mode in native SIL.

**Tech Stack:** ESP32 Arduino C++, C++17 native tests with 32-bit `g++`/QEMU shims,
Python 3 `unittest`, `arduino-cli`, CSV telemetry tools.

**Spec:** `docs/superpowers/specs/2026-08-18-low-throttle-yaw-authority-design.md`

## Global Constraints

- Work only in the isolated `feat/low-throttle-yaw-control` worktree.
- Preserve all unrelated user changes in the original checkout.
- No board upload, motor motion, push, merge, or PR.
- Use `/home/light/anaconda3/bin/python`, not system Python.
- Production changes require a test that was observed failing for the intended reason.
- Preserve motor order FL/RR/FR/RL and the existing roll, pitch, yaw signs.
- Unsaturated inputs must produce the same integer motor outputs and collective as
  the pre-change `mixAndDesaturate()`.
- RP has priority over yaw; yaw may not reduce RP unless RP alone exceeds the span.
- Do not change PID gains, `CTRL_MARGIN`, motor limits, dynamic idle, failsafe timing,
  failsafe throttle, or cut conditions.
- The yaw authority state machine is active only for armed manual flight with
  `Failsafe_Phase=0`.
- Telemetry is append-only: keep fields 1-65 and append exactly the ten spec fields
  in order. Legacy packets remain parseable.
- Keep the 1 kHz PID task as the only motor writer.
- Each task must commit only its named scope and record commands, exit codes, RED,
  GREEN, and self-review in its SDD report.

## Task 1: Priority control allocator

**Files:**

- Create: `firmware/flight/dual_imu_cascade_pwm/control_allocator.h`
- Create: `tools/native_tests/test_control_allocator.cpp`
- Create: `tools/test_control_allocator.py`
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`
- Modify: `tools/native_tests/test_control_math.cpp`

### Step 1: Write allocator behavior tests

Create a standalone native test for the hand-derived cases in spec §7.1. Include:

- zero output and pure roll/pitch/yaw outputs with literal expected motors
- unsaturated combined input equivalent to the old formula
- yaw-only saturation where RP motor-pair differences remain exactly unchanged,
  `rp_scale=1`, and `0≤yaw_scale<1`
- RP-only saturation where `rp_scale<1`
- positive/negative yaw symmetry
- collective shift before scaling
- finite extreme commands, invalid bounds, and motor range invariants

Name the production mutation each case catches in a test comment. Do not compute the
expected output through production helpers.

### Step 2: Verify RED

Run:

```bash
/home/light/anaconda3/bin/python tools/test_control_allocator.py -v
```

Expected: compilation fails because `control_allocator.h` or its API is absent. Record
the exact exit code and relevant diagnostic in the report.

### Step 3: Implement the minimum allocator

Implement `ControlAllocation` and a pure allocation function per spec §3. Use the
RP vector, solve the maximum feasible yaw scale from motor-pair span constraints,
move collective, then round and constrain motors.

### Step 4: Verify GREEN and integrate the sketch

Run the standalone test until green. Replace the `.ino` local mixer implementation
with the new header and output fields. Update the existing whole-sketch control-math
tests so they verify integration rather than duplicate the standalone algorithm.

Run:

```bash
/home/light/anaconda3/bin/python tools/test_control_allocator.py -v
/home/light/anaconda3/bin/python tools/test_native_control_math.py -v
```

### Step 5: Negative proof and commit

Temporarily mutate the allocator to uniform RP/yaw scaling or swap priority. Confirm
at least the yaw-only saturation test fails, then restore and rerun both commands.
Commit only Task 1 files with message:

```text
feat: prioritize attitude axes in motor allocator
```

## Task 2: Axis-selective anti-windup and yaw authority state

**Files:**

- Create: `firmware/flight/dual_imu_cascade_pwm/yaw_authority.h`
- Create: `tools/native_tests/test_yaw_authority.cpp`
- Create: `tools/test_yaw_authority.py`
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`
- Modify: `tools/native_tests/test_control_math.cpp`
- Modify: `tools/native_tests/test_yaw_command.cpp`

### Step 1: Write RED tests for anti-windup

Test a pure conditional integration helper with literal expected values:

- full authority integrates normally
- limited authority blocks a delta that increases the signed requested output
- limited authority permits a delta that reduces the signed requested output
- clamp at `I_TERM_MAX_US`
- throttle at or below 1100 µs resets to zero
- yaw limitation does not stop roll/pitch integration in the sketch integration test

### Step 2: Write RED tests for the state machine

Test spec §5 boundaries with synthetic millisecond timestamps:

- NORMAL before 150 ms and LIMITED exactly after sustained entry duration
- transient scale recovery clears the entry timer
- LIMITED→RECOVERING and 500 ms recovery hysteresis
- recovery failure returns to LIMITED
- pilot yaw rate input immediately resets/bypasses the state
- failsafe, disarm, and safety lock reset to NORMAL
- LIMITED/RECOVERING yields a slaved current heading and 0°/s yaw-rate target
- yaw override cannot bypass a limited state
- timestamp subtraction remains correct across `uint32_t` wrap

### Step 3: Verify RED

Run:

```bash
/home/light/anaconda3/bin/python tools/test_yaw_authority.py -v
```

Expected: compilation fails because the header/API is absent. Record evidence.

### Step 4: Implement and wire GREEN

Implement the pure helper and state machine. Wire it so the previous allocation scale
drives the next control tick. In LIMITED/RECOVERING and centered stick, bypass normal
heading hold, slave the target angle to current yaw, and command 0°/s. Preserve pilot
rate input and the failsafe `fs_hold_yaw` path.

Apply axis scale separately to conditional integration. Reset state and diagnostics
on locked/disarmed paths. Do not move the motor writer.

Run:

```bash
/home/light/anaconda3/bin/python tools/test_yaw_authority.py -v
/home/light/anaconda3/bin/python tools/test_yaw_command.py -v
/home/light/anaconda3/bin/python tools/test_native_control_math.py -v
/home/light/anaconda3/bin/python tools/test_failsafe_land.py -v
```

### Step 5: Negative proof and commit

Mutate the 150 ms entry gate or allow yaw override to bypass LIMITED. Confirm the
corresponding test fails, restore, and rerun. Commit Task 2 files with:

```text
feat: degrade yaw hold when authority is exhausted
```

## Task 3: Append allocator observability end to end

**Files:**

- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`
- Modify: `scripts/telemetry_schema.py`
- Modify: `scripts/control_dualsense.py`
- Modify: `scripts/analyze_flight_log.py`
- Modify: `tools/test_telemetry_schema.py`
- Modify: `tools/test_control_dualsense.py`
- Create or modify: analyzer tests under `tools/`

### Step 1: Write schema and presentation RED tests

Extend expected wire order with exactly fields 66-75 from spec §6. Add tests that:

- parse a 75-field packet with correct float/int types
- keep 65-field and older packets valid with new values `None`
- write a 76-column CSV row including timestamp
- render RP scale, yaw scale, and authority state in the armed diagnostic line
- render missing extended values as `-`
- reject malformed required values without weakening legacy behavior

Run the targeted schema and ground-station tests and record the expected failures.

### Step 2: Write analyzer RED tests

Use a small synthetic CSV fixture. Assert a consumer-visible summary of:

- RP/Yaw scale minimum and lower percentile
- LIMITED entry count and cumulative duration
- PID and I-term absolute maxima
- graceful unknown output for a legacy CSV without the new columns

### Step 3: Implement append-only telemetry

Snapshot and send the ten new values in exact order. Preserve all first 65 fields and
existing `Mixer_Scaled` meaning. Extend Python schema/types, the armed status formatter,
and analyzer without requiring the new fields in legacy packets.

### Step 4: Verify GREEN and negative proof

Run:

```bash
/home/light/anaconda3/bin/python tools/test_telemetry_schema.py -v
/home/light/anaconda3/bin/python tools/test_control_dualsense.py -v
/home/light/anaconda3/bin/python -m unittest discover -s tools -p 'test_*flight_log*.py' -v
```

If the analyzer test has a different focused filename, run that exact file instead of
the discovery pattern and record it. Temporarily swap fields 66 and 67; the wire-order
test must fail. Restore and rerun.

Commit Task 3 files with:

```text
feat: expose axis allocation telemetry
```

## Task 4: Low-throttle SIL regression and documentation

**Files:**

- Modify: `tools/native_tests/test_sil_attitude.cpp`
- Modify: `tools/test_sil_attitude.py` only if the wrapper needs a timeout/report change
- Modify: `docs/udp_protocol.md`
- Modify: `scripts/README.md`
- Modify: `docs/power_on_bench_procedure.md`
- Modify: `docs/presentations/ai-startup-camp-drone/SOURCES.md`

### Step 1: Add the SIL RED scenario

Add a deterministic scenario that establishes 1300 µs normal flight, descends to
1270 µs, then applies a constant positive yaw torque beyond available yaw authority
plus a smaller pitch torque. Assert observable behavior, not source text:

- yaw allocation limits before RP
- RP remains unscaled while its own demand fits
- the authority state reaches LIMITED
- after LIMITED, target yaw rate does not remain at ±180°/s for 0.5 seconds
- maximum pitch error is lower than an explicit uniform-scaling mutation/oracle run
- motor limits, PID loop cadence, safety and failsafe invariants remain valid

Run the SIL against the pre-scenario production behavior and record a failure caused by
the missing behavior, not by an unstable fixture.

### Step 2: Make the minimum integration changes

Only adjust the SIL harness or production integration if the RED test exposes a real
gap. Do not tune PID gains or widen acceptance merely to obtain GREEN.

Run:

```bash
/home/light/anaconda3/bin/python tools/test_sil_attitude.py -v
```

### Step 3: Update current-state documentation

Document 75 telemetry fields, 76 CSV columns, field meanings, state values, and the
manual bench/tether gates from spec §8. Keep optical-flow absence and manual position
correction explicit. Do not rewrite dated historical result sections; add a dated
correction or current procedure section where necessary.

### Step 4: Negative proof and commit

Run the SIL with the allocator priority mutation. Confirm the new scenario fails,
restore, and rerun. Commit Task 4 files with:

```text
test: cover low-throttle yaw saturation in SIL
```

## Task 5: Full clean verification

**Files:**

- Modify only files required to fix failures demonstrably caused by Tasks 1-4
- Do not expand behavior or tune control constants

### Step 1: Review the spec against the final diff

Check every invariant and excluded item. Confirm no unrelated file, generated binary,
log, presentation media, or worktree artifact is tracked.

### Step 2: Run the full host suite

```bash
MPLCONFIGDIR=/tmp/zetin-low-throttle-final-mpl \
PYTHONPYCACHEPREFIX=/tmp/zetin-low-throttle-final-pycache \
/home/light/anaconda3/bin/python -m unittest discover \
  -s tools -p 'test_*.py' -v

/home/light/anaconda3/bin/python tools/check_repo_layout.py
git diff --check
```

Record discovered/run/pass/fail/skip counts and each exit code.

### Step 3: Compile the actual ESP32-S3 sketch

```bash
XDG_CACHE_HOME=/tmp/zetin-low-throttle-arduino-cache \
arduino-cli compile --warnings all \
  --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-low-throttle-arduino-build \
  firmware/flight/dual_imu_cascade_pwm
```

Record exit code and warnings separately. A host compile is not board, motor, or flight
proof.

### Step 4: Final mutation proof

In a temporary copy or reversible patch, replace priority allocation with uniform
scaling. Run allocator and SIL tests and record that both reject the mutation. Restore
the exact production tree and rerun those tests GREEN.

### Step 5: Commit verification-only fixes

If no code changes were needed, do not make an empty commit. If a failure required a
scoped fix, follow RED→GREEN and commit only that fix with:

```text
fix: close low-throttle yaw verification gaps
```

The task report must state clearly that flashing and physical validation remain unrun.
