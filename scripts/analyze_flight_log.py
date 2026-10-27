import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from failsafe_telemetry import format_failsafe_phase, phase_number


SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR = SCRIPT_DIR.parent / "logs"


_ARGS = [arg for arg in sys.argv[1:] if not arg.startswith("-")]
# 벤치에서 로그만 빠르게 훑을 때 창이 뜨면 터미널이 블로킹된다.
NO_PLOT = "--no-plot" in sys.argv[1:]


def select_log_file():
    if _ARGS:
        selected = Path(_ARGS[0]).expanduser().resolve()
        if not selected.is_file():
            raise FileNotFoundError(f"지정한 CSV 파일이 없습니다: {selected}")
        return selected

    candidates = list(LOG_DIR.glob("*.csv"))
    if not candidates:
        raise FileNotFoundError(f"CSV 파일이 없습니다: {LOG_DIR}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


try:
    file_path = select_log_file()
    print(f"📂 분석 대상: {file_path}")
    df = pd.read_csv(file_path, skipinitialspace=True)
    df.columns = df.columns.str.strip()
    if df.empty:
        raise ValueError("파일이 비어 있습니다")
except (OSError, ValueError, pd.errors.ParserError) as exc:
    print(f"❌ 로그 로딩 실패: {exc}")
    sys.exit(1)

# 과거 실험 로그에서 사용했을 가능성이 있는 이름을 현재 스키마로 정규화한다.
column_aliases = {
    "Fault_IMU": "Fault_Critical",
    "Scaled": "Mixer_Scaled",
    "scaled": "Mixer_Scaled",
    "Active_Imus": "Active_IMUs",
    "active_imus": "Active_IMUs",
}
for old_name, new_name in column_aliases.items():
    if old_name in df.columns and new_name not in df.columns:
        df.rename(columns={old_name: new_name}, inplace=True)

for column in df.columns:
    if column != "Timestamp":
        df[column] = pd.to_numeric(df[column], errors="coerce")

print(f"✅ 데이터 로딩 완료! ({len(df)}개 샘플)")

cols_attitude = ["Roll", "Pitch", "Yaw"]
cols_gyro = ["Gyro_X", "Gyro_Y", "Gyro_Z"]
cols_accel = ["Accel_X", "Accel_Y", "Accel_Z"]
cols_control = ["Throttle", "Active_IMUs", "Mixer_Scaled"]
cols_trim = ["Trim_Roll", "Trim_Pitch"]
cols_fault = [
    "Fault_RC",
    "Fault_Critical",
    "Fault_IMU1",
    "Fault_IMU2",
    "Fault_Disagree",
    "Fault_Attitude",
    "Calibration_OK",
]

summary_columns = [
    column
    for column in cols_attitude + cols_gyro + cols_accel + cols_control + cols_trim
    if column in df.columns and df[column].notna().any()
]

print("\n" + "=" * 72)
print("📊 비행 데이터 요약 통계")
print("=" * 72)
if summary_columns:
    print(df[summary_columns].describe().round(3))
else:
    print("분석할 수치 열이 없습니다.")


def asserted_samples(column, invert=False):
    if column not in df.columns:
        return None
    valid = df[column].dropna()
    if valid.empty:
        return None
    asserted = valid <= 0 if invert else valid > 0
    return int(asserted.sum()), len(valid)


print("\n🔎 제어·고장 이벤트")
scaled_result = asserted_samples("Mixer_Scaled")
if scaled_result is not None:
    count, total = scaled_result
    print(f"  Mixer scaled: {count}/{total} 샘플 ({count / total * 100:.2f}%)")

if "Active_IMUs" in df.columns and df["Active_IMUs"].notna().any():
    active = df["Active_IMUs"].dropna()
    degraded = int((active < 2).sum())
    unavailable = int((active <= 0).sum())
    print(
        f"  Active IMUs: 최소 {active.min():.0f}, "
        f"degraded {degraded}샘플, unavailable {unavailable}샘플"
    )

for column in cols_fault:
    result = asserted_samples(column, invert=(column == "Calibration_OK"))
    if result is None:
        continue
    count, total = result
    label = "Calibration_Fail" if column == "Calibration_OK" else column
    print(f"  {label}: {count}/{total} 샘플")

if all(column in df.columns for column in ("RC_Total_Pkts", "RC_Dropped_Pkts")):
    total_series = df["RC_Total_Pkts"].dropna()
    dropped_series = df["RC_Dropped_Pkts"].dropna()
    if not total_series.empty and not dropped_series.empty:
        total = int(total_series.iloc[-1])
        dropped = int(dropped_series.iloc[-1])
        rate = dropped / total * 100 if total > 0 else 0.0
        print(f"  RC packet drops: {dropped}/{total} ({rate:.2f}%)")


def latest_number(column):
    if column not in df.columns:
        return None
    valid = df[column].dropna()
    return None if valid.empty else float(valid.iloc[-1])


trim_roll = latest_number("Trim_Roll")
trim_pitch = latest_number("Trim_Pitch")
if trim_roll is None and trim_pitch is None:
    print("  Trim: legacy/unknown")
else:
    roll_text = "unknown" if trim_roll is None else f"{trim_roll:+.2f}°"
    pitch_text = "unknown" if trim_pitch is None else f"{trim_pitch:+.2f}°"
    print(f"  Trim: Roll {roll_text}, Pitch {pitch_text}")
print("=" * 72)


def timestamp_seconds(value):
    parsed = pd.to_timedelta(str(value), errors="coerce")
    if pd.isna(parsed):
        return None
    return parsed.total_seconds()


def _valid_numeric(frame, column):
    if column not in frame.columns:
        return None
    valid = frame[column].dropna()
    valid = valid[valid.map(math.isfinite)]
    return None if valid.empty else valid


def _yaw_authority_number(value):
    state = phase_number(value)
    return state if state in (0, 1, 2) else None


def _absolute_axis_maxima(frame, columns):
    maxima = []
    for column in columns:
        valid = _valid_numeric(frame, column)
        if valid is None:
            return None
        maxima.append(float(valid.abs().max()))
    return maxima


print("\n🎛️ Allocator and yaw authority")
rp_scale = _valid_numeric(df, "Mixer_RP_Scale")
yaw_scale = _valid_numeric(df, "Mixer_Yaw_Scale")
if rp_scale is None or yaw_scale is None:
    print("  Allocator scale: legacy/unknown")
else:
    print(
        "  Allocator scale: "
        f"RP min {rp_scale.min():.3f}, p05 {rp_scale.quantile(0.05):.3f}; "
        f"Yaw min {yaw_scale.min():.3f}, p05 {yaw_scale.quantile(0.05):.3f}"
    )

authority = (
    None
    if "Yaw_Authority_State" not in df.columns
    else df["Yaw_Authority_State"].map(_yaw_authority_number).dropna()
)
if authority is None or authority.empty:
    print("  Yaw authority LIMITED: legacy/unknown")
else:
    entries = 0
    cumulative_seconds = 0.0
    previous_entry_state = None
    previous_timed_state = None
    previous_timed_seconds = None
    previous_clock_seconds = None
    day_offset = 0.0
    has_timed_sample = False
    incomplete_timing = False
    for index, raw_state in df["Yaw_Authority_State"].items():
        state = _yaw_authority_number(raw_state)
        if state is None:
            previous_entry_state = None
        else:
            if state == 1 and previous_entry_state != 1:
                entries += 1
            previous_entry_state = state

        raw_seconds = (
            timestamp_seconds(df.at[index, "Timestamp"])
            if "Timestamp" in df.columns
            else None
        )
        seconds = raw_seconds
        if raw_seconds is not None:
            seconds = raw_seconds + day_offset
            if (
                previous_clock_seconds is not None
                and seconds < previous_clock_seconds - 43200.0
            ):
                day_offset += 86400.0
                seconds += 86400.0
            previous_clock_seconds = seconds

        if state is None or seconds is None:
            incomplete_timing = True
            previous_timed_state = None
            previous_timed_seconds = None
            continue

        has_timed_sample = True
        if previous_timed_state == 1 and previous_timed_seconds is not None:
            cumulative_seconds += max(0.0, seconds - previous_timed_seconds)
        previous_timed_state = state
        previous_timed_seconds = seconds

    prefix = f"  Yaw authority LIMITED: {entries} entries, "
    if not has_timed_sample:
        print(prefix + "duration unavailable")
    elif incomplete_timing:
        print(
            prefix
            + f"{cumulative_seconds:.3f} s cumulative "
            "(incomplete: unknown state/time gap)"
        )
    else:
        print(prefix + f"{cumulative_seconds:.3f} s cumulative")

pid_maxima = _absolute_axis_maxima(
    df, ("PID_Roll_US", "PID_Pitch_US", "PID_Yaw_US")
)
if pid_maxima is None:
    print("  PID |max| (us): legacy/unknown")
else:
    print(
        "  PID |max| (us): "
        f"Roll {pid_maxima[0]:.3f}, Pitch {pid_maxima[1]:.3f}, "
        f"Yaw {pid_maxima[2]:.3f}"
    )

i_maxima = _absolute_axis_maxima(
    df, ("I_Roll_US", "I_Pitch_US", "I_Yaw_US")
)
if i_maxima is None:
    print("  I-term |max| (us): legacy/unknown")
else:
    print(
        "  I-term |max| (us): "
        f"Roll {i_maxima[0]:.3f}, Pitch {i_maxima[1]:.3f}, "
        f"Yaw {i_maxima[2]:.3f}"
    )


def collect_failsafe_analysis(frame):
    if "Failsafe_Phase" not in frame.columns:
        return [], [], [], False

    valid_samples = []
    day_offset = 0.0
    previous_seconds = None
    for index, raw_phase in frame["Failsafe_Phase"].items():
        phase = phase_number(raw_phase)
        if phase is None:
            continue

        timestamp = (
            str(frame.at[index, "Timestamp"])
            if "Timestamp" in frame.columns
            else str(index)
        )
        seconds = timestamp_seconds(timestamp)
        if seconds is not None:
            seconds += day_offset
            if previous_seconds is not None and seconds < previous_seconds - 43200.0:
                day_offset += 86400.0
                seconds += 86400.0
            previous_seconds = seconds
        valid_samples.append(
            {
                "index": int(index),
                "timestamp": timestamp,
                "seconds": seconds,
                "phase": phase,
            }
        )

    transitions = []
    for previous, current in zip(valid_samples, valid_samples[1:]):
        if previous["phase"] != current["phase"]:
            transitions.append(
                {
                    "index": current["index"],
                    "timestamp": current["timestamp"],
                    "from": previous["phase"],
                    "to": current["phase"],
                }
            )

    episodes = []
    descent_start = None
    for sample in valid_samples:
        if sample["phase"] == 1 and descent_start is None:
            descent_start = sample
        elif sample["phase"] != 1 and descent_start is not None:
            duration = None
            if descent_start["seconds"] is not None and sample["seconds"] is not None:
                duration = sample["seconds"] - descent_start["seconds"]
            episodes.append(
                {
                    "start": descent_start,
                    "end": sample,
                    "duration": duration,
                    "terminal_phase": sample["phase"],
                }
            )
            descent_start = None

    had_descent = any(sample["phase"] == 1 for sample in valid_samples)
    return valid_samples, transitions, episodes, had_descent


phase_samples, phase_transitions, auto_land_episodes, had_descent = (
    collect_failsafe_analysis(df)
)

print("\n🛬 Failsafe phase transitions")
if not phase_samples:
    print("  Failsafe_Phase: 전이 없음 (legacy/unknown)")
elif not phase_transitions:
    print(
        "  Failsafe_Phase: 전이 없음 "
        f"(전체 {format_failsafe_phase(phase_samples[0]['phase'])})"
    )
else:
    print(f"  {'시각':<15} {'샘플':>7}  전이")
    for transition in phase_transitions:
        print(
            f"  {transition['timestamp']:<15} {transition['index']:>7}  "
            f"{format_failsafe_phase(transition['from'])} → "
            f"{format_failsafe_phase(transition['to'])}"
        )

for episode_number, episode in enumerate(auto_land_episodes, start=1):
    prefix = "" if len(auto_land_episodes) == 1 else f" #{episode_number}"
    if episode["duration"] is None:
        print(f"  자동착륙{prefix} 소요 시간: 계산 불가")
    else:
        print(f"  자동착륙{prefix} 소요 시간: {episode['duration']:.3f}초")
    print(
        f"  종료 phase: {format_failsafe_phase(episode['terminal_phase'])}"
    )

if had_descent and not auto_land_episodes:
    print("  자동착륙: 로그 종료 시점에도 DESCENDING (종료 phase 없음)")

plt.style.use(
    "seaborn-v0_8-darkgrid"
    if "seaborn-v0_8-darkgrid" in plt.style.available
    else "default"
)

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(13, 15), sharex=True)
fig.suptitle(f"Flight Analysis: {file_path.name}", fontsize=16, fontweight="bold")
x_axis = pd.RangeIndex(len(df))

for column, color, style in (
    ("Roll", "red", "-"),
    ("Pitch", "blue", "-"),
    ("Yaw", "green", "--"),
):
    if column in df.columns:
        ax1.plot(x_axis, df[column], label=column, color=color, linestyle=style, linewidth=1.3)
ax1.axhline(0, color="black", linestyle=":", alpha=0.5)
ax1.set_ylabel("Angle (deg)")
ax1.set_title("1. Attitude Response")
ax1.set_ylim(-60, 60)
ax1.legend(loc="upper right")

for column, color in (
    ("Gyro_X", "red"),
    ("Gyro_Y", "blue"),
    ("Gyro_Z", "green"),
):
    if column in df.columns:
        ax2.plot(x_axis, df[column], label=column, color=color, alpha=0.65, linewidth=0.9)
ax2.set_ylabel("Angular rate (dps)")
ax2.set_title("2. Gyroscope (Vibration Check)")
ax2.legend(loc="upper right")

for column, color in (
    ("Accel_X", "red"),
    ("Accel_Y", "blue"),
    ("Accel_Z", "green"),
):
    if column in df.columns:
        ax3.plot(x_axis, df[column], label=column, color=color, alpha=0.55, linewidth=0.9)
ax3.set_ylabel("Acceleration (g)")
ax3.set_ylim(-2.0, 2.0)
ax3.set_title("3. Accelerometer & Throttle")
ax3.legend(loc="upper left")

ax3_right = ax3.twinx()
if "Throttle" in df.columns:
    ax3_right.plot(x_axis, df["Throttle"], label="Throttle", color="orange", linewidth=1.5)
    throttle_valid = df["Throttle"].dropna()
    if not throttle_valid.empty and throttle_valid.max() <= 100:
        throttle_label = "Throttle (%)"
        ax3_right.set_ylim(0, 100)
    else:
        throttle_label = "Throttle (PWM)"
        ax3_right.set_ylim(1000, 2000)
else:
    throttle_label = "Throttle"
ax3_right.set_ylabel(throttle_label, color="orange")
ax3_right.tick_params(axis="y", labelcolor="orange")
ax3_right.legend(loc="upper right")

event_rows = (
    ("Mixer_Scaled", "Mixer scaled", False, "tab:orange"),
    ("Fault_RC", "RC fault", False, "tab:red"),
    ("Fault_Critical", "Critical fault", False, "darkred"),
    ("Fault_IMU1", "IMU1 fault", False, "tab:purple"),
    ("Fault_IMU2", "IMU2 fault", False, "tab:pink"),
    ("Fault_Disagree", "IMU disagree", False, "tab:brown"),
    ("Fault_Attitude", "Attitude fault", False, "black"),
    ("Calibration_OK", "Calibration fail", True, "tab:gray"),
)

event_labels = []
event_positions = []
for column, label, invert, color in event_rows:
    if column not in df.columns or not df[column].notna().any():
        continue
    asserted = df[column].le(0) if invert else df[column].gt(0)
    position = len(event_labels)
    event_labels.append(label)
    event_positions.append(position)
    event_x = df.index[asserted.fillna(False)]
    if len(event_x):
        ax4.scatter(event_x, [position] * len(event_x), marker="|", s=90, color=color)

if event_labels:
    ax4.set_yticks(event_positions, event_labels)
    ax4.set_ylim(-0.7, len(event_labels) - 0.3)
else:
    ax4.text(
        0.5,
        0.5,
        "Legacy log: no extended fault/status fields",
        ha="center",
        va="center",
        transform=ax4.transAxes,
    )
    ax4.set_yticks([])

ax4_active = ax4.twinx()
if "Active_IMUs" in df.columns and df["Active_IMUs"].notna().any():
    ax4_active.step(
        x_axis,
        df["Active_IMUs"],
        where="post",
        color="tab:blue",
        linewidth=1.5,
        label="Active IMUs",
    )
    ax4_active.legend(loc="upper right")
ax4_active.set_ylabel("Active IMUs", color="tab:blue")
ax4_active.set_ylim(-0.1, 2.2)
ax4_active.set_yticks([0, 1, 2])
ax4.set_title("4. Saturation, Redundancy & Fault Events")
ax4.set_xlabel("Sample count")
ax4.grid(True, axis="x", alpha=0.4)

if "Failsafe_Phase" in df.columns and df["Failsafe_Phase"].notna().any():
    descending = df["Failsafe_Phase"].eq(1).fillna(False)
    start = None
    for index, active in descending.items():
        if active and start is None:
            start = int(index)
        elif not active and start is not None:
            for axis in (ax1, ax2, ax3, ax4):
                axis.axvspan(start - 0.5, int(index) - 0.5, color="gold", alpha=0.16)
            start = None
    if start is not None:
        for axis in (ax1, ax2, ax3, ax4):
            axis.axvspan(start - 0.5, len(df) - 0.5, color="gold", alpha=0.16)

fig.tight_layout(rect=(0, 0, 1, 0.97))
if not NO_PLOT:
    plt.show()
