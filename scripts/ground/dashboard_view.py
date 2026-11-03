"""Dashboard view model: snapshot -> styled screen lines.

Pure functions only. Importing this module must not touch the control station
(its import binds the UDP port and opens log files). The value formatters shared
with the console lines live in ground.status_format (no prompt_toolkit).
"""

import math

from prompt_toolkit.utils import get_cwidth

from ground.failsafe_telemetry import (
    FAILSAFE_PHASE_NAMES,
    FAILSAFE_PROBE_STATE_NAMES,
    phase_number,
)
from ground.status_format import (
    ALTHOLD_STATE_NAMES,
    POSHOLD_STATE_NAMES,
    _format_agl,
    _format_angle_error,
    _format_binary_flag,
    _format_gyro_disagreement,
    _format_optional_float,
    _format_optional_int,
    _format_yaw_authority,
    _state_name,
)
from ground.telemetry_schema import active_fault_names

# Same values as control_station.TELEM_TIMEOUT_SEC / TRIM_TOLERANCE.
STALE_AFTER_SEC = 1.5
TRIM_MATCH_TOLERANCE = 0.01
COMPACT_WIDTH = 80
COMPACT_HEIGHT = 24


# ---------------------------------------------------------------------------
# Link rate (the snapshot only has counters; the caller feeds this tracker)
# ---------------------------------------------------------------------------
class LinkRateTracker:
    """Packets/s from successive (time, counter) pairs."""

    def __init__(self):
        self._last = None

    def update(self, now, count):
        previous, self._last = self._last, (now, count)
        if previous is None:
            return None
        elapsed = now - previous[0]
        delta = count - previous[1]
        if elapsed <= 0 or delta < 0:
            return None
        return delta / elapsed


# ---------------------------------------------------------------------------
# Line building
# ---------------------------------------------------------------------------
def _clip(segments, width):
    """Cut a row of (style, text) pieces to ``width`` terminal cells."""
    out = []
    used = 0
    for style, text in segments:
        if not text:
            continue
        cells = get_cwidth(text)
        if used + cells <= width:
            out.append((style, text))
            used += cells
            continue
        piece = ""
        for char in text:
            char_cells = get_cwidth(char)
            if used + char_cells > width:
                break
            piece += char
            used += char_cells
        if piece:
            out.append((style, piece))
        break
    return out


class _Row:
    def __init__(self, value_style):
        self.segments = []
        self.value_style = value_style

    def label(self, text):
        self.segments.append(("class:dim", text + " "))
        return self

    def kv(self, name, value, style=None):
        self.segments.append(("class:dim", name + " "))
        self.segments.append((style or self.value_style, f"{value}  "))
        return self

    def text(self, text, style=None):
        self.segments.append((style or self.value_style, text))
        return self


def _valid_height(value):
    """Height_M is -1 when invalid (telemetry_schema); show only real heights."""
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    return numeric if math.isfinite(numeric) and numeric >= 0 else None


def _signed(value, decimals, suffix=""):
    text = _format_optional_float(value, decimals)
    if text == "-":
        return text
    try:
        return f"{float(value):+.{decimals}f}{suffix}"
    except (TypeError, ValueError):
        return "-"


def _on_off(value):
    flag = _format_binary_flag(value)
    return {"1": "ON", "0": "OFF"}.get(flag, "-")


def _top_bar(snap, sample, now, stale, age):
    segments = []
    if snap.get("arm_state") == "arming":
        segments.append(("class:warn", " ARMING "))
    elif snap.get("is_armed"):
        segments.append(("class:armed", " ARMED "))
    else:
        segments.append(("class:disarmed", " DISARMED "))

    if snap.get("last_telem_time", 0.0) <= 0:
        segments.append(("class:warn", "  링크 ○ 수신 대기"))
    elif stale:
        segments.append(("class:alert", f"  링크 ○ 수신 없음 {age:.1f}s"))
    else:
        hz = snap.get("link_hz")
        hz_text = "-" if hz is None else f"{hz:.1f}"
        segments.append(("class:ok", "  링크 ● "))
        segments.append(("class:value", f"{hz_text} Hz"))
    alerts = []
    if sample:
        if sample.get("Fault_RC") == 1:
            alerts.append("RC 타임아웃")
        if sample.get("Fault_Critical") == 1:
            alerts.append("치명 고장")
        if snap.get("is_armed") and sample.get("Armed") == 0 and not stale:
            alerts.append("시동 불일치")
    if snap.get("worker_fault"):
        label = {"telemetry": "수신", "controller": "조종"}.get(
            snap["worker_fault"], str(snap["worker_fault"]))
        alerts.append(f"{label} 스레드 오류")
    for alert in alerts:
        segments.append(("class:alert", f"  ! {alert}"))
    total = snap.get("telem_total_pkts") or 0
    dropped = snap.get("telem_dropped_pkts") or 0
    drop = f"{dropped / total * 100:.1f}%" if total > 0 else "-"
    segments.append(("class:value", f"  드롭 {drop}"))
    bad = snap.get("bad_packet_count") or 0
    segments.append(("class:warn" if bad else "class:dim", f"  불량 {bad}"))

    since = snap.get("armed_since")
    if snap.get("is_armed") and since is not None:
        seconds = max(0, int(now - since))
        segments.append(("class:value", f"  비행 {seconds // 60:02d}:{seconds % 60:02d}"))
    log_path = snap.get("log_path")
    if log_path is not None:
        segments.append(("class:dim", f"  로그 {getattr(log_path, 'name', log_path)}"))

    return segments


def render(snap, width, height, now):
    """Return the top bar and panels as lines of ``(style, text)`` pieces.

    ``height`` is the number of rows this view may use; the caller draws the
    event list and input line below. Takes no locks and never mutates ``snap``.
    """
    sample = snap.get("last_sample")
    has_sample = isinstance(sample, dict)
    data = sample if has_sample else {}
    last_telem = snap.get("last_telem_time") or 0.0
    age = now - last_telem if last_telem > 0 else None
    stale = (not has_sample) or age is None or age > STALE_AFTER_SEC
    value_style = "class:dim" if stale else "class:value"
    compact = width < COMPACT_WIDTH or height < COMPACT_HEIGHT
    listen = snap.get("mode") == "listen"

    lines = [_top_bar(snap, data, now, stale, age), [("class:dim", "─" * width)]]

    def row(label):
        return _Row(value_style).label(label + " " * max(0, 9 - get_cwidth(label)))

    # 자세
    r = row("자세")
    r.kv("Roll", _signed(data.get("Roll"), 2, "°"))
    r.kv("Pitch", _signed(data.get("Pitch"), 2, "°"))
    r.kv("Yaw", _signed(data.get("Yaw"), 1, "°"))
    r.kv("Yaw_Hold", _on_off(data.get("Yaw_Hold")))
    r.kv("Mag", _on_off(data.get("Mag_Enabled")))
    lines.append(r.segments)

    # 목표
    r = row("목표")
    r.kv("Roll", _signed(data.get("TgtAngle_Roll"), 2, "°"))
    r.kv("Pitch", _signed(data.get("TgtAngle_Pitch"), 2, "°"))
    r.kv("Yaw rate", _signed(data.get("TgtRate_Yaw"), 1, "°/s"))
    r.kv("eR", _format_angle_error(data.get("TgtAngle_Roll"), data.get("Roll")))
    r.kv("eP", _format_angle_error(data.get("TgtAngle_Pitch"), data.get("Pitch")))
    lines.append(r.segments)

    # 고도·추력
    r = row("고도·추력")
    h_m = _valid_height(data.get("Height_M"))
    if data.get("Height_M") is not None:
        r.kv("H", "-" if h_m is None else f"{h_m:.2f} m")
    r.kv("AGL", _format_agl(data))
    r.kv("Throttle", _format_optional_int(data.get("Throttle")))
    if not listen:
        r.kv("패드", _format_optional_int(snap.get("current_throttle")))
    r.kv("Hover_Est", _format_optional_float(data.get("Hover_Est"), 1))
    valid = phase_number(data.get("Hover_Valid"))
    r.text({1: "valid", 0: "invalid"}.get(valid, "-"))
    lines.append(r.segments)

    # 모터 / 믹서
    motors = [_format_optional_int(data.get(f"Motor_M{i}")) for i in range(1, 5)]
    if compact:
        r = row("모터")
        r.text(" ".join(motors) + "  ")
        r.kv("RP", _format_optional_float(data.get("Mixer_RP_Scale"), 2))
        r.kv("Yaw", _format_optional_float(data.get("Mixer_Yaw_Scale"), 2))
        lines.append(r.segments)
    else:
        r = row("모터")
        for name, motor in zip(("M1 FL", "M2 RR", "M3 FR", "M4 RL"), motors):
            r.kv(name, motor)
        try:
            # Same definition as the console [DIAG] line: (M1+M2) - (M3+M4).
            cw_ccw = (float(data["Motor_M1"]) + float(data["Motor_M2"])
                      - float(data["Motor_M3"]) - float(data["Motor_M4"]))
            if not math.isfinite(cw_ccw):
                raise ValueError
            r.kv("CW-CCW", f"{cw_ccw:+.0f}")
        except (KeyError, TypeError, ValueError):
            r.kv("CW-CCW", "-")
        lines.append(r.segments)

        r = row("믹서")
        r.kv("RP", _format_optional_float(data.get("Mixer_RP_Scale"), 2))
        r.kv("Yaw", _format_optional_float(data.get("Mixer_Yaw_Scale"), 2))
        r.kv("Yaw권한", _format_yaw_authority(data.get("Yaw_Authority_State")))
        r.kv("dG", _format_gyro_disagreement(data))
        if snap.get("raw_capture_enabled"):
            r.kv("Raw", _format_raw(snap, now))
        lines.append(r.segments)

    # 트림 + failsafe + faults
    r = row("트림")
    r.kv("Roll", _signed(snap.get("trim_roll"), 1))
    r.kv("Pitch", _signed(snap.get("trim_pitch"), 1))
    r.kv("드론", _drone_trim(snap, data))
    faults = ",".join(active_fault_names(data)) or "-"
    phase = phase_number(data.get("Failsafe_Phase"))
    in_failsafe = phase not in (None, 0)
    if not in_failsafe:
        r.kv("Failsafe", "-")
    r.kv("Faults", faults, "class:alert" if faults != "-" and not stale else None)
    lines.append(r.segments)

    # flow-hold panel (flow firmware only)
    if data.get("PosHold_State") is not None:
        r = row("위치")
        r.kv("PosHold", _state_name(POSHOLD_STATE_NAMES, data.get("PosHold_State")))
        r.kv("AltHold", _state_name(ALTHOLD_STATE_NAMES, data.get("AltHold_State")))
        r.kv("Vel R/P", f"{_signed(data.get('Flow_Vel_R'), 2)}/"
                        f"{_signed(data.get('Flow_Vel_P'), 2)}")
        r.kv("Vz", _signed(data.get("Vz_MS"), 2))
        lines.append(r.segments)

    # failsafe panel: loud while auto-landing
    if in_failsafe:
        name = FAILSAFE_PHASE_NAMES.get(phase, f"UNKNOWN({phase})")
        style = "class:alert"
        fr = _Row(style).label("Failsafe")
        fr.text(f"{name} ({phase})  ", style)
        probe = phase_number(data.get("Failsafe_Probe_State"))
        if probe is not None:
            probe_name = FAILSAFE_PROBE_STATE_NAMES.get(probe, f"UNKNOWN({probe})")
            fr.kv("Probe", probe_name, style)
            fr.kv("NoResp", _format_optional_int(data.get("Failsafe_Probe_NoResponse")),
                  style)
            fr.kv("Resp_G", _format_optional_float(data.get("Failsafe_Probe_Response_G"), 4),
                  style)
        lines.append(fr.segments)

    return [_clip(line, width) for line in lines[:max(0, height)]]


def _drone_trim(snap, sample):
    drone_roll = sample.get("Trim_Roll")
    drone_pitch = sample.get("Trim_Pitch")
    if drone_roll is None or drone_pitch is None:
        return "-"
    mismatch = (
        abs(drone_roll - snap.get("trim_roll", 0.0)) > TRIM_MATCH_TOLERANCE
        or abs(drone_pitch - snap.get("trim_pitch", 0.0)) > TRIM_MATCH_TOLERANCE
    )
    return f"{drone_roll:+.1f}/{drone_pitch:+.1f}{'!' if mismatch else ''}"


def _format_raw(snap, now):
    last = snap.get("raw_last_receive_time") or 0.0
    if last <= 0 or now - last > 1.0:
        return "-"
    return f"{snap.get('raw_batch_count', 0)}b/{snap.get('raw_last_dropped', 0)}d"
