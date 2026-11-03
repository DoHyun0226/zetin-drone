"""Formatting helpers for failsafe telemetry consumers."""

import math


FAILSAFE_PHASE_NAMES = {
    0: "NONE",
    1: "DESCENDING",
    2: "CUT_LANDED",
    3: "CUT_TIMEOUT",
    4: "CUT_ABORT",
}

FAILSAFE_PROBE_STATE_NAMES = {
    0: "WAIT",
    1: "DIP",
    2: "EVALUATE",
    3: "UNAVAILABLE",
    4: "BLOCKED",
}


def phase_number(value):
    """Return an integer phase, or ``None`` for absent/malformed values."""

    if value is None:
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(numeric) or not numeric.is_integer():
        return None
    return int(numeric)


def format_failsafe_phase(value):
    """Return a human-readable enum label while retaining the wire value."""

    phase = phase_number(value)
    if phase is None:
        return "legacy/unknown"
    return f"{FAILSAFE_PHASE_NAMES.get(phase, 'UNKNOWN')} ({phase})"


def format_failsafe_probe_state(value):
    """Return a human-readable probe-state label while retaining the wire value."""

    state = phase_number(value)
    if state is None:
        return "legacy/unknown"
    return f"{FAILSAFE_PROBE_STATE_NAMES.get(state, 'UNKNOWN')} ({state})"


def _format_trim(value):
    if value is None:
        return "unknown"
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "unknown"
    if not math.isfinite(numeric):
        return "unknown"
    return f"{numeric:+.2f}°"


def _format_hover_status(sample):
    values = (sample.get("Hover_Est"), sample.get("Hover_Valid"))
    if all(value is None for value in values):
        return ""

    try:
        hover_est = float(values[0])
    except (TypeError, ValueError):
        hover_est = math.nan
    hover_text = "-" if not math.isfinite(hover_est) else f"{hover_est:.1f}µs"
    hover_valid = phase_number(values[1])
    valid_text = "-" if hover_valid is None else str(hover_valid)
    return f" | Hover {hover_text}, valid {valid_text}"


def _format_probe_status(sample):
    values = (
        sample.get("Failsafe_Probe_State"),
        sample.get("Failsafe_Probe_NoResponse"),
        sample.get("Failsafe_Probe_Response_G"),
    )
    if all(value is None for value in values):
        return ""

    state = phase_number(values[0])
    state_text = format_failsafe_probe_state(state)
    count = phase_number(values[1])
    count_text = "-" if count is None else str(count)
    try:
        response = float(values[2])
    except (TypeError, ValueError):
        response = math.nan
    response_text = "-" if not math.isfinite(response) else f"{response:.4f}g"
    prefix = "⚠ Probe" if state == 4 else "Probe"
    suffix = " ⚠" if state == 4 else ""
    return (
        f" | {prefix} {state_text}{suffix}, "
        f"no-response {count_text}, response {response_text}"
    )


def format_monitor_status(sample):
    """Return status text and whether the monitor must use alert styling."""

    phase = phase_number(sample.get("Failsafe_Phase"))
    probe_state = phase_number(sample.get("Failsafe_Probe_State"))
    alert = (phase is not None and phase != 0) or probe_state == 4
    prefix = "AUTO-LAND ACTIVE — " if alert else ""
    text = (
        f"{prefix}Failsafe {format_failsafe_phase(phase)} | "
        f"Trim R {_format_trim(sample.get('Trim_Roll'))} / "
        f"P {_format_trim(sample.get('Trim_Pitch'))}"
        f"{_format_hover_status(sample)}"
        f"{_format_probe_status(sample)}"
    )
    return text, alert


def render_monitor_title(axis, system_summary, sample):
    """Render the monitor title and apply an alert box for nonzero phases."""

    failsafe_text, alert = format_monitor_status(sample)
    title = axis.set_title(f"{system_summary}\n{failsafe_text}")
    if alert:
        title.set_color("white")
        title.set_fontweight("bold")
        title.set_bbox(
            {
                "boxstyle": "round,pad=0.35",
                "facecolor": "crimson",
                "edgecolor": "darkred",
                "alpha": 0.95,
            }
        )
    return title
