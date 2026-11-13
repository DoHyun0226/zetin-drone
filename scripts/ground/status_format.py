"""콘솔 상태 줄과 대시보드가 같이 쓰는 순수 값 formatter.

표준 라이브러리와 ground.failsafe_telemetry에만 의존한다. 그래서
prompt_toolkit이 설치되지 않아도 control_station이 import할 수 있다.
"""

import math

from ground.failsafe_telemetry import phase_number

IMU_TELEMETRY_FIELDS = (
    "IMU1_Gyro_X", "IMU1_Gyro_Y", "IMU1_Gyro_Z",
    "IMU1_Accel_X", "IMU1_Accel_Y", "IMU1_Accel_Z",
    "IMU2_Gyro_X", "IMU2_Gyro_Y", "IMU2_Gyro_Z",
    "IMU2_Accel_X", "IMU2_Accel_Y", "IMU2_Accel_Z",
)

POSHOLD_STATE_NAMES = ("OFF", "ACTIVE", "COAST", "SUSPENDED", "OVERRIDE")
ALTHOLD_STATE_NAMES = ("OFF", "ACTIVE", "ADJUST", "SUSPENDED")

# ---------------------------------------------------------------------------
# 콘솔 상태 줄과 같이 쓰는 값 formatter
# ---------------------------------------------------------------------------
def _format_optional_float(value, decimal_places):
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(numeric):
        return "-"
    return f"{numeric:.{decimal_places}f}"


def _format_agl(sample):
    """3901-L0X 거리계. 신선하지 않으면 '-', 범위 밖이면 'OOR'.

    Range_MM 은 모듈 원값이라 범위 밖에서 음수다(8cm 미만도 음수). 그래서
    거리 숫자로 그대로 보여주면 안 되고, Range_Quality 로 신선도를 먼저 본다.
    """
    quality = sample.get("Range_Quality")
    distance = sample.get("Range_MM")
    if quality is None or distance is None:
        return "-"
    if quality < 0:
        return "-"
    if distance < 0:
        return f"OOR/q{quality}"
    return f"{distance / 1000.0:.2f}m/q{quality}"


def _format_angle_error(target, measured):
    try:
        error = float(target) - float(measured)
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(error):
        return "-"
    return f"{error:+.1f}"


def _format_optional_int(value):
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(numeric):
        return "-"
    if numeric.is_integer():
        return str(int(numeric))
    return f"{numeric:.1f}"


def _format_binary_flag(value):
    flag = phase_number(value)
    return str(flag) if flag in (0, 1) else "-"


def _format_yaw_authority(value):
    state = phase_number(value)
    if state is None:
        return "-"
    name = {
        0: "NORMAL",
        1: "LIMITED",
        2: "RECOVERING",
    }.get(state, "UNKNOWN")
    return f"{name}({state})"


def _state_name(names, value):
    if value is None:
        return "-"
    return names[value] if 0 <= value < len(names) else str(value)


def _format_flow_hold_status(sample):
    """zetin_fc만 보낸다. 옛 75필드 펌웨어에서는 이 필드들이 None이다."""
    if sample.get("PosHold_State") is None:
        return ""
    return (
        f"PosHold={_state_name(POSHOLD_STATE_NAMES, sample.get('PosHold_State'))} "
        f"AltHold={_state_name(ALTHOLD_STATE_NAMES, sample.get('AltHold_State'))} "
        f"H={_format_optional_float(sample.get('Height_M'), 2)} "
    )


def _format_gyro_disagreement(sample):
    if any(sample.get(name) is None for name in IMU_TELEMETRY_FIELDS):
        return "-"
    difference = max(
        abs(sample[f"IMU1_Gyro_{axis}"] - sample[f"IMU2_Gyro_{axis}"])
        for axis in ("X", "Y", "Z")
    )
    return f"{difference:.1f}"
