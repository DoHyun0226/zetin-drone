"""Shared parser for the drone UDP telemetry and CSV schema.

The first 10 fields are common to legacy firmware. The first 14 fields retain
the format used by firmware/archive/legacy_flight/dual_imu_pid_pwm, fields
15-21 are diagnostics added by firmware/flight/dual_imu_cascade_pwm, field 22
(Armed) reports the firmware safety-lock state so ground tools can detect a
refused/ignored start, and fields 23-30 are Tier 1 observability (motor
outputs, measured loop rate, and outer-loop target rates). Field 31 appends the
tilt-compensated BMM350 magnetic heading, and fields 32-34 append the
compensated body-frame magnetic XYZ components used to calculate it. Field 35
reports whether yaw heading hold is active. Fields 36-38 append the failsafe
phase and the roll/pitch trim applied by the firmware. Fields 39-40 append the
estimated hover throttle and its validity flag. Fields 41-43 append the
failsafe active-probe state, consecutive no-response count, and most recent
differential response in g. Probe-state values are 0=WAIT, 1=DIP,
2=EVALUATE, 3=UNAVAILABLE, and 4=BLOCKED (the mixer did not deliver at least
80% of the requested dip during the sampling interval). Fields 44-55 append
IMU1 and IMU2 gyro (dps) and accel (g) XYZ values in the same body frame as
the fused fields. Per-IMU gyro is sampled after its software LPF; per-IMU
accel has no software LPF. Within one control tick, when ``Active_IMUs == 2``
and ``Fault_Disagree == 0``, the fused gyro/accel axes are the averages of the
corresponding IMU1 and IMU2 axes. The twelve per-IMU values share a snapshot,
but the complete ASCII telemetry packet is not an atomic control-tick
snapshot. Other fields may come from a different tick, so per-row equality
within rounding error is not a telemetry guarantee.
Fields 56-58 append the roll, pitch, and yaw target angles. In normal flight,
``targetAngleX`` and ``targetAngleY`` are ``constrain(command + trim, ...)``,
so these setpoints already include ``Trim_Roll``/``Trim_Pitch``; adding the
trim again double-counts it. During descent (``Failsafe_Phase == 1``), the
reported targets contain ``trim_roll``, ``trim_pitch``, and ``fs_hold_yaw``:
the level descent setpoint and held heading. ``TgtAngle_Yaw`` is
``yawOuter.target_angle_deg``; when ``Yaw_Hold == 1`` it is the held heading,
and when ``Yaw_Hold == 0`` it is slaved to the current heading.
Field 59 reports the firmware's actual ``Mag_Enabled`` fusion state. Fields
60-64 append the Matek 3901-L0X readings: ``Range_MM`` is the module's raw
distance and is **negative when out of range** -- its working range is
80-2000 mm, so a value below the near limit is negative too and the sign
alone cannot say whether the ground is too close or too far. ``Range_Quality``
and ``Flow_Quality`` are 0-255 from the module, or **-1 when no fresh frame
has arrived**, which is the only way to tell a frozen sensor from a valid
reading. Field 65 appends ``Mag_Cal_Active``. While it is 1, ``Mag_X/Y/Z``
are the uncalibrated BMM350 raw µT values selected by ``magcal_fit.py``;
otherwise those three fields retain their corrected-domain flight meaning.
``MagHeading`` (field 31) retains its last value while magnetic fusion is off,
so it cannot establish whether fusion is active; ``Mag_Enabled`` is the sole
source of truth for that state.
Fields 66-75 append allocator scale/collective, the PID and integral terms
used for that control tick, and yaw-authority state (0=NORMAL, 1=LIMITED,
2=RECOVERING).
Fields 76-91 (zero-based indices 75-90) are sent only by
``firmware/flight/dual_imu_cascade_flow_hold``; packets from the main cascade
firmware stop at 75 fields and leave them ``None``. ``Flow_Reject`` is 0 when
the last flow interval was usable and otherwise a ``FlowReject`` code from
``flow_odometry.h``. ``Flow_Vel_R``/``Flow_Vel_P`` and ``Pos_R``/``Pos_P`` are
in the hold's correction space, where a positive value is corrected by a
positive roll/pitch change. With this firmware's convention (+pitch = nose
down, +roll = right wing down) ``Flow_Vel_P`` is minus forward velocity and
``Flow_Vel_R`` is minus rightward velocity. ``Pos_*`` is the body-frame offset
from the hold reference in the same space (moving forward makes ``Pos_P``
negative). ``Hold_Roll_Deg``/``Hold_Pitch_Deg`` and ``AltHold_US`` are the
*candidate* corrections, computed even while disarmed (dry run); whether they
are applied follows ``Armed`` and ``Failsafe_Phase``. ``PosHold_State`` is
0=OFF, 1=ACTIVE, 2=COAST, 3=SUSPENDED, 4=OVERRIDE; ``AltHold_State`` is
0=OFF, 1=ACTIVE, 2=ADJUST, 3=SUSPENDED; ``FlowCal_State`` is 0=IDLE,
1=RUNNING, 2=DONE_OK, 3=DONE_FAILED. ``Height_M`` is -1 when invalid.
"""

import math


TELEMETRY_FIELDS = (
    "Roll",
    "Pitch",
    "Yaw",
    "Gyro_X",
    "Gyro_Y",
    "Gyro_Z",
    "Accel_X",
    "Accel_Y",
    "Accel_Z",
    "Throttle",
    "Fault_RC",
    "Fault_Critical",
    "RC_Total_Pkts",
    "RC_Dropped_Pkts",
    "Fault_IMU1",
    "Fault_IMU2",
    "Fault_Disagree",
    "Active_IMUs",
    "Mixer_Scaled",
    "Fault_Attitude",
    "Calibration_OK",
    "Armed",
    "Motor_M1",
    "Motor_M2",
    "Motor_M3",
    "Motor_M4",
    "PID_Loop_Hz",
    "TgtRate_Roll",
    "TgtRate_Pitch",
    "TgtRate_Yaw",
    "MagHeading",
    "Mag_X",
    "Mag_Y",
    "Mag_Z",
    "Yaw_Hold",
    "Failsafe_Phase",
    "Trim_Roll",
    "Trim_Pitch",
    "Hover_Est",
    "Hover_Valid",
    "Failsafe_Probe_State",
    "Failsafe_Probe_NoResponse",
    "Failsafe_Probe_Response_G",
    "IMU1_Gyro_X",
    "IMU1_Gyro_Y",
    "IMU1_Gyro_Z",
    "IMU1_Accel_X",
    "IMU1_Accel_Y",
    "IMU1_Accel_Z",
    "IMU2_Gyro_X",
    "IMU2_Gyro_Y",
    "IMU2_Gyro_Z",
    "IMU2_Accel_X",
    "IMU2_Accel_Y",
    "IMU2_Accel_Z",
    "TgtAngle_Roll",
    "TgtAngle_Pitch",
    "TgtAngle_Yaw",
    "Mag_Enabled",
    "Range_MM",
    "Range_Quality",
    "Flow_X",
    "Flow_Y",
    "Flow_Quality",
    "Mag_Cal_Active",
    "Mixer_RP_Scale",
    "Mixer_Yaw_Scale",
    "Mixer_Collective_US",
    "PID_Roll_US",
    "PID_Pitch_US",
    "PID_Yaw_US",
    "I_Roll_US",
    "I_Pitch_US",
    "I_Yaw_US",
    "Yaw_Authority_State",
    "Flow_Seq",
    "Flow_Dt_MS",
    "Flow_Reject",
    "Flow_Vel_R",
    "Flow_Vel_P",
    "Pos_R",
    "Pos_P",
    "Height_M",
    "Vz_MS",
    "PosHold_State",
    "Hold_Roll_Deg",
    "Hold_Pitch_Deg",
    "AltHold_State",
    "AltHold_US",
    "FlowCal_State",
    "FlowCal_Samples",
)

TELEMETRY_FIELD_TYPES = {
    "Roll": float,
    "Pitch": float,
    "Yaw": float,
    "Gyro_X": float,
    "Gyro_Y": float,
    "Gyro_Z": float,
    "Accel_X": float,
    "Accel_Y": float,
    "Accel_Z": float,
    "Throttle": int,
    "Fault_RC": int,
    "Fault_Critical": int,
    "RC_Total_Pkts": int,
    "RC_Dropped_Pkts": int,
    "Fault_IMU1": int,
    "Fault_IMU2": int,
    "Fault_Disagree": int,
    "Active_IMUs": int,
    "Mixer_Scaled": int,
    "Fault_Attitude": int,
    "Calibration_OK": int,
    "Armed": int,
    "Motor_M1": int,
    "Motor_M2": int,
    "Motor_M3": int,
    "Motor_M4": int,
    "PID_Loop_Hz": int,
    "TgtRate_Roll": float,
    "TgtRate_Pitch": float,
    "TgtRate_Yaw": float,
    "MagHeading": float,
    "Mag_X": float,
    "Mag_Y": float,
    "Mag_Z": float,
    "Yaw_Hold": int,
    "Failsafe_Phase": int,
    "Trim_Roll": float,
    "Trim_Pitch": float,
    "Hover_Est": float,
    "Hover_Valid": int,
    "Failsafe_Probe_State": int,
    "Failsafe_Probe_NoResponse": int,
    "Failsafe_Probe_Response_G": float,
    "IMU1_Gyro_X": float,
    "IMU1_Gyro_Y": float,
    "IMU1_Gyro_Z": float,
    "IMU1_Accel_X": float,
    "IMU1_Accel_Y": float,
    "IMU1_Accel_Z": float,
    "IMU2_Gyro_X": float,
    "IMU2_Gyro_Y": float,
    "IMU2_Gyro_Z": float,
    "IMU2_Accel_X": float,
    "IMU2_Accel_Y": float,
    "IMU2_Accel_Z": float,
    "TgtAngle_Roll": float,
    "TgtAngle_Pitch": float,
    "TgtAngle_Yaw": float,
    "Mag_Enabled": int,
    "Range_MM": int,
    "Range_Quality": int,
    "Flow_X": int,
    "Flow_Y": int,
    "Flow_Quality": int,
    "Mag_Cal_Active": int,
    "Mixer_RP_Scale": float,
    "Mixer_Yaw_Scale": float,
    "Mixer_Collective_US": float,
    "PID_Roll_US": float,
    "PID_Pitch_US": float,
    "PID_Yaw_US": float,
    "I_Roll_US": float,
    "I_Pitch_US": float,
    "I_Yaw_US": float,
    "Yaw_Authority_State": int,
    "Flow_Seq": int,
    "Flow_Dt_MS": float,
    "Flow_Reject": int,
    "Flow_Vel_R": float,
    "Flow_Vel_P": float,
    "Pos_R": float,
    "Pos_P": float,
    "Height_M": float,
    "Vz_MS": float,
    "PosHold_State": int,
    "Hold_Roll_Deg": float,
    "Hold_Pitch_Deg": float,
    "AltHold_State": int,
    "AltHold_US": float,
    "FlowCal_State": int,
    "FlowCal_Samples": int,
}

GAIN_FIELDS = (
    "Kp_Angle_Roll",
    "Kp_Angle_Pitch",
    "Kp_Angle_Yaw",
    "Kp_Rate_Roll",
    "Kp_Rate_Pitch",
    "Kp_Rate_Yaw",
    "Ki_Rate_Roll",
    "Ki_Rate_Pitch",
    "Ki_Rate_Yaw",
    "Kd_Rate_Roll",
    "Kd_Rate_Pitch",
    "Kd_Rate_Yaw",
)

CSV_FIELDS = ("Timestamp",) + TELEMETRY_FIELDS
REQUIRED_FIELD_COUNT = 10


def _parse_finite_float(raw, name):
    value = float(raw)
    if not math.isfinite(value):
        raise ValueError(f"{name} is not finite")
    return value


def _parse_integer(raw, name):
    value = _parse_finite_float(raw, name)
    if not value.is_integer():
        raise ValueError(f"{name} is not an integer")
    return int(value)


def parse_telemetry_packet(line):
    """Parse a legacy, main (75-field) or flow-hold (91-field) packet.

    Fields unavailable in shorter packets are returned as ``None`` so CSV
    files retain the full header without inventing healthy/fault values. Extra
    future fields are ignored after the known 91 fields. The first
    ``REQUIRED_FIELD_COUNT`` fields must be non-empty: consumers format and
    do arithmetic on them, so a blank there is a malformed packet, not a
    legacy one.
    """

    parts = [part.strip() for part in line.strip().split(",")]
    if len(parts) < REQUIRED_FIELD_COUNT:
        raise ValueError(
            f"telemetry has {len(parts)} fields; need at least {REQUIRED_FIELD_COUNT}"
        )

    sample = dict.fromkeys(TELEMETRY_FIELDS)
    for index, name in enumerate(TELEMETRY_FIELDS):
        if index >= len(parts):
            break
        raw = parts[index]
        if raw == "":
            if index < REQUIRED_FIELD_COUNT:
                raise ValueError(f"required field {name} is empty")
            continue
        if TELEMETRY_FIELD_TYPES[name] is float:
            sample[name] = _parse_finite_float(raw, name)
        else:
            sample[name] = _parse_integer(raw, name)
    return sample


def is_gains_packet(line):
    """Return whether ``line`` is a gain-readback datagram."""

    return line.strip().startswith("GAINS,")


def parse_gains_packet(line):
    """Parse a gain-readback datagram into its 12 named float values."""

    parts = [part.strip() for part in line.strip().split(",")]
    if len(parts) != len(GAIN_FIELDS) + 1:
        raise ValueError(
            f"gains has {len(parts) - 1} fields; need exactly {len(GAIN_FIELDS)}"
        )
    if parts[0] != "GAINS":
        raise ValueError("not a GAINS packet")
    return {
        name: _parse_finite_float(raw, name)
        for name, raw in zip(GAIN_FIELDS, parts[1:])
    }


def is_flowcal_packet(line):
    """Return whether ``line`` is a flow-calibration result datagram."""

    return line.strip().startswith("FLOWCAL,")


def parse_flowcal_packet(line):
    """Parse ``FLOWCAL,ok,reason,m00,m01,m10,m11,delay_us,residual,cond,corr,samples``.

    Sent once by ``dual_imu_cascade_flow_hold`` when ``flowcal 0`` finishes.
    ``m`` is the 2x2 counts/rad matrix (sensor counts per body roll/pitch
    rotation); the firmware also prints the constant lines to paste.
    """

    parts = [part.strip() for part in line.strip().split(",")]
    if len(parts) != 12 or parts[0] != "FLOWCAL":
        raise ValueError(f"FLOWCAL has {len(parts) - 1} fields; need 11")
    m = tuple(_parse_finite_float(raw, "m") for raw in parts[3:7])
    return {
        "ok": _parse_integer(parts[1], "ok") == 1,
        "reason": parts[2],
        "m": ((m[0], m[1]), (m[2], m[3])),
        "delay_us": _parse_integer(parts[7], "delay_us"),
        # 진단값은 실패 결과에서 inf/nan 일 수 있다(한 축만 흔들면 조건수 무한대).
        # 실패 이유를 보여 주는 것이 목적이므로 그대로 받는다.
        "residual": float(parts[8]),
        "cond": float(parts[9]),
        "corr": float(parts[10]),
        "samples": _parse_integer(parts[11], "samples"),
    }


def sample_to_csv_row(timestamp, sample):
    """Return a row matching ``CSV_FIELDS``; unknown legacy values stay blank."""

    return [timestamp] + [
        "" if sample[name] is None else sample[name] for name in TELEMETRY_FIELDS
    ]


def active_fault_names(sample):
    """Return concise names for currently asserted fault fields."""

    fields = (
        ("Fault_RC", "RC"),
        ("Fault_Critical", "CRIT"),
        ("Fault_IMU1", "IMU1"),
        ("Fault_IMU2", "IMU2"),
        ("Fault_Disagree", "DISAGREE"),
        ("Fault_Attitude", "TILT"),
    )
    return [label for name, label in fields if sample.get(name) == 1]
