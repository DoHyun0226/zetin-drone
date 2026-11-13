"""드론 UDP 텔레메트리와 CSV 스키마의 공용 parser.

앞 10필드는 legacy 펌웨어와 공통이다. 앞 14필드는
firmware/archive/legacy_flight/dual_imu_pid_pwm이 쓰던 형식을 유지하고, 15-21필드는
firmware/archive/legacy_flight/dual_imu_cascade_pwm이 더한 진단값이다. 22필드
(Armed)는 펌웨어 safety-lock 상태를 알려서 지상 도구가 거부되거나 무시된 start를
알아챌 수 있게 하고, 23-30필드는 Tier 1 관측값(모터 출력, 실측 루프 속도,
바깥 루프 목표 각속도)이다. 31필드는 기울기 보상한 BMM350 자기 heading을 더하고,
32-34필드는 그 계산에 쓴 보상된 body 좌표계 자기장 XYZ 성분을 더한다. 35필드는
yaw heading hold가 켜져 있는지 알린다. 36-38필드는 failsafe phase와 펌웨어가
적용한 roll/pitch trim을 더한다. 39-40필드는 추정 hover 스로틀과 그 유효 플래그를
더한다. 41-43필드는 failsafe active-probe 상태, 연속 무응답 횟수, 가장 최근
차분 응답(g)을 더한다. probe 상태 값은 0=WAIT, 1=DIP, 2=EVALUATE,
3=UNAVAILABLE, 4=BLOCKED(샘플링 구간 동안 mixer가 요청한 dip의 80% 이상을
내지 못함)이다. 44-55필드는 IMU1과 IMU2의 gyro(dps)와 accel(g) XYZ 값을 융합
필드와 같은 body 좌표계로 더한다. IMU별 gyro는 소프트웨어 LPF 뒤에서 샘플링하고,
IMU별 accel에는 소프트웨어 LPF가 없다. 한 제어 tick 안에서 ``Active_IMUs == 2``이고
``Fault_Disagree == 0``이면 융합 gyro/accel 축은 해당 IMU1과 IMU2 축의 평균이다.
IMU별 값 12개는 같은 snapshot에서 나오지만, ASCII 텔레메트리 패킷 전체는 한
제어 tick의 원자적 snapshot이 아니다. 다른 필드는 다른 tick에서 올 수 있으므로,
행마다 반올림 오차 안에서 같다는 것은 텔레메트리가 보장하지 않는다.
56-58필드는 roll, pitch, yaw 목표 각도를 더한다. 정상 비행에서
``targetAngleX``와 ``targetAngleY``는 ``constrain(command + trim, ...)``이므로
이 setpoint에는 이미 ``Trim_Roll``/``Trim_Pitch``가 들어 있다. trim을 다시 더하면
두 번 센다. 하강 중(``Failsafe_Phase == 1``)에는 보고되는 목표에 ``trim_roll``,
``trim_pitch``, ``fs_hold_yaw``가 들어 있다: 수평 하강 setpoint와 유지하는 heading이다.
``TgtAngle_Yaw``는 ``yawOuter.target_angle_deg``다. ``Yaw_Hold == 1``이면 유지하는
heading이고, ``Yaw_Hold == 0``이면 현재 heading을 따라간다.
59필드는 펌웨어의 실제 ``Mag_Enabled`` 융합 상태를 알린다. 60-64필드는
Matek 3901-L0X 측정값을 더한다: ``Range_MM``은 모듈의 raw 거리이고
**범위를 벗어나면 음수**다 -- 동작 범위가 80-2000 mm라서 가까운 한계보다 작은
값도 음수가 되므로, 부호만으로는 지면이 너무 가까운지 너무 먼지 알 수 없다.
``Range_Quality``와 ``Flow_Quality``는 모듈이 주는 0-255이거나, **새 프레임이
오지 않았으면 -1**이다. 멈춘 센서와 유효한 측정을 가르는 방법은 이것뿐이다.
65필드는 ``Mag_Cal_Active``를 더한다. 이 값이 1인 동안 ``Mag_X/Y/Z``는
``ground.magcal``이 고르는 보정 전 BMM350 raw µT 값이다. 아니면 이 세 필드는
보정된 값이라는 비행 때의 뜻을 그대로 가진다. ``MagHeading``(31필드)은 자기 융합이
꺼져 있는 동안 마지막 값을 유지하므로, 융합이 켜져 있는지 판단할 수 없다.
그 상태의 유일한 근거는 ``Mag_Enabled``다.
66-75필드는 allocator scale/collective, 그 제어 tick에 쓴 PID 항과 적분 항,
yaw-authority 상태(0=NORMAL, 1=LIMITED, 2=RECOVERING)를 더한다.
76-91필드(0부터 센 인덱스 75-90)는 ``firmware/flight/zetin_fc``만 보낸다.
옛 75필드 펌웨어(archive의 dual_imu_cascade_pwm) 패킷은 75필드에서 끝나므로
이 필드는 ``None``으로 남는다. ``Flow_Reject``는 마지막 flow 구간이 쓸 만했으면
0이고, 아니면 ``flow_odometry.h``의 ``FlowReject`` 코드다. ``Flow_Vel_R``/``Flow_Vel_P``와
``Pos_R``/``Pos_P``는 hold의 보정 공간 값이다. 이 공간에서 양의 값은 양의
roll/pitch 변화로 보정된다. 이 펌웨어의 규약(+pitch = 기수 내림, +roll = 오른쪽
날개 내림)에서 ``Flow_Vel_P``는 전진 속도의 음수이고 ``Flow_Vel_R``은 오른쪽 속도의
음수다. ``Pos_*``는 같은 공간에서 hold 기준점으로부터의 body 좌표계 offset이다
(앞으로 가면 ``Pos_P``가 음수가 된다). ``Hold_Roll_Deg``/``Hold_Pitch_Deg``와
``AltHold_US``는 *후보* 보정값으로, disarm 중에도 계산한다(dry run). 실제로
적용되는지는 ``Armed``와 ``Failsafe_Phase``를 따른다. ``PosHold_State``는
0=OFF, 1=ACTIVE, 2=COAST, 3=SUSPENDED, 4=OVERRIDE이고, ``AltHold_State``는
0=OFF, 1=ACTIVE, 2=ADJUST, 3=SUSPENDED이고, ``FlowCal_State``는 0=IDLE,
1=RUNNING, 2=DONE_OK, 3=DONE_FAILED다. ``Height_M``은 무효일 때 -1이다.
92-96필드(인덱스 91-95)는 거리 기반 자동착륙을 나타내며, 역시 zetin_fc만 보낸다:
``Land_Mode`` 0=NONE, 1=RANGE, 2=TIMED, 3=TIMED_FALLBACK,
``Land_Stage`` 0=IDLE, 1=DESCENT, 2=APPROACH, 3=NEAR_GROUND, 4=CONTACT,
``Land_Ref_M`` 목표 높이, ``Land_Cut_Reason`` 0=NONE, 1=CONTACT,
2=NEAR_GROUND_LOSS, 3=NEAR_GROUND_STUCK, 4=BACKSTOP, ``Land_Req_US`` allocator에
요청한 collective(``Mixer_Collective_US``에서 이 값을 뺀 것이 allocator가 자세
제어 여유를 지키려고 더한 양이다). ``Failsafe_Phase``는 거리 기반 하강과 시간 기반
하강 모두 DESCENDING이다. 97필드(인덱스 96) ``RangeLand_On``은 수락된
``rangeland`` 스위치다(1=RC 손실 때 거리 기반 하강 허용, 0=시간 기반만. 기본 0,
disarm 중에만 바뀐다). 이 필드가 없는 패킷은 ``None``으로 남으며, 이는 OFF와
같지 않다.
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
    "Land_Mode",
    "Land_Stage",
    "Land_Ref_M",
    "Land_Cut_Reason",
    "Land_Req_US",
    "RangeLand_On",
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
    "Land_Mode": int,
    "Land_Stage": int,
    "Land_Ref_M": float,
    "Land_Cut_Reason": int,
    "Land_Req_US": int,
    "RangeLand_On": int,
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
    """legacy, 옛 75필드 또는 zetin_fc(91/96/97필드) 패킷을 파싱한다.

    짧은 패킷에 없는 필드는 ``None``으로 돌려준다. 그래서 CSV 파일은 정상/fault
    값을 지어내지 않고 전체 헤더를 유지한다. 알려진 97필드 뒤의 미래 추가 필드는
    무시한다. 앞 ``REQUIRED_FIELD_COUNT``개 필드는 비어 있으면 안 된다: 쓰는 쪽이
    이 값으로 서식을 만들고 계산을 하므로, 여기가 비어 있으면 legacy 패킷이 아니라
    깨진 패킷이다.
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
    """``line``이 gain readback datagram인지 돌려준다."""

    return line.strip().startswith("GAINS,")


def parse_gains_packet(line):
    """gain readback datagram을 이름 붙은 float 값 12개로 파싱한다."""

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
    """``line``이 flow 보정 결과 datagram인지 돌려준다."""

    return line.strip().startswith("FLOWCAL,")


def parse_flowcal_packet(line):
    """``FLOWCAL,ok,reason,m00,m01,m10,m11,delay_us,residual,cond,corr,samples``를 파싱한다.

    ``flowcal 0``이 끝나면 ``zetin_fc``가 한 번 보낸다.
    ``m``은 2x2 counts/rad 행렬(body roll/pitch 회전당 센서 카운트)이다. 펌웨어는
    붙여 넣을 상수 줄도 출력한다.
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
    """``CSV_FIELDS``에 맞춘 행을 돌려준다. 모르는 legacy 값은 빈칸으로 둔다."""

    return [timestamp] + [
        "" if sample[name] is None else sample[name] for name in TELEMETRY_FIELDS
    ]


def active_fault_names(sample):
    """지금 켜져 있는 fault 필드의 짧은 이름을 돌려준다."""

    fields = (
        ("Fault_RC", "RC"),
        ("Fault_Critical", "CRIT"),
        ("Fault_IMU1", "IMU1"),
        ("Fault_IMU2", "IMU2"),
        ("Fault_Disagree", "DISAGREE"),
        ("Fault_Attitude", "TILT"),
    )
    return [label for name, label in fields if sample.get(name) == 1]
