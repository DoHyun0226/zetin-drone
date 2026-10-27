import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import telemetry_schema  # noqa: E402
from telemetry_schema import (  # noqa: E402
    CSV_FIELDS,
    TELEMETRY_FIELDS,
    parse_telemetry_packet,
    sample_to_csv_row,
)


def packet(field_count):
    values = [str(index + 0.5) for index in range(9)]
    values.extend(str(index) for index in range(9, field_count))
    return ",".join(values)


GAIN_NAMES = (
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

IMU_TELEMETRY_NAMES = (
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
)

TARGET_ANGLE_TELEMETRY_NAMES = (
    "TgtAngle_Roll",
    "TgtAngle_Pitch",
    "TgtAngle_Yaw",
)

ALLOCATION_TELEMETRY_NAMES = (
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
)

EXPECTED_TELEMETRY_FIELDS = (
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
    *IMU_TELEMETRY_NAMES,
    *TARGET_ANGLE_TELEMETRY_NAMES,
    "Mag_Enabled",
    "Range_MM",
    "Range_Quality",
    "Flow_X",
    "Flow_Y",
    "Flow_Quality",
    "Mag_Cal_Active",
    *ALLOCATION_TELEMETRY_NAMES,
)


class TelemetryCompatibilityTest(unittest.TestCase):
    def test_10_field_packet_keeps_extended_values_unknown(self):
        sample = parse_telemetry_packet(packet(10))
        self.assertEqual(9, sample["Throttle"])
        self.assertIsNone(sample["Fault_RC"])
        self.assertEqual(len(CSV_FIELDS), len(sample_to_csv_row("00:00:00.000", sample)))

    def test_14_field_packet_populates_legacy_fault_and_rc_fields(self):
        sample = parse_telemetry_packet(packet(14))
        self.assertEqual(13, sample["RC_Dropped_Pkts"])
        self.assertIsNone(sample["Fault_IMU1"])

    def test_21_field_packet_leaves_armed_unknown(self):
        sample = parse_telemetry_packet(packet(21))
        self.assertEqual(20, sample["Calibration_OK"])
        self.assertIsNone(sample["Armed"])
        for name in (
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
            "Hover_Est",
            "Hover_Valid",
            "Failsafe_Probe_State",
            "Failsafe_Probe_NoResponse",
            "Failsafe_Probe_Response_G",
        ):
            self.assertIsNone(sample[name])
        # CSV 는 필드 + Timestamp 다. 개수를 박으면 필드를 추가할 때마다
        # 무관한 이유로 깨지므로 관계로 고정한다.
        self.assertEqual(len(TELEMETRY_FIELDS) + 1, len(CSV_FIELDS))

    def test_22_field_packet_populates_armed(self):
        sample = parse_telemetry_packet(packet(22))
        self.assertEqual(21, sample["Armed"])
        self.assertIsNone(sample["Motor_M1"])
        self.assertIsNone(sample["PID_Loop_Hz"])

    def test_30_field_packet_leaves_mag_heading_unknown(self):
        sample = parse_telemetry_packet(packet(30))
        self.assertEqual(29.0, sample["TgtRate_Yaw"])
        self.assertIsNone(sample["MagHeading"])

    def test_31_field_packet_appends_mag_heading(self):
        line = ",".join(
            (
                "1.25", "-2.50", "3.75",
                "4.50", "-5.25", "6.00",
                "0.100", "-0.200", "1.000",
                "1100", "0", "1", "123", "4", "0", "1", "0", "1", "0", "0", "1",
                "1",
                "1101", "1102", "1103", "1104", "998",
                "12.50", "-23.75", "0.00",
                "87.25",
            )
        )

        sample = parse_telemetry_packet(line)

        self.assertEqual(1, sample["Armed"])
        self.assertIs(type(sample["Armed"]), int)
        for name, expected in (
            ("Motor_M1", 1101),
            ("Motor_M2", 1102),
            ("Motor_M3", 1103),
            ("Motor_M4", 1104),
            ("PID_Loop_Hz", 998),
        ):
            self.assertEqual(expected, sample[name])
            self.assertIs(type(sample[name]), int)
        for name, expected in (
            ("TgtRate_Roll", 12.5),
            ("TgtRate_Pitch", -23.75),
            ("TgtRate_Yaw", 0.0),
            ("MagHeading", 87.25),
        ):
            self.assertEqual(expected, sample[name])
            self.assertIs(type(sample[name]), float)
        for name in ("Mag_X", "Mag_Y", "Mag_Z"):
            self.assertIsNone(sample[name])

    def test_34_field_packet_appends_compensated_mag_xyz(self):
        line = ",".join(
            (
                "1.25", "-2.50", "3.75",
                "4.50", "-5.25", "6.00",
                "0.100", "-0.200", "1.000",
                "1100", "0", "1", "123", "4", "0", "1", "0", "1", "0", "0", "1",
                "1",
                "1101", "1102", "1103", "1104", "998",
                "12.50", "-23.75", "0.00",
                "87.25",
                "21.50", "-7.25", "42.00",
            )
        )

        sample = parse_telemetry_packet(line)

        for name, expected in (
            ("MagHeading", 87.25),
            ("Mag_X", 21.5),
            ("Mag_Y", -7.25),
            ("Mag_Z", 42.0),
        ):
            self.assertEqual(expected, sample[name])
            self.assertIs(type(sample[name]), float)
        row = sample_to_csv_row("00:00:00.000", sample)
        self.assertEqual(
            ["87.25", "21.5", "-7.25", "42.0", ""],
            [
                str(row[CSV_FIELDS.index(name)])
                for name in ("MagHeading", "Mag_X", "Mag_Y", "Mag_Z", "Yaw_Hold")
            ],
        )

    def test_35_field_packet_parses_yaw_hold(self):
        packet = ",".join(["1"] * 34 + ["1"])
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertEqual(sample["Yaw_Hold"], 1)

    def test_34_field_packet_leaves_yaw_hold_none(self):
        packet = ",".join(["1"] * 34)
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertIsNone(sample["Yaw_Hold"])

    def test_38_field_packet_parses_new_fields(self):
        packet = ",".join(["1"] * 35 + ["2", "1.5", "-2.5"])
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertEqual(sample["Failsafe_Phase"], 2)
        self.assertAlmostEqual(sample["Trim_Roll"], 1.5)
        self.assertAlmostEqual(sample["Trim_Pitch"], -2.5)
        self.assertIsNone(sample["Hover_Est"])
        self.assertIsNone(sample["Hover_Valid"])

    def test_40_field_packet_appends_hover_estimate_and_validity(self):
        packet = ",".join(["1"] * 38 + ["1337.5", "1"])
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertAlmostEqual(sample["Hover_Est"], 1337.5)
        self.assertIs(type(sample["Hover_Est"]), float)
        self.assertEqual(sample["Hover_Valid"], 1)
        self.assertIs(type(sample["Hover_Valid"]), int)
        self.assertIsNone(sample["Failsafe_Probe_State"])
        self.assertIsNone(sample["Failsafe_Probe_NoResponse"])
        self.assertIsNone(sample["Failsafe_Probe_Response_G"])

    def test_43_field_packet_still_parses_failsafe_probe_diagnostics(self):
        packet = ",".join(["1"] * 40 + ["3", "2", "0.0125"])
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertEqual(sample["Failsafe_Probe_State"], 3)
        self.assertIs(type(sample["Failsafe_Probe_State"]), int)
        self.assertEqual(sample["Failsafe_Probe_NoResponse"], 2)
        self.assertIs(type(sample["Failsafe_Probe_NoResponse"]), int)
        self.assertAlmostEqual(sample["Failsafe_Probe_Response_G"], 0.0125)
        self.assertIs(type(sample["Failsafe_Probe_Response_G"]), float)
        self.assertIsNone(sample["Mag_Enabled"])

    def test_55_field_packet_appends_per_imu_body_frame_values(self):
        imu_values = (
            1.25, -2.5, 3.75,
            0.125, -0.25, 0.5,
            -4.5, 5.25, -6.0,
            -0.625, 0.75, -0.875,
        )
        current_packet = ",".join(["1"] * 43 + [str(value) for value in imu_values])

        sample = parse_telemetry_packet(current_packet)

        for name, expected in zip(IMU_TELEMETRY_NAMES, imu_values):
            self.assertEqual(expected, sample[name])
            self.assertIs(type(sample[name]), float)

    def test_58_field_packet_appends_target_angles_with_float_values(self):
        target_angles = (5.25, -6.5, 179.75)
        current_packet = ",".join(
            ["1"] * 55 + [str(value) for value in target_angles]
        )

        sample = parse_telemetry_packet(current_packet)

        for name, expected in zip(TARGET_ANGLE_TELEMETRY_NAMES, target_angles):
            self.assertIn(name, sample)
            self.assertEqual(expected, sample[name])
            self.assertIs(type(sample[name]), float)
        self.assertIsNone(sample["Mag_Enabled"])

    def test_59_field_packet_appends_actual_mag_enabled_state(self):
        current_packet = ",".join(["1"] * 58 + ["0"])

        sample = parse_telemetry_packet(current_packet)

        self.assertEqual(0, sample["Mag_Enabled"])
        self.assertIs(type(sample["Mag_Enabled"]), int)

    def test_64_field_packet_leaves_mag_cal_active_unknown(self):
        current_packet = ",".join(["1"] * 64)

        sample = parse_telemetry_packet(current_packet)

        self.assertIsNone(sample["Mag_Cal_Active"])

    def test_65_field_packet_appends_mag_cal_active_as_integer(self):
        for raw, expected in (("0", 0), ("1", 1)):
            with self.subTest(raw=raw):
                current_packet = ",".join(["1"] * 64 + [raw])

                sample = parse_telemetry_packet(current_packet)

                self.assertEqual(expected, sample["Mag_Cal_Active"])
                self.assertIs(type(sample["Mag_Cal_Active"]), int)

    def test_75_field_packet_appends_allocation_values_with_exact_types(self):
        allocation_values = (
            "0.75", "0.50", "1325.25",
            "12.50", "-8.25", "3.75",
            "1.50", "-2.25", "0.50", "1",
        )
        current_packet = packet(65) + "," + ",".join(allocation_values)

        sample = parse_telemetry_packet(current_packet)

        expected = (0.75, 0.50, 1325.25, 12.50, -8.25, 3.75,
                    1.50, -2.25, 0.50, 1)
        for name, value in zip(ALLOCATION_TELEMETRY_NAMES[:-1], expected[:-1]):
            self.assertEqual(value, sample[name])
            self.assertIs(type(sample[name]), float)
        self.assertEqual(1, sample["Yaw_Authority_State"])
        self.assertIs(type(sample["Yaw_Authority_State"]), int)

    def test_65_field_and_legacy_packets_leave_allocation_values_unknown(self):
        for field_count in (65, 43, 10):
            with self.subTest(field_count=field_count):
                sample = parse_telemetry_packet(packet(field_count))
                for name in ALLOCATION_TELEMETRY_NAMES:
                    self.assertIsNone(sample[name])

    def test_current_csv_row_has_timestamp_plus_75_wire_fields(self):
        sample = parse_telemetry_packet(
            packet(65) + ",0.9,0.8,1300,1,2,3,4,5,6,2"
        )

        row = sample_to_csv_row("12:34:56.789", sample)

        self.assertEqual(76, len(CSV_FIELDS))
        self.assertEqual(76, len(row))
        self.assertEqual("12:34:56.789", row[0])
        self.assertEqual(0.9, row[CSV_FIELDS.index("Mixer_RP_Scale")])
        self.assertEqual(2, row[CSV_FIELDS.index("Yaw_Authority_State")])

    def test_malformed_required_value_is_rejected_for_current_packet(self):
        fields = (packet(65) + ",0.9,0.8,1300,1,2,3,4,5,6,2").split(",")
        fields[0] = "not-a-number"

        with self.assertRaises(ValueError):
            parse_telemetry_packet(",".join(fields))

        self.assertEqual(9, parse_telemetry_packet(packet(10))["Throttle"])

    def test_55_field_packet_leaves_target_angles_unknown(self):
        sample = parse_telemetry_packet(",".join(["1"] * 55))

        for name in TARGET_ANGLE_TELEMETRY_NAMES:
            self.assertIn(name, sample)
            self.assertIsNone(sample[name])

    def test_43_field_packet_leaves_per_imu_values_unknown(self):
        sample = parse_telemetry_packet(",".join(["1"] * 43))

        for name in IMU_TELEMETRY_NAMES + TARGET_ANGLE_TELEMETRY_NAMES:
            self.assertIn(name, sample)
            self.assertIsNone(sample[name])

    def test_per_imu_field_names_are_appended_in_exact_wire_order(self):
        self.assertEqual(IMU_TELEMETRY_NAMES, TELEMETRY_FIELDS[43:55])

    def test_field_names_and_order_match_the_wire_contract(self):
        # 이름과 순서가 계약이다. 개수는 그 목록에서 유도한다.
        self.assertEqual(EXPECTED_TELEMETRY_FIELDS, TELEMETRY_FIELDS)
        self.assertEqual(len(EXPECTED_TELEMETRY_FIELDS), len(TELEMETRY_FIELDS))

    def test_35_field_packet_leaves_new_fields_none(self):
        packet = ",".join(["1"] * 35)
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertIsNone(sample["Failsafe_Phase"])
        self.assertIsNone(sample["Trim_Roll"])
        self.assertIsNone(sample["Trim_Pitch"])

    def test_csv_is_timestamp_plus_every_telemetry_field(self):
        self.assertEqual(
            len(telemetry_schema.CSV_FIELDS),
            len(telemetry_schema.TELEMETRY_FIELDS) + 1,
        )
        self.assertEqual("Timestamp", telemetry_schema.CSV_FIELDS[0])

    def test_explicit_type_map_covers_new_field_types(self):
        for name in (
            "Armed",
            "Motor_M1",
            "Motor_M2",
            "Motor_M3",
            "Motor_M4",
            "PID_Loop_Hz",
            "Failsafe_Phase",
            "Hover_Valid",
            "Failsafe_Probe_State",
            "Failsafe_Probe_NoResponse",
            "Mag_Enabled",
            "Mag_Cal_Active",
            "Yaw_Authority_State",
        ):
            self.assertIs(telemetry_schema.TELEMETRY_FIELD_TYPES[name], int)
        for name in (
            "TgtRate_Roll",
            "TgtRate_Pitch",
            "TgtRate_Yaw",
            "MagHeading",
            "Mag_X",
            "Mag_Y",
            "Mag_Z",
            "Trim_Roll",
            "Trim_Pitch",
            "Hover_Est",
            "Failsafe_Probe_Response_G",
            *IMU_TELEMETRY_NAMES,
            *TARGET_ANGLE_TELEMETRY_NAMES,
            *ALLOCATION_TELEMETRY_NAMES[:-1],
        ):
            self.assertIs(telemetry_schema.TELEMETRY_FIELD_TYPES.get(name), float)

    def test_short_packet_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_telemetry_packet(packet(9))

    def test_empty_required_field_is_rejected(self):
        # 필수 필드가 빈 문자열이면 None이 수신 도구의 포맷/연산을 죽이므로
        # 파서 단계에서 거부해야 한다.
        with self.assertRaises(ValueError):
            parse_telemetry_packet("1.0,2.0,3.0,4,5,6,7,8,9,")
        with self.assertRaises(ValueError):
            parse_telemetry_packet(",2.0,3.0,4,5,6,7,8,9,10")

    def test_empty_extended_field_stays_unknown(self):
        sample = parse_telemetry_packet(packet(10) + ",,1")
        self.assertIsNone(sample["Fault_RC"])
        self.assertEqual(1, sample["Fault_Critical"])


class GainsPacketTest(unittest.TestCase):
    def test_gains_packet_detection(self):
        self.assertTrue(telemetry_schema.is_gains_packet("  GAINS,1.0  "))
        self.assertFalse(telemetry_schema.is_gains_packet("GAINS"))
        self.assertFalse(telemetry_schema.is_gains_packet(packet(10)))

    def test_gains_packet_parses_all_named_values_as_floats(self):
        line = "GAINS," + ",".join(str(index / 10) for index in range(1, 13))

        gains = telemetry_schema.parse_gains_packet(line)

        self.assertEqual(list(GAIN_NAMES), list(gains))
        self.assertEqual(0.1, gains["Kp_Angle_Roll"])
        self.assertEqual(1.2, gains["Kd_Rate_Yaw"])
        self.assertTrue(all(type(value) is float for value in gains.values()))

    def test_gains_packet_rejects_wrong_field_count(self):
        with self.assertRaises(ValueError):
            telemetry_schema.parse_gains_packet("GAINS,1.0,2.0")


if __name__ == "__main__":
    unittest.main()
