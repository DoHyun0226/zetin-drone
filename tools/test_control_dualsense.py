"""Regression tests for the ground-station and bench failsafe review fixes."""

import builtins
import contextlib
import csv
import importlib.util
import io
import pathlib
import socket
import struct
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
CONTROL_PATH = SCRIPTS_DIR / "control_dualsense.py"


class _LoopComplete(BaseException):
    pass


class _FakeSocket:
    def __init__(self, packets=()):
        self.packets = iter(packets)
        self.sent = []

    def bind(self, _address):
        pass

    def settimeout(self, _timeout):
        pass

    def setsockopt(self, *_args):
        pass

    def sendto(self, data, address):
        self.sent.append((data, address))

    def recvfrom(self, _size):
        try:
            packet = next(self.packets)
        except StopIteration:
            raise _LoopComplete
        if isinstance(packet, BaseException):
            raise packet
        return packet, ("192.168.4.1", 4210)

    def close(self):
        pass


class _FakeThread:
    def __init__(self, *_, **__):
        pass

    def start(self):
        pass

    def join(self, timeout=None):
        pass

    def is_alive(self):
        return False


class _PacketsUntilReleasedSocket:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()

    def recvfrom(self, _size):
        self.entered.set()
        if self.release.wait(timeout=0.01):
            raise SystemExit
        return b"invalid", ("192.168.4.1", 4210)

    def sendto(self, _data, _address):
        pass


class _PumpUntilReleased:
    def __init__(self):
        self.calls = 0
        self.entered = threading.Event()
        self.release = threading.Event()

    def pump(self):
        self.calls += 1
        self.entered.set()
        if self.release.wait(timeout=0.01):
            raise SystemExit


class _LoopEvent:
    def __init__(self, iterations):
        self.iterations = iterations
        self.calls = 0

    def pump(self):
        self.calls += 1
        if self.calls > self.iterations:
            raise _LoopComplete


class _BlockingThrottle(float):
    """Pause a controller tick after it has loaded the old throttle value."""

    def __new__(cls, value, stale_read_started, resume_completed):
        instance = super().__new__(cls, value)
        instance.stale_read_started = stale_read_started
        instance.resume_completed = resume_completed
        return instance

    def __lt__(self, other):
        self.stale_read_started.set()
        self.resume_completed.wait(timeout=0.5)
        return super().__lt__(other)


class _Joystick:
    def __init__(
        self,
        event,
        *,
        start=False,
        resume=False,
        reset=False,
        hats=None,
    ):
        self.event = event
        self.start = start
        self.resume = resume
        self.reset = reset
        self.hats = hats or [(0, 0)]

    def init(self):
        pass

    def get_name(self):
        return "test joystick"

    def get_button(self, index):
        if index == 0:
            return self.start
        if index == 3:
            return self.resume
        if index == 12:
            return self.reset
        return False

    def get_axis(self, _index):
        return 0.0

    def get_hat(self, _index):
        loop_index = max(0, self.event.calls - 1)
        return self.hats[min(loop_index, len(self.hats) - 1)]


def _pygame_for(joystick, event):
    pygame = types.ModuleType("pygame")
    pygame.init = lambda: None
    pygame.event = event
    pygame.joystick = types.SimpleNamespace(
        init=lambda: None,
        get_count=lambda: 1,
        Joystick=lambda _index: joystick,
    )
    return pygame


def _telemetry_sample(**overrides):
    sample = {
        "Roll": 0.0,
        "Pitch": 0.0,
        "Yaw": 0.0,
        "Gyro_Z": 0.0,
        "Throttle": 1000,
        "Motor_M1": 1001,
        "Motor_M2": 1002,
        "Motor_M3": 1003,
        "Motor_M4": 1004,
        "TgtRate_Roll": 0.0,
        "TgtRate_Pitch": 0.0,
        "TgtRate_Yaw": 0.0,
        "TgtAngle_Roll": 0.0,
        "TgtAngle_Pitch": 0.0,
        "TgtAngle_Yaw": 0.0,
        "Yaw_Hold": 1,
        "Mag_Enabled": 0,
        "Fault_RC": 0,
        "Fault_Critical": 0,
        "Calibration_OK": 1,
        "RC_Total_Pkts": 0,
        "RC_Dropped_Pkts": 0,
        "Armed": 1,
        "Trim_Roll": 0.0,
        "Trim_Pitch": 0.0,
        "Failsafe_Phase": 1,
        "Hover_Est": 1360.0,
        "Hover_Valid": 1,
        "Failsafe_Probe_State": 0,
        "Failsafe_Probe_NoResponse": 0,
        "Failsafe_Probe_Response_G": 0.0,
        "IMU1_Gyro_X": 0.0,
        "IMU1_Gyro_Y": 0.0,
        "IMU1_Gyro_Z": 0.0,
        "IMU1_Accel_X": 0.0,
        "IMU1_Accel_Y": 0.0,
        "IMU1_Accel_Z": 0.0,
        "IMU2_Gyro_X": 0.0,
        "IMU2_Gyro_Y": 0.0,
        "IMU2_Gyro_Z": 0.0,
        "IMU2_Accel_X": 0.0,
        "IMU2_Accel_Y": 0.0,
        "IMU2_Accel_Z": 0.0,
    }
    sample.update(overrides)
    return sample


class ControlDualsenseRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(SCRIPTS_DIR))

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(str(SCRIPTS_DIR))

    def setUp(self):
        self.module, self.startup_socket = self._load_module()
        self.module.shutdown_event.clear()
        self.module.last_telem_time = self.module.time.monotonic()

    def _prime_prearm_telemetry(self, **overrides):
        values = {
            "telem_calibration_ok": 1,
            "telem_armed": 0,
            "telem_fault_critical": 0,
            "fault_critical_drone": False,
        }
        values.update(overrides)
        for name, value in values.items():
            setattr(self.module, name, value)

    def _load_module(self, argv=(), module_name="_control_dualsense_under_test"):
        sys.modules.pop(module_name, None)
        spec = importlib.util.spec_from_file_location(module_name, CONTROL_PATH)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module

        fake_pygame = types.ModuleType("pygame")
        fake_socket = _FakeSocket()
        fake_csv_file = io.StringIO()
        with (
            mock.patch.dict(sys.modules, {"pygame": fake_pygame}),
            mock.patch.object(socket, "socket", return_value=fake_socket),
            mock.patch.object(pathlib.Path, "mkdir"),
            mock.patch.object(pathlib.Path, "open", return_value=fake_csv_file),
            mock.patch.object(threading, "Thread", _FakeThread),
            mock.patch.object(builtins, "input", side_effect=KeyboardInterrupt),
            mock.patch.object(
                sys, "argv", [str(CONTROL_PATH), *argv],
            ),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            spec.loader.exec_module(module)
        return module, fake_socket

    def tearDown(self):
        sys.modules.pop(self.module.__name__, None)

    def _run_controller(
        self,
        *,
        iterations,
        start=False,
        resume=False,
        reset=False,
        hats=None,
    ):
        event = _LoopEvent(iterations)
        joystick = _Joystick(
            event,
            start=start,
            resume=resume,
            reset=reset,
            hats=hats,
        )
        self.module.pygame = _pygame_for(joystick, event)
        with (
            mock.patch.object(self.module.time, "sleep", return_value=None),
            contextlib.redirect_stdout(io.StringIO()),
            self.assertRaises(_LoopComplete),
        ):
            self.module.controller_thread()

    def _run_telemetry(self, samples):
        self.module.sock = _FakeSocket([b"packet"] * len(samples))
        self.module.csv_writer = types.SimpleNamespace(writerow=lambda _row: None)
        self.module.csv_file = types.SimpleNamespace(flush=lambda: None)
        output = io.StringIO()
        with (
            mock.patch.object(
                self.module,
                "parse_telemetry_packet",
                side_effect=samples,
            ),
            mock.patch.object(
                self.module,
                "sample_to_csv_row",
                return_value=[],
            ),
            contextlib.redirect_stdout(output),
            self.assertRaises(_LoopComplete),
        ):
            self.module.telemetry_thread()
        return output.getvalue()

    def _run_datagrams(self, packets, rows):
        self.module.sock = _FakeSocket(packets)
        self.module.csv_writer = types.SimpleNamespace(
            writerow=rows.append,
        )
        self.module.csv_file = types.SimpleNamespace(flush=lambda: None)
        with self.assertRaises(_LoopComplete):
            self.module.telemetry_thread()

    def _autoland_lines(self, samples):
        output = self._run_telemetry(samples)
        return [
            line.strip()
            for line in output.splitlines()
            if "[AUTO-LAND]" in line
        ]

    def _rc_timeout_fault_lines(self, samples):
        output = self._run_telemetry(samples)
        return [
            line.strip()
            for line in output.splitlines()
            if "[FAULT] 드론: RC 타임아웃" in line
        ]

    def _status_lines(self, samples):
        output = self._run_telemetry(samples)
        return [
            line.strip()
            for line in output.splitlines()
            if "[STATUS]" in line
        ]

    def test_resume_request_restarts_streaming_before_sending_resume(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3

        request_resume = getattr(self.module, "request_resume", None)
        self.assertIsNotNone(request_resume, "resume must be reachable from the ground station")
        accepted = request_resume("stdin")

        self.assertTrue(accepted)
        self.assertTrue(self.module.is_streaming)
        self.assertEqual(commands, [])

    def test_resume_request_rejects_stale_cached_telemetry(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.last_telem_time = 1.0

        output = io.StringIO()
        with (
            mock.patch.object(self.module.time, "monotonic", return_value=10.0),
            contextlib.redirect_stdout(output),
        ):
            accepted = self.module.request_resume("test")

        self.assertFalse(accepted)
        self.assertFalse(self.module.is_streaming)
        self.assertEqual(commands, [])
        self.assertIn("오래됨", output.getvalue())

    def test_resume_is_sent_only_after_telemetry_confirms_an_accepted_rc(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.module.request_resume("test")

        advance = getattr(self.module, "advance_resume_attempt", None)
        self.assertIsNotNone(
            advance,
            "telemetry must gate resume on an RC packet accepted by firmware",
        )
        advance(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=4),
            now=10.1,
        )
        self.assertEqual(commands, [])

        advance(
            _telemetry_sample(RC_Total_Pkts=43, RC_Dropped_Pkts=5),
            now=10.2,
        )
        self.assertEqual(commands, ["resume"])

    def test_resume_wait_rc_times_out_when_every_new_packet_is_dropped(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.module.request_resume("test")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(RC_Total_Pkts=45, RC_Dropped_Pkts=8),
                now=12.1,
            )

        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(commands, [])
        self.assertIn("실패", output.getvalue())
        self.assertIn("RC 수락", output.getvalue())

    def test_resume_rejects_rc_counter_rollback_instead_of_treating_it_as_wrap(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 10
        self.module.telem_dropped_pkts = 100
        self.module.request_resume("test")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(RC_Total_Pkts=0, RC_Dropped_Pkts=0),
                now=self.module.resume_deadline - 0.1,
            )

        self.assertEqual(commands, [])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertIn("카운터", output.getvalue())

    def test_resume_accepts_a_small_forward_uint32_counter_wrap(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 0xFFFFFFFE
        self.module.telem_dropped_pkts = 7
        self.module.request_resume("test")

        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=0, RC_Dropped_Pkts=7),
            now=self.module.resume_deadline - 0.1,
        )

        self.assertEqual(commands, ["resume"])
        self.assertEqual(self.module.resume_state, "wait_phase")

    def test_resume_success_syncs_local_throttle_to_hover_est_and_warns_operator(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.current_throttle = 1000
        self.module.throttle_f = 1000.0
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.module.request_resume("test")
        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),
            now=10.1,
        )

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Failsafe_Phase=0,
                    Hover_Est=1378.6,
                    Hover_Valid=1,
                    RC_Total_Pkts=42,
                    RC_Dropped_Pkts=3,
                ),
                now=10.2,
            )

        self.assertEqual(commands, ["resume"])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(self.module.current_throttle, 1379)
        self.assertAlmostEqual(self.module.throttle_f, 1378.6)
        self.assertIn("성공", output.getvalue())
        self.assertIn("즉시 스로틀을 올리", output.getvalue())

    def test_resume_success_sync_is_not_overwritten_by_a_controller_tick(self):
        direct_commands = []
        stale_read_started = threading.Event()
        resume_completed = threading.Event()
        controller_errors = []
        self.module.send_cmd = direct_commands.append
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.current_throttle = 1000
        self.module.throttle_f = _BlockingThrottle(
            1000.0,
            stale_read_started,
            resume_completed,
        )
        self.module.resume_state = "wait_phase"
        self.module.resume_result_total_baseline = 41
        self.module.resume_result_dropped_baseline = 3

        def run_controller_tick():
            try:
                self._run_controller(iterations=1)
            except BaseException as error:
                controller_errors.append(error)

        controller = threading.Thread(target=run_controller_tick)
        controller.start()
        try:
            self.assertTrue(
                stale_read_started.wait(timeout=1.0),
                "controller did not reach the throttle read-modify-write",
            )
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Failsafe_Phase=0,
                    Hover_Est=1340.0,
                    RC_Total_Pkts=42,
                    RC_Dropped_Pkts=3,
                ),
                now=10.2,
            )
        finally:
            resume_completed.set()
            controller.join(timeout=2.0)

        self.assertFalse(controller.is_alive())
        self.assertEqual(controller_errors, [])
        self.assertEqual(self.module.current_throttle, 1340)
        self.assertEqual(self.module.throttle_f, 1340.0)
        self.assertNotIn("th 1000", direct_commands)

    def test_stale_phase_zero_packet_cannot_confirm_resume_success(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.current_throttle = 1000
        self.module.throttle_f = 1000.0
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")
        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),
            now=10.1,
        )

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Failsafe_Phase=0,
                    Hover_Est=1378.6,
                    Hover_Valid=1,
                    RC_Total_Pkts=39,
                    RC_Dropped_Pkts=3,
                ),
                now=10.2,
            )

        self.assertEqual(commands, ["resume"])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(self.module.current_throttle, 1000)
        self.assertEqual(self.module.throttle_f, 1000.0)
        self.assertIn("실패", output.getvalue())
        self.assertNotIn("성공", output.getvalue())

    def test_resume_refusal_is_reported_when_phase_stays_descending(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.current_throttle = 1000
        self.module.throttle_f = 1000.0
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.module.request_resume("test")
        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),
            now=10.1,
        )

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Failsafe_Phase=1,
                    RC_Total_Pkts=42,
                    RC_Dropped_Pkts=3,
                ),
                now=11.2,
            )

        self.assertEqual(commands, ["resume"])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(self.module.current_throttle, 1000)
        self.assertEqual(self.module.throttle_f, 1000.0)
        self.assertIn("실패", output.getvalue())
        self.assertIn("Failsafe_Phase=1", output.getvalue())
        self.assertNotIn("성공", output.getvalue())

    def test_resume_timeout_is_checked_without_a_valid_telemetry_sample(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.module.request_resume("test")

        check_timeout = getattr(self.module, "check_resume_timeout", None)
        self.assertIsNotNone(
            check_timeout,
            "resume deadlines must advance even during malformed/absent telemetry",
        )
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            check_timeout(now=12.1)

        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(commands, [])
        self.assertIn("실패", output.getvalue())
        self.assertIn("시간 초과", output.getvalue())

    def test_rc_timeout_phase_one_reports_autoland_descent_once(self):
        samples = [
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=1, Armed=1),
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=1, Armed=1),
        ]

        lines = self._rc_timeout_fault_lines(samples)

        self.assertEqual(len(lines), 1)
        self.assertIn("자동착륙", lines[0])
        self.assertIn("하강", lines[0])
        self.assertIn("DESCENDING", lines[0])

    def test_rc_timeout_phase_zero_disarmed_reports_immediate_cut_once(self):
        samples = [
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=0, Armed=0),
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=0, Armed=0),
        ]

        lines = self._rc_timeout_fault_lines(samples)

        self.assertEqual(len(lines), 1)
        self.assertNotIn("자동착륙", lines[0])
        self.assertIn("즉시 컷", lines[0])
        self.assertIn("호버 추정치 없음", lines[0])
        self.assertIn("지상 스로틀", lines[0])
        self.assertIn("시리얼", lines[0])

    def test_rc_timeout_terminal_phases_report_the_named_end_state_once(self):
        for phase, phase_name in (
            (2, "CUT_LANDED"),
            (3, "CUT_TIMEOUT"),
            (4, "CUT_ABORT"),
        ):
            with self.subTest(phase=phase):
                samples = [
                    _telemetry_sample(
                        Fault_RC=1,
                        Failsafe_Phase=phase,
                        Armed=0,
                    ),
                    _telemetry_sample(
                        Fault_RC=1,
                        Failsafe_Phase=phase,
                        Armed=0,
                    ),
                ]

                lines = self._rc_timeout_fault_lines(samples)

                self.assertEqual(len(lines), 1)
                self.assertIn("이미 종료", lines[0])
                self.assertIn(phase_name, lines[0])

    def test_rc_timeout_unknown_phase_reports_observed_values_once(self):
        samples = [
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=None, Armed=0),
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=None, Armed=0),
        ]

        lines = self._rc_timeout_fault_lines(samples)

        self.assertEqual(len(lines), 1)
        self.assertIn("상태 확정 불가", lines[0])
        self.assertIn("Failsafe_Phase=None", lines[0])
        self.assertIn("Armed=0", lines[0])
        self.assertNotIn("자동착륙 진행", lines[0])

    def test_autoland_line_contains_named_phase_probe_and_all_stage_e_fields(self):
        lines = self._autoland_lines([
            _telemetry_sample(
                Failsafe_Phase=1,
                Failsafe_Probe_State=4,
                Failsafe_Probe_NoResponse=2,
                Failsafe_Probe_Response_G=0.05786,
                Hover_Est=1360.4,
                Throttle=1302,
                Motor_M1=1291,
                Motor_M2=1298,
                Motor_M3=1306,
                Motor_M4=1313,
            ),
        ])

        self.assertEqual(len(lines), 1)
        line = lines[0]
        self.assertIn("[PROBE BLOCKED]", line)
        self.assertIn("Failsafe_Phase=DESCENDING", line)
        self.assertIn("Failsafe_Probe_State=BLOCKED", line)
        self.assertNotIn("Failsafe_Phase=1", line)
        self.assertNotIn("Failsafe_Probe_State=4", line)
        for expected in (
            "Failsafe_Probe_NoResponse=2",
            "Throttle=1302",
            "Motor_M1=1291",
            "Motor_M2=1298",
            "Motor_M3=1306",
            "Motor_M4=1313",
        ):
            self.assertIn(expected, line)
        self.assertRegex(
            line,
            r"(?:^| )Failsafe_Probe_Response_G=0\.0579(?: |$)",
        )
        self.assertRegex(line, r"(?:^| )Hover_Est=1360\.4(?: |$)")

    def test_phase_zero_does_not_print_an_autoland_line(self):
        lines = self._autoland_lines([
            _telemetry_sample(Failsafe_Phase=0),
        ])

        self.assertEqual(lines, [])

    def test_autoland_line_is_printed_for_every_sample_without_decimation(self):
        self.module.is_armed = True
        lines = self._autoland_lines([
            _telemetry_sample(Failsafe_Phase=1),
            _telemetry_sample(Failsafe_Phase=1),
            _telemetry_sample(Failsafe_Phase=1),
        ])

        self.assertEqual(len(lines), 3)

    def test_cut_landed_autoland_line_is_printed_once_while_phase_persists(self):
        lines = self._autoland_lines([
            _telemetry_sample(Failsafe_Phase=2),
            _telemetry_sample(Failsafe_Phase=2),
            _telemetry_sample(Failsafe_Phase=2),
            _telemetry_sample(Failsafe_Phase=2),
            _telemetry_sample(Failsafe_Phase=2),
        ])

        self.assertEqual(len(lines), 1)

    def test_descending_samples_continue_then_cut_transition_prints_once(self):
        lines = self._autoland_lines([
            _telemetry_sample(Failsafe_Phase=1),
            _telemetry_sample(Failsafe_Phase=1),
            _telemetry_sample(Failsafe_Phase=1),
            _telemetry_sample(Failsafe_Phase=2),
            _telemetry_sample(Failsafe_Phase=2),
        ])

        self.assertEqual(len(lines), 4)
        self.assertIn("Failsafe_Phase=CUT_LANDED", lines[-1])

    def test_phase_zero_rearms_terminal_transition_output(self):
        lines = self._autoland_lines([
            _telemetry_sample(Failsafe_Phase=2),
            _telemetry_sample(Failsafe_Phase=2),
            _telemetry_sample(Failsafe_Phase=0),
            _telemetry_sample(Failsafe_Phase=3),
            _telemetry_sample(Failsafe_Phase=3),
        ])

        self.assertEqual(len(lines), 2)
        self.assertIn("Failsafe_Phase=CUT_LANDED", lines[0])
        self.assertIn("Failsafe_Phase=CUT_TIMEOUT", lines[1])

    def test_cut_timeout_and_cut_abort_each_print_once_while_persistent(self):
        for phase, phase_name in ((3, "CUT_TIMEOUT"), (4, "CUT_ABORT")):
            with self.subTest(phase=phase):
                lines = self._autoland_lines([
                    _telemetry_sample(Failsafe_Phase=phase),
                    _telemetry_sample(Failsafe_Phase=phase),
                    _telemetry_sample(Failsafe_Phase=phase),
                ])

                self.assertEqual(len(lines), 1)
                self.assertIn(f"Failsafe_Phase={phase_name}", lines[0])

    def test_cut_landed_autoland_line_has_the_highest_priority_abort_marker(self):
        lines = self._autoland_lines([
            _telemetry_sample(
                Failsafe_Phase=2,
                Failsafe_Probe_State=4,
            ),
            _telemetry_sample(
                Failsafe_Phase=2,
                Failsafe_Probe_State=4,
            ),
        ])

        self.assertEqual(len(lines), 1)
        self.assertTrue(
            lines[0].startswith(
                "[AUTO-LAND][!!! CUT_LANDED: IMMEDIATE ABORT !!!]"
            )
        )

    def test_autoland_line_survives_missing_stage_e_fields(self):
        lines = self._autoland_lines([
            _telemetry_sample(
                Failsafe_Phase=1,
                Failsafe_Probe_State=None,
                Failsafe_Probe_NoResponse=None,
                Failsafe_Probe_Response_G=None,
                Hover_Est=None,
                Throttle=None,
                Motor_M1=None,
                Motor_M2=None,
                Motor_M3=None,
                Motor_M4=None,
            ),
        ])

        self.assertEqual(len(lines), 1)
        line = lines[0]
        for field in (
            "Failsafe_Probe_State",
            "Failsafe_Probe_NoResponse",
            "Failsafe_Probe_Response_G",
            "Hover_Est",
            "Throttle",
            "Motor_M1",
            "Motor_M2",
            "Motor_M3",
            "Motor_M4",
        ):
            self.assertIn(f"{field}=-", line)

    def test_gains_packet_is_printed_in_tuning_console_format(self):
        self.module.sock = _FakeSocket([
            (
                b"GAINS,1,2,3,4,5,6,7,8,9,10,11,12"
            ),
        ])
        output = io.StringIO()

        with (
            contextlib.redirect_stdout(output),
            self.assertRaises(_LoopComplete),
        ):
            self.module.telemetry_thread()

        gains_lines = [
            line.strip()
            for line in output.getvalue().splitlines()
            if "[GAINS]" in line
        ]
        self.assertEqual(
            gains_lines,
            [
                "[GAINS] "
                "Kp_Angle_Roll=1.0000 Kp_Angle_Pitch=2.0000 "
                "Kp_Angle_Yaw=3.0000 Kp_Rate_Roll=4.0000 "
                "Kp_Rate_Pitch=5.0000 Kp_Rate_Yaw=6.0000 "
                "Ki_Rate_Roll=7.0000 Ki_Rate_Pitch=8.0000 "
                "Ki_Rate_Yaw=9.0000 Kd_Rate_Roll=10.0000 "
                "Kd_Rate_Pitch=11.0000 Kd_Rate_Yaw=12.0000"
            ],
        )

    def test_truncated_gains_packet_is_ignored_without_exception(self):
        self.module.sock = _FakeSocket([
            b"GAINS,1,2,3",
        ])
        output = io.StringIO()

        with (
            contextlib.redirect_stdout(output),
            self.assertRaises(_LoopComplete),
        ):
            self.module.telemetry_thread()

        self.assertNotIn("[GAINS]", output.getvalue())

    def test_prearm_status_prints_once_for_rapid_samples_with_required_fields(self):
        self.module.is_armed = False
        sample = _telemetry_sample(
            Roll=1.25,
            Pitch=-2.5,
            Yaw=37.75,
            Throttle=1120,
            Armed=0,
            Hover_Est=1342.5,
            Hover_Valid=1,
            Failsafe_Phase=0,
        )

        lines = self._status_lines([sample, sample, sample])

        self.assertEqual(len(lines), 1)
        for expected in (
            "Roll=1.25",
            "Pitch=-2.50",
            "Yaw=37.75",
            "Throttle=1120",
            "Armed=0",
            "Hover_Est=1342.5",
            "Hover_Valid=1",
            "Faults=-",
        ):
            self.assertIn(expected, lines[0])

    def test_status_output_continues_while_armed(self):
        self.module.is_armed = True
        self.module.last_arm_time = self.module.time.monotonic()

        lines = self._status_lines([
            _telemetry_sample(Armed=1, Failsafe_Phase=0),
        ])

        self.assertEqual(len(lines), 1)
        self.assertIn("Armed=1", lines[0])

    def test_status_line_reports_yaw_hold_binary_states(self):
        self.module.is_armed = False

        for yaw_hold in (0, 1):
            with self.subTest(yaw_hold=yaw_hold):
                lines = self._status_lines([
                    _telemetry_sample(
                        Armed=0,
                        Failsafe_Phase=0,
                        Yaw_Hold=yaw_hold,
                    ),
                ])

                self.assertEqual(len(lines), 1)
                self.assertIn(f"Yaw_Hold={yaw_hold}", lines[0])

    def test_status_line_formats_unknown_yaw_hold_as_dash(self):
        self.module.is_armed = False

        lines = self._status_lines([
            _telemetry_sample(
                Armed=0,
                Failsafe_Phase=0,
                Yaw_Hold=None,
            ),
        ])

        self.assertEqual(len(lines), 1)
        self.assertIn("Yaw_Hold=-", lines[0])

    def test_status_line_lists_active_fault_names(self):
        self.module.is_armed = False

        lines = self._status_lines([
            _telemetry_sample(
                Armed=0,
                Failsafe_Phase=0,
                Fault_IMU1=1,
                Fault_Attitude=1,
            ),
        ])

        self.assertEqual(len(lines), 1)
        self.assertIn("Faults=IMU1,TILT", lines[0])

    def test_prearm_telemetry_loss_and_recovery_are_each_reported(self):
        self.module.is_armed = False
        self.module.is_streaming = False
        self.module.last_telem_time = (
            self.module.time.monotonic()
            - self.module.TELEM_TIMEOUT_SEC
            - 0.5
        )
        self.module.sock = _FakeSocket([
            socket.timeout(),
            b"packet",
        ])
        self.module.csv_writer = types.SimpleNamespace(
            writerow=lambda _row: None,
        )
        self.module.csv_file = types.SimpleNamespace(flush=lambda: None)
        output = io.StringIO()

        with (
            mock.patch.object(
                self.module,
                "parse_telemetry_packet",
                return_value=_telemetry_sample(
                    Armed=0,
                    Failsafe_Phase=0,
                ),
            ),
            mock.patch.object(
                self.module,
                "sample_to_csv_row",
                return_value=[],
            ),
            contextlib.redirect_stdout(output),
            self.assertRaises(_LoopComplete),
        ):
            self.module.telemetry_thread()

        rendered = output.getvalue()
        self.assertEqual(rendered.count("[FAULT] 텔레메트리"), 1)
        self.assertEqual(rendered.count("[OK] 텔레메트리 수신 복구"), 1)
        self.assertNotIn("rc 중단", rendered)

    def test_malformed_datagrams_cannot_mask_streaming_telemetry_loss(self):
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.last_telem_time = (
            self.module.time.monotonic()
            - self.module.TELEM_TIMEOUT_SEC
            - 0.5
        )
        self.module.sock = _FakeSocket([
            b"malformed",
            b"still,malformed",
        ])
        output = io.StringIO()

        with (
            contextlib.redirect_stdout(output),
            self.assertRaises(_LoopComplete),
        ):
            self.module.telemetry_thread()

        self.assertFalse(self.module.is_streaming)
        self.assertEqual(output.getvalue().count("[FAULT] 텔레메트리"), 1)

    def test_armed_diag_line_contains_hover_estimate_and_validity(self):
        self.module.is_armed = True
        sample = _telemetry_sample(
            Failsafe_Phase=0,
            Hover_Est=1423.56,
            Hover_Valid=1,
            Yaw_Hold=1,
        )
        output = self._run_telemetry([sample] * 4)

        diag_lines = [
            line for line in output.splitlines() if "[DIAG]" in line
        ]
        self.assertEqual(len(diag_lines), 1)
        self.assertIn("Yaw_Hold=1", diag_lines[0])
        self.assertIn("Hover_Est=1423.6", diag_lines[0])
        self.assertIn("Hover_Valid=1", diag_lines[0])

    def test_armed_diag_line_reports_axis_scales_and_yaw_authority(self):
        self.module.is_armed = True
        sample = _telemetry_sample(
            Failsafe_Phase=0,
            Mixer_RP_Scale=0.75,
            Mixer_Yaw_Scale=0.5,
            Yaw_Authority_State=1,
        )

        output = self._run_telemetry([sample] * 4)

        diag = [line for line in output.splitlines() if "[DIAG]" in line][0]
        self.assertIn("FW_Armed=1", diag)
        self.assertIn("RP_Scale=0.75", diag)
        self.assertIn("Yaw_Scale=0.50", diag)
        self.assertIn("Yaw_Authority=LIMITED(1)", diag)

    def test_local_armed_diag_does_not_present_disarmed_reset_values_as_healthy(self):
        self.module.is_armed = True
        self.module.last_arm_time = self.module.time.monotonic()
        sample = _telemetry_sample(
            Armed=0,
            Mixer_RP_Scale=1.0,
            Mixer_Yaw_Scale=1.0,
            Yaw_Authority_State=0,
        )

        output = self._run_telemetry([sample] * 4)

        diag = [line for line in output.splitlines() if "[DIAG]" in line][0]
        self.assertIn("FW_Armed=0", diag)
        self.assertIn("RP_Scale=-", diag)
        self.assertIn("Yaw_Scale=-", diag)
        self.assertIn("Yaw_Authority=-", diag)
        self.assertNotIn("Yaw_Authority=NORMAL(0)", diag)

    def test_local_armed_legacy_diag_labels_firmware_armed_unknown(self):
        self.module.is_armed = True
        sample = _telemetry_sample(
            Armed=None,
            Mixer_RP_Scale=1.0,
            Mixer_Yaw_Scale=1.0,
            Yaw_Authority_State=0,
        )

        output = self._run_telemetry([sample] * 4)

        diag = [line for line in output.splitlines() if "[DIAG]" in line][0]
        self.assertIn("FW_Armed=-", diag)
        self.assertIn("RP_Scale=-", diag)
        self.assertIn("Yaw_Scale=-", diag)
        self.assertIn("Yaw_Authority=-", diag)
        self.assertNotIn("Yaw_Authority=NORMAL(0)", diag)

    def test_armed_diag_line_renders_missing_allocation_values_as_dash(self):
        self.module.is_armed = True
        sample = _telemetry_sample(
            Failsafe_Phase=0,
            Mixer_RP_Scale=None,
            Mixer_Yaw_Scale=None,
            Yaw_Authority_State=None,
        )

        output = self._run_telemetry([sample] * 4)

        diag = [line for line in output.splitlines() if "[DIAG]" in line][0]
        self.assertIn("RP_Scale=-", diag)
        self.assertIn("Yaw_Scale=-", diag)
        self.assertIn("Yaw_Authority=-", diag)

    def _trim_datagrams(self):
        return [
            data.decode() for data, _addr in self.module.sock.sent
            if data.startswith(b"trim ")
        ]

    def test_send_trim_sends_once_without_blocking_the_rc_thread(self):
        # send_trim은 rcr을 20Hz로 보내는 controller_thread에서 호출된다.
        # reliable_send는 STOP_RETRIES회 sleep하므로 RC 업링크를 멈춘다.
        self.module.sock = _FakeSocket()
        self.module.trim_roll = 0.4
        self.module.trim_pitch = -0.6
        blocking = []
        with (
            mock.patch.object(
                self.module, "reliable_send",
                side_effect=lambda cmd: blocking.append(cmd),
            ),
            mock.patch.object(
                self.module.time, "sleep",
                side_effect=lambda s: blocking.append(f"sleep {s}"),
            ),
        ):
            self.module.send_trim()

        self.assertEqual([], blocking)
        self.assertEqual(["trim 0.40 -0.60"], self._trim_datagrams())

    def test_trim_is_resent_while_the_drone_still_disagrees(self):
        self.module.trim_synced = True
        self.module.trim_roll = 1.0
        self.module.trim_pitch = -2.0
        self.module.last_trim_resend = 0.0

        self._run_telemetry([_telemetry_sample(Trim_Roll=0.0, Trim_Pitch=0.0)])

        self.assertEqual(["trim 1.00 -2.00"], self._trim_datagrams())

    def test_trim_is_not_resent_once_the_drone_agrees(self):
        self.module.trim_synced = True
        self.module.trim_roll = 1.0
        self.module.trim_pitch = -2.0
        self.module.last_trim_resend = 0.0

        self._run_telemetry(
            [_telemetry_sample(Trim_Roll=1.0, Trim_Pitch=-2.0)] * 4
        )

        self.assertEqual([], self._trim_datagrams())

    def test_trim_resend_is_rate_limited(self):
        self.module.trim_synced = True
        self.module.trim_roll = 1.0
        self.module.trim_pitch = 0.0
        self.module.last_trim_resend = 0.0

        # 20Hz 텔레메트리 8샘플은 TRIM_RESEND_PERIOD(0.2s) 안이다.
        self._run_telemetry(
            [_telemetry_sample(Trim_Roll=0.0, Trim_Pitch=0.0)] * 8
        )

        self.assertEqual(1, len(self._trim_datagrams()))

    def test_trim_resend_skipped_when_drone_trim_is_unknown(self):
        self.module.trim_synced = True
        self.module.trim_roll = 1.0
        self.module.trim_pitch = 0.0
        self.module.last_trim_resend = 0.0

        self._run_telemetry(
            [_telemetry_sample(Trim_Roll=None, Trim_Pitch=None)] * 4
        )

        self.assertEqual([], self._trim_datagrams())

    def test_initial_sync_still_adopts_the_drone_trim(self):
        self.module.trim_synced = False
        self.module.trim_roll = 0.0
        self.module.trim_pitch = 0.0

        self._run_telemetry([_telemetry_sample(Trim_Roll=1.4, Trim_Pitch=-0.8)])

        self.assertTrue(self.module.trim_synced)
        self.assertAlmostEqual(1.4, self.module.trim_roll)
        self.assertAlmostEqual(-0.8, self.module.trim_pitch)
        self.assertEqual([], self._trim_datagrams())

    def test_armed_diag_trim_reports_the_drone_value_not_the_local_one(self):
        self.module.is_armed = True
        self.module.trim_synced = True
        self.module.trim_roll = 9.9
        self.module.trim_pitch = 9.9
        self.module.last_trim_resend = self.module.time.monotonic()
        sample = _telemetry_sample(
            Failsafe_Phase=0, Trim_Roll=1.2, Trim_Pitch=-0.4,
        )

        output = self._run_telemetry([sample] * 4)

        diag = [line for line in output.splitlines() if "[DIAG]" in line][0]
        self.assertIn("Trim=+1.2/-0.4!", diag)

    def test_armed_diag_trim_has_no_marker_when_the_drone_agrees(self):
        self.module.is_armed = True
        self.module.trim_synced = True
        self.module.trim_roll = 1.2
        self.module.trim_pitch = -0.4
        sample = _telemetry_sample(
            Failsafe_Phase=0, Trim_Roll=1.2, Trim_Pitch=-0.4,
        )

        output = self._run_telemetry([sample] * 4)

        diag = [line for line in output.splitlines() if "[DIAG]" in line][0]
        self.assertIn("Trim=+1.2/-0.4 ", diag)
        self.assertNotIn("!", diag)

    def test_armed_diag_trim_is_dash_when_unknown(self):
        self.module.is_armed = True
        sample = _telemetry_sample(
            Failsafe_Phase=0, Trim_Roll=None, Trim_Pitch=None,
        )

        output = self._run_telemetry([sample] * 4)

        diag = [line for line in output.splitlines() if "[DIAG]" in line][0]
        self.assertIn("Trim=-", diag)

    def test_armed_diag_line_reports_target_minus_measured_angle_errors(self):
        self.module.is_armed = True
        sample = _telemetry_sample(
            Failsafe_Phase=0,
            Roll=2.0,
            Pitch=1.5,
            TgtAngle_Roll=5.0,
            TgtAngle_Pitch=-4.0,
        )

        output = self._run_telemetry([sample] * 4)

        diag_lines = [line for line in output.splitlines() if "[DIAG]" in line]
        self.assertEqual(1, len(diag_lines))
        self.assertIn("eR=+3.0", diag_lines[0])
        self.assertIn("eP=-5.5", diag_lines[0])

    def test_armed_diag_line_formats_unknown_target_angles_as_dash(self):
        self.module.is_armed = True
        sample = _telemetry_sample(
            Failsafe_Phase=0,
            TgtAngle_Roll=None,
            TgtAngle_Pitch=None,
            TgtAngle_Yaw=None,
        )

        output = self._run_telemetry([sample] * 4)

        diag_lines = [line for line in output.splitlines() if "[DIAG]" in line]
        self.assertEqual(1, len(diag_lines))
        self.assertIn("eR=-", diag_lines[0])
        self.assertIn("eP=-", diag_lines[0])

    def test_armed_diag_line_reports_maximum_per_axis_gyro_difference(self):
        self.module.is_armed = True
        sample = _telemetry_sample(
            Failsafe_Phase=0,
            IMU1_Gyro_X=1.0,
            IMU1_Gyro_Y=2.0,
            IMU1_Gyro_Z=3.0,
            IMU2_Gyro_X=0.0,
            IMU2_Gyro_Y=6.0,
            IMU2_Gyro_Z=1.0,
        )

        output = self._run_telemetry([sample] * 4)

        diag_lines = [line for line in output.splitlines() if "[DIAG]" in line]
        self.assertEqual(1, len(diag_lines))
        self.assertIn("dG=4.0", diag_lines[0])

    def test_armed_diag_line_reports_unknown_when_any_per_imu_field_is_missing(self):
        self.module.is_armed = True
        sample = _telemetry_sample(Failsafe_Phase=0, IMU2_Accel_Z=None)

        output = self._run_telemetry([sample] * 4)

        diag_lines = [line for line in output.splitlines() if "[DIAG]" in line]
        self.assertEqual(1, len(diag_lines))
        self.assertIn("dG=-", diag_lines[0])

    def test_current_telemetry_csv_row_keeps_all_per_imu_values(self):
        imu_names = (
            "IMU1_Gyro_X", "IMU1_Gyro_Y", "IMU1_Gyro_Z",
            "IMU1_Accel_X", "IMU1_Accel_Y", "IMU1_Accel_Z",
            "IMU2_Gyro_X", "IMU2_Gyro_Y", "IMU2_Gyro_Z",
            "IMU2_Accel_X", "IMU2_Accel_Y", "IMU2_Accel_Z",
        )
        imu_values = (
            1.25, -2.5, 3.75, 0.125, -0.25, 0.5,
            -4.5, 5.25, -6.0, -0.625, 0.75, -0.875,
        )
        packet = ",".join(["1"] * 43 + [str(value) for value in imu_values])

        sample = self.module.parse_telemetry_packet(packet)
        row = self.module.sample_to_csv_row("12:34:56.789", sample)

        self.assertEqual(
            list(imu_values),
            [row[self.module.CSV_FIELDS.index(name)] for name in imu_names],
        )

    def test_58_field_telemetry_csv_row_keeps_all_target_angles(self):
        target_names = (
            "TgtAngle_Roll",
            "TgtAngle_Pitch",
            "TgtAngle_Yaw",
        )
        target_values = (5.25, -6.5, 179.75)
        packet = ",".join(
            ["1"] * 55 + [str(value) for value in target_values]
        )

        sample = self.module.parse_telemetry_packet(packet)
        row = self.module.sample_to_csv_row("12:34:56.789", sample)

        for name in target_names:
            self.assertIn(name, self.module.CSV_FIELDS)
        self.assertEqual(
            list(target_values),
            [row[self.module.CSV_FIELDS.index(name)] for name in target_names],
        )

    def test_raw_magic_routes_zimu_and_zcal_to_binary_before_utf8_decode(self):
        zimu = (
            struct.pack("<4sBBHIII", b"ZIMU", 1, 1, 0, 7, 123_000, 9)
            + struct.pack("<H12h", 1000, -1, *range(1, 12))
        )
        zcal = struct.pack("<4sB3x13f", b"ZCAL", 1, *range(13))
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_path = pathlib.Path(temp_dir) / "imuraw_test.bin"
            self.module.raw_log_path = raw_path
            self.module.raw_file = None
            rows = []
            with mock.patch.object(
                self.module,
                "parse_telemetry_packet",
                side_effect=AssertionError("raw packet reached ASCII parser"),
            ):
                self._run_datagrams([zimu, zcal], rows)
            self.assertIsNotNone(
                self.module.raw_file,
                "raw magic must be routed before strict UTF-8 decode",
            )
            self.module.raw_file.close()

            self.assertEqual(raw_path.read_bytes(), zimu + zcal)
            self.assertEqual(rows, [])
            self.assertEqual(self.module.raw_batch_count, 1)
            self.assertEqual(self.module.raw_last_dropped, 9)

    def test_raw_packets_never_create_csv_rows(self):
        zimu = (
            struct.pack("<4sBBHIII", b"ZIMU", 1, 1, 0, 1, 1000, 0)
            + struct.pack("<H12h", 1000, *range(12))
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            self.module.raw_log_path = (
                pathlib.Path(temp_dir) / "imuraw_test.bin"
            )
            self.module.raw_file = None
            rows = []
            self._run_datagrams([zimu], rows)
            self.module.raw_file.close()

            self.assertEqual(rows, [])

    def test_current_55_field_ascii_telemetry_still_reaches_csv(self):
        packet = ",".join(str(value) for value in range(55)).encode("ascii")
        rows = []

        self._run_datagrams([packet], rows)

        self.assertEqual(len(rows), 1)
        self.assertEqual(len(rows[0]), len(self.module.CSV_FIELDS))
        self.assertEqual(rows[0][1], 0.0)
        self.assertEqual(
            rows[0][self.module.CSV_FIELDS.index("IMU2_Accel_Z")],
            54.0,
        )
        self.assertEqual(
            [
                rows[0][self.module.CSV_FIELDS.index(name)]
                for name in (
                    "TgtAngle_Roll",
                    "TgtAngle_Pitch",
                    "TgtAngle_Yaw",
                )
            ],
            ["", "", ""],
        )

    def test_65_field_magcal_raw_xyz_and_active_state_reach_csv(self):
        fields = ["0"] * 65
        fields[31:34] = ["12.5", "-7.25", "33.75"]
        fields[64] = "1"
        rows = []

        self._run_datagrams([",".join(fields).encode("ascii")], rows)

        self.assertEqual(len(rows), 1)
        self.assertEqual(
            [
                rows[0][self.module.CSV_FIELDS.index(name)]
                for name in ("Mag_X", "Mag_Y", "Mag_Z", "Mag_Cal_Active")
            ],
            [12.5, -7.25, 33.75, 1],
        )

    def test_session_without_raw_packet_does_not_create_binary_file(self):
        packet = ",".join(["0"] * 55).encode("ascii")
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_path = pathlib.Path(temp_dir) / "imuraw_test.bin"
            self.module.raw_log_path = raw_path
            self.module.raw_file = None
            rows = []

            self._run_datagrams([packet], rows)

            self.assertFalse(raw_path.exists())

    def test_shutdown_decodes_received_raw_capture_and_prints_summary(self):
        zimu = (
            struct.pack("<4sBBHIII", b"ZIMU", 1, 2, 0, 7, 123_000, 0)
            + struct.pack("<H12h", 0, *range(12))
            + struct.pack("<H12h", 1000, *range(12, 24))
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = pathlib.Path(temp_dir)
            raw_path = temp_path / "imuraw_test.bin"
            self.module.raw_log_path = raw_path
            self.module.raw_file = None
            self.module.csv_file = io.StringIO()
            self.module.log_path = temp_path / "flight.csv"
            self.module._append_raw_datagram(zimu)
            output = io.StringIO()

            with (
                contextlib.redirect_stdout(output),
                contextlib.redirect_stderr(output),
            ):
                self.module.shutdown_workers_and_close_log(())

            self.assertEqual(raw_path.read_bytes(), zimu)
            self.assertTrue(raw_path.with_suffix(".csv").exists())
            rendered = output.getvalue()
            self.assertIn("[LOG] IMU raw CSV 변환 시작:", rendered)
            self.assertIn(
                "batches=1 samples=2 wireless_lost=0 "
                "producer_dropped=0 average_sample_rate_hz=1000.000 "
                "dt_us_min=0 dt_us_max=1000",
                rendered,
            )

    def test_shutdown_without_raw_packet_creates_neither_bin_nor_raw_csv(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = pathlib.Path(temp_dir)
            raw_path = temp_path / "imuraw_test.bin"
            self.module.raw_log_path = raw_path
            self.module.raw_file = None
            self.module.csv_file = io.StringIO()
            self.module.log_path = temp_path / "flight.csv"

            self.module.shutdown_workers_and_close_log(())

            self.assertFalse(raw_path.exists())
            self.assertFalse(raw_path.with_suffix(".csv").exists())

    def test_shutdown_decode_failure_warns_and_preserves_binary_capture(self):
        zimu = (
            struct.pack("<4sBBHIII", b"ZIMU", 1, 1, 0, 9, 456_000, 0)
            + struct.pack("<H12h", 0, *range(12))
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = pathlib.Path(temp_dir)
            raw_path = temp_path / "imuraw_test.bin"
            self.module.raw_log_path = raw_path
            self.module.raw_file = None
            self.module.csv_file = io.StringIO()
            self.module.log_path = temp_path / "flight.csv"
            self.module._append_raw_datagram(zimu)
            output = io.StringIO()

            with (
                mock.patch.object(
                    self.module,
                    "decode_file",
                    side_effect=RuntimeError("synthetic decode failure"),
                    create=True,
                ),
                contextlib.redirect_stdout(output),
            ):
                self.module.shutdown_workers_and_close_log(())

            self.assertEqual(raw_path.read_bytes(), zimu)
            self.assertFalse(raw_path.with_suffix(".csv").exists())
            self.assertIn(
                "[WARN] IMU raw CSV 변환 실패: synthetic decode failure",
                output.getvalue(),
            )

    def test_no_raw_cli_sends_raw_zero_and_suppresses_binary_file(self):
        module_name = "_control_dualsense_no_raw_under_test"
        module, fake_socket = self._load_module(
            ("--no-raw",), module_name=module_name,
        )
        try:
            self.assertIn(
                (b"raw 0", (module.UDP_IP, module.UDP_PORT)),
                fake_socket.sent,
            )
            zimu = (
                struct.pack("<4sBBHIII", b"ZIMU", 1, 1, 0, 1, 1000, 0)
                + struct.pack("<H12h", 0, *range(12))
            )
            with tempfile.TemporaryDirectory() as temp_dir:
                raw_path = pathlib.Path(temp_dir) / "imuraw_test.bin"
                module.raw_log_path = raw_path
                module.raw_file = None

                module._append_raw_datagram(zimu)

                self.assertIsNone(module.raw_file)
                self.assertFalse(raw_path.exists())
        finally:
            sys.modules.pop(module_name, None)

    def test_no_mag_cli_starts_with_mag_enabled_choice_off(self):
        module_name = "_control_dualsense_mag_under_test"
        module, _fake_socket = self._load_module(
            ("--no-mag",), module_name=module_name,
        )
        try:
            self.assertFalse(module.mag_enabled_choice)
        finally:
            sys.modules.pop(module_name, None)

    def test_default_cli_selects_mag_fusion_on(self):
        self.assertTrue(self.module.mag_enabled_choice)

    def test_default_cli_keeps_raw_capture_on_and_sends_no_raw_zero(self):
        self.assertTrue(self.module.raw_capture_enabled)
        self.assertNotIn(
            (b"raw 0", (self.module.UDP_IP, self.module.UDP_PORT)),
            self.startup_socket.sent,
        )

    def test_status_line_reports_raw_batch_and_producer_drop_counts(self):
        self.module.raw_batch_count = 1234
        self.module.raw_last_dropped = 7
        self.module.raw_last_receive_time = self.module.time.monotonic()

        status = self.module.format_status_telemetry(_telemetry_sample())

        self.assertIn("Raw=1234b/7d", status)

    def test_status_line_reports_dash_when_raw_is_not_being_received(self):
        self.module.raw_batch_count = 1234
        self.module.raw_last_dropped = 7
        self.module.raw_last_receive_time = (
            self.module.time.monotonic() - 2.0
        )

        status = self.module.format_status_telemetry(_telemetry_sample())

        self.assertIn("Raw=-", status)

    def test_stdin_raw_on_and_off_map_to_firmware_gate_commands(self):
        commands = []
        self.module.send_cmd = commands.append

        self.module.handle_stdin_command("raw on")
        self.module.handle_stdin_command("raw off")

        self.assertEqual(commands, ["raw 1", "raw 0"])

    def test_stdin_mag_on_and_off_update_choice_and_send_firmware_commands(self):
        commands = []
        self.module.send_cmd = commands.append

        self.module.handle_stdin_command("mag on")
        choice_after_on = self.module.mag_enabled_choice
        self.module.handle_stdin_command("mag off")

        self.assertTrue(choice_after_on)
        self.assertFalse(self.module.mag_enabled_choice)
        self.assertEqual(commands, ["mag 1", "mag 0"])

    def test_status_mag_uses_telemetry_even_when_ground_choice_is_on(self):
        self.module.mag_enabled_choice = True

        status = self.module.format_status_telemetry(
            _telemetry_sample(Mag_Enabled=0)
        )

        self.assertIn("Mag=0", status)

    def test_status_mag_is_dash_when_telemetry_field_is_unknown(self):
        status = self.module.format_status_telemetry(
            _telemetry_sample(Mag_Enabled=None)
        )

        self.assertIn("Mag=-", status)

    def test_telemetry_thread_advances_resume_attempt(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        self._run_telemetry([
            _telemetry_sample(
                Failsafe_Phase=1,
                Hover_Est=1360.0,
                Hover_Valid=1,
                RC_Total_Pkts=41,
                RC_Dropped_Pkts=3,
            ),
        ])

        self.assertEqual(commands, ["resume"])
        self.assertEqual(self.module.resume_state, "wait_phase")

    def test_critical_fault_sample_cancels_resume_before_command_send(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Fault_Critical=1,
                    RC_Total_Pkts=41,
                    RC_Dropped_Pkts=3,
                ),
                now=10.0,
            )

        self.assertEqual(commands, [])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertIn("실패", output.getvalue())
        self.assertIn("Fault_Critical", output.getvalue())

    def test_disarmed_sample_cancels_resume_before_command_send(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Armed=0,
                    RC_Total_Pkts=41,
                    RC_Dropped_Pkts=3,
                ),
                now=10.0,
            )

        self.assertEqual(commands, [])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertIn("실패", output.getvalue())
        self.assertIn("Armed=0", output.getvalue())

    def test_triangle_button_starts_resume_without_disarming(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3

        self._run_controller(iterations=2, resume=True)

        self.assertTrue(self.module.is_armed)
        self.assertTrue(self.module.is_streaming)
        self.assertEqual(self.module.resume_state, "wait_rc")
        self.assertEqual(commands, [])

    def test_stdin_resume_uses_the_guarded_sequence_instead_of_direct_send(self):
        direct_commands = []
        reliable_commands = []
        self.module.send_cmd = direct_commands.append
        self.module.reliable_send = reliable_commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3

        handler = getattr(self.module, "handle_stdin_command", None)
        self.assertIsNotNone(handler, "stdin resume must use the guarded resume path")
        handler("  resume  ")

        self.assertEqual(self.module.resume_state, "wait_rc")
        self.assertTrue(self.module.is_streaming)
        self.assertEqual(direct_commands, [])
        self.assertEqual(reliable_commands, [])

    def test_stdin_start_waits_for_firmware_armed_before_rc_streaming(self):
        direct_commands = []
        reliable_commands = []
        self.module.send_cmd = direct_commands.append
        self.module.reliable_send = reliable_commands.append
        self.module.is_armed = False
        self.module.is_streaming = False
        self._prime_prearm_telemetry()

        self.module.handle_stdin_command("  start  ")

        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)
        self.assertEqual(self.module.current_throttle, 1000)
        self.assertEqual(self.module.throttle_f, 1000.0)
        self.assertEqual(getattr(self.module, "arm_state", None), "arming")
        self.assertEqual(reliable_commands, ["mag 1", "start"])
        self.assertEqual(
            direct_commands,
            [],
            "stdin start must not leak through send_cmd without arm state",
        )

    def test_stdin_throttle_updates_both_local_values_before_sending(self):
        commands = []
        telem_lock_free_during_send = []

        def capture_command(command):
            acquired = self.module.telem_lock.acquire(blocking=False)
            telem_lock_free_during_send.append(acquired)
            if acquired:
                self.module.telem_lock.release()
            commands.append(command)

        self.module.send_cmd = capture_command
        self.module.current_throttle = 1000
        self.module.throttle_f = 1000.0

        self.module.handle_stdin_command("th 1200")

        self.assertEqual(self.module.current_throttle, 1200)
        self.assertEqual(self.module.throttle_f, 1200.0)
        self.assertEqual(commands, ["th 1200"])
        self.assertEqual(telem_lock_free_during_send, [True])

    def test_stdin_throttle_cannot_be_overtaken_by_stale_controller_send(self):
        commands = []
        stale_send_started = threading.Event()
        stdin_command_sent = threading.Event()
        release_stale_send = threading.Event()
        controller_errors = []

        def controlled_send(command):
            if command == "th 1200":
                stale_send_started.set()
                release_stale_send.wait(timeout=2.0)
            commands.append(command)
            if command == "th 1210":
                stdin_command_sent.set()

        self.module.send_cmd = controlled_send
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.current_throttle = 1190
        self.module.throttle_f = 1190.0
        event = _LoopEvent(1)
        joystick = _Joystick(event)
        joystick.get_axis = lambda index: 1.0 if index == 5 else 0.0
        self.module.pygame = _pygame_for(joystick, event)

        def run_controller():
            try:
                self.module.controller_thread()
            except _LoopComplete:
                pass
            except BaseException as error:
                controller_errors.append(error)

        controller = threading.Thread(target=run_controller)
        controller.start()
        self.assertTrue(stale_send_started.wait(timeout=1.0))

        stdin_worker = threading.Thread(
            target=self.module.handle_stdin_command,
            args=("th 1210",),
        )
        stdin_worker.start()
        stdin_command_sent.wait(timeout=0.2)
        release_stale_send.set()
        controller.join(timeout=2.0)
        stdin_worker.join(timeout=2.0)

        self.assertFalse(controller.is_alive())
        self.assertFalse(stdin_worker.is_alive())
        self.assertEqual(controller_errors, [])
        self.assertEqual(
            [command for command in commands if command.startswith("th ")],
            ["th 1200", "th 1210"],
        )
        self.assertEqual(self.module.current_throttle, 1210)
        self.assertEqual(self.module.throttle_f, 1210.0)

    def test_stdin_throttle_clamps_to_controller_range_and_reports_it(self):
        commands = []
        self.module.send_cmd = commands.append
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            self.module.handle_stdin_command("th 2500")

        self.assertEqual(self.module.current_throttle, 1900)
        self.assertEqual(self.module.throttle_f, 1900.0)
        self.assertEqual(commands, ["th 1900"])
        self.assertIn("2500", output.getvalue())
        self.assertIn("1900", output.getvalue())
        self.assertIn("clamp", output.getvalue().lower())

    def test_invalid_stdin_throttle_prints_usage_without_sending(self):
        commands = []
        self.module.send_cmd = commands.append
        self.module.current_throttle = 1175
        self.module.throttle_f = 1175.0
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            self.module.handle_stdin_command("th abc")

        self.assertEqual(commands, [])
        self.assertEqual(self.module.current_throttle, 1175)
        self.assertEqual(self.module.throttle_f, 1175.0)
        self.assertIn("사용법", output.getvalue())
        self.assertIn("th <val>", output.getvalue())

    def test_stdin_stop_cancels_resume_before_sending_stop(self):
        direct_commands = []
        reliable_commands = []
        self.module.send_cmd = direct_commands.append
        self.module.reliable_send = reliable_commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        self.module.handle_stdin_command("stop")
        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),
            now=10.0,
        )

        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(direct_commands, [])
        self.assertEqual(reliable_commands, ["stop"])

    def test_disarm_cancels_a_ground_station_resume_attempt(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        self.module.disarm("test")
        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),
            now=10.0,
        )

        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(commands, ["stop"])

    def test_streaming_loss_reports_that_the_resume_attempt_failed(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.stop_streaming_only("텔레메트리 끊김")

        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(commands, [])
        self.assertIn("[RESUME] 실패", output.getvalue())
        self.assertIn("텔레메트리 끊김", output.getvalue())

    def test_concurrent_disarm_cannot_be_followed_by_resume(self):
        commands = []
        resume_send_started = threading.Event()
        release_resume_send = threading.Event()
        stop_sent = threading.Event()

        def controlled_reliable_send(command):
            if command == "resume":
                resume_send_started.set()
                release_resume_send.wait(timeout=2.0)
            commands.append(command)
            if command == "stop":
                stop_sent.set()

        self.module.reliable_send = controlled_reliable_send
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        resume_thread = threading.Thread(
            target=self.module.advance_resume_attempt,
            args=(_telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),),
            kwargs={"now": 10.0},
        )
        resume_thread.start()
        self.assertTrue(resume_send_started.wait(timeout=1.0))

        disarm_thread = threading.Thread(
            target=self.module.disarm,
            args=("test",),
        )
        disarm_thread.start()
        stop_sent.wait(timeout=0.2)
        release_resume_send.set()
        resume_thread.join(timeout=2.0)
        disarm_thread.join(timeout=2.0)

        self.assertFalse(resume_thread.is_alive())
        self.assertFalse(disarm_thread.is_alive())
        self.assertEqual(commands, ["resume", "stop"])
        self.assertEqual(self.module.resume_state, "idle")

    def test_resume_request_is_serialized_behind_an_in_progress_stop(self):
        stop_send_started = threading.Event()
        release_stop_send = threading.Event()
        request_done = threading.Event()
        request_results = []

        def controlled_reliable_send(command):
            if command == "stop":
                stop_send_started.set()
                release_stop_send.wait(timeout=2.0)

        self.module.reliable_send = controlled_reliable_send
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3

        disarm_thread = threading.Thread(
            target=self.module.disarm,
            args=("test",),
        )
        disarm_thread.start()
        self.assertTrue(stop_send_started.wait(timeout=1.0))

        def request_in_thread():
            request_results.append(self.module.request_resume("test"))
            request_done.set()

        request_thread = threading.Thread(target=request_in_thread)
        request_thread.start()
        finished_before_stop = request_done.wait(timeout=0.2)
        release_stop_send.set()
        disarm_thread.join(timeout=2.0)
        request_thread.join(timeout=2.0)

        self.assertFalse(finished_before_stop)
        self.assertFalse(disarm_thread.is_alive())
        self.assertFalse(request_thread.is_alive())
        self.assertEqual(request_results, [False])
        self.assertEqual(self.module.resume_state, "idle")

    def test_x_button_disarms_after_streaming_stops(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = True

        self.module.stop_streaming_only("test")
        self._run_controller(iterations=1, start=True)

        self.assertEqual(commands, ["stop"])
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)

    def test_critical_fault_disarms_after_streaming_stops(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = True

        self.module.stop_streaming_only("test")
        self._run_telemetry([
            _telemetry_sample(Fault_Critical=1),
        ])

        self.assertEqual(commands, ["stop"])
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)

    def test_stop_streaming_only_does_not_send_stop(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = True

        self.module.stop_streaming_only("test")

        self.assertEqual(commands, [])
        self.assertTrue(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)

    def test_arm_defaults_mag_on_before_start_and_does_not_send_trim(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.rc_seq = 73
        self.module.is_armed = False
        self.module.is_streaming = False
        self._prime_prearm_telemetry()

        accepted = self.module.arm()

        self.assertTrue(accepted)
        self.assertEqual(commands, ["mag 1", "start"])
        self.assertEqual(self.module.rc_seq, 73)
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)
        self.assertEqual(getattr(self.module, "arm_state", None), "arming")

    def test_arm_rejects_unhealthy_or_stale_prearm_telemetry(self):
        cases = (
            ("stale", {"last_telem_time": 1.0}, "오래됨"),
            ("calibration", {"telem_calibration_ok": 0}, "Calibration_OK"),
            ("critical", {"fault_critical_drone": True}, "Fault_Critical"),
            ("firmware armed", {"telem_armed": 1}, "Armed"),
        )
        for name, overrides, expected_reason in cases:
            with self.subTest(name=name):
                commands = []
                self.module.reliable_send = commands.append
                self.module.is_armed = False
                self.module.is_streaming = False
                self.module.arm_state = "idle"
                self.module.last_telem_time = 9.5
                self._prime_prearm_telemetry()
                for key, value in overrides.items():
                    setattr(self.module, key, value)
                output = io.StringIO()

                with (
                    mock.patch.object(
                        self.module.time, "monotonic", return_value=10.0
                    ),
                    contextlib.redirect_stdout(output),
                ):
                    accepted = self.module.arm()

                self.assertFalse(accepted)
                self.assertEqual(commands, [])
                self.assertFalse(self.module.is_armed)
                self.assertFalse(self.module.is_streaming)
                self.assertEqual(self.module.arm_state, "idle")
                self.assertIn(expected_reason, output.getvalue())

    def test_firmware_armed_telemetry_confirms_arm_and_starts_streaming(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = False
        self.module.is_streaming = False
        self._prime_prearm_telemetry()
        self.assertTrue(self.module.arm())

        output = self._run_telemetry([
            _telemetry_sample(
                Armed=1,
                Calibration_OK=1,
                Fault_Critical=0,
                Failsafe_Phase=0,
            ),
        ])

        self.assertEqual(self.module.arm_state, "armed")
        self.assertTrue(self.module.is_armed)
        self.assertTrue(self.module.is_streaming)
        self.assertEqual(self.module.current_throttle, 1100)
        self.assertEqual(self.module.throttle_f, 1100.0)
        self.assertIn("ARMED 확인", output)

    def test_arm_confirmation_timeout_aborts_with_reliable_stop(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = False
        self.module.is_streaming = False
        self._prime_prearm_telemetry()
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.assertTrue(self.module.arm())

        check_timeout = getattr(self.module, "check_arm_timeout", None)
        self.assertIsNotNone(check_timeout, "arming needs a wall-clock deadline")
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            check_timeout(now=12.1)

        self.assertEqual(commands, ["mag 1", "start", "stop"])
        self.assertEqual(self.module.arm_state, "idle")
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)
        self.assertEqual(self.module.current_throttle, 1000)
        self.assertIn("시간 초과", output.getvalue())

    def test_disarmed_telemetry_retries_start_at_a_bounded_cadence(self):
        reliable_commands = []
        direct_commands = []
        self.module.reliable_send = reliable_commands.append
        self.module.send_cmd = direct_commands.append
        self.module.is_armed = False
        self.module.is_streaming = False
        self._prime_prearm_telemetry()
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.assertTrue(self.module.arm())

        advance = getattr(self.module, "advance_arm_attempt", None)
        self.assertIsNotNone(advance, "arming needs telemetry-driven confirmation")
        waiting = _telemetry_sample(
            Armed=0,
            Calibration_OK=1,
            Fault_Critical=0,
            Failsafe_Phase=0,
        )
        advance(waiting, now=10.10)
        advance(waiting, now=10.25)
        advance(waiting, now=10.30)
        advance(waiting, now=10.50)

        self.assertEqual(reliable_commands, ["mag 1", "start"])
        self.assertEqual(direct_commands, ["start", "start"])
        self.assertEqual(self.module.arm_state, "arming")

    def test_arm_uses_latest_mag_choice_for_on_and_off(self):
        for choice_command, expected_mag_command in (
            ("mag on", "mag 1"),
            ("mag off", "mag 0"),
        ):
            with self.subTest(choice_command=choice_command):
                commands = []
                self.module.send_cmd = lambda _command: None
                self.module.reliable_send = commands.append
                self.module.handle_stdin_command(choice_command)
                self.module.is_armed = False
                self.module.is_streaming = False
                self.module.arm_state = "idle"
                self._prime_prearm_telemetry()

                self.module.arm()

                self.assertEqual(commands, [expected_mag_command, "start"])

    def test_arm_serializes_start_against_concurrent_disarm(self):
        commands = []
        start_send_started = threading.Event()
        release_start_send = threading.Event()
        stop_sent = threading.Event()

        def controlled_reliable_send(command):
            commands.append(command)
            if command == "start":
                start_send_started.set()
                release_start_send.wait(timeout=2.0)
            elif command == "stop":
                stop_sent.set()

        self.module.reliable_send = controlled_reliable_send
        self.module.is_armed = False
        self.module.is_streaming = False
        self._prime_prearm_telemetry()

        arm_thread = threading.Thread(target=self.module.arm)
        arm_thread.start()
        self.assertTrue(start_send_started.wait(timeout=1.0))

        disarm_thread = threading.Thread(
            target=self.module.disarm,
            args=("test",),
        )
        disarm_thread.start()
        stop_raced_start = stop_sent.wait(timeout=0.2)
        release_start_send.set()
        arm_thread.join(timeout=2.0)
        disarm_thread.join(timeout=2.0)

        self.assertFalse(stop_raced_start)
        self.assertFalse(arm_thread.is_alive())
        self.assertFalse(disarm_thread.is_alive())
        self.assertEqual(commands, ["mag 1", "start", "stop"])
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)

    def test_arm_updates_grace_time_before_publishing_arming(self):
        armed_state_seen_by_clock = []
        self.module.reliable_send = lambda _command: None
        self.module.is_armed = False
        self.module.is_streaming = False
        self._prime_prearm_telemetry()

        def observe_armed_state():
            armed_state_seen_by_clock.append(self.module.is_armed)
            return 123.0

        with mock.patch.object(
            self.module.time,
            "monotonic",
            side_effect=observe_armed_state,
        ):
            self.module.arm()

        self.assertEqual(armed_state_seen_by_clock, [False])
        self.assertEqual(self.module.last_arm_time, 123.0)
        self.assertFalse(self.module.is_armed)
        self.assertEqual(getattr(self.module, "arm_state", None), "arming")

    def test_first_complete_telemetry_trim_is_adopted_once(self):
        self.module.trim_roll = 0.0
        self.module.trim_pitch = 0.0
        self.module.trim_synced = False
        self.module.is_armed = False
        self.module.is_streaming = False

        self._run_telemetry([
            _telemetry_sample(Trim_Roll=None, Trim_Pitch=None),
            _telemetry_sample(Trim_Roll=1.5, Trim_Pitch=-2.5),
            _telemetry_sample(Trim_Roll=4.0, Trim_Pitch=3.0),
        ])

        self.assertTrue(self.module.trim_synced)
        self.assertEqual(
            (self.module.trim_roll, self.module.trim_pitch),
            (1.5, -2.5),
        )

    def test_dpad_trim_is_clamped_to_firmware_limit(self):
        commands = []
        # send_trim은 블로킹하지 않는 send_cmd 경로를 쓴다.
        self.module.send_cmd = lambda cmd: (
            commands.append(cmd) if cmd.startswith('trim ') else None
        )
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.trim_roll = 9.9
        self.module.trim_pitch = 0.0
        self.module.last_hat_state = (0, 0)

        self._run_controller(
            iterations=2,
            hats=[(1, 0), (0, 0)],
        )

        self.assertEqual(self.module.trim_roll, 10.0)
        self.assertEqual(commands, ["trim 10.00 0.00"])

    def test_trim_reset_button_is_edge_triggered(self):
        commands = []
        # send_trim은 블로킹하지 않는 send_cmd 경로를 쓴다.
        self.module.send_cmd = lambda cmd: (
            commands.append(cmd) if cmd.startswith('trim ') else None
        )
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.trim_roll = 1.0
        self.module.trim_pitch = -1.0
        self.module.last_btn_trim_reset = False

        self._run_controller(iterations=3, reset=True)

        self.assertEqual(
            commands,
            ["trim 0.00 0.00"],
        )

    def test_controller_banner_lists_supported_stdin_commands(self):
        event = _LoopEvent(0)
        joystick = _Joystick(event)
        self.module.pygame = _pygame_for(joystick, event)
        output = io.StringIO()

        with (
            contextlib.redirect_stdout(output),
            self.assertRaises(_LoopComplete),
        ):
            self.module.controller_thread()

        banner = output.getvalue()
        for expected in (
            "stdin",
            "pa|ia|da",
            "pr|ir|dr",
            "pp|ip|dp",
            "py|iy|dy",
            "ap",
            "ar|at|ay",
            "th <val>",
            "yaw <0|1>",
            "mag on|off",
            "gains",
            "start",
            "stop",
            "resume",
            "trim",
            "magc",
        ):
            self.assertIn(expected, banner)

    def test_stdin_help_is_still_printed_without_a_gamepad(self):
        pygame = types.ModuleType("pygame")
        pygame.init = lambda: None
        pygame.joystick = types.SimpleNamespace(
            init=lambda: None,
            get_count=lambda: 0,
        )
        self.module.pygame = pygame
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            self.module.controller_thread()

        rendered = output.getvalue()
        self.assertIn("stdin", rendered)
        self.assertIn("th <val>", rendered)
        self.assertIn("gains", rendered)
        self.assertIn("[ERR] 컨트롤러가 없습니다!", rendered)

    def test_telemetry_thread_exits_after_shutdown_event_is_set(self):
        shutdown_event = getattr(self.module, "shutdown_event", None)
        self.assertIsNotNone(shutdown_event)
        blocking_socket = _PacketsUntilReleasedSocket()
        self.module.sock = blocking_socket
        self.module.is_streaming = False
        shutdown_event.clear()
        worker = threading.Thread(
            target=self.module.telemetry_thread,
            daemon=True,
        )
        worker.start()
        self.assertTrue(blocking_socket.entered.wait(timeout=0.5))

        shutdown_event.set()
        worker.join(timeout=0.5)
        stopped_on_shutdown = not worker.is_alive()
        blocking_socket.release.set()
        worker.join(timeout=0.5)

        self.assertTrue(stopped_on_shutdown)

    def test_controller_thread_exits_after_shutdown_event_is_set(self):
        shutdown_event = getattr(self.module, "shutdown_event", None)
        self.assertIsNotNone(shutdown_event)
        event = _PumpUntilReleased()
        joystick = _Joystick(event)
        self.module.pygame = _pygame_for(joystick, event)
        self.module.is_streaming = False
        shutdown_event.clear()
        worker = threading.Thread(
            target=self.module.controller_thread,
            daemon=True,
        )
        worker.start()
        self.assertTrue(event.entered.wait(timeout=0.5))

        shutdown_event.set()
        worker.join(timeout=0.5)
        stopped_on_shutdown = not worker.is_alive()
        event.release.set()
        worker.join(timeout=0.5)

        self.assertTrue(stopped_on_shutdown)

    def test_shutdown_closes_csv_after_preserving_header_and_record(self):
        shutdown = getattr(
            self.module,
            "shutdown_workers_and_close_log",
            None,
        )
        self.assertIsNotNone(shutdown)
        with tempfile.TemporaryDirectory() as temp_dir:
            log_path = pathlib.Path(temp_dir) / "flight.csv"
            csv_file = log_path.open("w", newline="", encoding="utf-8")
            self.module.log_path = log_path
            self.module.csv_file = csv_file
            self.module.csv_writer = csv.writer(csv_file)
            record = ["12:34:56.789", "1.25"] + [""] * (
                len(self.module.CSV_FIELDS) - 2
            )
            self.module.csv_writer.writerow(self.module.CSV_FIELDS)
            self.module.csv_writer.writerow(record)

            self.module.shutdown_event.clear()
            shutdown(())

            self.assertTrue(csv_file.closed)
            with log_path.open(newline="", encoding="utf-8") as saved_log:
                rows = list(csv.reader(saved_log))
            self.assertEqual(rows, [list(self.module.CSV_FIELDS), record])


class BenchTrimResetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(SCRIPTS_DIR))

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(str(SCRIPTS_DIR))

    def _load_bench(self, filename):
        path = SCRIPTS_DIR / filename
        module_name = f"_{path.stem}_under_test"
        sys.modules.pop(module_name, None)
        spec = importlib.util.spec_from_file_location(module_name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        with (
            mock.patch.object(socket, "socket", return_value=_FakeSocket()),
            mock.patch.object(sys, "argv", [str(path)]),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            spec.loader.exec_module(module)
        self.addCleanup(sys.modules.pop, module_name, None)
        return module

    def test_bench_senders_clear_trim_before_start(self):
        for filename in (
            "bench_thrust_ramp.py",
            "bench_sign_test.py",
            "bench_yaw_test.py",
        ):
            with self.subTest(filename=filename):
                module = self._load_bench(filename)
                commands = []
                module._running = False
                module.send = commands.append
                with mock.patch.object(module.time, "sleep", return_value=None):
                    module.sender()

                self.assertIn("trim 0 0", commands)
                self.assertLess(
                    commands.index("trim 0 0"),
                    commands.index("start"),
                )


if __name__ == "__main__":
    unittest.main()
