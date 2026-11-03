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
import time
import types
import unittest
from unittest import mock


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
CONTROL_PATH = SCRIPTS_DIR / "ground" / "control_station.py"


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


class _BindFailSocket(_FakeSocket):
    def bind(self, _address):
        raise OSError(98, "Address already in use")


class _RecordingUI:
    def __init__(self):
        self.calls = []

    def event(self, text):
        self.calls.append(("event", text))

    def periodic(self, text):
        self.calls.append(("periodic", text))

    def transition(self, text):
        self.calls.append(("transition", text))

    def alert(self, text):
        self.calls.append(("alert", text))

    def update(self, snap):
        self.calls.append(("update", snap))

    def run(self, input_handler, on_interrupt, on_eof):
        self.calls.append(("run", None))

    def after_run(self):
        self.calls.append(("after_run", None))

    def texts(self, kind):
        return [text for call_kind, text in self.calls if call_kind == kind]


class _RaisingUI(_RecordingUI):
    """A broken UI: every output call raises, as a dead terminal pipe would."""

    def event(self, text):
        raise BrokenPipeError("synthetic event failure")

    def periodic(self, text):
        raise BrokenPipeError("synthetic periodic failure")

    def alert(self, text):
        raise BrokenPipeError("synthetic alert failure")


class _RaisingJoystick:
    def init(self):
        pass

    def get_name(self):
        return "broken joystick"

    def get_button(self, _index):
        raise RuntimeError("synthetic joystick failure")


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

    def _load_module(
        self,
        argv=(),
        module_name="_control_dualsense_under_test",
        *,
        input_side_effect=KeyboardInterrupt,
        before_main=None,
        fake_socket=None,
    ):
        sys.modules.pop(module_name, None)
        spec = importlib.util.spec_from_file_location(module_name, CONTROL_PATH)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module

        fake_pygame = types.ModuleType("pygame")
        if fake_socket is None:
            fake_socket = _FakeSocket()
        fake_csv_file = io.StringIO()
        with (
            mock.patch.dict(sys.modules, {"pygame": fake_pygame}),
            mock.patch.object(socket, "socket", return_value=fake_socket),
            mock.patch.object(pathlib.Path, "mkdir"),
            mock.patch.object(pathlib.Path, "open", return_value=fake_csv_file),
            mock.patch.object(threading, "Thread", _FakeThread),
            mock.patch.object(builtins, "input", side_effect=input_side_effect),
            mock.patch.object(
                sys, "argv", [str(CONTROL_PATH), *argv],
            ),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            spec.loader.exec_module(module)
            if before_main is not None:
                before_main(module)
            module.main()
        return module, fake_socket

    def _load_module_armed(self, *, input_side_effect):
        module_name = "_control_dualsense_armed_under_test"

        def arm_locally(module):
            module.is_armed = True
            module.is_streaming = True
            module.arm_state = "armed"

        module, fake_socket = self._load_module(
            module_name=module_name,
            input_side_effect=input_side_effect,
            before_main=arm_locally,
        )
        self.addCleanup(sys.modules.pop, module_name, None)
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
        # setUp stamps telemetry with the real clock; restate it on the mocked one
        self.module.last_telem_time = 9.5
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
        # setUp stamps telemetry with the real clock; restate it on the mocked one
        self.module.last_telem_time = 9.5
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
        # setUp stamps telemetry with the real clock; restate it on the mocked one
        self.module.last_telem_time = 9.5
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
        # setUp stamps telemetry with the real clock; restate it on the mocked one
        self.module.last_telem_time = 9.5
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
        # setUp stamps telemetry with the real clock; restate it on the mocked one
        self.module.last_telem_time = 9.5
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

    def test_status_line_shows_flow_hold_state_when_present(self):
        status = self.module.format_status_telemetry(
            _telemetry_sample(PosHold_State=1, AltHold_State=2, Height_M=0.48)
        )

        self.assertIn("PosHold=ACTIVE", status)
        self.assertIn("AltHold=ADJUST", status)
        self.assertIn("H=0.48", status)

    def test_status_line_omits_flow_hold_for_main_firmware(self):
        status = self.module.format_status_telemetry(_telemetry_sample())

        self.assertNotIn("PosHold=", status)

    def test_stdin_flow_hold_commands_pass_through(self):
        commands = []
        self.module.send_cmd = commands.append

        for command in ("poshold 1", "althold 0", "flowcal 1", "hg kv 6"):
            self.module.handle_stdin_command(command)

        self.assertEqual(
            commands, ["poshold 1", "althold 0", "flowcal 1", "hg kv 6"]
        )

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
        # setUp stamps telemetry with the real clock; restate it on the mocked one
        self.module.last_telem_time = 9.5
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

    def test_armed_telemetry_after_the_deadline_aborts_instead_of_confirming(self):
        # 수신 루프는 recvfrom() 전에 deadline을 검사한다. 그 검사(11.99)와
        # 패킷 처리(12.05) 사이에 deadline(12.00)이 지나면 Armed=1 패킷이
        # 늦게 도착해도 성공으로 처리되면 안 된다.
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = False
        self.module.is_streaming = False
        self._prime_prearm_telemetry()
        # setUp stamps telemetry with the real clock; restate it on the mocked one
        self.module.last_telem_time = 9.5
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.assertTrue(self.module.arm())

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.check_arm_timeout(now=11.99)
            self.module.advance_arm_attempt(
                _telemetry_sample(
                    Armed=1,
                    Calibration_OK=1,
                    Fault_Critical=0,
                    Failsafe_Phase=0,
                ),
                now=12.05,
            )

        self.assertEqual(commands, ["mag 1", "start", "stop"])
        self.assertEqual(self.module.arm_state, "idle")
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)
        self.assertEqual(self.module.current_throttle, 1000)
        self.assertIn("시간 초과", output.getvalue())
        self.assertNotIn("ARMED 확인", output.getvalue())

    def test_receive_loop_judges_arm_deadline_at_decision_time_not_receipt(self):
        # 패킷은 deadline(12.00) 전인 11.99에 받았지만, 트림 재전송 등으로
        # 판정이 12.05까지 밀리면 수신 시각이 아니라 판정 시각으로 본다.
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = False
        self.module.is_streaming = False
        self._prime_prearm_telemetry()
        # setUp stamps telemetry with the real clock; restate it on the mocked one
        self.module.last_telem_time = 9.5
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.assertTrue(self.module.arm())

        self.module.trim_synced = True
        self.module.trim_roll = 1.0
        self.module.trim_pitch = 0.0
        self.module.last_trim_resend = 0.0
        clock = {"now": 11.99}

        def send_cmd(command):
            if command.startswith("trim "):
                clock["now"] = 12.05

        with (
            mock.patch.object(
                self.module.time, "monotonic", side_effect=lambda: clock["now"],
            ),
            mock.patch.object(self.module, "send_cmd", side_effect=send_cmd),
        ):
            output = self._run_telemetry([
                _telemetry_sample(
                    Armed=1,
                    Calibration_OK=1,
                    Fault_Critical=0,
                    Failsafe_Phase=0,
                    Trim_Roll=0.0,
                    Trim_Pitch=0.0,
                ),
            ])

        self.assertEqual(commands, ["mag 1", "start", "stop"])
        self.assertEqual(self.module.arm_state, "idle")
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)
        self.assertIn("시간 초과", output)
        self.assertNotIn("ARMED 확인", output)

    def test_disarmed_telemetry_retries_start_at_a_bounded_cadence(self):
        reliable_commands = []
        direct_commands = []
        self.module.reliable_send = reliable_commands.append
        self.module.send_cmd = direct_commands.append
        self.module.is_armed = False
        self.module.is_streaming = False
        self._prime_prearm_telemetry()
        # setUp stamps telemetry with the real clock; restate it on the mocked one
        self.module.last_telem_time = 9.5
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
        # setUp stamps telemetry with the real clock; restate it on the mocked one
        self.module.last_telem_time = 122.5

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

    # --- worker wrapper, fault latch, EOF, port check, UI routing ---

    def _run_telemetry_wrapped(self, samples):
        """Run the wrapped telemetry worker; csv_writer is left to the caller."""
        self.module.sock = _FakeSocket([b"packet"] * len(samples))
        self.module.csv_file = types.SimpleNamespace(flush=lambda: None)
        output = io.StringIO()
        with (
            mock.patch.object(
                self.module, "parse_telemetry_packet", side_effect=samples,
            ),
            mock.patch.object(self.module, "sample_to_csv_row", return_value=[]),
            contextlib.redirect_stdout(output),
        ):
            try:
                self.module.telemetry_worker()
            except _LoopComplete:
                pass
        return output.getvalue()

    def _break_csv_writer(self):
        self.module.csv_writer = types.SimpleNamespace(
            writerow=mock.Mock(side_effect=OSError("disk")),
        )

    def _run_controller_with_joystick(self, joystick, iterations=2):
        event = _LoopEvent(iterations)
        self.module.pygame = _pygame_for(joystick, event)
        output = io.StringIO()
        with (
            mock.patch.object(self.module.time, "sleep", return_value=None),
            contextlib.redirect_stdout(output),
        ):
            try:
                self.module.controller_worker()
            except _LoopComplete:
                pass
        return output.getvalue()

    def test_worker_exception_stops_rc_streaming(self):
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.arm_state = "armed"
        self._break_csv_writer()

        out = self._run_telemetry_wrapped([_telemetry_sample(Failsafe_Phase=0)])

        self.assertFalse(self.module.is_streaming)
        self.assertIn("수신 스레드 오류", out)
        self.assertIn(
            "[ERR] 수신 스레드 오류: OSError: disk - rc 중단, 드론 자동착륙에 맡김. "
            "지상국을 다시 시작하세요",
            out,
        )
        self.assertIn(">>> [SYSTEM] 로컬 해제 (수신 스레드 오류) - stop 미전송", out)
        self.assertEqual(self.module.snapshot()["worker_fault"], "telemetry")
        # stop_streaming_only semantics: never a stop while the drone may be descending
        self.assertNotIn(b"stop", [data for data, _ in self.module.sock.sent])

    def test_controller_exception_stops_rc_streaming(self):
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.arm_state = "armed"
        self.module.sock = _FakeSocket()

        out = self._run_controller_with_joystick(_RaisingJoystick())

        self.assertFalse(self.module.is_streaming)
        self.assertIn(
            "[ERR] 조종 스레드 오류: RuntimeError: synthetic joystick failure - "
            "rc 중단, 드론 자동착륙에 맡김. 지상국을 다시 시작하세요",
            out,
        )
        self.assertEqual(self.module.snapshot()["worker_fault"], "controller")
        sent = [data for data, _ in self.module.sock.sent]
        self.assertNotIn(b"stop", sent)
        self.assertFalse(any(data.startswith(b"rcr") for data in sent))

    def test_worker_exception_cancels_a_resume_attempt(self):
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.arm_state = "armed"
        self.module.resume_state = "wait_rc"
        self.module.resume_deadline = self.module.time.monotonic() + 10.0
        self._break_csv_writer()

        out = self._run_telemetry_wrapped([_telemetry_sample(Failsafe_Phase=0)])

        self.assertEqual(self.module.resume_state, "idle")
        self.assertIn("[RESUME] 실패: 수신 스레드 오류로 복구 시도 취소", out)

    def test_worker_fault_latch_blocks_resume_and_rc(self):
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.arm_state = "armed"
        self._break_csv_writer()
        self._run_telemetry_wrapped([_telemetry_sample(Failsafe_Phase=0)])
        self.assertFalse(self.module.is_streaming)

        # every precondition for a resume is otherwise satisfied
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.last_telem_time = self.module.time.monotonic()
        latched_socket = _FakeSocket()
        self.module.sock = latched_socket
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            accepted = self.module.request_resume("패드")
            self.module.handle_stdin_command("resume")
            self.module.handle_stdin_command("rcr 1 0.00 0.00 0.0")
            self.module.handle_stdin_command("rc 1500 1500 1500 1500")
        self.assertFalse(accepted)
        self.assertFalse(self.module.is_streaming)
        self.assertEqual(self.module.resume_state, "idle")
        self.assertIn("[RESUME] 실패: 수신 스레드 오류로 잠김", output.getvalue())

        # even if streaming were somehow re-opened, no rc/rcr leaves the station
        self.module.is_streaming = True
        self._run_controller_with_joystick(_Joystick(_LoopEvent(0)), iterations=3)
        sent = [data for data, _ in latched_socket.sent]
        self.assertEqual(sent, [])
        self.module.send_cmd("rcr 9 0.00 0.00 0.0")
        self.module.send_cmd("rc 1500 1500 1500 1500")
        self.assertEqual(latched_socket.sent, [])
        # the latch blocks only rc/rcr, not other commands
        self.module.send_cmd("gains")
        self.assertEqual([d for d, _ in latched_socket.sent], [b"gains"])

    def test_controller_skips_rcr_while_latched_even_with_a_raw_sender(self):
        # the controller's own guard, independent of the send_cmd gate
        commands = []
        self.module.send_cmd = commands.append
        self.module.worker_fault = "telemetry"
        self.module.is_streaming = True
        self._run_controller(iterations=3)
        self.assertFalse(any(c.startswith("rcr") for c in commands))

    def test_worker_fault_latch_refuses_arm_without_sending(self):
        self._prime_prearm_telemetry()
        self.module.worker_fault = "controller"
        latched_socket = _FakeSocket()
        self.module.sock = latched_socket
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            accepted = self.module.arm()
        self.assertFalse(accepted)
        self.assertEqual(self.module.arm_state, "idle")
        self.assertEqual(latched_socket.sent, [])
        self.assertIn("[ARM] 거부: 조종 스레드 오류로 잠김", output.getvalue())

    def test_worker_fault_during_arming_blocks_arm_confirmation(self):
        self._prime_prearm_telemetry()
        commands = []
        self.module.reliable_send = commands.append
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(self.module.arm())
        self.assertEqual(self.module.arm_state, "arming")
        self.module.sock = _FakeSocket()
        commands.clear()

        out = self._run_controller_with_joystick(_RaisingJoystick())
        self.assertEqual(self.module.snapshot()["worker_fault"], "controller")
        # the arm attempt is cancelled the way every other arm cancel is: stop
        self.assertEqual(commands, ["stop"])
        self.assertIn("[ARM] 실패: 조종 스레드 오류로 무장 시도 취소, stop 전송", out)

        confirm_socket = _FakeSocket()
        self.module.sock = confirm_socket
        self.module.csv_writer = types.SimpleNamespace(writerow=lambda _row: None)
        self._run_telemetry_wrapped([
            _telemetry_sample(Armed=1, Failsafe_Phase=0),
            _telemetry_sample(Armed=0, Failsafe_Phase=0),
        ])
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)
        self.assertNotIn(b"start", [d for d, _ in self.module.sock.sent])

    def test_latched_advance_arm_attempt_neither_confirms_nor_retries(self):
        commands = []
        self.module.send_cmd = commands.append
        self.module.reliable_send = commands.append
        self.module.arm_state = "arming"
        self.module.arm_deadline = 100.0
        self.module.arm_next_retry = 0.0
        self.module.worker_fault = "telemetry"

        self.module.advance_arm_attempt(_telemetry_sample(Armed=1), now=10.0)
        self.module.advance_arm_attempt(_telemetry_sample(Armed=0), now=10.5)

        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)
        self.assertNotIn("start", commands)

    def test_ui_failure_does_not_skip_safety(self):
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.arm_state = "armed"
        self.module.ui = _RaisingUI()
        self._break_csv_writer()

        # must return normally even though every UI call raises
        self._run_telemetry_wrapped([_telemetry_sample(Failsafe_Phase=0)])

        self.assertFalse(self.module.is_streaming)
        self.assertEqual(self.module.snapshot()["worker_fault"], "telemetry")

        self.module.is_streaming = True
        self.module.sock = _FakeSocket()
        self._run_controller_with_joystick(_RaisingJoystick())
        self.assertFalse(self.module.is_streaming)
        # the first fault stays latched
        self.assertEqual(self.module.worker_fault, "telemetry")

    def test_eof_disarms_when_armed(self):
        m, sock = self._load_module_armed(input_side_effect=EOFError)
        self.assertTrue(any(d == b"stop" for d, _ in sock.sent))
        self.assertFalse(m.is_armed)

    def test_eof_does_not_send_stop_when_disarmed(self):
        self.assertFalse(any(d == b"stop" for d, _ in self.startup_socket.sent))
        module_name = "_control_dualsense_eof_under_test"
        _module, sock = self._load_module(
            module_name=module_name, input_side_effect=EOFError,
        )
        self.addCleanup(sys.modules.pop, module_name, None)
        self.assertFalse(any(d == b"stop" for d, _ in sock.sent))

    def test_keyboard_interrupt_still_disarms_when_armed(self):
        _m, sock = self._load_module_armed(input_side_effect=KeyboardInterrupt)
        self.assertTrue(any(d == b"stop" for d, _ in sock.sent))

    def test_bind_failure_exits_before_creating_log(self):
        module_name = "_control_dualsense_bind_under_test"
        sys.modules.pop(module_name, None)
        spec = importlib.util.spec_from_file_location(module_name, CONTROL_PATH)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        self.addCleanup(sys.modules.pop, module_name, None)
        output = io.StringIO()
        with (
            mock.patch.dict(sys.modules, {"pygame": types.ModuleType("pygame")}),
            mock.patch.object(socket, "socket", return_value=_BindFailSocket()),
            mock.patch.object(pathlib.Path, "mkdir") as mkdir,
            mock.patch.object(pathlib.Path, "open") as path_open,
            mock.patch.object(threading, "Thread", _FakeThread),
            mock.patch.object(sys, "argv", [str(CONTROL_PATH)]),
            contextlib.redirect_stdout(output),
            self.assertRaises(SystemExit) as raised,
        ):
            spec.loader.exec_module(module)
        self.assertEqual(raised.exception.code, 1)
        path_open.assert_not_called()
        mkdir.assert_not_called()
        self.assertIn(
            "[ERR] 포트 4210을 다른 지상국이 쓰고 있음 - 먼저 종료하세요",
            output.getvalue(),
        )

    def test_state_transitions_reach_ui(self):
        recording = _RecordingUI()
        self.module.ui = recording
        self._run_telemetry([
            _telemetry_sample(Failsafe_Phase=0, Armed=0),
            _telemetry_sample(Failsafe_Phase=1, Armed=0),
            _telemetry_sample(Failsafe_Phase=1, Armed=0),
            _telemetry_sample(Failsafe_Phase=0, Armed=0),
        ])
        self.assertEqual(
            recording.texts("transition"),
            [
                "[STATE] Failsafe_Phase 0 → 1",
                "[STATE] Failsafe_Phase 1 → 0",
            ],
        )

    def test_each_watched_state_field_reports_its_transition(self):
        for field in (
            "Failsafe_Phase",
            "Failsafe_Probe_State",
            "Yaw_Authority_State",
            "PosHold_State",
            "AltHold_State",
        ):
            with self.subTest(field=field):
                recording = _RecordingUI()
                self.module.ui = recording
                base = {
                    "Failsafe_Phase": 0, "Failsafe_Probe_State": 0,
                    "Yaw_Authority_State": 0, "PosHold_State": 0,
                    "AltHold_State": 0, "Armed": 0,
                }
                changed = dict(base, **{field: 2})
                self._run_telemetry([
                    _telemetry_sample(**base),
                    _telemetry_sample(**changed),
                ])
                self.assertEqual(
                    recording.texts("transition"), [f"[STATE] {field} 0 → 2"],
                )

    def test_periodic_lines_go_through_ui_periodic(self):
        recording = _RecordingUI()
        self.module.ui = recording
        self.module.is_armed = True
        self.module.last_arm_time = self.module.time.monotonic()
        self._run_telemetry([_telemetry_sample()] * 4)
        periodic = recording.texts("periodic")
        self.assertTrue(any(t.startswith("[STATUS]") for t in periodic))
        self.assertTrue(any(t.startswith("[AUTO-LAND]") for t in periodic))
        self.assertTrue(any(t.startswith(" [DIAG]") for t in periodic))
        self.assertTrue(recording.texts("update"))
        events = recording.texts("event")
        self.assertIn("[TELEM] 수신 스레드 시작", events)
        self.assertFalse(any("[STATUS]" in t or "[DIAG]" in t for t in events))

    def test_snapshot_exposes_the_dashboard_fields(self):
        self.module.bad_packet_count = 0
        self._run_telemetry([_telemetry_sample(Failsafe_Phase=0, Armed=0)])
        snap = self.module.snapshot()
        for key in (
            "arm_state", "is_armed", "is_streaming", "current_throttle",
            "trim_roll", "trim_pitch", "telem_total_pkts", "telem_dropped_pkts",
            "bad_packet_count", "last_telem_time", "log_path", "mode",
            "worker_fault", "last_sample", "raw_capture_enabled",
            "raw_batch_count", "raw_last_dropped", "raw_last_receive_time",
        ):
            self.assertIn(key, snap)
        self.assertEqual(snap["bad_packet_count"], 0)
        self.assertIsNone(snap["worker_fault"])
        self.assertEqual(snap["mode"], "pad")
        self.assertEqual(snap["last_sample"]["Failsafe_Phase"], 0)

    def test_station_imports_without_prompt_toolkit(self):
        module_name = "_control_dualsense_no_ptk_under_test"
        with mock.patch.dict(sys.modules, {"prompt_toolkit": None}):
            module, _sock = self._load_module(module_name=module_name)
        self.addCleanup(sys.modules.pop, module_name, None)
        self.assertEqual(module._format_optional_int(3), "3")

    def test_snapshot_armed_since_follows_the_arm_time(self):
        self.module.is_armed = False
        self.module.last_arm_time = 123.0
        self.assertIsNone(self.module.snapshot()["armed_since"])
        self.module.is_armed = True
        self.assertEqual(self.module.snapshot()["armed_since"], 123.0)

    def test_main_routes_input_through_the_ui_and_closes_the_log(self):
        module_name = "_control_dualsense_main_under_test"
        module, _sock = self._load_module(module_name=module_name)
        self.addCleanup(sys.modules.pop, module_name, None)
        recording = _RecordingUI()
        module.csv_file = io.StringIO()
        with (
            mock.patch.object(threading, "Thread", _FakeThread),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            module.main(ui=recording)
        kinds = [kind for kind, _ in recording.calls]
        self.assertIn("run", kinds)
        self.assertLess(kinds.index("run"), kinds.index("after_run"))
        self.assertIs(module.ui, recording)
        self.assertTrue(module.csv_file.closed)
        self.assertTrue(
            any(t.startswith("[LOG] 저장 완료:") for t in recording.texts("event"))
        )

    def test_plain_flag_is_parsed(self):
        self.assertTrue(self.module._parse_cli_args(["--plain"]).plain)
        self.assertFalse(self.module._parse_cli_args([]).plain)

    def test_main_without_ui_builds_it_with_make_ui_and_the_startup_help(self):
        module_name = "_control_dualsense_make_ui_under_test"
        module, _sock = self._load_module(module_name=module_name)
        self.addCleanup(sys.modules.pop, module_name, None)
        recording = _RecordingUI()
        module.csv_file = io.StringIO()
        with (
            mock.patch.object(module, "make_ui", return_value=recording) as factory,
            mock.patch.object(threading, "Thread", _FakeThread),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            module.main(["--plain"])
        factory.assert_called_once()
        args, kwargs = factory.call_args
        self.assertTrue(args[0].plain)
        self.assertEqual(list(kwargs["help_lines"]), module.startup_help_lines())
        self.assertIs(module.ui, recording)
        kinds = [kind for kind, _ in recording.calls]
        self.assertLess(kinds.index("run"), kinds.index("after_run"))

    def test_controller_prints_the_startup_help_from_one_place(self):
        recording = _RecordingUI()
        self.module.ui = recording
        self._run_controller(iterations=1)
        help_lines = self.module.startup_help_lines()
        self.assertEqual(help_lines[0], "========== DRONE CONTROLLER ==========")
        self.assertTrue(any(line.startswith(" PID:") for line in help_lines))
        self.assertTrue(any(line.startswith(" FlowHold:") for line in help_lines))
        self.assertTrue(any(line.startswith(" IMU cal:") for line in help_lines))
        self.assertEqual(recording.texts("event")[:len(help_lines)], help_lines)

    # --- review round 1 regressions ---

    def test_rc_send_cannot_interleave_with_the_latch_write(self):
        # Pause send_cmd between its latch check and sendto (inside encode),
        # write the latch from another thread, then let the send finish.
        module = self.module
        entered = threading.Event()
        release = threading.Event()
        observed = []

        class _PausingCmd(str):
            def encode(self, *args, **kwargs):
                entered.set()
                release.wait(timeout=2.0)
                return str.encode(self, *args, **kwargs)

        class _ObservingSocket(_FakeSocket):
            def sendto(self, data, address):
                observed.append((data, module.worker_fault))
                super().sendto(data, address)

        module.sock = _ObservingSocket()
        module.ui = _RecordingUI()
        sender = threading.Thread(
            target=module.send_cmd,
            args=(_PausingCmd("rcr 1 0.00 0.00 0.0"),),
            daemon=True,
        )
        sender.start()
        self.assertTrue(entered.wait(timeout=1.0))
        latcher = threading.Thread(
            target=module._handle_worker_fault,
            args=("telemetry", RuntimeError("synthetic")),
            daemon=True,
        )
        latcher.start()
        latcher.join(timeout=0.2)
        release.set()
        sender.join(timeout=1.0)
        latcher.join(timeout=1.0)
        self.assertFalse(sender.is_alive())
        self.assertFalse(latcher.is_alive())
        self.assertEqual(module.worker_fault, "telemetry")

        module.send_cmd("rcr 2 0.00 0.00 0.0")
        rc_sends = [(d, f) for d, f in observed if d.startswith(b"rcr")]
        # an rcr may leave only if the latch was not yet written when it left
        self.assertTrue(all(fault is None for _d, fault in rc_sends), rc_sends)
        self.assertLessEqual(len(rc_sends), 1)

    def test_latch_written_before_the_rc_lock_is_taken_drops_the_send(self):
        # Pause the sender just before it takes rc_send_lock, write the latch,
        # then let it continue: the latch check must happen inside the lock.
        module = self.module
        entered = threading.Event()
        release = threading.Event()
        observed = []

        class _PausingLock:
            def __init__(self, inner):
                self.inner = inner
                self.pause_thread = None

            def __enter__(self):
                if threading.current_thread() is self.pause_thread:
                    entered.set()
                    release.wait(timeout=2.0)
                self.inner.acquire()
                return self

            def __exit__(self, *_exc):
                self.inner.release()
                return False

        class _ObservingSocket(_FakeSocket):
            def sendto(self, data, address):
                observed.append((data, module.worker_fault))
                super().sendto(data, address)

        pausing = _PausingLock(module.rc_send_lock)
        module.rc_send_lock = pausing
        module.sock = _ObservingSocket()
        module.ui = _RecordingUI()
        sender = threading.Thread(
            target=module.send_cmd, args=("rcr 1 0.00 0.00 0.0",), daemon=True,
        )
        pausing.pause_thread = sender
        sender.start()
        self.assertTrue(entered.wait(timeout=1.0))
        latcher = threading.Thread(
            target=module._handle_worker_fault,
            args=("controller", RuntimeError("synthetic")),
            daemon=True,
        )
        latcher.start()
        latcher.join(timeout=1.0)
        self.assertFalse(latcher.is_alive())
        release.set()
        sender.join(timeout=1.0)
        self.assertFalse(sender.is_alive())
        self.assertEqual(
            [(d, f) for d, f in observed if d.startswith(b"rcr")], [],
        )

    def _state_observing_ui(self, sent_socket):
        module = self.module
        observed = []

        class _StateObservingUI(_RecordingUI):
            def event(self, text):
                observed.append((
                    text,
                    module.is_streaming,
                    module.is_armed,
                    b"stop" in [d for d, _ in sent_socket.sent],
                ))
                super().event(text)

        return _StateObservingUI(), observed

    def _fault_line(self, observed, needle):
        lines = [o for o in observed if needle in o[0]]
        self.assertEqual(len(lines), 1, observed)
        return lines[0]

    def test_telemetry_loss_fault_is_shown_only_after_rc_stops(self):
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.arm_state = "armed"
        self.module.last_telem_time = self.module.time.monotonic() - 5.0
        sock = _FakeSocket([])
        recording, observed = self._state_observing_ui(sock)
        self.module.ui = recording
        self.module.sock = sock
        with self.assertRaises(_LoopComplete):
            self.module.telemetry_thread()
        _text, streaming, _armed, _stop = self._fault_line(
            observed, "[FAULT] 텔레메트리",
        )
        self.assertFalse(streaming)
        # console order is unchanged: [FAULT] first, then the local release line
        texts = [o[0] for o in observed]
        self.assertLess(
            texts.index(_text),
            [i for i, t in enumerate(texts) if "로컬 해제" in t][0],
        )

    def _run_telemetry_observing(self, samples):
        self.module.sock = _FakeSocket([b"packet"] * len(samples))
        recording, observed = self._state_observing_ui(self.module.sock)
        self.module.ui = recording
        self.module.csv_writer = types.SimpleNamespace(writerow=lambda _row: None)
        self.module.csv_file = types.SimpleNamespace(flush=lambda: None)
        with (
            mock.patch.object(
                self.module, "parse_telemetry_packet", side_effect=samples,
            ),
            mock.patch.object(self.module, "sample_to_csv_row", return_value=[]),
            mock.patch.object(self.module.time, "sleep", return_value=None),
            self.assertRaises(_LoopComplete),
        ):
            self.module.telemetry_thread()
        return observed

    def test_rc_timeout_fault_is_shown_only_after_rc_stops(self):
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.arm_state = "armed"
        self.module.last_arm_time = self.module.time.monotonic()
        observed = self._run_telemetry_observing(
            [_telemetry_sample(Fault_RC=1, Failsafe_Phase=1, Armed=1)],
        )
        _text, streaming, _armed, _stop = self._fault_line(
            observed, "[FAULT] 드론: RC 타임아웃",
        )
        self.assertFalse(streaming)

    def test_critical_fault_is_shown_only_after_disarm_sent_stop(self):
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.arm_state = "armed"
        self.module.last_arm_time = self.module.time.monotonic()
        observed = self._run_telemetry_observing(
            [_telemetry_sample(Fault_Critical=1, Failsafe_Phase=0, Armed=1)],
        )
        _text, streaming, armed, stop_sent = self._fault_line(
            observed, "[FAULT] 드론: 치명 고장",
        )
        self.assertFalse(armed)
        self.assertFalse(streaming)
        self.assertTrue(stop_sent)

    def test_drone_not_armed_fault_is_shown_only_after_disarm_sent_stop(self):
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.arm_state = "armed"
        self.module.last_arm_time = self.module.time.monotonic() - 10.0
        observed = self._run_telemetry_observing(
            [_telemetry_sample(Armed=0, Failsafe_Phase=0)],
        )
        _text, _streaming, armed, stop_sent = self._fault_line(
            observed, "[FAULT] 드론이 시동 상태가 아님",
        )
        self.assertFalse(armed)
        self.assertTrue(stop_sent)

    def test_resume_rejections_call_the_ui_without_holding_state_locks(self):
        module = self.module
        held = []

        class _LockCheckingUI(_RecordingUI):
            def event(self, text):
                for name in ("safety_cmd_lock", "telem_lock"):
                    lock = getattr(module, name)
                    if lock.acquire(blocking=False):
                        lock.release()
                    else:
                        held.append((name, text))
                super().event(text)

        recording = _LockCheckingUI()
        module.ui = recording
        commands = []
        module.reliable_send = commands.append

        def ready(**overrides):
            module.resume_state = "idle"
            module.worker_fault = None
            module.is_armed = True
            module.telem_failsafe_phase = 1
            module.telem_hover_est = 1360.0
            module.telem_hover_valid = True
            module.last_telem_time = module.time.monotonic()
            for name, value in overrides.items():
                setattr(module, name, value)

        for overrides in (
            {"worker_fault": "telemetry"},
            {"resume_state": "wait_rc"},
            {"is_armed": False},
            {"telem_failsafe_phase": 0},
            {"last_telem_time": 0.0},
            {"telem_hover_valid": False},
        ):
            ready(**overrides)
            self.assertFalse(module.request_resume("test"))

        # accepted request, then the "RC accepted, resume sent" event
        ready()
        module.telem_total_pkts = 40
        module.telem_dropped_pkts = 3
        self.assertTrue(module.request_resume("test"))
        module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=42, RC_Dropped_Pkts=3),
            now=module.resume_deadline - 0.1,
        )
        self.assertEqual(commands, ["resume"])
        self.assertEqual(len(recording.texts("event")), 8)
        self.assertEqual(held, [])

    def test_no_ui_call_is_lexically_inside_a_state_lock(self):
        import ast

        tree = ast.parse(CONTROL_PATH.read_text(encoding="utf-8"))
        lock_names = {"safety_cmd_lock", "telem_lock", "rc_send_lock"}
        offenders = []

        def visit(node, locked):
            if isinstance(node, ast.With):
                names = {
                    item.context_expr.id
                    for item in node.items
                    if isinstance(item.context_expr, ast.Name)
                }
                locked = locked or bool(names & lock_names)
            if (
                locked
                and isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "ui"
            ):
                offenders.append(node.lineno)
            for child in ast.iter_child_nodes(node):
                visit(child, locked)

        visit(tree, False)
        self.assertEqual(offenders, [])

    # --- Task 7: --keyboard / --listen modes and bad-packet counting ---

    KEYBOARD_REFUSAL = "[STDIN] 키보드 모드: 시동·복구는 패드로만 합니다"
    LISTEN_REFUSAL = "[STDIN] 수신 전용 모드"

    def _load_mode_module(self, argv, name, **kwargs):
        module_name = f"_control_dualsense_{name}_under_test"
        module, sock = self._load_module(
            argv=argv, module_name=module_name, **kwargs,
        )
        self.addCleanup(sys.modules.pop, module_name, None)
        module.shutdown_event.clear()
        return module, sock

    @staticmethod
    def _arm_locally(module):
        module.is_armed = True
        module.is_streaming = True
        module.arm_state = "armed"

    def _started_threads(self, module, recording):
        started = []

        class _Recording(_FakeThread):
            def __init__(self, *_, target=None, **__):
                self.target = target

            def start(self):
                started.append(self.target)

        with (
            mock.patch.object(threading, "Thread", _Recording),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            module.main(ui=recording)
        return started

    def _replay(self, module, samples):
        """Feed parsed samples through telemetry_thread on the module's socket."""
        rows = []
        module.csv_writer = types.SimpleNamespace(writerow=rows.append)
        module.csv_file = types.SimpleNamespace(flush=lambda: None)
        module.sock.packets = iter([b"packet"] * len(samples))
        with (
            mock.patch.object(module, "parse_telemetry_packet", side_effect=samples),
            mock.patch.object(module, "sample_to_csv_row", return_value=[]),
            contextlib.redirect_stdout(io.StringIO()),
            self.assertRaises(_LoopComplete),
        ):
            module.telemetry_thread()
        return rows

    def test_modes_are_parsed_and_mutually_exclusive(self):
        parse = self.module._parse_cli_args
        self.assertTrue(parse(["--keyboard"]).keyboard)
        self.assertTrue(parse(["--listen", "--no-raw"]).listen)
        self.assertEqual(self.module.mode, "pad")
        with (
            contextlib.redirect_stderr(io.StringIO()),
            self.assertRaises(SystemExit),
        ):
            parse(["--keyboard", "--listen"])

    def test_keyboard_mode_refuses_start_and_resume(self):
        m, sock = self._load_mode_module(("--keyboard",), "kb_refuse")
        self.assertEqual(m.mode, "keyboard")
        recording = _RecordingUI()
        m.ui = recording
        # 무장·복구가 가능한 상태로 만들어 두어도 키보드 모드는 거부한다.
        m.last_telem_time = m.time.monotonic()
        m.telem_calibration_ok = 1
        m.telem_armed = 0
        m.telem_fault_critical = 0
        for cmd in ("start", "resume", "START", "Resume", "start 1"):
            m.handle_stdin_command(cmd)
        sent = [d for d, _ in sock.sent]
        self.assertFalse(
            any(d.split()[0].lower() in (b"start", b"resume") for d in sent if d),
            sent,
        )
        self.assertFalse(any(d.startswith(b"mag ") for d in sent), sent)
        self.assertEqual(m.arm_state, "idle")
        self.assertEqual(m.resume_state, "idle")
        self.assertEqual(recording.texts("event"), [self.KEYBOARD_REFUSAL] * 5)

    def test_keyboard_mode_still_passes_settings_and_stop_through(self):
        m, sock = self._load_mode_module(("--keyboard",), "kb_pass")
        m.ui = _RecordingUI()
        with mock.patch.object(m.time, "sleep", return_value=None):
            m.handle_stdin_command("pr 0.9")
            m.handle_stdin_command("stop")
        sent = [d for d, _ in sock.sent]
        self.assertIn(b"pr 0.9", sent)
        self.assertIn(b"stop", sent)

    def test_keyboard_mode_sends_nothing_on_exit(self):
        for index, exit_error in enumerate((KeyboardInterrupt, EOFError)):
            with self.subTest(exit=exit_error.__name__):
                # 로컬 무장 상태를 억지로 만들어도 종료 시 stop을 보내지 않는다:
                # 이전 패드 세션이 남긴 자동착륙 하강 중일 수 있다.
                m, sock = self._load_mode_module(
                    ("--keyboard",), f"kb_exit_{index}",
                    input_side_effect=exit_error,
                    before_main=self._arm_locally,
                )
                self.assertEqual(sock.sent, [])

    def test_keyboard_mode_does_not_start_controller_thread(self):
        m, _sock = self._load_mode_module(("--keyboard",), "kb_threads")
        recording = _RecordingUI()
        started = self._started_threads(m, recording)
        self.assertEqual(started, [m.telemetry_worker])
        # 패드가 없으니 입력줄 명령 목록은 main이 띄운다.
        events = recording.texts("event")
        self.assertEqual(events[:len(m.startup_help_lines())], m.startup_help_lines())

    def test_pad_mode_still_starts_both_workers(self):
        started = self._started_threads(self.module, _RecordingUI())
        self.assertEqual(
            started, [self.module.telemetry_worker, self.module.controller_worker],
        )

    def test_keyboard_mode_creates_no_log_files(self):
        module_name = "_control_dualsense_kb_files_under_test"
        sys.modules.pop(module_name, None)
        self.addCleanup(sys.modules.pop, module_name, None)
        spec = importlib.util.spec_from_file_location(module_name, CONTROL_PATH)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        good = ",".join(str(value) for value in range(55)).encode("ascii")
        zimu = (
            struct.pack("<4sBBHIII", b"ZIMU", 1, 1, 0, 1, 1000, 0)
            + struct.pack("<H12h", 0, *range(12))
        )
        recording = _RecordingUI()
        with (
            mock.patch.dict(sys.modules, {"pygame": types.ModuleType("pygame")}),
            mock.patch.object(socket, "socket", return_value=_FakeSocket()),
            mock.patch.object(pathlib.Path, "mkdir") as mkdir,
            mock.patch.object(pathlib.Path, "open") as path_open,
            mock.patch.object(threading, "Thread", _FakeThread),
            mock.patch.object(sys, "argv", [str(CONTROL_PATH), "--keyboard"]),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            spec.loader.exec_module(module)
            module.main(ui=recording)
            module.shutdown_event.clear()
            module.sock = _FakeSocket([zimu, good, b"\xff"])
            with self.assertRaises(_LoopComplete):
                module.telemetry_thread()
            module.shutdown_workers_and_close_log(())
        path_open.assert_not_called()
        mkdir.assert_not_called()
        self.assertIsNone(module.log_path)
        self.assertIsNone(module.raw_log_path)
        self.assertIsNone(module.csv_file)
        self.assertIsNone(module.raw_file)
        self.assertIsNone(module.snapshot()["log_path"])
        self.assertIsNotNone(module.last_sample)
        events = recording.texts("event")
        self.assertFalse(any(t.startswith("[LOG] 저장 완료") for t in events))
        self.assertIn("[LOG] 저장 0개, 잘못된 패킷 1개", events)

    def test_keyboard_mode_trim_follows_the_drone_without_resending(self):
        m, sock = self._load_mode_module(("--keyboard",), "kb_trim")
        m.ui = _RecordingUI()
        m.handle_stdin_command("trim 1 2")
        self._replay(m, [
            _telemetry_sample(Armed=0, Failsafe_Phase=0, Trim_Roll=0.0, Trim_Pitch=0.0),
            _telemetry_sample(Armed=0, Failsafe_Phase=0, Trim_Roll=1.0, Trim_Pitch=2.0),
        ])
        trims = [d for d, _ in sock.sent if d.startswith(b"trim ")]
        self.assertEqual(trims, [b"trim 1 2"])
        self.assertAlmostEqual(m.trim_roll, 1.0)
        self.assertAlmostEqual(m.trim_pitch, 2.0)

    def test_listen_mode_sends_only_connect(self):
        # trim (0,0) → (1,2) 두 패킷 재생 + 입력줄 명령 + --no-raw 시작 + 종료
        for index, exit_error in enumerate((KeyboardInterrupt, EOFError)):
            with self.subTest(exit=exit_error.__name__):
                m, sock = self._load_mode_module(
                    ("--listen", "--no-raw"), f"listen_only_{index}",
                    input_side_effect=exit_error,
                    before_main=self._arm_locally,
                )
                self.assertEqual(m.mode, "listen")
                recording = _RecordingUI()
                m.ui = recording
                m.is_armed = m.is_streaming = False
                m.arm_state = "idle"
                m.trim_synced = False
                rows = self._replay(m, [
                    _telemetry_sample(Armed=0, Failsafe_Phase=0,
                                      Trim_Roll=0.0, Trim_Pitch=0.0),
                    _telemetry_sample(Armed=0, Failsafe_Phase=0,
                                      Trim_Roll=1.0, Trim_Pitch=2.0),
                ])
                commands = ("pr 0.9", "stop", "start", "resume", "th 1200",
                            "raw on", "mag off", "rcr 1 0 0 0", "trim 1 2")
                with mock.patch.object(m.time, "sleep", return_value=None):
                    for command in commands:
                        m.handle_stdin_command(command)
                    # 송신 관문 자체도 connect만 통과시킨다.
                    m.send_cmd("trim 1.00 2.00")
                    m.reliable_send("stop")
                    m.send_trim()
                    self._arm_locally(m)
                    m._on_stdin_interrupt()
                    m._on_stdin_eof()
                self.assertEqual({d for d, _ in sock.sent}, {b"connect"})
                self.assertEqual(
                    recording.texts("event").count(self.LISTEN_REFUSAL),
                    len(commands),
                )
                self.assertEqual(m.current_throttle, 1000)
                self.assertTrue(m.mag_enabled_choice)
                # CSV는 기록한다.
                self.assertEqual(len(rows), 2)
                self.assertIsNotNone(m.log_path)

    def test_listen_mode_no_raw_neither_sends_raw_zero_nor_claims_to(self):
        recording = _RecordingUI()
        m, _sock = self._load_mode_module(("--listen", "--no-raw"), "listen_raw")
        self.assertFalse(m.raw_capture_enabled)
        started = self._started_threads(m, recording)
        self.assertEqual(started, [m.telemetry_worker])
        self.assertFalse(any(t.startswith("[RAW]") for t in recording.texts("event")))

    def test_listen_mode_keeps_raw_logging_on(self):
        m, _sock = self._load_mode_module(("--listen",), "listen_rawlog")
        self.assertTrue(m.raw_capture_enabled)
        self.assertIsNotNone(m.raw_log_path)

    def test_bad_packets_are_counted(self):
        good = ",".join(str(value) for value in range(55)).encode("ascii")
        rows = []
        self.module.bad_packet_count = 0
        self._run_datagrams([b"\xff\xfe", b"not,a,packet", good], rows)
        self.assertEqual(self.module.bad_packet_count, 2)
        self.assertEqual(self.module.snapshot()["bad_packet_count"], 2)
        self.assertEqual(len(rows), 1)

    def test_shutdown_reports_saved_and_bad_packet_counts(self):
        good = ",".join(str(value) for value in range(55)).encode("ascii")
        rows = []
        self.module.bad_packet_count = 0
        self._run_datagrams([good, b"\xff\xfe", good, b"x"], rows)
        recording = _RecordingUI()
        self.module.ui = recording
        self.module.csv_file = io.StringIO()
        self.module.shutdown_workers_and_close_log(())
        self.assertIn("[LOG] 저장 2개, 잘못된 패킷 2개", recording.texts("event"))

    # --- Task 7 review round 1 ---

    def test_malformed_only_traffic_publishes_the_bad_packet_count(self):
        recording = _RecordingUI()
        self.module.ui = recording
        self.module.bad_packet_count = 0
        self._run_datagrams([b"\xff\xfe", b"not,a,packet"], [])
        updates = recording.texts("update")
        self.assertEqual([u["bad_packet_count"] for u in updates], [1, 2])
        # 잘못된 패킷은 유효 수신으로 세지 않는다(링크 Hz가 부풀지 않게).
        self.assertEqual([u["valid_packet_count"] for u in updates], [0, 0])

    def test_valid_packet_count_counts_only_parsed_samples(self):
        good = ",".join(str(value) for value in range(55)).encode("ascii")
        recording = _RecordingUI()
        self.module.ui = recording
        self._run_datagrams([b"\xff", good, b"x", good], [])
        self.assertEqual(self.module.snapshot()["valid_packet_count"], 2)
        self.assertEqual(
            [u["valid_packet_count"] for u in recording.texts("update")],
            [0, 1, 1, 2],
        )

    def test_entry_point_accepts_every_station_option(self):
        # drone_control.py는 station을 import하기 전에 인자를 검사한다.
        # 두 파서가 어긋나면 진입점이 통과시킨 인자를 station이 거부한다.
        import drone_control

        def options(parser):
            return {
                flag for action in parser._actions for flag in action.option_strings
            }

        self.assertLessEqual(
            options(self.module._cli_parser()), options(drone_control._parser()),
        )

    def test_main_refuses_a_mode_other_than_the_import_time_one(self):
        with self.assertRaises(ValueError):
            self.module.main(["--keyboard"], ui=_RecordingUI())

    # --- final whole-branch review ---

    @staticmethod
    def _threading_with(module, thread_class):
        """The module's ``threading`` with only Thread replaced (prompt_toolkit untouched)."""
        return types.SimpleNamespace(
            Thread=thread_class, Event=threading.Event, Lock=threading.Lock,
            local=threading.local, current_thread=threading.current_thread,
            main_thread=threading.main_thread,
        )

    def _dashboard(self):
        from prompt_toolkit.input import create_pipe_input
        from prompt_toolkit.output import DummyOutput
        from ground.station_ui import DashboardUI

        pipe_context = create_pipe_input()
        pipe = pipe_context.__enter__()
        self.addCleanup(pipe_context.__exit__, None, None, None)
        return DashboardUI(input=pipe, output=DummyOutput()), pipe

    @staticmethod
    def _wait(predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return bool(predicate())

    def test_ctrl_c_during_dashboard_build_still_disarms_through_main(self):
        m, sock = self._load_mode_module((), "final_build_interrupt")
        self._arm_locally(m)
        m.csv_file = io.StringIO()
        dash, _pipe = self._dashboard()
        escaped = []
        with (
            mock.patch.object(type(dash), "_build_app", side_effect=KeyboardInterrupt),
            mock.patch.object(m, "threading", self._threading_with(m, _FakeThread)),
            mock.patch.object(m.time, "sleep", return_value=None),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            try:
                m.main(ui=dash)
            except KeyboardInterrupt:
                escaped.append(True)
        self.assertEqual(escaped, [])
        self.assertFalse(m.is_armed)
        self.assertIn(b"stop", [d for d, _ in sock.sent])
        self.assertTrue(m.csv_file.closed)

    def test_any_exception_escaping_ui_run_disarms_before_cleanup(self):
        for index, error in enumerate((KeyboardInterrupt(), RuntimeError("synthetic ui bug"))):
            with self.subTest(error=type(error).__name__):
                m, sock = self._load_mode_module((), f"final_escape_{index}")
                self._arm_locally(m)
                m.csv_file = io.StringIO()
                order = []

                class _EscapingUI(_RecordingUI):
                    def run(self, input_handler, on_interrupt, on_eof):
                        raise error

                    def after_run(self):
                        order.append(("after_run", b"stop" in [d for d, _ in sock.sent]))

                with (
                    mock.patch.object(m, "threading", self._threading_with(m, _FakeThread)),
                    mock.patch.object(m.time, "sleep", return_value=None),
                    contextlib.redirect_stdout(io.StringIO()),
                ):
                    try:
                        if isinstance(error, KeyboardInterrupt):
                            self.assertEqual(m.main(ui=_EscapingUI()), 0)
                        else:
                            with self.assertRaises(RuntimeError):
                                m.main(ui=_EscapingUI())
                    except KeyboardInterrupt:
                        self.fail("KeyboardInterrupt escaped main()")
                self.assertFalse(m.is_armed)
                self.assertEqual(order, [("after_run", True)])
                self.assertTrue(m.csv_file.closed)

    def test_ui_run_escape_in_padless_modes_sends_nothing(self):
        for index, argv in enumerate((("--keyboard",), ("--listen",))):
            with self.subTest(argv=argv):
                m, sock = self._load_mode_module(argv, f"final_escape_mode_{index}")
                self._arm_locally(m)
                sock.sent.clear()

                class _EscapingUI(_RecordingUI):
                    def run(self, input_handler, on_interrupt, on_eof):
                        raise KeyboardInterrupt

                with (
                    mock.patch.object(m, "threading", self._threading_with(m, _FakeThread)),
                    contextlib.redirect_stdout(io.StringIO()),
                ):
                    try:
                        m.main(ui=_EscapingUI())
                    except KeyboardInterrupt:
                        self.fail("KeyboardInterrupt escaped main()")
                self.assertFalse(any(d == b"stop" for d, _ in sock.sent), sock.sent)

    def test_main_publishes_a_snapshot_before_running_the_ui(self):
        recording = _RecordingUI()
        m, _sock = self._load_mode_module((), "final_initial_publish")
        with (
            mock.patch.object(m, "threading", self._threading_with(m, _FakeThread)),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            m.main(ui=recording)
        kinds = [kind for kind, _ in recording.calls]
        self.assertIn("update", kinds)
        self.assertLess(kinds.index("update"), kinds.index("run"))
        self.assertEqual(recording.texts("update")[0]["mode"], "pad")

    def test_dashboard_shows_local_safety_changes_without_a_datagram(self):
        # Real station state and snapshot(), real DashboardUI, wired by main().
        m, _sock = self._load_mode_module((), "final_dashboard_pull")
        self._arm_locally(m)
        m.current_throttle = 1400
        m.last_sample = _telemetry_sample(Failsafe_Phase=0)
        m.last_telem_time = m.time.monotonic()
        m.csv_file = io.StringIO()
        m.sock = _FakeSocket()
        dash, pipe = self._dashboard()
        result = []

        def run_main():
            with mock.patch.object(m, "threading", self._threading_with(m, _FakeThread)):
                result.append(m.main(ui=dash))

        runner = threading.Thread(target=run_main, daemon=True)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            runner.start()
            try:
                self.assertTrue(self._wait(lambda: dash.running))

                def shown():
                    return dash._draw_snapshot()

                self.assertTrue(self._wait(lambda: shown().get("is_streaming") is True))
                self.assertEqual(shown().get("current_throttle"), 1400)

                # 1) link loss while streaming: stop_streaming_only, no datagram after
                m.last_telem_time = m.time.monotonic() - 5.0
                m.sock = _FakeSocket([])
                with self.assertRaises(_LoopComplete):
                    m.telemetry_thread()
                self.assertFalse(m.is_streaming)
                self.assertTrue(self._wait(lambda: shown().get("is_streaming") is False),
                                shown())
                self.assertEqual(shown().get("current_throttle"), 1000)

                # 2) a telemetry-worker fault: latch shown without any packet
                m._handle_worker_fault("telemetry", RuntimeError("synthetic"))
                self.assertTrue(self._wait(
                    lambda: shown().get("worker_fault") == "telemetry"), shown())
                self.assertTrue(self._wait(lambda: "수신 스레드 오류" in dash._drawn_text()))
            finally:
                pipe.send_text("\x03")
                runner.join(timeout=5.0)
        self.assertFalse(runner.is_alive())
        self.assertEqual(result, [0])

    def _observe_every_ui_call(self, sent_socket):
        module = self.module
        observed = []

        class _AllCallsObservingUI(_RecordingUI):
            def _note(self, kind, text):
                observed.append((
                    kind,
                    text if isinstance(text, str) else "<snapshot>",
                    module.is_streaming,
                    module.is_armed,
                    b"stop" in [d for d, _ in sent_socket.sent],
                ))

            def event(self, text):
                self._note("event", text)
                super().event(text)

            def periodic(self, text):
                self._note("periodic", text)
                super().periodic(text)

            def transition(self, text):
                self._note("transition", text)
                super().transition(text)

            def alert(self, text):
                self._note("alert", text)
                super().alert(text)

            def update(self, snap):
                self._note("update", snap)
                super().update(snap)

        return _AllCallsObservingUI(), observed

    def _replay_observing(self, samples, *, before=None):
        self.module.sock = _FakeSocket([b"packet"] * len(samples))
        recording, observed = self._observe_every_ui_call(self.module.sock)
        self.module.ui = recording
        self.module.csv_writer = types.SimpleNamespace(writerow=lambda _row: None)
        self.module.csv_file = types.SimpleNamespace(flush=lambda: None)
        if before is not None:
            before()
        with (
            mock.patch.object(self.module, "parse_telemetry_packet", side_effect=samples),
            mock.patch.object(self.module, "sample_to_csv_row", return_value=[]),
            mock.patch.object(self.module.time, "sleep", return_value=None),
            self.assertRaises(_LoopComplete),
        ):
            self.module.telemetry_thread()
        return observed

    def test_rc_timeout_packet_stops_streaming_before_any_of_its_output(self):
        self._arm_locally(self.module)
        self.module.last_arm_time = self.module.time.monotonic()
        observed = self._replay_observing(
            [_telemetry_sample(Fault_RC=1, Failsafe_Phase=1, Armed=1)],
        )
        packet_calls = [o for o in observed if o[1] != "[TELEM] 수신 스레드 시작"]
        kinds = [o[1].split(" ")[0] for o in packet_calls]
        self.assertTrue(any(k == "[STATUS]" for k in kinds), packet_calls)
        self.assertTrue(any(k.startswith("[AUTO-LAND]") for k in kinds), packet_calls)
        self.assertTrue(all(not streaming for _k, _t, streaming, _a, _s in packet_calls),
                        packet_calls)
        # console order is unchanged: [STATUS], [AUTO-LAND], then the fault lines
        texts = [o[1] for o in packet_calls]
        status = next(i for i, t in enumerate(texts) if t.startswith("[STATUS]"))
        fault = next(i for i, t in enumerate(texts) if "[FAULT] 드론: RC 타임아웃" in t)
        self.assertLess(status, fault)

    def test_recovery_and_diag_lines_wait_for_the_critical_disarm(self):
        self._arm_locally(self.module)
        self.module.last_arm_time = self.module.time.monotonic()
        critical = _telemetry_sample(Failsafe_Phase=0, Armed=1, Fault_Critical=1)

        def lose_the_link_first():
            # not streaming: the loss is only reported, then [OK] on recovery
            self.module.is_streaming = False
            self.module.last_telem_time = self.module.time.monotonic() - 5.0

        observed = self._replay_observing([critical], before=lose_the_link_first)
        first_packet = []
        for entry in observed:
            if entry[1].startswith("[TELEM]") or "[FAULT] 텔레메트리" in entry[1]:
                continue
            first_packet.append(entry)
            if entry[0] == "update":
                break
        ok = [o for o in first_packet if o[1] == "[OK] 텔레메트리 수신 복구"]
        self.assertEqual(len(ok), 1, first_packet)
        self.assertFalse(ok[0][3])   # disarmed already
        self.assertTrue(ok[0][4])    # stop already sent
        self.assertTrue(all(not armed for _k, _t, _s, armed, _st in first_packet),
                        first_packet)

    def test_diag_line_waits_for_the_packets_disarm(self):
        self._arm_locally(self.module)
        self.module.last_arm_time = self.module.time.monotonic()
        healthy = _telemetry_sample(Failsafe_Phase=0, Armed=1)
        critical = _telemetry_sample(Failsafe_Phase=0, Armed=1, Fault_Critical=1)
        observed = self._replay_observing([healthy, healthy, healthy, critical])
        diag = [o for o in observed if o[1].startswith(" [DIAG]")]
        self.assertEqual(len(diag), 1, observed)
        self.assertFalse(diag[0][3])
        self.assertTrue(diag[0][4])

    def test_packet_output_is_flushed_after_the_worker_fault_safety(self):
        # An arm attempt is pending and the link was lost (reported only). The
        # next packet prints [OK] and a start-retry line, then the CSV write
        # crashes the receiver. Those lines must wait until the worker-fault
        # handler has cancelled the arm attempt with stop.
        self._prime_prearm_telemetry()
        self.module.arm_state = "arming"
        self.module.arm_deadline = self.module.time.monotonic() + 30.0
        self.module.arm_next_retry = 0.0
        self.module.is_streaming = False
        self.module.last_telem_time = self.module.time.monotonic() - 5.0
        sock = _FakeSocket([b"packet"])
        recording, observed = self._observe_every_ui_call(sock)
        self.module.sock = sock
        self.module.ui = recording
        self.module.csv_file = types.SimpleNamespace(flush=lambda: None)
        self._break_csv_writer()
        with (
            mock.patch.object(self.module, "parse_telemetry_packet",
                              side_effect=[_telemetry_sample(Failsafe_Phase=0, Armed=0)]),
            mock.patch.object(self.module, "sample_to_csv_row", return_value=[]),
            mock.patch.object(self.module.time, "sleep", return_value=None),
        ):
            self.module.telemetry_worker()
        self.assertEqual(self.module.worker_fault, "telemetry")
        self.assertEqual(self.module.arm_state, "idle")
        texts = [o[1] for o in observed]
        packet_lines = [o for o in observed
                        if o[1] in ("[OK] 텔레메트리 수신 복구",
                                    "[ARM] 펌웨어 확인 대기: start 재전송")]
        self.assertEqual(len(packet_lines), 2, observed)
        self.assertTrue(all(stop for _k, _t, _s, _a, stop in packet_lines), observed)
        # original console order: the packet's lines, then the fault lines
        ok = texts.index("[OK] 텔레메트리 수신 복구")
        err = next(i for i, t in enumerate(texts) if t.startswith("[ERR] 수신 스레드 오류"))
        cancel = next(i for i, t in enumerate(texts) if "무장 시도 취소" in t)
        self.assertLess(ok, cancel)
        self.assertLess(cancel, err)

    def test_sigterm_takes_the_ctrl_c_path_in_the_console(self):
        import signal

        before = signal.getsignal(signal.SIGTERM)
        seen = []

        def input_receiving_sigterm(*_args):
            handler = signal.getsignal(signal.SIGTERM)
            seen.append(handler)
            if not callable(handler) or handler is before:
                raise EOFError  # no SIGTERM handler: end so the test can fail cleanly
            handler(signal.SIGTERM, None)   # the main thread's handler, as the kernel would
            raise AssertionError("the SIGTERM handler returned inside input()")

        def arm_locally(module):
            self._arm_locally(module)

        module_name = "_control_dualsense_final_sigterm_under_test"
        self.addCleanup(sys.modules.pop, module_name, None)
        m, sock = self._load_module(
            module_name=module_name,
            input_side_effect=input_receiving_sigterm,
            before_main=arm_locally,
        )
        self.assertEqual(len(seen), 1)
        self.assertIsNot(seen[0], before)
        self.assertFalse(m.is_armed)
        self.assertIn(b"stop", [d for d, _ in sock.sent])
        # main() restores the previous handler when it returns
        self.assertIs(signal.getsignal(signal.SIGTERM), before)

    def test_sigterm_after_the_ui_returned_does_not_abort_log_saving(self):
        import signal

        handlers = []

        class _SignalDuringCleanupUI(_RecordingUI):
            def after_run(self):
                handler = signal.getsignal(signal.SIGTERM)
                handlers.append(handler)
                handler(signal.SIGTERM, None)   # must not raise during cleanup

        m, _sock = self._load_mode_module((), "final_sigterm_cleanup")
        m.csv_file = io.StringIO()
        with (
            mock.patch.object(m, "threading", self._threading_with(m, _FakeThread)),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            self.assertEqual(m.main(ui=_SignalDuringCleanupUI()), 0)
        self.assertEqual(len(handlers), 1)
        self.assertTrue(callable(handlers[0]))
        self.assertTrue(m.csv_file.closed)

    KEYBOARD_RC_REFUSAL = "[STDIN] 키보드 모드: RC는 패드로만 보냅니다"

    def test_keyboard_mode_refuses_rc_and_rcr(self):
        m, sock = self._load_mode_module(("--keyboard",), "kb_rc_refuse")
        recording = _RecordingUI()
        m.ui = recording
        commands = ("rc 1500 1500 1500 1500", "rcr 1 0.00 0.00 0.0", "RCR 2 0 0 0", "Rc 1")
        for command in commands:
            m.handle_stdin_command(command)
        self.assertEqual(recording.texts("event"), [self.KEYBOARD_RC_REFUSAL] * len(commands))
        # the keyboard-mode sending boundary itself drops RC and start/resume
        for command in ("rcr 3 0.00 0.00 0.0", "rc 1500 1500 1500 1500", "start", "resume"):
            m.send_cmd(command)
        sent = [d for d, _ in sock.sent]
        self.assertFalse(any(d.split()[0].lower() in (b"rc", b"rcr", b"start", b"resume")
                             for d in sent if d), sent)
        # other commands still pass
        m.send_cmd("gains")
        self.assertIn(b"gains", [d for d, _ in sock.sent])

    def test_listen_without_stdin_keeps_receiving_until_interrupted(self):
        m, sock = self._load_mode_module(("--listen",), "listen_no_stdin")
        sock.sent.clear()
        m.csv_file = io.StringIO()
        sleeps = []

        class _AliveThread(_FakeThread):
            def is_alive(self):
                return True

        def sleep(seconds):
            sleeps.append(seconds)
            if len(sleeps) >= 5:
                raise KeyboardInterrupt   # SIGINT/SIGTERM while receiving

        output = io.StringIO()
        with (
            mock.patch.object(m, "threading", self._threading_with(m, _AliveThread)),
            mock.patch.object(builtins, "input", side_effect=EOFError),
            mock.patch.object(m.time, "sleep", side_effect=sleep),
            contextlib.redirect_stdout(output),
        ):
            self.assertEqual(m.main(), 0)
        self.assertEqual(len(sleeps), 5)
        self.assertIn("[STDIN] 입력 종료 - 수신 계속 (Ctrl+C로 종료)", output.getvalue())
        self.assertEqual({d for d, _ in sock.sent} - {b"connect"}, set())
        self.assertTrue(m.csv_file.closed)

    def test_pad_mode_eof_still_ends_the_station_at_once(self):
        m, _sock = self._load_mode_module((), "pad_eof_no_hold")
        m.csv_file = io.StringIO()

        class _AliveThread(_FakeThread):
            def is_alive(self):
                return True

        with (
            mock.patch.object(m, "threading", self._threading_with(m, _AliveThread)),
            mock.patch.object(builtins, "input", side_effect=EOFError),
            mock.patch.object(m.time, "sleep",
                              side_effect=AssertionError("pad mode must not hold")),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            self.assertEqual(m.main(), 0)
        self.assertTrue(m.csv_file.closed)


class BenchTrimResetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(SCRIPTS_DIR))

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(str(SCRIPTS_DIR))

    def _load_bench(self, filename):
        path = SCRIPTS_DIR / "archive" / filename
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


# A station process for the SIGTERM test: real control_station + DashboardUI in
# a pty, with the UDP socket, pygame and the log directory replaced so nothing
# reaches the network or the repository's logs/.
_SIGTERM_STATION_DRIVER = r'''
import pathlib, socket, sys, time, types
scripts, log_dir, sent_path = sys.argv[1:4]
sys.path.insert(0, scripts)
import ground.paths
ground.paths.LOG_DIR = pathlib.Path(log_dir)

class _Socket:
    def bind(self, _address): pass
    def settimeout(self, _timeout): pass
    def setsockopt(self, *_args): pass
    def close(self): pass
    def recvfrom(self, _size):
        time.sleep(0.05)
        raise socket.timeout
    def sendto(self, data, _address):
        with open(sent_path, "ab") as sent:
            sent.write(data + b"\n")

pygame = types.ModuleType("pygame")
pygame.init = lambda: None
pygame.joystick = types.SimpleNamespace(init=lambda: None, get_count=lambda: 0)
pygame.event = types.SimpleNamespace(pump=lambda: None)
sys.modules["pygame"] = pygame
sys.argv = ["drone_control.py"]
# only the station's own socket is fake: asyncio still needs real socketpairs
real_socket = socket.socket
socket.socket = lambda *_a, **_k: _Socket()
try:
    import ground.control_station as station
finally:
    socket.socket = real_socket
station.is_armed = True
station.is_streaming = True
station.arm_state = "armed"
station.last_arm_time = time.monotonic()
sys.stderr.write("DRIVER-READY\n")
sys.exit(station.main([]))
'''


@unittest.skipUnless(sys.platform.startswith("linux"), "pty test needs Linux")
class StationSigtermTest(unittest.TestCase):
    """SIGTERM to a station running the dashboard in a pty (final review #6)."""

    def _drain(self, master, chunks, until=None, timeout=15.0):
        import os
        import select

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if until is not None and until():
                return True
            ready, _, _ = select.select([master], [], [], 0.05)
            if ready:
                try:
                    data = os.read(master, 65536)
                except OSError:
                    return bool(until and until())
                if not data:
                    return bool(until and until())
                chunks.append(data)
        return bool(until and until())

    def test_sigterm_disarms_and_restores_the_terminal(self):
        import fcntl
        import os
        import signal
        import struct as struct_module
        import subprocess
        import termios

        master, slave = os.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        fcntl.ioctl(slave, termios.TIOCSWINSZ, struct_module.pack("HHHH", 30, 110, 0, 0))
        cooked = termios.tcgetattr(slave)
        self.assertTrue(cooked[3] & termios.ICANON and cooked[3] & termios.ECHO)

        with tempfile.TemporaryDirectory() as tmp:
            sent_path = pathlib.Path(tmp) / "sent.txt"
            env = dict(os.environ, TERM="xterm-256color", PROMPT_TOOLKIT_NO_CPR="1",
                       PYTHONIOENCODING="utf-8")
            proc = subprocess.Popen(
                [sys.executable, "-c", _SIGTERM_STATION_DRIVER,
                 str(SCRIPTS_DIR), tmp, str(sent_path)],
                stdin=slave, stdout=slave, stderr=slave, env=env, close_fds=True,
            )
            self.addCleanup(lambda: proc.poll() is None and proc.kill())
            chunks = []

            def screen():
                return b"".join(chunks).decode("utf-8", errors="replace")

            # the dashboard is up once its panels and hint line are drawn
            self.assertTrue(
                self._drain(master, chunks, until=lambda: "도움말" in screen()),
                screen()[-2000:],
            )
            time.sleep(0.3)
            proc.send_signal(signal.SIGTERM)
            self._drain(master, chunks, until=lambda: proc.poll() is not None)
            self._drain(master, chunks, timeout=0.3)
            try:
                code = proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                self.fail("station did not exit after SIGTERM:\n" + screen()[-2000:])
            attrs = termios.tcgetattr(slave)
            sent = sent_path.read_bytes().split(b"\n") if sent_path.exists() else []

        self.assertEqual(code, 0, screen()[-2000:])
        self.assertTrue(attrs[3] & termios.ICANON, "ICANON left off")
        self.assertTrue(attrs[3] & termios.ECHO, "ECHO left off")
        self.assertIn(b"stop", sent)
        self.assertIn("DISARMED (키보드 인터럽트)", screen())
        self.assertIn("[LOG] 저장 완료", screen())


if __name__ == "__main__":
    unittest.main()
