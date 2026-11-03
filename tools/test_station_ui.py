"""Tests for the control-station output layer (ConsoleUI, DashboardUI, make_ui)."""

import builtins
import contextlib
import io
import re
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from ground.station_ui import ConsoleUI, DashboardUI, make_ui  # noqa: E402


class _ClosedStream(io.StringIO):
    def __init__(self):
        super().__init__()
        self.close()


class ConsoleUITest(unittest.TestCase):
    def test_control_station_has_no_bare_print(self):
        src = (SCRIPTS / "ground" / "control_station.py").read_text(encoding="utf-8")
        self.assertNotRegex(src, r"(?m)^\s*print\(")

    def test_console_ui_event_prints_same_text(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            ConsoleUI().event("\n>>> [SYSTEM] DISARMED (수동)")
        self.assertEqual(out.getvalue(), "\n>>> [SYSTEM] DISARMED (수동)\n")

    def test_periodic_and_alert_print_the_same_text_as_event(self):
        for method in ("periodic", "alert"):
            with self.subTest(method=method):
                out = io.StringIO()
                with contextlib.redirect_stdout(out):
                    getattr(ConsoleUI(), method)("[STATUS] Roll=0.00")
                self.assertEqual(out.getvalue(), "[STATUS] Roll=0.00\n")

    def test_transition_update_and_after_run_print_nothing(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            ui = ConsoleUI()
            ui.transition("[STATE] Failsafe_Phase 0 → 1")
            ui.update({"is_armed": True})
            ui.after_run()
        self.assertEqual(out.getvalue(), "")

    def test_ui_methods_never_raise(self):
        ui = ConsoleUI()
        err = io.StringIO()
        with (
            contextlib.redirect_stdout(_ClosedStream()),
            contextlib.redirect_stderr(err),
            mock.patch.object(builtins, "input", side_effect=EOFError),
        ):
            ui.event("[LOG] x")
            ui.periodic("[STATUS] x")
            ui.alert("[ERR] x")
            ui.transition("[STATE] x")
            ui.update({})
            ui.after_run()
            ui.run(lambda _msg: None, lambda: None, lambda: None)
        self.assertIn("ConsoleUI", err.getvalue())

    def test_ui_methods_never_raise_even_when_stderr_is_closed_too(self):
        ui = ConsoleUI()
        with (
            contextlib.redirect_stdout(_ClosedStream()),
            contextlib.redirect_stderr(_ClosedStream()),
        ):
            ui.event("[LOG] x")
            ui.alert("[ERR] x")

    def test_run_passes_non_empty_lines_to_the_handler_until_interrupt(self):
        handled = []
        calls = []
        with mock.patch.object(
            builtins, "input", side_effect=["pr 0.9", "", "gains", KeyboardInterrupt],
        ):
            ConsoleUI().run(
                handled.append,
                lambda: calls.append("interrupt"),
                lambda: calls.append("eof"),
            )
        self.assertEqual(handled, ["pr 0.9", "gains"])
        self.assertEqual(calls, ["interrupt"])

    def test_run_calls_on_eof_and_returns(self):
        calls = []
        with mock.patch.object(builtins, "input", side_effect=EOFError):
            ConsoleUI().run(
                lambda _msg: None,
                lambda: calls.append("interrupt"),
                lambda: calls.append("eof"),
            )
        self.assertEqual(calls, ["eof"])

    def test_keyboard_interrupt_inside_the_handler_also_reaches_on_interrupt(self):
        calls = []

        def handler(_msg):
            raise KeyboardInterrupt

        with mock.patch.object(builtins, "input", side_effect=["stop"]):
            ConsoleUI().run(handler, lambda: calls.append("interrupt"), lambda: None)
        self.assertEqual(calls, ["interrupt"])

    # --- review round 1: run() contains every failure ---

    def _run_collecting(self, input_side_effect=None, handler=None, stdin=None):
        calls = []
        handled = []
        out = io.StringIO()
        err = io.StringIO()
        patches = [contextlib.redirect_stdout(out), contextlib.redirect_stderr(err)]
        if input_side_effect is not None:
            patches.append(
                mock.patch.object(builtins, "input", side_effect=input_side_effect)
            )
        if stdin is not None:
            patches.append(mock.patch.object(sys, "stdin", stdin))
        with contextlib.ExitStack() as stack:
            for patch in patches:
                stack.enter_context(patch)
            ConsoleUI().run(
                handler or handled.append,
                lambda: calls.append("interrupt"),
                lambda: calls.append("eof"),
            )
        return calls, handled, out.getvalue()

    def test_closed_stdin_ends_through_the_eof_safety_path(self):
        calls, _handled, out = self._run_collecting(stdin=_ClosedStream())
        self.assertEqual(calls, ["eof"])
        self.assertIn("[STDIN] 입력 오류: ValueError", out)

    def test_input_oserror_ends_through_the_eof_safety_path(self):
        calls, _handled, out = self._run_collecting(
            input_side_effect=OSError(5, "Input/output error"),
        )
        self.assertEqual(calls, ["eof"])
        self.assertIn("[STDIN] 입력 오류: OSError", out)

    def test_input_error_runs_the_safety_callback_before_any_output(self):
        # stdout blocks until the EOF callback (the conditional disarm) has run;
        # a write that comes first waits out the timeout and is recorded.
        disarmed = threading.Event()
        writes = []

        class _BlockingStdout(io.StringIO):
            def write(self, text):
                writes.append((text, disarmed.is_set()))
                disarmed.wait(timeout=0.5)
                return super().write(text)

        def run():
            with (
                mock.patch.object(builtins, "input", side_effect=OSError(5, "EIO")),
                contextlib.redirect_stdout(_BlockingStdout()),
                contextlib.redirect_stderr(io.StringIO()),
            ):
                ConsoleUI().run(lambda _m: None, lambda: None, disarmed.set)

        worker = threading.Thread(target=run, daemon=True)
        worker.start()
        worker.join(timeout=3.0)
        self.assertFalse(worker.is_alive())
        self.assertTrue(disarmed.is_set())
        error_writes = [w for w in writes if "[STDIN] 입력 오류" in w[0]]
        self.assertEqual(len(error_writes), 1, writes)
        self.assertTrue(all(done for _text, done in writes), writes)

    def test_handler_exception_is_reported_and_the_loop_continues(self):
        handled = []

        def handler(msg):
            handled.append(msg)
            if msg == "boom":
                raise RuntimeError("synthetic handler failure")

        calls, _unused, out = self._run_collecting(
            input_side_effect=["boom", "gains", EOFError], handler=handler,
        )
        self.assertEqual(handled, ["boom", "gains"])
        self.assertEqual(calls, ["eof"])
        self.assertIn(
            "[STDIN] 명령 처리 오류: RuntimeError: synthetic handler failure", out,
        )

    def test_failing_safety_callback_does_not_escape_run(self):
        def broken():
            raise RuntimeError("synthetic callback failure")

        err = io.StringIO()
        for side_effect in (EOFError, KeyboardInterrupt, OSError("gone")):
            with self.subTest(side_effect=side_effect):
                with (
                    mock.patch.object(builtins, "input", side_effect=side_effect),
                    contextlib.redirect_stdout(io.StringIO()),
                    contextlib.redirect_stderr(err),
                ):
                    ConsoleUI().run(lambda _m: None, broken, broken)
        self.assertIn("synthetic callback failure", err.getvalue())



# ---------------------------------------------------------------------------
# DashboardUI (prompt_toolkit) — no real terminal: pipe input + DummyOutput
# ---------------------------------------------------------------------------
WAIT_SEC = 3.0
HELP_LINES = (
    "========== DRONE CONTROLLER ==========",
    " PID: pa|ia|da, pr|ir|dr, pp|ip|dp, py|iy|dy <val>",
    "======================================",
)


def ns(**values):
    return types.SimpleNamespace(**values)


def _wait_until(predicate, timeout=WAIT_SEC, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return bool(predicate())


def _fake_snapshot(**overrides):
    snap = {
        "mode": "pad",
        "arm_state": "disarmed",
        "is_armed": False,
        "is_streaming": False,
        "current_throttle": 1000,
        "trim_roll": 0.0,
        "trim_pitch": 0.0,
        "telem_total_pkts": 0,
        "telem_dropped_pkts": 0,
        "bad_packet_count": 0,
        "last_telem_time": time.monotonic(),
        "log_path": None,
        "raw_log_path": None,
        "raw_capture_enabled": False,
        "raw_batch_count": 0,
        "raw_last_dropped": 0,
        "raw_last_receive_time": 0.0,
        "mag_enabled_choice": True,
        "resume_state": "idle",
        "last_sample": {"Roll": 1.0, "Pitch": -0.5, "Yaw": 87.3, "Armed": 0},
        "worker_fault": None,
        "armed_since": None,
    }
    snap.update(overrides)
    return snap


class DashboardUITest(unittest.TestCase):
    def setUp(self):
        from prompt_toolkit.input import create_pipe_input
        from prompt_toolkit.output import DummyOutput

        pipe_context = create_pipe_input()
        self.pipe = pipe_context.__enter__()
        self.addCleanup(pipe_context.__exit__, None, None, None)
        self.ui = DashboardUI(
            help_lines=HELP_LINES, input=self.pipe, output=DummyOutput(),
        )
        self.handled = []
        self.calls = []
        self.thread = None

    def tearDown(self):
        if self.thread is not None and self.thread.is_alive():
            try:
                self.pipe.send_text("\x03")  # Ctrl+C exits whatever the input line holds
            except Exception:
                pass
            self.thread.join(timeout=WAIT_SEC)
        if self.thread is not None:
            self.assertFalse(self.thread.is_alive(), "dashboard did not stop")

    def _start(self, handler=None, on_eof=None):
        def run():
            self.ui.run(
                handler or self.handled.append,
                lambda: self.calls.append("interrupt"),
                on_eof or (lambda: self.calls.append("eof")),
            )

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()
        self.assertTrue(_wait_until(lambda: self.ui.running), "dashboard did not start")
        self.assertTrue(_wait_until(lambda: self.ui._app.render_counter > 0))

    def _join(self):
        self.thread.join(timeout=WAIT_SEC)
        self.assertFalse(self.thread.is_alive(), "run() did not return")

    def test_enter_sends_command_to_handler(self):
        self._start()
        self.pipe.send_text("pr 0.9\r")
        self.assertTrue(_wait_until(lambda: self.handled == ["pr 0.9"]), self.handled)
        self.assertTrue(_wait_until(lambda: self.ui._textarea.text == ""))

    def test_question_mark_shows_help_in_events(self):
        self._start()
        self.pipe.send_text("?")
        self.assertTrue(_wait_until(
            lambda: any("PID:" in text for text in self.ui._event_texts())
        ))
        self.assertTrue(_wait_until(lambda: "PID:" in self.ui._drawn_text()))
        self.assertEqual(self.ui._textarea.text, "")
        self.assertEqual(self.handled, [])

    def test_escape_clears_input_line(self):
        self._start()
        self.pipe.send_text("pr 0.")
        self.assertTrue(_wait_until(lambda: self.ui._textarea.text == "pr 0."))
        self.pipe.send_text("\x1b")
        self.assertTrue(_wait_until(lambda: self.ui._textarea.text == ""))
        self.assertTrue(self.ui.running)
        self.assertEqual(self.handled, [])

    def test_ctrl_c_calls_on_interrupt_and_returns(self):
        self._start()
        self.pipe.send_text("\x03")
        self._join()
        self.assertEqual(self.calls, ["interrupt"])

    def test_ctrl_d_calls_on_eof_and_returns(self):
        self._start()
        self.pipe.send_text("\x04")
        self._join()
        self.assertEqual(self.calls, ["eof"])

    def test_ctrl_d_with_text_on_the_line_does_not_exit(self):
        self._start()
        self.pipe.send_text("pr 0.9")
        self.assertTrue(_wait_until(lambda: self.ui._textarea.text == "pr 0.9"))
        self.pipe.send_text("\x04")
        time.sleep(0.3)
        self.assertTrue(self.ui.running)
        self.assertEqual(self.calls, [])
        self.assertEqual(self.ui._textarea.text, "pr 0.9")
        self.pipe.send_text("\x1b")
        self.assertTrue(_wait_until(lambda: self.ui._textarea.text == ""))
        self.pipe.send_text("\x04")
        self._join()
        self.assertEqual(self.calls, ["eof"])
        self.assertEqual(self.handled, [])

    def test_closed_input_runs_the_eof_safety_before_teardown_output(self):
        # Output blocks once the input has been seen closed, until the EOF
        # callback (the conditional disarm) has run; every such write records
        # whether the safety callback came first.
        from prompt_toolkit.output import DummyOutput

        disarmed = threading.Event()
        writes = []
        pipe = self.pipe

        class _BlockingOutput(DummyOutput):
            def flush(self):
                if pipe.closed:
                    writes.append(disarmed.is_set())
                    disarmed.wait(timeout=0.5)

        self.ui = DashboardUI(input=pipe, output=_BlockingOutput())

        def on_eof():
            self.calls.append("eof")
            disarmed.set()

        self._start(on_eof=on_eof)
        pipe.close()
        self._join()
        self.assertEqual(self.calls, ["eof"])
        self.assertTrue(writes, "no teardown output was observed")
        self.assertTrue(all(writes), writes)
        self.assertFalse(
            any(text.startswith("[STDIN] 입력 오류") for text in self.ui._event_texts()),
            self.ui._event_texts(),
        )

    def test_console_fallback_forwards_every_output_method(self):
        out = io.StringIO()
        with (
            mock.patch.object(DashboardUI, "_build_app",
                              side_effect=RuntimeError("synthetic build failure")),
            mock.patch.object(builtins, "input", side_effect=EOFError),
            contextlib.redirect_stdout(out),
        ):
            self.ui.event("[ARM] before")
            self.ui.run(self.handled.append,
                        lambda: self.calls.append("interrupt"),
                        lambda: self.calls.append("eof"))
            self.ui.periodic("[STATUS] Roll=0.00")
            self.ui.alert("[ERR] x")
            self.ui.event("[LOG] 저장 완료: x.csv")
            self.ui.transition("[STATE] x")
            self.ui.update(_fake_snapshot())
            self.ui.after_run()
        text = out.getvalue()
        self.assertEqual(self.calls, ["eof"])
        self.assertIn("[WARN] 대시보드 시작 실패: RuntimeError: synthetic build failure", text)
        self.assertIn("[ARM] before", text)
        self.assertTrue(
            text.endswith("[STATUS] Roll=0.00\n[ERR] x\n[LOG] 저장 완료: x.csv\n"), text,
        )

    def test_handler_exception_is_logged_and_app_keeps_running(self):
        def handler(msg):
            self.handled.append(msg)
            if msg == "boom":
                raise RuntimeError("synthetic handler failure")

        self._start(handler)
        self.pipe.send_text("boom\r")
        expected = "[STDIN] 명령 처리 오류: RuntimeError: synthetic handler failure"
        self.assertTrue(_wait_until(lambda: expected in self.ui._event_texts()),
                        self.ui._event_texts())
        self.pipe.send_text("gains\r")
        self.assertTrue(_wait_until(lambda: self.handled == ["boom", "gains"]))
        self.assertTrue(self.ui.running)
        self.assertEqual(self.calls, [])

    def test_event_from_other_thread_appears(self):
        self._start()
        worker = threading.Thread(target=self.ui.event, args=("\n[WARN] from worker",))
        worker.start()
        worker.join(timeout=WAIT_SEC)
        self.assertTrue(_wait_until(lambda: "[WARN] from worker" in self.ui._drawn_text()))
        self.assertIn("[WARN] from worker", self.ui._event_texts())

    def test_ui_calls_do_not_block_while_app_runs(self):
        self._start()
        fake_snapshot = _fake_snapshot()
        t0 = time.perf_counter()
        for i in range(2000):
            self.ui.event(f"[WARN] {i}")
            self.ui.update(fake_snapshot)
        self.assertLess(time.perf_counter() - t0, 0.5)

    def test_redraw_rate_is_capped(self):
        self._start()
        time.sleep(0.2)
        before = self.ui._app.render_counter
        t0 = time.monotonic()
        for i in range(2000):
            self.ui.event(f"[WARN] {i}")
            if i % 100 == 99:
                time.sleep(0.025)
        elapsed = time.monotonic() - t0
        renders = self.ui._app.render_counter - before
        self.assertGreaterEqual(renders, 1)
        self.assertLessEqual(renders, elapsed / 0.1 + 2, (renders, elapsed))

    def test_stale_age_keeps_updating_without_packets(self):
        self._start()
        self.ui.update(_fake_snapshot(last_telem_time=time.monotonic() - 2.0))

        def stale_age():
            match = re.search(r"수신 없음 (\d+\.\d)s", self.ui._drawn_text())
            return float(match.group(1)) if match else None

        self.assertTrue(_wait_until(lambda: stale_age() is not None),
                        self.ui._drawn_text())
        before_renders = self.ui._app.render_counter
        before_age = stale_age()
        time.sleep(0.35)
        self.assertGreaterEqual(self.ui._app.render_counter - before_renders, 3)
        self.assertGreater(stale_age(), before_age)

    def test_events_after_run_go_to_stdout(self):
        self._start()
        self.ui.event("[ARM] 펌웨어 Armed=1 확인 (mag on)")
        self.pipe.send_text("\x04")
        self._join()
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.ui.after_run()
            self.ui.event("[LOG] 저장 완료: logs/flight_log_x.csv")
        text = out.getvalue()
        self.assertRegex(text, r"\d\d:\d\d:\d\d \[ARM\] 펌웨어 Armed=1 확인 \(mag on\)")
        self.assertTrue(text.endswith("[LOG] 저장 완료: logs/flight_log_x.csv\n"), text)
        self.assertLess(text.index("[ARM]"), text.index("[LOG]"))

    def test_link_rate_comes_from_the_update_rate(self):
        times = iter([0.0] + [0.05 * i for i in range(1, 21)])
        ui = DashboardUI(clock=lambda: next(times))
        for count in range(21):
            ui.update(_fake_snapshot(telem_total_pkts=5 * count))
        self.assertAlmostEqual(ui._draw_snapshot()["link_hz"], 20.0)

    def test_link_rate_counts_valid_packets_not_update_calls(self):
        # 잘못된 패킷도 update()를 부르지만(불량 수 갱신) 링크 Hz는 유효 수신만 센다.
        times = iter([0.05 * i for i in range(41)])
        ui = DashboardUI(clock=lambda: next(times))
        for call in range(41):
            ui.update(_fake_snapshot(valid_packet_count=call // 2))
        self.assertAlmostEqual(ui._draw_snapshot()["link_hz"], 10.0)

    def test_malformed_only_updates_show_zero_rate_and_the_bad_count(self):
        times = iter([0.05 * i for i in range(41)])
        ui = DashboardUI(clock=lambda: next(times))
        for call in range(41):
            ui.update(_fake_snapshot(valid_packet_count=5, bad_packet_count=call))
        drawn = ui._draw_snapshot()
        self.assertAlmostEqual(drawn["link_hz"], 0.0)
        self.assertEqual(drawn["bad_packet_count"], 40)

    def test_hints_follow_the_mode(self):
        ui = DashboardUI()
        expected = {
            "pad": "X 시동  △ 복구  R2/L2 스로틀  ? 도움말 ",
            "keyboard": "? 도움말 ",
            "listen": "? 도움말 ",
        }
        for mode, hint in expected.items():
            with self.subTest(mode=mode):
                ui.update(_fake_snapshot(mode=mode))
                self.assertEqual("".join(t for _s, t in ui._hints()), hint)

    def test_periodic_lines_are_left_to_the_panels(self):
        self.ui.periodic("[STATUS] Roll=0.00")
        self.ui.event("[TRIM] x")
        self.assertEqual(self.ui._event_texts(), ["[TRIM] x"])

    def test_event_list_keeps_the_latest_500(self):
        for i in range(600):
            self.ui.event(f"[WARN] {i}")
        texts = self.ui._event_texts()
        self.assertEqual(len(texts), 500)
        self.assertEqual(texts[0], "[WARN] 100")
        self.assertEqual(texts[-1], "[WARN] 599")

    def test_dashboard_methods_never_raise(self):
        def broken():
            raise RuntimeError("synthetic redraw failure")

        self.ui._app = types.SimpleNamespace(invalidate=broken, is_running=True)
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            self.ui.event("[LOG] x")
            self.ui.alert("[ERR] x")
            self.ui.transition("[STATE] x")
            self.ui.update(_fake_snapshot())
            self.ui.periodic("[STATUS] x")
        self.assertIn("synthetic redraw failure", err.getvalue())

    # --- final review: interrupts, loop errors, snapshot pull, listen hold ---

    def _run_catching_interrupt(self, ui, *, on_eof=None):
        """ui.run on this thread; a KeyboardInterrupt escaping it is a failure."""
        try:
            ui.run(self.handled.append,
                   lambda: self.calls.append("interrupt"),
                   on_eof or (lambda: self.calls.append("eof")))
        except KeyboardInterrupt:
            self.fail("KeyboardInterrupt escaped DashboardUI.run()")

    def test_ctrl_c_while_the_dashboard_is_being_built_runs_on_interrupt(self):
        with (
            mock.patch.object(DashboardUI, "_build_app", side_effect=KeyboardInterrupt),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            self._run_catching_interrupt(self.ui)
        self.assertEqual(self.calls, ["interrupt"])

    def test_ctrl_c_as_the_app_starts_runs_on_interrupt(self):
        from prompt_toolkit.application import Application

        with (
            mock.patch.object(Application, "run", side_effect=KeyboardInterrupt),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            self._run_catching_interrupt(self.ui)
        self.assertEqual(self.calls, ["interrupt"])

    # Amendment (finding 2): an internal dashboard error is a fault, not an
    # operator stop. The station's fault handler runs first (latch, RC stop,
    # no disarm), then the UI falls back to line output and keeps running
    # until the operator presses Ctrl+C.

    def _run_fault_fallback(self, trigger, output=None):
        """Start the dashboard, call ``trigger(app)``, then drive the fallback console."""
        faults = []
        seen_at_fault = []
        console_lines = []
        input_started = threading.Event()
        release_input = threading.Event()

        def on_fault(error):
            seen_at_fault.append((list(self.ui._event_texts()), list(self.calls)))
            faults.append(error)

        def fallback_input(*_args):
            input_started.set()
            if not release_input.wait(timeout=WAIT_SEC):
                raise AssertionError("fallback input was never released")
            raise KeyboardInterrupt   # the operator's Ctrl+C on the line console

        self.ui = DashboardUI(input=self.pipe, output=output or self._dummy_output())
        self.ui.set_fault_handler(on_fault)
        out = io.StringIO()
        with (
            mock.patch.object(builtins, "input", side_effect=fallback_input),
            contextlib.redirect_stdout(out),
            contextlib.redirect_stderr(io.StringIO()) as err,
        ):
            self.thread = threading.Thread(
                target=self.ui.run,
                args=(self.handled.append,
                      lambda: self.calls.append("interrupt"),
                      lambda: self.calls.append("eof")),
                daemon=True,
            )
            self.thread.start()
            self.assertTrue(_wait_until(lambda: self.ui.running))
            self.assertTrue(_wait_until(lambda: self.ui._app.render_counter > 0))
            trigger(self.ui._app)
            self.assertTrue(input_started.wait(timeout=WAIT_SEC), "no console fallback")
            self.assertEqual(self.calls, [])   # no on_interrupt / on_eof (no disarm)
            self.ui.event("[WARN] after the fallback")
            console_lines.append(out.getvalue())
            release_input.set()
            self._join()
        self.assertEqual(self.calls, ["interrupt"])   # only the operator's Ctrl+C
        self.assertNotIn("Press ENTER", err.getvalue())
        return faults, seen_at_fault, console_lines[0]

    def test_render_failure_is_a_fault_and_falls_back_to_the_console(self):
        from prompt_toolkit.output import DummyOutput

        broken = threading.Event()

        class _FailingFlushOutput(DummyOutput):
            def flush(self):
                if broken.is_set():
                    raise OSError(5, "synthetic terminal write failure")

        def trigger(_app):
            broken.set()
            self.ui.event("[WARN] trigger a redraw")

        faults, seen_at_fault, console = self._run_fault_fallback(
            trigger, output=_FailingFlushOutput())
        self.assertEqual(len(faults), 1)
        self.assertIsInstance(faults[0], OSError)
        events_then, calls_then = seen_at_fault[0]
        self.assertFalse(any("대시보드 오류" in t for t in events_then), events_then)
        self.assertEqual(calls_then, [])
        self.assertIn("[WARN] 대시보드 오류: OSError", console)
        self.assertIn("[WARN] after the fallback", console)

    def test_background_task_failure_is_a_fault_and_falls_back_to_the_console(self):
        async def broken_task():
            raise RuntimeError("synthetic background failure")

        def trigger(app):
            app.loop.call_soon_threadsafe(
                lambda: app.create_background_task(broken_task()))

        faults, _seen, console = self._run_fault_fallback(trigger)
        self.assertEqual([str(f) for f in faults], ["synthetic background failure"])
        self.assertIn("[WARN] 대시보드 오류: RuntimeError: synthetic background failure",
                      console)
        self.assertIn("[WARN] after the fallback", console)

    def test_interrupt_request_ends_the_running_dashboard_like_ctrl_c(self):
        # SIGTERM path: the signal handler asks the running dashboard to stop.
        self.assertFalse(self.ui.interrupt())  # not running yet: caller raises instead
        self._start()
        self.assertTrue(self.ui.interrupt())
        self._join()
        self.assertEqual(self.calls, ["interrupt"])

    def test_snapshot_provider_is_pulled_on_the_refresh_tick(self):
        state = {"is_streaming": True, "worker_fault": None}
        pulls = []

        def provider():
            pulls.append(threading.current_thread())
            return _fake_snapshot(is_streaming=state["is_streaming"],
                                  worker_fault=state["worker_fault"])

        self.ui.set_snapshot_provider(provider)
        self._start()
        self.assertTrue(_wait_until(lambda: pulls))
        # a station-side change with no update() call reaches the panels
        state.update(is_streaming=False, worker_fault="telemetry")
        self.assertTrue(_wait_until(
            lambda: self.ui._draw_snapshot().get("worker_fault") == "telemetry"
        ))
        self.assertTrue(_wait_until(lambda: "수신 스레드 오류" in self.ui._drawn_text()))
        # pulled only from the app's own thread, never from the caller of update()
        self.assertEqual({t.name for t in pulls}, {self.thread.name})

    def test_output_methods_never_call_the_snapshot_provider(self):
        def provider():
            raise AssertionError("snapshot pulled from an output method")

        self.ui.set_snapshot_provider(provider)
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            self.ui.event("[LOG] x")
            self.ui.transition("[STATE] x")
            self.ui.alert("[ERR] x")
            self.ui.update(_fake_snapshot())
            self.ui.periodic("[STATUS] x")
        self.assertEqual(err.getvalue(), "")

    def test_closed_input_with_a_reception_hold_keeps_running_until_interrupt(self):
        # --listen: a closed terminal input must not end reception.
        held = threading.Event()
        released = threading.Event()

        def hold():
            held.set()
            if not released.wait(timeout=WAIT_SEC):
                raise AssertionError("hold was never released")
            raise KeyboardInterrupt

        self.ui = DashboardUI(input=self.pipe, output=self._dummy_output(),
                              input_end_hold=hold)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self._start()
            self.pipe.close()
            self.assertTrue(held.wait(timeout=WAIT_SEC))
            time.sleep(0.1)
            self.assertTrue(self.thread.is_alive())
            released.set()
            self._join()
        self.assertEqual(self.calls, ["eof", "interrupt"])
        self.assertIn("[STDIN] 입력 종료 - 수신 계속 (Ctrl+C로 종료)", out.getvalue())

    def test_ctrl_d_still_exits_with_a_reception_hold(self):
        def hold():
            raise AssertionError("an interactive Ctrl+D must not hold")

        self.ui = DashboardUI(input=self.pipe, output=self._dummy_output(),
                              input_end_hold=hold)
        self._start()
        self.pipe.send_text("\x04")
        self._join()
        self.assertEqual(self.calls, ["eof"])

    @staticmethod
    def _dummy_output():
        from prompt_toolkit.output import DummyOutput

        return DummyOutput()


class ConsoleHoldTest(unittest.TestCase):
    """--listen with a closed or non-interactive stdin keeps receiving."""

    def test_eof_with_a_hold_waits_and_ends_through_on_interrupt(self):
        calls = []

        def hold():
            calls.append("hold")
            raise KeyboardInterrupt  # SIGINT/SIGTERM while waiting

        out = io.StringIO()
        with (
            mock.patch.object(builtins, "input", side_effect=EOFError),
            contextlib.redirect_stdout(out),
        ):
            ConsoleUI(input_end_hold=hold).run(
                lambda _m: None,
                lambda: calls.append("interrupt"),
                lambda: calls.append("eof"),
            )
        self.assertEqual(calls, ["eof", "hold", "interrupt"])
        self.assertIn("[STDIN] 입력 종료 - 수신 계속 (Ctrl+C로 종료)", out.getvalue())

    def test_input_error_with_a_hold_also_waits(self):
        calls = []
        with (
            mock.patch.object(builtins, "input", side_effect=OSError(5, "EIO")),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            ConsoleUI(input_end_hold=lambda: calls.append("hold")).run(
                lambda _m: None,
                lambda: calls.append("interrupt"),
                lambda: calls.append("eof"),
            )
        self.assertEqual(calls, ["eof", "hold"])

    def test_make_ui_passes_the_hold_to_the_console(self):
        hold = lambda: None  # noqa: E731
        ui = make_ui(ns(plain=True), input_end_hold=hold)
        self.assertIs(ui._input_end_hold, hold)
        ui = make_ui(ns(plain=False), isatty=lambda: False, input_end_hold=hold)
        self.assertIs(ui._input_end_hold, hold)


class MakeUITest(unittest.TestCase):
    def test_non_tty_uses_console_ui(self):
        with mock.patch.object(sys.stdout, "isatty", return_value=False):
            self.assertIsInstance(make_ui(ns(plain=False)), ConsoleUI)

    def test_plain_flag_uses_console_ui(self):
        self.assertIsInstance(make_ui(ns(plain=True), isatty=lambda: True), ConsoleUI)

    def test_tty_with_prompt_toolkit_uses_the_dashboard(self):
        ui = make_ui(ns(plain=False), isatty=lambda: True, help_lines=HELP_LINES)
        self.assertIsInstance(ui, DashboardUI)
        self.assertEqual(ui._help_lines, list(HELP_LINES))

    def test_missing_prompt_toolkit_falls_back(self):
        out = io.StringIO()
        with (
            mock.patch.dict(sys.modules, {"prompt_toolkit": None}),
            contextlib.redirect_stdout(out),
        ):
            self.assertIsInstance(make_ui(ns(plain=False), isatty=lambda: True), ConsoleUI)
        self.assertIn("[WARN] prompt_toolkit 없음 - 줄 출력으로 실행", out.getvalue())


class DashboardStyleContrastTest(unittest.TestCase):
    def test_no_style_uses_bright_black_foreground(self):
        # ANSI bright black is the background colour in some terminal themes
        # (kitty, 2026-11-03 bench): labels and event times became invisible.
        from ground import station_ui
        for name, rule in station_ui._STYLE_RULES.items():
            with self.subTest(style=name):
                self.assertNotIn("fg:ansibrightblack", rule)


class DashboardEventWrapTest(unittest.TestCase):
    def test_long_event_wraps_instead_of_being_cut(self):
        # 2026-11-03 bench: the [GAINS] reply ran past the right edge and the
        # numbers after the edge were never shown.
        from prompt_toolkit.utils import get_cwidth
        from ground.station_ui import DashboardUI
        ui = DashboardUI()
        ui._size = lambda: (80, 30)
        ui._panel_rows = lambda: []
        text = "[GAINS] " + " ".join(f"Kp_rate{i}=0.{i:03d} 게인{i}" for i in range(20))
        ui.event(text)
        ui._event_fragments()
        rows = ui._drawn_events
        self.assertGreater(len(rows), 1)
        for row in rows:
            self.assertLessEqual(get_cwidth("".join(part for _, part in row)), 80)
        shown = "".join(part for row in rows for style, part in row if style != "class:event-time")
        self.assertEqual(shown.replace(" ", ""), text.replace(" ", ""))


if __name__ == "__main__":
    unittest.main()
