"""Tests for the control-station output layer (ConsoleUI)."""

import builtins
import contextlib
import io
import sys
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from ground.station_ui import ConsoleUI  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
