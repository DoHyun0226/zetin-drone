"""Tests for scripts/ground/pad_check.py (the --pad-check gamepad diagnosis)."""

import contextlib
import io
import socket
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from ground import pad_check  # noqa: E402


class FakeJoystick:
    """Scripted joystick: each pump() advances one step of the script."""

    def __init__(self, steps, name="Fake Pad"):
        self.steps = steps
        self.index = 0
        self.name = name

    def init(self):
        pass

    def get_name(self):
        return self.name

    def get_numaxes(self):
        return len(self.steps[0]["axes"])

    def get_numbuttons(self):
        return len(self.steps[0]["buttons"])

    def get_numhats(self):
        return len(self.steps[0]["hats"])

    def _step(self):
        return self.steps[min(self.index, len(self.steps) - 1)]

    def get_axis(self, i):
        return self._step()["axes"][i]

    def get_button(self, i):
        return self._step()["buttons"][i]

    def get_hat(self, i):
        return self._step()["hats"][i]


def fake_pygame(joystick, count=1):
    mod = types.SimpleNamespace()
    mod.init = lambda: None
    mod.quit = lambda: None
    mod.joystick = types.SimpleNamespace(
        init=lambda: None,
        get_count=lambda: count,
        Joystick=lambda _i: joystick,
    )

    def pump():
        joystick.index += 1

    mod.event = types.SimpleNamespace(pump=pump)
    return mod


class FakeTime:
    """monotonic advances 0.1 s per call; sleep raises after `limit` calls."""

    def __init__(self, limit):
        self.now = 0.0
        self.limit = limit
        self.sleeps = 0

    def monotonic(self):
        self.now += 0.1
        return self.now

    def sleep(self, _s):
        self.sleeps += 1
        if self.sleeps > self.limit:
            raise KeyboardInterrupt


def run_main(joystick, argv=(), limit=40, count=1):
    out = io.StringIO()
    with mock.patch.dict(sys.modules, {"pygame": fake_pygame(joystick, count)}), \
            mock.patch.object(pad_check, "time", FakeTime(limit)), \
            contextlib.redirect_stdout(out):
        code = pad_check.main(list(argv))
    return code, out.getvalue()


def steps_with_changes():
    rest = {"axes": [0.0, 0.0, -1.0], "buttons": [0, 0], "hats": [(0, 0)]}
    moved = {"axes": [0.0, 0.0, 1.0], "buttons": [1, 0], "hats": [(0, 1)]}
    return [rest] * 12 + [moved]


class PadCheckTest(unittest.TestCase):
    def test_reports_axis_button_hat(self):
        code, out = run_main(FakeJoystick(steps_with_changes()))
        self.assertEqual(code, 0)
        self.assertIn("Fake Pad", out)
        self.assertIn("AXIS  2:  -1.00 -> +1.00", out)
        self.assertIn("BUTTON 0:  DOWN", out)
        self.assertIn("HAT   0:  (0, 0) -> (0, 1)", out)
        self.assertIn("[휴지 스냅샷]", out)
        self.assertIn("[종료 스냅샷]", out)

    def test_live_mode_prints_continuous_values(self):
        code, out = run_main(FakeJoystick(steps_with_changes()), ["--live"])
        self.assertEqual(code, 0)
        self.assertIn("a2:+1.00", out)
        self.assertIn("BTN_0", out)

    def test_no_joystick_is_an_error(self):
        code, out = run_main(FakeJoystick(steps_with_changes()), count=0)
        self.assertEqual(code, 1)
        self.assertIn("[ERR]", out)

    def test_never_creates_socket(self):
        with mock.patch.object(socket, "socket") as s:
            run_main(FakeJoystick(steps_with_changes()))
        s.assert_not_called()

    def test_import_does_not_import_pygame(self):
        self.assertNotIn("pygame", pad_check.__dict__)


if __name__ == "__main__":
    unittest.main()
