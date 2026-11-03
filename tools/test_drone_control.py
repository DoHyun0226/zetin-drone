"""Tests for the scripts/drone_control.py control-station entry point."""

import contextlib
import io
import os
import socket
import subprocess
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
DRONE_CONTROL = SCRIPTS_DIR / "drone_control.py"
sys.path.insert(0, str(SCRIPTS_DIR))

import drone_control  # noqa: E402

STATION = "ground.control_station"


def run(args):
    return subprocess.run(
        [str(a) for a in args],
        capture_output=True,
        text=True,
        env=os.environ.copy(),
        timeout=60,
        check=False,
    )


class DroneControlTest(unittest.TestCase):
    def setUp(self):
        # 다른 시험이 이미 import 했더라도 이 시험에서는 새로 본다.
        saved = sys.modules.pop(STATION, None)
        if saved is not None:
            self.addCleanup(sys.modules.__setitem__, STATION, saved)
        self.addCleanup(sys.modules.pop, STATION, None)

    def _main_without_station(self, argv):
        """Run drone_control.main and fail the test if it touches the station."""
        with (
            mock.patch.object(socket, "socket") as make_socket,
            mock.patch.object(Path, "open") as path_open,
            mock.patch.object(Path, "mkdir") as mkdir,
            contextlib.redirect_stdout(io.StringIO()) as out,
            contextlib.redirect_stderr(io.StringIO()) as err,
            self.assertRaises(SystemExit) as raised,
        ):
            drone_control.main(argv)
        make_socket.assert_not_called()
        path_open.assert_not_called()
        mkdir.assert_not_called()
        self.assertNotIn(STATION, sys.modules)
        return raised.exception.code, out.getvalue(), err.getvalue()

    def test_modes_are_mutually_exclusive(self):
        result = run([sys.executable, DRONE_CONTROL, "--keyboard", "--listen"])
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("not allowed with", result.stderr)

    def test_help_does_not_open_socket_or_log(self):
        result = run([sys.executable, DRONE_CONTROL, "--help"])
        self.assertEqual(result.returncode, 0, result.stderr)
        for flag in ("--keyboard", "--listen", "--no-raw", "--no-mag", "--plain"):
            self.assertIn(flag, result.stdout)
        code, out, _err = self._main_without_station(["--help"])
        self.assertEqual(code, 0)
        self.assertIn("--keyboard", out)

    def test_invalid_arguments_do_not_import_the_station(self):
        for argv in (["--keyboard", "--listen"], ["--bogus"], ["--pad-check"], ["--plot"]):
            with self.subTest(argv=argv):
                code, _out, err = self._main_without_station(argv)
                self.assertNotEqual(code, 0)
                self.assertIn("usage:", err)

    def test_mode_arguments_reach_the_station_unchanged(self):
        for argv in ([], ["--keyboard"], ["--listen", "--no-raw"],
                     ["--no-mag", "--plain"]):
            with self.subTest(argv=argv):
                seen = {}

                def fake_main(station_argv):
                    seen["main_argv"] = list(station_argv)
                    return 7

                def fake_import(name):
                    # 스테이션은 import 시점에 sys.argv로 모드를 정하므로
                    # import 전에 같은 인자가 sys.argv에 있어야 한다.
                    seen["name"] = name
                    seen["argv_at_import"] = list(sys.argv[1:])
                    return types.SimpleNamespace(main=fake_main)

                with (
                    mock.patch.object(sys, "argv", ["drone_control.py", "--other"]),
                    mock.patch.object(drone_control.importlib, "import_module",
                                      side_effect=fake_import),
                ):
                    self.assertEqual(drone_control.main(argv), 7)
                self.assertEqual(seen["name"], STATION)
                self.assertEqual(seen["argv_at_import"], argv)
                self.assertEqual(seen["main_argv"], argv)


if __name__ == "__main__":
    unittest.main()
