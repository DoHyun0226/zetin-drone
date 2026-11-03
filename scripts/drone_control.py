#!/usr/bin/env python3
"""Control-station entry point: DualSense flight (default), --keyboard, --listen.

  drone_control.py [--keyboard | --listen] [--no-raw] [--no-mag] [--plain]

Only one tool may talk UDP to the drone at a time: the firmware sends
telemetry to the last sender, and even --listen sends ``connect``.
"""

import argparse
import importlib
import sys

STATION_MODULE = "ground.control_station"


def _parser():
    parser = argparse.ArgumentParser(
        prog="drone_control.py",
        description="Drone control station. Default: DualSense flight with the "
                    "dashboard, CSV and raw logging.",
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument(
        "--keyboard",
        action="store_true",
        help="no gamepad: command line for settings and reading only; start and "
             "resume are refused, nothing is sent on exit, no log files",
    )
    modes.add_argument(
        "--listen",
        action="store_true",
        help="receive only: display and CSV/raw logging; sends nothing but connect",
    )
    parser.add_argument(
        "--no-raw",
        action="store_true",
        help="disable the firmware raw stream and do not create IMU raw logs",
    )
    parser.add_argument(
        "--no-mag",
        action="store_true",
        help="start with magnetometer yaw fusion deselected (default: on)",
    )
    parser.add_argument(
        "--plain",
        action="store_true",
        help="plain line output instead of the full-screen dashboard",
    )
    return parser


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    # --help와 잘못된 조합은 여기서 끝난다. control_station은 import만으로
    # UDP 4210을 잡고 로그 파일을 만들기 때문에 인자 검사가 먼저다.
    _parser().parse_args(argv)
    # control_station은 import 시점에 sys.argv로 모드(로그 파일 생성 여부)를
    # 정하므로, main에 넘기는 인자와 같게 맞춘 뒤 import한다.
    sys.argv[1:] = argv
    station = importlib.import_module(STATION_MODULE)
    return station.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
