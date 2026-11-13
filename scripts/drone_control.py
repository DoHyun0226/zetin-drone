#!/usr/bin/env python3
"""지상국 진입점: DualSense 비행(기본), --keyboard, --listen.

  drone_control.py [--keyboard | --listen] [--no-raw] [--no-mag] [--plain]
  drone_control.py --listen --plot       실시간 텔레메트리 그래프 창 + CSV
  drone_control.py --pad-check [--live]  게임패드 진단. 아무것도 보내지 않는다

드론과 UDP로 통신하는 도구는 한 번에 하나만 둔다: 펌웨어는 마지막으로 보낸
쪽에 텔레메트리를 보내고, --listen도 ``connect``를 보낸다.
"""

import argparse
import importlib
import sys

STATION_MODULE = "ground.control_station"
PAD_CHECK_MODULE = "ground.pad_check"
PLOT_MODULE = "ground.monitor_plot"
STATION_ONLY_FLAGS = ("no_raw", "no_mag", "plain")


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
    modes.add_argument(
        "--pad-check",
        action="store_true",
        help="gamepad axis/button/hat diagnosis; sends nothing to the drone",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="with --listen: also show the live telemetry plot window",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="with --pad-check: print every value continuously",
    )
    parser.add_argument(
        "--no-raw",
        action="store_true",
        help="disable the firmware raw stream and do not create IMU raw logs; "
             "with --listen it only disables local raw capture (nothing is sent)",
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
    parser = _parser()
    args = parser.parse_args(argv)
    if args.plot and not args.listen:
        parser.error("--plot is only valid together with --listen")
    if args.live and not args.pad_check:
        parser.error("--live is only valid together with --pad-check")
    if args.pad_check or args.plot:
        mode = "--pad-check" if args.pad_check else "--listen --plot"
        for name in STATION_ONLY_FLAGS:
            if getattr(args, name):
                parser.error(f"--{name.replace('_', '-')} does not apply to {mode}")
        # 이 두 모드는 지상국(UDP 4210 점유·로그 파일 생성)을 import하지 않는다.
        module = importlib.import_module(PAD_CHECK_MODULE if args.pad_check
                                         else PLOT_MODULE)
        return module.main(argv)
    # control_station은 import 시점에 sys.argv로 모드(로그 파일 생성 여부)를
    # 정하므로, main에 넘기는 인자와 같게 맞춘 뒤 import한다.
    sys.argv[1:] = argv
    station = importlib.import_module(STATION_MODULE)
    return station.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
