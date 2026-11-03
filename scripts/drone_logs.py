#!/usr/bin/env python3
"""Offline log analysis entry point: flight, imuraw, probe, magcal."""

import importlib
import sys

SUBCOMMANDS = {
    "flight": ("ground.flight_log", "flight CSV summary and plots"),
    "imuraw": ("ground.imu_raw", "decode a raw ZIMU/ZCAL .bin capture to CSV"),
    "probe": ("ground.probe_response", "auto-land probe response analysis"),
    "magcal": ("ground.magcal", "fit magnetometer hard/soft-iron constants"),
}


def _usage():
    lines = ["usage: drone_logs.py <subcommand> [args...]", "", "subcommands:"]
    for name, (_, help_text) in SUBCOMMANDS.items():
        lines.append(f"  {name:<8}{help_text}")
    return "\n".join(lines)


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(_usage())
        return 0
    name, rest = argv[0], argv[1:]
    if name not in SUBCOMMANDS:
        print(f"drone_logs.py: unknown subcommand: {name}\n\n{_usage()}", file=sys.stderr)
        return 2
    module = importlib.import_module(SUBCOMMANDS[name][0])
    return module.main(rest)


if __name__ == "__main__":
    raise SystemExit(main())
