#!/usr/bin/env python3
"""Camera entry point: record (default) or mp4.

  drone_video.py [record] [--url --out-dir --no-preview --timeout]
  drone_video.py mp4 <csv> [--no-mp4]
"""

import importlib
import sys

SUBCOMMANDS = {
    "record": ("ground.camera_record", "record the camera stream with a live preview (default)"),
    "mp4": ("ground.camera_mp4", "aligned CSV and mp4 from a recording"),
}


def _usage():
    lines = ["usage: drone_video.py [record] [options] | drone_video.py mp4 <csv> [--no-mp4]", "", "subcommands:"]
    for name, (_, help_text) in SUBCOMMANDS.items():
        lines.append(f"  {name:<8}{help_text}")
    return "\n".join(lines)


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv and argv[0] in ("-h", "--help"):
        print(_usage())
        return 0
    if argv and not argv[0].startswith("-"):
        name, rest = argv[0], argv[1:]
        if name not in SUBCOMMANDS:
            print(f"drone_video.py: unknown subcommand: {name}\n\n{_usage()}", file=sys.stderr)
            return 2
    else:
        name, rest = "record", argv
    module = importlib.import_module(SUBCOMMANDS[name][0])
    return module.main(rest)


if __name__ == "__main__":
    raise SystemExit(main())
