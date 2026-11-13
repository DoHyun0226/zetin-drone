#!/usr/bin/env python3
"""Build a baseline revision and the working-tree sketch, then compare ELFs.

Proves that a source reorganisation of the flight firmware (moving code
between files, comments, renames) leaves the ESP32 program unchanged.

    python tools/prove_same_firmware.py \
        --base-rev a41a6f6 \
        --base-path firmware/flight/dual_imu_cascade_flow_hold \
        --sketch firmware/flight/zetin_fc

The baseline directory is exported whole with ``git archive`` (headers
included) into a temporary folder and renamed to the candidate sketch's name,
so both builds share the sketch name (the app descriptor's project name and
any name-derived strings). Both are built with the same arduino-cli, core,
libraries, FQBN and options, once per ``--debug`` value of
``WIFI_LATENCY_DEBUG`` (default: 0 and 1). The comparison is
``tools/compare_firmware_elf.py``.

The host view is compared too: ``g++ -E -P -x c++`` with the native shims
(what the SIL and native tests compile), blank lines dropped, otherwise
character for character.

Needs arduino-cli with the board core used by CI (see firmware/README.md).
"""

import argparse
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

import compare_firmware_elf  # noqa: E402

FQBN = "esp32:esp32:esp32s3"
SOURCE_DATE_EPOCH = "1794614400"   # 2026-11-13 00:00 UTC, fixed


def export_baseline(rev, base_path, dest_parent, name):
    archive = dest_parent / "base.tar"
    with archive.open("wb") as out:
        subprocess.run(["git", "-C", str(REPO_ROOT), "archive", rev, base_path],
                       stdout=out, check=True)
    with tarfile.open(archive) as tar:
        tar.extractall(dest_parent / "extract", filter="data")
    src = dest_parent / "extract" / base_path
    if not src.is_dir():
        raise SystemExit(f"{base_path} not found at {rev}")
    sketch = dest_parent / "base" / name
    shutil.copytree(src, sketch)
    inos = sorted(sketch.glob("*.ino"))
    if len(inos) != 1:
        raise SystemExit(f"baseline must have exactly one .ino, found {inos}")
    inos[0].rename(sketch / f"{name}.ino")
    return sketch


def build(sketch, build_dir, debug, jobs):
    cmd = ["arduino-cli", "compile", "--fqbn", FQBN, "--build-path", str(build_dir)]
    if jobs:
        cmd += ["--jobs", str(jobs)]
    if debug is not None:
        cmd += ["--build-property",
                f"compiler.cpp.extra_flags=-DWIFI_LATENCY_DEBUG={debug}"]
    cmd.append(str(sketch))
    # A reused build folder may hold core objects compiled with another pinned
    # time; start it over unless it was made with this one.
    marker = build_dir / ".source_date_epoch"
    if build_dir.exists() and (not marker.is_file()
                               or marker.read_text().strip() != SOURCE_DATE_EPOCH):
        shutil.rmtree(build_dir)
    build_dir.mkdir(parents=True, exist_ok=True)
    marker.write_text(SOURCE_DATE_EPOCH + "\n")
    # The ESP32 Arduino core prints a build date/time (__DATE__/__TIME__) in its
    # boot report; pin it so two builds of the same source are byte-identical.
    env = dict(os.environ, SOURCE_DATE_EPOCH=SOURCE_DATE_EPOCH)
    result = subprocess.run(cmd, capture_output=True, text=True, check=False, env=env)
    if result.returncode != 0:
        sys.stderr.write(result.stdout + result.stderr)
        raise SystemExit(f"build failed: {sketch} (debug={debug})")
    elf = build_dir / f"{sketch.name}.ino.elf"
    if not elf.is_file():
        raise SystemExit(f"no ELF at {elf}")
    return elf


def host_preprocess(sketch, debug):
    """Host view of the program (what the SIL and native tests compile).

    g++ -E -P with the native shims; blank lines dropped, everything else kept
    character for character (string literals included). Refuses empty output:
    without -x c++ g++ ignores a .ino and prints nothing.
    """
    shims = REPO_ROOT / "tools" / "native_tests" / "shims"
    ino = sketch / f"{sketch.name}.ino"
    cmd = ["g++", "-std=c++17", "-m32", "-E", "-P", "-x", "c++",
           f"-DWIFI_LATENCY_DEBUG={debug}", "-I", str(shims), "-I", str(sketch),
           str(ino)]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        sys.stderr.write(result.stderr)
        raise SystemExit(f"host preprocessing failed: {ino}")
    lines = [line for line in result.stdout.splitlines() if line.strip()]
    if len(lines) < 1000:
        raise SystemExit(f"host preprocessing of {ino} produced only {len(lines)} lines")
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--base-rev", required=True)
    ap.add_argument("--base-path", required=True,
                    help="sketch directory at the baseline revision")
    ap.add_argument("--sketch", required=True, help="working-tree sketch directory")
    ap.add_argument("--debug", nargs="*", default=["0", "1"],
                    help="WIFI_LATENCY_DEBUG values to build (default: 0 1)")
    ap.add_argument("--host-only", action="store_true",
                    help="only compare host preprocessing (no ESP32 builds)")
    ap.add_argument("--jobs", type=int, default=0,
                    help="parallel compile jobs (default: arduino-cli's own default)")
    ap.add_argument("--keep", type=Path,
                    help="keep sources and builds here; an existing baseline build "
                         "for the same revision is reused instead of rebuilt")
    args = ap.parse_args()
    candidate = (REPO_ROOT / args.sketch).resolve()
    name = candidate.name
    failed = False
    holder = None
    if args.keep:
        tmp = args.keep.resolve()
        tmp.mkdir(parents=True, exist_ok=True)
    else:
        holder = tempfile.TemporaryDirectory(prefix="prove-fw-")
        tmp = Path(holder.name)
    try:
        rev = subprocess.run(["git", "-C", str(REPO_ROOT), "rev-parse", args.base_rev],
                             capture_output=True, text=True, check=True).stdout.strip()
        stamp = tmp / "base.rev"
        reuse = stamp.is_file() and stamp.read_text().strip() == f"{rev} {args.base_path}"
        if not reuse:
            for sub in ("base", "extract", "base.tar"):
                path = tmp / sub
                if path.is_dir():
                    shutil.rmtree(path)
                elif path.exists():
                    path.unlink()
            for old in tmp.glob("build-base-*"):
                shutil.rmtree(old)
            export_baseline(rev, args.base_path, tmp, name)
            stamp.write_text(f"{rev} {args.base_path}\n")
        base = tmp / "base" / name
        # Same path depth and name for both: copy the candidate next to the baseline.
        cand = tmp / "cand" / name
        if cand.exists():
            shutil.rmtree(cand)
        shutil.copytree(candidate, cand)
        for debug in args.debug:
            host_a, host_b = host_preprocess(base, debug), host_preprocess(cand, debug)
            if host_a != host_b:
                failed = True
                first = next((i for i, (x, y) in enumerate(zip(host_a, host_b)) if x != y),
                             min(len(host_a), len(host_b)))
                print(f"[debug={debug}] HOST DIFF at non-blank line {first}: "
                      f"{host_a[first:first + 1]} vs {host_b[first:first + 1]}")
            else:
                print(f"[debug={debug}] host preprocessing identical "
                      f"({len(host_a)} non-blank lines)")
            if args.host_only:
                continue
            base_build = tmp / f"build-base-{debug}"
            base_elf = base_build / f"{name}.ino.elf"
            elf_a = base_elf if base_elf.is_file() else build(base, base_build, debug,
                                                              args.jobs)
            elf_b = build(cand, tmp / f"build-cand-{debug}", debug, args.jobs)
            diffs, notes = compare_firmware_elf.compare(elf_a, elf_b)
            for note in notes:
                print(f"[debug={debug}] note: {note}")
            if diffs:
                failed = True
                for d in diffs:
                    print(f"[debug={debug}] DIFF {d}")
            else:
                n = len(compare_firmware_elf.Elf32(elf_a).allocated())
                print(f"[debug={debug}] identical: entry, {n} allocated sections "
                      f"(layout and bytes; app descriptor time/date/sha256 masked)")
    finally:
        if holder is not None:
            holder.cleanup()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
