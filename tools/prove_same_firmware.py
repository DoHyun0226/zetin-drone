#!/usr/bin/env python3
"""기준 리비전과 작업 트리의 스케치를 빌드해 ELF 를 비교한다.

비행 펌웨어의 소스 재배치(파일 사이 코드 이동, 주석, 이름 변경)가 ESP32 프로그램을
바꾸지 않았음을 증명한다.

    python tools/prove_same_firmware.py \
        --base-rev a41a6f6 \
        --base-path firmware/flight/dual_imu_cascade_flow_hold \
        --sketch firmware/flight/zetin_fc

기준 디렉터리는 ``git archive`` 로 통째로(헤더 포함) 임시 폴더에 꺼내고 후보 스케치
이름으로 바꾼다. 그래야 두 빌드의 스케치 이름(앱 설명자의 프로젝트 이름, 이름에서 나온
문자열)이 같다. 둘 다 같은 arduino-cli·코어·라이브러리·FQBN·옵션으로, ``--debug`` 로
준 ``WIFI_LATENCY_DEBUG`` 값마다(기본 0 과 1) 빌드한다. 비교는
``tools/compare_firmware_elf.py`` 가 한다.

호스트 쪽도 비교한다: native shim 으로 ``g++ -E -P -x c++``(SIL·native 시험이 컴파일하는
것), 빈 줄만 빼고 문자 단위로.

CI 와 같은 보드 코어가 깔린 arduino-cli 가 필요하다(firmware/README.md).
"""

import argparse
import hashlib
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
    # 재사용하는 빌드 폴더에 다른 고정 시각으로 컴파일된 코어 오브젝트가 남아 있을 수
    # 있다. 이 값으로 만든 폴더가 아니면 처음부터 다시 만든다.
    marker = build_dir / ".source_date_epoch"
    if build_dir.exists() and (not marker.is_file()
                               or marker.read_text().strip() != SOURCE_DATE_EPOCH):
        shutil.rmtree(build_dir)
    build_dir.mkdir(parents=True, exist_ok=True)
    marker.write_text(SOURCE_DATE_EPOCH + "\n")
    # ESP32 Arduino 코어는 부팅 보고에 빌드 날짜·시각(__DATE__/__TIME__)을 찍는다.
    # 같은 소스의 두 빌드가 바이트까지 같도록 고정한다.
    env = dict(os.environ, SOURCE_DATE_EPOCH=SOURCE_DATE_EPOCH)
    result = subprocess.run(cmd, capture_output=True, text=True, check=False, env=env)
    if result.returncode != 0:
        sys.stderr.write(result.stdout + result.stderr)
        raise SystemExit(f"build failed: {sketch} (debug={debug})")
    elf = build_dir / f"{sketch.name}.ino.elf"
    if not elf.is_file():
        raise SystemExit(f"no ELF at {elf}")
    return elf


def build_conditions():
    """기준 빌드를 재사용해도 되는지 가르는 빌드 조건: 고정 시각, arduino-cli, 코어, 라이브러리."""
    parts = [f"SOURCE_DATE_EPOCH={SOURCE_DATE_EPOCH}", f"FQBN={FQBN}"]
    for args in (["version"], ["core", "list"], ["lib", "list"]):
        r = subprocess.run(["arduino-cli", *args], capture_output=True, text=True, check=False)
        parts.append(r.stdout.strip())
    return hashlib.sha256("\n".join(parts).encode()).hexdigest()


def host_preprocess(sketch, debug):
    """호스트에서 본 프로그램(SIL·native 시험이 컴파일하는 것).

    native shim 으로 g++ -E -P. 빈 줄만 빼고 나머지는 문자 단위로(문자열 리터럴 포함)
    남긴다. 출력이 비면 거부한다 — -x c++ 없이는 g++ 가 .ino 를 무시하고 아무것도 찍지
    않는다.
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
        # 리비전·경로뿐 아니라 빌드 조건(고정 시각, arduino-cli·코어·라이브러리)이 같을 때만
        # 기준 빌드를 다시 쓴다(Codex 검토 P2).
        key = f"{rev} {args.base_path} {build_conditions()}"
        reuse = stamp.is_file() and stamp.read_text().strip() == key
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
            stamp.write_text(key + "\n")
        base = tmp / "base" / name
        # 경로 깊이와 이름을 같게: 후보를 기준 옆에 복사한다.
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
            marker = base_build / ".source_date_epoch"
            reusable = (base_elf.is_file() and marker.is_file()
                        and marker.read_text().strip() == SOURCE_DATE_EPOCH)
            elf_a = base_elf if reusable else build(base, base_build, debug, args.jobs)
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
