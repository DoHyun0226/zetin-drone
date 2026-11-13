#!/usr/bin/env python3
"""기준 리비전과 작업 트리에 차분 하네스를 돌린다.

ELF 가 같을 수 없는 구조 리팩터링(함수 분해, 선언 이동)이 비행 펌웨어의 동작을 바꾸지
않았음을 증명한다. 두 스케치를 같은 shim·플랜트로 tools/native_tests/diff_harness 에
컴파일해 시나리오마다 돌리고, 기록(모든 모터 PWM 쓰기, 모든 UDP 데이터그램, 시리얼 출력,
NVS 쓰기, float 을 비트로 적은 tick 별 상태)이 바이트까지 같아야 한다. 시나리오는 노린
분기에 실제로 도달해야 한다(못 하면 하네스 종료 코드 3).

    python tools/diff_firmware_behaviour.py \
        --base-rev a41a6f6 \
        --base-path firmware/flight/dual_imu_cascade_flow_hold \
        --sketch firmware/flight/zetin_fc

``--sensitivity`` 를 주면 기준 복사본에 출력 채널마다 변형 하나씩을 심고, 비교가 그 각각을
잡아내는지도 확인한다.

WIFI_LATENCY_DEBUG 0 과 1 로 빌드한다(지연 시나리오는 디버그 빌드에만 있다).
"""

import argparse
import filecmp
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from prove_same_firmware import export_baseline  # noqa: E402
from ground import firmware_source  # noqa: E402

NATIVE = REPO_ROOT / "tools" / "native_tests"
HARNESS = NATIVE / "diff_harness" / "diff_harness.cpp"

# 관측 채널마다 심는 변형 하나: (이름, 바꿀 문자열, 새 문자열).
SENSITIVITY = [
    ("motor (descent delta)", "FS_DESCENT_DELTA_US = 60;", "FS_DESCENT_DELTA_US = 61;"),
    ("UDP gains reply", 'udp.printf("GAINS,', 'udp.printf("GAINZ,'),
    ("UDP telemetry field", '(int)landCutSnapshot, landReqSnapshot, (int)rangeland_enabled);',
     '(int)landCutSnapshot, landReqSnapshot + 1, (int)rangeland_enabled);'),
    ("NVS key", 'prefs.putBytes("cal",', 'prefs.putBytes("caL",'),
    ("serial text", '">>> Trim R:%.2f P:%.2f\\n"', '">>> Trim R:%.2f  P:%.2f\\n"'),
    ("ZIMU header version", "IMU_RAW_VERSION = 2;", "IMU_RAW_VERSION = 3;"),
    ("UDP destination", "laptopPort = udp.remotePort();", "laptopPort = udp.remotePort() + 1;"),
    ("hover estimator", "HOVER_LPF_TAU_S        = 3.0f;", "HOVER_LPF_TAU_S        = 3.01f;"),
]


def compile_harness(sketch, out, debug):
    cmd = ["g++", "-std=c++17", "-O1", "-m32", "-pthread", "-w",
           f"-DWIFI_LATENCY_DEBUG={debug}",
           "-I", str(NATIVE / "shims"), "-I", str(sketch), "-I", str(NATIVE),
           "-x", "c++", str(HARNESS), "-o", str(out)]
    r = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        raise SystemExit(f"harness build failed for {sketch} (debug={debug})")


def scenarios(binary):
    r = subprocess.run([str(binary), "--list"], capture_output=True, text=True, check=True)
    return r.stdout.split()


def run_all(binary, logdir):
    logdir.mkdir(parents=True, exist_ok=True)
    unreached = {}
    for name in scenarios(binary):
        r = subprocess.run([str(binary), name, str(logdir / f"{name}.log")],
                           capture_output=True, text=True, check=False, timeout=600)
        if r.returncode == 3:
            unreached[name] = r.stderr.strip()
        elif r.returncode != 0:
            raise SystemExit(f"{binary} {name} failed ({r.returncode}): {r.stderr}")
    return unreached


def first_diff(a, b):
    with a.open("rb") as fa, b.open("rb") as fb:
        for n, (la, lb) in enumerate(zip(fa, fb), 1):
            if la != lb:
                return n, la[:200], lb[:200]
    return None, b"(length differs)", b""


def compare_dirs(base_dir, cand_dir, names):
    diffs = []
    for name in names:
        a, b = base_dir / f"{name}.log", cand_dir / f"{name}.log"
        if not filecmp.cmp(a, b, shallow=False):
            diffs.append((name, *first_diff(a, b)))
    return diffs


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--base-rev", required=True)
    ap.add_argument("--base-path", required=True)
    ap.add_argument("--sketch", required=True,
                    help="후보 스케치 경로(작업 트리, 또는 --cand-rev 의 그 리비전 안 경로)")
    ap.add_argument("--cand-rev", help="작업 트리 대신 이 리비전의 스케치를 후보로 쓴다")
    ap.add_argument("--sensitivity", action="store_true")
    ap.add_argument("--keep", type=Path, help="keep builds and logs here")
    args = ap.parse_args()
    cand_src = (REPO_ROOT / args.sketch).resolve()
    holder = None
    if args.keep:
        tmp = args.keep.resolve()
        if tmp.exists():
            shutil.rmtree(tmp)
        tmp.mkdir(parents=True)
    else:
        holder = tempfile.TemporaryDirectory(prefix="diff-fw-")
        tmp = Path(holder.name)
    failed = False
    try:
        base = export_baseline(args.base_rev, args.base_path, tmp, cand_src.name)
        if args.cand_rev:
            (tmp / "candrev").mkdir()
            cand = export_baseline(args.cand_rev, args.sketch, tmp / "candrev", cand_src.name)
        else:
            cand = tmp / "cand" / cand_src.name
            shutil.copytree(cand_src, cand)
        for debug in ("0", "1"):
            bins = {}
            for label, sketch in (("base", base), ("cand", cand)):
                bins[label] = tmp / f"dh-{label}-{debug}"
                compile_harness(sketch, bins[label], debug)
            names = scenarios(bins["cand"])
            if names != scenarios(bins["base"]):
                raise SystemExit("scenario lists differ between builds")
            logs = {}
            for label in bins:
                logs[label] = tmp / f"logs-{label}-{debug}"
                unreached = run_all(bins[label], logs[label])
                for name, why in unreached.items():
                    failed = True
                    print(f"[debug={debug}] {label}: {why}")
            diffs = compare_dirs(logs["base"], logs["cand"], names)
            for name, line, a, b in diffs:
                failed = True
                print(f"[debug={debug}] DIFF {name} line {line}:\n  base {a!r}\n  cand {b!r}")
            if not diffs:
                print(f"[debug={debug}] {len(names)} scenarios identical: {' '.join(names)}")
        if args.sensitivity:
            for label, old, new in SENSITIVITY:
                mut = tmp / "mut" / label.replace(" ", "_").replace("(", "").replace(")", "") / cand_src.name
                shutil.copytree(base, mut)
                firmware_source.mutate_unique(mut, old, new)
                binary = tmp / "dh-mut"
                compile_harness(mut, binary, "0")
                mlogs = tmp / "logs-mut"
                if mlogs.exists():
                    shutil.rmtree(mlogs)
                run_all(binary, mlogs)
                names = scenarios(binary)
                caught = compare_dirs(tmp / "logs-base-0", mlogs, names)
                if caught:
                    print(f"[sensitivity] {label}: caught in {len(caught)} scenario(s), "
                          f"e.g. {caught[0][0]} line {caught[0][1]}")
                else:
                    failed = True
                    print(f"[sensitivity] {label}: NOT caught")
    finally:
        if holder is not None:
            holder.cleanup()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
