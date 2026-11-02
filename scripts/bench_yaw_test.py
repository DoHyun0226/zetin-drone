#!/usr/bin/env python3
"""전원 인가 벤치 Stage C(yaw): yaw 부호 자동 측정 (모터 무전원·무프롭 안전).

Stage A에서 프롭 회전방향이 확정됐으므로(FL/RR=CW, FR/RL=CCW), yaw 부호는 프롭
없이 텔레메트리로 판정할 수 있다(SIL S5 미해결 항목 닫기).

불변식(프롭 반작용은 프롭 회전의 반대 방향 토크):
  - 기체를 위에서 볼 때 CW로 비틀면 → 복원하려면 CCW 토크 → CW프롭(M1 FL, M2 RR)
    을 올려야 정상.
  - CCW로 비틀면 → CW 토크 → CCW프롭(M3 FR, M4 RL)을 올려야 정상.
  - 반대 쌍이 올라가면 yaw 부호 반전(증폭) → 믹서 yaw 부호를 뒤집어야 한다.

사람이 화면 큐에 맞춰 기체를 비틀기만 하면 스스로 측정·판정·기록하고 WiFi를
복원한다. 결과 파일은 WiFi 복원 뒤 Claude가 읽는다.

사용법(노트북 ZETIN-ROBOT/온라인에서 실행):
    /home/light/anaconda3/bin/python scripts/bench_yaw_test.py
"""
import datetime
import os
import signal
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground.telemetry_schema import parse_telemetry_packet  # noqa: E402

DRONE_SSID = "Drone_Tuning"
HOME_SSID = "ZETIN-ROBOT"
UDP_IP, UDP_PORT = "192.168.4.1", 4210
THROTTLE_CMD = "th 1150"
YAW_MIN_DEG = 10.0      # 이 미만이면 "비틀림 부족"
MARGIN_US = 4.0

GRAB_DELAY = 8.0
ANNOUNCE = 2.5
COUNTDOWN = 3
MEASURE = 3.0
BETWEEN = 1.5

RESULT_DIR = ("/tmp/claude-1000/-home-light-ZETIN-robotics-zetin-drone/"
              "e5f57906-8a56-4045-ab92-005d7cf74487/scratchpad")

# 프롭 회전으로 묶은 쌍: CW프롭=M1(FL)+M2(RR), CCW프롭=M3(FR)+M4(RL)
CW_PROPS = ["M1", "M2"]     # 올라가면 CCW 반작용
CCW_PROPS = ["M3", "M4"]    # 올라가면 CW 반작용
MOTOR_FIELD = {"M1": "Motor_M1", "M2": "Motor_M2", "M3": "Motor_M3", "M4": "Motor_M4"}
MOTOR_POS = {"M1": "FL", "M2": "RR", "M3": "FR", "M4": "RL"}

PHASES = [
    dict(key="level",   instr="수평·정지로 가만히 유지",                       rise=None),
    dict(key="yaw_cw",  instr="수평 유지하며 위에서 볼 때 시계(CW)로 ~40° 비틀어 유지",   rise=CW_PROPS),
    dict(key="yaw_ccw", instr="수평 유지하며 반시계(CCW)로 ~40° 비틀어 유지",           rise=CCW_PROPS),
]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", UDP_PORT))
sock.settimeout(0.3)

_running = True
_seq = 0
_lock = threading.Lock()
_latest = {}
_collecting = False
_buffer = []
_log_lines = []


def log(msg=""):
    print(msg, flush=True)
    _log_lines.append(msg)


def nmcli_up(ssid):
    try:
        subprocess.run(["nmcli", "connection", "up", ssid],
                       capture_output=True, text=True, timeout=30)
    except Exception as exc:  # noqa: BLE001
        log(f"[WIFI-ERR] up {ssid}: {exc}")


def wait_reachable(timeout=25.0):
    end = time.time() + timeout
    while time.time() < end:
        if subprocess.run(["ping", "-c1", "-W1", UDP_IP], capture_output=True).returncode == 0:
            return True
        time.sleep(0.5)
    return False


def send(cmd):
    try:
        sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))
    except OSError:
        pass


def sender():
    global _seq
    send("connect")
    time.sleep(0.2)
    for _ in range(3):
        send("trim 0 0")
        time.sleep(0.05)
    send("yaw 1")          # 시동 해제 상태에서만 수락된다 (start보다 먼저)
    time.sleep(0.05)
    for _ in range(3):
        send("start")
        time.sleep(0.05)
    send(THROTTLE_CMD)
    while _running:
        _seq += 1
        send(f"rc {_seq} 0 0 0")   # 오버라이드 중이므로 yaw 절대각 0 유지
        time.sleep(0.05)


def receiver():
    global _latest
    while _running:
        try:
            data, _ = sock.recvfrom(2048)
        except (socket.timeout, OSError):
            continue
        line = data.decode(errors="replace").strip()
        if not line or line.startswith("GAINS"):
            continue
        try:
            s = parse_telemetry_packet(line)
        except Exception:  # noqa: BLE001
            continue
        with _lock:
            _latest = s
            if _collecting:
                _buffer.append(s)


def collect_window(seconds):
    global _collecting
    with _lock:
        _buffer.clear()
        _collecting = True
    time.sleep(seconds)
    with _lock:
        _collecting = False
        return list(_buffer)


def mean(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else float("nan")


def cleanup_and_restore():
    global _running
    _running = False
    for _ in range(5):
        send("stop")
        time.sleep(0.02)
    try:
        sock.close()
    except OSError:
        pass
    log(f"\n[정리] stop 전송. WiFi {HOME_SSID} 복원 중...")
    nmcli_up(HOME_SSID)
    log("[정리] WiFi 복원 완료.")


def save_results(verdict_lines):
    os.makedirs(RESULT_DIR, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(RESULT_DIR, f"yaw_test_{ts}.txt")
    with open(path, "w") as f:
        f.write("\n".join(_log_lines))
        f.write("\n\n===== VERDICT =====\n")
        f.write("\n".join(verdict_lines))
        f.write("\n")
    return path


def sigint(*_):
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGINT, sigint)
    log("=" * 60)
    log(" 전원 인가 벤치 Stage C(yaw) : yaw 부호 자동 측정")
    log(" (모터 무전원·프롭 없이 — 기체를 두 손으로 잡고 비틀기만)")
    log("=" * 60)

    log(f"[WIFI] {DRONE_SSID} 전환...")
    nmcli_up(DRONE_SSID)
    if not wait_reachable():
        log(f"[에러] ESP({UDP_IP}) 도달 불가.")
        cleanup_and_restore()
        save_results(["ABORTED: ESP 도달 불가"])
        return

    threading.Thread(target=sender, daemon=True).start()
    threading.Thread(target=receiver, daemon=True).start()

    log("[대기] 텔레메트리/시동 확인...")
    armed = False
    for _ in range(40):
        time.sleep(0.15)
        with _lock:
            s = dict(_latest)
        if s and s.get("Armed"):
            armed = True
            break
    with _lock:
        s = dict(_latest)
    if not s:
        log("[에러] 텔레메트리 수신 없음.")
        cleanup_and_restore()
        save_results(["ABORTED: 텔레메트리 없음"])
        return
    if not armed:
        log(f"[에러] 시동 실패. calib={s.get('Calibration_OK')} "
            f"RC={s.get('Fault_RC')} IMU1={s.get('Fault_IMU1')} "
            f"IMU2={s.get('Fault_IMU2')} disagree={s.get('Fault_Disagree')} "
            f"att={s.get('Fault_Attitude')}")
        cleanup_and_restore()
        save_results(["ABORTED: 시동 실패"])
        return
    log(f"[OK] 시동 완료. calib={s.get('Calibration_OK')} armed={s.get('Armed')} "
        f"loopHz={s.get('PID_Loop_Hz')} (yaw hold ON)")

    log(f"\n>>> {GRAB_DELAY:.0f}초 후 시작. 기체를 두 손으로 잡으세요. <<<")
    for i in range(int(GRAB_DELAY), 0, -1):
        log(f"    ...{i}")
        time.sleep(1.0)

    fault_keys = ("Fault_RC", "Fault_Critical", "Fault_IMU1", "Fault_IMU2",
                  "Fault_Disagree", "Fault_Attitude")
    results = {}
    for ph in PHASES:
        log("\n" + "-" * 50)
        log(f"[동작] {ph['instr']}")
        # yaw 1은 시동 해제 상태에서만 수락되므로 start보다 먼저 보낸다.
        # 이미 무장 중이면 거부되지만 그때는 오버라이드가 이미 켜져 있고,
        # 중간에 disarm됐다면 이 순서라야 재무장 전에 다시 켤 수 있다.
        send("yaw 1")
        time.sleep(0.05)
        for _ in range(3):     # 수평 상태에서 재시동(혹시 disarm됐으면 복구)
            send("start")
            time.sleep(0.05)
        send(THROTTLE_CMD)
        time.sleep(ANNOUNCE)
        for c in range(COUNTDOWN, 0, -1):
            log(f"    측정까지 {c}...")
            time.sleep(1.0)
        log("    ● 측정 중 — 자세 유지!")
        samples = collect_window(MEASURE)
        armed_frac = mean([1.0 if x.get("Armed") else 0.0 for x in samples]) if samples else 0.0
        mm = {m: mean([x.get(MOTOR_FIELD[m]) for x in samples]) for m in MOTOR_FIELD}
        yaw = mean([x.get("Yaw") for x in samples])
        seen = sorted({k.replace("Fault_", "") for x in samples for k in fault_keys if x.get(k)})
        results[ph["key"]] = dict(phase=ph, motors=mm, yaw=yaw, n=len(samples),
                                  armed_frac=armed_frac, faults=seen)
        log(f"    M1(FL)={mm['M1']:.0f} M2(RR)={mm['M2']:.0f} "
            f"M3(FR)={mm['M3']:.0f} M4(RL)={mm['M4']:.0f} | "
            f"Yaw={yaw:+.1f} (n={len(samples)}, armed={armed_frac:.0%}, "
            f"faults={','.join(seen) if seen else '-'})")
        log("    → 수평·정지로 복귀")
        time.sleep(BETWEEN)

    # ---- 판정 ----
    verdict = []
    verdict.append("phase     CWprops(M1+M2)  CCWprops(M3+M4)  rise_pair  margin  yaw(deg)  result")
    all_pass = True
    for ph in PHASES:
        if ph["key"] == "level":
            continue
        r = results[ph["key"]]
        mm = r["motors"]
        cw_mean = mean([mm[m] for m in CW_PROPS])
        ccw_mean = mean([mm[m] for m in CCW_PROPS])
        rise = ph["rise"]
        rise_is_cw = (rise == CW_PROPS)
        rise_mean = cw_mean if rise_is_cw else ccw_mean
        other_mean = ccw_mean if rise_is_cw else cw_mean
        margin = rise_mean - other_mean
        twist_ok = abs(r["yaw"]) >= YAW_MIN_DEG
        disarmed = r["armed_frac"] < 0.8
        rise_lbl = "+".join(f"{m}({MOTOR_POS[m]})" for m in rise)
        if disarmed:
            res = f"INVALID(측정중 disarm, faults={','.join(r['faults']) or '?'})"
            all_pass = False
        elif not twist_ok:
            res = "INCONCLUSIVE(비틀림부족)"
            all_pass = False
        elif margin >= MARGIN_US:
            res = "PASS(복원)"
        else:
            res = "FAIL(반대쌍↑ = yaw 부호 반전/증폭)"
            all_pass = False
        verdict.append(
            f"{ph['key']:<9} {cw_mean:12.0f}  {ccw_mean:14.0f}   {rise_lbl:<12} "
            f"{margin:+6.0f}  {r['yaw']:+7.1f}   {res}")

    yc = results["yaw_cw"]["yaw"]
    ycc = results["yaw_ccw"]["yaw"]
    verdict.append("")
    verdict.append(f"Yaw 텔레메트리 부호: CW twist={yc:+.1f}, CCW twist={ycc:+.1f} "
                   f"(반대여야 정상: {'OK' if yc * ycc < 0 else '이상'})")
    verdict.append("")
    if all_pass:
        verdict.append("종합: yaw 부호 정상 ✅ (비틀림을 거스르는 프롭 쌍이 올라감 → 복원)")
    else:
        verdict.append("종합: 검토 필요 ⚠️ (위 표 — FAIL이면 믹서 yaw 부호 반전 필요, "
                       "INVALID/INCONCLUSIVE면 재측정)")

    log("\n" + "=" * 60)
    for line in verdict:
        log(line)
    log("=" * 60)
    return verdict


if __name__ == "__main__":
    verdict = ["(no verdict — aborted early)"]
    try:
        v = main()
        if v:
            verdict = v
    except KeyboardInterrupt:
        log("\n[중단] Ctrl-C")
    finally:
        cleanup_and_restore()
        path = save_results(verdict)
        log(f"\n[결과 저장] {path}")
        log(f"이제 {HOME_SSID} 로 복원됐습니다. Claude에게 '측정 끝' 이라고 하세요.")
