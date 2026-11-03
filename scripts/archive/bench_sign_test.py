#!/usr/bin/env python3
"""전원 인가 벤치 Stage B-3: roll/pitch 부호 자동 측정 (모터 무전원 안전).

Drone_Tuning에 붙어 있는 동안엔 노트북이 인터넷(→Claude)에 못 붙으므로, 사람이
화면 큐에 맞춰 기체를 기울이기만 하면 이 스크립트가 스스로 측정·판정·기록하고
WiFi까지 복원한다. 결과 파일은 WiFi를 되돌린 뒤 Claude가 읽고 해석한다.

원리: 모터 전원이 꺼져 있어도 펌웨어는 자세 오차에 대한 모터 명령(motorOut)을
계산해 텔레메트리로 보낸다. "누른(아래로 내린) 쪽 모터가 올라가야 한다"는
관례 무관 불변식을 motorOut 숫자로 검증한다. 프로펠러는 돌지 않으므로 안전하다.

사용법(노트북이 ZETIN-ROBOT/온라인 상태에서 실행):
    /home/light/anaconda3/bin/python scripts/archive/bench_sign_test.py
실행하면 8초 뒤 시작한다. 그동안 기체를 프로펠러 없이 두 손으로 잡는다.
중단하려면 Ctrl-C — stop 전송 후 WiFi를 복원한다.
"""
import datetime
import os
import signal
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ground.telemetry_schema import parse_telemetry_packet  # noqa: E402

# ---- 설정 ----
DRONE_SSID = "Drone_Tuning"
HOME_SSID = "ZETIN-ROBOT"          # 측정 후 복원할 네트워크
UDP_IP, UDP_PORT = "192.168.4.1", 4210
THROTTLE_CMD = "th 1150"
TILT_MIN_DEG = 6.0                  # 이 각도 미만이면 "기울임 부족"
MARGIN_US = 4.0                     # down-side가 up-side보다 이만큼은 커야 PASS

GRAB_DELAY = 8.0
ANNOUNCE = 2.5
COUNTDOWN = 3
MEASURE = 3.0
BETWEEN = 1.5

RESULT_DIR = ("/tmp/claude-1000/-home-light-ZETIN-robotics-zetin-drone/"
              "e5f57906-8a56-4045-ab92-005d7cf74487/scratchpad")

# 모터: M1=Motor_M1(FL) M2=Motor_M2(RR) M3=Motor_M3(FR) M4=Motor_M4(RL)
PHASES = [
    dict(key="level",      instr="수평(레벨)으로 가만히 유지",           down=[],            axis=None),
    dict(key="roll_right", instr="우측(오른쪽)을 아래로 기울여 유지",     down=["M2", "M3"],  axis="Roll"),
    dict(key="roll_left",  instr="좌측(왼쪽)을 아래로 기울여 유지",       down=["M1", "M4"],  axis="Roll"),
    dict(key="pitch_nose", instr="기수(앞)를 아래로 숙여 유지",          down=["M1", "M3"],  axis="Pitch"),
    dict(key="pitch_tail", instr="꼬리(뒤)를 아래로 숙여 유지",          down=["M2", "M4"],  axis="Pitch"),
]
MOTOR_FIELD = {"M1": "Motor_M1", "M2": "Motor_M2", "M3": "Motor_M3", "M4": "Motor_M4"}
MOTOR_POS = {"M1": "FL", "M2": "RR", "M3": "FR", "M4": "RL"}

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
        r = subprocess.run(["ping", "-c1", "-W1", UDP_IP],
                           capture_output=True)
        if r.returncode == 0:
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
    for _ in range(3):
        send("start")
        time.sleep(0.05)
    send(THROTTLE_CMD)
    while _running:
        _seq += 1
        send(f"rc {_seq} 0 0 0")
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
        samples = list(_buffer)
    return samples


def mean(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else float("nan")


def motor_means(samples):
    return {m: mean([s.get(MOTOR_FIELD[m]) for s in samples]) for m in MOTOR_FIELD}


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
    log("\n[정리] stop 전송 완료. WiFi를 %s(으)로 복원 중..." % HOME_SSID)
    nmcli_up(HOME_SSID)
    log("[정리] WiFi 복원 완료.")


def save_results(verdict_lines):
    os.makedirs(RESULT_DIR, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(RESULT_DIR, f"sign_test_{ts}.txt")
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
    log(" 전원 인가 벤치 Stage B-3 : roll/pitch 부호 자동 측정")
    log(" (모터 무전원 — 프로펠러 없이 두 손으로 기체를 잡으세요)")
    log("=" * 60)

    log(f"[WIFI] {DRONE_SSID} 로 전환...")
    nmcli_up(DRONE_SSID)
    if not wait_reachable():
        log(f"[에러] ESP({UDP_IP}) 도달 불가. 보드 전원/부팅/AP 확인.")
        cleanup_and_restore()
        save_results(["ABORTED: ESP 도달 불가"])
        return

    threading.Thread(target=sender, daemon=True).start()
    threading.Thread(target=receiver, daemon=True).start()

    # 텔레메트리 + calib + arm 대기
    log("[대기] 텔레메트리/시동 확인...")
    armed = False
    for _ in range(40):  # ~6s
        time.sleep(0.15)
        with _lock:
            s = dict(_latest)
        if s:
            if not s.get("Calibration_OK"):
                log(f"    calib={s.get('Calibration_OK')} armed={s.get('Armed')} "
                    f"(캘리브레이션 대기/실패 — 기체를 수평·정지 상태로)")
            if s.get("Armed"):
                armed = True
                break
    with _lock:
        s = dict(_latest)
    if not s:
        log("[에러] 텔레메트리 수신 없음. WiFi/보드 확인.")
        cleanup_and_restore()
        save_results(["ABORTED: 텔레메트리 없음"])
        return
    if not armed:
        log(f"[에러] 시동 실패. calib={s.get('Calibration_OK')} "
            f"faults: RC={s.get('Fault_RC')} IMU1={s.get('Fault_IMU1')} "
            f"IMU2={s.get('Fault_IMU2')} disagree={s.get('Fault_Disagree')} "
            f"attitude={s.get('Fault_Attitude')}")
        cleanup_and_restore()
        save_results(["ABORTED: 시동 실패 (위 로그 참조)"])
        return
    log(f"[OK] 시동 완료. calib={s.get('Calibration_OK')} armed={s.get('Armed')} "
        f"loopHz={s.get('PID_Loop_Hz')}")

    log(f"\n>>> {GRAB_DELAY:.0f}초 후 시작합니다. 기체를 두 손으로 잡으세요. <<<")
    for i in range(int(GRAB_DELAY), 0, -1):
        log(f"    ...{i}")
        time.sleep(1.0)

    fault_keys = ("Fault_RC", "Fault_Critical", "Fault_IMU1", "Fault_IMU2",
                  "Fault_Disagree", "Fault_Attitude")
    results = {}
    for ph in PHASES:
        log("\n" + "-" * 50)
        log(f"[동작] {ph['instr']}  (약 15° 정도만, 과하게 X)")
        # 이전 단계에서 과틸트/RC끊김으로 disarm됐을 수 있으니, 수평 상태인
        # 지금 재시동해서 fault를 해제하고 다시 arm한다. (이미 armed면 무시됨)
        for _ in range(3):
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
        mm = motor_means(samples)
        roll = mean([x.get("Roll") for x in samples])
        pitch = mean([x.get("Pitch") for x in samples])
        seen_faults = sorted({k.replace("Fault_", "") for x in samples
                              for k in fault_keys if x.get(k)})
        results[ph["key"]] = dict(phase=ph, motors=mm, roll=roll, pitch=pitch,
                                  n=len(samples), armed_frac=armed_frac,
                                  faults=seen_faults)
        log(f"    M1(FL)={mm['M1']:.0f} M2(RR)={mm['M2']:.0f} "
            f"M3(FR)={mm['M3']:.0f} M4(RL)={mm['M4']:.0f} | "
            f"Roll={roll:+.1f} Pitch={pitch:+.1f} "
            f"(n={len(samples)}, armed={armed_frac:.0%}, "
            f"faults={','.join(seen_faults) if seen_faults else '-'})")
        log("    → 레벨로 복귀 (다음 단계까지 수평 유지)")
        time.sleep(BETWEEN)

    # ---- 판정 ----
    verdict = []
    verdict.append("phase        down-side  up-side   margin  tilt(deg)  sign  result")
    all_pass = True
    for ph in PHASES:
        if ph["key"] == "level":
            continue
        r = results[ph["key"]]
        mm = r["motors"]
        down = ph["down"]
        up = [m for m in MOTOR_FIELD if m not in down]
        down_mean = mean([mm[m] for m in down])
        up_mean = mean([mm[m] for m in up])
        margin = down_mean - up_mean
        axis_val = r["roll"] if ph["axis"] == "Roll" else r["pitch"]
        tilt_ok = abs(axis_val) >= TILT_MIN_DEG
        sign_ok = margin >= MARGIN_US
        disarmed = r["armed_frac"] < 0.8
        if disarmed:
            # 측정 중 시동이 꺼져 motorOut이 전부 1000 → 부호 판정 불가(반전 아님)
            res = f"INVALID(측정중 disarm, faults={','.join(r['faults']) or '?'})"
            all_pass = False
        elif not tilt_ok:
            res = "INCONCLUSIVE(기울임부족)"
            all_pass = False
        elif sign_ok:
            res = "PASS"
        else:
            res = "FAIL(부호반전?)"
            all_pass = False
        verdict.append(
            f"{ph['key']:<12} {down_mean:7.0f}  {up_mean:7.0f}  {margin:+6.0f}   "
            f"{axis_val:+6.1f}   {'y' if tilt_ok else 'n'}    {res}   "
            f"[down={'+'.join(f'{m}({MOTOR_POS[m]})' for m in down)}]")

    # 각도 부호 일관성
    rr = results["roll_right"]["roll"]
    rl = results["roll_left"]["roll"]
    pn = results["pitch_nose"]["pitch"]
    pt = results["pitch_tail"]["pitch"]
    verdict.append("")
    verdict.append(f"Roll  부호: right={rr:+.1f}, left={rl:+.1f}  "
                   f"(반대여야 정상: {'OK' if rr * rl < 0 else '이상'})")
    verdict.append(f"Pitch 부호: nose={pn:+.1f}, tail={pt:+.1f}  "
                   f"(반대여야 정상: {'OK' if pn * pt < 0 else '이상'})")
    verdict.append("")
    verdict.append(f"종합: {'ALL PASS ✅ (roll/pitch 부호 체인 정상)' if all_pass else '검토 필요 ⚠️ (위 표)'}")

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
        log("이제 노트북이 %s 로 복원됐습니다. Claude에게 '측정 끝' 이라고 하세요." % HOME_SSID)
