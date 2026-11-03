#!/usr/bin/env python3
"""전원 인가 벤치: 고정 상태 저속 스로틀 램프 — 추력/균형/진동 확인.

⚠️ 프로펠러 ON. 기체를 단단히 고정(손X, 클리어)하고 PSU 전류제한. 부호는 이미
검증됐으므로 전복은 안 나지만, 이 테스트의 목적은 손에 들기 전에 (a) 4모터 정상
회전 (b) 레벨에서 균형 (c) 자기발진(진동) 없음 (d) 추력 증가 를 확인하는 것.

스로틀을 단계적으로 올리며 각 레벨에서 모터 평균/분산(진동)·자세·loopHz·fault를
기록하고 판정한다. 사람은 물리적으로 지켜보다 이상하면 PSU를 끈다.

사용법(노트북 ZETIN-ROBOT/온라인):
    /home/light/anaconda3/bin/python scripts/archive/bench_thrust_ramp.py [cap_us]
cap_us 생략 시 1300(≈30%). Ctrl-C 로 언제든 중단(th 1000 + stop + WiFi 복원).
"""
import datetime
import os
import signal
import socket
import statistics
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ground.telemetry_schema import parse_telemetry_packet  # noqa: E402

DRONE_SSID = "Drone_Tuning"
HOME_SSID = "ZETIN-ROBOT"
UDP_IP, UDP_PORT = "192.168.4.1", 4210

CAP = int(sys.argv[1]) if len(sys.argv) > 1 else 1300
CAP = max(1050, min(1500, CAP))            # 하드 안전 상한 1500
LEVELS = list(range(1050, CAP + 1, 50))
GRAB_DELAY = 5.0
HOLD = 2.5                                  # 레벨당 유지/측정 시간

RESULT_DIR = ("/tmp/claude-1000/-home-light-ZETIN-robotics-zetin-drone/"
              "e5f57906-8a56-4045-ab92-005d7cf74487/scratchpad")

MF = {"M1": "Motor_M1", "M2": "Motor_M2", "M3": "Motor_M3", "M4": "Motor_M4"}
POS = {"M1": "FL", "M2": "RR", "M3": "FR", "M4": "RL"}
FAULT_KEYS = ("Fault_RC", "Fault_Critical", "Fault_IMU1", "Fault_IMU2",
              "Fault_Disagree", "Fault_Attitude")

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
_log = []
_cur_throttle = 1000


def log(m=""):
    print(m, flush=True)
    _log.append(m)


def nmcli_up(ssid):
    try:
        subprocess.run(["nmcli", "connection", "up", ssid],
                       capture_output=True, text=True, timeout=30)
    except Exception as e:  # noqa: BLE001
        log(f"[WIFI-ERR] {ssid}: {e}")


def wait_reachable(t=25.0):
    end = time.time() + t
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
    for _ in range(3):
        send("start")
        time.sleep(0.05)
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


def set_throttle(v):
    global _cur_throttle
    _cur_throttle = v
    send(f"th {v}")


def collect(seconds):
    global _collecting
    with _lock:
        _buffer.clear()
        _collecting = True
    time.sleep(seconds)
    with _lock:
        _collecting = False
        return list(_buffer)


def mean(v):
    v = [x for x in v if x is not None]
    return sum(v) / len(v) if v else float("nan")


def std(v):
    v = [x for x in v if x is not None]
    return statistics.pstdev(v) if len(v) > 1 else 0.0


def cleanup():
    global _running
    for v in (1000,):
        send(f"th {v}")
    _running = False
    for _ in range(6):
        send("stop")
        time.sleep(0.02)
    try:
        sock.close()
    except OSError:
        pass
    log(f"\n[정리] th 1000 + stop 전송. WiFi {HOME_SSID} 복원...")
    nmcli_up(HOME_SSID)
    log("[정리] 복원 완료.")


def save(verdict):
    os.makedirs(RESULT_DIR, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    p = os.path.join(RESULT_DIR, f"thrust_ramp_{ts}.txt")
    with open(p, "w") as f:
        f.write("\n".join(_log) + "\n\n===== VERDICT =====\n" + "\n".join(verdict) + "\n")
    return p


def sigint(*_):
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGINT, sigint)
    log("=" * 60)
    log(" 벤치: 고정 저속 스로틀 램프 (추력/균형/진동)")
    log(f" 레벨 {LEVELS[0]}..{LEVELS[-1]}us, 레벨당 {HOLD:.1f}s")
    log(" ⚠️ 프롭 ON — 기체 고정, 클리어, PSU 전류제한 확인")
    log("=" * 60)

    log(f"[WIFI] {DRONE_SSID} 전환...")
    nmcli_up(DRONE_SSID)
    if not wait_reachable():
        log("[에러] ESP 도달 불가.")
        cleanup()
        save(["ABORTED: ESP 도달 불가"])
        return

    # 프롭 도는 테스트이므로 명시적 확인을 요구한다.
    try:
        input("\n>>> 기체 고정·클리어·PSU 전류제한 확인됐으면 Enter (취소=Ctrl-C) <<< ")
    except EOFError:
        pass

    threading.Thread(target=sender, daemon=True).start()
    threading.Thread(target=receiver, daemon=True).start()

    log("[대기] 시동 확인...")
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
    if not s or not armed:
        log(f"[에러] 시동 실패/텔레메트리 없음. calib={s.get('Calibration_OK') if s else '?'} "
            f"faults={[k for k in FAULT_KEYS if s and s.get(k)]}")
        cleanup()
        save(["ABORTED: 시동 실패"])
        return
    log(f"[OK] 시동. calib={s.get('Calibration_OK')} loopHz={s.get('PID_Loop_Hz')}")

    log(f"\n>>> {GRAB_DELAY:.0f}초 후 램프 시작. (기체 고정 확인) <<<")
    for i in range(int(GRAB_DELAY), 0, -1):
        log(f"    ...{i}")
        time.sleep(1.0)

    rows = []
    aborted = False
    for lvl in LEVELS:
        set_throttle(lvl)
        log(f"\n[throttle {lvl}]")
        time.sleep(0.4)                     # 스텝 안정화
        samples = collect(HOLD)
        if not samples:
            log("    (샘플 없음)")
            continue
        armed_frac = mean([1.0 if x.get("Armed") else 0.0 for x in samples])
        mm = {m: mean([x.get(MF[m]) for x in samples]) for m in MF}
        msd = {m: std([x.get(MF[m]) for x in samples]) for m in MF}
        ax = mean([x.get("Roll") for x in samples])
        ay = mean([x.get("Pitch") for x in samples])
        hz = mean([x.get("PID_Loop_Hz") for x in samples])
        scaled = mean([1.0 if x.get("Mixer_Scaled") else 0.0 for x in samples])
        faults = sorted({k.replace("Fault_", "") for x in samples for k in FAULT_KEYS if x.get(k)})
        spread = max(mm.values()) - min(mm.values())
        osc = max(msd.values())
        rows.append(dict(lvl=lvl, mm=mm, spread=spread, osc=osc, ax=ax, ay=ay,
                         hz=hz, scaled=scaled, faults=faults, armed=armed_frac))
        log(f"    M1={mm['M1']:.0f} M2={mm['M2']:.0f} M3={mm['M3']:.0f} M4={mm['M4']:.0f} "
            f"| spread={spread:.0f} osc(max σ)={osc:.1f} R={ax:+.1f} P={ay:+.1f} "
            f"Hz={hz:.0f} scaled={scaled:.0%} armed={armed_frac:.0%} "
            f"flt={','.join(faults) if faults else '-'}")
        if faults or armed_frac < 0.8:
            log("    [중단] fault/disarm 감지 — 램프 중지")
            aborted = True
            break

    # 하강
    for lvl in (1200, 1100, 1000):
        set_throttle(lvl)
        time.sleep(0.3)

    # ---- 판정 ----
    v = []
    v.append("throttle  M1   M2   M3   M4   spread  osc(σ)  R      P     Hz   flt")
    for r in rows:
        v.append(f"{r['lvl']:<8} {r['mm']['M1']:.0f} {r['mm']['M2']:.0f} {r['mm']['M3']:.0f} "
                 f"{r['mm']['M4']:.0f}  {r['spread']:5.0f}  {r['osc']:5.1f}  {r['ax']:+5.1f} "
                 f"{r['ay']:+5.1f} {r['hz']:.0f}  {','.join(r['faults']) if r['faults'] else '-'}")
    v.append("")
    if rows:
        max_spread = max(r["spread"] for r in rows)
        max_osc = max(r["osc"] for r in rows)
        any_fault = any(r["faults"] for r in rows) or aborted
        hz_ok = all(r["hz"] >= 950 for r in rows)
        v.append(f"최대 spread(레벨 불균형)={max_spread:.0f}us, 최대 osc(σ, 진동)={max_osc:.1f}us, "
                 f"loopHz>=950 {'OK' if hz_ok else '이상'}, fault {'있음' if any_fault else '없음'}")
        v.append("해석: spread 큼=CG/트림 치우침(적분기가 잡아야), osc 큼=자기발진(게인 과다), "
                 "fault=안전. 부호는 이미 검증됨.")
        if not any_fault and max_osc < 15 and hz_ok:
            v.append("종합: 정상 범위 ✅ (진동/fault 없음). 다음: 손파지 저속 틸트-복원 → 테더 호버.")
        else:
            v.append("종합: 검토 필요 ⚠️ (osc 크면 게인 낮추기, fault면 원인 규명 — 손파지 전에 해결).")
    else:
        v.append("종합: 데이터 없음.")

    log("\n" + "=" * 60)
    for line in v:
        log(line)
    log("=" * 60)
    return v


if __name__ == "__main__":
    verdict = ["(aborted early)"]
    try:
        r = main()
        if r:
            verdict = r
    except KeyboardInterrupt:
        log("\n[중단] Ctrl-C")
    finally:
        cleanup()
        p = save(verdict)
        log(f"\n[결과 저장] {p}")
        log(f"이제 {HOME_SSID} 복원됨. Claude에게 '측정 끝'.")
