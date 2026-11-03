#!/usr/bin/env python3
"""
receive_dual_imu_debug.py

Pairs with
firmware/diagnostics/icm42670_dual_loop_debug/icm42670_dual_loop_debug.ino.

Usage:
  1. Connect laptop WiFi to "DUAL_IMU_DEBUG" (password 12345678).
  2. python scripts/archive/receive_dual_imu_debug.py
  3. Tilt the drone by hand and watch the per-packet line + 2-second summary.

Each packet (50 ms cadence) shows angles, raw gyro/accel, and four
diagnostic fields:
  loops/50ms  - how many 1 kHz loop iterations actually ran in the
                last 50 ms window (target ~50; <50 means the loop is
                slower than 1 kHz)
  max_dt_us   - largest single-loop dt in the window
  clamps      - number of loops where real dt > 2000 us (dt clamp fired)
  alpha       - last complementary-filter alpha used (0.99/0.995/0.999)
"""

import math
import socket
import sys
import time
from collections import deque

DRONE_IP   = "192.168.4.1"
DRONE_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 0))
sock.settimeout(1.0)

FIELDS = [
    "angleX", "angleY", "angleZ",
    "raw_gx", "raw_gy", "raw_gz",
    "raw_ax", "raw_ay", "raw_az",
    "loops_50ms", "max_dt_us", "avg_dt_us", "clamps_50ms", "alpha",
]

WINDOW = 40  # 40 packets * 50ms = ~2 sec rolling window
loop_hz_hist = deque(maxlen=WINDOW)
max_dt_hist  = deque(maxlen=WINDOW)
clamp_hist   = deque(maxlen=WINDOW)
alpha_hist   = deque(maxlen=WINDOW)

print(f"Sending registration to {DRONE_IP}:{DRONE_PORT}...")
sock.sendto(b"hello", (DRONE_IP, DRONE_PORT))

last_register = time.time()
last_summary  = time.time()
silent_since  = None
pkt_seen      = 0

try:
    while True:
        now = time.time()
        if now - last_register > 2.0:
            sock.sendto(b"hello", (DRONE_IP, DRONE_PORT))
            last_register = now

        try:
            data, _ = sock.recvfrom(512)
        except socket.timeout:
            if silent_since is None:
                silent_since = now
            elif now - silent_since > 3.0:
                print("(no telemetry for 3s - check WiFi association)")
                silent_since = now
            continue
        silent_since = None

        try:
            parts = data.decode().strip().split(",")
            if len(parts) != len(FIELDS):
                print(f"bad packet ({len(parts)} fields, want {len(FIELDS)}): {data!r}")
                continue
            d = dict(zip(FIELDS, parts))
            angleX = float(d["angleX"]); angleY = float(d["angleY"]); angleZ = float(d["angleZ"])
            gx = float(d["raw_gx"]); gy = float(d["raw_gy"]); gz = float(d["raw_gz"])
            ax = float(d["raw_ax"]); ay = float(d["raw_ay"]); az = float(d["raw_az"])
            loops     = int(d["loops_50ms"])
            max_dt_us = int(d["max_dt_us"])
            avg_dt_us = float(d["avg_dt_us"])
            clamps    = int(d["clamps_50ms"])
            alpha     = float(d["alpha"])
        except (ValueError, KeyError) as ex:
            print(f"parse error: {ex} pkt={data!r}")
            continue

        loop_hz = 1e6 / avg_dt_us if avg_dt_us > 0 else 0.0
        loop_hz_hist.append(loop_hz)
        max_dt_hist.append(max_dt_us)
        clamp_hist.append(clamps)
        alpha_hist.append(alpha)
        pkt_seen += 1

        # What the angle WOULD be if comp filter trusted accel alone (alpha=0).
        # Same formula the firmware uses for accAngleX/Y.
        hor_x = math.sqrt(ax*ax + az*az)
        hor_y = math.sqrt(ay*ay + az*az)
        accX = math.degrees(math.atan2( ay, hor_x)) if hor_x > 0.01 else 0.0
        accY = math.degrees(math.atan2(-ax, hor_y)) if hor_y > 0.01 else 0.0
        acc_mag = math.sqrt(ax*ax + ay*ay + az*az)

        # NOTE: accCalc X/Y here uses the OLD sensor-frame formula and won't
        # match firmware's angleX/Y after axis remap. Use it as a sanity check
        # on the raw accel only, not as a 1:1 comparison with angleX/Y.
        print(
            f"angle X={angleX:+6.1f} Y={angleY:+6.1f} Z={angleZ:+6.1f} | "
            f"gyro {gx:+6.1f} {gy:+6.1f} {gz:+6.1f} | "
            f"acc {ax:+5.2f} {ay:+5.2f} {az:+5.2f} (|a|={acc_mag:.2f}G) | "
            f"loop {loop_hz:5.0f}Hz a={alpha:.4f}"
        )

        if now - last_summary > 2.0 and loop_hz_hist:
            n = len(loop_hz_hist)
            avg_hz = sum(loop_hz_hist) / n
            min_hz = min(loop_hz_hist)
            worst_dt = max(max_dt_hist)
            total_clamps = sum(clamp_hist)
            a_static  = sum(1 for a in alpha_hist if a < 0.992)
            a_normal  = sum(1 for a in alpha_hist if 0.992 <= a < 0.998)
            a_dynamic = sum(1 for a in alpha_hist if a >= 0.998)
            print(
                f"\n[SUMMARY last {n*0.05:.1f}s] "
                f"loop {avg_hz:6.1f}Hz avg, {min_hz:6.1f}Hz worst | "
                f"max dt {worst_dt}us | total clamps {total_clamps} | "
                f"alpha hits: static={a_static} normal={a_normal} dynamic={a_dynamic}\n"
            )
            last_summary = now
except KeyboardInterrupt:
    print(f"\nstopped after {pkt_seen} packets.")
    sys.exit(0)
