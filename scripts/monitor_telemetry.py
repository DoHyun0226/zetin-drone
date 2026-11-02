import csv
import datetime
import math
import socket
import time
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from ground.failsafe_telemetry import render_monitor_title
from ground.telemetry_schema import (
    CSV_FIELDS,
    active_fault_names,
    is_gains_packet,
    parse_telemetry_packet,
    sample_to_csv_row,
)


UDP_PORT = 4210
DRONE_IP = "192.168.4.1"
MAX_LEN = 200                 # 20 Hz telemetry 기준 약 10초
SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR = SCRIPT_DIR.parent / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

roll_data = deque(maxlen=MAX_LEN)
pitch_data = deque(maxlen=MAX_LEN)
yaw_data = deque(maxlen=MAX_LEN)
gyro_x_data = deque(maxlen=MAX_LEN)
gyro_y_data = deque(maxlen=MAX_LEN)
gyro_z_data = deque(maxlen=MAX_LEN)
accel_x_data = deque(maxlen=MAX_LEN)
accel_y_data = deque(maxlen=MAX_LEN)
accel_z_data = deque(maxlen=MAX_LEN)
throttle_data = deque(maxlen=MAX_LEN)
active_imus_data = deque(maxlen=MAX_LEN)
scaled_data = deque(maxlen=MAX_LEN)
critical_fault_data = deque(maxlen=MAX_LEN)
any_fault_data = deque(maxlen=MAX_LEN)
calibration_ok_data = deque(maxlen=MAX_LEN)
failsafe_phase_data = deque(maxlen=MAX_LEN)

filename = f"flight_log_{datetime.datetime.now():%Y-%m-%d_%H%M%S}.csv"
file_path = LOG_DIR / filename
csv_file = file_path.open("w", newline="", encoding="utf-8")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(CSV_FIELDS)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.settimeout(0.002)

print(f"📡 확장 모니터링 시작! (Port: {UDP_PORT})")
print(f"💾 로그 파일: {file_path}")

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True, figsize=(11, 14))
ax3_right = ax3.twinx()
fig.suptitle("Real-time Drone Telemetry")

last_handshake = 0.0
packet_count = 0
bad_packet_count = 0
latest_sample = None

MONITOR_STATUS_FIELDS = (
    "Failsafe_Phase",
    "Trim_Roll",
    "Trim_Pitch",
    "Hover_Est",
    "Hover_Valid",
    "Failsafe_Probe_State",
    "Failsafe_Probe_NoResponse",
    "Failsafe_Probe_Response_G",
)


def optional_number(value):
    return math.nan if value is None else value


def update_plot(_frame):
    global last_handshake, packet_count, bad_packet_count, latest_sample

    now_monotonic = time.monotonic()
    if now_monotonic - last_handshake >= 1.0:
        try:
            sock.sendto(b"connect", (DRONE_IP, UDP_PORT))
            last_handshake = now_monotonic
        except OSError:
            pass

    while True:
        try:
            data, _ = sock.recvfrom(2048)
            line = data.decode("utf-8", errors="strict")
            if is_gains_packet(line):
                continue
            sample = parse_telemetry_packet(line)
            latest_sample = sample

            now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            csv_writer.writerow(sample_to_csv_row(now, sample))
            packet_count += 1
            if packet_count % 20 == 0:
                csv_file.flush()

            roll_data.append(sample["Roll"])
            pitch_data.append(sample["Pitch"])
            yaw_data.append(sample["Yaw"])
            gyro_x_data.append(sample["Gyro_X"])
            gyro_y_data.append(sample["Gyro_Y"])
            gyro_z_data.append(sample["Gyro_Z"])
            accel_x_data.append(sample["Accel_X"])
            accel_y_data.append(sample["Accel_Y"])
            accel_z_data.append(sample["Accel_Z"])
            throttle_data.append(sample["Throttle"])
            active_imus_data.append(optional_number(sample["Active_IMUs"]))
            scaled_data.append(optional_number(sample["Mixer_Scaled"]))
            critical_fault_data.append(optional_number(sample["Fault_Critical"]))
            calibration_ok_data.append(optional_number(sample["Calibration_OK"]))
            failsafe_phase_data.append(optional_number(sample["Failsafe_Phase"]))

            known_fault_fields = [
                sample[name]
                for name in (
                    "Fault_RC",
                    "Fault_Critical",
                    "Fault_IMU1",
                    "Fault_IMU2",
                    "Fault_Disagree",
                    "Fault_Attitude",
                )
                if sample[name] is not None
            ]
            any_fault_data.append(
                math.nan if not known_fault_fields else int(any(known_fault_fields))
            )

        except socket.timeout:
            break
        except (UnicodeDecodeError, ValueError) as exc:
            bad_packet_count += 1
            print(f"⚠️ 잘못된 텔레메트리 #{bad_packet_count}: {exc}")
        except OSError as exc:
            print(f"⚠️ 소켓 오류: {exc}")
            break

    x = range(len(roll_data))

    ax1.cla()
    ax1.plot(x, roll_data, label="Roll", color="red")
    ax1.plot(x, pitch_data, label="Pitch", color="blue")
    ax1.plot(x, yaw_data, label="Yaw", color="green", linestyle="--")
    throttle = throttle_data[-1] if throttle_data else 0
    ax1.set_title(f"Attitude (Throttle: {throttle})")
    ax1.set_ylabel("Angle (deg)")
    ax1.set_ylim(-60, 60)
    ax1.legend(loc="upper right", fontsize="small")
    ax1.grid(True)

    ax2.cla()
    ax2.plot(x, gyro_x_data, label="Gyro X", color="red", alpha=0.7)
    ax2.plot(x, gyro_y_data, label="Gyro Y", color="blue", alpha=0.7)
    ax2.plot(x, gyro_z_data, label="Gyro Z", color="green", alpha=0.7)
    ax2.set_title("Gyroscope (Vibration Check)")
    ax2.set_ylabel("Angular rate (dps)")
    ax2.legend(loc="upper right", fontsize="small")
    ax2.grid(True)

    ax3.cla()
    ax3_right.cla()
    ax3.plot(x, accel_x_data, label="Accel X", color="red", alpha=0.65)
    ax3.plot(x, accel_y_data, label="Accel Y", color="blue", alpha=0.65)
    ax3.plot(x, accel_z_data, label="Accel Z", color="green", alpha=0.65)
    ax3_right.plot(x, throttle_data, label="Throttle", color="orange", linewidth=1.5)
    ax3.set_title("Accelerometer & Throttle")
    ax3.set_ylabel("Acceleration (g)")
    ax3.set_ylim(-2.0, 2.0)
    ax3_right.set_ylabel("Throttle (PWM)", color="orange")
    ax3_right.set_ylim(1000, 2000)
    ax3.legend(loc="upper left", fontsize="small")
    ax3_right.legend(loc="upper right", fontsize="small")
    ax3.grid(True)

    ax4.cla()
    ax4.step(x, active_imus_data, where="post", label="Active IMUs", linewidth=2)
    ax4.step(x, scaled_data, where="post", label="Mixer scaled", alpha=0.8)
    ax4.step(x, critical_fault_data, where="post", label="Critical fault", alpha=0.8)
    ax4.step(x, any_fault_data, where="post", label="Any fault", alpha=0.8)
    ax4.step(x, calibration_ok_data, where="post", label="Calibration OK", alpha=0.8)
    ax4.step(
        x,
        failsafe_phase_data,
        where="post",
        label="Failsafe phase",
        color="crimson",
        linewidth=2,
    )
    ax4.set_ylabel("State")
    ax4.set_ylim(-0.1, 4.2)
    ax4.set_yticks([0, 1, 2, 3, 4])
    ax4.set_xlabel("Recent sample")
    ax4.grid(True)
    ax4.legend(loc="upper right", fontsize="small", ncol=3)

    if latest_sample is None:
        ax4.set_title("System status (waiting for telemetry)")
    else:
        active = latest_sample["Active_IMUs"]
        active_text = "legacy/unknown" if active is None else str(active)
        faults = active_fault_names(latest_sample)
        fault_text = ", ".join(faults) if faults else "none"
        yaw_hold = latest_sample["Yaw_Hold"]
        yaw_hold_text = "-" if yaw_hold is None else str(yaw_hold)
        status_sample = {
            name: latest_sample.get(name) for name in MONITOR_STATUS_FIELDS
        }
        render_monitor_title(
            ax4,
            (
                f"System status — active IMUs: {active_text}, "
                f"faults: {fault_text}, Yaw_Hold={yaw_hold_text}"
            ),
            status_sample,
        )

    fig.tight_layout(rect=(0, 0, 1, 0.97))


animation = FuncAnimation(fig, update_plot, interval=50, cache_frame_data=False)

try:
    plt.show()
finally:
    csv_file.flush()
    csv_file.close()
    sock.close()
    print(f"✅ 저장 {packet_count}개, 잘못된 패킷 {bad_packet_count}개")
