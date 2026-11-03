"""Live telemetry plot and CSV logger for ``drone_control.py --listen --plot``.

Receive-only: sends the ``connect`` keepalive and nothing else, so it must not
run next to another UDP tool (the firmware sends telemetry to the last sender).
Importing this module has no side effects; ``main`` binds the socket, opens
the CSV and builds the window.
"""

import argparse
import csv
import datetime
import math
import socket
import time
from collections import deque

from ground.failsafe_telemetry import render_monitor_title
from ground.paths import LOG_DIR
from ground.telemetry_schema import (
    CSV_FIELDS,
    active_fault_names,
    is_gains_packet,
    parse_telemetry_packet,
    sample_to_csv_row,
)


UDP_PORT = 4210
DRONE_IP = "192.168.4.1"
ANIMATION_INTERVAL_MS = 100   # 화면 갱신 최대 10 Hz
MAX_LEN = 200                 # 20 Hz telemetry 기준 약 10초

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


def open_new_log(log_dir, stamp):
    """Create flight_log_<stamp>.csv exclusively; flight_log_<stamp>_<n> on collision."""
    n = 1
    while True:
        name = f"flight_log_{stamp}" if n == 1 else f"flight_log_{stamp}_{n}"
        path = log_dir / f"{name}.csv"
        try:
            return path, path.open("x", newline="", encoding="utf-8")
        except FileExistsError:
            n += 1


def build_figure():
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 14))
    fig.suptitle("Real-time Drone Telemetry")
    return fig, axes


def plt_close(fig):
    import matplotlib.pyplot as plt

    plt.close(fig)


def show_figure(fig, monitor):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    monitor.animation = FuncAnimation(
        fig, monitor.update, interval=ANIMATION_INTERVAL_MS, cache_frame_data=False)
    plt.show()


class Monitor:
    """Telemetry state plus the per-frame update of the four panels."""

    def __init__(self, sock, csv_file):
        self.sock = sock
        self.csv_file = csv_file
        self.csv_writer = csv.writer(csv_file)
        self.csv_writer.writerow(CSV_FIELDS)
        self.roll_data = deque(maxlen=MAX_LEN)
        self.pitch_data = deque(maxlen=MAX_LEN)
        self.yaw_data = deque(maxlen=MAX_LEN)
        self.gyro_x_data = deque(maxlen=MAX_LEN)
        self.gyro_y_data = deque(maxlen=MAX_LEN)
        self.gyro_z_data = deque(maxlen=MAX_LEN)
        self.accel_x_data = deque(maxlen=MAX_LEN)
        self.accel_y_data = deque(maxlen=MAX_LEN)
        self.accel_z_data = deque(maxlen=MAX_LEN)
        self.throttle_data = deque(maxlen=MAX_LEN)
        self.active_imus_data = deque(maxlen=MAX_LEN)
        self.scaled_data = deque(maxlen=MAX_LEN)
        self.critical_fault_data = deque(maxlen=MAX_LEN)
        self.any_fault_data = deque(maxlen=MAX_LEN)
        self.calibration_ok_data = deque(maxlen=MAX_LEN)
        self.failsafe_phase_data = deque(maxlen=MAX_LEN)
        self.fig, self.axes = build_figure()
        self.ax3_right = self.axes[2].twinx()
        self.last_handshake = 0.0
        self.packet_count = 0
        self.bad_packet_count = 0
        self.latest_sample = None
        self.animation = None

    def update(self, _frame):
        ax1, ax2, ax3, ax4 = self.axes
        ax3_right = self.ax3_right
        now_monotonic = time.monotonic()
        if now_monotonic - self.last_handshake >= 1.0:
            try:
                self.sock.sendto(b"connect", (DRONE_IP, UDP_PORT))
                self.last_handshake = now_monotonic
            except OSError:
                pass

        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
                line = data.decode("utf-8", errors="strict")
                if is_gains_packet(line):
                    continue
                sample = parse_telemetry_packet(line)
                self.latest_sample = sample

                now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                self.csv_writer.writerow(sample_to_csv_row(now, sample))
                self.packet_count += 1
                if self.packet_count % 20 == 0:
                    self.csv_file.flush()

                self.roll_data.append(sample["Roll"])
                self.pitch_data.append(sample["Pitch"])
                self.yaw_data.append(sample["Yaw"])
                self.gyro_x_data.append(sample["Gyro_X"])
                self.gyro_y_data.append(sample["Gyro_Y"])
                self.gyro_z_data.append(sample["Gyro_Z"])
                self.accel_x_data.append(sample["Accel_X"])
                self.accel_y_data.append(sample["Accel_Y"])
                self.accel_z_data.append(sample["Accel_Z"])
                self.throttle_data.append(sample["Throttle"])
                self.active_imus_data.append(optional_number(sample["Active_IMUs"]))
                self.scaled_data.append(optional_number(sample["Mixer_Scaled"]))
                self.critical_fault_data.append(optional_number(sample["Fault_Critical"]))
                self.calibration_ok_data.append(optional_number(sample["Calibration_OK"]))
                self.failsafe_phase_data.append(optional_number(sample["Failsafe_Phase"]))

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
                self.any_fault_data.append(
                    math.nan if not known_fault_fields else int(any(known_fault_fields))
                )

            except socket.timeout:
                break
            except (UnicodeDecodeError, ValueError) as exc:
                self.bad_packet_count += 1
                print(f"⚠️ 잘못된 텔레메트리 #{self.bad_packet_count}: {exc}")
            except OSError as exc:
                print(f"⚠️ 소켓 오류: {exc}")
                break

        x = range(len(self.roll_data))

        ax1.cla()
        ax1.plot(x, self.roll_data, label="Roll", color="red")
        ax1.plot(x, self.pitch_data, label="Pitch", color="blue")
        ax1.plot(x, self.yaw_data, label="Yaw", color="green", linestyle="--")
        throttle = self.throttle_data[-1] if self.throttle_data else 0
        ax1.set_title(f"Attitude (Throttle: {throttle})")
        ax1.set_ylabel("Angle (deg)")
        ax1.set_ylim(-60, 60)
        ax1.legend(loc="upper right", fontsize="small")
        ax1.grid(True)

        ax2.cla()
        ax2.plot(x, self.gyro_x_data, label="Gyro X", color="red", alpha=0.7)
        ax2.plot(x, self.gyro_y_data, label="Gyro Y", color="blue", alpha=0.7)
        ax2.plot(x, self.gyro_z_data, label="Gyro Z", color="green", alpha=0.7)
        ax2.set_title("Gyroscope (Vibration Check)")
        ax2.set_ylabel("Angular rate (dps)")
        ax2.legend(loc="upper right", fontsize="small")
        ax2.grid(True)

        ax3.cla()
        ax3_right.cla()
        ax3.plot(x, self.accel_x_data, label="Accel X", color="red", alpha=0.65)
        ax3.plot(x, self.accel_y_data, label="Accel Y", color="blue", alpha=0.65)
        ax3.plot(x, self.accel_z_data, label="Accel Z", color="green", alpha=0.65)
        ax3_right.plot(x, self.throttle_data, label="Throttle", color="orange", linewidth=1.5)
        ax3.set_title("Accelerometer & Throttle")
        ax3.set_ylabel("Acceleration (g)")
        ax3.set_ylim(-2.0, 2.0)
        ax3_right.set_ylabel("Throttle (PWM)", color="orange")
        ax3_right.set_ylim(1000, 2000)
        ax3.legend(loc="upper left", fontsize="small")
        ax3_right.legend(loc="upper right", fontsize="small")
        ax3.grid(True)

        ax4.cla()
        ax4.step(x, self.active_imus_data, where="post", label="Active IMUs", linewidth=2)
        ax4.step(x, self.scaled_data, where="post", label="Mixer scaled", alpha=0.8)
        ax4.step(x, self.critical_fault_data, where="post", label="Critical fault", alpha=0.8)
        ax4.step(x, self.any_fault_data, where="post", label="Any fault", alpha=0.8)
        ax4.step(x, self.calibration_ok_data, where="post", label="Calibration OK", alpha=0.8)
        ax4.step(
            x,
            self.failsafe_phase_data,
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

        if self.latest_sample is None:
            ax4.set_title("System status (waiting for telemetry)")
        else:
            active = self.latest_sample["Active_IMUs"]
            active_text = "legacy/unknown" if active is None else str(active)
            faults = active_fault_names(self.latest_sample)
            fault_text = ", ".join(faults) if faults else "none"
            yaw_hold = self.latest_sample["Yaw_Hold"]
            yaw_hold_text = "-" if yaw_hold is None else str(yaw_hold)
            status_sample = {
                name: self.latest_sample.get(name) for name in MONITOR_STATUS_FIELDS
            }
            render_monitor_title(
                ax4,
                (
                    f"System status — active IMUs: {active_text}, "
                    f"faults: {fault_text}, Yaw_Hold={yaw_hold_text}"
                ),
                status_sample,
            )

        self.fig.tight_layout(rect=(0, 0, 1, 0.97))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="drone_control.py --listen --plot",
        description="Live telemetry plot and CSV logger (sends connect only).",
    )
    # drone_control.py가 모드 플래그를 그대로 넘긴다.
    parser.add_argument("--listen", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--plot", action="store_true", help=argparse.SUPPRESS)
    parser.parse_args(argv)

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    csv_file = None
    monitor = None
    try:
        # 포트를 먼저 잡는다: 이미 떠 있는 수신기가 있으면 로그를 건드리지 않는다.
        sock.bind(("0.0.0.0", UDP_PORT))
        sock.settimeout(0.002)
        stamp = f"{datetime.datetime.now():%Y-%m-%d_%H%M%S}"
        file_path, csv_file = open_new_log(LOG_DIR, stamp)
        monitor = Monitor(sock, csv_file)
        print(f"📡 확장 모니터링 시작! (Port: {UDP_PORT})")
        print(f"💾 로그 파일: {file_path}")
        show_figure(monitor.fig, monitor)
    finally:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
        sock.close()
        if monitor is not None:
            plt_close(monitor.fig)
            print(f"✅ 저장 {monitor.packet_count}개, "
                  f"잘못된 패킷 {monitor.bad_packet_count}개")
    return 0
