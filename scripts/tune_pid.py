import socket
import threading
import time

from ground.telemetry_schema import (
    active_fault_names,
    is_gains_packet,
    parse_gains_packet,
    parse_telemetry_packet,
)

# ESP32 설정
UDP_IP = "192.168.4.1" # ESP32 SoftAP 기본 IP
UDP_PORT = 4210

STATUS_PERIOD = 1.0  # 텔레메트리 상태 출력 최소 간격 (초)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)


def receive_thread():
    # 명령을 보내는 순간 드론이 이 소켓 포트를 텔레메트리 목적지로
    # 등록하므로, 여기서 읽어주지 않으면 텔레메트리가 버려진다.
    # 20Hz 원본을 그대로 찍으면 콘솔을 못 쓰니 1초에 한 줄만 요약한다.
    last_status = 0.0
    while True:
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            break

        try:
            line = data.decode("utf-8", errors="strict")
            if is_gains_packet(line):
                gains = parse_gains_packet(line)
                print(
                    "\n[GAINS] "
                    + " ".join(
                        f"{name}={value:.4f}" for name, value in gains.items()
                    )
                )
                continue
        except (UnicodeDecodeError, ValueError):
            continue

        now = time.monotonic()
        if now - last_status < STATUS_PERIOD:
            continue
        try:
            sample = parse_telemetry_packet(line)
        except ValueError:
            continue
        faults = ",".join(active_fault_names(sample)) or "-"
        print(
            f"\n[TELEM] R:{sample['Roll']:6.2f} P:{sample['Pitch']:6.2f} "
            f"Y:{sample['Yaw']:6.2f} Thr:{sample['Throttle']} Fault:{faults}"
        )
        last_status = now


# 수신 스레드 시작
t = threading.Thread(target=receive_thread, daemon=True)
t.start()

print("========== DRONE TUNING CONSOLE ==========")
print(" Inner rate PID:")
print("  pa|ia|da <val> : Roll+Pitch P/I/D 공통 (ex: pa 0.5)")
print("  pr|ir|dr <val> : Roll P/I/D")
print("  pp|ip|dp <val> : Pitch P/I/D")
print("  py|iy|dy <val> : Yaw P/I/D")
print(" Outer angle P:")
print("  ap <val>       : Roll+Pitch 공통  |  ar/at/ay <val> : Roll/Pitch/Yaw")
print(" Control:")
print("  th <val>  : Base Throttle (ex: th 1150)")
print("  yaw <0|1> : Yaw 제어 on/off")
print("  gains     : 현재 12개 PID 게인 읽기")
print("  start     : ARM & Start Motors")
print("  stop      : DISARM (Emergency)")
print("==========================================")

while True:
    msg = input("Command > ")
    if msg:
        sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
