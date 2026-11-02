import csv
import datetime
import socket
import sys
import time
from pathlib import Path

from ground.telemetry_schema import (
    CSV_FIELDS,
    active_fault_names,
    is_gains_packet,
    parse_telemetry_packet,
    sample_to_csv_row,
)


UDP_PORT = 4210
DRONE_IP = "192.168.4.1"
SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR = SCRIPT_DIR.parent / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

filename = f"flight_log_{datetime.datetime.now():%Y-%m-%d_%H%M%S}.csv"
file_path = LOG_DIR / filename

try:
    csv_file = file_path.open("w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(CSV_FIELDS)
    print(f"💾 로그 파일 생성됨: {file_path}")
except OSError as exc:
    print(f"❌ 파일 생성 실패: {exc}")
    sys.exit(1)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.settimeout(0.05)

print(f"📡 데이터 수신 대기 중... (Port: {UDP_PORT})")
print("🛑 종료하려면 Ctrl+C를 누르세요.")

last_handshake = 0.0
packet_count = 0
bad_packet_count = 0

try:
    while True:
        now_monotonic = time.monotonic()
        if now_monotonic - last_handshake >= 1.0:
            try:
                sock.sendto(b"connect", (DRONE_IP, UDP_PORT))
                last_handshake = now_monotonic
            except OSError:
                pass

        try:
            data, _ = sock.recvfrom(2048)
            line = data.decode("utf-8", errors="strict").strip()
            if is_gains_packet(line):
                continue
            sample = parse_telemetry_packet(line)

            now_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            csv_writer.writerow(sample_to_csv_row(now_str, sample))
            packet_count += 1
            if packet_count % 20 == 0:
                csv_file.flush()

            faults = active_fault_names(sample)
            active_imus = sample["Active_IMUs"]
            scaled = sample["Mixer_Scaled"]
            armed = sample["Armed"]
            active_text = "?" if active_imus is None else str(active_imus)
            scaled_text = "?" if scaled is None else str(scaled)
            armed_text = "?" if armed is None else str(armed)
            fault_text = ",".join(faults) if faults else "-"

            print(
                f"[{now_str}] "
                f"R:{sample['Roll']:6.2f} P:{sample['Pitch']:6.2f} Y:{sample['Yaw']:6.2f} | "
                f"GX:{sample['Gyro_X']:7.2f} GY:{sample['Gyro_Y']:7.2f} "
                f"GZ:{sample['Gyro_Z']:7.2f} | "
                f"Thr:{sample['Throttle']:4d} Arm:{armed_text} IMU:{active_text} "
                f"Scaled:{scaled_text} Fault:{fault_text}"
            )

        except socket.timeout:
            continue
        except (UnicodeDecodeError, ValueError) as exc:
            bad_packet_count += 1
            print(f"⚠️ 잘못된 텔레메트리 #{bad_packet_count}: {exc}")
        except OSError as exc:
            print(f"⚠️ 소켓 오류: {exc}")

except KeyboardInterrupt:
    print("\n🛑 로그 저장 종료!")
    print(f"📊 저장 {packet_count}개, 잘못된 패킷 {bad_packet_count}개")
finally:
    csv_file.flush()
    csv_file.close()
    sock.close()
    print("✅ 파일이 안전하게 닫혔습니다.")
