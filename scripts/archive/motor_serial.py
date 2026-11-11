#!/usr/bin/env python3
"""motor_id_single 대화형 시리얼 브리지 (ESP32-S3, DTR 다운로드모드 회피).

/dev/ttyACM0 @115200 을 DTR/RTS 비활성으로 열어(GPIO0 strap 다운로드 모드 진입
방지) ESP 출력을 실시간 표시하고, 입력한 키를 그대로 보드로 보낸다.

⚠️ 프로펠러 제거(props OFF) + PSU 전류제한 상태에서만 사용 (Stage A 회전방향 확인).

키 (Enter로 전송):
  1/2/3/4 : 해당 모터만 저속(1120µs) 회전 — 회전방향 관찰
  0 또는 s : 전 모터 정지
  +/-     : setpoint ±10µs (상한 1250)
  r       : setpoint 1120으로 리셋
  R       : (로컬) RTS 리셋 — 보드가 응답 없으면 런모드로 재부팅
  Ctrl-C  : 종료 (종료 전 정지 신호 전송)
"""
import sys
import threading
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial 필요: /home/light/anaconda3/bin/pip install pyserial")

PORT = "/dev/ttyACM0"
BAUD = 115200

ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.timeout = 0.1
ser.dtr = False   # GPIO0 high 유지 → 다운로드 모드 회피
ser.rts = False
ser.open()


def reader():
    while True:
        try:
            data = ser.read(256)
        except Exception:  # noqa: BLE001
            break
        if data:
            sys.stdout.write(data.decode(errors="replace"))
            sys.stdout.flush()


threading.Thread(target=reader, daemon=True).start()
print(f"[연결됨 {PORT} @{BAUD}] 1/2/3/4 선택, 0/s 정지, +/- 조정, r 리셋, R=RTS리셋, Ctrl-C 종료")
print("[안내] ⚠️ 프로펠러 제거 확인! 응답 없으면 'R' 입력해 RTS 리셋.")

try:
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        s = line.strip()
        if s == "R":
            ser.rts = True
            time.sleep(0.1)
            ser.rts = False   # RTS-only 펄스 → 런모드 재부팅(GPIO0 high 유지)
            print("[로컬] RTS 리셋 전송")
            continue
        for ch in s:
            ser.write(ch.encode())
except KeyboardInterrupt:
    pass
finally:
    try:
        ser.write(b"s")
        time.sleep(0.05)
    except Exception:  # noqa: BLE001
        pass
    ser.close()
    print("\n[종료] 정지 신호(s) 전송 후 포트 닫음.")
