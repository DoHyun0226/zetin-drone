"""``drone_control.py --pad-check``용 게임패드 axis/button/hat 진단.

로컬 전용이다: 소켓을 열지 않고 드론에 아무것도 보내지 않는다. pygame은
``main`` 안에서 import하므로 이 모듈을 import해도 부작용이 없다.

기본: 정지 값을 잰 뒤(처음 0.7 s는 손을 뗀다), 바뀌는 입력만
axis/button/hat 번호와 함께 출력한다:

    AXIS  4:  -1.00 -> +1.00
    BUTTON 0:  DOWN / UP
    HAT   0:  (0, 0) -> (0, 1)

``--live``는 대신 모든 값을 계속 갱신되는 한 줄에 출력한다
(이름표는 보통의 DualSense 배치를 따른다. 번호는 직접 확인한다).
"""

import argparse
import time

AXIS_CHANGE_THRESHOLD = 0.15  # 스틱 노이즈 무시용
REST_SECONDS = 0.7
POLL_SECONDS = 0.02
LIVE_POLL_SECONDS = 0.05


def snapshot(joy):
    axes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
    btns = [joy.get_button(i) for i in range(joy.get_numbuttons())]
    hats = [joy.get_hat(i) for i in range(joy.get_numhats())]
    return axes, btns, hats


def print_full(joy):
    axes, btns, hats = snapshot(joy)
    print("  AXES :", "  ".join(f"a{i}={v:+.2f}" for i, v in enumerate(axes)))
    print("  BTNS :", "  ".join(f"b{i}={v}" for i, v in enumerate(btns)))
    print("  HATS :", "  ".join(f"h{i}={v}" for i, v in enumerate(hats)))


def _live_line(joy):
    axes, btns, hats = snapshot(joy)
    axis_text = " ".join(f"a{i}:{v:+.2f}" for i, v in enumerate(axes))
    pushed = [f"BTN_{i}" for i, v in enumerate(btns) if v]
    hat = hats[0] if hats else None
    return f"{axis_text} | HAT:{hat} | PUSH:{pushed}"


def _run_live(pygame, joy):
    try:
        while True:
            pygame.event.pump()
            print(f"\r{_live_line(joy)}      ", end="", flush=True)
            time.sleep(LIVE_POLL_SECONDS)
    except KeyboardInterrupt:
        print("\n종료합니다.")


def _run_diagnosis(pygame, joy):
    print("\n[휴지값 측정] 0.7초간 아무것도 만지지 마세요...")
    t_end = time.monotonic() + REST_SECONDS
    while time.monotonic() < t_end:
        pygame.event.pump()
        time.sleep(POLL_SECONDS)

    prev_axes, prev_btns, prev_hats = snapshot(joy)
    print("[휴지 스냅샷]")
    print_full(joy)
    print("\n이제 각 컨트롤을 하나씩 천천히 움직이세요 (반응한 항목만 출력).")
    print("확인 권장 순서: 왼쪽스틱↔↕ → 오른쪽스틱↔↕ → L2 → R2 → L1/R1 →")
    print("                X/O/△/□ → DPAD상하좌우 → 기타 버튼.")
    print("(Ctrl-C 로 종료)\n")

    try:
        while True:
            pygame.event.pump()
            axes, btns, hats = snapshot(joy)

            for i in range(len(axes)):
                if abs(axes[i] - prev_axes[i]) > AXIS_CHANGE_THRESHOLD:
                    print(f" AXIS  {i}:  {prev_axes[i]:+.2f} -> {axes[i]:+.2f}")
                    prev_axes[i] = axes[i]
            for i in range(len(btns)):
                if btns[i] != prev_btns[i]:
                    print(f" BUTTON {i}:  {'DOWN' if btns[i] else 'UP'}")
                    prev_btns[i] = btns[i]
            for i in range(len(hats)):
                if hats[i] != prev_hats[i]:
                    print(f" HAT   {i}:  {prev_hats[i]} -> {hats[i]}")
                    prev_hats[i] = hats[i]

            time.sleep(POLL_SECONDS)
    except KeyboardInterrupt:
        print("\n\n[종료 스냅샷]")
        print_full(joy)
        print()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="drone_control.py --pad-check",
        description="Gamepad axis/button/hat diagnosis. Sends nothing to the drone.",
    )
    # drone_control.py가 모드 플래그를 그대로 넘긴다.
    parser.add_argument("--pad-check", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--live",
        action="store_true",
        help="print every value continuously on one line instead of only changes",
    )
    args = parser.parse_args(argv)

    import pygame

    pygame.init()
    pygame.joystick.init()
    try:
        if pygame.joystick.get_count() == 0:
            print("[ERR] 조이스틱이 없습니다. 게임패드를 연결하세요.")
            return 1

        joy = pygame.joystick.Joystick(0)
        joy.init()
        print("=" * 56)
        print(f" 조이스틱: {joy.get_name()}")
        print(f" axes={joy.get_numaxes()}  buttons={joy.get_numbuttons()}  "
              f"hats={joy.get_numhats()}")
        print("=" * 56)
        if args.live:
            _run_live(pygame, joy)
        else:
            _run_diagnosis(pygame, joy)
        return 0
    finally:
        pygame.quit()
