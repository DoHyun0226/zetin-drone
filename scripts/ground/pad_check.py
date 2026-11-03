"""Gamepad axis/button/hat diagnosis for ``drone_control.py --pad-check``.

Local only: never opens a socket and sends nothing to the drone. pygame is
imported inside ``main`` so importing this module has no side effects.

Default: measure the resting values (first 0.7 s hands off), then print only
the inputs that change, with their axis/button/hat index:

    AXIS  4:  -1.00 -> +1.00
    BUTTON 0:  DOWN / UP
    HAT   0:  (0, 0) -> (0, 1)

``--live`` instead prints every value on one continuously refreshed line
(the labels there follow the usual DualSense layout; verify the numbers).
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
