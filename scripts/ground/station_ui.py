"""지상국의 출력 계층.

``control_station``은 ``print``를 직접 부르지 않고 모든 줄을 UI 객체에 넘긴다.
``ConsoleUI``는 지상국이 원래 내던 평범한 줄 출력을 글자 하나까지 똑같이 낸다.
``DashboardUI``는 전체 화면 prompt_toolkit view(상태 패널, 이벤트 목록, 명령 줄)다.
``make_ui``는 터미널이면 이것을 고르고, 아니면 ``ConsoleUI``로 돌아간다.
prompt_toolkit은 대시보드 경로에서만 늦게 import한다.

모든 UI의 계약: 어떤 메서드도 예외를 내지 않는다. 출력 메서드(``event``,
``periodic``, ``transition``, ``alert``, ``update``, ``after_run``)와 ``run``
모두 그렇다. 출력 메서드는 안전 경로(disarm, RC streaming 정지, worker-fault
처리)에서 상태를 바꾼 뒤 불리므로, 터미널이 망가졌다고 그 경로가 중단되면 안
된다. 실패는 삼키고 stderr에 많아야 한 줄 남긴다.

``DashboardUI``의 잠금 규칙: 출력 메서드는 ``telem_lock``을 잡은 텔레메트리
스레드와 안전 경로에서 불린다. 출력 메서드는 UI 자신의 leaf lock만 건드리고
(지상국 잠금이나 ``snapshot()``은 건드리지 않는다), redraw를 기다리지 않고,
예외를 내지 않는다. 지상국 snapshot provider는 event-loop 스레드에서 대시보드
자신의 redraw가 UI 잠금 밖에서만 부른다.

대시보드 내부 오류(event loop 안의 예외, 실패한 redraw)는 운용자 정지가 아니다:
``DashboardUI``는 진단 출력보다 먼저 지상국 fault handler(``set_fault_handler``:
latch, RC 정지, disarm 없음)를 부르고, 그 뒤 Ctrl+C/Ctrl+D까지 ``ConsoleUI``로
계속된다.

``input_end_hold``(``--listen``): 대화형 Ctrl+D 없이 입력이 끝나면 ``run``은 이
blocking callable을 불러 수신을 계속한다. 기다리는 동안 Ctrl+C/SIGTERM이 오면
Ctrl+C와 같이 실행을 끝낸다.
"""

import collections
import importlib.util
import sys
import threading
import time


def _report_ui_failure(method, error):
    """stderr에 한 줄 메모를 남기려 시도한다. 예외를 내지 않는다."""
    try:
        sys.stderr.write(f"[ConsoleUI] {method} 출력 실패: {error!r}\n")
    except Exception:
        pass


INPUT_END_HOLD_MESSAGE = "[STDIN] 입력 종료 - 수신 계속 (Ctrl+C로 종료)"


class ConsoleUI:
    """평범한 줄 출력: 지상국 원래의 ``print`` 동작."""

    def __init__(self, input_end_hold=None):
        self._input_end_hold = input_end_hold

    def set_snapshot_provider(self, provider):
        """콘솔은 패널을 그리지 않으므로 snapshot을 가져오지 않는다."""

    def set_fault_handler(self, handler):
        """콘솔에는 실패할 수 있는 내부 renderer가 없다. 알릴 것이 없다."""

    def event(self, text):
        """한 번 나오는 이벤트 줄(arm/disarm, fault, 경고, 로그...)."""
        try:
            print(text)
        except Exception as error:
            _report_ui_failure("event", error)

    def periodic(self, text):
        """반복되는 줄([STATUS], [DIAG], [AUTO-LAND], [AXIS], 아날로그 [TH])."""
        try:
            print(text)
        except Exception as error:
            _report_ui_failure("periodic", error)

    def alert(self, text):
        """놓치면 안 되는 줄. 콘솔에는 따로 둘 곳이 없다."""
        self.event(text)

    def transition(self, text):
        """대시보드용 상태 전이 이벤트. 콘솔은 아무것도 출력하지 않는다."""

    def update(self, snap):
        """대시보드용 새 지상국 snapshot. 콘솔은 무시한다."""

    def run(self, input_handler, on_interrupt, on_eof):
        """stdin 명령 루프. 예외를 내지 않는다.

        Ctrl+C -> ``on_interrupt()``, Ctrl+D(EOF) -> ``on_eof()``, 그리고
        돌아온다. 입력 스트림이 망가지면(stdin 닫힘, I/O 오류) EOF처럼 끝난다:
        ``on_eof()``의 조건부 disarm을 먼저 하고 오류를 보고한다. 예외를 낸
        명령은 이벤트로 보고하고 루프를 계속한다. ``input_end_hold``가 있으면
        EOF에서 끝나지 않는다: ``on_eof()`` 뒤 Ctrl+C/SIGTERM
        (``on_interrupt()``)이 오거나 hold가 돌아올 때까지 hold에서 기다린다.
        """
        while True:
            try:
                msg = input()
            except KeyboardInterrupt:
                self._call_safely("on_interrupt", on_interrupt)
                return
            except EOFError:
                self._call_safely("on_eof", on_eof)
                self._hold_after_input_end(on_interrupt)
                return
            except Exception as error:
                # 안전 경로(조건부 disarm)를 먼저 돌리고, 알림은 그 뒤에 낸다.
                # 출력이 막혀 있어도 disarm이 늦어지지 않게 한다.
                self._call_safely("on_eof", on_eof)
                self.event(
                    f"[STDIN] 입력 오류: {type(error).__name__}: {error} "
                    "- 입력 종료로 처리"
                )
                self._hold_after_input_end(on_interrupt)
                return
            if not msg:
                continue
            try:
                input_handler(msg)
            except KeyboardInterrupt:
                self._call_safely("on_interrupt", on_interrupt)
                return
            except Exception as error:
                self.event(
                    f"[STDIN] 명령 처리 오류: {type(error).__name__}: {error}"
                )

    @staticmethod
    def _call_safely(name, callback):
        try:
            callback()
        except Exception as error:
            _report_ui_failure(name, error)

    def _hold_after_input_end(self, on_interrupt):
        """--listen: 입력은 끝났지만 Ctrl+C/SIGTERM까지 수신은 계속한다."""
        hold = self._input_end_hold
        if hold is None:
            return
        try:
            self.event(INPUT_END_HOLD_MESSAGE)
            hold()
        except KeyboardInterrupt:
            self._call_safely("on_interrupt", on_interrupt)
        except Exception as error:
            _report_ui_failure("input_end_hold", error)

    def after_run(self):
        """``run``이 돌아온 뒤의 hook. 콘솔은 버퍼에 쌓아 둔 것이 없다."""


# ---------------------------------------------------------------------------
# 전체 화면 대시보드(prompt_toolkit, 늦게 import)
# ---------------------------------------------------------------------------
EVENT_HISTORY = 500
REDRAW_INTERVAL_SEC = 0.1       # min_redraw_interval과 refresh_interval: 10 Hz
LINK_RATE_WINDOW_SEC = 1.0      # packets/s는 최소 이 시간 동안 평균한다
_FIXED_ROWS = 3                 # 이벤트 제목 + 구분선 + 입력 줄
_MIN_EVENT_ROWS = 1
_PAD_HINTS = "X 시동  △ 복구  R2/L2 스로틀  ? 도움말 "
_LISTEN_HINTS = "? 도움말 "
def _wrap_cells(text, width):
    """``text``를 각각 터미널 칸 ``width``에 들어가는 조각으로 나눈다."""
    from prompt_toolkit.utils import get_cwidth

    width = max(1, width)
    pieces, current, used = [], [], 0
    for char in text:
        cells = get_cwidth(char)
        if current and used + cells > width:
            pieces.append("".join(current))
            current, used = [], 0
        current.append(char)
        used += cells
    pieces.append("".join(current))
    return pieces


_STYLE_RULES = {
    "armed": "bg:ansired fg:ansiwhite bold",
    "disarmed": "bg:ansigreen fg:ansiblack bold",
    "alert": "fg:ansired bold",
    "warn": "fg:ansiyellow",
    "dim": "fg:ansicyan",
    "ok": "fg:ansigreen",
    "value": "",
    "event-time": "fg:ansicyan",
    "input": "",
}


def _event_style(kind, text):
    if kind == "alert" or text.startswith(("[ERR]", "[FAULT]")):
        return "class:alert"
    if text.startswith("[WARN]"):
        return "class:warn"
    if kind in ("transition", "echo"):
        return "class:dim"
    return ""


class DashboardUI:
    """고정 상태 패널, 이벤트 목록, 명령 줄(스펙 6절).

    ``event``/``transition``/``alert``는 시각을 붙인 항목을 넣고 redraw를
    요청한다. ``periodic``은 무시한다(그 값은 패널이 보여준다). ``update``는
    최신 snapshot으로 바꿔 넣는다. ``Application``이 redraw 요청을 묶고
    (``min_redraw_interval``) ``refresh_interval``마다 스스로 다시 그려서
    "수신 없음 N.Ns" 경과 시간이 계속 움직인다.

    대시보드가 사라지면(``after_run`` 또는 시작 실패) 모든 출력 메서드가
    ``ConsoleUI``로 넘긴다. 그래서 이후 줄도 터미널에 나온다.
    """

    def __init__(self, help_lines=(), *, input=None, output=None, clock=time.monotonic,
                 snapshot_provider=None, input_end_hold=None, fault_handler=None):
        # prompt_toolkit(과 그것이 필요한 대시보드 view)은 여기서 불러온다.
        from ground import dashboard_view

        self._view = dashboard_view
        self._snapshot_provider = snapshot_provider
        self._input_end_hold = input_end_hold
        self._fault_handler = fault_handler
        self._fault = None   # 첫 대시보드 내부 오류(_report_fault 참고)
        self._help_lines = list(help_lines)
        self._input = input
        self._output = output
        self._clock = clock
        self._lock = threading.Lock()        # leaf lock: 이벤트, snapshot, 속도
        self._print_lock = threading.Lock()  # after_run 출력 순서를 지킨다
        self._events = collections.deque(maxlen=EVENT_HISTORY)
        self._snap = {}
        self._packets = 0
        self._rate = dashboard_view.LinkRateTracker()
        self._rate_time = None
        self._link_hz = None
        self._console = None   # 대시보드가 사라진 뒤의 ConsoleUI
        self._ended = False    # 안전 callback(on_interrupt/on_eof)이 실행됨
        self._app = None
        self._textarea = None
        self._panel_cache = None    # 한 프레임의 (render_counter, rows)
        self._drawn_panel = []
        self._drawn_events = []

    # --- 출력 메서드(아무 스레드. 막히지 않고 예외를 내지 않는다) -------------
    def event(self, text):
        self._add("event", "event", text)

    def transition(self, text):
        self._add("transition", "transition", text)

    def alert(self, text):
        self._add("alert", "alert", text)

    def periodic(self, text):
        """[STATUS]/[DIAG]/[AUTO-LAND]/[AXIS]/[TH]: 이 값들은 패널이 보여준다."""
        console = self._console
        if console is not None:
            with self._print_lock:
                console.periodic(text)

    def update(self, snap):
        """최신 지상국 snapshot. 텔레메트리 패킷마다 한 번 불린다(유효하든 깨졌든)."""
        try:
            console = self._console
            if console is not None:
                console.update(snap)
                return
            self._store_snapshot(snap)
            self._invalidate()
        except Exception as error:
            _report_ui_failure("update", error)

    def set_snapshot_provider(self, provider):
        """지상국 ``snapshot`` 함수. redraw마다 가져온다(event-loop 스레드)."""
        self._snapshot_provider = provider

    def set_fault_handler(self, handler):
        """대시보드 내부 오류용 지상국 callback ``handler(error)``.

        오류를 본 스레드에서 진단 출력보다 먼저 한 번 불린다. 오류를 운용자
        정지로 다루지 않고 지상국을 안전하게 해야 한다(지상국은 fault를 latch하고
        RC를 멈춘다).
        """
        self._fault_handler = handler

    def _report_fault(self, error):
        """첫 내부 오류: 지상국 fault handler를 정확히 한 번 부른다. 예외를 내지 않는다."""
        if self._fault is not None:
            return
        self._fault = error
        handler = self._fault_handler
        if handler is not None:
            ConsoleUI._call_safely("fault_handler", lambda: handler(error))

    def _store_snapshot(self, snap):
        now = self._clock()
        # 링크 Hz는 유효 수신 수로 잰다: 잘못된 패킷도 불량 수 갱신을 위해
        # update()를 부른다. 그 수가 없는 스냅숏은 호출마다 1로 센다.
        valid = snap.get("valid_packet_count") if isinstance(snap, dict) else None
        with self._lock:
            self._snap = snap
            if valid is None:
                self._packets += 1
            else:
                self._packets = valid
            if (self._rate_time is None
                    or now - self._rate_time >= LINK_RATE_WINDOW_SEC):
                self._link_hz = self._rate.update(now, self._packets)
                self._rate_time = now

    def _pull_snapshot(self):
        """refresh tick(event-loop 스레드): 지상국 상태를 직접 읽는다.

        로컬 안전 변화(disarm, 링크 손실 때 RC 정지, worker fault)는 datagram 없이
        일어나므로 뒤따르는 update()가 없다. provider는 telem_lock을 잡는다. 여기서는
        UI 잠금 없이 부르고, 출력 메서드에서는 부르지 않는다(그 호출자가
        telem_lock을 잡고 있을 수 있다).
        """
        provider = self._snapshot_provider
        if provider is None:
            return
        try:
            snap = provider()
        except Exception:
            return  # 마지막 snapshot을 유지한다. 패널이 경과 시간을 보여준다
        self._store_snapshot(snap)

    def _add(self, method, kind, text):
        try:
            lines = [line for line in str(text).split("\n") if line.strip()]
            stamp = time.strftime("%H:%M:%S")
            with self._lock:
                console = self._console
                if console is None:
                    for line in lines:
                        self._events.append((stamp, line, _event_style(kind, line)))
            if console is not None:
                # 대시보드가 끝난 뒤(로그 저장·raw 변환): 일반 출력으로 바로 낸다.
                with self._print_lock:
                    getattr(console, method)(text)
                return
            self._invalidate()
        except Exception as error:
            _report_ui_failure(method, error)

    def _invalidate(self):
        app = self._app
        if app is not None:
            app.invalidate()  # thread safe. 바로 돌아온다

    # --- 그리기와 시험용 상태 사본 --------------------------------
    @property
    def running(self):
        app = self._app
        return bool(app is not None and app.is_running)

    def _event_texts(self):
        with self._lock:
            return [text for _stamp, text, _style in self._events]

    def _draw_snapshot(self):
        with self._lock:
            snap = dict(self._snap)
            snap["link_hz"] = self._link_hz
        return snap

    def _drawn_text(self):
        rows = list(self._drawn_panel) + list(self._drawn_events)
        return "\n".join("".join(text for _style, text in row) for row in rows)

    # --- 그리기(event-loop 스레드 전용) ----------------------------------
    def _size(self):
        size = self._app.output.get_size()
        return max(1, size.columns), max(1, size.rows)

    def _panel_rows(self):
        """이 프레임의 상단 바 + 패널. render마다 캐시한다."""
        app = self._app
        counter = getattr(app, "render_counter", None)
        cached = self._panel_cache
        if cached is not None and cached[0] == counter:
            return cached[1]
        self._pull_snapshot()
        try:
            width, height = self._size()
            # compact 배치는 터미널 전체 크기로 정한다(80x24 규칙).
            # 이벤트 제목, 이벤트 한 행, 입력 줄은 계속 보인다.
            rows = self._view.render(self._draw_snapshot(), width, height, time.monotonic())
            rows = rows[:max(0, height - _FIXED_ROWS - _MIN_EVENT_ROWS)]
        except Exception as error:
            rows = [[("class:alert", f"[대시보드] 그리기 오류: {type(error).__name__}: {error}")]]
        self._panel_cache = (counter, rows)
        self._drawn_panel = rows
        return rows

    def _panel_fragments(self):
        return self._join_rows(self._panel_rows())

    def _panel_height(self):
        return max(1, len(self._panel_rows()))

    def _event_fragments(self):
        try:
            width, height = self._size()
            available = max(_MIN_EVENT_ROWS, height - _FIXED_ROWS - len(self._panel_rows()))
            with self._lock:
                entries = list(self._events)[-available:]
            rows = []
            for stamp, text, style in entries:
                lead = f"{stamp} "
                indent = " " * len(lead)
                for i, chunk in enumerate(_wrap_cells(text, width - len(lead))):
                    rows.append([("class:event-time", lead if i == 0 else indent), (style, chunk)])
            rows = rows[-available:]
        except Exception as error:
            rows = [[("class:alert", f"[대시보드] 사건 그리기 오류: {error!r}")]]
        self._drawn_events = rows
        return self._join_rows(rows)

    def _event_title(self):
        try:
            width, _height = self._size()
        except Exception:
            width = 80
        return [("class:dim", "─ 사건 " + "─" * max(0, width - 7))]

    def _hints(self):
        with self._lock:
            pad = self._snap.get("mode", "pad") == "pad"
        # --keyboard·--listen에는 패드가 없다: 시동·복구·스로틀 안내를 숨긴다.
        return [("class:dim", _PAD_HINTS if pad else _LISTEN_HINTS)]

    @staticmethod
    def _join_rows(rows):
        fragments = []
        for index, row in enumerate(rows):
            if index:
                fragments.append(("", "\n"))
            fragments.extend(row)
        return fragments

    # --- application ---------------------------------------------------
    def _build_app(self, input_handler, on_interrupt, on_eof):
        from prompt_toolkit.application import Application
        from prompt_toolkit.application.current import get_app_session
        from prompt_toolkit.filters import Condition
        from prompt_toolkit.key_binding import KeyBindings
        from prompt_toolkit.keys import Keys
        from prompt_toolkit.layout import HSplit, Layout, VSplit, Window
        from prompt_toolkit.layout.controls import FormattedTextControl
        from prompt_toolkit.styles import Style
        from prompt_toolkit.widgets import TextArea

        def accept(buffer):
            text = buffer.text
            if text:
                self._add("event", "echo", f"> {text}")
                try:
                    input_handler(text)
                except KeyboardInterrupt:
                    self._end(app_ref[0], "on_interrupt", on_interrupt)
                except Exception as error:
                    self.event(f"[STDIN] 명령 처리 오류: {type(error).__name__}: {error}")
            return False  # 줄을 비운다(기록에는 남는다)

        textarea = TextArea(
            prompt="> ", multiline=False, wrap_lines=False,
            accept_handler=accept, style="class:input",
        )
        self._textarea = textarea

        bindings = KeyBindings()

        @bindings.add("c-c", eager=True)
        @bindings.add(Keys.SIGINT)
        def _interrupt(event):
            self._end(event.app, "on_interrupt", on_interrupt)

        # 지금의 터미널 input()과 같이 빈 줄에서만 입력 종료(EOF)로 본다. 글자가
        # 있으면 기본 동작(커서 위치 글자 지우기)을 둔다.
        @bindings.add("c-d", eager=True, filter=Condition(lambda: textarea.text == ""))
        def _eof(event):
            self._end(event.app, "on_eof", on_eof)

        @bindings.add("escape", eager=True)
        def _clear(_event):
            textarea.text = ""

        @bindings.add("?")
        def _help(_event):
            for line in self._help_lines:
                self.event(line)

        layout = Layout(HSplit([
            Window(FormattedTextControl(self._panel_fragments),
                   height=self._panel_height, wrap_lines=False),
            Window(FormattedTextControl(self._event_title), height=1),
            Window(FormattedTextControl(self._event_fragments), wrap_lines=False),
            Window(char="─", height=1, style="class:dim"),
            VSplit([
                textarea,
                Window(FormattedTextControl(self._hints), dont_extend_width=True),
            ], height=1),
        ]), focused_element=textarea)

        # 입력이 닫히면 prompt_toolkit은 EOFError를 내기 전에 마지막 그리기를
        # 한다(출력이 막히면 거기서 멈춘다). 닫힘을 처음 보는 순간 EOF 안전
        # 콜백(조건부 disarm)을 먼저 돌린다.
        real_input = self._input if self._input is not None else get_app_session().input
        watched_input = _InputClosedWatch(
            real_input, lambda: self._run_safety("on_eof", on_eof),
        )
        app = Application(
            layout=layout,
            key_bindings=bindings,
            style=Style.from_dict(_STYLE_RULES),
            full_screen=True,
            min_redraw_interval=REDRAW_INTERVAL_SEC,
            refresh_interval=REDRAW_INTERVAL_SEC,
            input=watched_input,
            output=self._output,
        )
        app_ref = [app]
        return app

    def _run_safety(self, name, callback):
        """첫 안전 callback(on_interrupt/on_eof)을 정확히 한 번 실행한다."""
        if not self._ended:
            self._ended = True
            ConsoleUI._call_safely(name, callback)

    def _end(self, app, name, callback):
        """Ctrl+C / Ctrl+D: 안전 callback을 먼저 하고 app을 나간다."""
        self._run_safety(name, callback)
        try:
            app.exit()
        except Exception:
            pass  # 이미 나가는 중

    def run(self, input_handler, on_interrupt, on_eof):
        """Ctrl+C / Ctrl+D까지 대시보드를 돌린다. 예외를 내지 않는다.

        ``ConsoleUI.run``과 같은 계약: Ctrl+C -> ``on_interrupt()``,
        Ctrl+D -> ``on_eof()``. 입력이 다른 식으로 끝나면(입력 닫힘,
        application 안의 오류) ``on_eof()``를 먼저 하고 보고한다. 예외를 낸
        명령은 이벤트로 보고한다. 대시보드를 아직 만드는 중에(또는 키를 놓은
        뒤에) 온 Ctrl+C도 ``on_interrupt()``를 실행한다. 내부 오류(event loop
        안의 예외, 실패한 redraw)는 정지 요청이 아니다: fault handler를 먼저
        실행하고, 그 뒤 Ctrl+C/Ctrl+D까지 줄 콘솔로 계속된다.
        """
        try:
            self._run(input_handler, on_interrupt, on_eof)
        except KeyboardInterrupt:
            # 키 바인딩이 Ctrl+C를 받기 전(대시보드 구성 중·시작 직전) 또는 그 뒤
            self._run_safety("on_interrupt", on_interrupt)
        except BaseException as error:  # noqa: BLE001 - run()은 예외를 내면 안 된다
            self._report_fault(error)
            self._continue_on_console(error, input_handler, on_interrupt, on_eof)

    def _run(self, input_handler, on_interrupt, on_eof):
        try:
            app = self._build_app(input_handler, on_interrupt, on_eof)
        except Exception as error:
            # 대시보드를 못 띄우면 줄 출력으로 계속 조종할 수 있게 한다. 이후의
            # 모든 출력(주기 줄 포함)은 ConsoleUI로 넘긴다.
            console = self._switch_to_console()
            console.event(
                f"[WARN] 대시보드 시작 실패: {type(error).__name__}: {error} - 줄 출력으로 실행"
            )
            console.run(input_handler, on_interrupt, on_eof)
            return
        self._app = app
        error = None
        input_closed = False
        try:
            # prompt_toolkit의 기본 처리기는 예외를 찍고 ENTER를 기다리며 앱을
            # 그대로 둔다. 대신 지상국 고장 처리부터 돌리고 앱을 끝내는 처리기를 건다.
            app.run(
                pre_run=lambda: self._install_loop_exception_handler(app),
                set_exception_handler=False,
            )
        except KeyboardInterrupt:
            self._run_safety("on_interrupt", on_interrupt)
        except EOFError:
            # 입력이 닫혔다: on_eof는 닫힘을 본 순간 이미 돌았다(아래가 예비)
            input_closed = True
        except BaseException as caught:  # noqa: BLE001 - run()은 예외를 내면 안 된다
            error = caught
        finally:
            self._app = None
        if error is not None:
            # 앱 자체가 예외로 끝났다(그리기 실패 등): 조작자의 정지가 아니라 고장이다.
            self._report_fault(error)
        if self._fault is not None:
            # 고장 처리(래치·RC 중단, stop 없음)는 이미 끝났다. 줄 출력으로 계속한다.
            self._continue_on_console(self._fault, input_handler, on_interrupt, on_eof)
            return
        # 입력이 키 없이 끝났다: EOF와 같이 조건부 disarm부터(닫힘을 본 순간 이미 돌았다).
        self._run_safety("on_eof", on_eof)
        if input_closed and self._input_end_hold is not None:
            # --listen: 입력이 닫혀도(대화형 Ctrl+D가 아니면) 수신은 계속한다.
            self._switch_to_console()._hold_after_input_end(on_interrupt)

    def _install_loop_exception_handler(self, app):
        """pre_run(돌고 있는 루프 안): 루프 예외를 fault 경로로 보낸다."""
        import asyncio

        asyncio.get_running_loop().set_exception_handler(
            lambda _loop, context: self._on_loop_exception(app, context)
        )

    def _on_loop_exception(self, app, context):
        """event-loop 예외: 지상국 fault handler를 먼저 하고 app을 나간다.

        예외를 내지 않는다. 운용자 정지가 아니므로 여기서는 on_interrupt/on_eof가 없다.
        """
        try:
            exception = context.get("exception")
            if exception is None:
                exception = RuntimeError(str(context.get("message", "event loop error")))
            self._report_fault(exception)
            try:
                app.exit()
            except Exception:
                pass  # 이미 나가는 중
        except Exception as error:
            _report_ui_failure("event loop", error)

    def _continue_on_console(self, error, input_handler, on_interrupt, on_eof):
        """내부 오류 뒤: 진단을 내고, Ctrl+C/Ctrl+D까지 줄 콘솔로 간다."""
        console = self._switch_to_console()
        console.event(
            f"[WARN] 대시보드 오류: {type(error).__name__}: {error} - 줄 출력으로 계속"
        )
        console.run(input_handler, on_interrupt, on_eof)

    def interrupt(self):
        """signal handler(SIGTERM)에서 온 Ctrl+C 요청. 예외를 내지 않고 막히지 않는다.

        돌고 있는 대시보드가 요청을 받으면 True: 요청은 SIGINT 키로 event loop에
        넘어가므로, 보통의 Ctrl+C 바인딩이 안전 callback을 실행하고 app을 나간다.
        대시보드 루프가 돌고 있지 않으면 False: 그러면 호출한 쪽이
        KeyboardInterrupt를 내고, ``run``이 이것을 ``on_interrupt()``로 바꾼다.
        """
        try:
            app = self._app
            loop = getattr(app, "loop", None)
            if app is None or loop is None or not app.is_running or loop.is_closed():
                return False
            loop.call_soon_threadsafe(app.key_processor.send_sigint)
            return True
        except Exception:
            return False

    def after_run(self):
        """대시보드 닫힘: 이벤트 목록을 출력한다. 이후 출력은 ConsoleUI로 간다."""
        console = self._console
        if console is not None:
            console.after_run()
            return
        self._switch_to_console()

    def _switch_to_console(self):
        """쌓아 둔 이벤트 목록을 출력하고, 이후 모든 출력 메서드를 넘긴다."""
        console = ConsoleUI(input_end_hold=self._input_end_hold)
        try:
            with self._print_lock:
                with self._lock:
                    if self._console is not None:
                        return self._console
                    entries = list(self._events)
                    self._events.clear()
                    self._console = console
                for stamp, text, _style in entries:
                    print(f"{stamp} {text}")
        except Exception as error:
            _report_ui_failure("after_run", error)
        return console


class _InputClosedWatch:
    """prompt_toolkit 입력 wrapper: ``closed``가 처음 True로 보이면 ``on_closed``를 부른다.

    prompt_toolkit은 키를 읽은 직후 ``input.closed``를 확인하고, True이면
    마지막 redraw 한 번 뒤 EOFError로 app을 끝낸다.
    """

    def __init__(self, inner, on_closed):
        self._inner = inner
        self._on_closed = on_closed

    def __getattr__(self, name):
        return getattr(self._inner, name)

    @property
    def closed(self):
        closed = self._inner.closed
        if closed:
            self._on_closed()
        return closed


def make_ui(args, isatty=None, *, help_lines=(), snapshot_provider=None,
            input_end_hold=None):
    """터미널이면 대시보드. --plain, 파이프, prompt_toolkit 없음이면 ``ConsoleUI``.

    ``snapshot_provider``는 대시보드가 redraw 때 가져온다.
    ``input_end_hold``는 입력이 끝난 뒤에도 실행을 이어 간다(--listen).
    """
    if getattr(args, "plain", False):
        return ConsoleUI(input_end_hold=input_end_hold)
    try:
        if isatty is not None:
            on_terminal = bool(isatty())
        else:
            on_terminal = sys.stdout.isatty() and sys.stdin.isatty()
    except Exception:
        on_terminal = False
    if not on_terminal:
        return ConsoleUI(input_end_hold=input_end_hold)
    try:
        if importlib.util.find_spec("prompt_toolkit") is None:
            raise ImportError("prompt_toolkit")
        return DashboardUI(help_lines=help_lines, snapshot_provider=snapshot_provider,
                           input_end_hold=input_end_hold)  # fault handler: main()이 설정한다
    except (ImportError, ValueError):
        console = ConsoleUI(input_end_hold=input_end_hold)
        console.event("[WARN] prompt_toolkit 없음 - 줄 출력으로 실행")
        return console
