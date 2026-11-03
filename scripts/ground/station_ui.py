"""Output layer of the control station.

``control_station`` never calls ``print`` directly; it hands every line to a
UI object. ``ConsoleUI`` reproduces the plain line output the station has
always had, character for character. ``DashboardUI`` is the full-screen
prompt_toolkit view (status panels, event list, command line); ``make_ui``
picks it on a terminal and falls back to ``ConsoleUI`` otherwise.
prompt_toolkit is imported lazily, only on the dashboard path.

Contract for every UI: no method raises — neither the output methods
(``event``, ``periodic``, ``transition``, ``alert``, ``update``,
``after_run``) nor ``run``. The output methods are called from the safety
paths (disarm, stop of RC streaming, worker-fault handling) after the state
change, and a broken terminal must not abort those paths. Failures are
swallowed with at most one line on stderr.

Locking rule for ``DashboardUI``: its output methods are called from the
telemetry thread while ``telem_lock`` is held and from the safety paths.
They only touch the UI's own leaf lock (never a station lock, never
``snapshot()``), never wait for a redraw, and never raise. The station's
snapshot provider is called only from the dashboard's own redraw on the
event-loop thread, outside the UI lock.

An internal dashboard error (an exception in the event loop, a failing
redraw) is not an operator stop: ``DashboardUI`` calls the station's fault
handler (``set_fault_handler``: latch, RC stop, no disarm) before any
diagnostic, then continues as a ``ConsoleUI`` until Ctrl+C/Ctrl+D.

``input_end_hold`` (``--listen``): when the input ends without an
interactive Ctrl+D, ``run`` keeps reception going by calling this blocking
callable; a Ctrl+C/SIGTERM while it waits ends the run like Ctrl+C.
"""

import collections
import importlib.util
import sys
import threading
import time


def _report_ui_failure(method, error):
    """Best-effort one-line note on stderr; never raises."""
    try:
        sys.stderr.write(f"[ConsoleUI] {method} 출력 실패: {error!r}\n")
    except Exception:
        pass


INPUT_END_HOLD_MESSAGE = "[STDIN] 입력 종료 - 수신 계속 (Ctrl+C로 종료)"


class ConsoleUI:
    """Plain line output: the station's original ``print`` behaviour."""

    def __init__(self, input_end_hold=None):
        self._input_end_hold = input_end_hold

    def set_snapshot_provider(self, provider):
        """The console draws no panels; it never pulls a snapshot."""

    def set_fault_handler(self, handler):
        """The console has no internal renderer that could fail; nothing to report."""

    def event(self, text):
        """One-off event line (arm/disarm, faults, warnings, logs...)."""
        try:
            print(text)
        except Exception as error:
            _report_ui_failure("event", error)

    def periodic(self, text):
        """Repeating line ([STATUS], [DIAG], [AUTO-LAND], [AXIS], analog [TH])."""
        try:
            print(text)
        except Exception as error:
            _report_ui_failure("periodic", error)

    def alert(self, text):
        """Must-not-miss line; the console has nowhere else to put it."""
        self.event(text)

    def transition(self, text):
        """State-transition event for the dashboard; the console prints nothing."""

    def update(self, snap):
        """Fresh station snapshot for the dashboard; the console ignores it."""

    def run(self, input_handler, on_interrupt, on_eof):
        """The stdin command loop; never raises.

        Ctrl+C -> ``on_interrupt()``, Ctrl+D (EOF) -> ``on_eof()``, then
        return. A broken input stream (closed stdin, I/O error) ends like
        EOF: ``on_eof()``'s conditional disarm runs first, then the error is
        reported. A command that raises is reported as an event and the loop
        continues. With ``input_end_hold`` the run does not end at EOF: after
        ``on_eof()`` it waits in the hold until Ctrl+C/SIGTERM
        (``on_interrupt()``) or until the hold returns.
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
        """--listen: the input is gone but reception goes on until Ctrl+C/SIGTERM."""
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
        """Hook after ``run`` returns; the console has nothing buffered."""


# ---------------------------------------------------------------------------
# Full-screen dashboard (prompt_toolkit, imported lazily)
# ---------------------------------------------------------------------------
EVENT_HISTORY = 500
REDRAW_INTERVAL_SEC = 0.1       # min_redraw_interval and refresh_interval: 10 Hz
LINK_RATE_WINDOW_SEC = 1.0      # packets/s averaged over at least this long
_FIXED_ROWS = 3                 # event title + separator + input line
_MIN_EVENT_ROWS = 1
_PAD_HINTS = "X 시동  △ 복구  R2/L2 스로틀  ? 도움말 "
_LISTEN_HINTS = "? 도움말 "
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
    """Fixed status panels, an event list and a command line (spec section 6).

    ``event``/``transition``/``alert`` append a time-stamped entry and ask for
    a redraw; ``periodic`` is ignored (the panels show those values);
    ``update`` swaps in the latest snapshot. ``Application`` coalesces the
    redraw requests (``min_redraw_interval``) and redraws on its own every
    ``refresh_interval`` so the "수신 없음 N.Ns" age keeps moving.

    Once the dashboard is gone (``after_run`` or a failed start) every output
    method forwards to a ``ConsoleUI``, so later lines reach the terminal.
    """

    def __init__(self, help_lines=(), *, input=None, output=None, clock=time.monotonic,
                 snapshot_provider=None, input_end_hold=None, fault_handler=None):
        # prompt_toolkit (and the dashboard view, which needs it) load here.
        from ground import dashboard_view

        self._view = dashboard_view
        self._snapshot_provider = snapshot_provider
        self._input_end_hold = input_end_hold
        self._fault_handler = fault_handler
        self._fault = None   # first internal dashboard error (see _report_fault)
        self._help_lines = list(help_lines)
        self._input = input
        self._output = output
        self._clock = clock
        self._lock = threading.Lock()        # leaf lock: events, snapshot, rate
        self._print_lock = threading.Lock()  # keeps after_run output in order
        self._events = collections.deque(maxlen=EVENT_HISTORY)
        self._snap = {}
        self._packets = 0
        self._rate = dashboard_view.LinkRateTracker()
        self._rate_time = None
        self._link_hz = None
        self._console = None   # ConsoleUI once the dashboard is gone
        self._ended = False    # a safety callback (on_interrupt/on_eof) has run
        self._app = None
        self._textarea = None
        self._panel_cache = None    # (render_counter, rows) for one frame
        self._drawn_panel = []
        self._drawn_events = []

    # --- output methods (any thread; never block, never raise) -------------
    def event(self, text):
        self._add("event", "event", text)

    def transition(self, text):
        self._add("transition", "transition", text)

    def alert(self, text):
        self._add("alert", "alert", text)

    def periodic(self, text):
        """[STATUS]/[DIAG]/[AUTO-LAND]/[AXIS]/[TH]: the panels show these values."""
        console = self._console
        if console is not None:
            with self._print_lock:
                console.periodic(text)

    def update(self, snap):
        """Latest station snapshot; called once per telemetry packet, valid or malformed."""
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
        """Station ``snapshot`` function, pulled on every redraw (event-loop thread)."""
        self._snapshot_provider = provider

    def set_fault_handler(self, handler):
        """Station callback ``handler(error)`` for an internal dashboard error.

        Called once, before any diagnostic output, from the thread that saw
        the error. It must make the station safe without treating the error
        as an operator stop (the station latches a fault and stops RC).
        """
        self._fault_handler = handler

    def _report_fault(self, error):
        """First internal error: the station's fault handler, exactly once; never raises."""
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
        """Refresh tick (event-loop thread): read the station state itself.

        Local safety changes (disarm, RC stop on link loss, a worker fault)
        happen without a datagram, so no update() follows them. The provider
        takes telem_lock; it is called here with no UI lock held, and never
        from an output method (their callers may hold telem_lock).
        """
        provider = self._snapshot_provider
        if provider is None:
            return
        try:
            snap = provider()
        except Exception:
            return  # keep the last snapshot; the panels show its age
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
            app.invalidate()  # thread safe; returns at once

    # --- state copies for drawing and tests --------------------------------
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

    # --- drawing (event-loop thread only) ----------------------------------
    def _size(self):
        size = self._app.output.get_size()
        return max(1, size.columns), max(1, size.rows)

    def _panel_rows(self):
        """Top bar + panels for this frame; cached per render."""
        app = self._app
        counter = getattr(app, "render_counter", None)
        cached = self._panel_cache
        if cached is not None and cached[0] == counter:
            return cached[1]
        self._pull_snapshot()
        try:
            width, height = self._size()
            # compact layout is decided on the whole terminal (80x24 rule);
            # the event title, one event row and the input line stay visible.
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
            rows = [
                [("class:event-time", f"{stamp} "), (style, text)]
                for stamp, text, style in entries
            ]
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

    # --- the application ---------------------------------------------------
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
            return False  # clear the line (it stays in the history)

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
        """Run the first safety callback (on_interrupt/on_eof) exactly once."""
        if not self._ended:
            self._ended = True
            ConsoleUI._call_safely(name, callback)

    def _end(self, app, name, callback):
        """Ctrl+C / Ctrl+D: the safety callback first, then leave the app."""
        self._run_safety(name, callback)
        try:
            app.exit()
        except Exception:
            pass  # already exiting

    def run(self, input_handler, on_interrupt, on_eof):
        """Run the dashboard until Ctrl+C / Ctrl+D; never raises.

        Same contract as ``ConsoleUI.run``: Ctrl+C -> ``on_interrupt()``,
        Ctrl+D -> ``on_eof()``; any other end of the input (closed input,
        an error inside the application) runs ``on_eof()`` first and then
        reports. A command that raises is reported as an event. A Ctrl+C
        that arrives while the dashboard is still being built (or after it
        has let go of the keys) also runs ``on_interrupt()``. An internal
        error (an exception in the event loop, a failing redraw) is not a
        stop request: the fault handler runs first, then the run continues
        as a line console until Ctrl+C/Ctrl+D.
        """
        try:
            self._run(input_handler, on_interrupt, on_eof)
        except KeyboardInterrupt:
            # 키 바인딩이 Ctrl+C를 받기 전(대시보드 구성 중·시작 직전) 또는 그 뒤
            self._run_safety("on_interrupt", on_interrupt)
        except BaseException as error:  # noqa: BLE001 - run() must not raise
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
        except BaseException as caught:  # noqa: BLE001 - run() must not raise
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
        """pre_run (inside the running loop): route loop exceptions to the fault path."""
        import asyncio

        asyncio.get_running_loop().set_exception_handler(
            lambda _loop, context: self._on_loop_exception(app, context)
        )

    def _on_loop_exception(self, app, context):
        """Event-loop exception: the station's fault handler first, then leave the app.

        Never raises. Not an operator stop: no on_interrupt/on_eof here.
        """
        try:
            exception = context.get("exception")
            if exception is None:
                exception = RuntimeError(str(context.get("message", "event loop error")))
            self._report_fault(exception)
            try:
                app.exit()
            except Exception:
                pass  # already exiting
        except Exception as error:
            _report_ui_failure("event loop", error)

    def _continue_on_console(self, error, input_handler, on_interrupt, on_eof):
        """After an internal error: diagnostic, then the line console until Ctrl+C/Ctrl+D."""
        console = self._switch_to_console()
        console.event(
            f"[WARN] 대시보드 오류: {type(error).__name__}: {error} - 줄 출력으로 계속"
        )
        console.run(input_handler, on_interrupt, on_eof)

    def interrupt(self):
        """Ctrl+C request from a signal handler (SIGTERM); never raises, never blocks.

        Returns True when the running dashboard takes the request: it is
        handed to the event loop as the SIGINT key, so the usual Ctrl+C
        binding runs the safety callback and leaves the app. Returns False
        when no dashboard loop is running; the caller then raises
        KeyboardInterrupt, which ``run`` turns into ``on_interrupt()``.
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
        """Dashboard closed: print the event list; later output goes to a ConsoleUI."""
        console = self._console
        if console is not None:
            console.after_run()
            return
        self._switch_to_console()

    def _switch_to_console(self):
        """Print the buffered event list, then forward every output method."""
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
    """prompt_toolkit input wrapper: calls ``on_closed`` when ``closed`` is first seen True.

    prompt_toolkit checks ``input.closed`` right after reading the keys and,
    when it is True, ends the app with EOFError after one last redraw.
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
    """Dashboard on a terminal; ``ConsoleUI`` for --plain, pipes and no prompt_toolkit.

    ``snapshot_provider`` is pulled by the dashboard on its redraw;
    ``input_end_hold`` keeps a run going after the input ends (--listen).
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
                           input_end_hold=input_end_hold)  # fault handler: main() sets it
    except (ImportError, ValueError):
        console = ConsoleUI(input_end_hold=input_end_hold)
        console.event("[WARN] prompt_toolkit 없음 - 줄 출력으로 실행")
        return console
