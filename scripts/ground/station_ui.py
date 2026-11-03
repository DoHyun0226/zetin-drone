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
``snapshot()``), never wait for a redraw, and never raise.
"""

import collections
import sys
import threading
import time


def _report_ui_failure(method, error):
    """Best-effort one-line note on stderr; never raises."""
    try:
        sys.stderr.write(f"[ConsoleUI] {method} 출력 실패: {error!r}\n")
    except Exception:
        pass


class ConsoleUI:
    """Plain line output: the station's original ``print`` behaviour."""

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
        continues.
        """
        while True:
            try:
                msg = input()
            except KeyboardInterrupt:
                self._call_safely("on_interrupt", on_interrupt)
                return
            except EOFError:
                self._call_safely("on_eof", on_eof)
                return
            except Exception as error:
                # 안전 경로(조건부 disarm)를 먼저 돌리고, 알림은 그 뒤에 낸다.
                # 출력이 막혀 있어도 disarm이 늦어지지 않게 한다.
                self._call_safely("on_eof", on_eof)
                self.event(
                    f"[STDIN] 입력 오류: {type(error).__name__}: {error} "
                    "- 입력 종료로 처리"
                )
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
    "dim": "fg:ansibrightblack",
    "ok": "fg:ansigreen",
    "value": "",
    "event-time": "fg:ansibrightblack",
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
    """

    def __init__(self, help_lines=(), *, input=None, output=None, clock=time.monotonic):
        # prompt_toolkit (and the dashboard view, which needs it) load here.
        from ground import dashboard_view

        self._view = dashboard_view
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
        self._finished = False
        self._ended = False
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

    def update(self, snap):
        """Latest station snapshot; called once per telemetry packet."""
        try:
            now = self._clock()
            with self._lock:
                self._snap = snap
                self._packets += 1
                if (self._rate_time is None
                        or now - self._rate_time >= LINK_RATE_WINDOW_SEC):
                    self._link_hz = self._rate.update(now, self._packets)
                    self._rate_time = now
            self._invalidate()
        except Exception as error:
            _report_ui_failure("update", error)

    def _add(self, method, kind, text):
        try:
            lines = [line for line in str(text).split("\n") if line.strip()]
            stamp = time.strftime("%H:%M:%S")
            with self._lock:
                finished = self._finished
                if not finished:
                    for line in lines:
                        self._events.append((stamp, line, _event_style(kind, line)))
            if finished:
                # 대시보드가 끝난 뒤(로그 저장·raw 변환): 일반 출력으로 바로 낸다.
                with self._print_lock:
                    print(text)
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
            listen = self._snap.get("mode") == "listen"
        return [("class:dim", _LISTEN_HINTS if listen else _PAD_HINTS)]

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

        @bindings.add("c-d", eager=True)
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

        app = Application(
            layout=layout,
            key_bindings=bindings,
            style=Style.from_dict(_STYLE_RULES),
            full_screen=True,
            min_redraw_interval=REDRAW_INTERVAL_SEC,
            refresh_interval=REDRAW_INTERVAL_SEC,
            input=self._input,
            output=self._output,
        )
        app_ref = [app]
        return app

    def _end(self, app, name, callback):
        """Ctrl+C / Ctrl+D: the safety callback first, then leave the app."""
        if not self._ended:
            self._ended = True
            ConsoleUI._call_safely(name, callback)
        try:
            app.exit()
        except Exception:
            pass  # already exiting

    def run(self, input_handler, on_interrupt, on_eof):
        """Run the dashboard until Ctrl+C / Ctrl+D; never raises.

        Same contract as ``ConsoleUI.run``: Ctrl+C -> ``on_interrupt()``,
        Ctrl+D -> ``on_eof()``; any other end of the input (closed input,
        an error inside the application) runs ``on_eof()`` first and then
        reports. A command that raises is reported as an event.
        """
        try:
            app = self._build_app(input_handler, on_interrupt, on_eof)
        except Exception as error:
            # 대시보드를 못 띄우면 줄 출력으로 계속 조종할 수 있게 한다.
            self.after_run()
            self.event(f"[WARN] 대시보드 시작 실패: {type(error).__name__}: {error} - 줄 출력으로 실행")
            ConsoleUI().run(input_handler, on_interrupt, on_eof)
            return
        self._app = app
        error = None
        try:
            app.run()
        except KeyboardInterrupt:
            if not self._ended:
                self._ended = True
                ConsoleUI._call_safely("on_interrupt", on_interrupt)
        except BaseException as caught:  # noqa: BLE001 - run() must not raise
            error = caught
        finally:
            self._app = None
        if not self._ended:
            # 입력이 키 없이 끝났거나 앱이 오류로 끝났다: EOF와 같이 조건부 disarm부터.
            self._ended = True
            ConsoleUI._call_safely("on_eof", on_eof)
        if error is not None:
            self.event(
                f"[STDIN] 입력 오류: {type(error).__name__}: {error} - 입력 종료로 처리"
            )

    def after_run(self):
        """Dashboard closed: print the event list, then print later events directly."""
        try:
            with self._print_lock:
                with self._lock:
                    entries = list(self._events)
                    self._events.clear()
                    self._finished = True
                for stamp, text, _style in entries:
                    print(f"{stamp} {text}")
        except Exception as error:
            _report_ui_failure("after_run", error)


def make_ui(args, isatty=None, *, help_lines=()):
    """Dashboard on a terminal; ``ConsoleUI`` for --plain, pipes and no prompt_toolkit."""
    if getattr(args, "plain", False):
        return ConsoleUI()
    try:
        if isatty is not None:
            on_terminal = bool(isatty())
        else:
            on_terminal = sys.stdout.isatty() and sys.stdin.isatty()
    except Exception:
        on_terminal = False
    if not on_terminal:
        return ConsoleUI()
    try:
        import prompt_toolkit  # noqa: F401

        return DashboardUI(help_lines=help_lines)
    except ImportError:
        console = ConsoleUI()
        console.event("[WARN] prompt_toolkit 없음 - 줄 출력으로 실행")
        return console
