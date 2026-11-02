"""Output layer of the control station.

``control_station`` never calls ``print`` directly; it hands every line to a
UI object. ``ConsoleUI`` reproduces the plain line output the station has
always had, character for character.

Contract for every UI: output methods (``event``, ``periodic``,
``transition``, ``alert``, ``update``, ``after_run``) never raise. They are
called from the safety paths (disarm, stop of RC streaming, worker-fault
handling) after the state change, and a broken terminal must not abort those
paths. Failures are swallowed with at most one line on stderr.
"""

import sys


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
        """The stdin command loop.

        Ctrl+C -> ``on_interrupt()``, Ctrl+D (EOF) -> ``on_eof()``, then
        return. Other exceptions propagate exactly as the old top-level loop
        let them (``main``'s ``finally`` still closes the logs).
        """
        while True:
            try:
                msg = input()
                if msg:
                    input_handler(msg)
            except KeyboardInterrupt:
                on_interrupt()
                break
            except EOFError:
                on_eof()
                break

    def after_run(self):
        """Hook after ``run`` returns; the console has nothing buffered."""
