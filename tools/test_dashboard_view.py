"""Tests for the dashboard view model (snapshot -> styled screen lines)."""

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
TOOLS = REPO_ROOT / "tools"
for path in (SCRIPTS, TOOLS):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from prompt_toolkit.utils import get_cwidth  # noqa: E402

from ground.dashboard_view import LinkRateTracker, render  # noqa: E402
from ground.telemetry_schema import parse_telemetry_packet  # noqa: E402
from test_telemetry_schema import packet  # noqa: E402

NOW = 1000.0
STYLES = {
    "class:armed", "class:disarmed", "class:alert", "class:warn",
    "class:dim", "class:ok", "class:value", "",
}


def sample75(**overrides):
    sample = parse_telemetry_packet(packet(75))
    sample.update(overrides)
    return sample


def sample91(**overrides):
    sample = parse_telemetry_packet(packet(91))
    sample.update(overrides)
    return sample


def snap(sample, **overrides):
    base = {
        "mode": "pad",
        "arm_state": "armed",
        "is_armed": True,
        "is_streaming": True,
        "current_throttle": 1480,
        "trim_roll": 0.5,
        "trim_pitch": -1.0,
        "telem_total_pkts": 1000,
        "telem_dropped_pkts": 2,
        "bad_packet_count": 0,
        "last_telem_time": NOW - 0.05,
        "log_path": Path("/x/logs/flight_log_2026.csv"),
        "raw_log_path": Path("/x/logs/raw.bin"),
        "raw_capture_enabled": True,
        "raw_batch_count": 120,
        "raw_last_dropped": 0,
        "raw_last_receive_time": NOW - 0.1,
        "mag_enabled_choice": True,
        "resume_state": "idle",
        "last_sample": sample,
        "worker_fault": None,
        "armed_since": NOW - 133.0,
        "link_hz": 20.0,
    }
    base.update(overrides)
    return base


def flatten(lines):
    return "\n".join("".join(text for _, text in line) for line in lines)


class RenderTest(unittest.TestCase):
    def test_all_panels_render_for_flight_firmware_sample(self):
        text = flatten(render(snap(sample75()), 100, 30, NOW))
        for label in ("자세", "목표", "고도·추력", "모터", "믹서", "트림"):
            self.assertIn(label, text)
        self.assertIn("M1 FL", text)

    def test_missing_fields_show_dash(self):
        s = sample75()
        s["Hover_Est"] = None
        self.assertRegex(flatten(render(snap(s), 100, 30, NOW)), r"Hover_Est\s+-")

    def test_flow_panel_only_with_flow_fields(self):
        self.assertNotIn("PosHold", flatten(render(snap(sample75()), 100, 30, NOW)))
        t = flatten(render(snap(sample91(PosHold_State=1, Height_M=0.42)), 100, 30, NOW))
        self.assertIn("PosHold ACTIVE", t)
        self.assertIn("0.42", t)

    def test_probe_fields_hidden_when_absent(self):
        s = sample75()
        s["Failsafe_Probe_State"] = None
        self.assertNotIn("Probe", flatten(render(snap(s), 100, 30, NOW)))

    def test_descending_failsafe_is_alert_style(self):
        lines = render(snap(sample75(Failsafe_Phase=1)), 100, 30, NOW)
        self.assertTrue(any(
            st == "class:alert" and "DESCENDING" in tx for ln in lines for st, tx in ln
        ))

    def test_stale_link_dims_values_and_shows_age(self):
        lines = render(snap(sample75(), last_telem_time=NOW - 3.0), 100, 30, NOW)
        self.assertIn("수신 없음 3.0s", flatten(lines[:1]))
        value_styles = {st for ln in lines[2:8] for st, _ in ln}
        self.assertNotIn("class:value", value_styles)
        self.assertIn("class:dim", value_styles)

    def test_top_bar_alerts(self):
        top = flatten(render(
            snap(sample75(Fault_RC=1, Fault_Critical=1), worker_fault="telemetry"),
            100, 30, NOW)[:1])
        for word in ("RC", "치명", "스레드"):
            self.assertIn(word, top)

    def test_bad_packet_count_in_top_bar(self):
        top = flatten(render(snap(sample75(), bad_packet_count=7), 100, 30, NOW)[:1])
        self.assertIn("불량 7", top)

    def test_arm_mismatch_alert(self):
        top = flatten(render(snap(sample75(Armed=0)), 100, 30, NOW)[:1])
        self.assertIn("시동 불일치", top)

    def test_top_bar_state_flight_time_and_log(self):
        top = flatten(render(snap(sample75()), 120, 30, NOW)[:1])
        self.assertIn("ARMED", top)
        self.assertIn("02:13", top)
        self.assertIn("flight_log_2026.csv", top)
        self.assertIn("드롭 0.2%", top)
        self.assertIn("20.0 Hz", top)
        disarmed = render(snap(sample75(), is_armed=False, arm_state="idle",
                               armed_since=None), 120, 30, NOW)[0]
        self.assertIn("DISARMED", flatten([disarmed]))
        self.assertTrue(any(st == "class:disarmed" for st, _ in disarmed))

    def test_no_telemetry_yet(self):
        lines = render(snap(None, last_telem_time=0.0), 100, 30, NOW)
        self.assertIn("수신 대기", flatten(lines[:1]))
        self.assertIn("자세", flatten(lines))

    def test_small_terminal_collapses_motor_and_mixer_rows(self):
        t = flatten(render(snap(sample75()), 79, 23, NOW))
        self.assertNotIn("M1 FL", t)
        self.assertIn("자세", t)

    def test_lines_fit_width_with_korean_labels(self):
        for w in (60, 80, 120):
            for line in render(snap(sample91()), w, 30, NOW):
                self.assertLessEqual(get_cwidth("".join(t for _, t in line)), w)

    def test_height_limits_rows(self):
        self.assertLessEqual(len(render(snap(sample91()), 100, 5, NOW)), 5)

    def test_styles_are_known(self):
        for line in render(snap(sample91(Failsafe_Phase=1)), 100, 30, NOW):
            for st, _ in line:
                self.assertIn(st, STYLES)

    def test_listen_mode_hides_pad_throttle(self):
        pad = flatten(render(snap(sample75()), 100, 30, NOW))
        listen = flatten(render(snap(sample75(), mode="listen"), 100, 30, NOW))
        self.assertIn("패드", pad)
        self.assertNotIn("패드", listen)

    def test_trim_mismatch_marked(self):
        s = sample75(Trim_Roll=0.5, Trim_Pitch=-1.0)
        ok = flatten(render(snap(s), 100, 30, NOW))
        bad = flatten(render(snap(s, trim_roll=2.0), 100, 30, NOW))
        self.assertIn("+0.5/-1.0", ok)
        self.assertNotIn("+0.5/-1.0!", ok)
        self.assertIn("+0.5/-1.0!", bad)

    def test_invalid_height_sentinel_shows_dash(self):
        t = flatten(render(snap(sample91(Height_M=-1.0)), 100, 30, NOW))
        self.assertNotIn("-1.00", t)
        self.assertRegex(t, r"H -\s")
        t = flatten(render(snap(sample91(Height_M=float("nan"))), 100, 30, NOW))
        self.assertNotIn("nan", t)

    def test_cw_ccw_uses_the_console_definition(self):
        yaw = sample75(Motor_M1=1400, Motor_M2=1400, Motor_M3=1600, Motor_M4=1600)
        self.assertIn("CW-CCW -400", flatten(render(snap(yaw), 100, 30, NOW)))
        pitch = sample75(Motor_M1=1400, Motor_M2=1600, Motor_M3=1400, Motor_M4=1600)
        self.assertIn("CW-CCW +0", flatten(render(snap(pitch), 100, 30, NOW)))

    def test_formatters_import_without_prompt_toolkit(self):
        import subprocess
        code = (
            "import sys; sys.modules['prompt_toolkit'] = None; "
            "sys.path.insert(0, %r); "
            "import ground.status_format as f; "
            "assert f._format_optional_int(3) == '3'"
        ) % str(SCRIPTS)
        result = subprocess.run([sys.executable, "-I", "-c", code],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_render_does_not_mutate_snapshot(self):
        sn = snap(sample91())
        before = repr(sn)
        render(sn, 100, 30, NOW)
        self.assertEqual(before, repr(sn))


class LinkRateTrackerTest(unittest.TestCase):
    def test_rate_from_counter_delta(self):
        tr = LinkRateTracker()
        self.assertIsNone(tr.update(0.0, 0))
        self.assertAlmostEqual(tr.update(1.0, 20), 20.0)
        self.assertAlmostEqual(tr.update(2.0, 40), 20.0)

    def test_counter_reset_gives_none(self):
        tr = LinkRateTracker()
        tr.update(0.0, 100)
        self.assertIsNone(tr.update(1.0, 5))


if __name__ == "__main__":
    unittest.main()
