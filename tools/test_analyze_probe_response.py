import contextlib
import csv
import importlib
import io
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from ground.telemetry_schema import CSV_FIELDS  # noqa: E402


def analyzer_module():
    return importlib.import_module("analyze_probe_response")


class AnalyzeProbeResponseTests(unittest.TestCase):
    def write_csv(self, directory, name, samples, fields=CSV_FIELDS):
        path = Path(directory) / name
        with path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            for sample in samples:
                row = dict.fromkeys(fields, "")
                row.update(sample)
                writer.writerow(row)
        return path

    @staticmethod
    def event_samples(values, hover_est=1340.0, blocked_indexes=()):
        samples = [
            {
                "Timestamp": "12:00:00.000",
                "Throttle": 1280,
                "Failsafe_Phase": 1,
                "Hover_Est": hover_est,
                "Failsafe_Probe_State": 0,
                "Failsafe_Probe_NoResponse": 0,
                "Failsafe_Probe_Response_G": 0.0,
            }
        ]
        no_response = 0
        for index, value in enumerate(values):
            samples.append(
                {
                    "Timestamp": f"12:00:{index:02d}.050",
                    "Throttle": 1280,
                    "Failsafe_Phase": 1,
                    "Hover_Est": hover_est,
                    "Failsafe_Probe_State": 1,
                    "Failsafe_Probe_NoResponse": no_response,
                    "Failsafe_Probe_Response_G": samples[-1][
                        "Failsafe_Probe_Response_G"
                    ],
                }
            )
            blocked = index in blocked_indexes
            if not blocked and value <= 0.06:
                no_response += 1
            elif not blocked:
                no_response = 0
            samples.append(
                {
                    "Timestamp": f"12:00:{index:02d}.100",
                    "Throttle": 1280,
                    "Failsafe_Phase": 1,
                    "Hover_Est": hover_est,
                    "Failsafe_Probe_State": 4 if blocked else 2,
                    "Failsafe_Probe_NoResponse": no_response,
                    "Failsafe_Probe_Response_G": value,
                }
            )
            samples.append(
                {
                    "Timestamp": f"12:00:{index:02d}.150",
                    "Throttle": 1280,
                    "Failsafe_Phase": 1,
                    "Hover_Est": hover_est,
                    "Failsafe_Probe_State": 0,
                    "Failsafe_Probe_NoResponse": no_response,
                    "Failsafe_Probe_Response_G": value,
                }
            )
        return samples

    @staticmethod
    def repeated_descent_samples(values, hover_est=1340.0):
        """Model repeated start → auto-land → one probe → landed cycles."""

        samples = [
            {
                "Timestamp": "11:59:59.950",
                "Throttle": 1000,
                "Failsafe_Phase": 2,
                "Hover_Est": hover_est,
                "Failsafe_Probe_State": 0,
                "Failsafe_Probe_NoResponse": 0,
                "Failsafe_Probe_Response_G": 0.2,
            }
        ]
        for index, value in enumerate(values):
            prefix = f"12:{index:02d}"
            samples.extend(
                [
                    {
                        "Timestamp": f"{prefix}:00.000",
                        "Throttle": 1000,
                        "Failsafe_Phase": 0,
                        "Hover_Est": hover_est,
                        "Failsafe_Probe_State": 0,
                        "Failsafe_Probe_NoResponse": 0,
                        "Failsafe_Probe_Response_G": 0.0,
                    },
                    {
                        "Timestamp": f"{prefix}:00.050",
                        "Throttle": 1280,
                        "Failsafe_Phase": 1,
                        "Hover_Est": hover_est,
                        "Failsafe_Probe_State": 0,
                        "Failsafe_Probe_NoResponse": 0,
                        "Failsafe_Probe_Response_G": 0.0,
                    },
                    {
                        "Timestamp": f"{prefix}:00.100",
                        "Throttle": 1240,
                        "Failsafe_Phase": 1,
                        "Hover_Est": hover_est,
                        "Failsafe_Probe_State": 1,
                        "Failsafe_Probe_NoResponse": 0,
                        "Failsafe_Probe_Response_G": 0.0,
                    },
                    {
                        "Timestamp": f"{prefix}:00.150",
                        "Throttle": 1280,
                        "Failsafe_Phase": 1,
                        "Hover_Est": hover_est,
                        "Failsafe_Probe_State": 2,
                        "Failsafe_Probe_NoResponse": int(value <= 0.06),
                        "Failsafe_Probe_Response_G": value,
                    },
                    {
                        "Timestamp": f"{prefix}:00.200",
                        "Throttle": 1280,
                        "Failsafe_Phase": 1,
                        "Hover_Est": hover_est,
                        "Failsafe_Probe_State": 0,
                        "Failsafe_Probe_NoResponse": int(value <= 0.06),
                        "Failsafe_Probe_Response_G": value,
                    },
                    {
                        "Timestamp": f"{prefix}:00.250",
                        "Throttle": 1280,
                        "Failsafe_Phase": 1,
                        "Hover_Est": hover_est,
                        "Failsafe_Probe_State": 0,
                        "Failsafe_Probe_NoResponse": int(value <= 0.06),
                        "Failsafe_Probe_Response_G": value,
                    },
                    {
                        "Timestamp": f"{prefix}:00.300",
                        "Throttle": 1000,
                        "Failsafe_Phase": 2,
                        "Hover_Est": hover_est,
                        "Failsafe_Probe_State": 0,
                        "Failsafe_Probe_NoResponse": int(value <= 0.06),
                        "Failsafe_Probe_Response_G": value,
                    },
                ]
            )
        return samples

    def threshold(self):
        """픽스처를 펌웨어 임계에 상대적으로 만든다.

        예전에는 0.06 기준 절대값을 박아뒀는데, 2026-08-01 에 임계를 0.03 으로
        내리자 '지면' 픽스처가 임계 위로 올라가 버렸다. 배수로 두면 임계를
        바꿔도 시나리오의 의미(지면<임계<공중)가 유지된다.
        """
        analyzer = analyzer_module()
        return analyzer.read_firmware_float_constant(
            "FS_PROBE_RESPONSE_G", analyzer.FIRMWARE_PATH
        )

    @staticmethod
    def range_values(start, stop, count=20):
        return [
            start + (stop - start) * index / (count - 1)
            for index in range(count)
        ]

    def run_main(self, arguments):
        output = io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            try:
                return_code = analyzer_module().main(
                    [str(arg) for arg in arguments]
                )
            except SystemExit as exc:
                return_code = exc.code
        return return_code, output.getvalue()

    def test_extracts_each_probe_once_and_marks_fallback_evidence(self):
        samples = [
            {
                "Timestamp": "12:00:00.000",
                "Throttle": 1280,
                "Failsafe_Phase": 1,
                "Hover_Est": 1340.0,
                "Failsafe_Probe_State": 0,
                "Failsafe_Probe_NoResponse": 0,
                "Failsafe_Probe_Response_G": 0.0,
            },
            {
                "Timestamp": "12:00:00.050",
                "Throttle": 1240,
                "Failsafe_Phase": 1,
                "Hover_Est": 1340.0,
                "Failsafe_Probe_State": 1,
                "Failsafe_Probe_NoResponse": 0,
                "Failsafe_Probe_Response_G": 0.0,
            },
            {
                "Timestamp": "12:00:00.100",
                "Throttle": 1280,
                "Failsafe_Phase": 1,
                "Hover_Est": 1340.0,
                "Failsafe_Probe_State": 2,
                "Failsafe_Probe_NoResponse": 1,
                "Failsafe_Probe_Response_G": 0.04,
            },
            {
                "Timestamp": "12:00:00.150",
                "Throttle": 1240,
                "Failsafe_Phase": 1,
                "Hover_Est": 1340.0,
                "Failsafe_Probe_State": 1,
                "Failsafe_Probe_NoResponse": 1,
                "Failsafe_Probe_Response_G": 0.04,
            },
            {
                "Timestamp": "12:00:00.200",
                "Throttle": 1280,
                "Failsafe_Phase": 1,
                "Hover_Est": 1340.0,
                "Failsafe_Probe_State": 0,
                "Failsafe_Probe_NoResponse": 0,
                "Failsafe_Probe_Response_G": 0.11,
            },
        ]

        with tempfile.TemporaryDirectory() as tmp_dir:
            log_path = self.write_csv(tmp_dir, "events.csv", samples)
            events = analyzer_module().extract_probe_events(log_path, "air")

        self.assertEqual(2, len(events))
        self.assertEqual(["state-transition", "response-change"], [
            event.evidence for event in events
        ])
        self.assertEqual("12:00:00.100", events[0].timestamp)
        self.assertAlmostEqual(0.11, events[1].response_g)
        self.assertAlmostEqual(1340.0, events[1].hover_est)
        self.assertEqual(0, events[1].no_response)
        self.assertEqual(1, events[1].failsafe_phase)
        self.assertEqual(1280, events[1].throttle)

    def test_response_change_is_fallback_even_when_decision_states_are_missed(self):
        samples = [
            {
                "Timestamp": "12:00:00.000",
                "Throttle": 1280,
                "Failsafe_Phase": 1,
                "Hover_Est": 1340.0,
                "Failsafe_Probe_State": 0,
                "Failsafe_Probe_NoResponse": 0,
                "Failsafe_Probe_Response_G": 0.0,
            },
            {
                "Timestamp": "12:00:00.050",
                "Throttle": 1280,
                "Failsafe_Phase": 1,
                "Hover_Est": 1340.0,
                "Failsafe_Probe_State": 0,
                "Failsafe_Probe_NoResponse": 1,
                "Failsafe_Probe_Response_G": 0.045,
            },
        ]

        with tempfile.TemporaryDirectory() as tmp_dir:
            log_path = self.write_csv(tmp_dir, "missed-states.csv", samples)
            events = analyzer_module().extract_probe_events(log_path, "ground")

        self.assertEqual(1, len(events))
        self.assertEqual("response-change", events[0].evidence)
        self.assertAlmostEqual(0.045, events[0].response_g)

    def test_repeated_descents_exclude_zero_resets_and_report_count(self):
        values = [0.04, 0.11, 0.05]
        samples = self.repeated_descent_samples(values)

        with tempfile.TemporaryDirectory() as tmp_dir:
            log_path = self.write_csv(tmp_dir, "descents.csv", samples)
            events = analyzer_module().extract_probe_events(log_path, "ground")
            return_code, output = self.run_main(
                [log_path, "--label", "ground"]
            )

        self.assertEqual(values, [event.response_g for event in events])
        self.assertEqual(0, return_code)
        self.assertIn("프로브 이벤트 3개", output)
        self.assertIn("진입 리셋 3개 제외", output)
        self.assertNotIn("0.0000g", output)

    def test_zero_response_state_transition_is_excluded(self):
        samples = [
            {
                "Timestamp": "12:00:00.000",
                "Throttle": 1280,
                "Failsafe_Phase": 1,
                "Hover_Est": 1340.0,
                "Failsafe_Probe_State": 1,
                "Failsafe_Probe_NoResponse": 0,
                "Failsafe_Probe_Response_G": 0.04,
            },
            {
                "Timestamp": "12:00:00.050",
                "Throttle": 1280,
                "Failsafe_Phase": 1,
                "Hover_Est": 1340.0,
                "Failsafe_Probe_State": 2,
                "Failsafe_Probe_NoResponse": 0,
                "Failsafe_Probe_Response_G": 0.0,
            },
        ]

        with tempfile.TemporaryDirectory() as tmp_dir:
            log_path = self.write_csv(tmp_dir, "zero-transition.csv", samples)
            events = analyzer_module().extract_probe_events(log_path, "ground")

        self.assertEqual([], events)

    def test_blocked_event_is_counted_but_excluded_from_distribution(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            log_path = self.write_csv(
                tmp_dir,
                "blocked.csv",
                self.event_samples([0.04, 0.02, 0.11], blocked_indexes=(1,)),
            )
            events = analyzer_module().extract_probe_events(log_path, "ground")
            summary = analyzer_module().summarize_events(events)

        self.assertEqual(3, len(events))
        self.assertEqual(1, summary.blocked_count)
        self.assertEqual([0.04, 0.11], summary.values)

    def test_mixed_hover_est_prints_large_warning_and_separate_groups(self):
        samples = self.event_samples([0.04], hover_est=1340.0)
        samples.extend(self.event_samples([0.05], hover_est=1342.1))
        with tempfile.TemporaryDirectory() as tmp_dir:
            log_path = self.write_csv(tmp_dir, "mixed.csv", samples)
            return_code, output = self.run_main([log_path, "--label", "ground"])

        self.assertEqual(0, return_code)
        self.assertIn("경고", output)
        self.assertIn("Hover_Est 혼합", output)
        self.assertIn("1340.0µs", output)
        self.assertIn("1342.1µs", output)

    def test_clear_pass_when_air_minimum_has_one_point_five_margin(self):
        t = self.threshold()
        with tempfile.TemporaryDirectory() as tmp_dir:
            ground = self.write_csv(
                tmp_dir,
                "ground.csv",
                self.repeated_descent_samples(
                    self.range_values(0.6333 * t, 0.85 * t)
                ),
            )
            air = self.write_csv(
                tmp_dir,
                "air.csv",
                self.repeated_descent_samples(
                    self.range_values(1.8 * t, 2.1667 * t)
                ),
            )
            return_code, output = self.run_main(
                [ground, air, "--label", "ground", "--label", "air"]
            )

        self.assertEqual(0, return_code)
        self.assertIn("지면 최대 < 임계: 통과", output)
        self.assertIn("공중 최소 > 임계: 통과", output)
        self.assertIn("공중 최소 ≥ 1.5 × 임계: 통과", output)
        self.assertIn("최종 판정: 통과", output)

    def test_air_between_one_and_one_point_five_is_margin_short_not_pass(self):
        t = self.threshold()
        with tempfile.TemporaryDirectory() as tmp_dir:
            ground = self.write_csv(
                tmp_dir,
                "ground.csv",
                self.repeated_descent_samples(
                    self.range_values(0.6333 * t, 0.85 * t)
                ),
            )
            air = self.write_csv(
                tmp_dir,
                "air.csv",
                self.repeated_descent_samples(
                    self.range_values(1.0333 * t, 1.2333 * t)
                ),
            )
            return_code, output = self.run_main(
                [ground, air, "--label", "ground", "--label", "air"]
            )

        self.assertEqual(0, return_code)
        self.assertIn("공중 최소 ≥ 1.5 × 임계: 여유 부족", output)
        self.assertIn("최종 판정: 여유 부족", output)
        self.assertNotIn("최종 판정: 통과", output)

    def test_overlap_fails_and_calculates_dip_fraction_suggestion(self):
        t = self.threshold()
        with tempfile.TemporaryDirectory() as tmp_dir:
            ground = self.write_csv(
                tmp_dir,
                "ground.csv",
                self.repeated_descent_samples(
                    self.range_values(0.6333 * t, 0.85 * t)
                ),
            )
            air = self.write_csv(
                tmp_dir,
                "air.csv",
                self.repeated_descent_samples(
                    self.range_values(0.7333 * t, 1.0167 * t)
                ),
            )
            return_code, output = self.run_main(
                [ground, air, "--label", "ground", "--label", "air"]
            )

        self.assertEqual(0, return_code)
        self.assertIn("최종 판정: 실패", output)
        self.assertIn("FS_PROBE_DIP_FRAC 제안: 0.2414", output)

    def test_csv_and_label_arguments_can_be_intermixed(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ground = self.write_csv(
                tmp_dir,
                "ground.csv",
                self.event_samples([0.04]),
            )
            air = self.write_csv(
                tmp_dir,
                "air.csv",
                self.event_samples([0.11]),
            )
            return_code, output = self.run_main(
                [ground, "--label", "ground", air, "--label", "air"]
            )

        self.assertEqual(0, return_code, output)
        self.assertIn("[ground] 요약", output)
        self.assertIn("[air] 요약", output)

    def test_reads_current_firmware_constants_and_fails_clearly_when_missing(self):
        analyzer = analyzer_module()
        import re as _re
        expected = float(_re.search(
            r"FS_PROBE_RESPONSE_G\s*=\s*([0-9.]+)f",
            analyzer.FIRMWARE_PATH.read_text(encoding="utf-8")).group(1))
        self.assertAlmostEqual(
            expected,
            analyzer.read_firmware_float_constant(
                "FS_PROBE_RESPONSE_G",
                analyzer.FIRMWARE_PATH,
            ),
        )
        self.assertAlmostEqual(
            0.118,
            analyzer.read_firmware_float_constant(
                "FS_PROBE_DIP_FRAC",
                analyzer.FIRMWARE_PATH,
            ),
        )

        with tempfile.TemporaryDirectory() as tmp_dir:
            sketch = Path(tmp_dir) / "missing.ino"
            sketch.write_text(
                "constexpr float SOME_OTHER_CONSTANT = 1.0f;\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                RuntimeError,
                "FS_PROBE_RESPONSE_G.*찾을 수 없습니다",
            ):
                analyzer.read_firmware_float_constant(
                    "FS_PROBE_RESPONSE_G",
                    sketch,
                )

    def test_legacy_csv_without_probe_fields_has_zero_events_and_exits_normally(self):
        legacy_fields = CSV_FIELDS[:11]
        with tempfile.TemporaryDirectory() as tmp_dir:
            log_path = self.write_csv(
                tmp_dir,
                "legacy.csv",
                [{"Timestamp": "12:00:00.000", "Throttle": 1000}],
                fields=legacy_fields,
            )
            return_code, output = self.run_main([log_path])

        self.assertEqual(0, return_code)
        self.assertIn("프로브 이벤트 0개", output)


if __name__ == "__main__":
    unittest.main()
