#!/usr/bin/env python3
"""Extract and judge failsafe probe responses from telemetry CSV logs."""

import argparse
import csv
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path

from ground.failsafe_telemetry import format_failsafe_phase
from ground.paths import REPO_ROOT
from ground.telemetry_schema import CSV_FIELDS


FIRMWARE_PATH = (
    REPO_ROOT
    / "firmware"
    / "flight"
    / "dual_imu_cascade_pwm"
    / "dual_imu_cascade_pwm.ino"
)
MIN_SAMPLE_COUNT = 20
HOVER_TOLERANCE_US = 1.0


@dataclass(frozen=True)
class ProbeEvent:
    source: Path
    label: str
    row_number: int
    timestamp: str
    response_g: float | None
    hover_est: float | None
    no_response: int | None
    failsafe_phase: int | None
    throttle: int | None
    probe_state: int | None
    blocked: bool
    evidence: str


@dataclass(frozen=True)
class EventSummary:
    values: list[float]
    blocked_count: int
    missing_response_count: int


@dataclass(frozen=True)
class ProbeExtraction:
    events: list[ProbeEvent]
    excluded_reset_count: int


def read_firmware_float_constant(name, firmware_path=FIRMWARE_PATH):
    """Read a finite ``const[expr] float`` value from the current sketch."""

    try:
        source = Path(firmware_path).read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(
            f"펌웨어 상수 파일을 읽을 수 없습니다: {firmware_path}: {exc}"
        ) from exc

    number = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
    match = re.search(
        rf"\b(?:const|constexpr)\s+float\s+{re.escape(name)}\s*=\s*"
        rf"({number})f?\s*;",
        source,
    )
    if match is None:
        raise RuntimeError(
            f"펌웨어에서 float 상수 {name}를 찾을 수 없습니다: {firmware_path}"
        )

    value = float(match.group(1))
    if not math.isfinite(value):
        raise RuntimeError(f"펌웨어 상수 {name}가 유한수가 아닙니다: {value}")
    return value


def _optional_float(raw, field_name, row_number):
    if raw is None or str(raw).strip() == "":
        return None
    try:
        value = float(str(raw).strip())
    except ValueError as exc:
        raise ValueError(
            f"{row_number}행 {field_name} 값이 숫자가 아닙니다: {raw!r}"
        ) from exc
    if not math.isfinite(value):
        raise ValueError(
            f"{row_number}행 {field_name} 값이 유한수가 아닙니다: {raw!r}"
        )
    return value


def _optional_int(raw, field_name, row_number):
    value = _optional_float(raw, field_name, row_number)
    if value is None:
        return None
    if not value.is_integer():
        raise ValueError(
            f"{row_number}행 {field_name} 값이 정수가 아닙니다: {raw!r}"
        )
    return int(value)


def _event_from_row(
    path,
    label,
    row,
    row_number,
    evidence,
    probe_state,
):
    return ProbeEvent(
        source=path,
        label=label,
        row_number=row_number,
        timestamp=str(row.get("Timestamp") or f"row-{row_number}").strip(),
        response_g=_optional_float(
            row.get("Failsafe_Probe_Response_G"),
            "Failsafe_Probe_Response_G",
            row_number,
        ),
        hover_est=_optional_float(
            row.get("Hover_Est"),
            "Hover_Est",
            row_number,
        ),
        no_response=_optional_int(
            row.get("Failsafe_Probe_NoResponse"),
            "Failsafe_Probe_NoResponse",
            row_number,
        ),
        failsafe_phase=_optional_int(
            row.get("Failsafe_Phase"),
            "Failsafe_Phase",
            row_number,
        ),
        throttle=_optional_int(row.get("Throttle"), "Throttle", row_number),
        probe_state=probe_state,
        blocked=probe_state == 4,
        evidence=evidence,
    )


def extract_probe_events_with_stats(csv_path, label="unlabeled"):
    """Extract one event per observed probe decision.

    A sampled DIP→EVALUATE/BLOCKED transition is authoritative. If the 1 ms
    decision state falls between 20 Hz samples, a change in the latched
    response value is the fallback evidence. Exact 0.0 responses are firmware
    resets, not probe decisions, and are counted separately.
    """

    path = Path(csv_path).expanduser().resolve()
    events = []
    excluded_reset_count = 0
    previous_state = None
    previous_response = None

    try:
        csv_file = path.open("r", newline="", encoding="utf-8-sig")
    except OSError as exc:
        raise RuntimeError(f"CSV를 읽을 수 없습니다: {path}: {exc}") from exc

    with csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            return ProbeExtraction(events, excluded_reset_count)
        normalized_fields = {
            str(field).strip(): field for field in reader.fieldnames if field is not None
        }

        for row_number, raw_row in enumerate(reader, start=2):
            row = {
                name: raw_row.get(normalized_fields[name])
                if name in normalized_fields
                else None
                for name in CSV_FIELDS
            }
            state = _optional_int(
                row.get("Failsafe_Probe_State"),
                "Failsafe_Probe_State",
                row_number,
            )
            response = _optional_float(
                row.get("Failsafe_Probe_Response_G"),
                "Failsafe_Probe_Response_G",
                row_number,
            )

            state_transition = previous_state == 1 and state in (2, 4)
            response_changed = (
                previous_response is not None
                and response is not None
                and response != previous_response
            )

            evidence = None
            if state_transition:
                evidence = "state-transition"
            elif response_changed:
                evidence = "response-change"

            if evidence is not None and response == 0.0:
                excluded_reset_count += 1
            elif evidence is not None:
                events.append(
                    _event_from_row(
                        path,
                        label,
                        row,
                        row_number,
                        evidence,
                        state,
                    )
                )

            previous_state = state
            if response is not None:
                previous_response = response

    return ProbeExtraction(events, excluded_reset_count)


def extract_probe_events(csv_path, label="unlabeled"):
    """Return probe events while preserving the original list-only API."""

    return extract_probe_events_with_stats(csv_path, label).events


def summarize_events(events):
    usable = [
        event.response_g
        for event in events
        if not event.blocked and event.response_g is not None
    ]
    return EventSummary(
        values=usable,
        blocked_count=sum(event.blocked for event in events),
        missing_response_count=sum(
            not event.blocked and event.response_g is None for event in events
        ),
    )


def _format_optional(value, suffix="", digits=None):
    if value is None:
        return "-"
    if digits is None:
        return f"{value}{suffix}"
    return f"{value:.{digits}f}{suffix}"


def print_events(events):
    if not events:
        print("프로브 이벤트 0개")
        return

    print(f"프로브 이벤트 {len(events)}개")
    print(
        "  #  label   timestamp        response    Hover_Est  no-resp  "
        "Failsafe phase       throttle  evidence"
    )
    for index, event in enumerate(events, start=1):
        blocked = " BLOCKED" if event.blocked else ""
        print(
            f"  {index:>2} {event.label:<7} {event.timestamp:<16} "
            f"{_format_optional(event.response_g, 'g', 4):>9} "
            f"{_format_optional(event.hover_est, 'µs', 1):>11} "
            f"{_format_optional(event.no_response):>7}  "
            f"{format_failsafe_phase(event.failsafe_phase):<20} "
            f"{_format_optional(event.throttle):>8}  "
            f"{event.evidence}{blocked}"
        )


def _hover_groups(events):
    known = sorted(
        (event for event in events if event.hover_est is not None),
        key=lambda event: event.hover_est,
    )
    groups = []
    for event in known:
        if not groups or (
            event.hover_est - groups[-1][0].hover_est > HOVER_TOLERANCE_US
        ):
            groups.append([event])
        else:
            groups[-1].append(event)
    unknown = [event for event in events if event.hover_est is None]
    return groups, unknown


def check_hover_consistency(events):
    known_values = [
        event.hover_est for event in events if event.hover_est is not None
    ]
    mixed = bool(known_values) and max(known_values) - min(known_values) > (
        HOVER_TOLERANCE_US
    )
    if mixed:
        print("\n" + "!" * 78)
        print(
            "!!! 경고: Hover_Est 혼합 — 1µs 초과 차이의 측정을 "
            "같은 분포로 판정할 수 없습니다 !!!"
        )
        print("!" * 78)
        groups, unknown = _hover_groups(events)
        for group in groups:
            values = [event.hover_est for event in group]
            print(
                f"  Hover_Est {min(values):.1f}~{max(values):.1f}µs: "
                f"{len(group)}개"
            )
            for event in group:
                print(
                    f"    [{event.label}] {event.source.name}:"
                    f"{event.row_number} {event.timestamp} "
                    f"{_format_optional(event.response_g, 'g', 4)} "
                    f"({event.evidence})"
                )
        if unknown:
            print(f"  Hover_Est 미상: {len(unknown)}개")
    elif events and any(event.hover_est is None for event in events):
        print(
            "\n⚠ 경고: Hover_Est가 비어 있는 이벤트가 있어 "
            "일관성을 완전히 확인할 수 없습니다."
        )
    return mixed


def print_label_summary(label, events, excluded_reset_count=0):
    summary = summarize_events(events)
    values = summary.values
    print(f"\n[{label}] 요약")
    print(
        f"  전체 이벤트 {len(events)}개, BLOCKED {summary.blocked_count}개 "
        "(응답 분포 제외)"
    )
    print(f"  진입 리셋 {excluded_reset_count}개 제외")
    if summary.missing_response_count:
        print(f"  응답값 없음 {summary.missing_response_count}개 (응답 분포 제외)")
    if not values:
        print("  n=0, 최소=-, 최대=-, 중앙값=-")
        print("  개별 값: []")
        return summary
    print(
        f"  n={len(values)}, 최소={min(values):.4f}g, "
        f"최대={max(values):.4f}g, "
        f"중앙값={statistics.median(values):.4f}g"
    )
    print("  개별 값: [" + ", ".join(f"{value:.4f}" for value in values) + "]")
    if len(values) < MIN_SAMPLE_COUNT:
        print(f"  ⚠ n<{MIN_SAMPLE_COUNT}: E-4a 최소 표본 수 미달")
    return summary


def print_distribution_judgement(
    ground_events,
    air_events,
    response_threshold,
    dip_fraction,
    hover_mixed=False,
):
    ground = summarize_events(ground_events).values
    air = summarize_events(air_events).values
    print("\n[ground/air 판정]")
    print(
        f"  펌웨어 기준: FS_PROBE_RESPONSE_G={response_threshold:.4f}g, "
        f"FS_PROBE_DIP_FRAC={dip_fraction:.4f}"
    )
    if not ground or not air:
        print("  최종 판정: 판정 불가 (지면 또는 공중 응답값 없음)")
        return "unavailable"

    ground_max = max(ground)
    air_min = min(air)
    margin_threshold = 1.5 * response_threshold
    ground_pass = ground_max < response_threshold
    air_pass = air_min > response_threshold
    margin_pass = air_min >= margin_threshold
    overlap = ground_max >= air_min

    print(
        f"  지면 최대 < 임계: {'통과' if ground_pass else '실패'} "
        f"({ground_max:.4f}g < {response_threshold:.4f}g)"
    )
    print(
        f"  공중 최소 > 임계: {'통과' if air_pass else '실패'} "
        f"({air_min:.4f}g > {response_threshold:.4f}g)"
    )
    margin_result = (
        "통과"
        if margin_pass
        else ("여유 부족" if air_pass else "실패")
    )
    print(
        "  공중 최소 ≥ 1.5 × 임계: "
        f"{margin_result} ({air_min:.4f}g ≥ {margin_threshold:.4f}g)"
    )
    print(f"  지면/공중 분포 겹침: {'예' if overlap else '아니오'}")

    if overlap:
        if air_min > 0.0:
            target_response = max(margin_threshold, ground_max)
            required_factor = target_response / air_min
            suggestion = dip_fraction * required_factor
            print(
                "  FS_PROBE_DIP_FRAC 제안: "
                f"{suggestion:.4f} 초과 "
                f"(필요 배수 {required_factor:.4f} × 현재 {dip_fraction:.4f})"
            )
            print(
                "  ⚠ 지면 응답도 딥에 비례하면 비율 증대만으로 분포 자체의 "
                "겹침은 사라지지 않으므로 재측정이 필수입니다."
            )
        else:
            print(
                "  FS_PROBE_DIP_FRAC 제안 계산 불가: 공중 최소 응답이 0g입니다."
            )

    if hover_mixed:
        print("  최종 판정: 판정 보류 (Hover_Est 혼합)")
        return "hover-mixed"
    if len(ground) < MIN_SAMPLE_COUNT or len(air) < MIN_SAMPLE_COUNT:
        print(f"  최종 판정: 판정 보류 (각 라벨 n≥{MIN_SAMPLE_COUNT} 필요)")
        return "insufficient"
    if not ground_pass or not air_pass or overlap:
        print("  최종 판정: 실패")
        return "fail"
    if not margin_pass:
        print("  최종 판정: 여유 부족")
        return "margin-short"
    print("  최종 판정: 통과")
    return "pass"


def plot_distributions(events, response_threshold):
    """Plot usable labeled responses; matplotlib stays an optional import."""

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "--plot을 사용하려면 matplotlib이 필요합니다"
        ) from exc

    labels = [
        label
        for label in ("ground", "air", "unlabeled")
        if any(event.label == label and not event.blocked for event in events)
    ]
    figure, axis = plt.subplots(figsize=(9, 5))
    for position, label in enumerate(labels):
        values = [
            event.response_g
            for event in events
            if event.label == label
            and not event.blocked
            and event.response_g is not None
        ]
        axis.scatter([position] * len(values), values, label=label, alpha=0.8)
    axis.axhline(
        response_threshold,
        color="crimson",
        linestyle="--",
        label="FS_PROBE_RESPONSE_G",
    )
    axis.axhline(
        1.5 * response_threshold,
        color="darkorange",
        linestyle=":",
        label="1.5 × threshold",
    )
    axis.set_xticks(range(len(labels)), labels)
    axis.set_ylabel("Failsafe probe response (g)")
    axis.set_title("Failsafe probe response distributions")
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend()
    figure.tight_layout()
    plt.show()


def _labels_for_paths(paths, labels):
    if not labels:
        return ["unlabeled"] * len(paths)
    if len(labels) == 1:
        return labels * len(paths)
    if len(labels) == len(paths):
        return labels
    raise RuntimeError(
        "--label은 한 번만 주어 모든 CSV에 적용하거나, "
        "CSV마다 하나씩 같은 순서로 주어야 합니다"
    )


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Stage E-4a failsafe 프로브 응답 분포 분석",
    )
    parser.add_argument("csv", nargs="+", type=Path, help="텔레메트리 CSV")
    parser.add_argument(
        "--label",
        action="append",
        choices=("ground", "air"),
        help=(
            "CSV 라벨. 한 번이면 모든 CSV에 적용하고, 여러 번이면 "
            "CSV 순서대로 대응"
        ),
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="선택적으로 matplotlib 분포 그림 표시",
    )
    return parser


def main(argv=None):
    args = build_argument_parser().parse_intermixed_args(argv)
    try:
        response_threshold = read_firmware_float_constant(
            "FS_PROBE_RESPONSE_G"
        )
        dip_fraction = read_firmware_float_constant("FS_PROBE_DIP_FRAC")
        labels = _labels_for_paths(args.csv, args.label)
        all_events = []
        excluded_resets_by_label = {}
        for path, label in zip(args.csv, labels):
            extraction = extract_probe_events_with_stats(path, label)
            file_events = extraction.events
            print(f"\n📂 {Path(path).expanduser().resolve()} [{label}]")
            print_events(file_events)
            all_events.extend(file_events)
            excluded_resets_by_label[label] = (
                excluded_resets_by_label.get(label, 0)
                + extraction.excluded_reset_count
            )

        hover_mixed = check_hover_consistency(all_events)
        summary_labels = []
        for label in labels:
            if label not in summary_labels:
                summary_labels.append(label)
        for label in summary_labels:
            print_label_summary(
                label,
                [event for event in all_events if event.label == label],
                excluded_resets_by_label[label],
            )

        if "ground" in labels and "air" in labels:
            print_distribution_judgement(
                [event for event in all_events if event.label == "ground"],
                [event for event in all_events if event.label == "air"],
                response_threshold,
                dip_fraction,
                hover_mixed,
            )

        if args.plot:
            plot_distributions(all_events, response_threshold)
    except (OSError, RuntimeError, ValueError, csv.Error) as exc:
        print(f"❌ 분석 실패: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
