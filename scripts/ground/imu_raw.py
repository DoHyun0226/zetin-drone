#!/usr/bin/env python3
"""Decode ZIMU/ZCAL datagrams appended to a binary capture into CSV.

Raw columns preserve the twelve int16 register counts. When at least one ZCAL
record is present, physical columns are also emitted in the IMU1 sensor frame:
IMU1 stays on its sensor axes and IMU2 is sign-aligned to those axes. No body
frame transform or firmware software LPF is applied by this decoder.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import pathlib
import struct
import sys
from typing import Optional, Sequence


ZIMU_MAGIC = b"ZIMU"
ZCAL_MAGIC = b"ZCAL"
# v1 = 26바이트 샘플, v2 = 27바이트 (failsafe_phase 추가).
# v1 을 계속 읽는 것이 중요하다 — 2026-08-01 의 라벨된 두 비행이 v1 이고,
# 착지 판별식 후보를 평가할 유일한 실측 데이터셋이다.
VERSION = 2
SUPPORTED_VERSIONS = (1, 2)
MAX_SAMPLES = 50
ZIMU_HEADER = struct.Struct("<4sBBHIII")
ZIMU_SAMPLE_V1 = struct.Struct("<H12h")
ZIMU_SAMPLE_V2 = struct.Struct("<H12hB")
ZIMU_SAMPLE_BY_VERSION = {1: ZIMU_SAMPLE_V1, 2: ZIMU_SAMPLE_V2}
ZCAL_PACKET = struct.Struct("<4sB3x13f")

RAW_COLUMNS = (
    "imu1_gyro_x_raw", "imu1_gyro_y_raw", "imu1_gyro_z_raw",
    "imu1_accel_x_raw", "imu1_accel_y_raw", "imu1_accel_z_raw",
    "imu2_gyro_x_raw", "imu2_gyro_y_raw", "imu2_gyro_z_raw",
    "imu2_accel_x_raw", "imu2_accel_y_raw", "imu2_accel_z_raw",
)
PHYSICAL_COLUMNS = (
    "imu1_gyro_x_dps", "imu1_gyro_y_dps", "imu1_gyro_z_dps",
    "imu1_accel_x_g", "imu1_accel_y_g", "imu1_accel_z_g",
    "imu2_gyro_x_dps", "imu2_gyro_y_dps", "imu2_gyro_z_dps",
    "imu2_accel_x_g", "imu2_accel_y_g", "imu2_accel_z_g",
)


@dataclasses.dataclass(frozen=True)
class Calibration:
    gyro_bias1: tuple[float, float, float]
    gyro_bias2: tuple[float, float, float]
    accel_scale1: float
    accel_scale2: float
    gyro_scale: float
    accel_scale: float
    imu2_sign: tuple[float, float, float]


@dataclasses.dataclass(frozen=True)
class DecodedSample:
    sample_idx: int
    t_us: int
    raw: tuple[int, ...]
    calibration: Optional[Calibration]
    failsafe_phase: Optional[int] = None


@dataclasses.dataclass
class DecodeSummary:
    total_batches: int = 0
    total_samples: int = 0
    lost_batches: int = 0
    producer_dropped: int = 0
    average_sample_rate_hz: float = 0.0
    dt_us_min: Optional[int] = None
    dt_us_max: Optional[int] = None
    truncated_records: int = 0
    unknown_magic_records: int = 0
    unsupported_version_records: int = 0
    calibration_records: int = 0


def _next_magic(data: bytes, start: int) -> int:
    positions = (
        position
        for position in (
            data.find(ZIMU_MAGIC, start),
            data.find(ZCAL_MAGIC, start),
        )
        if position >= 0
    )
    return min(positions, default=-1)


def _calibration_from_packet(values: Sequence[float]) -> Calibration:
    return Calibration(
        gyro_bias1=tuple(values[0:3]),
        gyro_bias2=tuple(values[3:6]),
        accel_scale1=values[6],
        accel_scale2=values[7],
        gyro_scale=values[8],
        accel_scale=values[9],
        imu2_sign=tuple(values[10:13]),
    )


def _physical_values(
    raw: Sequence[int], calibration: Calibration
) -> tuple[float, ...]:
    gyro1 = tuple(
        raw[axis] * calibration.gyro_scale
        - calibration.gyro_bias1[axis]
        for axis in range(3)
    )
    accel1 = tuple(
        raw[3 + axis]
        * calibration.accel_scale
        * calibration.accel_scale1
        for axis in range(3)
    )
    gyro2 = tuple(
        calibration.imu2_sign[axis]
        * raw[6 + axis]
        * calibration.gyro_scale
        - calibration.gyro_bias2[axis]
        for axis in range(3)
    )
    accel2 = tuple(
        calibration.imu2_sign[axis]
        * raw[9 + axis]
        * calibration.accel_scale
        * calibration.accel_scale2
        for axis in range(3)
    )
    return gyro1 + accel1 + gyro2 + accel2


def _parse_capture(data: bytes) -> tuple[list[DecodedSample], DecodeSummary]:
    summary = DecodeSummary()
    samples: list[DecodedSample] = []
    current_calibration: Optional[Calibration] = None
    first_calibration: Optional[Calibration] = None
    previous_batch_seq: Optional[int] = None
    rate_dt_values: list[int] = []
    all_dt_values: list[int] = []
    position = 0

    while position < len(data):
        magic = data[position:position + 4]
        if magic not in (ZIMU_MAGIC, ZCAL_MAGIC):
            summary.unknown_magic_records += 1
            next_position = _next_magic(data, position + 1)
            if next_position < 0:
                break
            position = next_position
            continue

        if magic == ZCAL_MAGIC:
            record_end = position + ZCAL_PACKET.size
            next_position = _next_magic(data, position + 4)
            if record_end > len(data) or (
                0 <= next_position < record_end
            ):
                summary.truncated_records += 1
                if next_position < 0:
                    break
                position = next_position
                continue
            unpacked = ZCAL_PACKET.unpack_from(data, position)
            # ZCAL 레이아웃은 v1/v2 가 동일하다. ZIMU 버전 상향 때문에
            # 예전 로그의 ZCAL 을 버리면 물리량 복원이 통째로 날아간다.
            if unpacked[1] not in SUPPORTED_VERSIONS:
                summary.unsupported_version_records += 1
            else:
                current_calibration = _calibration_from_packet(unpacked[2:])
                if first_calibration is None:
                    first_calibration = current_calibration
                summary.calibration_records += 1
            position = record_end
            continue

        if position + ZIMU_HEADER.size > len(data):
            summary.truncated_records += 1
            break
        (
            _,
            version,
            n_samples,
            _reserved,
            batch_seq,
            base_t_us,
            dropped,
        ) = ZIMU_HEADER.unpack_from(data, position)
        if not 1 <= n_samples <= MAX_SAMPLES:
            summary.truncated_records += 1
            next_position = _next_magic(data, position + 4)
            if next_position < 0:
                break
            position = next_position
            continue
        # 레코드 길이는 버전마다 다르므로(v1 26B, v2 27B) 버전을 먼저 본다.
        # 모르는 버전은 길이를 알 수 없어 다음 매직까지 건너뛴다.
        if version not in SUPPORTED_VERSIONS:
            summary.unsupported_version_records += 1
            next_position = _next_magic(data, position + 4)
            if next_position < 0:
                break
            position = next_position
            continue
        sample_struct = ZIMU_SAMPLE_BY_VERSION[version]
        record_end = (
            position + ZIMU_HEADER.size + n_samples * sample_struct.size
        )
        next_position = _next_magic(data, position + 4)
        if record_end > len(data) or (
            0 <= next_position < record_end
        ):
            summary.truncated_records += 1
            if next_position < 0:
                break
            position = next_position
            continue

        if previous_batch_seq is not None:
            sequence_delta = (batch_seq - previous_batch_seq) & 0xFFFFFFFF
            if 1 < sequence_delta < 0x80000000:
                summary.lost_batches += sequence_delta - 1
        previous_batch_seq = batch_seq
        summary.total_batches += 1
        summary.producer_dropped = dropped

        sample_position = position + ZIMU_HEADER.size
        sample_t_us = base_t_us
        for sample_in_batch in range(n_samples):
            unpacked_sample = sample_struct.unpack_from(data, sample_position)
            dt_us = unpacked_sample[0]
            raw = tuple(unpacked_sample[1:13])
            failsafe_phase = (
                unpacked_sample[13] if version >= 2 else None
            )
            if sample_in_batch > 0:
                sample_t_us = (sample_t_us + dt_us) & 0xFFFFFFFF
            if samples:
                rate_dt_values.append(dt_us)
            all_dt_values.append(dt_us)
            samples.append(
                DecodedSample(
                    sample_idx=len(samples),
                    t_us=sample_t_us,
                    raw=raw,
                    calibration=current_calibration,
                    failsafe_phase=failsafe_phase,
                )
            )
            sample_position += sample_struct.size

        summary.total_samples += n_samples
        position = record_end

    if first_calibration is not None:
        samples = [
            sample
            if sample.calibration is not None
            else dataclasses.replace(
                sample, calibration=first_calibration
            )
            for sample in samples
        ]
    if rate_dt_values and sum(rate_dt_values) > 0:
        summary.average_sample_rate_hz = (
            1_000_000.0 * len(rate_dt_values) / sum(rate_dt_values)
        )
    if all_dt_values:
        summary.dt_us_min = min(all_dt_values)
        summary.dt_us_max = max(all_dt_values)
    return samples, summary


def _write_csv(
    output_path: pathlib.Path,
    samples: Sequence[DecodedSample],
    include_physical: bool,
) -> None:
    # v1 로그에는 failsafe_phase 가 없다. 컬럼은 항상 두되 값은 비워서,
    # 예전 로그와 새 로그를 같은 스키마로 다룰 수 있게 한다.
    columns = ["sample_idx", "t_us", "failsafe_phase", *RAW_COLUMNS]
    if include_physical:
        columns.extend(PHYSICAL_COLUMNS)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for sample in samples:
            row: list[object] = [
                sample.sample_idx,
                sample.t_us,
                "" if sample.failsafe_phase is None else sample.failsafe_phase,
                *sample.raw,
            ]
            if include_physical:
                if sample.calibration is None:
                    row.extend([""] * len(PHYSICAL_COLUMNS))
                else:
                    row.extend(
                        f"{value:.9g}"
                        for value in _physical_values(
                            sample.raw, sample.calibration
                        )
                    )
            writer.writerow(row)


def _print_summary(summary: DecodeSummary) -> None:
    dt_min = "-" if summary.dt_us_min is None else summary.dt_us_min
    dt_max = "-" if summary.dt_us_max is None else summary.dt_us_max
    print(
        "summary "
        f"batches={summary.total_batches} "
        f"samples={summary.total_samples} "
        f"wireless_lost_batches={summary.lost_batches} "
        f"producer_dropped={summary.producer_dropped} "
        f"average_sample_rate_hz={summary.average_sample_rate_hz:.3f} "
        f"dt_us_min={dt_min} "
        f"dt_us_max={dt_max} "
        f"truncated={summary.truncated_records} "
        f"unknown_magic={summary.unknown_magic_records} "
        f"unsupported_version={summary.unsupported_version_records}",
        file=sys.stderr,
    )


def decode_file(
    input_path: pathlib.Path | str,
    output_path: pathlib.Path | str | None = None,
) -> DecodeSummary:
    """Decode one append-only capture and return its loss/corruption summary."""
    input_path = pathlib.Path(input_path)
    output_path = (
        pathlib.Path(output_path)
        if output_path is not None
        else input_path.with_suffix(".csv")
    )
    samples, summary = _parse_capture(input_path.read_bytes())
    include_physical = summary.calibration_records > 0
    _write_csv(output_path, samples, include_physical)
    if not include_physical:
        print(
            "WARNING: no ZCAL record found; output contains raw counts only",
            file=sys.stderr,
        )
    _print_summary(summary)
    return summary


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Decode ZIMU/ZCAL .bin capture to IMU1-sensor-frame CSV"
        )
    )
    parser.add_argument("input", type=pathlib.Path, help="input .bin capture")
    parser.add_argument(
        "output",
        nargs="?",
        type=pathlib.Path,
        help="output .csv (default: input path with .csv suffix)",
    )
    arguments = parser.parse_args(argv)
    decode_file(arguments.input, arguments.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
