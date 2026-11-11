"""Align a camera recording to laptop time and export an mp4.

  python scripts/drone_video.py mp4 logs/camera_YYYY-MM-DD_HHMMSS.csv [--no-mp4]

Writes next to the input:
  camera_..._aligned.csv  index rows + capture time on the laptop clock, to
                          compare with the flight log ``Timestamp`` column
  camera_....mp4          variable-frame-rate video (needs ffmpeg)

The offset per connection segment is min(receive - capture): event-level
alignment (about +-100 ms), not motion-analysis sync. See the design doc 5.3.
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

from ground.camera_stream import (
    INDEX_FIELDS,
    aligned_rows,
    clock_jumps,
    drift_report,
    estimate_offsets,
    frame_durations,
)


ALIGNED_FIELDS = INDEX_FIELDS + ("capture_epoch_s", "capture_time")


def load_index(csv_path: Path) -> list[dict]:
    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_aligned_csv(rows: list[dict], path: Path) -> None:
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=ALIGNED_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def extract_frames(mjpeg_path: Path, rows, out_dir: Path) -> tuple[list[Path], list[dict]]:
    """Write each indexed JPEG to ``out_dir`` and return (files, rows actually extracted).

    A truncated LAST frame (recorder killed mid-write) is skipped with a warning;
    a truncated frame anywhere else means the pair is inconsistent and raises.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    frames: list[Path] = []
    kept: list[dict] = []
    with Path(mjpeg_path).open("rb") as f:
        for i, r in enumerate(rows):
            size = int(r["size"])
            f.seek(int(r["offset"]))
            data = f.read(size)
            if len(data) != size:
                if i == len(rows) - 1:
                    print(f"[WARN] last frame {r['frame']} is truncated ({len(data)}/{size} B); skipped")
                    break
                raise ValueError(f"frame {r['frame']} is truncated ({len(data)}/{size} B)")
            path = out_dir / f"frame_{len(frames):06d}.jpg"
            path.write_bytes(data)
            frames.append(path)
            kept.append(r)
    return frames, kept


def build_video_inputs(mjpeg_path: Path, aligned: list[dict], out_dir: Path) -> tuple[list[Path], list[float]]:
    """Frames and their on-screen durations, computed from the rows that survived extraction."""
    frames, kept = extract_frames(mjpeg_path, aligned, out_dir)
    durations = frame_durations(kept)
    assert len(frames) == len(durations), (len(frames), len(durations))
    return frames, durations


def write_concat_list(frames: list[Path], durations: list[float], list_path: Path) -> None:
    # "option framerate 1000" per file: without it the concat demuxer puts JPEG
    # inputs on a 25 Hz time base and rounds the durations (measured: 0.1 s ->
    # 0.12 s, ffmpeg n9.0.2). The last file is repeated so its duration applies.
    lines: list[str] = []
    for path, duration in zip(frames, durations):
        lines += [f"file '{Path(path).resolve()}'", "option framerate 1000", f"duration {duration:.6f}"]
    if frames:
        lines += [f"file '{Path(frames[-1]).resolve()}'", "option framerate 1000"]
    Path(list_path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_ffmpeg_command(list_path: Path, out_path: Path) -> list[str]:
    return [
        "ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", str(list_path),
        "-fps_mode", "vfr", "-c:v", "libx264", "-enc_time_base", "1:1000",
        "-pix_fmt", "yuv420p", str(out_path),
    ]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Align a camera recording to laptop time and export mp4")
    parser.add_argument("csv", type=Path, help="logs/camera_<time>.csv written by drone_video.py record")
    parser.add_argument("--no-mp4", action="store_true", help="only write the aligned CSV")
    args = parser.parse_args(argv)

    csv_path: Path = args.csv
    mjpeg_path = csv_path.with_suffix(".mjpeg")
    rows = load_index(csv_path)
    if not rows:
        print(f"[ERROR] no frames in {csv_path}")
        return 1

    for segment, offset in sorted(estimate_offsets(rows).items()):
        print(f"[SYNC] segment {segment}: offset {offset:.3f} s")
    for segment, drift in sorted(drift_report(rows).items()):
        text = "segment too short" if drift is None else f"{drift * 1000:+.1f} ms"
        print(f"[SYNC] segment {segment} drift: {text}")
    jumps = clock_jumps(rows)
    if jumps:
        print(f"[WARN] laptop wall clock jumped at frames {jumps}; alignment around them is unreliable")

    aligned = aligned_rows(rows)
    aligned_path = csv_path.with_name(csv_path.stem + "_aligned.csv")
    write_aligned_csv(aligned, aligned_path)
    print(f"[LOG] saved {aligned_path}")

    if args.no_mp4:
        return 0
    mp4_path = csv_path.with_suffix(".mp4")
    # next to the output, not /tmp: a long recording is tens of thousands of
    # JPEGs and /tmp is often a RAM-backed tmpfs
    with tempfile.TemporaryDirectory(dir=csv_path.parent, prefix=".camera_frames_") as tmp:
        frames, durations = build_video_inputs(mjpeg_path, aligned, Path(tmp))
        list_path = Path(tmp) / "concat.txt"
        write_concat_list(frames, durations, list_path)
        completed = subprocess.run(build_ffmpeg_command(list_path, mp4_path), check=False)
    if completed.returncode != 0:
        print(f"[ERROR] ffmpeg failed (exit {completed.returncode})")
        return 1
    print(f"[LOG] saved {mp4_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
