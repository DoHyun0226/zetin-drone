import csv
import sys
import tempfile
import threading
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from ground.camera_stream import INDEX_FIELDS, MjpegParser, aligned_rows  # noqa: E402
from ground.camera_mp4 import (  # noqa: E402
    build_ffmpeg_command,
    build_video_inputs,
    extract_frames,
    load_index,
    write_aligned_csv,
    write_concat_list,
)
from ground.camera_record import RecordState, record_chunks  # noqa: E402


BOUNDARY = b"zetincamframe"


def part(jpeg, seq, ts):
    return (
        b"\r\n--" + BOUNDARY + b"\r\nContent-Length: %d\r\n" % len(jpeg)
        + b"X-Timestamp: " + ts.encode() + b"\r\nX-Frame-Seq: %d\r\n\r\n" % seq + jpeg
    )


def jpeg(i):
    return b"\xff\xd8" + bytes([65 + i]) * (20 + i) + b"\xff\xd9"


class Clock:
    def __init__(self, start):
        self.t = start

    def __call__(self):
        self.t += 0.001
        return self.t


def make_recording(folder: Path, captures=("0.000000", "0.100000", "0.200000")):
    """A real .mjpeg/.csv pair written by ground.camera_record.record_chunks."""
    mjpeg_path = folder / "camera_test.mjpeg"
    csv_path = folder / "camera_test.csv"
    jpegs = [jpeg(i) for i in range(len(captures))]
    data = b"".join(part(j, i, ts) for i, (j, ts) in enumerate(zip(jpegs, captures)))
    with mjpeg_path.open("wb") as m, csv_path.open("w", newline="", encoding="utf-8") as c:
        writer = csv.DictWriter(c, fieldnames=INDEX_FIELDS)
        writer.writeheader()
        record_chunks(
            [data], MjpegParser(BOUNDARY), m, writer, 0, RecordState(), None, threading.Event(),
            now_epoch=Clock(1_793_000_000.0), now_mono=Clock(10.0),
        )
    return mjpeg_path, csv_path, jpegs


class ExtractTest(unittest.TestCase):
    def test_extract_frames_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            mjpeg_path, csv_path, jpegs = make_recording(tmp)
            frames, kept = extract_frames(mjpeg_path, load_index(csv_path), tmp / "frames")
            self.assertEqual([f.read_bytes() for f in frames], jpegs)
            self.assertEqual([f.name for f in frames], ["frame_000000.jpg", "frame_000001.jpg", "frame_000002.jpg"])
            self.assertEqual(len(kept), 3)

    def test_extract_frames_truncated_middle_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            mjpeg_path, csv_path, _ = make_recording(tmp)
            rows = load_index(csv_path)
            rows[1]["size"] = str(int(rows[1]["size"]) + 10_000)
            with self.assertRaises(ValueError):
                extract_frames(mjpeg_path, rows, tmp / "frames")

    def test_extract_frames_truncated_last_skipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            mjpeg_path, csv_path, jpegs = make_recording(tmp)
            data = mjpeg_path.read_bytes()
            mjpeg_path.write_bytes(data[:-5])  # killed while writing the last frame
            frames, kept = extract_frames(mjpeg_path, load_index(csv_path), tmp / "frames")
            self.assertEqual([f.read_bytes() for f in frames], jpegs[:2])
            self.assertEqual([r["frame"] for r in kept], ["0", "1"])


class ConcatTest(unittest.TestCase):
    def test_concat_list_format(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            frames = [tmp / "frame_000000.jpg", tmp / "frame_000001.jpg"]
            list_path = tmp / "list.txt"
            write_concat_list(frames, [0.1, 0.1], list_path)
            a, b = (str(f.resolve()) for f in frames)
            expected = (
                f"file '{a}'\noption framerate 1000\nduration 0.100000\n"
                f"file '{b}'\noption framerate 1000\nduration 0.100000\n"
                f"file '{b}'\noption framerate 1000\n"
            )
            self.assertEqual(list_path.read_text(encoding="utf-8"), expected)

    def test_build_ffmpeg_command(self):
        cmd = build_ffmpeg_command(Path("/x/list.txt"), Path("/x/out.mp4"))
        self.assertEqual(
            cmd,
            ["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", "/x/list.txt", "-fps_mode", "vfr",
             "-c:v", "libx264", "-enc_time_base", "1:1000", "-pix_fmt", "yuv420p", "/x/out.mp4"],
        )

    def test_build_video_inputs_recomputes_after_truncation(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            mjpeg_path, csv_path, _ = make_recording(tmp, captures=("0.000000", "0.100000", "10.000000"))
            mjpeg_path.write_bytes(mjpeg_path.read_bytes()[:-5])
            frames, durations = build_video_inputs(mjpeg_path, aligned_rows(load_index(csv_path)), tmp / "frames")
            self.assertEqual(len(frames), 2)
            self.assertEqual(len(durations), 2)
            self.assertAlmostEqual(durations[0], 0.1, delta=1e-6)
            self.assertAlmostEqual(durations[1], 0.1, delta=1e-6)  # last_s, not 9.9


class AlignedCsvTest(unittest.TestCase):
    def test_aligned_csv_columns(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            _, csv_path, _ = make_recording(tmp)
            out = tmp / "aligned.csv"
            write_aligned_csv(aligned_rows(load_index(csv_path)), out)
            with out.open(newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                rows = list(reader)
            self.assertEqual(tuple(reader.fieldnames), INDEX_FIELDS + ("capture_epoch_s", "capture_time"))
            self.assertEqual(len(rows), 3)
            self.assertNotEqual(rows[0]["capture_time"], "")


if __name__ == "__main__":
    unittest.main()
