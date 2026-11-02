import datetime
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from ground.camera_stream import (  # noqa: E402
    MjpegParser,
    aligned_rows,
    clock_jumps,
    drift_report,
    estimate_offsets,
    frame_durations,
    StreamFormatError,
    StreamPart,
    boundary_from_content_type,
    format_recv_time,
    summarize,
)


BOUNDARY = b"zetincamframe"
JPEG = b"\xff\xd8abc\xff\xd9"


def make_part(jpeg, ts="3000.000123", seq=7, headers=None):
    """Bytes for one part exactly as the camera sketch sends it."""
    if headers is None:
        headers = [b"Content-Type: image/jpeg", b"Content-Length: %d" % len(jpeg)]
        if ts is not None:
            headers.append(b"X-Timestamp: " + ts.encode())
        if seq is not None:
            headers.append(b"X-Frame-Seq: %d" % seq)
    return b"\r\n--" + BOUNDARY + b"\r\n" + b"\r\n".join(headers) + b"\r\n\r\n" + jpeg


def make_stream(parts):
    return b"".join(make_part(*p) if isinstance(p, tuple) else make_part(p) for p in parts)


def feed_all(parser, *chunks, tag=None):
    """Feed chunks, then finish(): the parser holds a part until its trailing boundary or EOF."""
    out = []
    for chunk in chunks:
        out.extend(parser.feed(chunk, tag=tag))
    remaining, _ = parser.finish()
    return out + remaining


def row(segment, seq, mono, epoch=None, capture=None, frame=0):
    return {
        "frame": str(frame),
        "segment": str(segment),
        "camera_seq": "" if seq is None else str(seq),
        "offset": "0",
        "size": "1",
        "capture_s": "" if capture is None else repr(capture),
        "recv_epoch_s": repr(mono if epoch is None else epoch),
        "recv_monotonic_s": repr(mono),
        "recv_time": "",
    }


class MjpegParserTest(unittest.TestCase):
    def test_single_part(self):
        parts = feed_all(MjpegParser(BOUNDARY), make_part(JPEG))
        self.assertEqual(parts, [StreamPart(JPEG, 3000.000123, 7)])

    def test_split_at_every_byte(self):
        jpegs = [JPEG, b"\xff\xd8" + b"x" * 300 + b"\xff\xd9", b"\xff\xd8\xff\xd9"]
        data = make_stream([(j, f"{i}.000001", i) for i, j in enumerate(jpegs)])
        parser = MjpegParser(BOUNDARY)
        out = []
        for i in range(len(data)):
            out.extend(parser.feed(data[i : i + 1]))
        out.extend(parser.finish()[0])
        self.assertEqual([p.jpeg for p in out], jpegs)
        self.assertEqual([p.seq for p in out], [0, 1, 2])

    def test_payload_containing_boundary(self):
        tricky = b"\xff\xd8" + b"\r\n--zetincamframe\r\n" + b"\xff\xd9"
        parts = feed_all(MjpegParser(BOUNDARY), make_stream([tricky, JPEG]))
        self.assertEqual([p.jpeg for p in parts], [tricky, JPEG])

    def test_missing_content_length_raises(self):
        data = make_part(JPEG, headers=[b"Content-Type: image/jpeg"])
        with self.assertRaises(StreamFormatError):
            MjpegParser(BOUNDARY).feed(data)

    def test_non_numeric_content_length_raises(self):
        data = make_part(JPEG, headers=[b"Content-Length: abc"])
        with self.assertRaises(StreamFormatError):
            MjpegParser(BOUNDARY).feed(data)

    def test_missing_optional_headers_are_none(self):
        parts = feed_all(MjpegParser(BOUNDARY), make_part(JPEG, ts=None, seq=None))
        self.assertEqual(parts, [StreamPart(JPEG, None, None)])

    def test_header_names_case_insensitive(self):
        data = make_part(
            JPEG,
            headers=[b"content-length: %d" % len(JPEG), b"x-timestamp: 1.5", b"x-frame-seq: 3"],
        )
        self.assertEqual(feed_all(MjpegParser(BOUNDARY), data), [StreamPart(JPEG, 1.5, 3)])

    def test_garbage_before_first_boundary_is_dropped(self):
        parts = feed_all(MjpegParser(BOUNDARY), b"HTTP noise" + make_part(JPEG))
        self.assertEqual([p.jpeg for p in parts], [JPEG])

    def test_boundary_from_content_type(self):
        self.assertEqual(
            boundary_from_content_type("multipart/x-mixed-replace;boundary=zetincamframe"), BOUNDARY
        )
        self.assertEqual(
            boundary_from_content_type('multipart/x-mixed-replace; boundary="zetincamframe"'), BOUNDARY
        )
        with self.assertRaises(StreamFormatError):
            boundary_from_content_type("image/jpeg")

    def test_tag_is_header_arrival_chunk(self):
        data = make_part(b"\xff\xd8" + b"y" * 100 + b"\xff\xd9")
        cut = data.index(b"\r\n\r\n") + 4 + 10  # headers complete, body still incomplete
        parser = MjpegParser(BOUNDARY)
        self.assertEqual(parser.feed(data[:cut], tag="A"), [])
        parts = parser.feed(data[cut:], tag="B") + parser.finish()[0]
        self.assertEqual(len(parts), 1)
        self.assertEqual(parts[0].tag, "A")

    def test_length_mismatch_raises(self):
        body = b"\xff\xd8" + b"z" * 20 + b"\xff\xd9"
        bad = make_part(
            body, headers=[b"Content-Length: %d" % (len(body) - 2), b"X-Frame-Seq: 0"]
        )
        with self.assertRaises(StreamFormatError):
            MjpegParser(BOUNDARY).feed(bad + make_part(JPEG))

    def test_finish_reports_truncated_part(self):
        data = make_part(b"\xff\xd8" + b"q" * 50 + b"\xff\xd9")
        parser = MjpegParser(BOUNDARY)
        parser.feed(data[:-10])
        self.assertEqual(parser.finish(), ([], True))

        parser = MjpegParser(BOUNDARY)
        self.assertEqual(parser.feed(data), [])  # held until the next boundary or EOF
        remaining, truncated = parser.finish()
        self.assertEqual([p.jpeg for p in remaining], [b"\xff\xd8" + b"q" * 50 + b"\xff\xd9"])
        self.assertFalse(truncated)

    def test_short_length_rejected_at_every_split(self):
        body = b"\xff\xd8" + b"z" * 20 + b"\xff\xd9"
        bad = make_part(body, headers=[b"Content-Length: %d" % (len(body) - 2), b"X-Frame-Seq: 0"])
        data = bad + make_part(JPEG)
        for cut in range(1, len(data)):
            with self.subTest(cut=cut):
                parser = MjpegParser(BOUNDARY)
                emitted = []
                with self.assertRaises(StreamFormatError):
                    emitted.extend(parser.feed(data[:cut]))
                    emitted.extend(parser.feed(data[cut:]))
                self.assertEqual(emitted, [])  # the frame without its EOI is never handed out

    def test_bad_numeric_header_is_stream_format_error(self):
        for headers in (
            [b"Content-Length: %d" % len(JPEG), b"X-Frame-Seq: bad"],
            [b"Content-Length: %d" % len(JPEG), b"X-Timestamp: bad"],
        ):
            with self.subTest(headers=headers):
                with self.assertRaises(StreamFormatError):
                    feed_all(MjpegParser(BOUNDARY), make_part(JPEG, headers=headers))

    def test_non_finite_timestamp_is_stream_format_error(self):
        for value in (b"nan", b"inf", b"-inf", b"1e309"):
            with self.subTest(value=value):
                headers = [b"Content-Length: %d" % len(JPEG), b"X-Timestamp: " + value]
                with self.assertRaises(StreamFormatError):
                    feed_all(MjpegParser(BOUNDARY), make_part(JPEG, headers=headers))

    def test_oversized_content_length_rejected_at_header(self):
        parser = MjpegParser(BOUNDARY)
        header_only = b"\r\n--" + BOUNDARY + b"\r\nContent-Length: 1000000000\r\n\r\n"
        with self.assertRaises(StreamFormatError):
            parser.feed(header_only)  # rejected before any body is buffered

    def test_valid_part_survives_following_bad_part(self):
        body = b"\xff\xd8" + b"z" * 20 + b"\xff\xd9"
        bad = make_part(body, headers=[b"Content-Length: %d" % (len(body) - 2), b"X-Frame-Seq: 1"])
        parser = MjpegParser(BOUNDARY)
        parts = parser.feed(make_part(JPEG) + bad + make_part(JPEG))
        self.assertEqual([p.jpeg for p in parts], [JPEG])
        with self.assertRaises(StreamFormatError):
            parser.feed(b"")


class RecvTimeTest(unittest.TestCase):
    def test_format_recv_time_matches_flight_log(self):
        t = 1_793_000_000.123456
        expected = datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]
        self.assertEqual(format_recv_time(t), expected)


class SummarizeTest(unittest.TestCase):
    def test_summarize_counts_gaps_within_segment_only(self):
        rows = [row(0, 0, 0.0), row(0, 1, 0.1), row(0, 3, 0.2), row(1, 0, 1.0), row(1, 1, 1.1)]
        summary = summarize(rows)
        self.assertEqual(summary.seq_gaps, 1)
        self.assertEqual(summary.segments, 2)
        self.assertEqual(summary.frames, 5)

    def test_summarize_fps_and_longest_gap(self):
        rows = [row(0, i, t) for i, t in enumerate([0.0, 0.1, 0.2, 0.9])]
        summary = summarize(rows)
        self.assertAlmostEqual(summary.longest_gap_s, 0.7, delta=1e-9)
        self.assertAlmostEqual(summary.effective_fps, 3 / 0.9, delta=1e-9)

        two = [row(0, 0, 0.0), row(0, 1, 1.0), row(1, 0, 5.0), row(1, 1, 5.5), row(1, 2, 6.0)]
        self.assertAlmostEqual(summarize(two).effective_fps, 3 / 2.0, delta=1e-9)

    def test_summarize_reconnect_gaps(self):
        rows = [row(0, 0, 0.0), row(0, 1, 1.0), row(1, 0, 3.5), row(1, 1, 4.0)]
        summary = summarize(rows)
        self.assertEqual(len(summary.reconnect_gaps_s), 1)
        self.assertAlmostEqual(summary.reconnect_gaps_s[0], 2.5, delta=1e-9)
        self.assertAlmostEqual(summary.longest_gap_s, 1.0, delta=1e-9)

    def test_summarize_ignores_wall_clock(self):
        rows = [row(0, 0, 0.0, epoch=100.0), row(0, 1, 0.1, epoch=50.0)]
        self.assertAlmostEqual(summarize(rows).longest_gap_s, 0.1, delta=1e-9)

    def test_summarize_empty(self):
        summary = summarize([])
        self.assertEqual((summary.frames, summary.segments, summary.seq_gaps), (0, 0, 0))
        self.assertEqual(summary.effective_fps, 0.0)


class ClockAlignmentTest(unittest.TestCase):
    def test_offset_is_min_delay(self):
        rows = [
            row(0, 0, 0, epoch=100.05, capture=0.0),
            row(0, 1, 0, epoch=100.30, capture=0.1),
            row(0, 2, 0, epoch=100.21, capture=0.2),
        ]
        self.assertAlmostEqual(estimate_offsets(rows)[0], 100.01, delta=1e-9)

    def test_offsets_per_segment(self):
        rows = [
            row(0, 0, 0, epoch=100.0, capture=0.0),
            row(0, 1, 0, epoch=100.1, capture=0.1),
            row(1, 0, 0, epoch=200.0, capture=1.0),
        ]
        offsets = estimate_offsets(rows)
        self.assertAlmostEqual(offsets[0], 100.0, delta=1e-9)
        self.assertAlmostEqual(offsets[1], 199.0, delta=1e-9)

    def test_offsets_skip_rows_without_capture(self):
        rows = [row(0, 0, 0, epoch=5.0, capture=None), row(0, 1, 0, epoch=10.0, capture=1.0)]
        self.assertEqual(estimate_offsets(rows), {0: 9.0})

    def test_drift_report_short_segment_none(self):
        rows = [row(0, i, i * 0.1, epoch=1000 + i * 0.1, capture=i * 0.1) for i in range(1000)]
        self.assertEqual(drift_report(rows), {0: None})  # 100 s < 2 x 60 s

    def test_drift_report_linear_drift(self):
        rows = []
        for i in range(3001):  # 300 s at 10 Hz, offset grows as 1e-4 * t
            t = i * 0.1
            rows.append(row(0, i, t, epoch=1000 + t + 1e-4 * t, capture=t))
        self.assertAlmostEqual(drift_report(rows)[0], 0.024, delta=0.001)

    def test_clock_jump_detected(self):
        rows = []
        for i in range(100):
            mono = 10 + i * 0.1
            epoch = 1000 + i * 0.1 + (2.0 if i >= 50 else 0.0)
            rows.append(row(0, i, mono, epoch=epoch, capture=i * 0.1, frame=i))
        self.assertEqual(clock_jumps(rows), [50])

    def test_frame_durations(self):
        aligned = [{"segment": "0", "capture_epoch_s": repr(t), "recv_epoch_s": "0"} for t in (0, 0.1, 0.1000001, 0.3)]
        durations = frame_durations(aligned)
        for got, want in zip(durations, [0.1, 0.001, 0.2, 0.1]):
            self.assertAlmostEqual(got, want, delta=1e-6)
        self.assertEqual(len(durations), 4)

    def test_frame_durations_fall_back_to_recv_time(self):
        aligned = [
            {"segment": "0", "capture_epoch_s": "", "recv_epoch_s": "5.0"},
            {"segment": "0", "capture_epoch_s": "", "recv_epoch_s": "5.25"},
        ]
        self.assertEqual(frame_durations(aligned), [0.25, 0.1])

    def test_aligned_rows_adds_columns(self):
        rows = [row(0, 0, 0, epoch=1_793_000_000.5, capture=2.0)]
        out = aligned_rows(rows)
        self.assertAlmostEqual(float(out[0]["capture_epoch_s"]), 1_793_000_000.5, delta=1e-6)
        self.assertEqual(out[0]["capture_time"], format_recv_time(1_793_000_000.5))
        self.assertEqual(out[0]["frame"], rows[0]["frame"])


if __name__ == "__main__":
    unittest.main()
