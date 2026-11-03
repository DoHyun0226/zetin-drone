import csv
import http.client
import io
import socket
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from ground.camera_stream import INDEX_FIELDS, MjpegParser, format_recv_time  # noqa: E402
from unittest import mock  # noqa: E402

from ground.camera_record import (  # noqa: E402
    LatestFrame,
    OutputError,
    OpenedStream,
    RecordState,
    build_arg_parser,
    iter_response,
    record_chunks,
    run,
)


BOUNDARY = b"zetincamframe"
CONTENT_TYPE = "multipart/x-mixed-replace;boundary=zetincamframe"


def part(jpeg, seq, ts=None):
    ts = f"{seq}.000000" if ts is None else ts
    return (
        b"\r\n--" + BOUNDARY + b"\r\n"
        + b"Content-Type: image/jpeg\r\nContent-Length: %d\r\n" % len(jpeg)
        + b"X-Timestamp: " + ts.encode() + b"\r\nX-Frame-Seq: %d\r\n\r\n" % seq
        + jpeg
    )


def jpeg(i):
    return b"\xff\xd8" + bytes([i]) * (10 + i) + b"\xff\xd9"


class FakeClock:
    def __init__(self, start):
        self.t = start

    def __call__(self):
        self.t += 0.01
        return self.t


def read_index(out_dir):
    paths = sorted(Path(out_dir).glob("camera_*.csv"))
    assert len(paths) == 1, paths
    with paths[0].open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f)), paths[0].with_suffix(".mjpeg")


def opened(chunks, sock=None, closed=None):
    def close():
        if closed is not None:
            closed.append(True)

    return OpenedStream(BOUNDARY, iter(chunks), sock, close)


class RecordChunksTest(unittest.TestCase):
    def test_record_chunks_writes_mjpeg_and_index(self):
        jpegs = [jpeg(i) for i in range(3)]
        data = b"".join(part(j, i) for i, j in enumerate(jpegs))
        mjpeg = io.BytesIO()
        text = io.StringIO()
        writer = csv.DictWriter(text, fieldnames=INDEX_FIELDS)
        writer.writeheader()
        state = RecordState()
        record_chunks(
            [data[:7], data[7:40], data[40:]], MjpegParser(BOUNDARY), mjpeg, writer, 0, state, None,
            threading.Event(), now_epoch=FakeClock(1_793_000_000.0), now_mono=FakeClock(50.0),
        )
        self.assertEqual(mjpeg.getvalue(), b"".join(jpegs))
        rows = list(csv.DictReader(io.StringIO(text.getvalue())))
        self.assertEqual([int(r["frame"]) for r in rows], [0, 1, 2])
        self.assertEqual([int(r["size"]) for r in rows], [len(j) for j in jpegs])
        self.assertEqual([int(r["offset"]) for r in rows], [0, len(jpegs[0]), len(jpegs[0]) + len(jpegs[1])])
        self.assertEqual([r["camera_seq"] for r in rows], ["0", "1", "2"])
        self.assertEqual(rows[0]["recv_time"], format_recv_time(float(rows[0]["recv_epoch_s"])))
        self.assertEqual(state.next_frame, 3)

    def test_record_chunks_uses_header_arrival_time(self):
        data = part(jpeg(5), 0)
        cut = data.index(b"\r\n\r\n") + 6
        text = io.StringIO()
        writer = csv.DictWriter(text, fieldnames=INDEX_FIELDS)
        writer.writeheader()
        record_chunks(
            [data[:cut], data[cut:]], MjpegParser(BOUNDARY), io.BytesIO(), writer, 0, RecordState(), None,
            threading.Event(), now_epoch=FakeClock(100.0), now_mono=FakeClock(10.0),
        )
        row = next(csv.DictReader(io.StringIO(text.getvalue())))
        self.assertAlmostEqual(float(row["recv_epoch_s"]), 100.01, delta=1e-9)
        self.assertAlmostEqual(float(row["recv_monotonic_s"]), 10.01, delta=1e-9)

    def test_record_chunks_updates_latest(self):
        latest = LatestFrame()
        writer = csv.DictWriter(io.StringIO(), fieldnames=INDEX_FIELDS)
        record_chunks(
            [part(jpeg(1), 0) + part(jpeg(2), 1)], MjpegParser(BOUNDARY), io.BytesIO(), writer, 0,
            RecordState(), latest, threading.Event(),
        )
        generation, data, _ = latest.get()
        self.assertEqual(generation, 2)
        self.assertEqual(data, jpeg(2))


class OutputErrorTest(unittest.TestCase):
    def test_disk_write_error_is_output_error(self):
        class FullDisk(io.BytesIO):
            def write(self, data):
                raise OSError(28, "No space left on device")

        writer = csv.DictWriter(io.StringIO(), fieldnames=INDEX_FIELDS)
        with self.assertRaises(OutputError) as ctx:
            record_chunks(
                [part(jpeg(1), 0)], MjpegParser(BOUNDARY), FullDisk(), writer, 0, RecordState(), None,
                threading.Event(),
            )
        self.assertNotIsInstance(ctx.exception, OSError)  # must not look like a link drop


class LatestFrameTest(unittest.TestCase):
    def test_latest_frame_generation(self):
        latest = LatestFrame()
        self.assertIsNone(latest.get())
        latest.set(b"a", 1.0)
        self.assertEqual(latest.get()[0], 1)
        latest.set(b"b", 2.0)
        self.assertEqual(latest.get(), (2, b"b", 2.0))


class IterResponseTest(unittest.TestCase):
    def test_iter_response_keeps_partial_on_incomplete_read(self):
        body = part(jpeg(3), 0)
        raw = (
            b"HTTP/1.1 200 OK\r\nContent-Type: " + CONTENT_TYPE.encode()
            + b"\r\nTransfer-Encoding: chunked\r\n\r\n"
            + b"%x\r\n" % len(body) + body + b"\r\n"
            + b"40\r\n" + b"\r\n--" + BOUNDARY  # next chunk cut short: connection dropped
        )

        class FakeSock:
            def makefile(self, *args, **kwargs):
                return io.BytesIO(raw)

        resp = http.client.HTTPResponse(FakeSock())
        resp.begin()
        parser = MjpegParser(BOUNDARY)
        parts = []
        for chunk in iter_response(resp):
            parts.extend(parser.feed(chunk))
        parts.extend(parser.finish()[0])
        self.assertEqual([p.jpeg for p in parts], [jpeg(3)])


class RunTest(unittest.TestCase):
    def test_reconnect_starts_new_segment(self):
        calls = []
        stop = threading.Event()

        def first():
            yield part(jpeg(0), 0) + part(jpeg(1), 1)
            yield part(jpeg(2), 2)
            raise OSError("link dropped")

        def second():
            yield part(jpeg(3), 0) + part(jpeg(4), 1)  # camera rebooted: seq restarts
            stop.set()

        def opener(url, timeout_s):
            calls.append(url)
            if len(calls) == 1:
                return opened(first())
            if len(calls) == 2:
                return opened(second())
            raise OSError("no more")

        with tempfile.TemporaryDirectory() as tmp:
            summary = run("http://cam", Path(tmp), preview=False, timeout_s=1.0, retry_s=0, opener=opener, stop=stop)
            rows, mjpeg_path = read_index(tmp)
            self.assertEqual(mjpeg_path.read_bytes(), b"".join(jpeg(i) for i in range(5)))
        self.assertEqual(summary.frames, 5)
        self.assertEqual(summary.segments, 2)
        self.assertEqual(summary.seq_gaps, 0)
        self.assertEqual([r["segment"] for r in rows], ["0", "0", "0", "1", "1"])
        self.assertEqual([r["frame"] for r in rows], ["0", "1", "2", "3", "4"])

    def test_opener_failure_retries_without_crash(self):
        stop = threading.Event()
        calls = []

        def good():
            yield part(jpeg(7), 0)
            stop.set()

        def opener(url, timeout_s):
            calls.append(1)
            if len(calls) <= 2:
                raise OSError("camera not up yet")
            return opened(good())

        with tempfile.TemporaryDirectory() as tmp:
            summary = run("http://cam", Path(tmp), preview=False, timeout_s=1.0, retry_s=0, opener=opener, stop=stop)
        self.assertEqual(summary.frames, 1)
        self.assertGreaterEqual(len(calls), 3)

    def test_stop_ends_endless_stream(self):
        stop = threading.Event()
        closed = []

        def endless():
            seq = 0
            while True:
                yield part(jpeg(seq % 50), seq)
                seq += 1
                time.sleep(0.005)

        def opener(url, timeout_s):
            return opened(endless(), closed=closed)

        threading.Timer(0.2, stop.set).start()
        start = time.monotonic()
        with tempfile.TemporaryDirectory() as tmp:
            summary = run("http://cam", Path(tmp), preview=False, timeout_s=1.0, retry_s=0, opener=opener, stop=stop)
        self.assertLess(time.monotonic() - start, 2.0)
        self.assertGreater(summary.frames, 0)
        self.assertTrue(closed)

    def test_stop_while_read_blocked(self):
        stop = threading.Event()
        servers = []
        threads_before = set(threading.enumerate())

        def opener(url, timeout_s):
            srv, cli = socket.socketpair()
            servers.append(srv)
            body = part(jpeg(9), 0)
            srv.sendall(
                b"HTTP/1.1 200 OK\r\nContent-Type: " + CONTENT_TYPE.encode()
                + b"\r\nTransfer-Encoding: chunked\r\n\r\n" + b"%x\r\n" % len(body) + body + b"\r\n"
            )  # then silence: the next read1() blocks
            resp = http.client.HTTPResponse(cli)
            resp.begin()

            def close():
                resp.close()
                cli.close()

            return OpenedStream(BOUNDARY, iter_response(resp), cli, close)

        threading.Timer(0.2, stop.set).start()
        start = time.monotonic()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                summary = run("http://cam", Path(tmp), preview=False, timeout_s=30.0, retry_s=0, opener=opener, stop=stop)
                rows, _ = read_index(tmp)
        finally:
            for s in servers:
                s.close()
        self.assertLess(time.monotonic() - start, 2.0)
        self.assertEqual(len(rows), 1)
        self.assertEqual(summary.frames, 1)
        leftover = [t for t in threading.enumerate() if t not in threads_before and t.name.startswith("camera-recorder")]
        self.assertEqual(leftover, [])


class SafetyTest(unittest.TestCase):
    def test_existing_recording_is_not_overwritten(self):
        stop = threading.Event()

        def opener(url, timeout_s):
            def chunks():
                yield part(jpeg(1), 0)
                stop.set()
            return opened(chunks())

        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / "camera_2026-11-02_120000.csv").write_text("old index\n")
            (tmp / "camera_2026-11-02_120000.mjpeg").write_bytes(b"old video")
            run("http://cam", tmp, preview=False, timeout_s=1.0, retry_s=0, opener=opener, stop=stop,
                stamp="2026-11-02_120000")
            self.assertEqual((tmp / "camera_2026-11-02_120000.csv").read_text(), "old index\n")
            self.assertEqual((tmp / "camera_2026-11-02_120000.mjpeg").read_bytes(), b"old video")
            self.assertEqual(len(list(tmp.glob("camera_*.mjpeg"))), 2)

    def test_output_error_stops_without_reconnecting(self):
        calls = []

        def opener(url, timeout_s):
            calls.append(1)
            return opened(iter([part(jpeg(1), 0)]))

        with mock.patch("ground.camera_record.record_chunks", side_effect=OutputError("disk full")):
            with tempfile.TemporaryDirectory() as tmp:
                with self.assertRaises(RuntimeError):
                    run("http://cam", Path(tmp), preview=False, timeout_s=1.0, retry_s=0, opener=opener)
        self.assertEqual(len(calls), 1)

    def test_bad_header_reconnects_and_keeps_recording(self):
        stop = threading.Event()
        calls = []
        bad = (
            b"\r\n--" + BOUNDARY + b"\r\nContent-Length: %d\r\nX-Frame-Seq: bad\r\n\r\n" % len(jpeg(1))
            + jpeg(1)
        )

        def good():
            yield part(jpeg(2), 0)
            stop.set()

        def opener(url, timeout_s):
            calls.append(1)
            return opened(iter([bad, part(jpeg(3), 1)])) if len(calls) == 1 else opened(good())

        with tempfile.TemporaryDirectory() as tmp:
            summary = run("http://cam", Path(tmp), preview=False, timeout_s=1.0, retry_s=0, opener=opener, stop=stop)
        self.assertGreaterEqual(len(calls), 2)
        self.assertGreaterEqual(summary.frames, 1)

    def test_unexpected_worker_error_fails_the_run(self):
        def opener(url, timeout_s):
            return opened(iter([part(jpeg(1), 0)]))

        with mock.patch("ground.camera_record.record_chunks", side_effect=ZeroDivisionError("bug")):
            with tempfile.TemporaryDirectory() as tmp:
                with self.assertRaises(RuntimeError):
                    run("http://cam", Path(tmp), preview=False, timeout_s=1.0, retry_s=0, opener=opener)

    def test_stop_during_connect_is_not_missed(self):
        stop = threading.Event()
        servers = []

        def opener(url, timeout_s):
            stop.set()          # user quits while we are still connecting ...
            time.sleep(0.3)     # ... and main's interrupt() runs before the socket exists
            srv, cli = socket.socketpair()
            servers.append(srv)
            srv.sendall(
                b"HTTP/1.1 200 OK\r\nContent-Type: " + CONTENT_TYPE.encode()
                + b"\r\nTransfer-Encoding: chunked\r\n\r\n"
            )
            resp = http.client.HTTPResponse(cli)
            resp.begin()

            def close():
                resp.close()
                cli.close()

            return OpenedStream(BOUNDARY, iter_response(resp), cli, close)

        start = time.monotonic()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                run("http://cam", Path(tmp), preview=False, timeout_s=5.0, retry_s=0, opener=opener, stop=stop)
        finally:
            for s in servers:
                s.close()
        self.assertLess(time.monotonic() - start, 2.0)


class CliTest(unittest.TestCase):
    def test_default_url_uses_stream_port(self):
        args = build_arg_parser().parse_args([])
        self.assertEqual(args.url, "http://192.168.4.200:81/stream")
        self.assertEqual(args.timeout, 5.0)
        self.assertFalse(args.no_preview)


if __name__ == "__main__":
    unittest.main()
