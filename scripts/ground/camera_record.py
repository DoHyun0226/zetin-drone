"""Record the drone camera's MJPEG stream on the laptop, with a live preview.

Run it in its own terminal next to ``control_dualsense.py``; it never touches
the flight UDP link. Output goes to ``logs/``:

  camera_<YYYY-MM-DD_HHMMSS>.mjpeg  the received JPEGs back to back (no re-encode)
  camera_<YYYY-MM-DD_HHMMSS>.csv    one row per frame (see camera_stream.INDEX_FIELDS)

``ground/camera_mp4.py`` turns the pair into a flight-log-aligned CSV and an mp4.
The stream comes from ``firmware/camera/esp32cam_stream``.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import http.client
import socket
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Callable, Iterable, Iterator, TextIO
from urllib.parse import urlsplit

from ground.paths import LOG_DIR
from ground.camera_stream import (
    INDEX_FIELDS,
    MjpegParser,
    RecordingSummary,
    StreamFormatError,
    boundary_from_content_type,
    format_recv_time,
    summarize,
)


DEFAULT_URL = "http://192.168.4.200:81/stream"
READ_BYTES = 4096


class OutputError(Exception):
    """Writing the recording failed (disk full, permissions, name clash).

    Deliberately not an OSError: those mean "the link dropped, reconnect",
    while this must stop the recorder and fail the run.
    """


class LatestFrame:
    """Newest received JPEG, handed from the recorder thread to the preview."""

    def __init__(self):
        self._lock = threading.Lock()
        self._value: tuple[int, bytes, float] | None = None
        self._generation = 0

    def set(self, jpeg: bytes, recv_monotonic_s: float) -> None:
        with self._lock:
            self._generation += 1
            self._value = (self._generation, jpeg, recv_monotonic_s)

    def get(self) -> tuple[int, bytes, float] | None:
        with self._lock:
            return self._value


@dataclass
class RecordState:
    # Lives across reconnects so frame numbers stay continuous even when a
    # connection ends in an exception.
    next_frame: int = 0


@dataclass
class OpenedStream:
    boundary: bytes
    chunks: Iterator[bytes]
    sock: socket.socket | None
    close: Callable[[], None]


class ActiveConnection:
    """The socket the recorder thread is reading, so another thread can wake it.

    Only ``shutdown`` is called from outside: closing a response from another
    thread does not wake a blocked ``read1()`` and can block on the same buffer
    lock. The recorder thread closes its own connection in ``finally``.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._sock: socket.socket | None = None
        self._interrupted = False

    def set(self, sock) -> None:
        # A socket registered after interrupt() (stop arrived while connecting)
        # is shut down at once, so its first read cannot block.
        with self._lock:
            self._sock = sock
            interrupted = self._interrupted
        if interrupted:
            _shutdown(sock)

    def clear(self) -> None:
        with self._lock:
            self._sock = None

    def interrupt(self) -> None:
        with self._lock:
            self._interrupted = True
            sock = self._sock
        _shutdown(sock)


def _shutdown(sock) -> None:
    if sock is None:
        return
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass


def write_parts(
    parts,
    mjpeg_file: BinaryIO,
    writer: csv.DictWriter,
    segment: int,
    state: RecordState,
    latest: LatestFrame | None,
    csv_file: TextIO | None = None,
) -> None:
    for part in parts:
        recv_epoch, recv_mono = part.tag
        try:
            offset = mjpeg_file.tell()
            mjpeg_file.write(part.jpeg)
            writer.writerow(
                {
                    "frame": state.next_frame,
                    "segment": segment,
                    "camera_seq": "" if part.seq is None else part.seq,
                    "offset": offset,
                    "size": len(part.jpeg),
                    "capture_s": "" if part.capture_s is None else f"{part.capture_s:.6f}",
                    "recv_epoch_s": f"{recv_epoch:.6f}",
                    "recv_monotonic_s": f"{recv_mono:.6f}",
                    "recv_time": format_recv_time(recv_epoch),
                }
            )
            mjpeg_file.flush()
            if csv_file is not None:
                csv_file.flush()
        except OSError as error:
            raise OutputError(f"writing the recording failed: {error}") from error
        state.next_frame += 1
        if latest is not None:
            latest.set(part.jpeg, recv_mono)


def record_chunks(
    chunks: Iterable[bytes],
    parser: MjpegParser,
    mjpeg_file: BinaryIO,
    writer: csv.DictWriter,
    segment: int,
    state: RecordState,
    latest: LatestFrame | None,
    stop: threading.Event,
    now_epoch=time.time,
    now_mono=time.monotonic,
    csv_file: TextIO | None = None,
) -> None:
    """Record parts from ``chunks``. At a clean end of stream the last held part
    is written too; on an exception or stop the caller finishes the parser."""
    for chunk in chunks:
        if stop.is_set():
            return
        parts = parser.feed(chunk, tag=(now_epoch(), now_mono()))
        write_parts(parts, mjpeg_file, writer, segment, state, latest, csv_file)
        if stop.is_set():
            return
    remaining, truncated = parser.finish()
    write_parts(remaining, mjpeg_file, writer, segment, state, latest, csv_file)
    if truncated:
        print("[WARN] dropped an incomplete frame at the end of the connection")


def iter_response(resp: http.client.HTTPResponse) -> Iterator[bytes]:
    # read1, not read(n): the camera uses chunked transfer, and when the link
    # drops read(n) hides already-complete bytes inside IncompleteRead.partial.
    while True:
        try:
            data = resp.read1(READ_BYTES)
        except http.client.IncompleteRead as error:
            if error.partial:
                yield error.partial
            return
        if not data:
            return
        yield data


def open_stream(url: str, timeout_s: float) -> OpenedStream:
    parts = urlsplit(url)
    conn = http.client.HTTPConnection(parts.hostname, parts.port or 80, timeout=timeout_s)
    try:
        conn.connect()
        sock = conn.sock
        local = sock.getsockname()[0]
        if not local.startswith("192.168.4."):
            print(f"[WARN] local address {local} is not on Drone_Tuning (192.168.4.x) - check the WiFi interface")
        conn.request("GET", parts.path or "/")
        resp = conn.getresponse()
        if resp.status != 200:
            raise OSError(f"camera answered HTTP {resp.status}")
        boundary = boundary_from_content_type(resp.getheader("Content-Type", ""))
    except BaseException:
        conn.close()
        raise

    def close():
        resp.close()
        conn.close()

    return OpenedStream(boundary, iter_response(resp), sock, close)


def record_loop(
    url: str,
    mjpeg_path: Path,
    csv_path: Path,
    latest: LatestFrame | None,
    stop: threading.Event,
    active: ActiveConnection,
    timeout_s: float,
    retry_s: float = 1.0,
    opener=open_stream,
    errors: list | None = None,
) -> None:
    """Recorder thread body. This thread owns both output files.

    Output failures end the thread: they are appended to ``errors`` and
    ``stop`` is set so the main thread exits and reports a failure.
    """
    try:
        _record_loop(url, mjpeg_path, csv_path, latest, stop, active, timeout_s, retry_s, opener)
    except (OutputError, OSError) as error:
        message = str(error) if isinstance(error, OutputError) else f"cannot write the recording: {error}"
        _fail(message, errors, stop)
    except Exception as error:  # a bug must not end the run looking like success
        _fail(f"recorder crashed: {type(error).__name__}: {error}", errors, stop)


def _fail(message: str, errors: list | None, stop: threading.Event) -> None:
    print(f"[ERROR] {message}")
    if errors is not None:
        errors.append(message)
    stop.set()


def _record_loop(url, mjpeg_path, csv_path, latest, stop, active, timeout_s, retry_s, opener) -> None:
    state = RecordState()
    segment = -1
    # "x": never overwrite an existing recording
    with mjpeg_path.open("xb") as mjpeg_file, csv_path.open("x", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=INDEX_FIELDS)
        writer.writeheader()
        csv_file.flush()
        while not stop.is_set():
            try:
                opened = opener(url, timeout_s)
            except (OSError, StreamFormatError, http.client.HTTPException) as error:
                print(f"[CAM] connect failed: {error}")
                stop.wait(retry_s)
                continue
            segment += 1
            parser = MjpegParser(opened.boundary)
            active.set(opened.sock)
            print(f"[CAM] connected (segment {segment})")
            try:
                if stop.is_set():  # stop arrived while connecting: do not start reading
                    continue
                record_chunks(
                    opened.chunks, parser, mjpeg_file, writer, segment, state, latest, stop,
                    csv_file=csv_file,
                )
            except (OSError, StreamFormatError, http.client.HTTPException) as error:
                if not stop.is_set():
                    print(f"[CAM] stream ended: {error}")
            finally:
                active.clear()
                try:
                    opened.close()
                except OSError:
                    pass
            remaining, truncated = parser.finish()  # after an exception or stop
            write_parts(remaining, mjpeg_file, writer, segment, state, latest, csv_file)
            if truncated and not stop.is_set():
                print("[WARN] dropped an incomplete frame at the end of the connection")
            if not stop.is_set():
                stop.wait(retry_s)


def preview_loop(latest: LatestFrame, stop: threading.Event) -> None:
    """OpenCV preview. Runs on the main thread (Linux GUI backends want that)."""
    try:
        import cv2
        import numpy as np
    except ImportError as error:
        print(f"[WARN] preview disabled ({error}); recording continues")
        stop.wait()
        return
    window = "drone camera"
    shown = 0
    stamps: list[float] = []
    frame = None
    while not stop.is_set():
        item = latest.get()
        if item is not None and item[0] != shown:
            shown, data, recv_mono = item
            image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is not None:
                frame, frame_recv = image, recv_mono
                stamps = [s for s in stamps if recv_mono - s < 1.0] + [recv_mono]
        if frame is not None:
            view = frame.copy()
            age_ms = (time.monotonic() - frame_recv) * 1000
            cv2.putText(view, f"fps {len(stamps)}  age {age_ms:.0f} ms", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow(window, view)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            stop.set()
            break
        time.sleep(1 / 30)
    cv2.destroyAllWindows()


def run(
    url: str,
    out_dir: Path,
    preview: bool,
    timeout_s: float,
    retry_s: float = 1.0,
    opener=open_stream,
    stop: threading.Event | None = None,
    stamp: str | None = None,
) -> RecordingSummary:
    stop = stop or threading.Event()
    out_dir.mkdir(parents=True, exist_ok=True)
    mjpeg_path, csv_path = _new_recording_paths(out_dir, stamp or f"{datetime.datetime.now():%Y-%m-%d_%H%M%S}")
    latest = LatestFrame() if preview else None
    active = ActiveConnection()
    errors: list[str] = []
    worker = threading.Thread(
        target=record_loop,
        args=(url, mjpeg_path, csv_path, latest, stop, active, timeout_s, retry_s, opener, errors),
        name="camera-recorder",
        daemon=True,
    )
    worker.start()
    print(f"[LOG] camera: {mjpeg_path}")
    try:
        if latest is not None:
            preview_loop(latest, stop)
        else:
            while not stop.is_set() and worker.is_alive():
                stop.wait(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        active.interrupt()
        worker.join(timeout_s + 2)
    if worker.is_alive():
        print("[ERROR] recorder thread did not exit")
        raise RuntimeError("recorder thread did not exit")
    if errors:
        raise RuntimeError(errors[0])
    with csv_path.open(newline="", encoding="utf-8") as f:
        summary = summarize(list(csv.DictReader(f)))
    print(
        f"[LOG] frames {summary.frames}, segments {summary.segments} "
        f"(reconnects {max(0, summary.segments - 1)}), seq gaps {summary.seq_gaps}, "
        f"fps {summary.effective_fps:.1f}, longest gap {summary.longest_gap_s:.2f} s"
    )
    if summary.reconnect_gaps_s:
        gaps = ", ".join(f"{g:.1f}" for g in summary.reconnect_gaps_s)
        print(f"[LOG] reconnect gaps (s): {gaps}")
    print(f"[LOG] saved {mjpeg_path}")
    print(f"[LOG] saved {csv_path}")
    return summary


def _new_recording_paths(out_dir: Path, stamp: str) -> tuple[Path, Path]:
    """camera_<stamp>.{mjpeg,csv}, or camera_<stamp>_<n> if either name is taken."""
    base = f"camera_{stamp}"
    n = 1
    while True:
        name = base if n == 1 else f"{base}_{n}"
        mjpeg_path, csv_path = out_dir / f"{name}.mjpeg", out_dir / f"{name}.csv"
        if not mjpeg_path.exists() and not csv_path.exists():
            return mjpeg_path, csv_path
        n += 1


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record the drone camera stream with a live preview")
    parser.add_argument("--url", default=DEFAULT_URL, help=f"stream URL (default {DEFAULT_URL})")
    parser.add_argument("--out-dir", type=Path, default=LOG_DIR, help="output folder (default logs/)")
    parser.add_argument("--no-preview", action="store_true", help="record without the OpenCV window")
    parser.add_argument("--timeout", type=float, default=5.0, help="socket timeout in seconds")
    return parser


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)
    try:
        run(args.url, args.out_dir, preview=not args.no_preview, timeout_s=args.timeout)
    except RuntimeError:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
