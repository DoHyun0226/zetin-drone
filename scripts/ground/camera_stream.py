"""Shared logic for the drone camera stream: MJPEG parsing, the recording index
schema, recording summaries and camera-to-laptop clock alignment.

Standard library only. ``record_camera.py`` (recording) and ``camera_to_mp4.py``
(post-processing) build on this module; the stream format is produced by
``firmware/camera/esp32cam_stream``.
"""

from __future__ import annotations

import datetime
import math
import re
from dataclasses import dataclass


class StreamFormatError(ValueError):
    """The byte stream does not follow the multipart/x-mixed-replace format."""


@dataclass(frozen=True)
class StreamPart:
    jpeg: bytes
    capture_s: float | None
    seq: int | None
    # Value passed to the feed() call in which this part's HEADERS completed.
    # The recorder passes the receive time here: the camera paces its sends at
    # the byte-rate cap, so the last byte of a frame arrives up to ~100 ms after
    # the header. Header arrival keeps that pacing delay out of clock alignment.
    tag: object = None


INDEX_FIELDS = (
    "frame",
    "segment",
    "camera_seq",
    "offset",
    "size",
    "capture_s",
    "recv_epoch_s",
    "recv_monotonic_s",
    "recv_time",
)

# Largest JPEG part accepted. Far above anything the ESP32-CAM frame buffer can
# hold (UXGA at high quality is a few hundred kB); a larger Content-Length is a
# corrupt header, and buffering it would swallow every later frame.
MAX_JPEG_BYTES = 1_000_000

_BOUNDARY_RE = re.compile(r'boundary\s*=\s*"?([^";\s]+)"?', re.IGNORECASE)


def boundary_from_content_type(value: str) -> bytes:
    match = _BOUNDARY_RE.search(value or "")
    if match is None:
        raise StreamFormatError(f"no multipart boundary in Content-Type {value!r}")
    return match.group(1).encode("ascii")


def format_recv_time(epoch_s: float) -> str:
    """Same ``HH:MM:SS.mmm`` local-time form as the flight log ``Timestamp`` column."""
    return datetime.datetime.fromtimestamp(epoch_s).strftime("%H:%M:%S.%f")[:-3]


class MjpegParser:
    """Incremental parser for the camera's multipart stream.

    Bodies are read by ``Content-Length``, never by searching for the boundary,
    so JPEG bytes that happen to contain the boundary are kept intact. The bytes
    right after a body must be the next boundary; anything else means the length
    lied and raises ``StreamFormatError``.

    A part is handed out only once that following boundary has been checked (or
    at ``finish()``), so a part whose length lied is never returned, wherever
    the chunks happen to split. Parts already checked in the same ``feed()``
    call are returned first and the error is raised on the next call.
    """

    _SEEK, _HEADERS, _BODY, _AFTER = range(4)

    def __init__(self, boundary: bytes):
        self._open = b"--" + boundary + b"\r\n"
        self._delim = b"\r\n" + self._open
        self._buf = bytearray()
        self._state = self._SEEK
        self._headers: dict[str, str] = {}
        self._length = 0
        self._tag: object = None
        self._pending: StreamPart | None = None
        self._error: StreamFormatError | None = None

    def feed(self, data: bytes, tag: object = None) -> list[StreamPart]:
        if self._error is not None:
            error, self._error = self._error, None
            raise error
        self._buf += data
        parts: list[StreamPart] = []
        while True:
            if self._state == self._SEEK:
                at = self._buf.find(self._open)
                if at < 0:
                    # keep a tail that may be the start of a split boundary
                    keep = len(self._open) - 1
                    if len(self._buf) > keep:
                        del self._buf[: len(self._buf) - keep]
                    return parts
                del self._buf[: at + len(self._open)]
                self._state = self._HEADERS
            elif self._state == self._HEADERS:
                end = self._buf.find(b"\r\n\r\n")
                if end < 0:
                    return parts
                self._headers = _parse_headers(bytes(self._buf[:end]))
                del self._buf[: end + 4]
                self._length = _content_length(self._headers)
                self._tag = tag
                self._state = self._BODY
            elif self._state == self._BODY:
                if len(self._buf) < self._length:
                    return parts
                jpeg = bytes(self._buf[: self._length])
                del self._buf[: self._length]
                self._pending = StreamPart(
                    jpeg,
                    _optional_float(self._headers.get("x-timestamp")),
                    _optional_int(self._headers.get("x-frame-seq")),
                    self._tag,
                )
                self._state = self._AFTER
            else:  # _AFTER: the next boundary must follow immediately
                have = min(len(self._buf), len(self._delim))
                if bytes(self._buf[:have]) != self._delim[:have]:
                    self._pending = None
                    self._buf.clear()
                    self._state = self._SEEK
                    error = StreamFormatError("part body is not followed by the boundary (Content-Length mismatch)")
                    if parts:
                        self._error = error
                        return parts
                    raise error
                if have < len(self._delim):
                    return parts
                del self._buf[: len(self._delim)]
                parts.append(self._pending)
                self._pending = None
                self._state = self._HEADERS

    def finish(self) -> tuple[list[StreamPart], bool]:
        """Call when the connection ends.

        Returns the last complete part still waiting for its trailing boundary,
        and whether an incomplete part was dropped.
        """
        remaining = [self._pending] if self._pending is not None else []
        truncated = self._state == self._BODY or (self._state == self._HEADERS and len(self._buf) > 0)
        self._buf.clear()
        self._state = self._SEEK
        self._pending = None
        self._error = None
        return remaining, truncated


def _parse_headers(block: bytes) -> dict[str, str]:
    headers: dict[str, str] = {}
    for line in block.decode("latin-1").split("\r\n"):
        name, sep, value = line.partition(":")
        if sep:
            headers[name.strip().lower()] = value.strip()
    return headers


def _content_length(headers: dict[str, str]) -> int:
    raw = headers.get("content-length")
    if raw is None:
        raise StreamFormatError("part without Content-Length")
    if not raw.isdigit():
        raise StreamFormatError(f"non-numeric Content-Length {raw!r}")
    length = int(raw)
    if length > MAX_JPEG_BYTES:
        raise StreamFormatError(f"Content-Length {length} exceeds {MAX_JPEG_BYTES}")
    return length


def _optional_float(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except ValueError:
        raise StreamFormatError(f"bad X-Timestamp {value!r}") from None
    if not math.isfinite(number):  # nan / inf would break clock alignment later
        raise StreamFormatError(f"bad X-Timestamp {value!r}")
    return number


def _optional_int(value):
    if value is None or value == "":
        return None
    try:
        return int(value)
    except ValueError:
        raise StreamFormatError(f"bad X-Frame-Seq {value!r}") from None


@dataclass(frozen=True)
class RecordingSummary:
    frames: int
    segments: int
    seq_gaps: int
    effective_fps: float
    longest_gap_s: float  # longest gap INSIDE a connection segment
    reconnect_gaps_s: tuple[float, ...]  # last frame of segment k -> first of k+1


def _segments(rows):
    """Rows grouped by segment, in recording order."""
    groups: list[list[dict]] = []
    current = None
    for r in rows:
        seg = int(r["segment"])
        if seg != current:
            groups.append([])
            current = seg
        groups[-1].append(r)
    return groups


def summarize(rows) -> RecordingSummary:
    """Summary of a recording index. Values may be CSV strings.

    Rates and gaps use the monotonic receive clock so a wall-clock jump (NTP)
    during the recording does not distort them.
    """
    groups = _segments(rows)
    seq_gaps = 0
    intervals = 0
    span = 0.0
    longest = 0.0
    reconnects: list[float] = []
    prev_last = None
    for group in groups:
        mono = [float(r["recv_monotonic_s"]) for r in group]
        if prev_last is not None:
            reconnects.append(mono[0] - prev_last)
        prev_last = mono[-1]
        for a, b in zip(mono, mono[1:]):
            longest = max(longest, b - a)
        if len(mono) >= 2:
            intervals += len(mono) - 1
            span += mono[-1] - mono[0]
        seqs = [int(r["camera_seq"]) for r in group if r.get("camera_seq", "") != ""]
        for a, b in zip(seqs, seqs[1:]):
            seq_gaps += max(0, b - a - 1)
    return RecordingSummary(
        frames=len(rows),
        segments=len(groups),
        seq_gaps=seq_gaps,
        effective_fps=intervals / span if span > 0 else 0.0,
        longest_gap_s=longest,
        reconnect_gaps_s=tuple(reconnects),
    )


def _capture(r):
    value = r.get("capture_s", "")
    return None if value in ("", None) else float(value)


def estimate_offsets(rows) -> dict[int, float]:
    """Per segment, ``min(recv_epoch_s - capture_s)``: camera clock -> laptop clock.

    The estimate is the true offset plus the smallest delivery delay in the
    segment (a few ms on an idle AP). The flight log ``Timestamp`` is also a
    receive time, so it carries a similar bias.
    """
    offsets: dict[int, float] = {}
    for r in rows:
        cap = _capture(r)
        if cap is None:
            continue
        seg = int(r["segment"])
        delay = float(r["recv_epoch_s"]) - cap
        if seg not in offsets or delay < offsets[seg]:
            offsets[seg] = delay
    return offsets


def drift_report(rows, window_s: float = 60.0) -> dict[int, float | None]:
    """Per segment, min offset of the last window minus min offset of the first.

    ``None`` when the segment is shorter than two windows.
    """
    report: dict[int, float | None] = {}
    for group in _segments(rows):
        seg = int(group[0]["segment"])
        pts = [(c, float(r["recv_epoch_s"]) - c) for r in group if (c := _capture(r)) is not None]
        if not pts or pts[-1][0] - pts[0][0] < 2 * window_s:
            report[seg] = None
            continue
        start, end = pts[0][0], pts[-1][0]
        first = min(o for c, o in pts if c < start + window_s)
        last = min(o for c, o in pts if c >= end - window_s)
        report[seg] = last - first
    return report


def clock_jumps(rows, threshold_s: float = 0.5) -> list[int]:
    """Frames where the laptop wall clock jumped against its monotonic clock."""
    jumps: list[int] = []
    prev = None
    for r in rows:
        skew = float(r["recv_epoch_s"]) - float(r["recv_monotonic_s"])
        if prev is not None and abs(skew - prev) > threshold_s:
            jumps.append(int(r["frame"]))
        prev = skew
    return jumps


def aligned_rows(rows) -> list[dict]:
    """Rows plus ``capture_epoch_s`` / ``capture_time`` on the laptop clock."""
    offsets = estimate_offsets(rows)
    out: list[dict] = []
    for r in rows:
        cap = _capture(r)
        seg = int(r["segment"])
        aligned = dict(r)
        if cap is None or seg not in offsets:
            aligned["capture_epoch_s"] = ""
            aligned["capture_time"] = ""
        else:
            epoch = cap + offsets[seg]
            aligned["capture_epoch_s"] = f"{epoch:.6f}"
            aligned["capture_time"] = format_recv_time(epoch)
        out.append(aligned)
    return out


def frame_durations(aligned: list[dict], last_s: float = 0.1, min_s: float = 0.001) -> list[float]:
    """How long each frame stays on screen in an exported video.

    The difference to the next frame's laptop-clock capture time, across
    segment boundaries too (both sides are on the laptop clock). Rows without a
    capture time fall back to the receive time.
    """
    times = [
        float(r["capture_epoch_s"]) if r.get("capture_epoch_s", "") != "" else float(r["recv_epoch_s"])
        for r in aligned
    ]
    durations = [max(min_s, b - a) for a, b in zip(times, times[1:])]
    if times:
        durations.append(last_s)
    return durations
