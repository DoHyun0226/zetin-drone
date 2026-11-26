"""드론 카메라 스트림 공용 로직: MJPEG 파싱, 녹화 index 스키마,
녹화 요약, 카메라-노트북 시계 정렬.

표준 라이브러리만 쓴다. ``camera_record.py``(녹화)와 ``camera_mp4.py``
(후처리)가 이 모듈 위에 있다. 스트림 형식은 ``firmware/camera/esp32cam_stream``이
만든다.
"""

from __future__ import annotations

import datetime
import math
import re
from dataclasses import dataclass


class StreamFormatError(ValueError):
    """바이트 스트림이 multipart/x-mixed-replace 형식을 따르지 않는다."""


@dataclass(frozen=True)
class StreamPart:
    jpeg: bytes
    capture_s: float | None
    seq: int | None
    # 이 part의 HEADERS가 완성된 feed() 호출에 넘긴 값.
    # 녹화기는 여기에 수신 시각을 넘긴다: 카메라가 byte-rate 상한에 맞춰 보내므로
    # 프레임의 마지막 바이트는 헤더보다 최대 ~100 ms 늦게 도착한다. 헤더 도착
    # 시각을 쓰면 그 지연이 시계 정렬에 들어가지 않는다.
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

# 받아들이는 JPEG part의 최대 크기. ESP32-CAM 프레임 버퍼가 담을 수 있는 어떤 것보다
# 훨씬 크다(고화질 UXGA가 수백 kB). 이보다 큰 Content-Length는 깨진 헤더이고,
# 그것을 버퍼링하면 뒤따르는 모든 프레임을 삼킨다.
MAX_JPEG_BYTES = 1_000_000

_BOUNDARY_RE = re.compile(r'boundary\s*=\s*"?([^";\s]+)"?', re.IGNORECASE)


def boundary_from_content_type(value: str) -> bytes:
    match = _BOUNDARY_RE.search(value or "")
    if match is None:
        raise StreamFormatError(f"no multipart boundary in Content-Type {value!r}")
    return match.group(1).encode("ascii")


def format_recv_time(epoch_s: float) -> str:
    """비행 로그 ``Timestamp`` 열과 같은 ``HH:MM:SS.mmm`` 지역 시각 형식."""
    return datetime.datetime.fromtimestamp(epoch_s).strftime("%H:%M:%S.%f")[:-3]


class MjpegParser:
    """카메라 multipart 스트림의 점진적 parser.

    본문은 boundary를 찾지 않고 ``Content-Length``로 읽는다. 그래서 JPEG 바이트에
    우연히 boundary가 들어 있어도 그대로 남는다. 본문 바로 뒤 바이트는 다음
    boundary여야 하며, 아니면 길이가 거짓이었다는 뜻이므로 ``StreamFormatError``를 낸다.

    part는 뒤따르는 boundary를 확인한 뒤에야(또는 ``finish()``에서) 내준다. 그래서
    chunk가 어디서 갈리든 길이가 거짓인 part는 돌려주지 않는다. 같은 ``feed()``
    호출에서 이미 확인된 part를 먼저 돌려주고, 오류는 다음 호출에서 낸다.
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
                    # 쪼개진 boundary의 앞부분일 수 있는 꼬리를 남긴다
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
            else:  # _AFTER: 다음 boundary가 바로 이어져야 한다
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
        """연결이 끝날 때 부른다.

        아직 뒤따르는 boundary를 기다리던 마지막 완전한 part와,
        불완전한 part를 버렸는지를 돌려준다.
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
    if not math.isfinite(number):  # nan / inf는 나중에 시계 정렬을 망가뜨린다
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
    longest_gap_s: float  # 연결 구간 안에서의 최장 간격
    reconnect_gaps_s: tuple[float, ...]  # 구간 k의 마지막 프레임 -> k+1의 첫 프레임


def _segments(rows):
    """구간별로 묶은 행. 녹화 순서대로."""
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
    """녹화 index 요약. 값은 CSV 문자열일 수 있다.

    속도와 간격은 monotonic 수신 시계로 잰다. 그래서 녹화 중 벽시계가 뛰어도(NTP)
    값이 왜곡되지 않는다.
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
    """구간마다 ``min(recv_epoch_s - capture_s)``: 카메라 시계 -> 노트북 시계.

    이 추정값은 실제 offset에 구간 안 최소 전달 지연(한가한 AP에서 수 ms)을
    더한 값이다. 비행 로그 ``Timestamp``도 수신 시각이라 비슷한 편향을 가진다.
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
    """구간마다 마지막 창의 최소 offset에서 첫 창의 최소 offset을 뺀 값.

    구간이 창 두 개보다 짧으면 ``None``.
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
    """노트북 벽시계가 monotonic 시계에 비해 뛴 프레임."""
    jumps: list[int] = []
    prev = None
    for r in rows:
        skew = float(r["recv_epoch_s"]) - float(r["recv_monotonic_s"])
        if prev is not None and abs(skew - prev) > threshold_s:
            jumps.append(int(r["frame"]))
        prev = skew
    return jumps


def aligned_rows(rows) -> list[dict]:
    """행에 노트북 시계 기준 ``capture_epoch_s`` / ``capture_time``을 더한 것."""
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
    """내보낸 영상에서 각 프레임이 화면에 머무는 시간.

    다음 프레임의 노트북 시계 촬영 시각과의 차이. 구간 경계를 넘어서도 같다
    (양쪽 모두 노트북 시계 기준). 촬영 시각이 없는 행은 수신 시각을 쓴다.
    """
    times = [
        float(r["capture_epoch_s"]) if r.get("capture_epoch_s", "") != "" else float(r["recv_epoch_s"])
        for r in aligned
    ]
    durations = [max(min_s, b - a) for a, b in zip(times, times[1:])]
    if times:
        durations.append(last_s)
    return durations
