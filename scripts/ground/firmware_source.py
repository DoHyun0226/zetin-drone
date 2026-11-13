"""비행 펌웨어 소스를 파일 여러 개에 걸쳐 하나처럼 읽는다.

비행 스케치 ``firmware/flight/zetin_fc`` 는 ``.ino``, ``fc/*.h``(펌웨어 모듈),
``logic/*.h``(순수 로직)로 나뉜다. 펌웨어에서 상수를 읽는 도구와 뮤테이션을 심는 시험은
정의가 어느 파일에 있는지 가정하면 안 된다. 이 도우미는 모든 파일을 보고 정확히 한 곳에서만
찾도록 강제한다. 정의가 옮겨지거나 중복되면 조용히 틀린 값을 쓰는 대신 오류가 난다.
"""

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SKETCH_DIR = REPO_ROOT / "firmware" / "flight" / "zetin_fc"


def sketch_files(sketch_dir=SKETCH_DIR):
    """``.ino`` 먼저, 다음 ``fc/*.h`` 와 ``logic/*.h``(정렬), 마지막으로 스케치 폴더
    바로 아래의 헤더."""
    sketch_dir = Path(sketch_dir)
    files = sorted(sketch_dir.glob("*.ino"))
    for sub in ("fc", "logic"):
        files.extend(sorted((sketch_dir / sub).glob("*.h")))
    files.extend(sorted(sketch_dir.glob("*.h")))
    if not files:
        raise FileNotFoundError(f"no sketch sources under {sketch_dir}")
    return files


def find_unique(pattern, sketch_dir=SKETCH_DIR, flags=0):
    """스케치 파일 전체에서 ``pattern`` 의 유일한 ``re.Match`` 를 돌려준다.

    한 곳도 없거나 두 곳 이상이면 ``LookupError``.
    """
    regex = re.compile(pattern, flags)
    found = []
    for path in sketch_files(sketch_dir):
        for match in regex.finditer(path.read_text(encoding="utf-8")):
            found.append((path, match))
    if len(found) != 1:
        where = ", ".join(f"{p.relative_to(Path(sketch_dir))}" for p, _ in found) or "nowhere"
        raise LookupError(
            f"pattern {pattern!r} must match exactly once in the sketch; matched {where}")
    return found[0][1]


def find_unique_file(pattern, sketch_dir=SKETCH_DIR, flags=0):
    """:func:`find_unique` 와 같되 ``(path, match)`` 를 돌려준다."""
    regex = re.compile(pattern, flags)
    found = []
    for path in sketch_files(sketch_dir):
        for match in regex.finditer(path.read_text(encoding="utf-8")):
            found.append((path, match))
    if len(found) != 1:
        where = ", ".join(f"{p.relative_to(Path(sketch_dir))}" for p, _ in found) or "nowhere"
        raise LookupError(
            f"pattern {pattern!r} must match exactly once in the sketch; matched {where}")
    return found[0]


def read_float_constant(name, sketch_dir=SKETCH_DIR):
    """``const``/``constexpr float NAME = <수>f;`` 의 값을 적힌 그대로(예: ``0.03f``)."""
    match = find_unique(
        rf"\b(?:const|constexpr)\s+float\s+{re.escape(name)}\s*=\s*"
        r"([0-9]+(?:\.[0-9]+)?f)\s*;",
        sketch_dir,
    )
    return match.group(1)


def mutate_unique(sketch_dir, old, new):
    """복사한 스케치에서 ``old`` 가 나오는 단 한 곳을 바꾼다.

    ``sketch_dir`` 는 임시 복사본이어야 한다. 바꾼 파일을 돌려준다. ``old`` 가 파일
    전체에서 정확히 한 번 나오지 않으면 ``LookupError``.
    """
    hits = []
    for path in sketch_files(sketch_dir):
        count = path.read_text(encoding="utf-8").count(old)
        hits.extend([path] * count)
    if len(hits) != 1:
        raise LookupError(f"{old!r} must occur exactly once in the sketch; found {len(hits)}")
    path = hits[0]
    path.write_text(path.read_text(encoding="utf-8").replace(old, new), encoding="utf-8")
    return path


def subn_unique(sketch_dir, pattern, replacement):
    """:func:`mutate_unique` 의 정규식판. 바꾼 파일을 돌려준다."""
    path, _ = find_unique_file(pattern, sketch_dir)
    text, n = re.subn(pattern, replacement, path.read_text(encoding="utf-8"), count=1)
    if n != 1:
        raise LookupError(f"pattern {pattern!r} did not substitute")
    path.write_text(text, encoding="utf-8")
    return path
