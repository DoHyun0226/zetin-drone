"""Read the flight firmware's source as one text, across all its files.

The flight sketch ``firmware/flight/zetin_fc`` is split into the ``.ino``,
``fc/*.h`` (firmware modules) and ``logic/*.h`` (pure logic). Tools that read
a constant from the firmware, and tests that plant a mutation in it, must not
assume which file a definition lives in. These helpers look in every file
and insist on exactly one match, so a moved or duplicated definition is an
error instead of a silently wrong value.
"""

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SKETCH_DIR = REPO_ROOT / "firmware" / "flight" / "zetin_fc"


def sketch_files(sketch_dir=SKETCH_DIR):
    """The ``.ino`` first, then ``fc/*.h`` and ``logic/*.h`` (sorted), then
    any header directly in the sketch folder."""
    sketch_dir = Path(sketch_dir)
    files = sorted(sketch_dir.glob("*.ino"))
    for sub in ("fc", "logic"):
        files.extend(sorted((sketch_dir / sub).glob("*.h")))
    files.extend(sorted(sketch_dir.glob("*.h")))
    if not files:
        raise FileNotFoundError(f"no sketch sources under {sketch_dir}")
    return files


def find_unique(pattern, sketch_dir=SKETCH_DIR, flags=0):
    """Return the one ``re.Match`` of ``pattern`` across the sketch files.

    Raises ``LookupError`` when it matches nowhere or more than once.
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
    """Like :func:`find_unique` but return ``(path, match)``."""
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
    """Value of ``const``/``constexpr float NAME = <number>f;`` as written (e.g. ``0.03f``)."""
    match = find_unique(
        rf"\b(?:const|constexpr)\s+float\s+{re.escape(name)}\s*=\s*"
        r"([0-9]+(?:\.[0-9]+)?f)\s*;",
        sketch_dir,
    )
    return match.group(1)


def mutate_unique(sketch_dir, old, new):
    """Replace the one literal occurrence of ``old`` in a copied sketch.

    ``sketch_dir`` must be a scratch copy. Returns the file that was changed.
    Raises ``LookupError`` unless ``old`` occurs exactly once across the files.
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
    """Regex form of :func:`mutate_unique`; returns the changed file."""
    path, _ = find_unique_file(pattern, sketch_dir)
    text, n = re.subn(pattern, replacement, path.read_text(encoding="utf-8"), count=1)
    if n != 1:
        raise LookupError(f"pattern {pattern!r} did not substitute")
    path.write_text(text, encoding="utf-8")
    return path
