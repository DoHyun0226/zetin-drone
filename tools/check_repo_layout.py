#!/usr/bin/env python3
import re
import sys
from pathlib import Path
from urllib.parse import unquote


REPO_ROOT = Path(__file__).resolve().parents[1]
LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
MAP_RE = re.compile(r"^\|\s*`([^`]+)`\s*\|\s*`([^`]+)`\s*\|")

LEGACY_TOKENS = (
    "firmware/examples/",
    "firmware/src/",
    "firmware/platformio_config/",
    "Drone_Control_Dualsense.py",
    "Drone_Tuning.py",
    "Drone_Reciever.py",
    "Drone_Monitor.py",
    "Drone_Analasys.py",
    "drone_telemetry.py",
    "dual_imu_pid_debug_receiver.py",
    "Controller_test.py",
    "GPS_Reciever.py",
    "ZETIN_Drone",
)


def _link_target(raw):
    raw = raw.strip()
    if raw.startswith("<") and ">" in raw:
        raw = raw[1 : raw.index(">")]
    else:
        raw = raw.split(maxsplit=1)[0]
    return unquote(raw)


def check_markdown_links(repo, files):
    errors = []
    for source in files:
        text = source.read_text(encoding="utf-8")
        for match in LINK_RE.finditer(text):
            target = _link_target(match.group(1))
            if not target or target.startswith("#"):
                continue
            if re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*:", target):
                continue
            target_path = target.split("#", 1)[0]
            if not target_path:
                continue
            if target_path.startswith("/"):
                errors.append(f"{source.relative_to(repo)}: absolute link {target}")
                continue
            resolved = (source.parent / target_path).resolve()
            if not resolved.exists():
                errors.append(
                    f"{source.relative_to(repo)}: missing link target {target_path}"
                )
    return errors


def check_sketch_names(repo):
    errors = []
    for ino in sorted((repo / "firmware").rglob("*.ino")):
        if ino.parent.name != ino.stem:
            errors.append(
                f"sketch name mismatch: {ino.relative_to(repo)} "
                f"(directory {ino.parent.name!r}, file {ino.stem!r})"
            )
    return errors


def check_migration_map(repo, map_path, expected_rows=44):
    errors = []
    rows = []
    for line_number, line in enumerate(
        map_path.read_text(encoding="utf-8").splitlines(), 1
    ):
        match = MAP_RE.match(line)
        if match and match.group(1) != "Old path":
            rows.append((line_number, match.group(1), match.group(2)))

    if len(rows) != expected_rows:
        errors.append(
            f"migration map has {len(rows)} rows; expected {expected_rows}"
        )

    old_seen = set()
    new_seen = set()
    for line_number, old, new in rows:
        if old in old_seen:
            errors.append(f"migration map line {line_number}: duplicate old path {old}")
        if new in new_seen:
            errors.append(f"migration map line {line_number}: duplicate new path {new}")
        old_seen.add(old)
        new_seen.add(new)
        if (repo / old).exists():
            errors.append(f"migration map line {line_number}: old path still exists {old}")
        if not (repo / new).exists():
            errors.append(f"migration map line {line_number}: new path missing {new}")
    return errors


def check_stale_tokens(repo, files):
    errors = []
    for source in files:
        text = source.read_text(encoding="utf-8")
        for token in LEGACY_TOKENS:
            if token in text:
                errors.append(f"{source.relative_to(repo)}: stale token {token}")
    return errors


def maintained_markdown_files(repo):
    explicit = [
        repo / "README.md",
        repo / "docs/README.md",
        repo / "docs/project_overview.md",
        repo / "docs/firmware_catalog.md",
        repo / "docs/udp_protocol.md",
        repo / "docs/migration_map.md",
        repo / "firmware/README.md",
        repo / "scripts/README.md",
        repo / "logs/README.md",
        repo / "firmware/archive/README.md",
        repo / "firmware/archive/dshot/README.md",
        repo / "firmware/archive/platformio_skeleton/README.md",
        repo / "scripts/archive/README.md",
    ]
    return [path for path in explicit if path.exists()]


def stale_scan_files(repo):
    files = [
        path
        for path in maintained_markdown_files(repo)
        if path != repo / "docs/migration_map.md"
    ]
    files.extend(sorted((repo / "firmware/flight").rglob("*.ino")))
    files.extend(sorted((repo / "firmware/diagnostics").rglob("*.ino")))
    files.extend(sorted((repo / "scripts").glob("*.py")))
    return files


def main():
    errors = []
    markdown = maintained_markdown_files(REPO_ROOT)
    errors.extend(check_markdown_links(REPO_ROOT, markdown))
    errors.extend(check_sketch_names(REPO_ROOT))
    errors.extend(
        check_migration_map(REPO_ROOT, REPO_ROOT / "docs/migration_map.md")
    )
    errors.extend(check_stale_tokens(REPO_ROOT, stale_scan_files(REPO_ROOT)))

    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print("repository layout checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
