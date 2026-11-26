"""지상국 도구들이 같이 쓰는 저장소 경로."""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = REPO_ROOT / "logs"
