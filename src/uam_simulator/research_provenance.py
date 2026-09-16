"""Execution context for future research runs, not a substitute for a source commit."""

from datetime import datetime, timezone
from pathlib import Path
import platform
import shlex
import subprocess
import sys

import numpy as np


def execution_metadata(root: Path, arguments: list[str]) -> dict:
    def git(*args):
        try:
            result = subprocess.run(["git", "-C", str(root), *args], capture_output=True,
                                    text=True, timeout=10, check=False)
            return result.stdout.rstrip("\n") if result.returncode == 0 else None
        except (OSError, subprocess.TimeoutExpired):
            return None

    status = git("status", "--porcelain=v1", "--untracked-files=all")
    return {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit": git("rev-parse", "HEAD"),
        "git_branch": git("symbolic-ref", "--short", "HEAD"),
        "git_dirty": None if status is None else bool(status),
        "git_status": None if status is None else status.splitlines(),
        "python": platform.python_version(), "numpy": np.__version__,
        "platform": platform.platform(), "working_directory": str(Path.cwd()),
        "command": shlex.join([sys.executable, *arguments]),
        "source_snapshot_included": False,
        "reproducibility_note": "Dirty or unavailable Git state is not a recoverable source version; "
                                "commit scientific code before a formal run. Input/code hashes are separate.",
    }
