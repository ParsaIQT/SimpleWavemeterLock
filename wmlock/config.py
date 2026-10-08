"""Settings file: plain JSON, written atomically."""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path


def default_path(sim: bool = False) -> Path:
    return Path.home() / (".simplewavemeterlock-sim.json" if sim else ".simplewavemeterlock.json")


def load(path) -> dict:
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as e:
        print(f"warning: ignoring unreadable settings {path}: {e}", file=sys.stderr)
        return {}


def save(path, data: dict):
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1, sort_keys=True)
    os.replace(tmp, path)
