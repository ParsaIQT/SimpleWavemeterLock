"""Settings file: plain JSON, written atomically; held by the copy of the program using it (FileLock)."""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path


def default_path(sim: bool = False) -> Path:
    return Path.home() / (".simplewavemeterlock-sim.json" if sim else ".simplewavemeterlock.json")


def lock_path(path) -> Path:
    """The file a copy of the program holds while it uses the settings file path."""
    path = Path(path)
    return path.with_name(path.name + ".lock")


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


class FileLock:
    """A hold on a file (made if missing) that one holder at a time can have, another program or another FileLock
    of this one: kept until released, or the program ends, however it ends (the system lets go of it then)."""

    def __init__(self, path):
        self.path = Path(path)
        self._f = None

    @property
    def held(self) -> bool:
        return self._f is not None

    def acquire(self) -> bool:
        """Hold it: True if it is held now, False if another holder has it; OSError if the file cannot be made."""
        if self._f is not None:
            return True
        f = open(self.path, "a+b")
        try:
            if os.name == "nt":
                import msvcrt

                f.seek(0)
                msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)  # (a byte past its end: nothing in it is locked)
            else:
                import fcntl

                fcntl.flock(f.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            f.close()
            return False
        self._f = f
        return True

    def release(self):
        f, self._f = self._f, None
        if f is None:
            return
        try:
            if os.name == "nt":
                import msvcrt

                f.seek(0)
                msvcrt.locking(f.fileno(), msvcrt.LK_UNLCK, 1)
        except OSError:
            pass
        f.close()  # (that alone lets go of it elsewhere)
