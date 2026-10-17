"""Wavemeter sources.

A source is a thread that polls the wavemeter as fast as is useful and pushes
*changes* to a sink (the Engine):

    sink.on_active([1, 2, 5], [3])    channels the WLM is measuring, and those switched off
                                      ('Use' unticked in the WLM software) that it can switch on
    sink.on_readings([Reading, ...])  only channels whose value changed
    sink.on_state(ok, message)        connection state of the source
    sink.on_switch_failed(3, True, why)  a switch(3, True) that did not work

A changed value is a new measurement, so the lock acts once per measurement and
never on stale data. A channel switched on again shows its old value until it
is measured anew: that value is not passed on. Frequencies are in THz; the
vacuum wavelength is C_NM_THZ / f, independent of the air/vacuum setting of the
WLM software.

switch(ch, on) ticks or unticks a channel's 'Use' (from the source's thread,
soon), where the source can (`switchable`): the DLL, the HTTP server of
server/server.py, the simulation.
"""
from __future__ import annotations

import ctypes
import http.client
import json
import sys
import threading
import time
from collections import deque
from typing import NamedTuple
from urllib.parse import urlsplit

C_NM_THZ = 299792.458  # speed of light in nm*THz
NAN = float("nan")

# Return codes of GetFrequencyNum (wlmConst.py)
WLM_ERRORS = {
    0: "no value", -1: "no signal", -2: "bad signal", -3: "low signal",
    -4: "overexposed", -5: "WLM missing", -6: "not available", -8: "no pulse",
    -10: "channel n/a", -13: "div by 0", -14: "out of range", -15: "unit n/a",
}
INF_NOTHING_CHANGED = -7
# Return codes of SetSwitcherSignalStates (wlmConst.py, ResERR_...)
WLM_SET_ERRORS = {
    -1: "WLM missing", -2: "could not set", -3: "parameter out of range", -4: "WLM out of resources",
    -5: "WLM internal error", -6: "not available", -7: "WLM busy", -8: "not in measurement mode",
    -10: "channel not available", -11: "channel temporarily not available",
}

# Status strings of server/server.py
HTTP_STATUS = {
    "No Signal": "no signal", "Bad Signal": "bad signal", "Low Signal": "low signal",
    "High Signal": "overexposed", "Out of Range": "out of range", "WLM inactive": "WLM missing",
}


class Reading(NamedTuple):
    ch: int
    freq: float  # THz, NaN when invalid
    status: str  # '' when valid, else e.g. 'low signal'
    t: float     # time.monotonic() when the new value was seen


def decode_wlm(ch: int, value: float, t: float) -> Reading:
    if value > 0:
        return Reading(ch, value, "", t)
    return Reading(ch, NAN, WLM_ERRORS.get(int(value), f"error {int(value)}"), t)


class Source(threading.Thread):
    label = "source"
    switchable = False  # it can switch channels on and off (switch)

    def __init__(self, sink, interval: float):
        super().__init__(daemon=True, name=type(self).__name__)
        self.sink = sink
        self.interval = interval
        self._stop_ev = threading.Event()
        self._last: dict = {}  # ch -> raw value last pushed
        self._stale: dict = {}  # ch -> raw value it had when it stopped being measured (shown until measured anew)
        self._active = None
        self._off = None
        self._state = None
        self._switches = deque()  # (ch, on) to do, on this thread

    def stop(self):
        self._stop_ev.set()

    @property
    def switching(self) -> bool:
        """Something switch() was asked is still to be done."""
        return bool(self._switches)

    def switch(self, ch: int, on: bool):
        """Tick (on) or untick channel ch's 'Use' in the WLM software, from this source's thread, soon; a
        failure is reported to sink.on_switch_failed."""
        self._switches.append((int(ch), bool(on)))

    def _do_switches(self, fn) -> bool:
        """Carry out what switch() was asked, with fn(ch, on) -> '' or why not; True if anything was done. If fn
        raises, the request is kept for another go."""
        done = False
        while self._switches:
            ch, on = self._switches.popleft()
            try:
                why = fn(ch, on)
            except Exception:
                self._switches.appendleft((ch, on))
                raise
            done = True
            if why:
                failed = getattr(self.sink, "on_switch_failed", None)
                if failed is not None:
                    failed(ch, on, why)
        return done

    def _set_state(self, ok: bool, msg: str = ""):
        if (ok, msg) != self._state:
            self._state = (ok, msg)
            self.sink.on_state(ok, msg)

    def _set_active(self, chans, off=()):
        chans = list(chans)
        off = [ch for ch in sorted(off) if ch not in chans]
        if chans != self._active or off != self._off:
            for ch in set(self._active or ()) - set(chans):  # no longer measured: what it shows now is stale
                if ch in self._last:
                    self._stale[ch] = self._last.pop(ch)
            self._active, self._off = chans, off
            self.sink.on_active(chans, off)

    def _push(self, raw: dict, t: float):
        """raw: ch -> hashable raw value; pushes the channels whose value changed (not a channel's value from
        before it stopped being measured, until it shows another)."""
        last, stale = self._last, self._stale
        changed = []
        for ch, v in raw.items():
            if ch in stale:
                if stale[ch] == v:
                    continue
                del stale[ch]
            if last.get(ch, NAN) != v:
                changed.append(ch)
        if changed:
            for ch in changed:
                last[ch] = raw[ch]
            self.sink.on_readings([self._decode(ch, raw[ch], t) for ch in changed])

    def _decode(self, ch, raw, t) -> Reading:
        return decode_wlm(ch, raw, t)


# --------------------------------------------------------------------------- DLL
class WlmDll:
    """ctypes binding for the few wlmData functions we need."""

    def __init__(self, path: str | None = None):
        if sys.platform == "win32":
            lib = ctypes.WinDLL(path or "wlmData.dll")
        else:
            lib = ctypes.CDLL(path or "libwlmData.so")
        L, D, P = ctypes.c_long, ctypes.c_double, ctypes.POINTER(ctypes.c_long)
        self.has = set()
        for name, args, res, required in (
            ("GetWLMCount", [L], L, True),
            ("GetFrequencyNum", [L, D], D, True),
            ("GetChannelsCount", [L], L, False),
            ("GetSwitcherMode", [L], L, False),
            ("GetSwitcherChannel", [L], L, False),
            ("GetSwitcherSignalStates", [L, P, P], L, False),
            ("SetSwitcherSignalStates", [L, L, L], L, False),
        ):
            try:
                f = getattr(lib, name)
            except AttributeError:
                if required:
                    raise OSError(f"{name} missing from wlmData") from None
                continue
            f.argtypes, f.restype = args, res
            self.has.add(name)
        self.lib = lib
        self._use, self._show = ctypes.c_long(), ctypes.c_long()

    def running(self) -> bool:
        return self.lib.GetWLMCount(0) > 0

    def channels(self):
        """(measured, off): the channels the WLM measures, and those its switch has with 'Use' unticked (when it
        cycles, and set_use can tick them)."""
        lib, has = self.lib, self.has
        n = lib.GetChannelsCount(0) if "GetChannelsCount" in has else 1
        if n <= 1:
            return [1], []
        if "GetSwitcherMode" in has and lib.GetSwitcherMode(0) == 0:
            # switch not cycling: only the selected channel is measured
            ch = lib.GetSwitcherChannel(0) if "GetSwitcherChannel" in has else 1
            return [ch if 1 <= ch <= n else 1], []
        if "GetSwitcherSignalStates" not in has:
            return list(range(1, n + 1)), []
        used, off = [], []
        for ch in range(1, n + 1):
            if lib.GetSwitcherSignalStates(ch, ctypes.byref(self._use), ctypes.byref(self._show)) < 0:
                return list(range(1, n + 1)), []
            (used if self._use.value else off).append(ch)
        return used, off if "SetSwitcherSignalStates" in has else []

    def active_channels(self) -> list:
        return self.channels()[0]

    def set_use(self, ch: int, on: bool) -> str:
        """Tick or untick channel ch's 'Use' (its 'Show' left as it is): '' if done, else why not."""
        lib, has = self.lib, self.has
        if "SetSwitcherSignalStates" not in has:
            return "wlmData has no SetSwitcherSignalStates"
        show = 1
        if "GetSwitcherSignalStates" in has and \
                lib.GetSwitcherSignalStates(ch, ctypes.byref(self._use), ctypes.byref(self._show)) >= 0:
            show = self._show.value
        r = lib.SetSwitcherSignalStates(ch, int(bool(on)), show)
        return "" if r >= 0 else f"SetSwitcherSignalStates: {WLM_SET_ERRORS.get(r, f'error {r}')}"

    def frequency(self, ch: int) -> float:
        return self.lib.GetFrequencyNum(ch, 0.0)


class DllSource(Source):
    """Reads wlmData.dll directly (run the GUI on the wavemeter PC). Fastest."""

    switchable = True

    def __init__(self, sink, path: str | None = None, interval: float = 0.002, channels=None):
        super().__init__(sink, interval)
        self.path = path or None
        self.fixed = list(channels) if channels else None
        self.label = path or "wlmData.dll"

    def run(self):
        wlm, active, next_check = None, [], 0.0
        while not self._stop_ev.is_set():
            now = time.monotonic()
            try:
                if wlm is None:
                    wlm = WlmDll(self.path)
                if self._switches and self._do_switches(wlm.set_use):
                    next_check = 0.0  # what the switch does now, at once
                if now >= next_check:
                    next_check = now + 1.0
                    if not wlm.running():
                        self._set_state(False, "wlmServer is not running")
                        self._stop_ev.wait(1.0)
                        continue
                    active, off = (self.fixed, []) if self.fixed else wlm.channels()
                    self._set_active(active, off)
                    self._set_state(True)
                raw = {}
                for ch in active:
                    v = wlm.frequency(ch)
                    if v != INF_NOTHING_CHANGED:
                        raw[ch] = v
                self._push(raw, now)
            except OSError as e:
                wlm = None
                self._set_state(False, f"cannot load wlmData: {e}")
                self._stop_ev.wait(2.0)
                continue
            self._stop_ev.wait(self.interval)


# -------------------------------------------------------------------------- HTTP
class HttpSource(Source):
    """Polls GET /api/v1/wavelength of server/server.py over one keep-alive connection; switches a channel
    with POST /api/v1/channel/<n>/use (a server.py from before that cannot)."""

    switchable = True

    def __init__(self, sink, url: str, interval: float = 0.02):
        super().__init__(sink, interval)
        u = urlsplit(url if "://" in url else "http://" + url)
        self.host, self.port = u.hostname or "localhost", u.port or 80
        self.path = u.path if u.path not in ("", "/") else "/api/v1/wavelength"
        self.label = f"{self.host}:{self.port}"

    def _use_path(self, ch: int) -> str:
        base = self.path.rsplit("/", 1)[0] if self.path.endswith("/wavelength") else "/api/v1"
        return f"{base}/channel/{ch}/use"

    def _switch_now(self, conn, ch: int, on: bool) -> str:
        conn.request("POST", self._use_path(ch), body=json.dumps({"use": on}),
                     headers={"Content-Type": "application/json"})
        r = conn.getresponse()
        body = r.read()
        if r.status == 200:
            return ""
        try:
            detail = json.loads(body).get("detail") or ""
        except (ValueError, AttributeError):
            detail = ""
        if r.status == 405 or (r.status == 404 and detail in ("", "Not Found")):
            return "this server cannot switch channels: update its server.py"
        return f"HTTP {r.status}" + (f": {detail}" if detail else "")

    def run(self):
        conn = None
        while not self._stop_ev.is_set():
            t0 = time.monotonic()
            try:
                if conn is None:
                    conn = http.client.HTTPConnection(self.host, self.port, timeout=2.0)
                if self._switches:
                    self._do_switches(lambda ch, on, c=conn: self._switch_now(c, ch, on))
                conn.request("GET", self.path)
                r = conn.getresponse()
                body = r.read()
                if r.status == 503:
                    self._set_state(False, "wlmServer is not running")
                    self._stop_ev.wait(1.0)
                    continue
                if r.status != 200:
                    raise OSError(f"HTTP {r.status}")
                raw, active, off = {}, [], []
                for key, c in json.loads(body).get("channels", {}).items():
                    status = c.get("status") or ""
                    ch = int(key.rsplit("_", 1)[-1])
                    if status == "Disabled":
                        if c.get("use") is False:  # 'Use' unticked (a server.py that says so can tick it)
                            off.append(ch)
                        continue
                    f = c.get("frequency_thz")
                    raw[ch] = (f if c.get("is_valid") and f else None, status)
                    active.append(ch)
                self._set_active(sorted(active), off)
                self._set_state(True)
                self._push(raw, t0)
            except (OSError, http.client.HTTPException, ValueError, AttributeError) as e:
                if conn is not None:
                    conn.close()
                    conn = None
                self._set_state(False, f"{self.label}: {e or type(e).__name__}")
                self._stop_ev.wait(1.0)
                continue
            self._stop_ev.wait(max(0.0, self.interval - (time.monotonic() - t0)))

    def _decode(self, ch, raw, t):
        f, status = raw
        if f:
            return Reading(ch, float(f), "", t)
        return Reading(ch, NAN, HTTP_STATUS.get(status, status.lower() or "invalid"), t)
