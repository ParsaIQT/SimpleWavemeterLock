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

Several wavemeters on one PC (each with its WLM software running): a source
reads one, by its serial number (`serial`; HighFinesse's 'version' number, the
1234 of 'WS7 1234'), and keeps to it: given none, it takes the only one there
is, or the first that nothing else reads, and says which:

    sink.on_wavemeter('WS7', 1234)    the wavemeter it reads (from now on)

A wavemeter it keeps to that is not running is not stood in for by another:
nothing is read until it runs again. The DLL addresses one wavemeter at a time
(PresetWLMIndex), for the whole program: WlmHub has the sources of a PC's
wavemeters take turns, each setting it to its own before each read, and checking
whose readings they were after. Claims keep two copies of the program on a PC
from reading the same one.
"""
from __future__ import annotations

import ctypes
import http.client
import json
import logging
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import NamedTuple
from urllib.parse import urlsplit

from .config import FileLock

log = logging.getLogger(__name__)

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


class Found(NamedTuple):
    """A wavemeter running on this PC (its WLM software)."""
    index: int   # the DLL's for it now (PresetWLMIndex): from 0, as the DLL lists them
    model: str   # 'WS7' ('' if it does not say)
    serial: int  # its serial number (0: it does not say)


def meter_name(model: str, serial) -> str:
    """A wavemeter, as said: 'WS7 1234'."""
    return " ".join(x for x in (model or "", str(serial) if serial else "") if x)


class Source(threading.Thread):
    label = "source"
    model = ""          # what the wavemeter is ('WS7'), if it says
    serial = None       # its serial number, once known
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
    def name(self) -> str:
        """The wavemeter it reads, as said ('WS7 1234'; '' if it does not say)."""
        return meter_name(self.model, self.serial)

    def _set_meter(self, model: str, serial: int):
        """It reads that wavemeter (from now on): said to the sink."""
        if (model, serial) == (self.model, self.serial):
            return
        self.model, self.serial = model, serial
        said = getattr(self.sink, "on_wavemeter", None)
        if said is not None:
            said(model, serial)

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

    preset_used = False  # PresetWLMIndex was called (there were several wavemeters): set before each read since

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
            ("GetWLMVersion", [L], L, False),
            ("PresetWLMIndex", [L], L, False),
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
        return self.count() > 0

    def count(self) -> int:
        """How many wavemeters run on this PC (their WLM software)."""
        return self.lib.GetWLMCount(0)

    def model(self) -> str:
        """'WS7', say ('' if it does not say), of the wavemeter addressed."""
        t = self.lib.GetWLMVersion(0) if "GetWLMVersion" in self.has else 0
        return f"WS{t}" if 4 <= t <= 30 else ""

    def serial(self) -> int:
        """The serial number of the wavemeter addressed (0 if it does not say)."""
        v = self.lib.GetWLMVersion(1) if "GetWLMVersion" in self.has else 0
        return v if v > 0 else 0

    def preset(self, index: int):
        """Address that wavemeter (its index: Found.index) with every call from now on, in this whole program."""
        if "PresetWLMIndex" in self.has:
            self.lib.PresetWLMIndex(index)
            self.preset_used = True

    def scan(self) -> list:
        """The wavemeters running on this PC, in the DLL's order: [Found]. With several, this leaves another one
        addressed: set it again (preset) before reading one."""
        n = self.count()
        if n <= 0:
            return []
        if n == 1 or "PresetWLMIndex" not in self.has or "GetWLMVersion" not in self.has:
            if self.preset_used:
                self.preset(0)
            return [Found(0, self.model(), self.serial())]
        out, seen = [], set()
        for i in range(n + 4):  # (as HighFinesse's own example: indices from 0, until each one is found)
            self.preset(i)
            s = self.serial()
            if s and s not in seen:
                seen.add(s)
                out.append(Found(i, self.model(), s))
                if len(out) == n:
                    break
        return out

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


class Claims:
    """Which copy of this program on this PC reads which wavemeter: a lock file each (FileLock) in folder, held by
    the copy that reads it."""

    def __init__(self, folder):
        self.folder = Path(folder)
        self._held = {}  # serial -> FileLock

    def _path(self, serial: int) -> Path:
        return self.folder / f".simplewavemeterlock-wlm{int(serial)}.lock"

    def acquire(self, serial: int) -> bool:
        """This copy reads it from now on: False if another copy does."""
        lock = self._held.get(serial) or FileLock(self._path(serial))
        try:
            if not lock.acquire():
                return False
        except OSError:  # (no lock file can be made there: no telling, as without claims)
            pass
        self._held[serial] = lock
        return True

    def release(self, serial: int):
        lock = self._held.pop(serial, None)
        if lock is not None:
            lock.release()

    def free(self, serial: int) -> bool:
        """No other copy reads it."""
        if serial in self._held:
            return True
        lock = FileLock(self._path(serial))
        try:
            ok = lock.acquire()
        except OSError:
            return True
        lock.release()
        return ok


class WlmHub:
    """This PC's wavemeters, through one wlmData for the whole program: which run (scan), and which source reads
    which (take, give; with claims, none that another copy of the program reads). The DLL addresses one wavemeter
    at a time, for the whole program: with several, a source sets it to its own before each read, under `lock`."""

    SCAN_S = 0.5  # s: a scan is answered again for this long

    def __init__(self, path: str | None = None, claims: Claims | None = None):
        self.path = path or None
        self.claims = claims
        self.lock = threading.RLock()
        self.wlm = None       # WlmDll, once loaded
        self.found = []       # [Found], as last scanned...
        self._scanned = -1e9  # ...then (monotonic)
        self.taken = {}       # serial -> the source of this program that reads it

    def dll(self) -> WlmDll:
        """The DLL, loaded (OSError if it cannot be)."""
        with self.lock:
            if self.wlm is None:
                self.wlm = WlmDll(self.path)
            return self.wlm

    def scan(self) -> list:
        """The wavemeters running on this PC: [Found] (OSError if the DLL cannot be loaded)."""
        with self.lock:
            now = time.monotonic()
            if now - self._scanned >= self.SCAN_S:
                try:
                    self.found = self.dll().scan()
                except OSError:
                    self.wlm, self.found = None, []
                    raise
                self._scanned = now
            return self.found

    def known(self) -> list:
        """The wavemeters running on this PC as the sources last found them (each looks every second): the
        window's, which never waits for the DLL."""
        return self.found

    def take(self, serial: int, src) -> str:
        """src reads that wavemeter from now on: '' if it may, else why not."""
        with self.lock:
            other = self.taken.get(serial)
            if other is not None and other is not src:
                return "another of this program's panels reads it"
            if other is None and self.claims is not None and not self.claims.acquire(serial):
                return "another copy of this program on this PC reads it"
            self.taken[serial] = src
            return ""

    def give(self, serial, src):
        """src reads it no more."""
        with self.lock:
            if serial is not None and self.taken.get(serial) is src:
                del self.taken[serial]
                if self.claims is not None:
                    self.claims.release(serial)

    def free(self, serial: int) -> bool:
        """Nothing reads that wavemeter: no source of this program, no other copy of it on this PC. (Never waits for
        the lock: the window asks.)"""
        return serial not in self.taken and (self.claims is None or self.claims.free(serial))

    def swap(self, a, b):
        """Sources a and b each read the other's wavemeter from now on (each in a moment, on its thread)."""
        with self.lock:
            sa, sb = a.wanted, b.wanted
            for src, mine, theirs, other in ((a, sa, sb, b), (b, sb, sa, a)):
                if mine is not None and self.taken.get(mine) is src:
                    self.taken[mine] = other  # (at once: neither finds the other in its way)
                src.select(theirs)

    def make_source(self, sink, serial=None, model: str = "", interval: float = 0.002, channels=None):
        """A source for the sink (an engine) that reads that wavemeter (None: the only one, or the first free; one
        of several free ones as the sink chooses: choose_wavemeter)."""
        return DllSource(sink, self.path, interval, channels, serial=serial, model=model, hub=self,
                         choose=getattr(sink, "choose_wavemeter", None))


class DllSource(Source):
    """Reads wlmData.dll directly (run the GUI on the wavemeter PC). Fastest. With several wavemeters on the PC,
    it reads the one it keeps to (serial): the DLL is set to it before each read, and asked whose the readings were
    after (another wavemeter in its place, its WLM software restarted: nothing passed on)."""

    switchable = True
    CHECK_S = 1.0  # s between looks at which wavemeters run, and at the switch

    def __init__(self, sink, path: str | None = None, interval: float = 0.002, channels=None, serial=None,
                 model: str = "", hub: WlmHub | None = None, choose=None):
        """serial: the wavemeter to read (None: the only one, or the first that nothing else reads, kept to from then
        on); model: what it is, while it is not running ('WS7'); hub: shared by the sources of a PC's wavemeters;
        choose([(Found, {ch: THz})]) -> Found or None: which of several free ones, for a source given none (the one
        whose readings its settings know, say)."""
        super().__init__(sink, interval)
        self.path = path or None
        self.fixed = list(channels) if channels else None
        self.hub = hub if hub is not None else WlmHub(path)
        self.wanted = int(serial) if serial else None
        self.wanted_model = model
        self.choose = choose
        self.found = []          # this PC's wavemeters, as last scanned
        self.label = path or "wlmData.dll"
        self._index = None       # the DLL's index of the one it reads
        self._selects = deque()  # another wavemeter to read (select), on this thread

    def select(self, serial):
        """Read that wavemeter from now on (in a moment, on this source's thread)."""
        self._selects.append(int(serial) if serial else None)

    def run(self):
        hub = self.hub
        active, next_check = [], 0.0
        try:
            while not self._stop_ev.is_set():
                now = time.monotonic()
                try:
                    wlm = hub.dll()
                    if self._selects:
                        self._select_now()
                        next_check = 0.0
                    if self._switches:
                        with hub.lock:
                            if self._aim(wlm) and self._do_switches(wlm.set_use):
                                next_check = 0.0  # what the switch does now, at once
                    if now >= next_check:
                        next_check = now + self.CHECK_S
                        why = self._check(wlm)
                        if why:
                            self._set_state(False, why)
                            self._stop_ev.wait(self.CHECK_S)
                            continue
                        with hub.lock:
                            aimed = self._aim(wlm)
                            if aimed:
                                active, off = (self.fixed, []) if self.fixed else wlm.channels()
                        if aimed:
                            self._set_active(active, off)
                            self._set_state(True)
                    raw, mine = {}, False
                    with hub.lock:
                        if self._aim(wlm):
                            for ch in active:
                                v = wlm.frequency(ch)
                                if v != INF_NOTHING_CHANGED:
                                    raw[ch] = v
                            mine = self._aim(wlm, again=True)
                    if mine:
                        self._push(raw, now)
                    else:  # (another wavemeter where it was: which run, at once)
                        next_check = min(next_check, now + 0.1)
                except OSError as e:
                    self._index = None
                    self._set_state(False, f"cannot load wlmData: {e}")
                    self._stop_ev.wait(2.0)
                    continue
                self._stop_ev.wait(self.interval)
        finally:
            hub.give(self.serial, self)

    def _aim(self, wlm, again: bool = False) -> bool:
        """With several wavemeters on this PC (the DLL has been set to one): set it to the one this source reads
        (again: only ask whether it still is that one). False if it is not that one: nothing read is passed on."""
        if self._index is None:  # (the last look found it not there, or not to be read)
            return False
        if not wlm.preset_used:
            return True
        if not self.serial:
            return False
        if not again:
            wlm.preset(self._index)
        return wlm.serial() == self.serial

    def _select_now(self):
        serial = self._selects.pop()
        self._selects.clear()
        if serial == self.wanted:
            return
        self.hub.give(self.serial, self)
        self.wanted, self.wanted_model, self._index = serial, "", None
        self._last, self._stale, self._active, self._off = {}, {}, None, None  # (another wavemeter: all anew)

    def _check(self, wlm) -> str:
        """Which wavemeters run; the one it reads among them, taken: '' if it can read it, else why not."""
        self._index = None
        found = self.found = self.hub.scan()
        if not found:
            return "wlmServer is not running"
        if self.wanted is None:
            if len(found) == 1 and not found[0].serial:  # (a DLL that does not say: the one there is, as ever)
                self._index = found[0].index
                self._set_meter(found[0].model, 0)
                return ""
            free = [f for f in found if f.serial and (self.hub.taken.get(f.serial) is self or self.hub.free(f.serial))]
            if not free:
                return (f"{self._list(found)}: {'it is' if len(found) == 1 else 'each is'} read by another copy of "
                        f"this program on this PC")
            pick = self._choose(wlm, free) if len(free) > 1 and self.choose is not None else free[0]
            self.wanted = pick.serial
        f = next((f for f in found if f.serial == self.wanted), None)
        if f is None:
            name = meter_name(self.wanted_model or self.model, self.wanted)
            return f"{name} is not running" + (f" (this PC has {self._list(found)})" if found else "")
        why = self.hub.take(f.serial, self)
        if why:
            return f"{meter_name(f.model, f.serial)}: {why}"
        self._index = f.index
        self._set_meter(f.model, f.serial)
        return ""

    @staticmethod
    def _list(found) -> str:
        return ", ".join(meter_name(f.model, f.serial) or f"wavemeter {f.index + 1}" for f in found)

    def _choose(self, wlm, free):
        """Of several free wavemeters, the one choose() says (its readings shown to it), else the first."""
        cands = []
        with self.hub.lock:
            for f in free:
                wlm.preset(f.index)
                if wlm.serial() != f.serial:
                    continue
                freqs = {}
                for ch in wlm.channels()[0]:
                    v = wlm.frequency(ch)
                    if v > 0:
                        freqs[ch] = v
                cands.append((f, freqs))
        try:
            pick = self.choose(cands)
        except Exception:  # (whatever it does: never the end of this thread)
            log.exception("choosing a wavemeter")
            pick = None
        return pick if pick in free else free[0]


class NullSource(Source):
    """No wavemeter on this PC (--view): the other PCs' are shown (share.py)."""

    label = "no wavemeter"
    viewer = True

    def __init__(self, sink):
        super().__init__(sink, 1.0)

    def run(self):
        self._set_state(True)
        self._stop_ev.wait()


# -------------------------------------------------------------------------- HTTP
class HttpSource(Source):
    """Polls GET /api/v1/wavelength of server/server.py over one keep-alive connection; switches a channel
    with POST /api/v1/channel/<n>/use (a server.py from before that cannot). On a PC with several wavemeters, it
    asks for the one it keeps to (?wlm=1234), and checks that the answer is that one's."""

    switchable = True

    def __init__(self, sink, url: str, interval: float = 0.02, serial=None, model: str = ""):
        """serial: the wavemeter to read (None: the server's only one, or its first; kept to from then on, if the
        server says which it is); model: what it is ('WS7'), while it is not running."""
        super().__init__(sink, interval)
        u = urlsplit(url if "://" in url else "http://" + url)
        self.host, self.port = u.hostname or "localhost", u.port or 80
        self.path = u.path if u.path not in ("", "/") else "/api/v1/wavelength"
        self.label = f"{self.host}:{self.port}"
        self.wanted = int(serial) if serial else None
        self.wanted_model = model
        self._selects = deque()  # another wavemeter to read (select), on this thread

    def select(self, serial):
        """Read that wavemeter of the server's from now on (in a moment, on this source's thread)."""
        self._selects.append(int(serial) if serial else None)

    def _query(self) -> str:
        return f"?wlm={self.wanted}" if self.wanted is not None else ""

    def _use_path(self, ch: int) -> str:
        base = self.path.rsplit("/", 1)[0] if self.path.endswith("/wavelength") else "/api/v1"
        return f"{base}/channel/{ch}/use{self._query()}"

    def _switch_now(self, conn, ch: int, on: bool) -> str:
        conn.request("POST", self._use_path(ch), body=json.dumps({"use": on}),
                     headers={"Content-Type": "application/json"})
        r = conn.getresponse()
        body = r.read()
        if r.status == 200:
            return ""
        detail = _detail(body)
        if r.status == 405 or (r.status == 404 and detail in ("", "Not Found")):
            return "this server cannot switch channels: update its server.py"
        return f"HTTP {r.status}" + (f": {detail}" if detail else "")

    def run(self):
        conn = None
        while not self._stop_ev.is_set():
            t0 = time.monotonic()
            try:
                if self._selects:
                    self.wanted, self.wanted_model = self._selects.pop(), ""
                    self._selects.clear()
                    self._last, self._stale, self._active, self._off = {}, {}, None, None  # (another: all anew)
                if conn is None:
                    conn = http.client.HTTPConnection(self.host, self.port, timeout=2.0)
                if self._switches:
                    self._do_switches(lambda ch, on, c=conn: self._switch_now(c, ch, on))
                conn.request("GET", self.path + self._query())
                r = conn.getresponse()
                body = r.read()
                if r.status == 503:
                    self._set_state(False, "wlmServer is not running")
                    self._stop_ev.wait(1.0)
                    continue
                if r.status == 404 and self.wanted is not None and _detail(body) not in ("", "Not Found"):
                    self._set_state(False, _detail(body))  # (that wavemeter is not running there)
                    self._stop_ev.wait(1.0)
                    continue
                if r.status != 200:
                    raise OSError(f"HTTP {r.status}")
                data = json.loads(body)
                meter = data.get("wavemeter")
                serial = meter.get("serial") if isinstance(meter, dict) else None
                if isinstance(serial, int) and not isinstance(serial, bool) and serial > 0:
                    if self.wanted is not None and serial != self.wanted:
                        raise ValueError(f"it answered for wavemeter {serial}, not {self.wanted}")
                    self.wanted = serial
                    self._set_meter(str(meter.get("model") or ""), serial)
                elif self.wanted is not None:
                    self._set_state(False, f"{self.label} cannot tell its wavemeters apart: update its server.py")
                    self._stop_ev.wait(1.0)
                    continue
                raw, active, off = {}, [], []
                for key, c in data.get("channels", {}).items():
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


def _detail(body: bytes) -> str:
    """What a server.py's error answer says ('' if nothing)."""
    try:
        detail = json.loads(body).get("detail") or ""
    except (ValueError, AttributeError):
        return ""
    return detail if isinstance(detail, str) else ""


class HttpMeters:
    """The wavemeters a server.py serves (GET /api/v1/wavemeters), asked every SCAN_S in a thread of its own (none
    from a server.py from before, which serves one); a source for each (HttpSource)."""

    SCAN_S = 5.0

    def __init__(self, url: str, interval: float = 0.02):
        self.url, self.interval = url, interval
        u = urlsplit(url if "://" in url else "http://" + url)
        self.host, self.port = u.hostname or "localhost", u.port or 80
        path = u.path if u.path not in ("", "/") else "/api/v1/wavelength"
        self.path = (path.rsplit("/", 1)[0] if path.endswith("/wavelength") else "/api/v1") + "/wavemeters"
        self.found = []
        self._stop_ev = threading.Event()
        self._thread = None

    def scan(self) -> list:
        """The server's wavemeters, as last heard: [Found]."""
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True, name="HttpMeters")
            self._thread.start()
        return self.found

    def stop(self):
        self._stop_ev.set()

    def _run(self):
        while not self._stop_ev.is_set():
            conn = http.client.HTTPConnection(self.host, self.port, timeout=2.0)
            try:
                conn.request("GET", self.path)
                r = conn.getresponse()
                body = r.read()
                if r.status == 200:
                    self.found = [Found(int(m.get("index", k)), str(m.get("model") or ""), int(m["serial"]))
                                  for k, m in enumerate(json.loads(body).get("wavemeters", []))
                                  if isinstance(m, dict) and isinstance(m.get("serial"), int) and m["serial"] > 0]
            except (OSError, http.client.HTTPException, ValueError, AttributeError, TypeError, KeyError):
                pass
            finally:
                conn.close()
            self._stop_ev.wait(self.SCAN_S)

    def free(self, serial: int) -> bool:
        return True

    def make_source(self, sink, serial=None, model: str = "", interval: float | None = None, channels=None):
        return HttpSource(sink, self.url, self.interval if interval is None else interval, serial=serial, model=model)

    @staticmethod
    def swap(a, b):
        wa, wb = a.wanted, b.wanted
        a.select(wb)
        b.select(wa)
