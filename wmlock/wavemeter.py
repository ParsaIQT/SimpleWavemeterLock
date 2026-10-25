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

detail(ch, patterns) has a source (`details`: the DLL, the HTTP server of
server/server.py, the simulation) say more of each new reading of a channel
(a laser map's, lasermap.py), in its extra (Reading.extra, else None):

    power      µW the wavemeter measured (None: it did not say)
    exposure   [ms, ms]: its two arrays' exposure for that reading (on auto, it
               lengthens as the light goes, and readings come that much slower)
    amps       [min1, max1, avg1, min2, max2, avg2]: its fringes' amplitudes
    patterns   with patterns: {kind: (typecode, array)}, its interference
               patterns as its window shows them (kind 0: its Fizeau
               interferometers, or grating; 1: its wide interferometer; with
               its analysis on, 4 and 5: the spectrum it computes, x and y)

and exposure(ch) says a detailed channel's exposure now, between readings.

The WLM software stuck (an exception of its own, say, its dialog waiting for a click: it shows readings, but no new
one comes) or gone (closed, or crashed): where a source can (`restartable`: the DLL, the simulation), and unless told
not to (`restart`), it restarts it, as you would: asked to close, killed if it will not, started again (the same
wavemeter's, by its serial number), its measurement started. Stuck is the readings it shows not changing for STALL_S,
one of them at least a frequency (no light anywhere is not stuck), while it says it measures (stopped by someone is
not stuck either). Gone is not running for GONE_S once it ran. At most once every RESTART_GAP_S, and no more after
GIVE_UP restarts that brought no readings back, until readings come again. Each is said in the log, and in
`restarted`.
"""
from __future__ import annotations

import base64
import ctypes
import http.client
import json
import logging
import os
import re
import subprocess
import sys
import threading
import time
from array import array
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
AMPS = (0, 2, 4, 1, 3, 5)  # GetAmplitudeNum's cMin1, cMax1, cAvg1, cMin2, cMax2, cAvg2 (wlmConst.py)
PATTERNS = (0, 1)          # cSignal1Interferometers, cSignal1WideInterferometer (or cSignal1Grating)
ANALYSIS = (4, 5)          # cSignalAnalysisX, cSignalAnalysisY: the spectrum its analysis computes, if it is on
ITEM = {2: ("H", ctypes.c_uint16), 4: ("I" if array("I").itemsize == 4 else "L", ctypes.c_uint32),
        8: ("d", ctypes.c_double)}  # a pattern's item size in bytes: its array typecode, its ctypes type
C_CTRL_SHOW, C_CTRL_EXIT, C_CTRL_START_SILENT, C_CTRL_SILENT = 0x1, 0x3, 0x20, 0x40  # ControlWLM's (wlmConst.py)
C_MEASUREMENT = 2  # GetOperationState's, and Operation's cCtrlStartMeasurement (wlmConst.py)
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
    extra: dict | None = None  # a detailed channel's (Source.detail): power, exposure, amps, patterns


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
    details = False     # it can say more of a channel's readings (detail)
    restartable = False  # it can restart the WLM software when it is stuck or gone (the module's docstring)
    STALL_S = 30.0      # s: no new reading this long, while showing one (a frequency at least), is stuck
    GONE_S = 15.0       # s: not running this long, once it ran, is gone
    RESTART_GAP_S = 600.0  # s between restarts at least
    GIVE_UP = 3         # restarts in a row that brought no readings back: no more, until readings come again

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
        self._detail = {}         # ch -> its readings said in detail, with their patterns (True) or without
        self._exposure = {}       # ch -> s, a detailed channel's exposure as last seen
        self.pattern_info = {}    # kind -> (items, bytes an item) of the patterns it reads, once known
        self.restart = True       # restart the WLM software when stuck or gone, if restartable
        self.restarted = ""       # the last restart (or try), said ('' none yet)...
        self.restarted_t = 0.0    # ...and when (time.time())
        self.new_t = time.monotonic()  # the last new reading (or the last time it could be read again)
        self._restart_t = -1e9    # monotonic: the last restart tried
        self._restarts = 0        # restarts in a row that brought no readings back...
        self._gave_up = False     # ...as many as GIVE_UP: said
        self._ran = False         # the WLM software ran, while this source read it
        self._gone_t = None       # monotonic: since then it is not running (None: it runs)

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

    def detail(self, ch: int, patterns):
        """Say more of each new reading of channel ch from now on (its extra: the module's docstring), with its
        patterns or (patterns False) without; patterns None: no more."""
        if patterns is None:
            self._detail.pop(int(ch), None)
            self._exposure.pop(int(ch), None)
        else:
            self._detail[int(ch)] = bool(patterns)

    def exposure(self, ch: int):
        """s: a detailed channel's exposure now (its arrays' together), as last seen; None: not known."""
        return self._exposure.get(int(ch))

    def _exposed(self, ch: int, extra):
        exp = (extra or {}).get("exposure")
        if exp and all(isinstance(x, (int, float)) and x >= 0 for x in exp):
            self._exposure[ch] = sum(exp) / 1e3

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
            if ok and not (self._state or (False,))[0]:
                self.new_t = time.monotonic()  # (readable again: STALL_S from now for a new reading)
            self._state = (ok, msg)
            self.sink.on_state(ok, msg)

    # ------------------------------------------------------- the WLM software stuck or gone (the module's docstring)
    def _stuck(self, now: float, measuring=True) -> str:
        """Why the WLM software looks stuck and is to be restarted now ('' if it does not, or is not to be): the
        readings it shows, a frequency among them, unchanged for STALL_S, while it measures (measuring: what it says;
        None: it does not say)."""
        if measuring is False or not (self.restart and self.restartable):
            return ""
        shown = [self._last.get(ch, NAN) for ch in self._active or ()]
        if not any(v > 0 for v in shown if isinstance(v, (int, float))):
            return ""  # (no light on any channel: nothing to measure, not stuck)
        quiet = now - self.new_t
        if quiet < self.STALL_S:
            return ""
        why = f"stuck (no new reading for {quiet:.0f} s, its last ones frozen)"
        return why if self._may_restart(now, why) else ""

    def _gone(self, now: float, running: bool) -> str:
        """Why the WLM software looks gone and is to be started again now ('' if not): not running for GONE_S once
        it ran."""
        if running:
            self._ran, self._gone_t = True, None
            return ""
        if not self._ran:
            return ""  # (never seen running by this source: left as it is)
        if self._gone_t is None:
            self._gone_t = now
        quiet = now - self._gone_t
        if quiet < self.GONE_S or not (self.restart and self.restartable):
            return ""
        why = f"gone (not running for {quiet:.0f} s)"
        return why if self._may_restart(now, why) else ""

    def _may_restart(self, now: float, why: str) -> bool:
        """Not within RESTART_GAP_S of the last restart, nor after GIVE_UP that brought no readings back (said)."""
        if now - self._restart_t < self.RESTART_GAP_S:
            return False
        if self._restarts >= self.GIVE_UP:
            if not self._gave_up:
                self._gave_up = True
                log.error("%s: the WLM software is %s, but %d restarts in a row brought no readings back: not "
                          "restarted again until readings come", self.name or self.label, why, self._restarts)
            return False
        return True

    def _restarting(self, why: str):
        """A restart begins (why: stuck (...) or gone (...)): counted, said."""
        self._restart_t = time.monotonic()
        self._restarts += 1
        log.warning("%s: the WLM software is %s: restarting it", self.name or self.label, why)
        self._set_state(False, f"restarting the WLM software: it was {why}")

    def _restart_over(self, ok: bool, text: str):
        """It is over: said (restarted), and in the log; STALL_S from now for a new reading."""
        self.new_t, self._gone_t = time.monotonic(), None
        self.restarted, self.restarted_t = f"{text} ({time.strftime('%H:%M:%S')})", time.time()
        (log.warning if ok else log.error)("%s: %s", self.name or self.label, text)

    def _set_active(self, chans, off=()):
        chans = list(chans)
        off = [ch for ch in sorted(off) if ch not in chans]
        if chans != self._active or off != self._off:
            for ch in set(self._active or ()) - set(chans):  # no longer measured: what it shows now is stale
                if ch in self._last:
                    self._stale[ch] = self._last.pop(ch)
            self._active, self._off = chans, off
            self.sink.on_active(chans, off)

    def _push(self, raw: dict, t: float, extras=None):
        """raw: ch -> hashable raw value; pushes the channels whose value changed (not a channel's value from
        before it stopped being measured, until it shows another); extras: ch -> its reading's extra (detail)."""
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
            self.new_t, self._restarts, self._gave_up = t, 0, False  # (a new reading: it measures)
            for ch in changed:
                last[ch] = raw[ch]
            out = []
            for ch in changed:
                r = self._decode(ch, raw[ch], t)
                if extras and extras.get(ch) is not None:
                    r = r._replace(extra=extras[ch])
                out.append(r)
            self.sink.on_readings(out)

    def _decode(self, ch, raw, t) -> Reading:
        return decode_wlm(ch, raw, t)


# --------------------------------------------------------------------------- the WLM software's processes
def wlm_processes(serial=None) -> list:
    """The WLM software running on this PC: [(pid, its .exe's path)] of the processes named wlm*.exe (wlm_ws7.exe,
    say), those whose path has the wavemeter's serial number if any has (its folder: 'Wavelength Meter WS7 1234').
    [] where it cannot tell (not Windows)."""
    if sys.platform != "win32":
        return []
    try:
        from ctypes import wintypes

        k32 = ctypes.WinDLL("kernel32", use_last_error=True)

        class Entry(ctypes.Structure):  # PROCESSENTRY32W
            _fields_ = [("dwSize", wintypes.DWORD), ("cntUsage", wintypes.DWORD), ("th32ProcessID", wintypes.DWORD),
                        ("th32DefaultHeapID", ctypes.c_size_t), ("th32ModuleID", wintypes.DWORD),
                        ("cntThreads", wintypes.DWORD), ("th32ParentProcessID", wintypes.DWORD),
                        ("pcPriClassBase", ctypes.c_long), ("dwFlags", wintypes.DWORD),
                        ("szExeFile", ctypes.c_wchar * 260)]

        k32.CreateToolhelp32Snapshot.restype = wintypes.HANDLE
        k32.OpenProcess.restype = wintypes.HANDLE
        snap = k32.CreateToolhelp32Snapshot(0x2, 0)  # (TH32CS_SNAPPROCESS)
        if not snap or snap == ctypes.c_void_p(-1).value:
            return []
        found = []
        try:
            e = Entry()
            e.dwSize = ctypes.sizeof(Entry)
            more = k32.Process32FirstW(snap, ctypes.byref(e))
            while more:
                if re.fullmatch(r"wlm.*\.exe", e.szExeFile, re.IGNORECASE):
                    found.append(int(e.th32ProcessID))
                more = k32.Process32NextW(snap, ctypes.byref(e))
        finally:
            k32.CloseHandle(snap)
        out = []
        for pid in found:
            path = ""
            h = k32.OpenProcess(0x1000, False, pid)  # (PROCESS_QUERY_LIMITED_INFORMATION)
            if h:
                try:
                    buf, size = ctypes.create_unicode_buffer(1024), wintypes.DWORD(1024)
                    if k32.QueryFullProcessImageNameW(h, 0, buf, ctypes.byref(size)):
                        path = buf.value
                finally:
                    k32.CloseHandle(h)
            out.append((pid, path))
    except (OSError, AttributeError, ValueError):
        return []
    return wavemeters_own(out, serial)


def wavemeters_own(procs: list, serial) -> list:
    """Of the WLM software's processes [(pid, path)], that wavemeter's: those with its serial number in their path
    (its folder: 'Wavelength Meter WS7 1234'), if any has; else the one there is; [] if there are several and none
    says (another wavemeter's is never killed)."""
    mine = [x for x in procs if serial and re.search(rf"(?<!\d){int(serial)}(?!\d)", x[1])]
    return mine or (list(procs) if len(procs) == 1 else [])


def kill_process(pid: int) -> bool:
    """Ended at once (TerminateProcess): False if it could not be."""
    if sys.platform != "win32":
        return False
    try:
        from ctypes import wintypes

        k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k32.OpenProcess.restype = wintypes.HANDLE
        h = k32.OpenProcess(0x1, False, int(pid))  # (PROCESS_TERMINATE)
        if not h:
            return False
        try:
            return bool(k32.TerminateProcess(h, 1))
        finally:
            k32.CloseHandle(h)
    except (OSError, AttributeError, ValueError):
        return False


def start_process(path: str) -> bool:
    """That program started, on its own (it outlives this one), in its folder: False if it could not be."""
    try:
        flags = getattr(subprocess, "DETACHED_PROCESS", 0) | getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
        subprocess.Popen([path], cwd=os.path.dirname(path) or None, close_fds=True, creationflags=flags)
        return True
    except (OSError, ValueError):
        return False


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
            ("GetPowerNum", [L, D], D, False),  # (what a laser map reads besides: detail)
            ("GetExposureNum", [L, L, L], L, False),
            ("GetAmplitudeNum", [L, L, L], L, False),
            ("GetPatternItemSize", [L], L, False),
            ("GetPatternItemCount", [L], L, False),
            ("SetPattern", [L, L], L, False),
            ("GetPatternDataNum", [L, L, ctypes.c_void_p], L, False),
            ("GetAnalysisMode", [ctypes.c_bool], ctypes.c_bool, False),
            ("GetAnalysisItemSize", [L], L, False),
            ("GetAnalysisItemCount", [L], L, False),
            ("GetAnalysisData", [L, ctypes.c_void_p], L, False),
            ("SetAnalysis", [L, L], L, False),
            ("GetOperationState", [ctypes.c_ushort], ctypes.c_ushort, False),  # (the WLM software stuck or gone:
            ("Operation", [ctypes.c_ushort], L, False),                         # restarted)
            ("ControlWLM", [L, ctypes.c_ssize_t, L], L, False),
            ("ControlWLMEx", [L, ctypes.c_ssize_t, L, L, L], L, False),
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

    def measuring(self):
        """The wavemeter addressed measures (True), or has been stopped (False); None: it does not say."""
        if "GetOperationState" not in self.has:
            return None
        return self.lib.GetOperationState(0) == C_MEASUREMENT

    def start_measuring(self):
        """Its measurement started (as Start clicked)."""
        if "Operation" in self.has:
            self.lib.Operation(C_MEASUREMENT)

    def exit_server(self, version: int) -> bool:
        """That wavemeter's WLM software asked to close (its serial number: version; 0 the one there is); False if
        this DLL cannot ask."""
        if "ControlWLM" not in self.has:
            return False
        self.lib.ControlWLM(C_CTRL_EXIT, 0, int(version or 0))
        return True

    def start_server(self, version: int) -> bool:
        """That wavemeter's WLM software started (as its installation has it; not waited for); False if this DLL
        cannot."""
        how = C_CTRL_SHOW | C_CTRL_START_SILENT | C_CTRL_SILENT  # (no message boxes waiting for a click)
        if "ControlWLMEx" in self.has:
            self.lib.ControlWLMEx(how, 0, int(version or 0), 0, 0)
        elif "ControlWLM" in self.has:
            self.lib.ControlWLM(how, 0, int(version or 0))
        else:
            return False
        return True

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

    def exposure(self, ch: int):
        """s: channel ch's exposure now, its two arrays' together (None: it does not say)."""
        if "GetExposureNum" not in self.has:
            return None
        ms = [self.lib.GetExposureNum(ch, k, 0) for k in (1, 2)]
        return sum(max(m, 0) for m in ms) / 1e3 if ms[0] > 0 else None

    def details(self, ch: int, buffers=None) -> dict:
        """The extra of channel ch's reading now (the module's docstring); buffers (patterns_on's): its patterns
        too."""
        lib, has, out = self.lib, self.has, {}
        if "GetPowerNum" in has:
            power = lib.GetPowerNum(ch, 0.0)
            out["power"] = power if power > 0 else None
        if "GetExposureNum" in has:
            out["exposure"] = [max(lib.GetExposureNum(ch, k, 0), 0) for k in (1, 2)]
        if "GetAmplitudeNum" in has:
            out["amps"] = [lib.GetAmplitudeNum(ch, k, 0) for k in AMPS]
        if buffers:
            got = {}
            for kind, (code, buf) in buffers.items():
                ok = lib.GetAnalysisData(kind, buf) if kind in ANALYSIS else lib.GetPatternDataNum(ch, kind, buf)
                if ok > 0:
                    a = array(code)
                    a.frombytes(bytes(buf))
                    got[kind] = (code, a)
            out["patterns"] = got
        return out

    def patterns_on(self) -> dict:
        """Its interference patterns exported from now on (SetPattern; with its analysis on, the spectrum that computes
        too), and a buffer for each: {kind: (typecode, buffer)} ({}: it cannot)."""
        lib, has, out = self.lib, self.has, {}
        if {"SetPattern", "GetPatternItemCount", "GetPatternItemSize", "GetPatternDataNum"} <= has:
            for kind in PATTERNS:
                lib.SetPattern(kind, 1)
                n, size = lib.GetPatternItemCount(kind), lib.GetPatternItemSize(kind)
                if n > 0 and size in ITEM and n * size <= 1 << 24:
                    out[kind] = (ITEM[size][0], (ITEM[size][1] * n)())
                else:
                    lib.SetPattern(kind, 0)
        analysis = {"GetAnalysisMode", "SetAnalysis", "GetAnalysisItemCount", "GetAnalysisItemSize", "GetAnalysisData"}
        if analysis <= has and lib.GetAnalysisMode(False):
            for kind in ANALYSIS:
                lib.SetAnalysis(kind, 1)
                n, size = lib.GetAnalysisItemCount(kind), lib.GetAnalysisItemSize(kind)
                if n > 0 and size in ITEM and n * size <= 1 << 24:
                    out[kind] = (ITEM[size][0], (ITEM[size][1] * n)())
                else:
                    lib.SetAnalysis(kind, 0)
        return out

    def patterns_off(self, buffers):
        """No more patterns exported (those patterns_on began)."""
        for kind in buffers:
            if kind in ANALYSIS:
                self.lib.SetAnalysis(kind, 0)
            else:
                self.lib.SetPattern(kind, 0)


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
    after (another wavemeter in its place, its WLM software restarted: nothing passed on). A detailed channel's new
    reading has its extra read at once, and its frequency again after: changed meanwhile, the extra is not passed
    on (it was the next reading's); its patterns are exported (SetPattern) while a detailed channel wants them."""

    switchable = True
    details = True
    restartable = True
    CHECK_S = 1.0  # s between looks at which wavemeters run, and at the switch
    EXIT_S = 15.0  # s for the WLM software to close when asked (else it is killed)...
    START_S = 60.0  # ...and to start again

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
        self._buffers = None     # {kind: (typecode, buffer)} while its patterns are exported (detail)

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
                        missing = why == "wlmServer is not running" or " is not running" in why
                        gone = self._gone(now, not why) if not why or missing else ""
                        if gone:  # (closed, or crashed: started again)
                            self._restart_wlm(wlm, gone, running=False)
                            next_check = 0.0
                            continue
                        if why:
                            self._set_state(False, why)
                            self._stop_ev.wait(self.CHECK_S)
                            continue
                        with hub.lock:
                            aimed = self._aim(wlm)
                            if aimed:
                                active, off = (self.fixed, []) if self.fixed else wlm.channels()
                                measuring = wlm.measuring()
                        if aimed:
                            self._set_active(active, off)
                            stuck = self._stuck(now, measuring)
                            if stuck:  # (an exception of its own, say: restarted)
                                self._restart_wlm(wlm, stuck, running=True)
                                next_check = 0.0
                                continue
                            self._set_state(True)
                    raw, mine, extras = {}, False, None
                    with hub.lock:
                        if self._aim(wlm):
                            for ch in active:
                                v = wlm.frequency(ch)
                                if v != INF_NOTHING_CHANGED:
                                    raw[ch] = v
                            if self._detail or self._buffers:
                                extras = self._details(wlm, raw)
                            mine = self._aim(wlm, again=True)
                    if mine:
                        self._push(raw, now, extras)
                    else:  # (another wavemeter where it was: which run, at once)
                        next_check = min(next_check, now + 0.1)
                except OSError as e:
                    self._index = None
                    self._set_state(False, f"cannot load wlmData: {e}")
                    self._stop_ev.wait(2.0)
                    continue
                self._stop_ev.wait(self.interval)
        finally:
            if self._buffers:
                with hub.lock:
                    try:
                        if self._aim(hub.dll()):
                            hub.dll().patterns_off(self._buffers)
                    except OSError:
                        pass
            hub.give(self.serial, self)

    def _running(self) -> bool:
        """Its wavemeter's WLM software runs (as the DLL lists them)."""
        try:
            found = self.hub.scan()
        except OSError:
            return False
        serial = self.serial or self.wanted
        return any(f.serial == serial for f in found) if serial else bool(found)

    def _waits(self, cond, seconds: float) -> bool:
        """Until cond(), seconds at most: False if it never was (or this source stops)."""
        end = time.monotonic() + seconds
        while not cond():
            if time.monotonic() >= end or self._stop_ev.wait(0.25):
                return False
        return True

    def _restart_wlm(self, wlm, why: str, running: bool):
        """Its wavemeter's WLM software restarted (the module's docstring): asked to close and, if it will not, killed
        (running), then started again, and its measurement."""
        self._restarting(why)
        version = self.serial or self.wanted or 0
        procs = wlm_processes(version)
        if running:
            with self.hub.lock:
                wlm.exit_server(version)
            if not self._waits(lambda: not self._running(), self.EXIT_S):
                killed = [pid for pid, _ in procs if kill_process(pid)]
                if not killed or not self._waits(lambda: not self._running(), 10.0):
                    self._restart_over(False, f"the WLM software ({why}) would not close, nor be killed")
                    return
                log.warning("%s: the WLM software would not close: killed (process %s)", self.name or self.label,
                            ", ".join(map(str, killed)))
        with self.hub.lock:
            started = wlm.start_server(version)
        if not started:  # (a DLL that cannot: the program, where it ran from)
            started = any(start_process(path) for _, path in procs if path)
        if not started or not self._waits(lambda: self._running() and not self._check(wlm), self.START_S):
            self._restart_over(False, f"the WLM software ({why}) did not start again")
            return
        for _ in range(5):  # (it may take a moment to take orders)
            with self.hub.lock:
                if self._aim(wlm) and wlm.measuring() is not False:
                    break
                if self._aim(wlm):
                    wlm.start_measuring()
            self._stop_ev.wait(1.0)
        self._restart_over(True, f"the WLM software restarted (it was {why})")

    def _details(self, wlm, raw: dict) -> dict:
        """The extras of the detailed channels' new readings (and each one's exposure now); its patterns exported
        while one wants them, and no longer."""
        want = any(self._detail.values())
        if want and self._buffers is None:
            self._buffers = wlm.patterns_on()
            self.pattern_info = {k: (len(buf), ctypes.sizeof(buf) // max(len(buf), 1))
                                 for k, (_, buf) in self._buffers.items()}
        elif not want and self._buffers is not None:
            wlm.patterns_off(self._buffers)
            self._buffers = None
        out = {}
        for ch, patterns in list(self._detail.items()):
            exp = wlm.exposure(ch)
            if exp is not None:
                self._exposure[ch] = exp
            v = raw.get(ch)
            if v is None or self._last.get(ch, NAN) == v:
                continue  # (not a new reading)
            extra = wlm.details(ch, self._buffers if patterns else None)
            if wlm.frequency(ch) == v:  # (else the next reading came meanwhile: the extra was its)
                out[ch] = extra
        return out

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
    asks for the one it keeps to (?wlm=1234), and checks that the answer is that one's. A detailed channel's new
    reading is asked for again with all its details (GET /api/v1/channel/<n>/detail): that answer's is the reading
    passed on (it may be a newer one than the poll's), with its extra."""

    switchable = True
    details = True

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
        self.no_detail = False   # the server cannot say a reading in detail (a server.py from before)

    def select(self, serial):
        """Read that wavemeter of the server's from now on (in a moment, on this source's thread)."""
        self._selects.append(int(serial) if serial else None)

    def _details(self, conn, raw: dict) -> dict:
        """The extras of the detailed channels' new readings, asked of the server; raw takes the frequency and status
        of its answer (a newer reading, perhaps, than the poll's)."""
        out = {}
        base = self.path.rsplit("/", 1)[0] if self.path.endswith("/wavelength") else "/api/v1"
        for ch, patterns in list(self._detail.items()):
            if self.no_detail or ch not in raw or self._last.get(ch, NAN) == raw[ch]:
                continue
            q = f"?patterns={int(patterns)}" + (f"&wlm={self.wanted}" if self.wanted is not None else "")
            conn.request("GET", f"{base}/channel/{ch}/detail{q}")
            r = conn.getresponse()
            body = r.read()
            if r.status in (404, 405) and _detail(body) in ("", "Not Found"):
                self.no_detail = True  # (a server.py from before: readings without their details)
                continue
            if r.status != 200:
                continue
            d = json.loads(body)
            f = d.get("frequency_thz")
            raw[ch] = (f if d.get("is_valid") and f else None, d.get("status") or "")
            extra = {"power": d.get("power_uw"), "exposure": d.get("exposure_ms"), "amps": d.get("amplitudes")}
            if patterns:
                extra["patterns"] = {}
                for kind, pat in (d.get("patterns") or {}).items():
                    a = array(str(pat.get("type") or "H"))
                    a.frombytes(base64.b64decode(pat.get("data") or ""))
                    if sys.byteorder != "little":
                        a.byteswap()
                    extra["patterns"][int(kind)] = (a.typecode, a)
                    self.pattern_info.setdefault(int(kind), (len(a), a.itemsize))
            self._exposed(ch, extra)
            out[ch] = extra
        return out

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
                extras = self._details(conn, raw) if self._detail else None
                self._set_active(sorted(active), off)
                self._set_state(True)
                self._push(raw, t0, extras)
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
