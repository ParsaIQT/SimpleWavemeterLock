"""Taking a laser over from its DLC pro's cavity lock (and from its SHG cavity's lock), and handing
it back the way you would by hand.

By hand, for a TA-SHG pro whose master is locked to a reference cavity (PDH through a FALC pro):
  1  unlock the master and the SHG cavity, stop the master's scan
  2  scan the SHG cavity and look at its intra-cavity signal: is the master single-mode?
  3  lock the SHG cavity
  4  start the master's scan, bring the right reference-cavity peak into it, and select that
     peak as the lockpoint in the XY display (scan output against Fine In 1)
  5  null the error signal's DC level with the FALC pro's input offset (some 0.01 V)
  6  lock the master
Here 1 is take_over; 2 happens throughout the recentering (Recenter.map, check and mode_ok);
3 is lock_shg, which also makes sure the intra-cavity signal comes up as high as usual (two
modes of about equal power can look like one clean mode in a sweep, but not there); 4-6
relock_lockpoint, which also makes sure the wavemeter and the transmission agree that it
locked to the right peak. relock_plain is for a lock that finds its resonance by
itself (lock without lockpoint, or ReLock): it re-enables it and lets the wavemeter judge;
with a FALC pro it has the scan on for a moment first, to null the error signal's DC level (5).

The transmission (the input its lock detection watches, Fine In 1 as a rule): while the master
scans in a relock (by lockpoint, or for the FALC pro before one without), its level between the
peaks is measured (the zero: any DC offset), and once locked by lockpoint, its level locked. The
engine watches it against those two.

Every write goes through Runner, which refuses anything but WRITABLE (modehop.py) and puts it
back if the recovery fails; a cavity lock that was turned off stays off then. The scope
channels are borrowed only when they do not already show what is needed, and given back.

These are generator methods of Recenter (modehop.py), which mixes this class in.
"""
from __future__ import annotations

import math
import statistics

from . import cavity
from . import modehop as mh
from .dlcpro import (IO_VALUE, OUT_CC, SCAN_OFFSET, SCAN_OUT, SHG_INTRA, DecopError, drives, float_arrays, lock_points,
                     output_name, signal_name)

LOCK = "dl:lock:lock-enabled"
SCAN = "scan:enabled"
SHG_LOCK = "nlo:shg:lock:lock-enabled"
SHG_SCAN = "nlo:shg:scan:enabled"
SCAN_X = (34, 50, 101)  # signals that are the scan output / the piezo voltage
SHG_PIEZO = (90, 91)    # the SHG cavity's piezo voltage: its scan
TRANS = ("dl:lock:window:enabled", "dl:lock:window:input-channel", "dl:lock:spectrum-input-channel")


def says_locked(text) -> bool:
    return isinstance(text, str) and text.strip().lower().startswith("locked")


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def transmission_input(window_on, window_in, spectrum_in):
    """The parameter holding the present value of the cavity transmission, from TRANS: the input its lock
    detection (the lock's window) watches if that is on, else the lock input (the XY display's y, the peaks),
    else the window's; None if none of them can be read."""
    order = (window_in, spectrum_in) if window_on else (spectrum_in, window_in)
    return next((IO_VALUE[s] for s in order if isinstance(s, int) and not isinstance(s, bool) and s in IO_VALUE),
                None)


def transmission_signal(window_on, window_in, spectrum_in):
    """The signal id of the cavity transmission, from TRANS (as transmission_input chooses it, for the
    recorder): the input its lock detection watches if that is on, else the lock input; None if neither is
    known."""
    order = (window_in, spectrum_in) if window_on else (spectrum_in, window_in)
    ok = [s for s in order if isinstance(s, int) and not isinstance(s, bool) and s >= 0]
    return next((s for s in ok if s in IO_VALUE), ok[0] if ok else None)


def usable_zero(zero: float, locked: float) -> float:
    """The zero to go by, V: the one measured, unless it is not well below the level locked (a scan that spends
    its time on the peak, say: then 0)."""
    return zero if locked > 0 and zero < 0.5 * locked else 0.0


def volts(x: float, scale: float = 0.0) -> str:
    """x (V) with the decimals a level of about `scale` V needs (x's own, without): three at least."""
    s = abs(scale or x)
    d = 3 if not s or s >= 0.1 else min(2 - math.floor(math.log10(s)), 6)
    return f"{x:.{d}f}"


class CavitySteps:
    SHG_LOCK_S = 10.0   # s for the SHG cavity to lock
    LOCK_S = 20.0       # s for the cavity lock to report locked
    PEAK_LOOKS = 4      # scans looked at, at most, to tell the right cavity peak
    SETTLE_S = 0.5      # s after starting a scan or changing a scope channel, for fresh traces
    FALC_TOL = 0.002    # V: an error-signal DC level within this (or within 3 times its scatter) is left alone
    FALC_NOISE = 0.0003  # V: the scatter of a look at the DC level taken to be at least this
    FALC_TRACES = 3     # traces per look at the DC level (their median)
    FALC_PROBE = (0.1, 0.3, 0.7, 1.5)  # how much the input offset moves the error signal not known: moved this
    #                     much of the way (as if one to one) until the error signal shows it
    FALC_STEPS = 5      # steps at most once that is known
    ZERO_WAITS = (0.05, 0.08, 0.13, 0.07, 0.11, 0.17, 0.06, 0.09, 0.14, 0.1)  # s between looks at the transmission
    #                     while the master scans (uneven: never in step with the scan); their median is its zero
    LEVEL_WAITS = (0.05, 0.08, 0.13, 0.07, 0.11, 0.06)  # ...and locked: their median is its level locked

    def __init__(self, shg: bool):
        self.shg = shg
        self.shg_ok = shg          # its SHG sweep can be read
        ref = self.p.shg_ref
        self.shg_ref = ref if ref.known else None
        self.best_area = 0.0       # the biggest main peak in an SHG sweep in this recovery (the second biggest)
        self._areas = []
        self.sweep_tells = bool(self.p.shg_area or self.p.shg_lock)  # its SHG sweep shows resonances when it lases
        self.top_seen = 0.0        # V: the tallest resonance in an SHG sweep in this recovery
        self.shg_key = "y"         # the SHG scope's channel with the intra-cavity signal
        self.shg_scanning = False
        self.last_sweep = None
        self.orig = {}             # what the controller had when the recovery began
        self.relock_mode = "plain"
        self.scan_after_unlock = False
        self.lp = None             # (x, y, type) of the lockpoint chosen by hand, if any
        self.lp_kind = self.p.lp_kind
        self.peak_y = self.p.peak_y
        self.falc_name, self.falc_orig = "", 0.0
        self.falc_now = 0.0        # V: the input offset as it is now (a relock may have moved it)
        self.falc_failed = False   # the last relock failed with the input offset moved: the next goes back
        self.scope = {}            # scope channel -> signal shown now (what we may have changed)
        self.shg_level = None      # V: the intra-cavity signal with the SHG cavity locked, at the start
        self.mode_text = ""        # how single-mode it was when handed back
        self.falc_text = ""
        self.relock_trouble = ""   # why the last relock did not work
        self.unlocked = False      # its cavity lock is off, turned off by the recovery
        self.doubt = False         # its SHG cavity would not lock as it should: not single-mode after all
        self.explored = 0          # times the walk has trimmed the current for that
        self.shg_text = ""         # its SHG signal short of usual, handed back all the same
        self._find_refused = False  # the DLC pro would not find-candidates
        self.lock_e = math.nan     # MHz: where the wavemeter puts the laser, relocked (the target re-centered on it)
        self.zero = None           # V: the transmission off resonance, measured while the master scans
        self._blobs = {}

    # ------------------------------------------------------------ taking over
    def take_over(self):
        names = [LOCK, "dl:lock:state-txt", SCAN, "scan:amplitude", "scan:frequency", "nlo:shg:scan:frequency"]
        if self.cavity:
            names += ["dl:lock:lock-without-lockpoint", "dl:lock:candidates", "dl:lock:lockpoint:position",
                      "dl:lock:lockpoint:type", "dl:lock:error-channel", "dl:lock:falc-selection",
                      "scope:channel1:signal", "scope:channel2:signal", "scope:channelx:xy-signal",
                      *TRANS] + [f"falc{k}:input:offset" for k in range(1, 5)]
        if self.shg:
            names += [SHG_LOCK, SHG_SCAN, "nlo:shg:lock:state-txt", "nlo:shg:scope:channel1:signal",
                      "nlo:shg:scope:channel2:signal", "nlo:pd:shg-int:photodiode", "nlo:pd:shg:power"]
        values = yield mh.Get(tuple(names))
        o = self.orig = dict(zip(names, values))
        self.scope = {k: o.get(k) for k in ("scope:channel1:signal", "scope:channel2:signal",
                                            "nlo:shg:scope:channel1:signal", "nlo:shg:scope:channel2:signal")}
        if self.cavity:
            self._decide(o)
        if self.shg and not isinstance(o.get(SHG_LOCK), bool):
            self.shg = self.shg_ok = False
            self.note("this laser has no SHG stage (no nlo:shg:lock): no SHG checks")
        if self.shg and not isinstance(o.get("nlo:shg:scope:channel1:signal"), int):
            self.shg_ok = False
            self.note("cannot read the SHG cavity's scope: no SHG checks")
        level = o.get("nlo:pd:shg-int:photodiode")
        if self.shg and o.get(SHG_LOCK) and says_locked(o.get("nlo:shg:lock:state-txt")) and _num(level) and level > 0:
            self.shg_level = level
            self.sweep_tells = True
        what = []
        if self.cavity:
            what.append(f"turn its cavity lock off (now {o.get('dl:lock:state-txt') or '?'})")
        if self.shg and o.get(SHG_LOCK):
            what.append("turn the SHG cavity's lock off")
        what.append("stop the scan if it runs")
        yield from self.ask("Take the laser over: " + ", ".join(what))
        self.stage = "taking over"
        if self.cavity:
            yield mh.Set(LOCK, False, restore=False)
            self.unlocked = True
        if self.shg and o.get(SHG_LOCK):
            yield mh.Set(SHG_LOCK, False, restore=True)
        if self.cavity or o.get(SCAN):  # unlocked, a DLC pro may let its scan run: the wavemeter needs it still
            (scan,) = yield mh.Get((SCAN,))
            self.scan_after_unlock = bool(scan)
            if scan:
                yield mh.Set(SCAN, False, restore=True)
        if self.shg:
            (scanning,) = yield mh.Get((SHG_SCAN,))
            if scanning:
                yield mh.Set(SHG_SCAN, False, restore=True)
        self.note("took over: " + ", ".join(w.replace("turn ", "turned ").replace("stop the scan if it runs",
                                                                                 "scan stopped")
                                            for w in what))

    def _decide(self, o):
        """How to relock, on which peak height and lockpoint type, and with which FALC pro."""
        p = self.p
        mode = p.relock if p.relock in ("lockpoint", "plain") else (
            "plain" if o.get("dl:lock:lock-without-lockpoint") else "lockpoint")
        if mode == "lockpoint" and not isinstance(o.get("dl:lock:candidates"), bytes):
            self.note("the DLC pro shows no lockpoint candidates: relocking by re-enabling its lock")
            mode = "plain"
        self.relock_mode = mode
        try:
            picked = lock_points(o["dl:lock:candidates"]).get("l") if mode == "lockpoint" else None
        except DecopError:
            picked = None
        if picked:
            self.lp = picked[0]
            x, y, kind = self.lp
            if not self.lp_kind:
                self.lp_kind = self.learned["lp_kind"] = kind
            if not self.peak_y and y > 0:
                self.peak_y = self.learned["peak_y"] = round(y, 4)
        boards = [k for k in range(1, 5) if _num(o.get(f"falc{k}:input:offset"))]
        sel = o.get("dl:lock:falc-selection")
        board = boards[0] if len(boards) == 1 else (sel if sel in boards else 0)
        if p.falc and boards:
            if board and isinstance(o.get("dl:lock:error-channel"), int):
                self.falc_name = f"falc{board}:input:offset"
                self.falc_orig = self.falc_now = float(o[self.falc_name])
            else:
                self.note("cannot tell which FALC pro the lock uses, or on which input its error signal is: "
                          "its input offset is left alone")

    def take_back(self):
        """After a relock that did not work: unlock again, for another go."""
        if self.cavity:
            yield mh.Set(LOCK, False, restore=False)
            self.unlocked = True
            (scan,) = yield mh.Get((SCAN,))
            if scan:
                yield mh.Set(SCAN, False, restore=self.scan_after_unlock)
        if self.shg:
            yield mh.Set(SHG_LOCK, False, restore=bool(self.orig.get(SHG_LOCK)))
        yield from self.read()

    # ------------------------------------------------------------- the SHG
    def shg_scan(self, on: bool):
        if on == self.shg_scanning:
            return
        o = self.orig
        if on:
            c1, c2 = "nlo:shg:scope:channel1:signal", "nlo:shg:scope:channel2:signal"
            if self.scope.get(c1) == SHG_INTRA:
                self.shg_key = "y"
            elif self.scope.get(c2) == SHG_INTRA:
                self.shg_key = "Y"
            else:  # borrow channel 1 of the SHG scope
                yield mh.Set(c1, SHG_INTRA, restore=o.get(c1))
                self.scope[c1], self.shg_key = SHG_INTRA, "y"
        yield mh.Set(SHG_SCAN, on, restore=bool(o.get(SHG_SCAN)))
        self.shg_scanning = on
        if on:
            yield mh.Wait(self.SETTLE_S)

    def settle(self, freq_name: str) -> float:
        """How long until a scope trace shows a change: three scan periods, at least SETTLE_S."""
        f = self.orig.get(freq_name)
        return min(max(self.SETTLE_S, 3.0 / f), 5.0) if _num(f) and f > 0 else 1.0

    def fresh(self, name):
        """A binary parameter (a scope trace), waiting a little if it has not changed since the
        last read (the scope had not updated yet)."""
        blob = None
        for k in range(11):
            (blob,) = yield mh.Get((name,))
            if not isinstance(blob, bytes):
                return None
            if blob != self._blobs.get(name) or k == 10:
                break
            yield mh.Wait(0.1)
        self._blobs[name] = blob
        return blob

    def shg_sweep(self, check: bool = True):
        """The SHG cavity's intra-cavity sweep, measured: a cavity.ModeCheck, None if unreadable. check: a
        flat one where there should be resonances means it does not lase (self.dark: NotLasing)."""
        yield mh.Wait(self.settle("nlo:shg:scan:frequency"), after_write=True)  # a sweep since the last change
        blob = yield from self.fresh("nlo:shg:scope:data")
        y = scan = None
        if blob is not None:
            try:
                arrays = float_arrays(blob)
            except DecopError:
                arrays = {}
            y = arrays.get(self.shg_key)
            other = "Y" if self.shg_key == "y" else "y"  # the other channel, if it shows the scan: where it turns
            if self.scope.get(f"nlo:shg:scope:channel{2 if other == 'Y' else 1}:signal") in SHG_PIEZO:
                scan = arrays.get(other)
        if not y:
            self.shg_ok = False
            self.note("cannot read the SHG cavity's intra-cavity signal: no more SHG checks")
            return None
        m = cavity.sweep(y, scan)
        self.last_sweep = m
        self._last_trace = y
        if self.sweep_lases(m):
            self.sweep_tells = self.seen_lasing = True
            self.top_seen = max(self.top_seen, m.height)
        elif check and self.flat_is_dark():
            yield from self.dark()
        return m

    def sweep_lases(self, m) -> bool:
        """Does SHG sweep m show the laser lasing: a resonance at least a tenth as tall as they are (the tallest
        seen in this recovery, or its intra-cavity signal locked, as usual)? Not lasing, it is flat."""
        ref = max(self.top_seen, self.p.shg_lock)
        return bool(m.seen) and m.height >= 0.1 * ref

    def judge(self, m):
        """(ok, text) for an SHG sweep (cavity.judge): against the laser's single-mode look if that is
        known; else by its side peaks, and its main peaks against the biggest seen in this recovery
        (all the power in one mode, they are biggest; two modes of about equal power would
        otherwise look like one clean mode)."""
        if m.seen:
            self._areas = sorted(self._areas + [m.area])[-2:]
            self.best_area = self._areas[0]  # the second biggest: one odd sweep does not set it
        if self.shg_ref is not None and self.shg_ref.known:
            return cavity.judge(m, self.shg_ref)
        ok, text = cavity.judge(m)
        if ok and m.area < cavity.AREA_MIN * self.best_area:
            return False, f"multimode: main peaks at {m.area / self.best_area:.2f} of the biggest seen"
        return ok, text

    def keep_sweep(self, m, why: str):
        """Keep the last sweep for a look afterwards (the latest few, and the maps' examples)."""
        ok, text = self.judge(m)
        sweeps = self.record["sweeps"]
        sweeps.append({"why": why, "i": round(self.mA(self.I), 3), "ok": ok, "text": text,
                       "y": cavity.decimate(self._last_trace, 300)})
        late = [s for s in sweeps if not s["why"].startswith("map")]
        if len(late) > 6:
            sweeps.remove(late[0])

    def mode_ok(self):
        """At the operating point: single mode, by the SHG sweep (True if there is no telling)."""
        if not self.shg_ok:
            return True
        yield from self.shg_scan(True)
        s = yield from self.shg_sweep()
        if s is None:
            return True
        ok, text = self.judge(s)
        self.keep_sweep(s, "operating point")
        if ok is None:
            self.note(f"SHG cavity: {text}: carrying on without")
            return True
        if not ok:
            self.note(f"at {self.mA(self.I):.2f} mA, SHG cavity: {text}")
            return False
        self.adopt(s, text)
        return True

    def adopt(self, s, text: str):
        """The laser is single-mode where it is to be handed back, by SHG sweep s: how it looks is kept
        (and learned, for telling single-mode next time)."""
        self.mode_text = text
        learned = cavity.updated(self.p.shg_ref if self.p.shg_ref.known else None, s)
        self.learned.update(shg_area=round(learned.area, 5), shg_side=round(learned.side, 4), shg_n=learned.n_main)

    def lock_shg(self):
        """Lock the SHG cavity: its intra-cavity signal must come up, and as high as usual (as learned
        while it ran, else where it was locked when the recovery began; the catch: within 1 - catch_min of
        the optimum's): if not, the master is not single-mode, whatever its sweep looked like (self.doubt),
        unless trimming the current for that has not brought it up already (then it is handed back all the
        same, and says so). The catch learns how its sweep's peak compares with the signal locked."""
        s = self.last_sweep
        yield from self.ask(f"Lock the SHG cavity ({self.mode_text or 'its mode was not checked'})")
        self.stage = "locking the SHG cavity"
        if self.shg_scanning:
            yield from self.shg_scan(False)
        yield mh.Set(SHG_LOCK, True, restore=bool(self.orig.get(SHG_LOCK)))
        if self.shg_level is not None:  # as high as it was locked before, give or take a factor 2
            level = 0.5 * self.shg_level
        else:  # or half way up the main peak of the sweep (the scope may scale it differently, though)
            level = s.floor + 0.5 * s.height if s is not None and s.seen else None
        catch = self.method == "catch"
        usual = self.usual_shg() if catch else (self.p.shg_lock or self.shg_level or 0.0)  # locked, single-mode
        good, text, intra, top = 0, None, None, 0.0
        for _ in range(int(self.SHG_LOCK_S / 0.5)):
            yield mh.Wait(0.5)
            text, intra = yield mh.Get(("nlo:shg:lock:state-txt", "nlo:pd:shg-int:photodiode"))
            up = _num(intra) and intra >= level if level is not None else says_locked(text)
            good = good + 1 if up else 0
            top = max(top, intra) if up and _num(intra) else top
            if good >= 2:
                break
        else:
            self.relock_trouble = (f"the SHG cavity does not lock ({text or '?'}"
                                   + (f", intra-cavity signal {intra:.3f} V" if _num(intra) else "") + ")")
            self.note(self.relock_trouble)
            self.doubt = True
            return False
        floor = s.floor if catch and s is not None and s.seen else 0.0
        least = self.p.catch_min if catch else cavity.AREA_MIN
        share = (top - floor) / (usual - floor) if usual > floor and top else 1.0
        if share < least:
            if catch and s is not None and s.top > 0 and top > floor and not self.explored:
                # its sweep made more of the peak than the signal locked shows: from now on, as the signal locked
                self.ratio = max(self.ratio, s.top / (top - floor))
                self.explored += 1
                self.relock_trouble = (f"the SHG cavity locked, but its intra-cavity signal is {top:.3f} V, "
                                       f"{share:.2f} of the optimum's, short of what its sweep showed: caught again")
                self.note(self.relock_trouble)
                self.doubt = True
                return False
            if not catch and not self.explored:
                self.relock_trouble = (f"the SHG cavity locked, but its intra-cavity signal is {top:.3f} V, "
                                       f"{share:.2f} of usual: the master is not single-mode")
                self.note(self.relock_trouble)
                self.doubt = True
                return False
            # (a second mode whose peaks the sweep cannot tell from the main ones, or a weaker TA, say)
            self.shg_text = f"SHG signal only {share:.2f} of {'the optimum' if catch else 'usual'}"
            self.note(f"SHG cavity locked, its intra-cavity signal {top:.3f} V, {share:.2f} of "
                      f"{'the optimum' if catch else 'usual'}, and trimming the current did not bring it up: handed "
                      f"back all the same")
            return True
        self.note("SHG cavity locked" + (f", intra-cavity signal {top:.3f} V" if top else "")
                  + (f", {share:.2f} of the optimum's" if catch and usual > floor and top else ""))
        if top:
            self.learned["shg_lock"] = round(top if not self.p.shg_lock else 0.7 * self.p.shg_lock + 0.3 * top, 4)
        if s is not None and s.seen and s.top > 0 and top > s.floor:  # (single-mode here, the catch's or the walk's)
            r = s.top / (top - s.floor)  # its sweep's peak, as a share of the signal locked (both above the floor)
            if 0.2 <= r <= 5.0:  # (another scale on the scope, say: the catch compares with the optimum's so)
                self.learned["shg_ratio"] = round(r if not self.p.shg_ratio else 0.7 * self.p.shg_ratio + 0.3 * r, 4)
        return True

    # ----------------------------------------------------------- handing back
    def hand_back(self):
        """Lock the SHG cavity, then the cavity lock: True when locked where it belongs."""
        if self.shg and not (yield from self.lock_shg()):
            return False
        if self.cavity:
            relock = self.relock_lockpoint if self.relock_mode == "lockpoint" else self.relock_plain
            if not (yield from relock()):
                self.falc_failed = bool(self.falc_name) and abs(self.falc_now - self.falc_orig) > 1e-6
                return False
        yield from self.tidy()
        return True

    def relock_plain(self):
        """Re-enable the lock, set to lock without a lockpoint (else it locks only at a selected one):
        it finds the resonance by itself, the laser being at its target. With a FALC pro, the error
        signal's DC level nulled first (scan_dc). Relocked: its lock reports locked, the wavemeter agrees, and its
        transmission is up as usual (lock_min of the way from its zero to its level locked, if that is known; the
        wavemeter alone would take a laser parked on target, its lock on hold, for one relocked)."""
        free = self.orig.get("dl:lock:lock-without-lockpoint")
        scanning = (yield from self.scan_dc()) if self.falc_name else False
        yield from self.ask("Relock: re-enable the cavity lock" + ("" if free else ", ticking lock without lockpoint"))
        self.stage = "relocking the cavity"
        if free is False:
            yield mh.Set("dl:lock:lock-without-lockpoint", True, restore=False)
            self.orig["dl:lock:lock-without-lockpoint"] = True
        if self.scan_after_unlock and not scanning:
            yield mh.Set(SCAN, True, restore=True)
        yield mh.Set(LOCK, True, restore=False)
        self.unlocked = False
        if not (yield from self.wait_locked()):
            state = yield mh.Get(("dl:lock:state-txt",))
            self.relock_trouble = f"the cavity lock would not lock (“{state[0] or '?'}” after {self.LOCK_S:g} s)"
            self.note(self.relock_trouble)
            return False
        if not (yield from self.verify()):
            self.relock_trouble = f"the cavity lock caught a peak {self.e:+.0f} MHz away"
            self.note(self.relock_trouble)
            return False
        usual = self.p.trans_locked
        level = (yield from self.sample_level(self.LEVEL_WAITS)) if usual > 0 else None
        z = self.zero if self.zero is not None else usable_zero(self.p.trans_zero, usual)
        if level is not None and usual > z and level - z < self.p.lock_min * (usual - z):
            self.relock_trouble = (f"locked, but the transmission is {volts(level, usual)} V, "
                                   f"{(level - z) / (usual - z):.2f} of the way from its zero ({volts(z, usual)} V) to "
                                   f"its level locked ({volts(usual)} V)")
            self.note(self.relock_trouble)
            return False
        if level is not None:
            self.note(f"relocked, transmission {volts(level, usual)} V")
        return True

    def scan_dc(self):
        """What only the master's scan shows, before a relock without lockpoint: the error signal's DC level,
        nulled with the FALC pro's input offset, and the transmission's zero. The scan runs about the laser,
        which is at its target; then it is as the relock wants it (off, unless the DLC pro had it on when its
        lock was turned off). True if it is left on."""
        amp = self.orig.get("scan:amplitude")
        if not (_num(amp) and amp > 0):
            self.note("the scan amplitude is 0: the error signal's DC level cannot be seen, the FALC pro's input "
                      "offset is left alone")
            return False
        yield from self.ask("Start the scan, to see the error signal's DC level and null it with the FALC pro's "
                            "input offset")
        self.stage = "scanning, for the error signal's DC level"
        yield mh.Set(SCAN, True, restore=self.scan_after_unlock)
        yield mh.Wait(self.settle("scan:frequency"))
        zero = yield from self.sample_level(self.ZERO_WAITS)  # mostly between the peaks: its zero
        top = max(self.peak_y, self.p.trans_locked)  # the peak's height, or the level locked, as known
        if zero is not None and top > 0:
            if zero < 0.5 * top:
                self.zero = self.learned["trans_zero"] = round(zero, 5)
                self.note(f"transmission off resonance, scanning: {volts(zero, top)} V (its zero)")
            else:
                self.note(f"the transmission while scanning, {volts(zero, top)} V, is no zero next to the "
                          f"{volts(top)} V it has locked: not taken")
        yield from self.falc_before_lock()
        if not self.scan_after_unlock:
            yield mh.Set(SCAN, False)
            yield mh.Wait(self.SETTLE_S)
        return self.scan_after_unlock

    def relock_lockpoint(self):
        p = self.p
        x_pred = self.V - (self.e if math.isfinite(self.e) else 0.0) / self.kv  # the target, on the scan
        amp = self.orig.get("scan:amplitude")
        amp = float(amp) if _num(amp) else 0.0
        settle = self.settle("scan:frequency")
        yield from self.ask(f"Start the scan and find the cavity peak where the target is, near {x_pred:.3f} V")
        self.stage = "finding the cavity peak"
        yield mh.Set(SCAN, True, restore=self.scan_after_unlock)
        yield mh.Wait(settle)
        zero = yield from self.sample_level(self.ZERO_WAITS)  # mostly between the peaks: its zero
        choice, cands, seen = None, [], []
        for _ in range(self.PEAK_LOOKS):
            cands = yield from self.candidates()
            seen += cands  # a narrow peak sampled a sample or so wide: its tallest look counts (choose_peak)
            choice = cavity.choose_peak(seen, x_pred, self.kv, p.window, self.peak_y, self.lp_kind)
            if choice.ok:
                break
            if amp and abs(x_pred - self.V) > 0.4 * amp and self.v_lo <= x_pred <= self.v_hi:
                yield from self.move(offset=x_pred, read=False)  # the target in the middle of the scan
                seen = []
            yield mh.Wait(settle)
        yield from self.keep_scan(x_pred, choice, cands)
        if not choice.ok:
            why = f"cannot tell which cavity peak is right: {choice.text}"
            return (yield from self.lock_as_selected(x_pred, why, zero))
        self.note(f"cavity peak: {choice.text}")
        if zero is not None:
            if choice.y > 0 and zero < 0.5 * choice.y:
                self.zero = self.learned["trans_zero"] = round(zero, 5)
                self.note(f"transmission off resonance, scanning: {volts(zero, choice.y)} V (its zero)")
            else:
                self.note(f"the transmission while scanning, {volts(zero, choice.y)} V, is no zero next to the "
                          f"peak's {volts(choice.y)} V: not taken")
        yield from self.falc_before_lock()
        picked = yield from self.select(x_pred, choice)
        if picked is None:
            why = "the DLC pro would not take the lockpoint"
            return (yield from self.lock_as_selected(x_pred, why, zero, nulled=True))
        yield from self.ask(f"Lock to the peak at {picked.x:.3f} V")
        if not (yield from self.lock_and_check(max(choice.y, picked.y))):
            return False
        y = 0.7 * self.peak_y + 0.3 * choice.y if self.peak_y else choice.y  # (its tallest look, as it looks usually)
        self.learned.update(peak_y=round(y, 4), lp_kind=choice.kind)
        return True

    def lock_and_check(self, top: float):
        """Enable the cavity lock (at the lockpoint selected): locked, and where the wavemeter says it belongs,
        with its transmission at least half way from its zero up to the peak's top (V) in the scan? Its level
        locked is learned, if so."""
        self.stage = "locking the cavity"
        yield mh.Set(LOCK, True, restore=False)
        self.unlocked = False
        if not (yield from self.wait_locked()):
            self.relock_trouble = "the cavity lock would not lock at the lockpoint"
            self.note(self.relock_trouble)
            return False
        if not (yield from self.verify()):
            self.relock_trouble = f"the cavity lock caught a peak {self.e:+.0f} MHz away"
            self.note(self.relock_trouble)
            return False
        level = yield from self.sample_level(self.LEVEL_WAITS)
        z = self.zero if self.zero is not None else usable_zero(self.p.trans_zero, top)
        if level is not None and top > z and level - z < 0.5 * (top - z):
            self.relock_trouble = (f"locked, but the transmission is {volts(level, top)} V, "
                                   f"{(level - z) / (top - z):.2f} of the peak's {volts(top)} V above zero "
                                   f"({volts(z, top)} V)")
            self.note(self.relock_trouble)
            return False
        if level is not None and level > z:
            self.learned["trans_locked"] = round(level, 5)  # locked where it belongs: its transmission as it should be
        self.note("locked to the right peak" + (f", transmission {volts(level, top)} V" if level is not None
                                                else ""))
        return True

    def lock_as_selected(self, x_pred: float, why: str, zero, nulled: bool = False):
        """The right peak could not be picked here (why): if the DLC pro has a lockpoint selected near the target
        (selected by hand, or by an earlier relock), lock with that, as you would, the wavemeter judging; else
        leave the laser at the target, unlocked, scanning, for you. nulled: the error signal's DC level already."""
        p = self.p
        (pos,) = yield mh.Get(("dl:lock:lockpoint:position",))
        x = pos[0] if isinstance(pos, tuple) and len(pos) >= 2 and _num(pos[0]) else None
        y = pos[1] if x is not None and _num(pos[1]) else 0.0
        if x is None or (abs(x) < 1e-9 and abs(y) < 1e-9) or abs(x - x_pred) * abs(self.kv) > p.window:
            where = "none" if x is None or (abs(x) < 1e-9 and abs(y) < 1e-9) else (
                f"at {x:.3f} V, {(x - x_pred) * self.kv:+.0f} MHz from the target")
            raise mh.Failed(f"{why}; the lockpoint selected is {where} (left at the target, unlocked, scanning: "
                            f"select the peak by hand)", keep=True)
        self.note(f"{why}: locking with the lockpoint the DLC pro has, at {x:.3f} V "
                  f"({(x - x_pred) * self.kv:+.0f} MHz from the target)")
        if not nulled:
            yield from self.falc_before_lock()
        yield from self.ask(f"{why}: lock with the lockpoint selected, at {x:.3f} V, as you would")
        if self.zero is None and zero is not None and y > 0 and zero < 0.5 * y:
            self.zero = zero
        return (yield from self.lock_and_check(y if y > 0 else self.peak_y))

    def candidates(self):
        """The lockpoint candidates in the scan running now: the DLC pro finds (and shows) them only when
        asked (find-candidates; the Lock points of its display), so it is asked each time."""
        r = yield mh.Exec("dl:lock:find-candidates", ())
        if isinstance(r, Exception) and not self._find_refused:
            self._find_refused = True
            self.note(f"find-candidates: {r}: the candidates are as the DLC pro shows them")
        (blob,) = yield mh.Get(("dl:lock:candidates",))
        if not isinstance(blob, bytes):
            return []
        try:
            return lock_points(blob).get("c", [])
        except DecopError:
            return []

    def select(self, x_pred, c):
        """select-lockpoint on the right peak's candidate; the candidate chosen (None if it would
        not take one). The candidates move with the scan and the laser's drift, so each try uses
        them as read just before it, and a 'not found' is tried once more with fresh ones. The
        type argument is a number; which number a candidate's type character stands for is
        learned (its character code first)."""
        p = self.p
        codes = []
        for code in ([p.lp_code] if p.lp_code >= 0 else []) + [ord(c.kind) if c.kind else -1, 0, 1, 2, 3]:
            if code >= 0 and code not in codes:
                codes.append(code)
        want = self.orig.get("dl:lock:lockpoint:type")
        want = want if isinstance(want, str) and want else None
        yield from self.ask(f"Select the lockpoint at {c.x:.3f} V, {c.y:.3f} V")
        self.stage = "selecting the lockpoint"
        why = "no answer"
        gap = 0.25 * p.window / max(abs(self.kv), 1e-9)
        for code in codes:
            for _ in range(2):
                cands = yield from self.candidates()
                hit = cavity.same_peak(cands, c.x, c.kind, gap)  # the peak chosen, as this scan shows it
                if hit is None:
                    why = f"no candidate on the peak at {c.x:.3f} V in this scan"
                    continue
                fresh = cavity.PeakChoice(True, hit[0], hit[1], hit[2], c.off, c.text)
                r = yield mh.Exec("dl:lock:select-lockpoint", (float(fresh.x), float(fresh.y), int(code)))
                if isinstance(r, Exception):
                    why = str(r)
                    continue
                pos, kind = yield mh.Get(("dl:lock:lockpoint:position", "dl:lock:lockpoint:type"))
                if isinstance(pos, tuple) and len(pos) >= 2 and _num(pos[0]) and _num(pos[1]) and not (
                        abs(pos[0] - fresh.x) <= 0.02 and abs(pos[1] - fresh.y) <= 0.1 * max(abs(fresh.y), 0.05)):
                    why = f"it selected ({pos[0]:.3f}, {pos[1]:.3f}) instead"
                    break
                if want and isinstance(kind, str) and kind and kind != want and code not in (p.lp_code, ord(c.kind)):
                    why = f"it selected a {kind!r} lockpoint, not {want!r}"
                    break
                if code != p.lp_code:
                    self.learned["lp_code"] = code
                self.note(f"lockpoint selected at {fresh.x:.3f} V (type {code})")
                return fresh
        self.note(f"select-lockpoint near {c.x:.3f} V: {why}")
        return None

    def wait_locked(self):
        for _ in range(int(self.LOCK_S / 0.5)):
            yield mh.Wait(0.5)
            state, text = yield mh.Get(("dl:lock:state", "dl:lock:state-txt"))
            if state == 5 or says_locked(text):
                return True
            if state is None and text is None:
                return True  # it cannot say: the wavemeter will
        return False

    def sample_level(self, waits):
        """The transmission's present value (the input its lock detection watches, as a rule), looked at after
        each of `waits` (s): their median, or None if it cannot be read."""
        o = self.orig
        name = transmission_input(*(o.get(n) for n in TRANS))
        if name is None:
            return None
        got = []
        for w in waits:
            yield mh.Wait(w)
            (v,) = yield mh.Get((name,))
            if _num(v):
                got.append(float(v))
        return statistics.median(got) if got else None

    def keep_scan(self, x_pred, choice, cands):
        """Keep the cavity scan, its candidates and the choice, for a look afterwards."""
        rec = {"x_pred": round(x_pred, 4), "window_v": round(self.p.window / max(abs(self.kv), 1e-9), 4),
               "cands": [[round(x, 4), round(y, 4), t] for x, y, t in cands[:60]],
               "choice": None if choice is None else [round(choice.x, 4) if choice.ok else None,
                                                      round(choice.y, 4) if choice.ok else None,
                                                      choice.kind, choice.text]}
        spec = self.orig.get("dl:lock:spectrum-input-channel")
        if self.orig.get("scope:channelx:xy-signal") in SCAN_X and spec in (self.scope.get("scope:channel1:signal"),
                                                                           self.scope.get("scope:channel2:signal")):
            blob = yield from self.fresh("scope:data")
            try:
                a = float_arrays(blob) if blob is not None else {}
            except DecopError:
                a = {}
            y = a.get("y" if spec == self.scope.get("scope:channel1:signal") else "Y")
            x = a.get("x")
            amp = self.orig.get("scan:amplitude")
            if x and y and len(x) == len(y) and _num(amp) and 0.5 * amp <= max(x) - min(x) <= 2 * amp:
                rec["x"], rec["y"] = cavity.decimate_xy(x, y, 400)  # an XY trace, against the scan output
        self.record["scan"] = rec

    # ------------------------------------------------------- the FALC pro
    def error_trace(self, settle: bool = True):
        """(error signal, transmission or None, scan voltage or None) over a scan, borrowing a scope channel if
        need be. settle: one acquired since the last write (else just a fresh one). The scope's x is the scan
        voltage if it shows one (XY against the scan output or the piezo, as wide as the scan)."""
        o = self.orig
        err, spec = o.get("dl:lock:error-channel"), o.get("dl:lock:spectrum-input-channel")
        c1, c2 = "scope:channel1:signal", "scope:channel2:signal"
        if not isinstance(err, int):
            return None
        if err not in (self.scope.get(c1), self.scope.get(c2)):
            keep = c2 if self.scope.get(c1) == spec else c1  # keep the transmission in view
            yield mh.Set(keep, err, restore=o.get(keep))
            self.scope[keep] = err
            settle = True
        if settle:
            yield mh.Wait(self.settle("scan:frequency"), after_write=True)  # a trace acquired since the last change
        blob = yield from self.fresh("scope:data")
        try:
            a = float_arrays(blob) if blob is not None else {}
        except DecopError:
            return None
        e = a.get("y" if self.scope.get(c1) == err else "Y")
        t = a.get("y" if self.scope.get(c1) == spec else "Y") if spec in (self.scope.get(c1),
                                                                         self.scope.get(c2)) else None
        if not e:
            return None
        x, amp = a.get("x"), o.get("scan:amplitude")
        scan = (o.get("scope:channelx:xy-signal") in SCAN_X and x and len(x) == len(e) and _num(amp) and amp > 0
                and 0.5 * amp <= max(x) - min(x) <= 2 * amp)
        return e, t, (x if scan else None)

    def dc_now(self):
        """The error signal's DC level now (cavity.dc_level): the median of FALC_TRACES traces, their spread (the
        highest less the lowest) and the last trace; None if there is no trace."""
        got, last = [], None
        for k in range(self.FALC_TRACES):
            tr = yield from self.error_trace(settle=k == 0)
            if tr is None:
                break
            last = tr
            v = cavity.dc_level(*tr)
            if math.isfinite(v):
                got.append(v)
        if not got:
            return None
        return statistics.median(got), max(got) - min(got), last

    def falc_before_lock(self):
        """Before (re)locking: null the error signal's DC level with the FALC pro's input offset; but after a relock
        that failed with the offset moved, back to where it was, with which it was locked before (alternately,
        then: a lock that fails for another reason is tried both ways)."""
        if not self.falc_name:
            return
        if self.falc_failed:
            self.falc_failed = False
            name, was, now = self.falc_name, self.falc_orig, self.falc_now
            yield from self.ask(f"The last relock failed with the FALC input offset at {now:+.4f} V: back to "
                                f"{was:+.4f} V, as it was locked before")
            self.stage = "putting the FALC input offset back"
            yield mh.Set(name, was, restore=was)
            self.falc_now = was
            self.falc_text = f"FALC input offset back to {was:+.4f} V (nulled, the relock had failed)"
            self.note(f"the last relock failed with the FALC input offset at {now:+.4f} V: back to {was:+.4f} V, "
                      f"as it was locked before, not nulled this time")
            return
        yield from self.null_falc()

    def null_falc(self):
        """Null the error signal's DC level with the FALC pro's input offset, by at most falc_max from where it was
        when the recovery began. Each look at the level is the median of FALC_TRACES traces; a level within
        FALC_TOL, or within 3 times their scatter, is left alone. How much the offset moves the error signal
        (falc_gain, V per V, sign and all) is learned: until it is, the offset is moved FALC_PROBE of the way (as
        if one to one) until the error signal shows that clearly, which tells it; known, straight there, again from
        where that got it, until within half its tolerance (a step that should have shown and did not: found out
        afresh). The best offset seen is kept, and if that is no better than where it began, it is put back."""
        p, name, o0 = self.p, self.falc_name, self.falc_orig
        look = yield from self.dc_now()
        if look is None:
            self.note("no error-signal trace: the FALC pro's input offset is left alone")
            return
        dc0, spread, tr = look
        start = cur = self.falc_now
        rec = self.record["falc"] = {"name": name, "offset0": round(start, 5), "dc0": round(dc0, 5),
                                     "error": cavity.decimate(tr[0], 300)}
        noise = max(spread, self.FALC_NOISE)
        tol = max(self.FALC_TOL, 3 * noise)
        if abs(dc0) <= tol:
            self.falc_text = f"error signal DC {dc0 * 1e3:+.1f} mV, left alone"
            return
        aim = max(0.5 * tol, 2 * noise)  # once at it, as close as that
        lo, hi = o0 - p.falc_max, o0 + p.falc_max
        seen, dc, gain, lost = [(start, dc0)], dc0, p.falc_gain, False
        probe, base = (None, None) if gain else (0, (start, dc0))  # finding out how much, from base (offset, level)
        for _ in range(2 * len(self.FALC_PROBE) + self.FALC_STEPS):
            if probe is None and abs(dc) <= max(aim, 2 * noise):
                break
            if probe is not None:
                if probe >= len(self.FALC_PROBE):
                    break
                new = base[0] - self.FALC_PROBE[probe] * base[1] / float(p.falc_sign or 1)
                probe += 1
                why = "See how the FALC input offset moves the error signal's DC level"
            else:
                new = cur - dc / gain
                why = "Null the error signal's DC level"
            new = min(max(new, lo), hi)
            if abs(new - cur) < 1e-6:
                self.note(f"the FALC input offset is at its limit, {o0:+.4f} ± {p.falc_max:g} V")
                break
            was = cur, dc
            yield from self.ask(f"{why} ({dc * 1e3:+.1f} mV): FALC input offset {cur:+.4f} → {new:+.4f} V")
            self.stage = "nulling the error signal's DC level"
            yield mh.Set(name, new, restore=o0)
            cur = self.falc_now = new
            got = yield from self.dc_now()
            if got is None:  # (the scope refused, busy, say): back to the best offset seen, below
                lost = True
                break
            dc, spread, _ = got
            noise = max(noise, spread)
            seen.append((cur, dc))
            if probe is not None:
                if abs(dc - base[1]) >= 3 * noise:  # it shows: that much
                    gain, probe = (dc - base[1]) / (cur - base[0]), None
            elif abs(dc - was[1]) >= 3 * noise:
                gain = (dc - was[1]) / (cur - was[0])
            elif abs(was[1]) >= 6 * noise:  # it should have shown, and did not: not as learned, find out afresh
                gain, probe, base = 0.0, 0, (cur, dc)
        if not gain:
            moved = cur - (base[0] if base else start)
            if lost:
                self.note(f"no error-signal trace after moving the FALC input offset to {cur:+.4f} V: put back to "
                          f"{start:+.4f} V")
            elif 0.3 * abs(moved) >= 3 * noise:
                err = self.orig.get("dl:lock:error-channel")
                self.note(f"the error signal on the scope ({signal_name(err)}) does not follow the FALC pro's input "
                          f"offset, so it is not the FALC's input: offset put back, left alone")
                self.learned["falc_gain"] = 0.0
                self.learned["falc_sign"] = 0
            else:
                self.note(f"moving the FALC input offset {moved:+.4f} V did not clearly move the error signal's DC "
                          f"level ({dc0 * 1e3:+.1f} mV): put back")
            yield mh.Set(name, start, restore=o0)
            self.falc_now = start
            rec.update(offset=round(start, 5), dc=round(dc0, 5))
            return
        self.learned["falc_gain"] = round(gain, 4)
        self.learned["falc_sign"] = 1 if gain > 0 else -1
        best_o, best_dc = min(seen, key=lambda od: abs(od[1]))
        steps = f"{len(seen) - 1} step{'s' if len(seen) != 2 else ''}; the offset moves it {gain:+.3g} V/V"
        if abs(best_dc) > abs(dc0) - 2 * noise:  # no better than it was: as it was
            best_o, best_dc = start, dc0
            self.falc_text = f"error signal DC {dc0 * 1e3:+.1f} mV, nulling did not bring it down: offset put back"
        else:
            self.falc_text = (f"FALC input offset {start:+.4f} → {best_o:+.4f} V, error signal DC "
                              f"{dc0 * 1e3:+.1f} → {best_dc * 1e3:+.1f} mV")
        if lost:
            self.note(f"no error-signal trace after moving the FALC input offset to {cur:+.4f} V: back to "
                      f"{best_o:+.4f} V")
        self.note(f"{self.falc_text} ({steps})")
        if abs(best_o - cur) > 1e-9:
            yield from self.ask(f"Back to the best FALC input offset seen, {best_o:+.4f} V")
            yield mh.Set(name, best_o, restore=o0)
            self.falc_now = best_o
        rec.update(offset=round(best_o, 5), dc=round(best_dc, 5), gain=round(gain, 4), steps=len(seen) - 1)

    def tidy(self):
        """Give back the scope channels, and the scan setting the laser had while locked."""
        for key, was in self.orig.items():
            if key.endswith(":signal") and key in self.scope and isinstance(was, int) and self.scope[key] != was:
                yield mh.Set(key, was)
                self.scope[key] = was
        if self.cavity and self.orig.get(SCAN) is False and self.relock_mode == "lockpoint":
            yield mh.Set(SCAN, False)


# ------------------------------------------------------------ setup check
def setup_report(client, laser: int, hop, lp) -> list:
    """Everything a recovery would use on this laser, read (only reads) from its controller, and
    what looks wrong: [(level, text)], level 'head', 'ok', 'info', 'warn' or 'error'.
    client: a DLCPro; hop: HopParams; lp: the wavemeter lock's LockParams."""
    out = []

    def add(level, text):
        out.append((level, text))

    def get(*names):
        return dict(zip(names, mh.perform(client, laser, ("get", names))))

    def fmt(v, unit="", digits=3):
        if v is None:
            return "—"
        if isinstance(v, bool):
            return "on" if v else "off"
        if _num(v):
            return f"{v:.{digits}f}{unit}"
        return str(v)

    g = get("system-type", "fw-ver", "serial-number", "product-name", "emission")
    add("head", "Controller")
    if g["fw-ver"] is None and g["product-name"] is None:
        add("error", f"laser{laser} does not answer: is this the right controller and laser?")
        return out
    add("ok", f"{g['system-type'] or 'DLC pro'} {g['serial-number'] or ''}, firmware {g['fw-ver'] or '?'}; "
              f"laser{laser}: {g['product-name'] or '?'}, emission {fmt(g['emission'])}")
    try:
        fw = tuple(int(x) for x in str(g["fw-ver"]).split(".")[:2])
    except ValueError:
        fw = None

    out_ch = get(SCAN_OUT)[SCAN_OUT]  # what its scan drives: which parameters set its piezo and its current
    out_ch = int(out_ch) if _num(out_ch) else None
    piezo, cc = drives(out_ch)
    g = get(cc, "dl:cc:current-act", "dl:cc:current-clip", "dl:cc:current-offset",
            "dl:cc:feedforward-enabled", "dl:cc:feedforward-factor", "dl:cc:output-filter:slew-rate-enabled",
            "dl:cc:output-filter:slew-rate")
    add("head", "Current")
    i, act, clip = g[cc], g["dl:cc:current-act"], g["dl:cc:current-clip"]
    off = act - i if _num(i) and _num(act) else None
    add("ok" if _num(i) and _num(act) else "error",
        f"measured {fmt(act, ' mA')}, set {fmt(i, ' mA')}" + (" (the scan's offset: its scan drives the current)"
                                                             if cc == SCAN_OFFSET else "")
        + (f" ({off:+.3f} mA apart)" if off is not None else "")
        + f", limit (current-clip, on the set current) {fmt(clip, ' mA')}")
    ff, factor = g["dl:cc:feedforward-enabled"], g["dl:cc:feedforward-factor"]
    add("info", f"current offset {fmt(g['dl:cc:current-offset'], ' mA')}; feedforward from the piezo "
                + (f"on, {factor:+.4f} mA/V" if ff and _num(factor) else fmt(ff)))
    if ff and _num(factor) and factor:
        add("info", f"with the feedforward, the piezo moves the current too: up to ±{abs(factor) * hop.v_window:.2f} "
                    f"mA over the ±{hop.v_window:g} V a recovery may move it. A recovery keeps the measured current "
                    "within its window, and above where it stopped lasing, wherever the piezo goes (the current set "
                    "follows), moves the piezo in steps that change it by 0.2 mA at most, and before walking the piezo "
                    "the way that takes the current down, parks the current high (up while the SHG sweep stays as "
                    "good, then back two steps), again whenever it has come down 0.3 mA")
    if g["dl:cc:output-filter:slew-rate-enabled"]:
        add("warn", f"the current's slew rate is limited ({fmt(g['dl:cc:output-filter:slew-rate'])}): a recovery "
                    "steps it 0.2 mA at a time and expects each step to take effect at once")
    if hop.i_nom:
        lo, hi = hop.i_nom - hop.i_below, hop.i_nom + hop.i_window
        top = hop.i_nom + max(hop.jump_max, hop.i_window)
        if _num(clip) and off is not None and hi - off > clip:
            add("info", f"the window's top, {hi:.2f} mA, is above current-clip: {clip + off:.2f} mA measured is "
                        "the limit")
            hi = clip + off
        if _num(clip) and off is not None and top - off > clip:
            add("warn", f"current-clip keeps a laser that stopped lasing below {clip + off:.2f} mA measured, short "
                        f"of the {top:.2f} mA it may jump to")
            top = clip + off
        inside = _num(act) and lo <= act <= hi
        add("ok" if inside else "warn", f"recoveries keep the measured current within {lo:.2f} … {hi:.2f} mA "
                                        f"({hop.i_nom:.2f} − {hop.i_below:g} / + {hop.i_window:g}), setting it "
                                        f"≤ 0.2 mA a step; " + ("it is inside now" if inside else
                                                                "it is OUTSIDE now: a recovery brings it in first "
                                                                "(from above slowly, watching that it lases, and on "
                                                                "down until the SHG sweep is single-mode)"))
        add("info", f"a laser that stopped lasing: the current to {hop.i_nom + hop.jump:.2f} mA at once, then up to "
                    f"{top:.2f} mA until it lases, and back down at {hop.ramp:g} mA/s")
    else:
        add("info", "the nominal current is read, as measured, when recovery is enabled")

    names = hop.seed_names
    if names:
        add("head", "Lasing (its TA's seed power)")
        g = get(*names, mh.SEED_FLOOR, "amp:seed-limits:power-min-shutdown-delay", "amp:seed-limits:status-txt",
                mh.TA_ON, "amp:cc:emission", "emission")
        name = next((n for n in names if _num(g[n])), "")
        if not name:
            add("info" if hop.seed.strip().lower() == "auto" else "error",
                f"no seed power to read ({', '.join(names)}): whether it lases is not watched"
                + (" (it has no TA?)" if hop.seed.strip().lower() == "auto" else ""))
        else:
            seed, floor, delay = g[name], g[mh.SEED_FLOOR], g["amp:seed-limits:power-min-shutdown-delay"]
            floor = floor if _num(floor) and floor > 0 else 0.0
            least = hop.seed_least(hop.seed_ok, floor)
            lasing = seed >= least if least else None
            add("error" if lasing is False else "ok", f"seed power {seed:.2f} mW ({name})"
                + (": NOT LASING" if lasing is False else ""))
            add("info", (f"not lasing below {least:.2f} mW: " if least else "")
                + (f"{hop.seed_min:g} of its usual {hop.seed_ok:.2f} mW" if hop.seed_ok else "its usual seed power is "
                   "learned while it lases on target")
                + (f", and no less than the TA's power-min, {floor:.2f} mW" if floor else ""))
            if floor and _num(delay):
                add("info",
                    f"the TA switches itself off after {delay:g} (power-min-shutdown-delay) with less than "
                    f"{floor:.2f} mW of seed: a laser that stops lasing has its current jumped within a second or "
                    "two of the seed power showing it; if the TA is off by then, switch it on again by hand")
            ta, em = g[mh.TA_ON], g["amp:cc:emission"]
            add("ok" if ta is not False else "warn", f"TA {fmt(ta)}, emitting {fmt(em)}, seed limits "
                f"“{g['amp:seed-limits:status-txt'] or '?'}”; laser emission {fmt(g['emission'])}")
            if hop.shg:
                add("info", "a master that does not lase may still send its TA much of its power as broadband "
                            "light: the SHG sweep has the last word (flat, it does not lase; see SHG below)")

    g = get(*dict.fromkeys((piezo, SCAN_OFFSET, "scan:enabled", "scan:amplitude", "scan:frequency", "scan:unit")))
    add("head", "Piezo and scan")
    v, unit = g[piezo], g["scan:unit"] or "V"
    inside = _num(v) and lp.v_min <= v <= lp.v_max
    what = "offset" if piezo == SCAN_OFFSET else f"piezo voltage ({piezo})"
    add("ok" if inside else "error", f"{what} {fmt(v, ' V')} (lock limits {lp.v_min:g} … {lp.v_max:g} V); "
                                     f"recoveries move it at most ±{hop.v_window:g} V"
                                     + (f" (the catch's tries: the optimum's ±{hop.pc_try:g} V)"
                                        if hop.how() == "catch" and hop.pc_try > 0 else ""))
    if out_ch is None:
        add("info", f"what its scan drives is not known ({SCAN_OUT} cannot be read): its offset is taken for the piezo")
    elif piezo == SCAN_OFFSET:
        add("ok", f"its scan drives the piezo ({SCAN_OUT} {out_ch}): the lock moves the scan's offset")
    else:
        add("info", f"its scan drives {output_name(out_ch)} ({SCAN_OUT} {out_ch}), not the piezo: the lock moves the "
                    f"piezo's own set value, {piezo}" + (f", and recoveries the current by the scan's offset, now "
                                                        f"{fmt(g[SCAN_OFFSET], ' ' + unit)}" if out_ch == OUT_CC
                                                        else ""))
    scanning = g["scan:enabled"] is True and out_ch == OUT_CC
    add("warn" if scanning else "ok", f"scan {fmt(g['scan:enabled'])}, amplitude {fmt(g['scan:amplitude'], ' ' + unit)}"
                                      f", {fmt(g['scan:frequency'], ' Hz', 1)}"
                                      + (": it sweeps the current while it runs, which no lock holds through: "
                                         "switch it off" if scanning else ""))
    amp = g["scan:amplitude"]
    if hop.kv:
        add("info", f"measured piezo tuning {hop.kv / 1e3:+.3f} GHz/V"
                    + (f": the scan spans {abs(hop.kv) * amp / 1e3:.2f} GHz" if _num(amp) and piezo == SCAN_OFFSET
                       else "")
                    + (f", modes {hop.spacing / 1e3:.2f} GHz apart" if hop.spacing else ""))

    if hop.cavity:
        add("head", "Cavity lock (DLC pro lock module)")
        g = get("dl:lock:lock-enabled", "dl:lock:state", "dl:lock:state-txt", "dl:lock:lock-without-lockpoint",
                "dl:lock:lockpoint:position", "dl:lock:lockpoint:type", "dl:lock:candidates", "dl:lock:error-channel",
                "dl:lock:falc-selection", "dl:lock:relock:enabled", "dl:lock:window:level-low",
                "dl:lock:window:level-high", *TRANS)
        add("ok" if g["dl:lock:lock-enabled"] is not None else "error",
            f"lock {fmt(g['dl:lock:lock-enabled'])}, state {g['dl:lock:state']} “{g['dl:lock:state-txt'] or '?'}”; "
            f"ReLock {fmt(g['dl:lock:relock:enabled'])}, window {fmt(g['dl:lock:window:enabled'])}")
        add("info", "this program does not lock it to the wavemeter: its row's button is Relock, which has it "
                    "watched and relocked after a mode hop, on the wrong peak, or when its lock keeps losing it (see "
                    "below; and relocks it then and there if it needs it when you click)")
        wo = g["dl:lock:lock-without-lockpoint"]
        mode = hop.relock if hop.relock in ("lockpoint", "plain") else ("plain" if wo else "lockpoint")
        blob = g["dl:lock:candidates"]
        pts = {}
        if isinstance(blob, bytes):
            try:
                pts = lock_points(blob)
            except DecopError:
                add("warn", "the lockpoint candidates are not in the format expected")
        if mode == "lockpoint" and not isinstance(blob, bytes):
            add("error", "cannot read dl:lock:candidates: relocking would only re-enable the lock")
            mode = "plain"
        if mode == "lockpoint":
            add("ok", f"relock by lockpoint (lock-without-lockpoint {fmt(wo)}): start the scan, have the DLC pro find "
                      f"its lock points (find-candidates), pick the tall peak within ±{hop.window:.0f} MHz of where "
                      f"the wavemeter puts the setpoint, select it, lock")
        else:
            add("info", f"relock by re-enabling the lock (lock-without-lockpoint {fmt(wo)}"
                        + ("" if wo else ": ticked first") + "): it must find the right resonance by itself; the "
                        "wavemeter checks it did (with a FALC pro, the scan is on for a moment first, to null the "
                        "error signal's DC level)")
        add("info", "after a relock, the setpoint is moved to where the wavemeter puts the laser locked (the lockpoint "
                    "in the wavemeter's numbers)")
        pos = g["dl:lock:lockpoint:position"]
        if pts.get("l"):
            x, y, kind = pts["l"][0]
            add("ok", f"lockpoint chosen by hand: {x:.3f} V, {y:.3f} V, type {kind!r} (code {ord(kind)}); "
                      f"position {fmt(pos)}, type “{g['dl:lock:lockpoint:type'] or ''}”")
        else:
            add("info", f"lockpoint: position {fmt(pos)}, type “{g['dl:lock:lockpoint:type'] or ''}”")
        if pts:
            add("info", f"{len(pts.get('c', []))} lockpoint candidates now (the DLC pro finds them when asked, its "
                        f"Lock points shown: a recovery asks before each look at them)")
        spec, err = g["dl:lock:spectrum-input-channel"], g["dl:lock:error-channel"]
        win_on, win = g["dl:lock:window:enabled"], g["dl:lock:window:input-channel"]
        add("ok" if isinstance(spec, int) else "warn",
            f"lock input (the XY display's y, the peaks): {signal_name(spec) if spec is not None else '—'}")
        lo, hi = g["dl:lock:window:level-low"], g["dl:lock:window:level-high"]
        add("info", f"its lock detection (window) {fmt(win_on)}, on {signal_name(win) if win is not None else '—'}"
                    + (f", locked between {fmt(lo, ' V', 4)} and {fmt(hi, ' V', 4)}" if _num(lo) and _num(hi) else ""))
        add("info", f"watched, armed: locked more than {hop.peak_tol:g} MHz from the target, it is on the wrong peak: "
                    "relocked")
        name = transmission_input(*(g[n] for n in TRANS))
        if name is None:
            add("warn", "the transmission's present value cannot be read (neither input has one): a lock that keeps "
                        "losing it is not noticed")
        else:
            now = get(name)[name]
            usual, zero = hop.trans_locked, usable_zero(hop.trans_zero, hop.trans_locked)
            scale = usual or (now if _num(now) else 0.0)
            which = "its lock detection's input" if name == IO_VALUE.get(win) else "the lock input"
            add("ok" if _num(now) else "warn",
                f"transmission watched: {name} ({which}), now {volts(now, scale) + ' V' if _num(now) else '—'}")
            if not hop.trans_zero:
                zero_text = "0 V until measured (while the master scans, in a relock by lockpoint)"
            elif zero != hop.trans_zero:
                zero_text = f"{volts(hop.trans_zero, scale)} V measured, not well below its level locked: 0 V used"
            else:
                zero_text = f"{volts(zero, scale)} V, measured while the master scanned in a relock"
            add("info", f"its zero (off resonance, any DC offset): {zero_text}")
            add("info", "its level locked: " + (f"{volts(usual, scale)} V, " if usual else "not known yet, ")
                + "learned while it runs locked on target (the median of the last few minutes) and by a relock")
            limit = f"{volts(zero + hop.lock_min * (usual - zero), scale)} V" if usual > zero else "its limit"
            add("info", f"watched, armed: averaging below {limit} over 10 s ({hop.lock_min:g} of the way from its zero "
                        "to its level locked), the lock keeps losing it: relocked")
        add("ok" if isinstance(err, int) else "warn",
            f"error signal: {signal_name(err) if err is not None else '— (firmware before 2.1?)'}")
        if hop.peak_y:
            add("info", f"the right peak stood {hop.peak_y:.3f} V high last time; lockpoint type number "
                        f"{hop.lp_code if hop.lp_code >= 0 else 'not known yet'}")
        if mode == "lockpoint" and _num(amp) and amp <= 0:
            add("error", "the scan amplitude is 0: no peaks to pick from")

        add("head", "FALC pro")
        boards = {}
        for k in range(1, 5):
            b = get(f"falc{k}:input:offset", f"falc{k}:status-txt", f"falc{k}:serial-number",
                    f"falc{k}:main:lock-state")
            if _num(b[f"falc{k}:input:offset"]):
                boards[k] = b
                add("ok", f"falc{k} {b[f'falc{k}:serial-number'] or ''}: input offset "
                          f"{b[f'falc{k}:input:offset']:+.4f} V, status “{b[f'falc{k}:status-txt'] or '?'}”, "
                          f"main loop {'locked' if b[f'falc{k}:main:lock-state'] else 'not locked'}")
        sel = g["dl:lock:falc-selection"]
        if not boards:
            add("warn" if hop.falc else "info", "no FALC pro found"
                + (" (firmware before 2.2?)" if fw and fw < (2, 2) else "") + ": the error signal's DC is left alone")
        elif not hop.falc:
            add("info", "nulling the error signal's DC is off in the recovery settings")
        else:
            use = next(iter(boards)) if len(boards) == 1 else (sel if sel in boards else 0)
            if use and isinstance(err, int):
                add("ok", f"before relocking, the DC level of the error signal ({signal_name(err)}) is nulled with "
                          f"falc{use}'s input offset, by at most ±{hop.falc_max:g} V from where it is, as the scan "
                          f"shows it (left alone within {CavitySteps.FALC_TOL * 1e3:g} mV)"
                          + (" (the scan on for a moment: it relocks without lockpoint)" if mode == "plain" else ""))
                if hop.falc_gain:
                    add("info", f"the input offset moves the error signal {hop.falc_gain:+.3g} V/V (learned): straight "
                                f"there")
                else:
                    add("info", "how much the input offset moves the error signal is not known yet: the first relock "
                                "moves it a little of the way, then more, until the trace shows it, and learns that")
                if _num(amp) and amp <= 0:
                    add("warn", "the scan amplitude is 0: no DC level to see, the input offset is left alone")
            else:
                add("warn", f"cannot tell which FALC pro the lock uses (falc-selection {sel}): left alone")

        add("head", "Lock dropouts")
        r = get("recorder:enabled", "recorder:state-txt", "recorder:sample-count-limit")
        if not hop.dropouts:
            add("info", "not watched (Mode-hop recovery…: Log its lock dropouts)")
        elif r["recorder:enabled"] is None:
            add("warn", f"this controller has no recorder (laser{laser}:recorder): dropouts are watched by polling "
                        f"the transmission, which can miss those shorter than the time between reads (some ms)")
        else:
            add("ok", f"watched with the DLC pro's recorder (now “{r['recorder:state-txt'] or '?'}”), the transmission "
                      f"every {hop.drop_ms:g} ms, read every 2 s: while this program runs the recorder is its own, "
                      f"and its settings are put back when it closes")
            add("info", "the recorder is stopped while the lock should not hold (off, not reporting locked, being "
                        "recovered): while it records, the DLC pro refuses scope traces and its lock going on as busy "
                        "now and then, which a recovery needs")
        if hop.dropouts:
            add("info", f"a dropout: the transmission below the slipping limit (above) for two samples or more "
                        f"({2 * hop.drop_ms:g} ms), while its lock is on, reporting locked, and no recovery runs; each "
                        f"one goes to the console and to ~/.simplewavemeterlock-dropouts.jsonl, and how many, once an "
                        f"hour")

        add("head", "Scope (laser scope)")
        s = get("scope:channel1:signal", "scope:channel2:signal", "scope:channelx:xy-signal", "scope:variant")
        c1, c2, cx = s["scope:channel1:signal"], s["scope:channel2:signal"], s["scope:channelx:xy-signal"]
        add("ok", f"channel 1 {signal_name(c1)}, channel 2 {signal_name(c2)}, x {signal_name(cx)} "
                  f"(variant {s['scope:variant']})")
        if isinstance(err, int) and err not in (c1, c2) and boards and hop.falc:
            add("info", "the error signal is on neither channel: a recovery shows it on one for a moment, "
                        "then gives the channel back")

    if hop.shg:
        add("head", "SHG")
        g = get("nlo:shg:lock:lock-enabled", "nlo:shg:lock:state-txt", "nlo:shg:scan:enabled",
                "nlo:shg:scan:amplitude", "nlo:shg:scan:frequency", "nlo:shg:scan:offset",
                "nlo:pd:shg-int:photodiode", "nlo:pd:shg:power", "nlo:shg:scope:channel1:signal",
                "nlo:shg:scope:channel2:signal")
        if g["nlo:shg:lock:lock-enabled"] is None:
            add("error", "this laser has no SHG stage (no nlo:shg:lock): untick it in Mode-hop recovery…")
        else:
            add("ok", f"SHG cavity lock {fmt(g['nlo:shg:lock:lock-enabled'])} “{g['nlo:shg:lock:state-txt'] or '?'}”, "
                      f"intra-cavity signal {fmt(g['nlo:pd:shg-int:photodiode'], ' V')}, "
                      f"SHG power {fmt(g['nlo:pd:shg:power'], ' mW', 1)}")
            add("ok", f"SHG scan {fmt(g['nlo:shg:scan:enabled'])}: amplitude {fmt(g['nlo:shg:scan:amplitude'], ' V')}"
                      f" around {fmt(g['nlo:shg:scan:offset'], ' V')}, {fmt(g['nlo:shg:scan:frequency'], ' Hz', 1)}"
                      " (it must sweep at least one free spectral range of the SHG cavity)")
            c1, c2 = g["nlo:shg:scope:channel1:signal"], g["nlo:shg:scope:channel2:signal"]
            shown = SHG_INTRA in (c1, c2)
            add("ok" if shown else "info", f"SHG scope: channel 1 {signal_name(c1)}, channel 2 {signal_name(c2)}"
                + ("" if shown else ": a recovery shows the intra-cavity signal on channel 1 meanwhile, then "
                                    "gives it back"))
            if hop.shg_area:
                add("info", f"single-mode look learned: main peak area {hop.shg_area:.3g}, biggest side peak "
                            f"{hop.shg_side * 100:.0f} %")
            catch = hop.how() == "catch" and hop.opt_shg > 0
            add("info", (f"intra-cavity signal locked, usually {hop.shg_lock:.3f} V: " if hop.shg_lock else
                         "its usual intra-cavity signal locked is learned while it runs: ")
                + (f"a recovery's SHG lock must come within {100 * (1 - hop.catch_min):.0f} % of the optimum's "
                   f"({hop.opt_shg:.3f} V), else it is caught again" if catch else
                   f"a recovery's SHG lock must come up to {cavity.AREA_MIN:g} of that, else the master is not "
                   "single-mode (two modes alike look like one in a sweep) and the current is trimmed")
                + f"; watched, its average over 10 s below {hop.lock_min:g} of it starts a recovery")
            add("info", "whether the master lases is told by the SHG sweep too: flat (no resonance), it does not, "
                        "whatever its seed power says (a master that does not lase still sends its TA broadband "
                        "light); then the current jumps up until the sweep shows resonances again")

    add("head", "Finding a mode on target")
    if hop.how() == "catch":
        add("info", f"catch, as by hand: back onto the right mode (the one it was on target on before it hopped) by "
                    f"the current: up to the top of its window, then down {hop.step:g} mA at a time until the laser is "
                    f"on it and its SHG peak (the tallest resonance in the SHG sweep, between samples) is within "
                    f"{100 * (1 - hop.catch_min):.0f} % of the optimum's; on into the mode's best and back up to the "
                    f"top end of it. Not caught in {hop.catch_tries} goes, the same from the optimum's piezo voltage "
                    f"+ and - {hop.pc_try:g} V; then the piezo onto the target in {hop.walk_step:g} V steps, the "
                    f"current parked high in the mode every 4. A laser on the right mode and as good as that has its "
                    f"current parked at the top end of where the mode is at its best all the same (up {hop.step:g} mA "
                    f"at a time while its SHG peak stays at its best, then two back, no lower than it was: the "
                    f"feedforward takes it down as the piezo drifts up, relock after relock), then the piezo onto the "
                    f"target, and relocked. Not to be had at all: the mode it is on walked onto the target instead "
                    f"(walk)")
        if not hop.shg:
            add("info", f"with nothing but the wavemeter (no SHG stage): caught where it is on the right mode coming "
                        f"down, it is at the mode's edge; so the current goes on down until it hops off and back up "
                        f"until it is on it again, {hop.step:g} mA at a time, and it is parked in the middle of where "
                        f"the mode holds. Too little of it in the window, or not caught: the modes are mapped instead")
        if hop.optimum:
            g = get(piezo, "dl:cc:current-act")
            v, act = g[piezo], g["dl:cc:current-act"]
            add("ok", f"the optimum, taken {hop.opt_t or 'by hand'}: {hop.opt_i:.2f} mA, {hop.opt_v:.2f} V"
                      + (f", SHG {hop.opt_shg:.3f} V locked" if hop.opt_shg else "")
                      + (f" (now {act:.2f} mA, {v:.2f} V)" if _num(v) and _num(act) else ""))
            if not hop.opt_shg and hop.shg:
                add("warn", "the optimum's SHG level is not known (its SHG cavity was not locked when it was taken): "
                            "the SHG peak is compared with its usual level locked instead; take it again, locked")
            lo, hi = hop.opt_v - hop.pc_try, hop.opt_v + hop.pc_try
            if hop.pc_try > 0 and (lo < lp.v_min or hi > lp.v_max):
                add("warn", f"the tries from {lo:.1f} and {hi:.1f} V go beyond the lock's offset limits "
                            f"({lp.v_min:g} … {lp.v_max:g} V): those beyond are not tried")
        else:
            add("warn" if hop.shg else "info",
                "the optimum is not taken yet: set the laser up by hand, then click Relock (or right-click › Take the "
                "optimum now); until then the SHG peak is compared with its usual level locked, and the tries go "
                f"from where the piezo is ± {hop.pc_try:g} V")
        if hop.shg_ratio:
            add("info", f"its SHG sweep shows the peak at {hop.shg_ratio:.2f} of the signal locked (learned)")
    elif hop.how() == "walk":
        add("info", f"walk: single mode first by the SHG sweep, trimming the current; within {hop.reach:g} GHz (else "
                    f"first hop to a nearer mode with the current), the piezo onto the target in {hop.walk_step:g} V "
                    f"steps, the mode looked at after each and the current trimmed when it gets worse. A laser that "
                    f"is fine is only relocked: its current is not touched")
        if hop.kv and hop.reach * 1e3 > abs(hop.kv) * hop.v_window:
            add("warn", f"walking {hop.reach:g} GHz takes {hop.reach * 1e3 / abs(hop.kv):.1f} V of piezo, more than "
                        f"the ±{hop.v_window:g} V a recovery may move it")
    else:
        add("info", f"map: the current swept over its window both ways in {hop.step:g} mA steps, the best mode "
                    f"parked on the way it lases, the piezo onto the target")

    add("head", "What a recovery may change")
    add("info", "the current (within its window, ≤ 0.2 mA a step; higher for a laser that stopped lasing), the scan "
                "offset, the cavity lock and SHG lock on/off, the cavity lock's lock-without-lockpoint, the scan and "
                "SHG scan on/off, a scope channel (given back), a FALC pro's input offset (within its limit), "
                "find-candidates and select-lockpoint; nothing else. On failure all goes back (the current down "
                "slowly), except that the cavity lock stays off, and a laser that stopped lasing stays where it "
                "lases.")
    return out
