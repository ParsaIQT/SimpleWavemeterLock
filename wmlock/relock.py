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
3 is lock_shg; 4-6 relock_lockpoint, which also makes sure the wavemeter and the transmission
agree that it locked to the right peak. relock_plain is for a lock that finds its resonance by
itself (lock without lockpoint, or ReLock): it re-enables it and lets the wavemeter judge.

Every write goes through Runner, which refuses anything but WRITABLE (modehop.py) and puts it
back if the recovery fails; a cavity lock that was turned off stays off then. The scope
channels are borrowed only when they do not already show what is needed, and given back.

These are generator methods of Recenter (modehop.py), which mixes this class in.
"""
from __future__ import annotations

import math

from . import cavity
from . import modehop as mh
from .dlcpro import IO_VALUE, SHG_INTRA, DecopError, float_arrays, lock_points, signal_name

LOCK = "dl:lock:lock-enabled"
SCAN = "scan:enabled"
SHG_LOCK = "nlo:shg:lock:lock-enabled"
SHG_SCAN = "nlo:shg:scan:enabled"
SCAN_X = (34, 50, 101)  # signals that are the scan output / the piezo voltage


def says_locked(text) -> bool:
    return isinstance(text, str) and text.strip().lower().startswith("locked")


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


class CavitySteps:
    SHG_LOCK_S = 10.0   # s for the SHG cavity to lock
    LOCK_S = 10.0       # s for the cavity lock to report locked
    SETTLE_S = 0.5      # s after starting a scan or changing a scope channel, for fresh traces
    FALC_TOL = 0.002    # V: an error-signal DC level smaller than this is left alone

    def __init__(self, shg: bool):
        self.shg = shg
        self.shg_ok = shg          # its SHG sweep can be read
        ref = self.p.shg_ref
        self.shg_ref = ref if ref.known else None
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
        self.scope = {}            # scope channel -> signal shown now (what we may have changed)
        self.shg_level = None      # V: the intra-cavity signal with the SHG cavity locked, at the start
        self.mode_text = ""        # how single-mode it was when handed back
        self.falc_text = ""
        self.relock_trouble = ""   # why the last relock did not work
        self._blobs = {}

    # ------------------------------------------------------------ taking over
    def take_over(self):
        names = [LOCK, "dl:lock:state-txt", SCAN, "scan:amplitude", "scan:frequency", "nlo:shg:scan:frequency"]
        if self.cavity:
            names += ["dl:lock:lock-without-lockpoint", "dl:lock:candidates", "dl:lock:lockpoint:position",
                      "dl:lock:lockpoint:type", "dl:lock:spectrum-input-channel", "dl:lock:error-channel",
                      "dl:lock:falc-selection", "scope:channel1:signal", "scope:channel2:signal",
                      "scope:channelx:xy-signal"] + [f"falc{k}:input:offset" for k in range(1, 5)]
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
        if p.falc and mode == "lockpoint" and boards:
            if board and isinstance(o.get("dl:lock:error-channel"), int):
                self.falc_name = f"falc{board}:input:offset"
                self.falc_orig = float(o[self.falc_name])
            else:
                self.note("cannot tell which FALC pro the lock uses, or on which input its error signal is: "
                          "its input offset is left alone")

    def take_back(self):
        """After a relock that did not work: unlock again, for another go."""
        if self.cavity:
            yield mh.Set(LOCK, False, restore=False)
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

    def shg_sweep(self):
        """The SHG cavity's intra-cavity sweep, measured: a cavity.ModeCheck, None if unreadable."""
        yield mh.Wait(self.settle("nlo:shg:scan:frequency"), after_write=True)  # a sweep since the last change
        blob = yield from self.fresh("nlo:shg:scope:data")
        y = None
        if blob is not None:
            try:
                y = float_arrays(blob).get(self.shg_key)
            except DecopError:
                y = None
        if not y:
            self.shg_ok = False
            self.note("cannot read the SHG cavity's intra-cavity signal: no more SHG checks")
            return None
        m = cavity.sweep(y)
        self.last_sweep = m
        self._last_trace = y
        return m

    def keep_sweep(self, m, why: str):
        """Keep the last sweep for a look afterwards (the latest few, and the maps' examples)."""
        ok, text = cavity.judge(m, self.shg_ref)
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
        ok, text = cavity.judge(s, self.shg_ref)
        self.keep_sweep(s, "operating point")
        if ok is None:
            self.note(f"SHG cavity: {text}: carrying on without")
            return True
        if not ok:
            self.note(f"at {self.mA(self.I):.2f} mA, SHG cavity: {text}")
            return False
        self.mode_text = text
        learned = cavity.updated(self.p.shg_ref if self.p.shg_ref.known else None, s)
        self.learned.update(shg_area=round(learned.area, 5), shg_side=round(learned.side, 4), shg_n=learned.n_main)
        return True

    def lock_shg(self):
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
        good, text, intra = 0, None, None
        for _ in range(int(self.SHG_LOCK_S / 0.5)):
            yield mh.Wait(0.5)
            text, intra = yield mh.Get(("nlo:shg:lock:state-txt", "nlo:pd:shg-int:photodiode"))
            up = _num(intra) and intra >= level if level is not None else says_locked(text)
            good = good + 1 if up else 0
            if good >= 2:
                self.note("SHG cavity locked" + (f", intra-cavity signal {intra:.3f} V" if _num(intra) else ""))
                return True
        self.relock_trouble = (f"the SHG cavity does not lock ({text or '?'}"
                               + (f", intra-cavity signal {intra:.3f} V" if _num(intra) else "") + ")")
        self.note(self.relock_trouble)
        return False

    # ----------------------------------------------------------- handing back
    def hand_back(self):
        """Lock the SHG cavity, then the cavity lock: True when locked where it belongs."""
        if self.shg and not (yield from self.lock_shg()):
            return False
        if self.cavity:
            relock = self.relock_lockpoint if self.relock_mode == "lockpoint" else self.relock_plain
            if not (yield from relock()):
                return False
        yield from self.tidy()
        return True

    def relock_plain(self):
        yield from self.ask("Relock: re-enable the cavity lock")
        self.stage = "relocking the cavity"
        if self.scan_after_unlock:
            yield mh.Set(SCAN, True, restore=True)
        yield mh.Set(LOCK, True, restore=False)
        if (yield from self.verify()):
            return True
        self.relock_trouble = f"the cavity lock caught a peak {self.e:+.0f} MHz away"
        self.note(self.relock_trouble)
        return False

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
        choice, cands = None, []
        for _ in range(3):
            cands = yield from self.candidates()
            choice = cavity.choose_peak(cands, x_pred, self.kv, p.window, self.peak_y, self.lp_kind)
            if choice.ok:
                break
            if amp and abs(x_pred - self.V) > 0.4 * amp and self.v_lo <= x_pred <= self.v_hi:
                yield from self.move(offset=x_pred, read=False)  # the target in the middle of the scan
            yield mh.Wait(settle)
        yield from self.keep_scan(x_pred, choice, cands)
        if not choice.ok:
            raise mh.Failed(f"cannot tell which cavity peak is right: {choice.text} "
                            f"(left at the target, unlocked, scanning)", keep=True)
        self.note(f"cavity peak: {choice.text}")
        if self.falc_name:
            yield from self.null_falc()
        choice = yield from self.select(x_pred, choice)
        if choice is None:
            raise mh.Failed("the DLC pro would not take the lockpoint: select the peak by hand "
                            "(the laser is at the target, scanning)", keep=True)
        yield from self.ask(f"Lock to the peak at {choice.x:.3f} V")
        self.stage = "locking the cavity"
        yield mh.Set(LOCK, True, restore=False)
        if not (yield from self.wait_locked()):
            self.relock_trouble = "the cavity lock would not lock at the lockpoint"
            self.note(self.relock_trouble)
            return False
        if not (yield from self.verify()):
            self.relock_trouble = f"the cavity lock caught a peak {self.e:+.0f} MHz away"
            self.note(self.relock_trouble)
            return False
        level = yield from self.lock_level()
        if level is not None and choice.y > 0 and level < 0.5 * choice.y:
            self.relock_trouble = f"locked, but the transmission is only {level / choice.y:.2f} of the peak's"
            self.note(self.relock_trouble)
            return False
        self.learned.update(peak_y=round(choice.y, 4), lp_kind=choice.kind)
        self.note("locked to the right peak" + (f", transmission {level:.3f} V" if level is not None else ""))
        return True

    def candidates(self):
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
        for code in codes:
            for _ in range(2):
                cands = yield from self.candidates()
                fresh = cavity.choose_peak(cands, x_pred, self.kv, p.window, self.peak_y, self.lp_kind)
                if not fresh.ok:
                    why = fresh.text
                    break
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

    def lock_level(self):
        """The lock input's present value (the transmission, when locked), or None."""
        name = IO_VALUE.get(self.orig.get("dl:lock:spectrum-input-channel"))
        if name is None:
            return None
        (v,) = yield mh.Get((name,))
        return float(v) if _num(v) else None

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
    def error_trace(self):
        """(error signal, transmission or None) over a scan, borrowing a scope channel if need be."""
        o = self.orig
        err, spec = o.get("dl:lock:error-channel"), o.get("dl:lock:spectrum-input-channel")
        c1, c2 = "scope:channel1:signal", "scope:channel2:signal"
        if not isinstance(err, int):
            return None
        if err not in (self.scope.get(c1), self.scope.get(c2)):
            keep = c2 if self.scope.get(c1) == spec else c1  # keep the transmission in view
            yield mh.Set(keep, err, restore=o.get(keep))
            self.scope[keep] = err
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
        return e, t

    def null_falc(self):
        """Null the error signal's DC level with the FALC pro's input offset, by at most falc_max,
        checking on the error signal that each step does what it should."""
        p, name, o0 = self.p, self.falc_name, self.falc_orig
        tr = yield from self.error_trace()
        if tr is None:
            self.note("no error-signal trace: the FALC pro's input offset is left alone")
            return
        dc = before = cavity.dc_level(*tr)
        rec = self.record["falc"] = {"name": name, "offset0": o0, "dc0": round(dc, 5),
                                     "error": cavity.decimate(tr[0], 300)}
        if not math.isfinite(dc) or abs(dc) <= self.FALC_TOL:
            self.falc_text = f"error signal DC {dc * 1e3:+.1f} mV" if math.isfinite(dc) else ""
            return
        gain = p.falc_sign or 1  # how the DC level follows the input offset
        cur = o0
        for _ in range(4):
            new = min(max(cur - dc / gain, o0 - p.falc_max), o0 + p.falc_max)
            if abs(new - cur) < 1e-5:
                self.note(f"the FALC input offset is at its limit, {o0:+.4f} ± {p.falc_max:g} V")
                break
            yield from self.ask(f"Null the error signal's DC level ({dc * 1e3:+.1f} mV): FALC input offset "
                                f"{cur:+.4f} → {new:+.4f} V")
            self.stage = "nulling the error signal's DC level"
            yield mh.Set(name, new, restore=o0)
            tr = yield from self.error_trace()
            if tr is None:
                break
            dc2 = cavity.dc_level(*tr)
            g = (dc2 - dc) / (new - cur)
            if not math.isfinite(g) or abs(g) < 0.3:
                yield mh.Set(name, o0, restore=o0)
                err = self.orig.get("dl:lock:error-channel")
                self.note(f"the error signal on the scope ({signal_name(err)}) does not follow the FALC pro's input "
                          f"offset, so it is not the FALC's input: offset put back, left alone")
                self.learned["falc_sign"] = 0
                return
            gain, cur, dc = g, new, dc2
            if abs(dc) <= self.FALC_TOL:
                break
        self.learned["falc_sign"] = 1 if gain > 0 else -1
        rec.update(offset=round(cur, 5), dc=round(dc, 5))
        self.falc_text = (f"FALC input offset {o0:+.4f} → {cur:+.4f} V, error signal DC "
                          f"{before * 1e3:+.1f} → {dc * 1e3:+.1f} mV")
        self.note(self.falc_text)

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

    g = get("dl:cc:current-set", "dl:cc:current-act", "dl:cc:current-clip", "dl:cc:current-offset",
            "dl:cc:feedforward-enabled", "dl:cc:feedforward-factor", "dl:cc:output-filter:slew-rate-enabled",
            "dl:cc:output-filter:slew-rate")
    add("head", "Current")
    i, act, clip = g["dl:cc:current-set"], g["dl:cc:current-act"], g["dl:cc:current-clip"]
    off = act - i if _num(i) and _num(act) else None
    add("ok" if _num(i) and _num(act) else "error",
        f"measured {fmt(act, ' mA')}, set {fmt(i, ' mA')}" + (f" ({off:+.3f} mA apart)" if off is not None else "")
        + f", limit (current-clip, on the set current) {fmt(clip, ' mA')}")
    ff, factor = g["dl:cc:feedforward-enabled"], g["dl:cc:feedforward-factor"]
    add("info", f"current offset {fmt(g['dl:cc:current-offset'], ' mA')}; feedforward from the piezo "
                + (f"on, {factor:+.4f} mA/V" if ff and _num(factor) else fmt(ff)))
    if ff and _num(factor) and abs(factor) * hop.v_window >= 0.05:
        add("info", f"with the feedforward, a recovery's piezo moves (at most ±{hop.v_window:g} V) move the measured "
                    f"current too, by up to ±{abs(factor) * hop.v_window:.2f} mA, besides what it sets")
    if g["dl:cc:output-filter:slew-rate-enabled"]:
        add("warn", f"the current's slew rate is limited ({fmt(g['dl:cc:output-filter:slew-rate'])}): a recovery "
                    "steps it 0.2 mA at a time and expects each step to take effect at once")
    if hop.i_nom:
        lo, hi = hop.i_nom - hop.i_window, hop.i_nom + hop.i_window
        if _num(clip) and off is not None and hi - off > clip:
            add("info", f"the window's top, {hi:.2f} mA, is above current-clip: {clip + off:.2f} mA measured is "
                        "the limit")
            hi = clip + off
        inside = _num(act) and lo <= act <= hi
        add("ok" if inside else "error", f"recoveries keep the measured current within {lo:.2f} … {hi:.2f} mA "
                                         f"({hop.i_nom:.2f} ± {hop.i_window:g}), setting it ≤ 0.2 mA a step; "
                                         + ("it is inside now" if inside else "it is OUTSIDE now: no recovery"))
    else:
        add("info", "the nominal current is read, as measured, when recovery is enabled")

    g = get("scan:offset", "scan:enabled", "scan:amplitude", "scan:frequency", "scan:unit")
    add("head", "Piezo and scan")
    v = g["scan:offset"]
    inside = _num(v) and lp.v_min <= v <= lp.v_max
    add("ok" if inside else "error", f"offset {fmt(v, ' V')} (lock limits {lp.v_min:g} … {lp.v_max:g} V); "
                                     f"recoveries move it at most ±{hop.v_window:g} V")
    add("ok", f"scan {fmt(g['scan:enabled'])}, amplitude {fmt(g['scan:amplitude'], ' V')}, "
              f"{fmt(g['scan:frequency'], ' Hz', 1)}")
    amp = g["scan:amplitude"]
    if hop.kv:
        add("info", f"measured piezo tuning {hop.kv / 1e3:+.3f} GHz/V"
                    + (f": the scan spans {abs(hop.kv) * amp / 1e3:.2f} GHz" if _num(amp) else "")
                    + (f", modes {hop.spacing / 1e3:.2f} GHz apart" if hop.spacing else ""))

    if hop.cavity:
        add("head", "Cavity lock (DLC pro lock module)")
        g = get("dl:lock:lock-enabled", "dl:lock:state", "dl:lock:state-txt", "dl:lock:lock-without-lockpoint",
                "dl:lock:lockpoint:position", "dl:lock:lockpoint:type", "dl:lock:candidates",
                "dl:lock:spectrum-input-channel", "dl:lock:error-channel", "dl:lock:falc-selection",
                "dl:lock:relock:enabled", "dl:lock:window:enabled")
        add("ok" if g["dl:lock:lock-enabled"] is not None else "error",
            f"lock {fmt(g['dl:lock:lock-enabled'])}, state {g['dl:lock:state']} “{g['dl:lock:state-txt'] or '?'}”; "
            f"ReLock {fmt(g['dl:lock:relock:enabled'])}, window {fmt(g['dl:lock:window:enabled'])}")
        add("info", "this program does not lock it to the wavemeter: its row's button is Relock, which has it "
                    "watched and relocked after a mode hop (and relocks it then and there if it is not locked on "
                    "target)")
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
            add("ok", f"relock by lockpoint (lock-without-lockpoint {fmt(wo)}): start the scan, pick the tall peak "
                      f"within ±{hop.window:.0f} MHz of where the wavemeter puts the setpoint, select it, lock")
        else:
            add("info", f"relock by re-enabling the lock (lock-without-lockpoint {fmt(wo)}): it must find the "
                        f"right resonance by itself; the wavemeter checks it did")
        pos = g["dl:lock:lockpoint:position"]
        if pts.get("l"):
            x, y, kind = pts["l"][0]
            add("ok", f"lockpoint chosen by hand: {x:.3f} V, {y:.3f} V, type {kind!r} (code {ord(kind)}); "
                      f"position {fmt(pos)}, type “{g['dl:lock:lockpoint:type'] or ''}”")
        else:
            add("info", f"lockpoint: position {fmt(pos)}, type “{g['dl:lock:lockpoint:type'] or ''}”")
        if pts:
            add("info", f"{len(pts.get('c', []))} lockpoint candidates now (they are found while the scan runs)")
        spec, err = g["dl:lock:spectrum-input-channel"], g["dl:lock:error-channel"]
        level = None
        if IO_VALUE.get(spec):
            level = get(IO_VALUE[spec])[IO_VALUE[spec]]
        add("ok" if isinstance(spec, int) else "warn",
            f"lock input (the XY display's y, the peaks): {signal_name(spec) if spec is not None else '—'}"
            + (f", now {level:.3f} V" if _num(level) else ""))
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
                          f"falc{use}'s input offset, by at most ±{hop.falc_max:g} V from where it is")
            else:
                add("warn", f"cannot tell which FALC pro the lock uses (falc-selection {sel}): left alone")

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

    add("head", "What a recovery may change")
    add("info", "the current (within its window, ≤ 0.2 mA a step), the scan offset, the cavity lock and SHG lock "
                "on/off, the scan and SHG scan on/off, a scope channel (given back), a FALC pro's input offset "
                "(within its limit) and select-lockpoint; nothing else. On failure all goes back, "
                "except that the cavity lock stays off.")
    return out
