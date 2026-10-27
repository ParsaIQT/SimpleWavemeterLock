"""This PC's wavemeters, each with an engine of its own (and its own instance on the network: share.py).

A PC may run several wavemeters (each with its WLM software). One program reads them all, an engine each: each is
framed in the window under its own name, with its own channels, lasers, reference laser and recoveries, and has a
frame of its own on the other PCs. The settings file is the first one's, as it was with one wavemeter (saying which
one it is: 'wavemeter': {'serial': 1234, 'model': 'WS7'}); the others' are in it under 'other_wavemeters', by serial
number. A wavemeter found running on this PC without an engine gets one, with settings of its own, unless it was
hidden; the first engine is given its wavemeter before any other is: the only one, or of several the one whose
readings its setpoints know (else the first not read by another copy of the program).

Settings given the wrong wavemeter (each laser's channel shows another laser): swap them with the other's. One copy
of the program reads a PC's wavemeters (claims, wavemeter.py, keep another copy off them): MCC's analog output
boards also take one program at a time (dac.py), and this one drives all the lasers on them, whichever wavemeter
reads them.
"""
from __future__ import annotations

import logging
import threading
import time

from .wavemeter import meter_name

log = logging.getLogger(__name__)


def _serial(key):
    try:
        n = int(key)
    except (TypeError, ValueError):
        return None
    return n if n > 0 else None


class Meter:
    """A wavemeter of this PC: its engine, and its instance on the network (None: without)."""

    def __init__(self, engine, share=None):
        self.engine, self.share = engine, share

    @property
    def serial(self):
        """Its serial number: the source's, once it reads one; until then, what its settings say (None: nothing)."""
        return getattr(self.engine.source, "serial", None) or self.engine.meter.get("serial")

    @property
    def name(self) -> str:
        """'WS7 1234' ('' if nothing says)."""
        m = self.engine.meter
        return getattr(self.engine.source, "name", "") or meter_name(m.get("model", ""), m.get("serial"))

    @property
    def label(self) -> str:
        """Its name on every PC (share.py), or what it is."""
        return self.share.label if self.share is not None else (self.name or "this PC")

    def idle(self) -> bool:
        """Nothing of it is locked, watched or being recovered."""
        with self.engine.lock:
            return all(ch.state == "off" for ch in self.engine.channels.values())

    def start(self):
        self.engine.start()
        if self.share is not None:
            self.share.start()

    def stop(self):
        if self.share is not None:
            self.share.stop()
        self.engine.stop()


class Meters:
    """The wavemeters of this PC, a Meter each: the first's settings are the file's, the others' under it."""

    SWAP_S = 5.0  # s after a swap: nothing new made meanwhile (each source on its way to the other's wavemeter)

    def __init__(self, config: dict, first: Meter, backend=None, make=None):
        """config: the settings (the first wavemeter's; the others' under 'other_wavemeters'); first: its meter (its
        engine has config); backend: what finds the wavemeters running here and makes their sources (known() or
        scan() -> [Found], quickly; free(serial): nothing reads it; swap(a, b): two sources each read the other's),
        None: there is no other; make(section, serial, model) -> Meter: a meter for that wavemeter, with its
        settings section (its engine and share not started)."""
        self.config, self.backend, self.make = config, backend, make
        self.items = [first]
        self.version = 0   # bumped when meters come or go
        self.dirty = False  # settings changed here (meters come or go), not yet saved
        self._quiet_until = 0.0  # (monotonic) a swap: no meter made until then
        sections = self.sections
        for key in list(sections):
            serial = _serial(key)
            if serial is None or not isinstance(sections[key], dict):
                del sections[key]
            elif not sections[key].get("hidden") and make is not None and serial != first.serial:
                self.items.append(self._make(sections[key], serial))

    @property
    def sections(self) -> dict:
        """serial (str) -> the settings of a wavemeter other than the first (with 'hidden': not shown, not read)."""
        s = self.config.get("other_wavemeters")
        if not isinstance(s, dict):
            s = self.config["other_wavemeters"] = {}
        return s

    def _make(self, section: dict, serial: int, model: str = "") -> Meter:
        meter = section.get("wavemeter")
        if not isinstance(meter, dict) or meter.get("serial") != serial:
            section["wavemeter"] = {"serial": serial, "model": model or (meter or {}).get("model", "")}
        return self.make(section, serial, section["wavemeter"].get("model", ""))

    def start(self):
        for m in self.items:
            m.start()

    def stop(self):
        """Each closed, as the program closes: their shares first (no adjustments from other PCs meanwhile)."""
        for m in self.items:
            if m.share is not None:
                m.share.stop()
        for m in self.items:
            if m.share is not None:
                m.share.join(1.0)
        threads = [threading.Thread(target=m.engine.stop, daemon=True) for m in self.items]
        for t in threads:  # (each may wait for a recovery that was cut short to put its laser back)
            t.start()
        for t in threads:
            t.join(30.0)

    # ----------------------------------------------------------------- found
    def poll(self) -> list:
        """Wavemeters newly found running here, each given a meter, started: those meters."""
        if self.backend is None or self.make is None or not self.items[0].serial:
            return []  # (the first engine is given its wavemeter before any other is)
        if time.monotonic() < self._quiet_until:
            return []
        try:
            found = getattr(self.backend, "known", self.backend.scan)()  # (as last found: never waiting on the DLL)
        except OSError:
            return []
        have = {m.serial for m in self.items}
        new = []
        for f in found:
            if not f.serial or f.serial in have:
                continue
            section = self.sections.get(str(f.serial))
            if (section is not None and section.get("hidden")) or not self.backend.free(f.serial):
                continue
            if section is None:
                section = self.sections[str(f.serial)] = {}
                log.info("another wavemeter on this PC: %s", meter_name(f.model, f.serial))
            m = self._make(section, f.serial, f.model)
            self.items.append(m)
            have.add(f.serial)
            m.start()
            new.append(m)
        if new:
            self.version += 1
            self.dirty = True
        return new

    # ----------------------------------------------------------------- the window's
    def hidden(self) -> list:
        """The wavemeters hidden: [(serial, name)]."""
        out = []
        for key, section in sorted(self.sections.items()):
            serial = _serial(key)
            if serial is not None and isinstance(section, dict) and section.get("hidden"):
                m = section.get("wavemeter") or {}
                out.append((serial, meter_name(m.get("model", ""), serial)))
        return out

    def hide(self, m: Meter):
        """That wavemeter (not the first; nothing of it locked) neither shown nor read any more; its settings kept
        (show() brings it back). Its engine stops in a thread of its own."""
        if m is self.items[0] or m not in self.items:
            return
        self.items.remove(m)
        cfg = m.engine.to_config()
        cfg["hidden"] = True
        self.sections[str(m.serial)] = cfg
        threading.Thread(target=m.stop, daemon=True, name="hide wavemeter").start()
        self.version += 1
        self.dirty = True

    def show(self, serial: int):
        """A hidden wavemeter shown, and read, again: its meter (started), or None."""
        section = self.sections.get(str(serial))
        if not isinstance(section, dict) or self.make is None or any(m.serial == serial for m in self.items):
            return None
        section.pop("hidden", None)
        m = self._make(section, serial)
        self.items.append(m)
        m.start()
        self.version += 1
        self.dirty = True
        return m

    def swap(self, a: Meter, b: Meter):
        """a's settings (its channels, lasers, reference...) go with b's wavemeter and b's with a's: for settings
        that were given the wrong one. Neither may have anything locked. Each source reads the other's in a moment."""
        sa, sb = a.engine.source, b.engine.source
        self._quiet_until = time.monotonic() + self.SWAP_S
        swap = getattr(self.backend, "swap", None)
        if swap is not None:
            swap(sa, sb)
        else:
            wa, wb = getattr(sa, "wanted", None), getattr(sb, "wanted", None)
            sa.select(wb)
            sb.select(wa)  # (each engine's settings say their new wavemeter as its source reads it: saved then)

    # ----------------------------------------------------------------- settings
    def to_config(self) -> dict:
        """All the settings: the first's, with the others' (hidden ones too) under 'other_wavemeters'."""
        cfg = self.items[0].engine.to_config()
        others = {k: v for k, v in self.sections.items() if isinstance(v, dict) and v.get("hidden")}
        for m in self.items[1:]:
            cfg_m = m.engine.to_config()
            serial = (cfg_m.get("wavemeter") or {}).get("serial") or m.serial  # (its settings say whose they are)
            if serial:
                others[str(serial)] = cfg_m
        self.config["other_wavemeters"] = others
        if others:
            cfg["other_wavemeters"] = others
        else:  # (a PC with one wavemeter: its settings as they were)
            cfg.pop("other_wavemeters", None)
        self.dirty = False
        return cfg


# ---------------------------------------------------------------------------------------- lasers elsewhere
LOST_FLAGS = ("SLS LOCK LOST", "BEAT LOCK LOST", "NOT LASING", "CAVITY LOCK SLIPPING")  # on a row: its lock lost


def row_holds(v: dict) -> tuple:
    """Whether a laser's lock holds, by what its row shows (view.py; another PC's as it sends it): (True, False, None:
    no telling), and why not. Its own PC says (holds); one running an older version, by its row's words."""
    if "holds" in v:
        return v["holds"] if isinstance(v["holds"], bool) else None, str(v.get("holds_why") or "")
    state, info = v.get("state"), v.get("info") or ""
    if state in ("recovering", "asking"):
        return False, "being recovered"
    flag = next((f for f in LOST_FLAGS if f in info), None)
    if flag:
        return False, flag.lower()
    if state == "on" and "SLS UNIT NOT ANSWERING" not in info and "SLS LOCK OFF" not in info:
        return True, ""
    return None, ""


class Elsewhere:
    """Lasers on this PC's other wavemeters and on other PCs, as an engine follows one (a beat lock's master: sls.py).
    row(label, n): {'f': THz or None, 't': monotonic time of that reading (None: none), 'holds': its lock, by what it
    shows (True, False, None: no telling), 'why', 'name', 'label'}, or None (not shown, or its PC not heard from);
    another PC's readings are asked for at once (fast) while followed. lasers(): [(label, n, name)] of them all. A
    lock held by the caller is fine: no engine's lock is taken (another wavemeter's channel is read as it is), and
    the network's only briefly."""

    WATCH_S = 10.0  # s another PC's readings are asked for at once, after each look

    def __init__(self, meters, share=None):
        """meters: () -> this PC's wavemeters (Meter); share: this PC's instance on the network that hears the others'
        rows (the first wavemeter's), None: none."""
        self.meters, self.share = meters, share

    def label_of(self, engine) -> str:
        """The name on the network of that engine's wavemeter ('': none of this PC's)."""
        return next((m.label for m in self.meters() if m.engine is engine), "")

    def lasers(self) -> list:
        out = []
        for m in self.meters():
            out += [(m.label, n, ch.name) for n, ch in sorted(m.engine.channels.items()) if not ch.hidden]
        share = self.share
        if share is not None and share.on:
            now = time.monotonic()
            for p in share.snapshot():
                if p.status(now) not in ("live", "late"):
                    continue
                with share.lock:
                    rows = [(n, (p.views.get(n) or {}).get("name", "")) for n in p.rows]
                out += [(p.label, n, name) for n, name in rows]
        return out

    def row(self, label: str, n: int):
        now = time.monotonic()
        for m in self.meters():
            if m.label == label:
                ch = m.engine.channels.get(n)
                if ch is None:
                    return None
                holds, why = m.engine.holds_of(ch)
                f = ch.freq if ch.freq > 0 else None
                return {"f": f, "t": ch.t if ch.t > 0 else None, "holds": holds, "why": why, "name": ch.name,
                        "label": label, "dev": ch.error_mhz if f and ch.setpoint_nm else None}
        share = self.share
        if share is None or not share.on:
            return None
        for p in share.snapshot():
            if p.label != label or p.status(now) not in ("live", "late"):
                continue
            with share.lock:
                v, got = p.views.get(n), p.got.get(n)
            if v is None:
                return None
            share.watch(p.id, self.WATCH_S)
            age = v.get("age")
            t = got - age if age is not None and got is not None else None
            f = v.get("f")
            holds, why = row_holds(v)
            dev = v.get("dev")
            return {"f": f if f else None, "t": t, "holds": holds, "why": why, "name": v.get("name", ""),
                    "label": label, "dev": dev if isinstance(dev, (int, float)) and f else None}
        return None
