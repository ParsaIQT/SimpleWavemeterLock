"""SLS (Stable Laser Systems) lock electronics on the LAN: a laser one of them locks, watched; one a PLL beat-locks to
a master, guarded.

Each unit (a Raspberry Pi with an FPGA, known on the LAN by its serial number: P1029A, P1029H, ...) answers HTTP on
port 8080 with the parameters of its serial console (115200 baud on its front USB: get, set):

    http://P1029H:8080/?parameter=PLLLockState                      -> PLLLockState=AUTO_LOCKED
    http://P1029H:8080/?parameter=PLLManualLockEnable&value=false   -> PLLManualLockEnable=false   (a write: echoed)
    http://P1029H:8080/?getValues                                   -> every parameter, Name=value a line
    http://P1029H:8080/?appCommand=save                             -> OK

Three kinds of unit (their names for what this reads):
  SLS Laser Servo             locks a laser to the cavity (PDH): AutoLockEnable, AutoLockState ('Locked: 3 (0d
                              01:00:21)', its relocks and how long since the last; 'Idle' off), Transmission and
                              TransmissionThreshold (V; its lock detection relocks below), LockCount, LockTime (s)
  SuperHeterodyne Laser PLL   beat-locks a laser to another, one channel: AutoLockEnable, PLLManualLockEnable,
                              PLLLockState, PLLAmplitude and PLLAmpThreshold (V: the beat note in its IF band, there
                              with the servos off too), PLLPhase, CurrentServoOutput (fast), PZTServoOutput (slow,
                              ±5 V), PLLLockCount, LockFrequency (Hz)
  3-Channel Laser PLL         the same for three lasers, ServoN... (N = 1, 2, 3): ServoNAutoLockEnable,
                              ServoNManualLockEnable, ServoNLockState, ServoNPhaseError, ServoNFastServoOutput,
                              ServoNSlowServoOutput, ServoNLockCount, ServoNLockFrequency (no amplitude)

A laser such a unit locks is watched (SlsWatch, POLL_S apart): whether it holds (by what the unit says: its state,
its transmission or beat amplitude; a PLL's slow output at its rail or running away; the beat as the wavemeter reads
it), its lock dropouts (from the last poll it held to the first it holds again: a drop shorter than a poll shows as a
relock counted, between the two polls).

A slave a PLL beat-locks with its manual lock (the autolock not used) is lost for good when its master's lock drops
even for a moment: the beat leaves, the PLL's integrators run its current and piezo to their rails, the slave stops
lasing, and its TA, without seed, switches itself off. The guard (Guard), for such a slave: its PLL released
(manual lock off: the outputs back to zero) the moment its master's lock or the beat is lost, before the outputs run
away; the slave, if it stopped lasing or hopped, recovered by its mode-hop recovery (its current jumped up first, for
the TA's sake); once its master has held MASTER_S, the slave brought by the wavemeter lock to its beat point (its
master's reading plus the beat, as learned locked) and its PLL engaged again there, or a little either side, until it
holds; after `tries` that do not, it waits RETRY_S and tries again, the slave left lasing where it is.
"""
from __future__ import annotations

import html
import http.client
import ipaddress
import logging
import math
import re
import socket
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass, fields
from urllib.parse import quote

log = logging.getLogger(__name__)

PORT = 8080
KINDS = ("pdh", "pll")
TITLES = {"pdh": "SLS Laser Servo (its lock to the cavity)", "pll": "SLS PLL (beat-locked to a master)"}

PDH = {"auto": "AutoLockEnable", "state": "AutoLockState", "trans": "Transmission",
       "trans_min": "TransmissionThreshold", "detect": "LockDetectEnable", "count": "LockCount", "held": "LockTime",
       "fast": "CurrentServoOutput", "slow": "PZTServoOutput"}
PLL1 = {"auto": "AutoLockEnable", "auto2": "PLLAutoLockEnable", "manual": "PLLManualLockEnable",
        "state": "PLLLockState", "status": "PLLLockStatus", "amp": "PLLAmplitude", "amp_min": "PLLAmpThreshold",
        "phase": "PLLPhase", "fast": "CurrentServoOutput", "slow": "PZTServoOutput", "count": "PLLLockCount",
        "held": "PLLLockTime", "freq": "LockFrequency", "detect": "LockDetectEnable"}
PLL3 = {"auto": "Servo{n}AutoLockEnable", "manual": "Servo{n}ManualLockEnable", "state": "Servo{n}LockState",
        "status": "Servo{n}LockStatus", "phase": "Servo{n}PhaseError", "fast": "Servo{n}FastServoOutput",
        "slow": "Servo{n}SlowServoOutput", "count": "Servo{n}LockCount", "held": "Servo{n}LockTime",
        "freq": "Servo{n}LockFrequency", "detect": "Servo{n}LockDetectEnable"}
BAD_WORDS = ("UNLOCK", "FAIL", "LOST", "SEARCH", "ACQUIR", "SWEEP", "ABORT", "RELOCK")  # in a PLL's state: not holding
BAND_K, FAST_FLOOR = 6.0, 0.05    # a PLL's fast output further than this many times its usual spread (and V) from
PHASE_K, PHASE_FLOOR = 8.0, 0.1   # where it is holding: lost (likewise its phase error)


def names(kind: str, servo: int = 0) -> dict:
    """What each thing read is called on that unit: role -> parameter (pll: servo 0, a one-channel PLL; 1..3, that
    channel of a 3-channel PLL)."""
    if kind == "pdh":
        return dict(PDH)
    if servo:
        return {k: v.format(n=int(servo)) for k, v in PLL3.items()}
    return dict(PLL1)


class SlsError(Exception):
    """The unit answered, but not with what was asked (an unknown parameter, an HTTP error)."""


def parse(text: str) -> dict:
    """'Name=value' lines: {name: value} (values as text, stripped)."""
    out = {}
    for line in text.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            if k.strip():
                out[k.strip()] = v.strip()
    return out


def _text(body: bytes) -> str:
    """A reply's text: plain, as the units send it (a browser shows it as such), or with tags, dropped."""
    s = body.decode("utf-8", errors="replace")
    if "<" in s:
        s = re.sub(r"(?i)<br\s*/?>|</p>|</div>|</li>|</tr>", "\n", s)
        s = html.unescape(re.sub(r"<[^>]*>", "", s))
    return s


def as_bool(s):
    """A unit's on/off ('1', 'true', 'TRUE', 'on', 'yes'...): True, False, or None if it is neither."""
    w = str(s).strip().lower() if s is not None else ""
    return True if w in ("1", "true", "on", "yes") else False if w in ("0", "false", "off", "no") else None


def as_float(s):
    try:
        x = float(str(s).strip())
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def lookup(host: str) -> str:
    """A unit's address: its name (its serial number, P1029F) looked up; not found so, as it says it itself by mDNS
    (P1029F.local: Windows 10 and on look such names up). socket.gaierror if neither is found."""
    host = host.strip()
    try:
        return socket.gethostbyname(host)
    except socket.gaierror as e:
        if not host or "." in host or ":" in host:
            raise
        try:
            return socket.gethostbyname(host + ".local")
        except OSError:
            raise e from None


class Sls:
    """One unit's HTTP server, for one thread at a time (a connection kept open while it lets it)."""

    def __init__(self, host: str, port: int = PORT, timeout: float = 1.0):
        self.host, self.port, self.timeout = host, int(port or PORT), timeout
        self._conn = None
        self._ip = None  # its address, its name looked up once (a slow name service: not at every poll)

    def close(self):
        conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except OSError:
                pass

    def ask(self, query: str) -> str:
        """GET /?query: the reply's text. OSError if the unit does not answer; SlsError for an HTTP error."""
        for attempt in range(2):
            fresh = self._conn is None
            try:
                if fresh:
                    if self._ip is None:
                        self._ip = lookup(self.host)
                    self._conn = http.client.HTTPConnection(self._ip, self.port, timeout=self.timeout)
                self._conn.request("GET", "/?" + query)
                r = self._conn.getresponse()
                body = r.read()
            except (http.client.HTTPException, OSError) as e:
                self.close()
                if fresh:
                    self._ip = None  # (looked up again next time: its address may have changed)
                if fresh or attempt:  # (a connection kept open that the unit had closed: once more, on a new one)
                    raise e if isinstance(e, OSError) else ConnectionError(f"{type(e).__name__}: {e}") from e
                continue
            if r.will_close:
                self.close()
            if r.status != 200:
                raise SlsError(f"HTTP {r.status} {r.reason}: {_text(body).strip()[:120]}")
            return _text(body)
        raise ConnectionError("no answer")  # (not reached)

    def get(self, name: str) -> str:
        return self._value(self.ask("parameter=" + quote(name)), name)

    def set(self, name: str, value) -> str:
        """Write a parameter: the value the unit says it took."""
        if isinstance(value, bool):
            v = "true" if value else "false"
        else:
            v = repr(float(value)) if isinstance(value, float) else str(value)
        return self._value(self.ask(f"parameter={quote(name)}&value={quote(v)}"), name)

    def values(self) -> dict:
        """Every parameter, {name: value as text}."""
        got = parse(self.ask("getValues"))
        if not got:
            raise SlsError("getValues: nothing")
        return got

    def command(self, name: str) -> str:
        """An application command (save, reboot, resetDrift): what it says ('OK')."""
        return self.ask("appCommand=" + quote(name)).strip()

    @staticmethod
    def _value(text: str, name: str) -> str:
        got = parse(text)
        for k, v in got.items():
            if k.lower() == name.lower():
                return v
        raise SlsError(f"{name}: {text.strip()[:120] or 'no answer'}")


# ------------------------------------------------------------------------------------------------- settings
@dataclass
class SlsParams:
    kind: str = ""            # '': none; 'pdh': an SLS Laser Servo locks it to the cavity; 'pll': a PLL beat-locks it
    host: str = ""            # the unit, by its name on the LAN (its serial number: P1029H) or address
    port: int = PORT
    servo: int = 0            # pll: 0 a one-channel PLL (SuperHeterodyne); 1..3 that channel of a 3-channel PLL
    master: int = 0           # pll: its master's wavemeter channel (0: none)...
    master_on: str = ""       # ...on that wavemeter, as the network names it ('': this one): another of this PC's, or
    #                           another PC's
    master_host: str = ""     # pll: its master's SLS Laser Servo, polled from here too ('': not; its master's PC may
    master_port: int = PORT   # watch it, and say so on its row)
    beat: float = 0.0         # pll: MHz, its frequency less its master's as the wavemeters read them, its PLL
    beat_known: bool = False  # holding it (kept learned while it does: two wavemeters' disagreement drifts)
    guard: bool = True        # pll: armed (Relock), its PLL released when its master's lock or its beat is lost, and
    #                           relocked once the master holds again (else only watched)
    capture: float = 0.6      # pll: MHz: brought this close to its beat point by the wavemeter, its PLL is engaged
    tries: int = 6            # pll: tries in a row that fail before it waits Guard.RETRY_S, and tries again
    off_mhz: float = 10.0     # pll: its reading further than this from its beat point (two in a row), it has lost it
    slow_max: float = 5.0     # pll: V: its slow output's range, either way; at 95 % of it, it is at its rail (0: never)
    fast_max: float = 0.0     # pll: V: likewise its fast output (0: not watched: what units it shows is not known)
    ramp: float = 1.0         # pll: V/s: its slow output running faster than this, it has lost it (0: not watched)
    amp_usual: float = 0.0    # one-channel PLL: V, its beat amplitude locked (learned)
    fast_mid: float = 0.0     # pll: V, its fast output, holding: its usual (median)...
    fast_dev: float = 0.0     # ...and how far from it it goes (learned; 0: not yet): far beyond, it has lost the beat
    phase_mid: float = 0.0    # pll: likewise its phase error (its unit's units)
    phase_dev: float = 0.0
    trim: bool = False        # pll: its slow servo unused (as on Sirius): the slave's piezo moved a little at a time to
    #                           keep its PLL's fast output near 0 (else that runs out of range as the slave drifts)
    trim_gain: float = 0.0    # pll: V its fast output moves for a V of the slave's piezo (learned; 0: not yet)...
    trim_cur: float = 0.0     # ...and for a mA of its current (learned for an upkeep: its piezo then moves with it)
    FRESH = ("fast_mid", "fast_dev", "phase_mid", "phase_dev")  # (learned afresh each time its PLL holds it: not kept)

    @classmethod
    def from_dict(cls, d) -> "SlsParams":
        out = cls()
        for f in fields(cls):
            if isinstance(d, dict) and f.name in d:
                kind = {"bool": bool, "int": int, "str": str}.get(str(f.type), float)
                try:
                    setattr(out, f.name, kind(d[f.name]))
                except (TypeError, ValueError):
                    pass
        if out.kind not in KINDS:
            out.kind = ""
        if isinstance(d, dict) and "beat_known" not in d and out.beat:  # (settings from before beat_known)
            out.beat_known = True
        return out

    def to_dict(self) -> dict:
        d = asdict(self)
        for k in self.FRESH:
            d.pop(k, None)
        return d

    @property
    def on(self) -> bool:
        return self.kind in KINDS and bool(self.host.strip())

    @property
    def unit(self) -> str:
        """The unit (and channel) as said: 'P1029H', 'P1029F servo 2'."""
        return self.host.strip() + (f" servo {self.servo}" if self.kind == "pll" and self.servo else "")

    def same_unit(self, other: "SlsParams") -> bool:
        return ((self.kind, self.host.strip(), self.port, self.servo)
                == (other.kind, other.host.strip(), other.port, other.servo))

    @property
    def master_where(self) -> str:
        """Its master as said: 'channel 9' (this wavemeter's), 'channel 3 on Lab B' ('': none)."""
        if not self.master:
            return ""
        return f"channel {self.master}" + (f" on {self.master_on.strip()}" if self.master_on.strip() else "")


# ------------------------------------------------------------------------------------------------- what it says
@dataclass
class View:
    """What one poll of a unit says of the laser it locks (kept on its channel: the row, the API, the guard)."""
    kind: str
    t: float                    # time.time(): when it was polled
    took: float = 0.0           # s the poll took
    mode: str = ""              # its lock: 'auto' (its autolock), 'manual' (a PLL's manual lock), '' off
    holds: bool | None = None   # it holds the laser, by what it shows (None: off, or no telling)
    why: str = ""               # why not
    state: str = ""             # its state as it says it
    count: int | None = None    # relocks counted
    held: float | None = None   # s since its last (re)lock, as it says
    trans: float | None = None  # pdh: V
    trans_min: float | None = None
    amp: float | None = None    # one-channel PLL: V, the beat note
    amp_min: float | None = None  # ...below this, no beat: its threshold, or half its usual
    fast: float | None = None   # V: its outputs
    slow: float | None = None
    phase: float | None = None
    freq: float | None = None   # pll: Hz, its lock frequency
    band: bool = False          # pll: not holding by where its outputs sit (far from where they sat holding it)

    def said(self) -> str:
        """Its state, short, for the row."""
        if self.kind == "pdh":
            text = self.state or "?"
            if self.trans is not None:
                below = f" (relocks below {self.trans_min:.3f})" if self.trans_min else ""
                text += f" · transmission {self.trans:.3f} V{below}"
            return text
        text = {"manual": "manual lock", "auto": "autolock", "": "off"}[self.mode]
        if self.state:
            text += f" ({self.state})"
        if self.amp is not None:
            text += f" · beat {self.amp:.2f} V" + (f" (lost below {self.amp_min:.2f})" if self.amp_min else "")
        outs = [f"{name} {v:+.3f} V" for name, v in (("slow", self.slow), ("fast", self.fast)) if v is not None]
        return text + (" · " + ", ".join(outs) if outs else "")


def _num(values: dict, name: str):
    return as_float(values.get(name)) if name else None


def pdh_view(t: float, values: dict, took: float = 0.0) -> View:
    """An SLS Laser Servo's lock to the cavity: holding if its autolock says locked, its transmission (its lock
    detection on) not below its threshold."""
    n = PDH
    v = View("pdh", t, took)
    auto = as_bool(values.get(n["auto"]))
    v.state = values.get(n["state"], "").strip()
    v.trans, v.trans_min = _num(values, n["trans"]), _num(values, n["trans_min"])
    v.fast, v.slow = _num(values, n["fast"]), _num(values, n["slow"])
    count, v.held = _num(values, n["count"]), _num(values, n["held"])
    v.count = int(count) if count is not None else None
    detect = as_bool(values.get(n["detect"]))
    v.mode = "auto" if auto or (auto is None and v.state.lower().startswith("locked")) else ""
    if not v.mode:
        v.holds, v.why = None, "its autolock is off"
    elif not v.state.lower().startswith("locked"):
        v.holds, v.why = False, f"“{v.state or '?'}”"
    elif detect is not False and v.trans is not None and v.trans_min and v.trans < v.trans_min:
        v.holds, v.why = False, f"transmission {v.trans:.3f} V, below its threshold {v.trans_min:.3f} V"
    else:
        v.holds = True
    return v


def pll_view(t: float, values: dict, servo: int, p: SlsParams, prev: View | None = None, took: float = 0.0) -> View:
    """A PLL's beat lock: on in its manual lock or its autolock; holding unless its autolock's state says otherwise,
    its beat amplitude (a one-channel PLL) is below its threshold (else half its usual), its slow output is at its rail
    or running away (since prev, the poll before), its fast output at its rail (fast_max), or its fast output or its
    phase error far from where they sat while it held it (learned: a 3-channel PLL shows no amplitude)."""
    n = names("pll", servo)
    v = View("pll", t, took)
    manual = as_bool(values.get(n["manual"]))
    auto = as_bool(values.get(n["auto"]))
    if auto is None and "auto2" in n:
        auto = as_bool(values.get(n["auto2"]))
    v.mode = "manual" if manual and not auto else "auto" if auto else ""
    v.state = (values.get(n["state"]) or values.get(n["status"]) or "").strip()
    v.amp, v.phase = _num(values, n.get("amp")), _num(values, n["phase"])
    v.fast, v.slow, v.freq = _num(values, n["fast"]), _num(values, n["slow"]), _num(values, n["freq"])
    count, v.held = _num(values, n["count"]), _num(values, n["held"])
    v.count = int(count) if count is not None else None
    if v.amp is not None:
        amp_min = _num(values, n.get("amp_min"))
        v.amp_min = amp_min if amp_min else (0.5 * p.amp_usual if p.amp_usual > 0 else None)
    if not v.mode:
        v.holds, v.why = None, "its PLL is off"
        return v
    up = v.state.upper()
    bad = None
    if v.mode == "auto":  # (its state is its autolock's: in its manual lock, maybe a failed autolock's, long gone)
        bad = next((w for w in BAD_WORDS if w in up), None) or ("IDLE" if "IDLE" in up else None)
    if bad:
        v.holds, v.why = False, f"its PLL says “{v.state}”"
    elif v.amp is not None and v.amp_min and v.amp < v.amp_min:
        v.holds, v.why = False, f"no beat: amplitude {v.amp:.3f} V (below {v.amp_min:.3f} V)"
    elif p.slow_max > 0 and v.slow is not None and abs(v.slow) >= 0.95 * p.slow_max:
        v.holds, v.why = False, f"its slow output at its rail, {v.slow:+.2f} V"
    elif p.fast_max > 0 and v.fast is not None and abs(v.fast) >= 0.95 * p.fast_max:
        v.holds, v.why = False, f"its fast output at its rail, {v.fast:+.3f} V"
    elif p.fast_dev > 0 and v.fast is not None and abs(v.fast - p.fast_mid) > max(BAND_K * p.fast_dev, FAST_FLOOR):
        v.holds, v.band, v.why = False, True, (f"its fast output {v.fast:+.3f} V, far from where it holds it "
                                               f"({p.fast_mid:+.3f} ± {p.fast_dev:.3f} V)")
    elif (p.phase_dev > 0 and v.phase is not None
          and abs(v.phase - p.phase_mid) > max(PHASE_K * p.phase_dev, PHASE_FLOOR)):
        v.holds, v.band, v.why = False, True, (f"its phase error {v.phase:+.3f}, far from where it holds it "
                                               f"({p.phase_mid:+.3f} ± {p.phase_dev:.3f})")
    elif (p.ramp > 0 and prev is not None and prev.mode and v.slow is not None and prev.slow is not None
          and 0.0 < v.t - prev.t < 2.0 and abs(v.slow - prev.slow) >= max(p.ramp * (v.t - prev.t), 0.05)):
        v.holds, v.why = False, (f"its slow output running away ({(v.slow - prev.slow) / (v.t - prev.t):+.1f} V/s, "
                                 f"now {v.slow:+.2f} V)")
    else:
        v.holds = True
    return v


BRIEF = {"pll": ("manual", "fast", "slow", "phase", "amp"), "pdh": ("auto", "state", "trans")}  # what a guard needs at
#   every poll of a unit read a value at a time (the rest at every SlsWatch.FULL_N-th)


class Interrupted(Exception):
    """A poll left half done: something more urgent came up (its master's lock lost)."""


def read(client: Sls, kind: str, servo: int = 0, whole: bool = True, brief: bool = False, abort=None) -> dict:
    """What a poll needs of a unit: all its values in one request (whole), else the parameters one by one (a unit that
    has not every one, or none for getValues, says so: SlsError); brief: only those BRIEF names; abort: () -> True to
    stop half way (Interrupted)."""
    if whole:
        return client.values()
    out = {}
    for role, name in names(kind, servo).items():
        if role in ("detect", "freq", "status", "auto2") or (brief and role not in BRIEF[kind]):
            continue  # (not needed at every poll)
        if abort is not None and abort():
            raise Interrupted()
        try:
            out[name] = client.get(name)
        except SlsError:
            if role in ("state", "manual" if kind == "pll" else "auto"):
                raise
    return out


VNC_PORT, SSH_PORT = 5900, 22  # what else a unit answers on (its GUI, by VNC; its Raspberry Pi, by SSH)
SLOW_S = 6.0  # s a unit is given for one value, its whole list (getValues) not answering in time


def knock(ip: str, port: int, timeout: float = 1.5) -> str:
    """A TCP port at an address: 'open', 'refused' (nothing listens there), 'silent' (no answer: dropped on the way,
    or nothing there), or why not (no route...)."""
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return "open"
    except ConnectionRefusedError:
        return "refused"
    except socket.timeout:
        return "silent"
    except OSError as e:
        return str(e) or type(e).__name__


def diagnose(host: str, port: int = PORT, timeout: float = 1.5) -> tuple:
    """Why a unit does not answer its HTTP requests, as far as can be told from here: (what, why), what one of 'name'
    (not known to this PC), 'refused', 'open' (its port takes connections: its server is stuck, or slow), 'dropped'
    (it is there, VNC or SSH answering, but its port does not), 'absent' (nothing answers), or 'error'."""
    try:
        ip = lookup(host)
    except OSError:
        return "name", (f"the name {host.strip()} is not known to this PC: type its IP address instead (on its "
                        "touchscreen: Settings › System Control)")
    http = knock(ip, port, timeout)
    if http == "refused":
        return "refused", (f"{ip} is there, but nothing listens on its port {port}: its HTTP server is not running "
                           "(Settings › System Control › Reboot; else ask Stable Laser Systems)")
    if http == "open":
        return "open", (f"{ip} takes connections on its port {port} but does not answer them: its HTTP server is "
                        "stuck or slow (Settings › System Control › Reboot)")
    others = [name for name, p in (("VNC", VNC_PORT), ("SSH", SSH_PORT)) if knock(ip, p, timeout) == "open"]
    if others:
        return "dropped", (f"{ip} is there ({' and '.join(others)} answer{'' if len(others) > 1 else 's'}), but its "
                           f"port {port} does not answer at all: a firewall drops it (on the unit, or between), or "
                           "its HTTP server is not running")
    if http == "silent":
        return "absent", f"{ip} does not answer at all: not on a network this PC reaches, or off"
    return "error", f"{ip}: {http}"


def _one_by_one(host: str, port: int, kind: str, servo: int) -> tuple:
    """What a poll needs, a value at a time, each given SLOW_S (a unit whose whole list is too slow), its firmware's
    version too: (values, s its first value took (behind a whole list it was still sending), s each other took).
    OSError, SlsError as read()."""
    c = Sls(host, port, SLOW_S)
    try:
        t0 = time.monotonic()
        try:
            app = c.get("AppVersion")
        except SlsError:
            app = None
        t1 = time.monotonic()
        values = read(c, kind, servo, whole=False)
        each = (time.monotonic() - t1) / max(len(values), 1)
        if app is not None:
            values["AppVersion"] = app
        return values, t1 - t0, each
    finally:
        c.close()


def probe(host: str, port: int = PORT, kind: str = "pll", servo: int = 0, timeout: float = 2.0) -> str:
    """One look at a unit (the settings dialog's Test): what it is and what it says now, or why not."""
    c = Sls(host, port, timeout)
    note = ""
    try:
        t0 = time.monotonic()
        try:
            values = c.values()
        except socket.timeout:  # (TimeoutError from Python 3.10 on)
            c.close()
            what, why = diagnose(host, port)
            if what != "open":  # (its port does not even take connections)
                return f"{host}:{port}: timed out\n{why}"
            t0 = time.monotonic()
            values, first, each = _one_by_one(host, port, kind or "pll", servo)
            note = (f"\n(its whole list did not come in {timeout:g} s: read a value at a time, as it is polled: "
                    f"{each * 1e3:.0f} ms each; the first {first * 1e3:.0f} ms, behind the whole list)")
        took = time.monotonic() - t0
    except (OSError, SlsError) as e:
        text = f"{host}:{port}: {e or type(e).__name__}"
        if isinstance(e, SlsError):
            return text
        return text + "\n" + diagnose(host, port)[1]
    finally:
        c.close()
    app = values.get("AppVersion", "").replace("AppVer:", "").strip()
    is_pll = any(k.startswith(("PLL", "Servo1")) for k in values)
    is_three = "Servo1LockState" in values
    what = "3-channel PLL" if is_three else "PLL" if is_pll else "Laser Servo" if "AutoLockState" in values else "?"
    head = f"{host}: {what}" + (f", firmware {app}" if app else "") + f", answered in {took * 1e3:.0f} ms"
    if kind == "pll":
        if is_three != bool(servo) or not is_pll:
            return head + (": not a 3-channel PLL: servo 0" if is_pll and servo else
                           ": a 3-channel PLL: which servo, 1 to 3?" if is_three else ": not a PLL")
        view = pll_view(time.time(), values, servo, SlsParams(kind="pll"))
    else:
        if is_pll:
            return head + ": a PLL, not a Laser Servo"
        view = pdh_view(time.time(), values)
    return f"{head}\n{view.said()}" + (f"\nnot holding: {view.why}" if view.holds is False else "") + note


def _identify(host: str, port: int) -> dict:
    """What a unit is and what it does, a value at a time (its whole list too slow), each given SLOW_S."""
    c = Sls(host, port, SLOW_S)
    out = {}
    try:
        for name in ("AppVersion", "Servo1LockState", "PLLLockState", "AutoLockState"):
            try:
                out[name] = c.get(name)
            except SlsError:
                pass
        if "Servo1LockState" in out:
            for k in (1, 2, 3):
                out.update(read(c, "pll", k, whole=False))
                out[f"Servo{k}LockFrequency"] = c.get(f"Servo{k}LockFrequency")
        elif "PLLLockState" in out:
            out.update(read(c, "pll", 0, whole=False))
            out["LockFrequency"] = c.get("LockFrequency")
        elif "AutoLockState" in out:
            out.update(read(c, "pdh", 0, whole=False))
        return out
    finally:
        c.close()


def survey(host: str, port: int = PORT, timeout: float = 1.5) -> str:
    """What a unit is and what it says now, in a line (the settings dialog's Find units), or why not."""
    c = Sls(host, port, timeout)
    try:
        try:
            values = c.values()
        except socket.timeout:  # (TimeoutError from Python 3.10 on)
            c.close()
            what, why = diagnose(host, port, timeout)
            if what != "open":
                return why
            values = _identify(host, port)  # (its port takes connections: its whole list is slow)
    except (OSError, SlsError) as e:
        text = str(e) or type(e).__name__
        if "Name or service" in text or "getaddrinfo" in text or "11001" in text or "nodename" in text:
            return "not found on the LAN"
        return text if isinstance(e, SlsError) else diagnose(host, port, timeout)[1]
    finally:
        c.close()
    now = time.time()
    if "Servo1LockState" in values:
        parts = []
        for k in (1, 2, 3):
            v = pll_view(now, values, k, SlsParams(kind="pll"))
            freq = f" at {v.freq / 1e6:g} MHz" if v.freq else ""
            parts.append(f"servo {k} " + ({"manual": "manual lock", "auto": "autolock", "": "off"}[v.mode]) + freq)
        return "3-channel PLL: " + "; ".join(parts)
    if any(k.startswith("PLL") for k in values):
        v = pll_view(now, values, 0, SlsParams(kind="pll"))
        freq = f" at {v.freq / 1e6:g} MHz" if v.freq else ""
        return f"one-channel PLL{freq}: {v.said()}"
    if "AutoLockState" in values:
        return f"Laser Servo (to the cavity): {pdh_view(now, values).said()}"
    return f"answers, but is none of the units known here ({len(values)} values)"


def private_nets(addresses) -> list:
    """The /24 networks of this PC's private addresses (10/8, 172.16/12, 192.168/16: where lab instruments sit; no
    other network is scanned)."""
    nets = []
    for a in addresses:
        try:
            ip = ipaddress.IPv4Address(a)
        except ValueError:
            continue
        if ip.is_private and not (ip.is_loopback or ip.is_link_local or ip.is_unspecified):
            net = ipaddress.IPv4Network(f"{a}/24", strict=False)
            if net not in nets:
                nets.append(net)
    return nets


def scan(nets, ports=(PORT, VNC_PORT), timeout: float = 0.4, workers: int = 64) -> dict:
    """{address: [those of its ports that are open]} of the hosts of those networks that answer on any of them (a
    connection each, many at once)."""
    jobs = [(str(h), p) for net in nets for h in net.hosts() for p in ports]
    found = {}
    if not jobs:
        return found
    with ThreadPoolExecutor(max_workers=min(workers, len(jobs))) as pool:
        for (ip, p), state in zip(jobs, pool.map(lambda j: knock(j[0], j[1], timeout), jobs)):
            if state == "open":
                found.setdefault(ip, []).append(p)
    return found


def units(prefix: str = "P1029", letters: str = "ABCDEFGH", port: int = PORT, timeout: float = 1.5,
          nets=None) -> list:
    """[(name or address, what it is and says)]: the units named prefix + a letter (P1029A ... P1029H), asked at once;
    then the hosts of this PC's private networks (nets: those; None, as its addresses say) that answer on port, and
    those that answer VNC only (said so: a unit whose HTTP server does not answer)."""
    names = [prefix + c for c in letters]
    found = {}

    def ask(where):
        found[where] = survey(where, port, timeout)

    def ask_all(wheres):
        threads = [threading.Thread(target=ask, args=(w,), daemon=True) for w in wheres]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout + 4 * SLOW_S)

    ask_all(names)
    out = [(n, found.get(n, "no answer")) for n in names]
    if nets is None:
        from .dlcpro import lan_addresses

        nets = private_nets(lan_addresses())
    if not nets:
        return out
    known = set()
    for n, said in out:
        if said != "not found on the LAN":
            try:
                known.add(lookup(n))
            except OSError:
                pass
    hosts = scan(nets, (port, VNC_PORT))
    order = sorted(hosts, key=ipaddress.IPv4Address)
    serving = [ip for ip in order if port in hosts[ip] and ip not in known]
    ask_all(serving)
    out += [(ip, found.get(ip, "no answer")) for ip in serving]
    out += [(ip, f"VNC answers, its port {port} does not: a unit whose HTTP server does not answer?")
            for ip in order if port not in hosts[ip]]
    return out


# ------------------------------------------------------------------------------------------------- the guard
@dataclass
class Look:
    """What the guard is told at each step."""
    now: float                    # time.time()
    armed: bool                   # its row's Relock on (Watching)
    pll: View | None              # its PLL's last poll (None: it does not answer)
    fresh: bool = True            # pll is a new poll (else a wake-up: its master's lock lost)
    master: bool | None = None    # its master's lock: holding, lost, None no telling (not watched, not answering)
    master_s: float | None = None  # s its master has held (or, its lock not watched, read steady); None no telling
    beat_off: str = ""            # the wavemeter reads it off its beat point (two readings in a row): how far
    near_n: int = 0               # readings in a row within capture of where it is being steered
    lasing: bool | None = None    # by its seed power (None: no telling)
    far: str = ""                 # the wavemeter reads it too far from its beat point to steer it (a hop), or not
    can_steer: bool = False       # the wavemeter lock can steer it there (a controller linked, readings, a beat point)
    can_recover: bool = False     # its mode-hop recovery is set up
    recovering: bool = False      # its mode-hop recovery runs
    recovered: str | None = None  # its mode-hop recovery just ended: '' it did, else how it failed


class Guard:
    """The guard of a slave its PLL's manual lock beat-locks (see the module): told what is known at each step (Look),
    it says what to do: ('release', why), ('engage',), ('steer', MHz from the beat point), ('unsteer',), ('recover',
    why, urgent), ('disarm', why), ('held',), ('gave_up', why). Its stage and why are shown.

    Stages: idle (not armed), auto (its autolock has it: watched), hold (its manual lock holds it), released (by this
    guard: waiting for its master and the slave), recover (its mode-hop recovery has it), steer (the wavemeter
    lock brings it to its beat point), engage (its manual lock just engaged: does it hold?), failed (given up for
    RETRY_S)."""

    BAD_N = 2          # polls in a row not holding (its own signals) before it is released; at once, its master lost
    MASTER_S = 3.0     # s its master must have held (or read steady) before the slave is engaged again
    STEER_N = 3        # readings in a row within capture of the point steered to: there (no amplitude: engaged)
    DWELL_N = 4        # with a beat amplitude: readings more there, its amplitude looked at, before the next point
    ENGAGE_AMP = 0.7   # with a beat amplitude: engaged once it is this much of its usual (and above its threshold)
    STEER_S = 45.0     # s steering to a point may take
    SETTLE_S = 0.15    # s after engaging before it is judged (released at the first poll not holding)...
    VERIFY_S = 1.5     # ...and how long then it must hold
    RETRY_S = 300.0    # s after giving up before it tries again...
    GIVE_UPS = 3       # ...given up this many times in a row (no relock between), it tries no more (Relock: again)
    WAIT_S = 90.0      # s a slave that does not lase, or reads far off, is waited for with nothing to recover it
    AGAIN_S = 1.0      # s before a write the unit did not take is made again
    SEARCH = (0, 1, -1, 2, -2, 3, -3, 4, -4)  # engagements: at the beat point, then a half capture either side...

    def __init__(self, params: SlsParams):
        self.p = params
        self.stage = "idle"
        self.why = ""        # why it was released (or gave up)
        self.since = 0.0     # time.time(): the stage began
        self.tries = 0       # engagements that did not hold, in a row
        self.gave_up = 0     # times it gave up, in a row
        self.next_t = 0.0    # failed: tries again then
        self.bad_n = 0
        self.wrote = 0.0     # time.time(): the last release or engagement written...
        self.writes = 0      # ...engagements written in this stage
        self.steer_to = 0.0  # MHz from the beat point, being steered to...
        self.search_j = 0    # ...the search's point (a beat amplitude to go by: looked for there, not engaged)
        self.tried_recovery = False

    @property
    def relocking(self) -> bool:
        """It has the slave (released, recovering, steering, engaging): its channel is being recovered."""
        return self.stage in ("released", "recover", "steer", "engage")

    def _go(self, stage: str, now: float):
        self.stage, self.since = stage, now

    def step(self, k: Look) -> list:
        out = []
        p, now = k.pll, k.now
        mode = p.mode if p is not None else None
        if not k.armed:
            if self.stage == "steer":
                out.append(("unsteer",))
            if self.stage != "idle":
                self._go("idle", now)
            return out
        if self.stage == "idle":
            if p is None:
                return out  # (armed while its PLL does not answer: nothing to go by)
            self.tries, self.why = 0, ""
            if mode == "auto":
                self._go("auto", now)
            elif mode == "manual" and p.holds is not False and k.master is not False and not k.beat_off:
                self._go("hold", now)
            else:  # (armed off, or not holding: relocked)
                why = ("its PLL was off" if mode == "" else p.why or (k.beat_off and f"the wavemeter reads it "
                                                                                     f"{k.beat_off}")
                       or "its master's lock is lost")
                self._release(out, now, why, write=mode == "manual")
            return out
        if self.stage == "auto":
            if mode == "manual":
                self._go("hold", now)
            elif mode == "":
                out.append(("disarm", "its PLL was switched off at its box"))
                self._go("idle", now)
            return out
        if self.stage == "hold":
            if p is None:
                return out  # (no answer: nothing can be done; the row says so)
            if mode == "":
                out.append(("disarm", "its PLL was switched off at its box"))
                self._go("idle", now)
                return out
            if mode == "auto":
                self._go("auto", now)
                return out
            if k.master is False:
                self._release(out, now, "its master's lock was lost")
                return out
            bad = p.why if p.holds is False else (f"the wavemeter reads it {k.beat_off}" if k.beat_off else "")
            if bad and k.fresh:
                self.bad_n += 1
            elif not bad:
                self.bad_n = 0
            if self.bad_n >= self.BAD_N:
                self._release(out, now, bad)
            return out
        if self.stage == "released":
            if mode == "manual" and now - self.wrote >= self.AGAIN_S:  # (the release did not take: again)
                out.append(("release", self.why))
                self.wrote = now
                return out
            if mode == "auto":
                self._go("auto", now)
                return out
            if self._rescue(k, out):
                return out
            if not self._master_ready(k):
                return out
            if k.can_steer:
                self.search_j = 0
                self._steer(out, now, self._point(p))
            else:
                self._engage(out, now)
            return out
        if self.stage == "recover":
            if k.recovered is not None:
                self._go("released", now)
                if k.recovered:
                    self._give_up(out, now, f"its mode-hop recovery failed: {k.recovered}")
            return out
        if self.stage == "steer":
            amp = p is not None and p.amp is not None
            want = max(p.amp_min or 0.0, self.ENGAGE_AMP * self.p.amp_usual) if amp else 0.0
            if k.master is False or k.lasing is False or k.far:
                out.append(("unsteer",))
                self._go("released", now)
            elif amp and want and p.amp >= want:  # (the beat in its band: engaged)
                out.append(("unsteer",))
                self._engage(out, now)
            elif k.near_n >= self.STEER_N and not (amp and want):
                out.append(("unsteer",))
                self._engage(out, now)
            elif k.near_n >= self.STEER_N + self.DWELL_N:  # (no beat there: the next point)
                self.search_j += 1
                if self.search_j >= len(self.SEARCH):
                    out.append(("unsteer",))
                    self._failed_try(out, now, f"no beat found within {self._offset(len(self.SEARCH) - 1):.1f} "
                                               f"MHz of its beat point (its amplitude stayed below {want:.2f} V)")
                else:
                    self._steer(out, now, self._offset(self.search_j))
            elif now - self.since > self.STEER_S:
                out.append(("unsteer",))
                self._failed_try(out, now, f"not brought within {self.p.capture:g} MHz of its beat point in "
                                           f"{self.STEER_S:g} s")
            return out
        if self.stage == "engage":
            t = now - self.since
            if t < self.SETTLE_S:
                return out
            if p is None:
                if t > self.SETTLE_S + 4 * self.VERIFY_S:
                    self._failed_try(out, now, "its PLL did not answer after it was engaged")
                return out
            if mode != "manual":
                if now - self.wrote >= self.AGAIN_S:
                    if self.writes >= 2:  # (written twice: it does not take it)
                        self._failed_try(out, now, "its PLL does not take its manual lock")
                    else:
                        out.append(("engage",))
                        self.wrote, self.writes = now, self.writes + 1
                return out
            bad = (p.why if p.holds is False else "its master's lock was lost" if k.master is False else
                   f"the wavemeter reads it {k.beat_off}" if k.beat_off else "")
            if bad and k.fresh:
                out.append(("release", f"it did not hold: {bad}"))
                self.wrote = now
                self._failed_try(out, now, bad)
            elif not bad and t >= self.SETTLE_S + self.VERIFY_S:
                self.tries, self.bad_n, self.gave_up = 0, 0, 0
                self._go("hold", now)
                out.append(("held",))
            return out
        if self.stage == "failed":
            if mode == "manual" and p.holds:  # (engaged by hand, and it holds)
                self.tries = self.gave_up = 0
                self._go("hold", now)
                out.append(("held",))
            elif now >= self.next_t and self.gave_up < self.GIVE_UPS:
                self.tries = 0
                self.tried_recovery = False
                self._go("released", now)
        return out

    def _release(self, out: list, now: float, why: str, write: bool = True):
        if write:
            out.append(("release", why))
        self.why, self.wrote, self.bad_n = why, now, 0
        self.tried_recovery = False
        self._go("released", now)

    def _engage(self, out: list, now: float):
        out.append(("engage",))
        self.wrote, self.writes = now, 1
        self.tried_recovery = False  # (an engagement that does not hold may stop it lasing: recovered again)
        self._go("engage", now)

    def _offset(self, j: int) -> float:
        """MHz from the beat point: the search's j-th point (half a capture apart, either way in turn)."""
        return 0.5 * self.p.capture * abs(self.SEARCH[j]) * (1 if self.SEARCH[j] >= 0 else -1)

    def _point(self, p) -> float:
        """Where to steer it first: its beat point (a beat amplitude to go by, the search finds it about there); with
        none, a point further out at each engagement that did not hold."""
        if p is not None and p.amp is not None:
            return 0.0
        return self._offset(self.tries % len(self.SEARCH))

    def _steer(self, out: list, now: float, mhz: float):
        self.steer_to = mhz
        out.append(("steer", mhz))
        self._go("steer", now)

    def _rescue(self, k: Look, out: list) -> bool:
        """The slave does not lase, or reads too far off to steer: its mode-hop recovery (once) if it has one; else
        waited for, WAIT_S at most. True: not yet to be engaged."""
        if k.lasing is not False and not k.far:
            return False
        why = "it does not lase" if k.lasing is False else k.far
        if k.recovering:
            return True
        if k.can_recover and not self.tried_recovery:
            self.tried_recovery = True
            out.append(("recover", why, k.lasing is False))
            self._go("recover", k.now)
        elif k.now - self.since > self.WAIT_S:
            self._give_up(out, k.now, f"{why}" + ("" if k.can_recover else " (no mode-hop recovery set up for it)"))
        return True

    def _master_ready(self, k: Look) -> bool:
        """Its master has held MASTER_S (watched), or read steady that long; not told, MASTER_S since the release."""
        if k.master is False:
            return False
        if k.master_s is not None:
            return k.master_s >= self.MASTER_S
        return k.now - self.since >= self.MASTER_S

    def _failed_try(self, out: list, now: float, why: str):
        self.tries += 1
        self.why = why
        if self.tries >= max(self.p.tries, 1):
            self._give_up(out, now, f"{self.tries} tries in a row failed, the last: {why}")
        else:
            self._go("released", now)

    def _give_up(self, out: list, now: float, why: str):
        self.why = why
        self.gave_up += 1
        self.next_t = now + self.RETRY_S
        self._go("failed", now)
        out.append(("gave_up", why))

    def said(self, now: float) -> str:
        """Its stage, for the row."""
        if self.stage == "released":
            return "its PLL released: waiting for its master to hold, the slave to lase"
        if self.stage == "recover":
            return "its PLL released: the slave being recovered"
        if self.stage == "steer":
            off = f" {self.steer_to:+.2f} MHz" if self.steer_to else ""
            return f"its PLL released: the wavemeter brings it to its beat point{off} (try {self.tries + 1})"
        if self.stage == "engage":
            return f"its PLL engaged: does it hold? (try {self.tries + 1})"
        if self.stage == "failed":
            if self.gave_up >= self.GIVE_UPS:
                return f"its PLL released, gave up {self.gave_up} times in a row: {self.why}; Relock to try again"
            return f"its PLL released, gave up: {self.why}; tries again in {max(self.next_t - now, 0.0):.0f} s"
        return {"auto": "its autolock has it", "hold": "its manual lock holds it", "idle": ""}.get(self.stage, "")


# ------------------------------------------------------------------------------------------------- the watch
class SlsWatch(threading.Thread):
    """Polls one unit for one channel's laser, POLL_S apart (as fast as it answers, if slower), on a connection of its
    own; calls back step(watch, view or None, fresh) -> [actions], each poll (None: no answer) and when woken, and does
    the writes it says: ('release', why), ('engage',). wrote(watch, action, error or '') after each."""

    POLL_S = 0.1
    SLOW_S = 0.3   # a poll taking longer than this: said once (a dropout shorter than a poll may be missed)
    RETRY_S = 2.0  # s between tries while it does not answer
    WHOLE_S = 0.5  # s its whole list (getValues) may take (not coming in its timeout, or slower: read a value at a time
    #                from then on; a slow server sends it whole long after it is no longer wanted)
    ONE_S = 3.0    # s each value is given, read a value at a time (the first may wait for a whole list it still sends)
    FULL_N = 10    # read a value at a time: every value at every FULL_N-th poll, the others only what BRIEF names

    def __init__(self, p: SlsParams, step, wrote=None, name: str = "", clock=time.time):
        super().__init__(daemon=True, name=f"sls {name or p.unit}")
        self.p = p
        self.client = Sls(p.host.strip(), p.port, timeout=1.0)
        self.step, self.wrote_cb, self.clock = step, wrote, clock
        self.label = name or p.unit
        self._stop_ev = threading.Event()
        self._wake = threading.Event()
        self.whole = True      # all values in one request (getValues), else one by one...
        self.carry = {}        # ...its values at the last poll of them all (carried to the brief ones between)
        self.error = ""        # why it does not answer ('' it does)
        self.last = None       # its last View
        self.polls = 0
        self.slow_said = False

    def wake(self):
        """Look now (its master's lock lost, say)."""
        self._wake.set()

    def stop(self):
        self._stop_ev.set()
        self._wake.set()

    def run(self):
        while not self._stop_ev.is_set():
            t0 = self.clock()
            woken = self._wake.is_set()
            self._wake.clear()
            if woken and self.last is not None and not self.error:
                view, fresh = self.last, False  # (act at once on what is known: its master's lock lost, say)
            else:
                view, fresh = self._poll(self.last)
            try:
                actions = self.step(self, view, fresh)
            except Exception:
                log.exception("%s: step", self.label)
                actions = []
            for a in actions or ():
                self._write(a)
            if self._stop_ev.is_set():
                break
            wait = self.POLL_S if not self.error else self.RETRY_S
            self._wake.wait(max(wait - (self.clock() - t0), 0.0))
        self.client.close()

    def _poll(self, prev) -> tuple:
        """(view, fresh): a new poll's, or the last (a poll cut short: woken), or (None, False): no answer."""
        t0 = self.clock()
        try:
            try:
                values = self._read()
            except SlsError as e:
                if not self.whole:
                    raise
                self._one_by_one(str(e))  # (no getValues there, or not all of it)
                values = self._read()
            except socket.timeout:  # (TimeoutError from Python 3.10 on)
                if not self.whole:
                    raise
                self._one_by_one(f"its whole list does not come in {self.client.timeout:g} s")
                values = self._read()
        except Interrupted:
            self._wake.clear()  # (acted on now)
            return (self.last, False) if self.last is not None else (None, False)
        except (OSError, SlsError) as e:
            self._fail(str(e) or type(e).__name__)
            return None, False
        t1 = self.clock()
        if self.whole and t1 - t0 > self.WHOLE_S:
            self._one_by_one(f"its whole list takes {(t1 - t0) * 1e3:.0f} ms")
        if self.error:
            log.info("%s: %s answers again", self.label, self.p.host)
        self.error = ""
        self.polls += 1
        if t1 - t0 > self.SLOW_S and not self.slow_said and self.polls > 3:
            self.slow_said = True
            log.warning("%s: %s takes %.0f ms to answer a poll: a dropout shorter than that may be missed", self.label,
                        self.p.host, (t1 - t0) * 1e3)
        t = 0.5 * (t0 + t1)
        if self.p.kind == "pdh":
            view = pdh_view(t, values, t1 - t0)
        else:
            view = pll_view(t, values, self.p.servo, self.p, prev, t1 - t0)
        self.last = view
        return view, True

    def _read(self) -> dict:
        """Its values: in one request, else one at a time (the brief ones, all of them every FULL_N-th poll), a poll
        cut short when woken (Interrupted)."""
        if self.whole:
            return read(self.client, self.p.kind, self.p.servo, True)
        brief = bool(self.carry) and self.polls % self.FULL_N != 0
        got = read(self.client, self.p.kind, self.p.servo, False, brief=brief, abort=self._wake.is_set)
        if brief:
            return {**self.carry, **got}
        self.carry = dict(got)
        return got

    def _one_by_one(self, why: str):
        self.whole = False
        self.client.close()
        self.client.timeout = max(self.client.timeout, self.ONE_S)
        log.info("%s: %s: its values read one at a time from now on", self.label, why)

    def _fail(self, why: str):
        if why != self.error:
            log.warning("%s: %s does not answer: %s", self.label, self.p.host, why)
        self.error = why or "no answer"
        self.client.close()

    def _write(self, action):
        what = action[0]
        if what not in ("release", "engage"):
            return
        name = names("pll", self.p.servo)["manual"]
        err = ""
        try:
            took = self.client.set(name, what == "engage")
            if as_bool(took) is not (what == "engage"):
                err = f"{name} is {took}"
        except (OSError, SlsError) as e:
            err = str(e) or type(e).__name__
            self.client.close()
        if err:
            log.warning("%s: %s: %s", self.label, what, err)
        if self.wrote_cb is not None:
            try:
                self.wrote_cb(self, action, err)
            except Exception:
                log.exception("%s: wrote", self.label)
