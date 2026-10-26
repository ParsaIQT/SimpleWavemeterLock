"""Several PCs, each with its wavemeter: they find each other on the LAN, each shows the others' lasers as they show
them, and any of them can adjust the others' (where those allow it).

Each copy of this program (an instance) owns its wavemeter and the lasers it locks. It sends what its rows show
(view.py) to the others, and applies the adjustments they ask for through its engine, as if clicked there: nothing
is ever written to a wavemeter or a laser but by the PC that owns it. A PC with several wavemeters has an instance
for each (meters.py), all in one program: siblings, which ignore each other (the window shows them as its own), and
only the first of which asks the others for their rows.

UDP, two sockets per instance:
  the group port (PORT; shared by every instance on a PC, SO_REUSEADDR) hears the others' hellos, broadcast on the
    LAN as DLC pro discovery is (to every subnet's broadcast address), or sent to the addresses listed in its
    settings (another subnet);
  its own port (any free one) sends everything, and hears what is meant for this instance alone.
A datagram is b"WML1", a flags byte (1: compressed, 2: signed), with a key a 16-byte HMAC-SHA256 tag, then JSON
(zlib-compressed if flagged): a dict with k (kind), id (the sender, random each start) and g (its group). With a key
set only datagrams signed with it are taken, without one only unsigned ones; the group must be the same.

  hello  to each instance known every HELLO_S, by broadcast every BEACON_S: who it is (label: the wavemeter's name,
         host), whether it lets others adjust its lasers (ctl), its wavemeter's state (src), what it wants from the
         one it is sent to (want: fast, all of it; slow, every FULL_S only), and the others it knows (knows: one
         that hears of an instance it does not know says hello to it)
  state  what rows show: on a change, the keys that changed (at most every STATE_S), and all of it every FULL_S
         (and to one that has just asked for it); with the order of the rows
  fast   the new readings (f, dev) of the rows whose reading changed, at most every TICK, to those that want fast
  cmd    an adjustment of a row (setpoint, lock, unlock, stop, continue, setpoint_to_reading, recenter, rename,
         lock_params; Engine.remote), or a look at it (observe, observe_end, observed: its lock observed for ARTIQ,
         rpc.py), sent again every RETRY_S until answered (TRIES at most), done once
  ack    its answer: done, or why not (and what a look found: r)
  bye    it closes

What it costs (measured, 7 channels, 5 of them locked): about 2.5 kB/s to each PC that shows them, 0.3 kB/s to
one that has them folded away or is minimized; with no other PC about, a 170-byte hello every BEACON_S. Every
datagram stays under MAX bytes.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import logging
import math
import os
import select
import socket
import threading
import time
import zlib

from . import view
from .dlcpro import broadcast_addresses
from .wavemeter import meter_name

log = logging.getLogger(__name__)

PORT = 47474      # the group port
SIM_PORT = 47475  # ...of the demo (--sim): demos and real instances never mix
MAGIC = b"WML1"
TAG = 16          # bytes of HMAC-SHA256 kept
MAX = 1200        # bytes of a datagram at most (no IP fragments)
RAW_MAX = 1 << 20  # bytes of JSON a datagram may inflate to
COLORS = ("fg", "dim", "ok", "warn", "err", "accent")
STATES = ("off", "error", "on", "engaging", "recovering", "asking", "mapping", "ref", "ref-hold", "ref-moved")
FAST = ("f", "dev")  # what a fast datagram carries (and age: 0, it is new)
ACTIONS = ("setpoint", "lock", "unlock", "stop", "continue", "setpoint_to_reading", "recenter", "rename",
           "lock_params", "observe", "observe_end", "observed")
LOOKS = ("observe", "observe_end", "observed")  # actions that change nothing (a lock observed: observe.py): taken
#                                                 from other PCs even where adjusting is off


def secret(key: str) -> bytes:
    """The HMAC key of a key as typed ('': none)."""
    return hashlib.sha256(key.encode()).digest() if key else b""


def encode(msg: dict, key: bytes = b"") -> bytes:
    body = json.dumps(msg, separators=(",", ":"), ensure_ascii=False).encode()
    flags = 0
    if len(body) > 200:
        z = zlib.compress(body, 6)
        if len(z) < len(body):
            body, flags = z, 1
    if key:
        flags |= 2
        tag = hmac.new(key, MAGIC + bytes([flags]) + body, hashlib.sha256).digest()[:TAG]
        return MAGIC + bytes([flags]) + tag + body
    return MAGIC + bytes([flags]) + body


def decode(data: bytes, key: bytes = b""):
    """The message in a datagram, or None if it is not one of ours, not signed with our key (or signed while we have
    none), or broken."""
    if len(data) < 5 or data[:4] != MAGIC:
        return None
    flags, body = data[4], data[5:]
    if bool(flags & 2) != bool(key):
        return None
    if key:
        tag, body = body[:TAG], body[TAG:]
        if not hmac.compare_digest(tag, hmac.new(key, data[:5] + body, hashlib.sha256).digest()[:TAG]):
            return None
    try:
        if flags & 1:
            z = zlib.decompressobj()
            body = z.decompress(body, RAW_MAX)
            if z.unconsumed_tail:
                return None
        msg = json.loads(body)
    except (zlib.error, ValueError, RecursionError):
        return None
    return msg if isinstance(msg, dict) and isinstance(msg.get("id"), str) else None


def _ip(text: str) -> bool:
    try:
        socket.inet_aton(text)
        return text.count(".") == 3
    except OSError:
        return False


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _int(x):
    try:
        return int(x)
    except (TypeError, ValueError, OverflowError):
        return None


def clean(v) -> dict:
    """A row's view as sent by another PC, with only what a row paints, of the right types."""
    if not isinstance(v, dict):
        return {}
    out = {}
    for k in ("name", "bad", "info", "prompt", "notes"):
        if isinstance(v.get(k), str):
            out[k] = v[k][:2000]
    for k in ("bad_c", "info_c", "dev_c"):
        if k in v:
            out[k] = v[k] if v[k] in COLORS else "fg"
    if "state" in v:
        out["state"] = v["state"] if v["state"] in STATES else "off"
    for k in ("f", "age", "dev", "sp", "anchor", "corr"):
        if k in v:
            out[k] = v[k] if _num(v[k]) else None
    for k in ("ref_unit", "relock", "linked"):
        if k in v:
            out[k] = bool(v[k])
    if v.get("sls") in ("pdh", "pll"):
        out["sls"] = v["sls"]
    if "refch" in v:
        out["refch"] = v["refch"] if isinstance(v["refch"], int) and not isinstance(v["refch"], bool) else None
    if isinstance(v.get("lp"), dict):
        out["lp"] = {k: x for k, x in v["lp"].items() if isinstance(k, str) and _num(x)}
    return out


def compact(v: dict) -> dict:
    """A row's view, for sending: rounded to what is shown."""
    out = dict(v)
    out.pop("n", None)
    for k, digits in (("f", 7), ("dev", 1), ("age", 2), ("corr", 1)):
        x = out.get(k)
        if x is not None:
            out[k] = round(x, digits) if math.isfinite(x) else None
    return out


def address(text: str, port: int):
    """'host' or 'host:port' -> (host, port)."""
    text = text.strip()
    host, sep, p = text.rpartition(":")
    return (host, int(p)) if sep and host and p.isdigit() and 0 < int(p) < 65536 else (text, port)


class Peer:
    """Another instance, as heard: what its rows show, and what is going on with it."""

    def __init__(self, pid: str, addr, now: float, share: "Share"):
        self.share = share
        self.id = pid
        self.addr = addr          # (ip, port): its own port
        self.label = ""           # its wavemeter's name
        self.host = ""
        self.control = True       # it lets others adjust its lasers
        self.src = {}             # its wavemeter: ok, msg, rate (readings/s)
        self.rows = []            # its rows, in order
        self.views = {}           # n -> what row n shows (view.py, cleaned)
        self.got = {}             # n -> monotonic time its age was as said
        self.layout = 0           # bumped when its rows change
        self.heard = now          # monotonic: the last datagram from it
        self.found = now
        self.lost = False         # not heard from for LOST_S
        self.bye = False          # it closed...
        self.bye_wall = 0.0       # ...then (time.time())
        self.want = ""            # what it wants of this instance: fast | slow ('' nothing)...
        self.sub_until = 0.0      # ...until then
        self.needs_full = False   # all of it at once (it has just asked)
        self.wanted = "fast" if share.subscribe else ""  # what this instance wants of it (the window says)
        self.watched_until = 0.0  # its readings wanted at once (fast) until then, whatever the window wants (rpc.py)
        self.feedback = {}        # n -> (text, until): an adjustment of it that was not done, and why

    def age(self, now: float) -> float:
        return now - self.heard

    def want_now(self, now: float) -> str:
        """What this instance asks of it: what the window wants, or fast while ARTIQ's calls want its readings."""
        return "fast" if self.watched_until > now else self.wanted

    def status(self, now: float) -> str:
        """live | late | lost | closed"""
        if self.bye:
            return "closed"
        quiet = now - self.heard
        return "live" if quiet < self.share.LATE_S else "late" if quiet < self.share.LOST_S else "lost"


class Settings:
    """A sibling's settings (a PC's second wavemeter, and on): its name (label) its own, the network's those of the
    first's."""

    OWN = ("label",)

    def __init__(self, own: dict, shared: dict):
        self.own, self.shared = own, shared

    def _of(self, key) -> dict:
        return self.own if key in self.OWN else self.shared

    def get(self, key, default=None):
        return self._of(key).get(key, default)

    def __getitem__(self, key):
        return self._of(key)[key]

    def __setitem__(self, key, value):
        self._of(key)[key] = value

    def __contains__(self, key) -> bool:
        return key in self._of(key)

    def setdefault(self, key, default=None):
        return self._of(key).setdefault(key, default)

    def update(self, other=(), **kw):
        for key, value in dict(other, **kw).items():
            self[key] = value


class _Cmd:
    def __init__(self, pid: str, n: int, do: str, msg: dict, answer=None):
        self.pid, self.n, self.do, self.msg = pid, n, do, msg
        self.tries, self.next = 0, 0.0
        self.answer = answer  # (Event, list): its answer waited for (ask), put in the list; None: said in feedback

    def settle(self, ok: bool, why: str, r=None) -> bool:
        """Its answer (and what a look found) to the one waiting for it: False if no one is."""
        if self.answer is None:
            return False
        event, box = self.answer
        box.append((ok, why, r))
        event.set()
        return True


class Share:
    TICK = 0.1        # s: the pace of the loop; new readings go out at most this often
    STATE_S = 0.25    # s: changes of what rows show go out at most this often...
    FULL_S = 5.0      # s: ...and all of it this often
    HELLO_S = 2.0     # s between hellos to each instance known
    BEACON_S = 5.0    # s between broadcasts, and hellos to the listed addresses
    START = (0.0, 0.4, 1.2)  # s: broadcasts at the start (then every BEACON_S)
    SUB_S = 6.5       # s: what an instance wants lapses without a hello this long
    LATE_S = 3.5      # s without a datagram: late...
    LOST_S = 10.0     # ...lost
    CLOSED_S = 600.0  # s: one that closed is shown that long
    RETRY_S = 0.3     # s between tries of an adjustment...
    TRIES = 7         # ...that many at most
    TIP_S = 1.0       # s: tooltips computed again at most this often
    DONE_S = 120.0    # s: an adjustment done is remembered this long (one sent again is answered, not done again)
    CLOCK_S = 120.0   # s: with a key, an adjustment stamped further than this from the clock here is refused
    GOSSIP_S = 30.0   # s: an address heard of is said hello to at most this often
    RESOLVE_S = 60.0  # s: the listed addresses are looked up again this often

    def __init__(self, engine, settings: dict, model: str = "", siblings: set | None = None, subscribe: bool = True):
        """settings: the 'share' part of the settings file (kept, and changed by configure): on, label, group, key,
        peers (also talk to: 'host' or 'host:port'), control (let others adjust), port, lan (False: this PC only);
        a sibling's: Settings. model: what the wavemeter is, for its default name. siblings: the ids of the
        instances of this program (a PC's wavemeters), shared by them: they ignore each other. subscribe: ask the
        others for their rows (False: a sibling's, the first asks)."""
        self.engine = engine
        self.settings = settings
        self.model = model
        self.subscribe = subscribe
        self.id = os.urandom(6).hex()
        self.siblings = siblings if siblings is not None else set()
        self.siblings.add(self.id)
        self.host = socket.gethostname()
        self.lock = threading.RLock()
        self.peers = {}           # id -> Peer
        self.version = 0          # bumped when peers come or go, or change their names
        self.group_sock = self.own = None
        self.error = ""           # why it cannot share ('' if it can)
        self.port_note = ""       # the group port could not be had: others find this one only through those it finds
        self._stop_ev = threading.Event()
        self._thread = None
        self._err_t = -1e9
        self._reopen = True
        self._listed = []         # (ip, port) of the listed addresses, looked up
        self._broadcast = []      # (ip, port) broadcasts go to
        self._listed_at = -1e9
        self._tried = {}          # (ip, port) heard of -> monotonic: said hello to then
        self._pending = {}        # cid -> _Cmd, waiting for an answer
        self._cid = 0
        self._done = {}           # (sender, cid) -> (ok, why, monotonic)
        self._last_cid = {}       # (sender, n, do) -> the latest cid done: an older one is superseded
        self._next_tick = self._next_state = self._next_full = 0.0
        self._next_hello = self._next_beacon = 0.0
        self._starts = []
        self._sent_fast = {}      # n -> (f, dev) as last sent fast
        self._sent_slow = {}      # n -> the rest of its view, as last sent
        self._rows_sent = None
        self._tips = {}           # n -> (monotonic, notes)
        self.bytes_out = self.bytes_in = 0
        self.rate_out = self.rate_in = 0.0  # bytes/s
        self._rate_t, self._rate_b = time.monotonic(), (0, 0)
        self._key = b""

    # ------------------------------------------------------------- settings
    @property
    def on(self) -> bool:
        return bool(self.settings.get("on", True))

    @property
    def port(self) -> int:
        return int(self.settings.get("port") or PORT)

    @property
    def group(self) -> str:
        return str(self.settings.get("group") or "")

    @property
    def control(self) -> bool:
        return bool(self.settings.get("control", True))

    @property
    def default_label(self) -> str:
        """Its wavemeter's name, if none is set: what it is (its serial number too, if it says), and the PC's."""
        src = self.engine.source
        model = meter_name(self.model or getattr(src, "model", ""), getattr(src, "serial", None))
        return f"{model} · {self.host}" if model else self.host

    @property
    def label(self) -> str:
        """Its wavemeter's name."""
        return str(self.settings.get("label") or "").strip()[:80] or self.default_label

    @property
    def who(self) -> str:
        """This instance, as the others' logs and rows say it."""
        return self.label if self.host in self.label else f"{self.label} ({self.host})"

    def configure(self):
        """Its settings changed: sockets, key and all taken anew."""
        with self.lock:
            self._reopen = True
            self._listed_at = -1e9
            self.version += 1

    # ---------------------------------------------------------------- for the GUI
    def snapshot(self) -> list:
        """The other instances, by name."""
        with self.lock:
            return sorted(self.peers.values(), key=lambda p: (p.label.lower(), p.host, p.id))

    def want(self, pid: str, want: str):
        """What this instance wants of that one: fast (its rows shown) or slow (folded away, or minimized)."""
        with self.lock:
            p = self.peers.get(pid)
            if p is None or p.wanted == want:
                return
            p.wanted = want
        self._hello(p)  # at once: no waiting for the next one

    def forget(self, pid: str):
        with self.lock:
            if self.peers.pop(pid, None) is not None:
                self.version += 1

    def command(self, pid: str, n: int, do: str, *, by: str = "", answer=None, **args) -> bool:
        """Ask that instance to adjust its row n (an action of ACTIONS, with its arguments); a refusal, or no answer,
        is said in Peer.feedback (or to answer: see ask). by: who asks, as its row and log say ('': this instance).
        False if it was not sent (that instance gone, or no network)."""
        now = time.monotonic()
        with self.lock:
            p = self.peers.get(pid)
            if p is None or self.own is None:
                return False
            if do == "setpoint":  # the latest wins: one still unanswered is not sent again
                for cid, c in list(self._pending.items()):
                    if c.pid == pid and c.n == n and c.do == do:
                        del self._pending[cid]
                        c.settle(False, "another setpoint was sent after it")
            self._cid += 1
            c = _Cmd(pid, n, do, {"k": "cmd", "c": self._cid, "ch": n, "do": do, "a": args, "by": by or self.who,
                                  "t": round(time.time(), 3)}, answer)
            self._pending[self._cid] = c
            if answer is None:
                p.feedback.pop(n, None)
            self._send_cmd(c, p, now)
            return True

    def ask(self, pid: str, n: int, do: str, *, by: str = "", **args) -> tuple:
        """command(), its answer waited for (RETRY_S × TRIES at most): (done, why not, what a look found)."""
        answer = (threading.Event(), [])
        if not self.command(pid, n, do, by=by, answer=answer, **args):
            with self.lock:
                gone = pid not in self.peers
            return False, "that PC is gone" if gone else (self.error or "this PC is not on the network"), None
        answer[0].wait(self.RETRY_S * self.TRIES + 1.0)
        return answer[1][0] if answer[1] else (False, "no answer", None)

    def watch(self, pid: str, seconds: float):
        """That instance's readings wanted at once (fast) for so long, whatever the window wants (ARTIQ's calls
        that read them: rpc.py)."""
        now = time.monotonic()
        with self.lock:
            p = self.peers.get(pid)
            if p is None:
                return
            was = p.want_now(now)
            p.watched_until = max(p.watched_until, now + seconds)
        if was != "fast":
            self._hello(p)  # at once

    # ------------------------------------------------------------------ loop
    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True, name="share")
        self._thread.start()

    def stop(self):
        """Close (saying so to the others), at once."""
        self._stop_ev.set()  # (its id stays among its siblings': its last datagrams, ignored too)

    def join(self, timeout=None):
        if self._thread is not None:
            self._thread.join(timeout)

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run(self):
        try:
            while not self._stop_ev.is_set():
                if self._reopen:
                    self._open()
                if self.own is None:
                    self._stop_ev.wait(0.5)
                    continue
                now = time.monotonic()
                socks = [s for s in (self.group_sock, self.own) if s is not None]
                try:
                    ready = select.select(socks, [], [], max(0.0, self._next_tick - now))[0]
                except (OSError, ValueError):
                    ready = []
                    self._stop_ev.wait(0.05)
                for s in ready:
                    self._drain(s)
                now = time.monotonic()
                if now >= self._next_tick:
                    self._next_tick = now + self.TICK
                    try:
                        self._tick(now)
                    except Exception:
                        if now - self._err_t > 60.0:  # (one that keeps happening: said once a minute)
                            self._err_t = now
                            log.exception("share")
        finally:
            self._close(bye=True)

    def _open(self):
        self._close(bye=True)
        with self.lock:
            self._reopen = False
            self._key = secret(str(self.settings.get("key") or ""))
            self.error = self.port_note = ""
            self._sent_fast, self._sent_slow, self._rows_sent = {}, {}, None
            if not self.on:
                if self.peers:
                    self.peers.clear()
                    self.version += 1
                return
            try:
                own = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                own.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                own.bind(("", 0))
                self._quiet(own)
            except OSError as e:
                self.error = f"no network: {e}"
                return
            group = None
            try:
                group = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                group.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                group.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                group.bind(("", self.port))
                self._quiet(group)
            except OSError as e:
                if group is not None:
                    group.close()
                group = None
                self.port_note = f"port {self.port} taken ({e}): the others find this PC only when it finds them"
            self.own, self.group_sock = own, group
            now = time.monotonic()
            self._starts = [now + s for s in self.START]
            self._next_beacon = self._starts.pop(0)
            self._next_hello = now + self.HELLO_S

    @staticmethod
    def _quiet(s):
        s.setblocking(False)
        if hasattr(socket, "SIO_UDP_CONNRESET"):  # Windows: no error on receiving after a send to nobody
            try:
                s.ioctl(socket.SIO_UDP_CONNRESET, False)
            except (OSError, ValueError):
                pass

    def _close(self, bye: bool):
        with self.lock:
            own, group = self.own, self.group_sock
            self.own = self.group_sock = None
            peers = [p for p in self.peers.values() if not p.bye]
        if own is not None:
            if bye:
                data = encode({"k": "bye", "id": self.id, "g": self.group}, self._key)
                for p in peers:
                    try:
                        own.sendto(data, p.addr)
                    except OSError:
                        pass
            own.close()
        if group is not None:
            group.close()

    # ---------------------------------------------------------------- sending
    def _send(self, msg: dict, addr) -> bool:
        msg["id"], msg["g"] = self.id, self.group
        return self._send_raw(encode(msg, self._key), addr)

    def _send_raw(self, data: bytes, addr) -> bool:
        s = self.own
        if s is None:
            return False
        try:
            s.sendto(data, addr)
        except OSError:  # (a broadcast address with no route, say)
            return False
        self.bytes_out += len(data)
        return True

    def _hello(self, p: Peer | None = None, addr=None):
        msg = {"k": "hello", "v": 1, "label": self.label, "host": self.host, "ctl": self.control,
               "src": self._src()}
        if p is not None:
            msg["want"] = p.want_now(time.monotonic())
            with self.lock:
                msg["knows"] = [f"{o.addr[0]}:{o.addr[1]}" for o in self.peers.values()
                                if o is not p and not o.bye and not o.lost][:16]
            addr = p.addr
        self._send(msg, addr)

    def _src(self) -> dict:
        """Its wavemeter: read as it should be (ok), or why not; readings a second; what it is read through; no
        wavemeter at all (viewer: --view)."""
        e = self.engine
        return {"ok": bool(e.source_ok), "msg": str(e.source_msg or "")[:200],
                "rate": round(e.rate) if _num(e.rate) else 0, "what": str(getattr(e.source, "label", ""))[:80],
                "viewer": bool(getattr(e.source, "viewer", False))}

    def _send_cmd(self, c: _Cmd, p: Peer, now: float):
        c.tries += 1
        c.next = now + self.RETRY_S
        self._send(dict(c.msg), p.addr)

    # ---------------------------------------------------------------- receiving
    def _drain(self, s):
        for _ in range(500):
            try:
                data, addr = s.recvfrom(65536)
            except BlockingIOError:
                return
            except OSError:  # (Windows: a reset from an earlier send to nobody; or the socket closed)
                return
            self.bytes_in += len(data)
            msg = decode(data, self._key)
            if msg is None or msg.get("id") in self.siblings or msg.get("g", "") != self.group:
                continue  # (its own, or a sibling's: shown as its own by the window)
            try:
                self._handle(msg, addr, time.monotonic())
            except Exception:  # (a datagram that is not what it should be: never the end of this thread)
                continue

    def _handle(self, msg: dict, addr, now: float):
        k = msg.get("k")
        if k == "hello":
            self._on_hello(msg, addr, now)
            return
        if k == "cmd":
            self._on_cmd(msg, addr, now)
            return
        with self.lock:
            p = self.peers.get(msg["id"])
            if p is None:
                return  # (its hello comes)
            p.heard, p.addr = now, addr
            if k == "state":
                self._on_state(p, msg, now)
            elif k == "fast":
                self._on_fast(p, msg, now)
            elif k == "ack":
                self._on_ack(p, msg, now)
            elif k == "bye":
                p.bye, p.bye_wall, p.sub_until = True, time.time(), 0.0
                log.info("%s closed", p.label or p.host)

    def _on_hello(self, msg: dict, addr, now: float):
        pid = msg["id"]
        label, host = str(msg.get("label", ""))[:80], str(msg.get("host", ""))[:80]
        with self.lock:
            p = self.peers.get(pid)
            new = p is None
            if new:  # a restarted instance takes the place of its old self
                for old in [o for o in self.peers.values() if o.label == label and o.host == host and (
                        o.bye or now - o.heard > self.LATE_S)]:
                    del self.peers[old.id]
                p = self.peers[pid] = Peer(pid, addr, now, self)
                self.version += 1
            if (p.label, p.host) != (label, host):
                self.version += 1
            p.label, p.host, p.addr, p.heard = label, host, addr, now
            p.control = bool(msg.get("ctl", True))
            if isinstance(msg.get("src"), dict):
                p.src = msg["src"]
            if p.bye:
                p.bye = False
            want = msg.get("want")
            if want in ("fast", "slow"):
                if p.sub_until <= now or p.want != want:
                    p.needs_full = True
                p.want, p.sub_until = want, now + self.SUB_S
            knows = [address(a, self.port) for a in msg.get("knows", []) if isinstance(a, str)][:16]
            known = {o.addr for o in self.peers.values()}
        if new:
            log.info("found %s (%s, %s)", label or host, host, addr[0])
            self._hello(p)  # at once: so it learns of this one too
        for a in knows:  # another one it knows: say hello to it
            if a not in known and now - self._tried.get(a, -1e9) > self.GOSSIP_S:
                self._tried[a] = now
                self._hello(addr=a)

    def _on_state(self, p: Peer, msg: dict, now: float):
        rows = [n for n in (_int(x) for x in msg.get("rows", [])) if n is not None]
        if rows != p.rows:
            p.rows = rows
            p.views = {n: v for n, v in p.views.items() if n in rows}
            p.layout += 1
        full = bool(msg.get("full"))
        views = dict(p.views)
        for k, v in (msg.get("c") or {}).items():
            n = _int(k)
            if n not in rows:
                continue
            v = clean(v)
            views[n] = v if full else {**views.get(n, {}), **v}
            if "age" in v:
                p.got[n] = now
        p.views = views
        if isinstance(msg.get("src"), dict):
            p.src = msg["src"]

    def _on_fast(self, p: Peer, msg: dict, now: float):
        views = dict(p.views)
        for entry in msg.get("r", []):
            n, f, dev = _int(entry[0]), entry[1], entry[2]
            if n in views:
                views[n] = {**views[n], "f": f if _num(f) else None, "dev": dev if _num(dev) else None, "age": 0.0}
                p.got[n] = now
        p.views = views

    def _on_ack(self, p: Peer, msg: dict, now: float):
        c = self._pending.pop(_int(msg.get("c")), None)
        if c is None or c.pid != p.id:
            return
        why = str(msg.get("why") or "")
        if not c.settle(bool(msg.get("ok")), why or ("" if msg.get("ok") else "refused"), msg.get("r")) and not \
                msg.get("ok"):
            p.feedback[c.n] = (f"not done on {p.label}: {why or 'refused'}", now + 10.0)

    def _on_cmd(self, msg: dict, addr, now: float):
        sender, cid, n, do = msg["id"], _int(msg.get("c")), _int(msg.get("ch")), str(msg.get("do", ""))
        who = str(msg.get("by", "") or addr[0])[:120]
        with self.lock:
            p = self.peers.get(sender)
            if p is not None:
                p.heard = now
            done = self._done.get((sender, cid))
        if done is None:
            ok, why, r = self._do(sender, cid, n, do, msg.get("a") or {}, msg.get("t"), who)
            with self.lock:
                self._done[(sender, cid)] = (ok, why, now, r)
        else:
            ok, why, r = done[0], done[1], done[3]  # (sent again: answered again, not done again)
        ack = {"k": "ack", "c": cid, "ok": ok, "why": why}
        if r is not None:
            ack["r"] = r
        self._send(ack, addr)

    def _do(self, sender: str, cid, n, do: str, args: dict, stamp, who: str) -> tuple:
        """(done, why not, what a look found)."""
        if not self.control and do not in LOOKS:
            return False, "adjusting from other PCs is off there", None
        if do not in ACTIONS or n is None or cid is None or not isinstance(args, dict):
            return False, f"cannot {do!r} from another PC", None
        if self._key and not (_num(stamp) and abs(time.time() - stamp) < self.CLOCK_S):
            return False, "its clock and this PC's differ by over 2 min (or an old message replayed)", None
        last = self._last_cid.get((sender, n, do))
        if do == "setpoint" and last is not None and cid < last:
            return True, "superseded", None  # (an older one, overtaken on the way)
        self._last_cid[(sender, n, do)] = cid
        try:
            ok, why, *found = self.engine.remote(n, do, args, who)
        except Exception as e:  # (a bad argument, say: refused, never the end of this thread)
            log.exception("channel %s: %s from %s", n, do, who)
            ok, why, found = False, f"cannot {do}: {e}", []
        if ok and why != "unchanged" and do not in LOOKS:
            log.info("channel %d: %s", n, self.engine.channels[n].remote_note or f"{do} from {who}")
        elif not ok:
            log.info("channel %s: %s from %s refused: %s", n, do, who, why)
        return ok, "" if why == "unchanged" else why, found[0] if found else None

    # ------------------------------------------------------------- the tick
    def _tick(self, now: float):
        self._hellos(now)
        self._retry(now)
        self._watch(now)
        self._publish(now)
        if now - self._rate_t >= 1.0:
            b = (self.bytes_out, self.bytes_in)
            dt = now - self._rate_t
            self.rate_out = (b[0] - self._rate_b[0]) / dt
            self.rate_in = (b[1] - self._rate_b[1]) / dt
            self._rate_t, self._rate_b = now, b

    def _targets(self, now: float) -> list:
        """Where broadcasts go: every subnet's broadcast address (this PC only, without lan), and the listed
        addresses."""
        if now - self._listed_at > self.RESOLVE_S:
            self._listed_at = now
            lan = self.settings.get("lan", True)
            self._broadcast = [(b, self.port) for b in broadcast_addresses()] if lan else [("127.0.0.1", self.port)]
            listed = [address(a, self.port) for a in self.settings.get("peers") or []
                      if isinstance(a, str) and a.strip()]
            self._listed = [a for a in listed if _ip(a[0])]
            names = [a for a in listed if not _ip(a[0])]
            if names:  # (a name may take a while to look up: not in this thread)
                threading.Thread(target=self._resolve, args=(names,), daemon=True, name="share lookup").start()
        return self._broadcast + self._listed

    def _resolve(self, names):
        found = []
        for host, port in names:
            try:
                found.append((socket.gethostbyname(host), port))
            except OSError as e:
                log.warning("cannot find %s: %s", host, e)
        self._listed = self._listed + found

    def _hellos(self, now: float):
        if now >= self._next_beacon:
            self._next_beacon = self._starts.pop(0) if self._starts else now + self.BEACON_S
            for addr in self._targets(now):
                self._hello(addr=addr)
        if now >= self._next_hello:
            self._next_hello = now + self.HELLO_S
            with self.lock:
                peers = [p for p in self.peers.values() if not p.bye]
            for p in peers:
                self._hello(p)

    def _retry(self, now: float):
        with self.lock:
            for cid, c in list(self._pending.items()):
                if c.next > now:
                    continue
                p = self.peers.get(c.pid)
                if p is None:
                    del self._pending[cid]
                    c.settle(False, "that PC is gone")
                elif c.tries >= self.TRIES:
                    del self._pending[cid]
                    if not c.settle(False, f"no answer from {p.label}"):
                        p.feedback[c.n] = (f"not done: no answer from {p.label}", now + 10.0)
                else:
                    self._send_cmd(c, p, now)
            for key, (_, _, t, _) in list(self._done.items()):
                if now - t > self.DONE_S:
                    del self._done[key]
            for a, t in list(self._tried.items()):
                if now - t > self.GOSSIP_S:
                    del self._tried[a]

    def _watch(self, now: float):
        """Instances lost (said in the log once), back again, closed long ago (forgotten)."""
        with self.lock:
            for p in list(self.peers.values()):
                lost = now - p.heard >= self.LOST_S and not p.bye
                if lost != p.lost:
                    p.lost = lost
                    if lost:
                        log.warning("%s (%s): not heard from for %.0f s", p.label or p.host, p.host, now - p.heard)
                    else:
                        log.warning("%s (%s): heard from again", p.label or p.host, p.host)
                if p.bye and time.time() - p.bye_wall > self.CLOSED_S:
                    del self.peers[p.id]
                    self.version += 1

    # -------------------------------------------------------------- publishing
    def _views(self, now: float):
        """The rows shown here and what each shows (as sent: rounded, with what the others' menus need)."""
        e = self.engine
        with e.lock:
            rows = e.rows()
            ref = e.reference
            views = {}
            for n in rows:
                ch = e.channels[n]
                v = view.row(e, n, now)
                t, notes = self._tips.get(n, (-1e9, ""))
                if now - t > self.TIP_S:
                    notes = view.notes(e, n)
                    self._tips[n] = (now, notes)
                v["notes"] = notes
                referenced = ref is not None and ref.channel != n and ref.active and ch.raw > 0 and ch.freq > 0
                v["refch"] = ref.channel if referenced else None
                v["corr"] = (ch.freq - ch.raw) * 1e6 if referenced else None
                v["linked"] = bool(ch.device)
                v["lp"] = ch.lock.p.to_dict()
                views[n] = compact(v)
        return rows, views

    def _publish(self, now: float):
        with self.lock:
            subs = [p for p in self.peers.values() if p.sub_until > now and not p.bye and p.want]
        if not subs:
            self._rows_sent = None
            return
        fast = [p for p in subs if p.want == "fast"]
        rows, views = self._views(now)
        slow = {n: {k: x for k, x in v.items() if k not in FAST and k != "age"} for n, v in views.items()}
        if now >= self._next_full:  # all of it, to all
            self._next_full = now + self.FULL_S
            self._send_state(subs, rows, {n: views[n] for n in rows}, full=True)
            for p in subs:
                p.needs_full = False
            self._sent_slow, self._rows_sent = slow, rows
            self._sent_fast = {n: tuple(views[n].get(k) for k in FAST) for n in rows}
            return
        newbies = [p for p in subs if p.needs_full]
        if newbies:  # all of it, to those that have just asked
            self._send_state(newbies, rows, {n: views[n] for n in rows}, full=True)
            for p in newbies:
                p.needs_full = False
        if not fast:
            return
        if now >= self._next_state:  # what changed, to those that want it all (a new row: all of it)
            changes = {}
            for n in rows:
                old = self._sent_slow.get(n)
                if old is None:
                    changes[n] = views[n]
                    continue
                diff = {k: x for k, x in slow[n].items() if old.get(k, _MISSING) != x}
                if diff:
                    changes[n] = diff
            if changes or rows != self._rows_sent:
                self._next_state = now + self.STATE_S
                self._send_state(fast, rows, changes, full=False)
                for n in changes:
                    self._sent_slow[n] = slow[n]
                self._rows_sent = rows
        moved = [n for n in rows if n in self._sent_slow
                 and tuple(views[n].get(k) for k in FAST) != self._sent_fast.get(n)]
        if moved:  # new readings
            data = encode({"k": "fast", "id": self.id, "g": self.group,
                           "r": [[n, views[n].get("f"), views[n].get("dev")] for n in moved]}, self._key)
            for p in fast:
                self._send_raw(data, p.addr)
            for n in moved:
                self._sent_fast[n] = tuple(views[n].get(k) for k in FAST)

    def _send_state(self, peers, rows, items: dict, full: bool):
        """items: n -> what to send of its view; in as few datagrams as fit in MAX bytes each."""
        base = {"k": "state", "id": self.id, "g": self.group, "rows": rows, "full": full}
        if full:
            base["src"] = self._src()
        chunks, chunk, data = [], {}, None
        for n, v in items.items():
            trial = {**chunk, str(n): v}
            enc = encode({**base, "c": trial}, self._key)
            if len(enc) > MAX and chunk:
                chunks.append(data)
                chunk = {str(n): v}
                data = encode({**base, "c": chunk}, self._key)
            else:
                chunk, data = trial, enc
        if data is None:  # (no rows, or nothing changed but their order)
            data = encode({**base, "c": {}}, self._key)
        chunks.append(data)
        for p in peers:
            for d in chunks:
                self._send_raw(d, p.addr)


_MISSING = object()
