"""Lock dropouts: a laser its DLC pro locks to a cavity losing the lock, if only for a few milliseconds
(enough to spoil a measurement, or to lose an ion), found and logged.

The engine's polls, twice a second, would see few of them. A monitor per laser, on a connection of its
own, watches the cavity transmission (the input its lock detection watches, as a rule Fine In 1):

  recorder  the DLC pro's recorder (laserN:recorder) records it every millisecond (dt), and the monitor
            reads what it has recorded every READ_S, a few thousand samples at a time (recorder:data:get-data);
            when a recording is full, the next starts at once. (If it will not be read while it records, each
            recording is read once full, before the next: a gap of that long.) How often it really samples is
            measured, and what it records is checked against the input's present value. Its settings are put
            back when the monitor stops (when the program closes, say).
  polling   without a recorder, or with one that does not record as expected (the console says why), the
            input's present value is read POLL_HZ times a second: a dropout shorter than the interval between
            reads may be missed.
A DLC pro has to do all this besides its own work and its other clients' (TOPAS's displays, say): so it is
asked little, and seldom (TOPAS said "acquisition busy" now and then while the recorder was read back to
back, every quarter of a millisecond). And its data acquisition is shared: while the recorder records, the
DLC pro now and then refuses a scope trace, or its lock going on, as busy (decop -1032 "data acquisition
busy"), which a recovery needs. So nothing is recorded while the lock should not hold (off, being
recovered, just relocked, not reporting locked): the recorder is stopped then, at once when the engine
starts a recovery (nudge), and the recovery's operations are tried again when refused as busy
(modehop.perform).

A dropout is the transmission below its limit for two samples at least, MIN_S at the least (Detector),
while the lock should hold (status(): its cavity lock on, no recovery running, and settled). Each is
reported (report), and once an hour the monitor's own figures (summary): how it watches, and how much of
the time.
"""
from __future__ import annotations

import logging
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass

from .dlcpro import IO_VALUE, NONE, DecopError, DLCPro, float_arrays, signal_name

log = logging.getLogger(__name__)


@dataclass
class Dropout:
    t: float           # time.time() when it began
    duration: float    # s
    low: float         # V: the lowest the transmission went
    limit: float       # V: below this, it counted
    method: str        # 'recorder' or 'polling'
    resolution: float  # s between samples


class Detector:
    """Runs of samples below a limit: dropouts, if they span MIN_S and have min_n samples below it. A run
    ends with two samples in a row above the limit (one in the middle of a dropout does not split it). Fed
    evenly spaced samples in order, numbered (i0 the first of a batch; one at a time, polling); a run open at
    the end of a batch carries over to the next, unless gap() says the data stops for a while."""

    def __init__(self, min_s: float, min_n: int = 2):
        self.min_s, self.min_n = min_s, min_n
        self._run = None   # [first, last, lowest, samples below] of the run going on
        self._high = 0     # samples above the limit since its last one below

    def feed(self, i0: int, dt: float, values, limit: float) -> list:
        """values, samples i0, i0 + 1, ..., dt apart: the dropouts that ended in them, [(first sample, samples
        spanned, lowest)]."""
        out = []
        for k, v in enumerate(values):
            if v < limit:
                r = self._run
                if r is None:
                    self._run = [i0 + k, i0 + k, v, 1]
                else:
                    r[1], r[2], r[3] = i0 + k, min(r[2], v), r[3] + 1
                self._high = 0
            elif self._run is not None:
                self._high += 1
                if self._high >= 2:
                    out += self._close(dt)
        return out

    def gap(self, dt: float) -> list:
        """The data stops here for a while (or is not to be looked at): a run going on ends here."""
        return self._close(dt)

    def going(self):
        """The first sample of the run going on (a dropout, perhaps, still to end), None if none."""
        return self._run[0] if self._run is not None else None

    def _close(self, dt: float) -> list:
        r, self._run, self._high = self._run, None, 0
        if r is None:
            return []
        span = r[1] - r[0] + 1
        return [(r[0], span, r[2])] if r[3] >= self.min_n and span * dt >= self.min_s - 1e-12 else []


class NoRecorder(Exception):
    """The recorder is not there, or does not record as expected: why."""


class Monitor(threading.Thread):
    """Watches one laser's cavity transmission for dropouts (see the module), on a connection of its own.
    Called from its thread: status() -> (should hold, limit V or None, signal id or None), before each batch
    of samples; report(Dropout); summary(text) once an hour."""

    DT = 0.001           # s between the recorder's samples asked for, by default (1 kHz)
    RECORD_S = 30.0      # s a recording lasts; the next starts when it is full
    READ_S = 2.0         # s between reads of what it has recorded
    STALL_S = 3.0        # s without a new sample: it stopped short of full
    CHUNK = 4000         # samples read at a time, at most
    START_S = 3.0        # s for a recording to start
    MIN_S = 0.0005       # s: shorter, not a dropout (with min_n samples at least)
    POLL_HZ = 20.0       # polling: at most this many reads a second
    SUMMARY_S = 3600.0
    RETRY_S = 10.0       # s before trying again after the controller stopped answering
    SAVED = ("inputs:channel1:signal", "inputs:channel1:low-pass-filter:enabled", "inputs:channel2:signal",
             "recording-time", "sample-count-set", "enabled")

    def __init__(self, host: str, port: int, laser: int, status, report, summary, name: str = "",
                 dt: float = DT):
        """dt: s between the recorder's samples."""
        super().__init__(daemon=True, name=f"dropouts {name or host}")
        self.client = DLCPro(host, port, timeout=2.0)
        self.dt = dt
        self.laser, self.label = laser, name or f"laser{laser}"
        self.status, self.report, self.summary = status, report, summary
        self._stop_ev = threading.Event()
        self._wake = threading.Event()  # set by nudge() (and stop()): look at status() now
        self.method = ""        # 'recorder' or 'polling', once it watches
        self.why_not = ""       # why not with the recorder
        self.resolution = 0.0   # s between samples
        self.covered = 0.0      # s of samples looked at while the lock should hold
        self.held = 0.0         # s the lock should have held
        self.seen = 0.0         # time.time(): its dropouts until then are reported (or were not to be looked for)...
        self.open_since = None  # ...but one that began then (time.time()) and goes on (None: none goes on)
        self.saved = {}         # the recorder's settings as found (put back at the end)
        self._mono = time.monotonic()
        self._summary_at = self._mono + self.SUMMARY_S
        self._said = ""

    def stop(self):
        self._stop_ev.set()
        self._wake.set()

    def nudge(self):
        """Look at status() now, not at the next read: a recovery is beginning, say (the recorder stops for it)."""
        self._wake.set()

    def _sleep(self, seconds: float) -> bool:
        """Wait that long, or until nudged or stopped; True if stopping."""
        self._wake.wait(seconds)
        self._wake.clear()
        return self._stop_ev.is_set()

    def _idle(self) -> bool:
        """While its lock should not hold (off, being recovered, just relocked, not reporting locked): nothing
        recorded, the recorder stopped, its data acquisition left to the recovery and to your displays. True once it
        should hold again, False if stopping."""
        stopped = False
        while not self._stop_ev.is_set():
            self._tick()
            if self.status()[0]:
                return True
            self.seen, self.open_since = time.time(), None  # (nothing to look for meanwhile)
            if not stopped:
                stopped = True
                try:
                    self.client.set(self._p("enabled"), False)
                except DecopError:
                    pass
            self._sleep(0.5)
        return False

    # --------------------------------------------------------------- running
    def run(self):
        while not self._stop_ev.is_set():
            try:
                self._watch()
            except (OSError, DecopError) as e:  # it stopped answering, or refused something
                self._say(f"the controller: {e}; again in {self.RETRY_S:g} s", warn=True)
            except Exception:  # (a bug: say so, and go on)
                log.exception("%s: dropout monitor", self.label)
            else:
                continue
            self.client.close()
            self._stop_ev.wait(self.RETRY_S)
        self._restore()
        self.client.close()

    def _watch(self):
        sig = None
        while sig is None and not self._stop_ev.is_set():  # (until the engine knows its transmission input)
            sig = self.status()[2]
            if sig is None:
                self._stop_ev.wait(1.0)
        if self._stop_ev.is_set():
            return
        if self.method != "polling":
            try:
                self._record(sig)
                return
            except NoRecorder as e:
                self.why_not = str(e)
                self._restore()
                self.method = ""
        self._poll(sig)

    def _say(self, text: str, warn: bool = False):
        if text != self._said:
            self._said = text
            (log.warning if warn else log.info)("%s: dropout monitor: %s", self.label, text)

    def _account(self, n: int, dt: float, detector: Detector):
        """n samples dt apart just read: looked at if the lock should hold (and its limit is known), else a
        gap; returns the limit (None: not looked at)."""
        hold, limit, _ = self.status()
        if not hold or limit is None:
            detector.gap(dt)
            return None
        self.covered += n * dt
        return limit

    def _tick(self):
        """Time passing: how long the lock should have held, and the hourly summary."""
        now = time.monotonic()
        if self.status()[0]:
            self.held += now - self._mono
        self._mono = now
        if now >= self._summary_at:
            self._summary_at = now + self.SUMMARY_S
            self.summary(self.figures())
            self.covered = self.held = 0.0

    def figures(self) -> str:
        """How it watches, and how much of the time the lock should hold it saw."""
        if self.method == "recorder":
            share = (f", {100 * min(self.covered / self.held, 1.0):.1f} % of the time it should hold" if self.held
                     else "")
            return f"the DLC pro's recorder, {self.resolution * 1e3:.2f} ms samples{share}"
        if self.method == "polling":
            rate = 1.0 / self.resolution if self.resolution else 0.0
            return (f"polling its present value, {rate:.0f} reads a second (shorter than {self.resolution * 1e3:.0f} "
                    f"ms, a dropout may be missed)" + (f"; not the recorder: {self.why_not}" if self.why_not else ""))
        return "not watching yet"

    # ------------------------------------------------------------- recorder
    def _p(self, name: str) -> str:
        return f"laser{self.laser}:recorder:{name}"

    def _get(self, name: str):
        try:
            return self.client.get_value(self._p(name))
        except DecopError:
            return None

    def _record(self, sig: int):
        c, p = self.client, self._p
        found = {n: self._get(n) for n in self.SAVED}
        if found["enabled"] is None or found["inputs:channel1:signal"] is None or found["sample-count-set"] is None:
            raise NoRecorder("this controller has no recorder (laser{}:recorder)".format(self.laser))
        if not self.saved:
            self.saved = {n: v for n, v in found.items() if v is not None}
        limit = self._get("sample-count-limit")
        n = int(self.RECORD_S / self.dt)
        if isinstance(limit, int) and limit > 0:
            n = min(n, limit)
        try:
            c.set(p("enabled"), False)
            c.set(p("inputs:channel1:signal"), int(sig))
            if found["inputs:channel1:low-pass-filter:enabled"] is not None:
                c.set(p("inputs:channel1:low-pass-filter:enabled"), False)
            c.set(p("sample-count-set"), n)
            c.set(p("recording-time"), n * self.dt)
        except DecopError as e:
            raise NoRecorder(f"it would not take its settings ({e})") from None
        if found["inputs:channel2:signal"] not in (None, NONE):
            try:  # (its samples would come along too: twice as much to send)
                c.set(p("inputs:channel2:signal"), NONE)
            except DecopError:
                pass
        self.resolution = self._asked_dt() or self.dt
        self.chunk = self.CHUNK
        self.when_full = False  # read a recording only once it is full (it would not be read while it records)
        detector = Detector(self.MIN_S)
        first, empty = True, 0
        while self._idle():
            got = self._one(n, sig, detector, first)
            detector.gap(self.resolution)
            if got is None:  # stopped short: the lock should not hold now
                continue
            first = False
            empty = empty + 1 if got == 0 else 0
            if empty >= 3:
                raise NoRecorder("its recordings stopped coming")

    def _one(self, n: int, sig: int, detector: Detector, first: bool):
        """One recording, read as it comes in, every READ_S (and once full): the number of samples read; None if it
        was stopped short, the lock no longer to hold (the recorder stopped at once)."""
        c, p = self.client, self._p
        c.set(p("enabled"), False)
        t_wall, t0 = time.time(), time.monotonic()
        c.set(p("enabled"), True)
        count = self._count()
        while count <= 0 and time.monotonic() - t0 < self.START_S and not self._stop_ev.is_set():
            self._stop_ev.wait(0.05)
            count = self._count()
        if count <= 0:
            if first:
                raise NoRecorder(f"it does not record (state “{self._get('state-txt') or '?'}”, trigger mode "
                                 f"{self._get('trigger-mode')}, recording mode {self._get('recording-mode')})")
            return 0
        size = self._get("sample-count")
        size = size if isinstance(size, int) and size > 0 else n
        read, t_full, wait = 0, None, 0.0
        last, grew = count, time.monotonic()  # its count of samples, and when it last grew
        while read < size and not self._stop_ev.is_set():
            if wait > 0 and self._sleep(wait):
                break
            if not self.status()[0]:  # (a recovery begins, say: its data acquisition is wanted)
                c.set(p("enabled"), False)
                self.seen, self.open_since = time.time(), None
                return None
            count = min(self._count(), size)
            now = time.monotonic()  # (the samples it had then)
            if count >= size and t_full is None:
                t_full = now  # (when it was full, give or take a poll)
            self._tick()
            if count < read:  # it started afresh by itself (someone else, say)
                return read
            if count != last:
                last, grew = count, now
            elif count < size and now - grew > self.STALL_S:
                return read  # it stopped short of full
            if count > read and not (self.when_full and count < size):
                while read < count and not self._stop_ev.is_set():
                    try:
                        values = self._samples(read, min(count - read, self.CHUNK))
                    except NoRecorder:
                        if self.when_full or count >= size:
                            raise
                        self.when_full, self.chunk = True, self.CHUNK  # (refused while recording: once it is full)
                        self._say("the recorder's data cannot be read while it records: each recording is read "
                                  "once full, before the next (a gap of that long)", warn=True)
                        break
                    if not values:
                        raise NoRecorder("its data had no samples")
                    dt = self._dt(t0, count, size, t_full, now)
                    if first:
                        self._check(values, sig, dt)
                        first = False
                    limit = self._account(len(values), dt, detector)
                    if limit is not None:
                        for i0, span, low in detector.feed(read, dt, values, limit):
                            self.report(Dropout(t_wall + i0 * dt, span * dt, low, limit, "recorder", dt))
                    read += len(values)
                    run = detector.going()
                    self.seen, self.open_since = t_wall + read * dt, t_wall + run * dt if run is not None else None
            # the next look: READ_S on, or as it should be full, whichever comes first
            end = t0 + size * (self.resolution or self.dt)
            wait = max(min(self.READ_S, end + 0.02 - time.monotonic()), 0.2)
        return read

    def _count(self) -> int:
        v = self._get("data:recorded-sample-count")
        return v if isinstance(v, int) else -1

    def _samples(self, start: int, k: int) -> list:
        """Channel 1's samples start ... start + k - 1, or fewer (block 'y' of get-data's binary data). Refused,
        fewer at a time."""
        while True:
            k = min(k, self.chunk)
            try:
                blob = self.client.exec_bytes(self._p("data:get-data"), int(start), int(k))
                data = float_arrays(blob)
                break
            except DecopError as e:
                if self.chunk <= 250:
                    raise NoRecorder(f"its data cannot be read ({e})") from None
                self.chunk //= 4
        values = data.get("y")
        if values is None:
            raise NoRecorder(f"its data has no channel 1 (blocks {', '.join(sorted(data)) or 'none'})")
        return values[:k]

    def _asked_dt(self) -> float:
        """s between samples, by the recorder's sampling-interval, if it is in s, ms or us (0: no telling)."""
        v = self._get("sampling-interval")
        if not isinstance(v, (int, float)) or isinstance(v, bool) or v <= 0:
            return 0.0
        for scale in (1.0, 1e-3, 1e-6):
            if 0.2 * self.dt <= v * scale <= 5 * self.dt:
                return v * scale
        return 0.0

    def _dt(self, t0: float, count: int, size: int, t_full, at: float) -> float:
        """s between samples: as measured (the unit of its sampling-interval is not taken on trust), once the
        recording has gone on a while (count samples at `at`), or by when it was full; until then by that
        sampling-interval, or as asked for."""
        if t_full is not None:
            if t_full - t0 > 1.0:
                self.resolution = (t_full - t0) / size
        elif count > 0 and at - t0 > 1.0:
            self.resolution = (at - t0) / count
        return self.resolution

    def _check(self, values, sig: int, dt: float):
        """The first samples: do they show what the input shows?"""
        self.method = "recorder"
        name = IO_VALUE.get(sig)
        now = None
        if name is not None:
            try:
                now = self.client.get_float(name)
            except DecopError:
                now = None
        med = statistics.median(values)
        if now is not None and abs(med - now) > max(0.3 * abs(now), 0.005):
            self._say(f"the recorder's samples (median {med:.4f} V) do not look like {signal_name(sig)} now "
                      f"({now:.4f} V): watching them all the same", warn=True)
        self._say(f"watching {signal_name(sig)} with the DLC pro's recorder, {dt * 1e3:.2f} ms samples (its "
                  f"settings are put back when the program closes)")

    def _restore(self):
        """The recorder's settings as they were found."""
        if not self.saved:
            return
        saved, self.saved = self.saved, {}
        c, p = self.client, self._p
        try:
            c.set(p("enabled"), False)
            for name in self.SAVED[:-1]:
                if name in saved:
                    c.set(p(name), saved[name])
            if saved.get("enabled"):
                c.set(p("enabled"), True)
        except (OSError, DecopError) as e:
            log.warning("%s: dropout monitor: cannot put the recorder's settings back: %s", self.label, e)

    # -------------------------------------------------------------- polling
    def _poll(self, sig: int):
        name = IO_VALUE.get(sig)
        if name is None:
            self._say(f"{signal_name(sig)} has no present value to read: dropouts are not watched", warn=True)
            self._stop_ev.wait(60.0)
            return
        self.method = "polling"
        detector = Detector(0.0, 1)
        gaps = deque(maxlen=200)
        last = None
        period = 1.0 / self.POLL_HZ
        said_at = time.monotonic() + 5.0
        times = deque(maxlen=64)  # (sample number, time.time()) of the last reads
        i = 0
        while not self._stop_ev.is_set():
            if not self.status()[0]:  # nothing to watch: not asked
                detector.gap(period)
                self._tick()
                last = None
                self.seen, self.open_since = time.time(), None
                self._sleep(0.5)
                continue
            t, mono = time.time(), time.monotonic()
            v = self.client.get_float(name)
            i += 1
            times.append((i, t))
            if last is not None:
                gaps.append(mono - last)
            last = mono
            dt = statistics.median(gaps) if gaps else period
            self.resolution = dt
            self._tick()
            if said_at and mono >= said_at:
                said_at = 0.0
                self._say(f"watching {signal_name(sig)} by polling: {self.figures()}", warn=True)
            limit = self._account(1, dt, detector)
            if limit is not None:
                for first, span, low in detector.feed(i, dt, [v], limit):
                    began = next((tt for k, tt in times if k == first), t)
                    self.report(Dropout(began, span * dt, low, limit, "polling", dt))
            run = detector.going()
            self.seen, self.open_since = t, next((tt for k, tt in times if k == run), t) if run is not None else None
            wait = period - (time.monotonic() - mono)
            if wait > 0:
                self._stop_ev.wait(wait)
