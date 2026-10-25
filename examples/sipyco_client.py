"""SimpleWavemeterLock from any Python program, with sipyco's client: no ARTIQ, no device_db.

    pip install git+https://github.com/m-labs/sipyco    (it comes with ARTIQ; it is not on PyPI)

Any PC running SimpleWavemeterLock answers for every laser on every PC (right-click › Network…: ARTIQ's calls,
port 3280). A Client is one connection: open it once, call it as often as you like, close it at the end. It is for
one thread at a time: a second thread opens a Client of its own (Watchdog, below). A call that cannot be done
raises, saying why: LookupError (no such laser, or two that fit), ValueError, RuntimeError (refused, or not locked),
PermissionError (that PC lets no other PC adjust its lasers), TimeoutError; ConnectionRefusedError if nothing answers
on that PC and port.

Lasers are named as their rows are ('399'), or by wavelength ('398.91XXX': the one whose setpoint begins so; a
number: the one nearest it), or by wavemeter channel ('lab-pc-2#5'). Wavelengths are in vacuum, nm.

    python sipyco_client.py lab-pc-1             every laser, and whether it is locked
    python sipyco_client.py lab-pc-1 399 935     ...and whether those are okay
"""
import asyncio
import threading
import time
from contextlib import contextmanager

import numpy as np
from sipyco.pc_rpc import Client

HOST, PORT = "lab-pc-1", 3280  # any PC running SimpleWavemeterLock


@contextmanager
def wavemeter(host=HOST, port=PORT):
    """A connection, closed at the end:  with wavemeter() as wm: print(wm.is_ok("399"))"""
    wm = Client(host, port)  # (its one target, "wmlock", is taken by itself)
    try:
        yield wm
    finally:
        wm.close_rpc()


# ------------------------------------------------------------------------------------------- reading
def show_lasers(host=HOST, port=PORT):
    """Every laser shown there (that PC's and the others'): where, what it reads, its setpoint, locked or not."""
    with wavemeter(host, port) as wm:
        for x in wm.lasers():  # (a dict each: wm.status("399") gives one)
            reading = f"{x['nm']:.6f} nm" if x["nm"] else "no reading"
            setpoint = f"set {x['setpoint_nm']:.6f}" if x["setpoint_nm"] else "no setpoint"
            print(f"{x['wavemeter']}#{x['channel']:<3} {x['name']:<12} {reading:<16} {setpoint:<18} "
                  f"{'locked' if x['locked'] else x['state']}")


def read(laser="399", host=HOST, port=PORT):
    """Its latest reading, setpoint and deviation."""
    with wavemeter(host, port) as wm:
        nm = wm.get_wavelength(laser)    # nm, vacuum (wm.get_frequency: THz)
        setpoint = wm.get_setpoint(laser)  # nm, or None
        mhz = wm.get_deviation(laser)    # the reading less the setpoint
        print(f"{laser}: {nm:.7f} nm, setpoint {setpoint} nm, {mhz:+.2f} MHz, locked: {wm.is_locked(laser)}")


# ------------------------------------------------------------------------------------------- setting
def go_to(nm, laser=None, host=HOST, port=PORT):
    """Its setpoint to exactly nm, then until it is locked there (within 1 MHz). laser None: the one nearest nm (within
    30 GHz). A setpoint moved more than 30 GHz is refused (a typo?) unless force=True."""
    with wavemeter(host, port) as wm:
        done = wm.set_setpoint(nm, laser)  # {laser, wavemeter, channel, setpoint_nm, was_nm}
        which = f"{done['wavemeter']}#{done['channel']}"  # (that laser and no other, from now on)
        mhz = wm.wait(which, mhz=1.0, timeout=20.0)
        print(f"{done['laser']}: {done['was_nm']} -> {done['setpoint_nm']} nm, locked {mhz:+.2f} MHz from it")


def scan(laser="399", start=398.91100, stop=398.91140, steps=21, host=HOST, port=PORT):
    """Its setpoint stepped across a line, each step measured once it is locked there; put back at the end."""
    with wavemeter(host, port) as wm:
        before = wm.get_setpoint(laser)
        try:
            for nm in np.linspace(start, stop, steps):
                wm.set_setpoint(float(nm), laser)
                wm.wait(laser, mhz=1.0, timeout=20.0)
                take_point(nm)
            # (or in frequency: wm.shift_setpoint(+10.0, laser) moves it 10 MHz bluer)
        finally:
            if before:
                wm.set_setpoint(before, laser)


def lock(laser="399", host=HOST, port=PORT):
    with wavemeter(host, port) as wm:
        wm.lock(laser)  # (a laser its DLC pro locks to a cavity: Relock, watched)
        wm.wait(laser, mhz=2.0, timeout=30.0)
        # wm.unlock(laser)


# ------------------------------------------------------------------------------------------- okay?
def oven_run(lasers=("399", "369", "935"), seconds=600.0, host=HOST, port=PORT):
    """The oven on only while every laser it needs is fine: not started if one is not, stopped as soon as one stops
    being fine. True if it ran its course."""
    with wavemeter(host, port) as wm:
        ok, why = wm.is_ok(*lasers)  # (True, "") or (False, what is wrong, laser by laser)
        if not ok:
            print(f"oven run cancelled: {why}")  # e.g. "‘935’: not locked; ‘399’: +35.2 MHz from its setpoint"
            return False
        oven_on()
        try:
            end = time.time() + seconds
            while time.time() < end:
                time.sleep(1.0)
                ok, why = wm.is_ok(*lasers, mhz=20.0)  # (mhz: further than this from its setpoint is not fine)
                if not ok:
                    print(f"oven run stopped: {why}")
                    return False
        finally:
            oven_off()
    return True


class Watchdog(threading.Thread):
    """The lasers checked every `every` s on a connection of its own, while your code does something else (with a
    Client of its own: a Client is for one thread): .bad is set, and .why says why, the moment one is not fine.

        dog = Watchdog(["399", "935"]); dog.start()
        while measuring and not dog.bad.is_set(): ...
        dog.stop()
    """

    def __init__(self, lasers, every=1.0, mhz=20.0, host=HOST, port=PORT):
        super().__init__(daemon=True)
        self.lasers, self.every, self.mhz, self.host, self.port = list(lasers), every, mhz, host, port
        self.bad = threading.Event()
        self.why = ""
        self._stop_ev = threading.Event()

    def run(self):
        try:
            with wavemeter(self.host, self.port) as wm:
                while not self._stop_ev.wait(self.every):
                    ok, why = wm.is_ok(*self.lasers, mhz=self.mhz)
                    if not ok:
                        self.why = why
                        self.bad.set()
                        return
        except Exception as e:  # (the PC gone, say: not fine either)
            self.why = f"cannot ask: {e!r}"
            self.bad.set()

    def stop(self):
        self._stop_ev.set()
        self.join(5.0)


# ------------------------------------------------------------------------------------------- dropouts
def measure_with_dropouts(laser="Mulungu", points=100, host=HOST, port=PORT):
    """A measurement, its points taken while the laser was not locked (a lock dropout of a few ms, an unlock, a
    mode-hop recovery) marked bad. Returns (drops, the bad points' numbers)."""
    taken = []  # (began, ended) of each point, unix times
    with wavemeter(host, port) as wm:
        observation = wm.observe(laser)  # (its number)
        try:
            for i in range(points):
                began = time.time()
                take_point(i)
                taken.append((began, time.time()))
        finally:
            drops = wm.end_observe(observation)  # numpy array, n x 2: [unix time it began, s], in order
    starts, ends = drops[:, 0], drops[:, 0] + drops[:, 1]
    bad = [i for i, (began, ended) in enumerate(taken) if np.any((starts < ended) & (ends > began))]
    for t, s in drops:
        print(f"not locked from {time.strftime('%H:%M:%S', time.localtime(t))}.{int(t % 1 * 1000):03d} for "
              f"{s * 1e3:.1f} ms")
    print(f"{len(bad)} of {len(taken)} points marked bad")
    # (one still going on at end_observe lasts until then; one that began before observe, from when it began. Times
    # are by the clock of the PC whose laser it is: keep the PCs' clocks set, NTP)
    return drops, bad


# ------------------------------------------------------------------------------------------- errors
def careful(laser="399", host=HOST, port=PORT):
    """What each kind of refusal looks like."""
    try:
        wm = Client(host, port, timeout=60.0)  # (a timeout longer than any wait() you call)
    except ConnectionRefusedError:
        print(f"nothing answers on {host}:{port}: SimpleWavemeterLock not running there, or ARTIQ's calls off")
        return
    try:
        try:
            wm.set_setpoint(398.91121, "nope")
        except LookupError as e:  # no such laser, or two that fit (it says which)
            print(e)
        try:
            wm.set_setpoint(wm.get_setpoint(laser) - 9.0, laser)  # (a typo: 389.91121 for 398.91121)
        except ValueError as e:  # more than 30 GHz from where it was: wm.set_setpoint(..., force=True) to mean it
            print(e)
        try:
            wm.wait(laser, mhz=0.5, timeout=5.0)
        except TimeoutError as e:  # not within 0.5 MHz in 5 s
            print(e)
        except RuntimeError as e:  # not locked at all
            print(e)
    finally:
        wm.close_rpc()


# ------------------------------------------------------------------------------------------- asyncio
async def in_asyncio(host=HOST, port=PORT):
    """The same from asyncio code (sipyco's AsyncioClient)."""
    from sipyco.pc_rpc import AsyncioClient, AutoTarget

    wm = AsyncioClient()
    await wm.connect_rpc(host, port, AutoTarget)
    try:
        ok, why = await wm.is_ok("399", "935")
        print(ok, why)
    finally:
        closing = wm.close_rpc()  # (a coroutine in sipyco since 2025, a plain call before)
        if asyncio.iscoroutine(closing):
            await closing


# ------------------------------------------------------------------------------------------- yours
def take_point(i):
    time.sleep(0.05)  # your measurement


def oven_on():
    pass  # your oven's current on


def oven_off():
    pass


if __name__ == "__main__":
    import sys

    host = sys.argv[1] if len(sys.argv) > 1 else HOST
    show_lasers(host)
    if len(sys.argv) > 2:
        with wavemeter(host) as wm:
            print(wm.is_ok(*sys.argv[2:]))
