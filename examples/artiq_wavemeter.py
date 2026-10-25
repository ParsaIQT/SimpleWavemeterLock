"""ARTIQ experiments that call SimpleWavemeterLock (wmlock/rpc.py): copy what you need into your experiment
repository. Each running copy of the program answers for every laser on every PC, so the device_db names any one
of them (best, one that is always on):

    device_db["wavemeter"] = {
        "type": "controller",
        "host": "lab-pc-1",  # that PC's name or address
        "port": 3280,        # its Network… settings: ARTIQ port
    }

Lasers are named as their rows are ("399"), or by their wavelength ("398.91XXX": the one whose setpoint begins so),
or by wavemeter channel ("lab-pc-2#5"). A call that cannot be done raises, saying why: LookupError (no such laser,
or two of them), ValueError, RuntimeError, PermissionError (that PC lets no other PC adjust its lasers),
TimeoutError.
"""
import time

import numpy as np
from artiq.experiment import EnvExperiment


class OvenRun(EnvExperiment):
    """Oven run, only while every laser it needs is fine: cancelled before it starts if one is not, stopped as soon as
    one stops being fine."""

    LASERS = ("399", "369", "935")  # (as their rows name them)
    RUN_S = 600.0                   # s the oven is on
    CHECK_S = 1.0                   # s between checks while it is on

    def build(self):
        self.setattr_device("wavemeter")

    def run(self):
        ok, why = self.wavemeter.is_ok(*self.LASERS)  # (True, "") or (False, what is wrong, laser by laser)
        if not ok:
            print(f"oven run cancelled: {why}")  # e.g. "‘935’: not locked; ‘399’: +35.2 MHz from its setpoint"
            return
        self.oven_on()
        try:
            end = time.time() + self.RUN_S
            while time.time() < end:
                time.sleep(self.CHECK_S)
                ok, why = self.wavemeter.is_ok(*self.LASERS, mhz=20.0)  # (mhz: how far off target is too far)
                if not ok:
                    print(f"oven run stopped: {why}")
                    break
        finally:
            self.oven_off()

    def oven_on(self):
        pass  # your oven's current on (a kernel, a TTL, a power supply's controller...)

    def oven_off(self):
        pass


class ScanWithDropouts(EnvExperiment):
    """A scan, its points taken while a laser was not locked (a lock dropout, an unlock, a mode-hop recovery) marked
    bad."""

    LASER = "Mulungu"

    def build(self):
        self.setattr_device("wavemeter")

    def run(self):
        taken = []  # (began, ended) of each point, unix times
        observation = self.wavemeter.observe(self.LASER)
        try:
            for point in range(100):
                began = time.time()
                self.take_point(point)
                taken.append((began, time.time()))
        finally:
            drops = self.wavemeter.end_observe(observation)  # numpy array, n x 2: [unix time it began, s]
        bad = [i for i, (began, ended) in enumerate(taken)
               if np.any((drops[:, 0] < ended) & (drops[:, 0] + drops[:, 1] > began))]
        self.set_dataset("drops", drops)
        self.set_dataset("bad_points", np.array(bad, dtype=int))
        print(f"{len(drops)} times {self.LASER} was not locked; {len(bad)} points marked bad")

    def take_point(self, point):
        time.sleep(0.1)  # your measurement


class SetpointScan(EnvExperiment):
    """A laser's setpoint stepped across a line, each step taken once it is locked there."""

    LASER = "399"

    def build(self):
        self.setattr_device("wavemeter")

    def run(self):
        start = self.wavemeter.get_setpoint(self.LASER)  # nm, vacuum
        try:
            for nm in np.linspace(398.91100, 398.91140, 21):
                self.wavemeter.set_setpoint(nm, self.LASER)  # exactly nm (a step over 30 GHz: refused, force=True)
                self.wavemeter.wait(self.LASER, mhz=1.0, timeout=20.0)  # locked within 1 MHz of it
                print(f"{nm:.5f} nm: {self.wavemeter.get_deviation(self.LASER):+.2f} MHz")
        finally:
            self.wavemeter.set_setpoint(start, self.LASER)
