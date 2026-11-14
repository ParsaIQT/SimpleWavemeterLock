"""A recapture by hand, recorded: what someone does to get a hopped laser back (its set current and its piezo as
they go, and what the wavemeter read meanwhile), kept as a line of JSON, and read as a manoeuvre the learned recovery
can take into its laser's memory (learned.Memory): the way a person found is then its policy's first choice in a
like situation.

Why. A person gets the lab's 493 nm laser back in a minute, by eye. What they do was never kept: which way the
current went first, how far, where it was left. Recorded, it is the laser's own word on what works, from someone
who knows it; and, many of them together, something to try a policy against.

Recording (Engine.record_hand; right-click a channel: Record a recapture by hand): from then on every reading of
the channel is kept with the set current and the piezo as last polled (twice a second), until it is stopped, or the
laser has been on target for DONE_S, or MOST_S have passed. The lock and the recovery are best left off meanwhile:
it is the hand that is recorded. Nothing is written to the laser.

    python -m wmlock.handrec FILE.jsonl               # what was recorded: when, how long, how it ended
    python -m wmlock.handrec FILE.jsonl --map MAP     # ...and each as a manoeuvre, by the model kept in that map

read(recording, model) says what manoeuvre it was (learned.actions(): turning point, way, depth), as far as it was
one: which mode the laser was on at each reading (by its model's combs, from where it ran on target), where it came
onto its right mode and which way the current was going then, how far the current had been the other way before,
how deep in it was left. Needs numpy for that; recording does not.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field

DONE_S = 5.0     # s on target (every reading within its tolerance): recaptured, the recording ends
MOST_S = 900.0   # s a recording goes on at most
RIGHT = 0.3      # on its right mode: within this part of its modes' spacing of where that mode is
MOVED = 0.02     # mA: less of a change of its current is none


@dataclass
class Recording:
    """One recapture: the channel (n, name), when it began (time), the target (THz), where the laser ran on target
    before (ref: [piezo V, mA measured], None: not known), its current's window ([lo, hi] mA measured, None: none
    set), how far from the target counts as on it (tol, MHz), and what was seen: samples [s from the start, set mA,
    measured mA, piezo V, MHz from the target], None for whatever was not known at a reading; how it ended (end:
    'on target', 'stopped', 'too long')."""
    n: int
    name: str = ""
    time: str = ""
    target: float = math.nan
    ref: list | None = None
    window: list | None = None
    tol: float = 100.0
    samples: list = field(default_factory=list)
    end: str = ""
    t0: float | None = None
    on_since: float | None = None

    def add(self, t: float, i_set, i_act, v, e) -> bool:
        """A reading at t (s, monotonic) with what was known then; returns True once the recording is over (on
        target for DONE_S, or MOST_S long)."""
        if self.t0 is None:
            self.t0 = t
            self.time = self.time or time.strftime("%Y-%m-%d %H:%M:%S")
        r = lambda x, k: None if x is None or not math.isfinite(x) else round(float(x), k)  # noqa: E731
        self.samples.append([round(t - self.t0, 3), r(i_set, 4), r(i_act, 4), r(v, 4), r(e, 1)])
        on = e is not None and math.isfinite(e) and abs(e) <= self.tol
        self.on_since = (self.on_since if self.on_since is not None else t) if on else None
        if on and t - self.on_since >= DONE_S and len(self.samples) > 5:
            self.end = "on target"
        elif t - self.t0 >= MOST_S:
            self.end = "too long"
        return bool(self.end)

    def to_dict(self) -> dict:
        d = asdict(self)
        d.pop("t0"), d.pop("on_since")
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Recording":
        return cls(**{k: d[k] for k in ("n", "name", "time", "target", "ref", "window", "tol", "samples", "end")
                      if k in d})


def save(path, rec: Recording):
    """Appended to the file as a line of JSON."""
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec.to_dict(), separators=(",", ":"), ensure_ascii=False) + "\n")


def load(path) -> list:
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.strip():
                out.append(Recording.from_dict(json.loads(line)))
    return out


def read(rec: Recording, model) -> dict:
    """The recording as a manoeuvre of the learned recovery's (learned.actions()), by the laser's model: {x: the
    situation it began in (learned.features), k: which manoeuvre it comes nearest, ok: it ended on its right mode,
    jump: the modes it had hopped by, turn, way, depth: mA from where it ran, +1 up or -1 down, mA; entered, left:
    mA measured where it came onto its right mode and where it was left; why: '' or why it is none (then no x, k:
    it never left its mode, never came back onto it, came back by the piezo alone, where it ran is not known)}."""
    import numpy as np

    from . import fastsim, learned

    pts = [(s[0], s[2] if s[2] is not None else s[1], s[3], s[4]) for s in rec.samples
           if s[4] is not None and s[3] is not None and (s[2] is not None or s[1] is not None)]
    if len(pts) < 5:
        return {"why": "too few readings with its current and its piezo known"}
    if rec.ref is not None:
        v_ref, i_ref = float(rec.ref[0]), float(rec.ref[1])
    else:  # (where it ran is not known: where the recording began is taken for it)
        v_ref, i_ref = pts[0][2], pts[0][1]
    de, vsa = model.ext.spacing, model.v_set_to_act
    b = fastsim.Batch.of(model, 1)
    at_ref = float(b.mode_ghz(0, i_ref, v_ref + vsa)[0])
    jumps = [int(round((e / 1e3 - (float(b.mode_ghz(0, i, v + vsa)[0]) - at_ref)) / de)) for _, i, v, e in pts]
    jump0 = next((j for j in jumps if j), 0)
    if not jump0:
        return {"why": "it was on its right mode all along"}
    first = jumps.index(jump0)
    last = len(jumps) - 1
    while last > first and jumps[last] == 0:
        last -= 1
    ok = last < len(jumps) - 3  # (its last readings on its right mode)
    if not ok:
        return {"why": "it did not end on its right mode", "jump": jump0, "ok": False}
    k_on = last + 1
    i_on, i_end = pts[k_on][1], pts[-1][1]
    before = [p[1] for p in pts[first:k_on + 1]]
    way = 0.0
    for a in reversed(before[:-1]):  # (the way its current was going as it came onto it)
        if abs(i_on - a) >= MOVED:
            way = 1.0 if i_on > a else -1.0
            break
    if not way:
        return {"why": "it came back with its current where it was (by its piezo alone)", "jump": jump0, "ok": True}
    far = (min(before) if way > 0 else max(before)) - i_ref  # (how far the other way it had been before)
    turn = far if way * far < -0.2 else None
    depth = abs(i_end - i_on)
    turns, ways, depths = learned.actions()
    cost = (np.where(np.isnan(turns), 0.0 if turn is None else 9.0,
                     9.0 if turn is None else np.abs(np.nan_to_num(turns) - turn))
            + 9.0 * (ways != way) + 0.1 * np.abs(depths - depth))
    k = int(np.argmin(cost))
    lo, hi = (rec.window[0] - i_ref, rec.window[1] - i_ref) if rec.window else (min(before) - i_ref, max(before) - i_ref)
    lo, hi = min(max(lo, -2.5), 0.0), max(min(hi, 2.5), 0.0)
    x = learned.features(model, i_ref, v_ref + vsa, jump0, lo, hi)[0]
    return {"x": [round(float(a), 4) for a in x], "k": k, "ok": True, "jump": jump0, "turn": turn, "way": way,
            "depth": round(depth, 3), "entered": round(i_on, 3), "left": round(i_end, 3), "why": ""}


def main(argv=None):
    import argparse

    ap = argparse.ArgumentParser(prog="python -m wmlock.handrec", description=__doc__.split("\n\n")[0])
    ap.add_argument("file", help="recordings (lines of JSON: the program's ...-hand.jsonl)")
    ap.add_argument("--map", help="a map's file: each recording read as a manoeuvre, by the model kept in it")
    args = ap.parse_args(argv)
    model = None
    if args.map:
        from . import lasermap, lasermodel

        with lasermap.load(args.map) as m:
            model = lasermodel.stored(m)
        if model is None:
            ap.error(f"{args.map} keeps no model")
    for rec in load(args.file):
        took = rec.samples[-1][0] if rec.samples else 0.0
        print(f"{rec.time}  channel {rec.n} {rec.name}: {len(rec.samples)} readings in {took:.0f} s, {rec.end or '?'}")
        if model is not None:
            got = read(rec, model)
            if got.get("why"):
                print(f"    no manoeuvre: {got['why']}")
            else:
                turn = "no turn" if got["turn"] is None else f"by way of {got['turn']:+.2f} mA"
                print(f"    hopped {got['jump']:+d} modes; {turn}, {'up' if got['way'] > 0 else 'down'} onto its right "
                      f"mode at {got['entered']:.2f} mA, left {got['depth']:.2f} mA in at {got['left']:.2f} mA")


if __name__ == "__main__":
    main()
