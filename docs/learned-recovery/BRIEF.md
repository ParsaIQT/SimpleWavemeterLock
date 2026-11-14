# Learned mode-hop recovery: the brief

This branch is for one piece of work: a mode-hop recovery that **learns** how to bring a hopped laser
back, rather than following hand-written rules, and that gets it back on target, on its right mode,
**within seconds**.

## Why

- The lab's 493 nm laser (a Toptica DL pro: a grating ECDL, locked by this program to the wavemeter, no
  SHG stage) hops between its diode chip's modes (about 87 GHz apart) as well as between its
  external-cavity modes (about 8 GHz apart). Near its target, its right mode lases over only about half
  a milliamp of its 3 mA current window. Neighbouring chip modes take over on either side.
- The current methods (`map` and `model`: `modehop.Recenter`, `modelhop.Planner`) take 60 to 120 s on it,
  and often fail. They fail even when a sweep has already shown the right mode crossing the target.
- A person gets it back by hand, by eye, in about a minute. The program should beat the person, not
  lose to them. The aim: from the hop being seen to the laser locked on target, on its right mode and
  holding, in seconds. That goes for this laser, and for the other kinds the program recovers.

## What to build

Build a new recovery method alongside the existing ones (say `HopParams.method = "learned"`). It is used
only for a laser that has it chosen, and the existing methods remain its fallback. How it decides is up
to you: reinforcement learning, imitation learning, a learned model of the laser with fast planning, or
a mix. Choose what can really be trained with the data there is, and say why in the README.

Things to weigh:

- **Train in simulation.** A try on the real laser costs tens of seconds, and hops are rare, so learning
  on the laser itself would be slow. Use `wmlock/sim.py` and `wmlock/trials.py`. Their lasers include
  "blue" and "red", modelled on the lab's 493 and 650 nm lasers, and "model", random lasers of that
  kind. Make the simulator as like the real laser as you can.
- **Check against real records.** The lab can supply real recovery records: the recoveries JSONL file the
  program writes, with every map's readings and every step a recovery took. They are not in this
  repository, which is public.
- **Learning from people.** What a person does to get a laser back by hand is not recorded today. If
  learning from that would help, add a way to record it, so the lab can collect it: the current and
  piezo as set, and the readings, while someone recaptures a laser by hand.
- **Running in the program.** The policy runs inside this program on the lab's Windows PCs. numpy and
  scipy are there (numba is optional). Keep anything heavier out of the program at run time; a
  training script may use more. A decision must take milliseconds, not seconds.

## Hard rules

- Talk to the laser only through the requests a recovery yields (`modehop`: Move, Sweep, Read, Get, Set,
  Wait, ...), run by `modehop.Runner`. Keep every safety rule as it is:
  - the current within its window, at most `STEP_MAX` per write;
  - only the parameters in `modehop.WRITABLE`;
  - the piezo within its window;
  - everything put back on failure.
- The existing methods, their settings and their defaults stay as they are. Nothing changes for a laser
  that has not chosen the new method.
- No lab data, IP addresses or credentials in the repository.
- Keep the test suite passing (`pytest tests`; the GUI tests need Qt, and the model tests need scipy).
  Add tests of your own: the new method run through the trials' rig on many random lasers, with its
  safety rules checked on every write, as `tests/test_sweep.py` does for the ramps.

## How it is judged

Use the trials harness (`python -m wmlock.trials recover blue|red|model|far|near|shg|strong`), timed as on
the real laser, and compare against the current methods on the same lasers:

- how many lasers are recovered (on target, on a mode that holds);
- the time from the hop being seen to the lock handed back: the median, nine in ten, and the longest.

The target is seconds, not a minute, on the 493-like lasers, and no worse than now on any kind. Put the
table in the README.

## Where to start

- README sections: *Mode-hop recovery*, *Recovery by the laser's model*, *Ramps, not steps*, *Trials*.
- `docs/mode_hop_literature.md`.
- Code:
  - `wmlock/modehop.py`: Recenter, Runner, the requests;
  - `wmlock/modelhop.py`: Planner;
  - `wmlock/lasermodel.py`: the laser's model;
  - `wmlock/sim.py`: simulated lasers and the fake DLC pro;
  - `wmlock/trials.py`.

## Working

- Work on this branch only, and push to it, in small commits with the tests passing at each one.
- Don't open pull requests.
- Keep a short log in this folder (`LOG.md`) of what you tried and how it did, including what didn't
  work.
