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
  that has not chosen the new method. One exception, from round 2 below: the margin test may change for
  every method.
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

## Round 2: what the first tests on the 493 nm laser showed

On 14 November the learned method was tried on the real 493 nm laser. The policy had two real goes and
neither brought the laser back:

- The first three recoveries ran on the old method, because the policy was still being trained in the
  background (several minutes on the lab PC).
- In its first go, the policy took the current to the top of its window and judged the laser to be on its
  right mode, 3.3 GHz off. The piezo could not bring it onto the target, and the old method took over.
- In its second go, it found no manoeuvre with a chance within the window. The old method then brought
  the laser back in 15 s, to a spot the person had just found by hand.

Before these tests, the person changed the laser's temperature, because it was running on two modes at
its set point. Its model was fitted on 25 October, at the old temperature, so it no longer describes the
laser. The lab will map it again at the new temperature.

### To fix

- **The margin test destroys good spots.** The old method, which is also the learned method's fallback,
  put the laser on its right mode near the target 14 times that morning. Each time, the margin test (at
  least 0.15 mA, hard-coded) pushed the current over the mode's edge. Because of hysteresis, the laser did
  not come back, and the spot was given up. Make the test non-destructive: when it knocks the laser off a
  spot that was on target, bring it back the way it came in, and move away from the edge it found. If
  nowhere has the full margin, settle on target at the best spot found and say so, rather than leave the
  laser unlocked. This may change for every method.
- **The memory counted a failure as a success.** Reaching what the model calls the right mode is not
  success: count a manoeuvre as good only if it ends with the laser on target.
- **A recapture by hand was not learned from.** It ended on target, but the model judged its end point
  not to be the right mode, so it was not taken into the memory. On target is the right mode: trust the
  wavemeter over the model.
- **Training takes too long.** Recoveries in the meantime used the old method. Make it faster, or train
  ahead of time (when a model is taken, say).
- **The window got in the way.** Both of the policy's goes ran into the current window's limits.

### New: temperature as a slow knob

What fixed the laser by hand was its temperature. Temperature moves the diode's modes relative to the
external cavity, and that decides how wide the right mode's range is at the target. Allow the recovery,
and the keeping, to change it:

- It is slow: the diode takes seconds to tens of seconds to settle after a step. So it can't be the fast
  recapture. Use it as a second stage: recapture with current and piezo first, then, locked on target,
  move the temperature in small steps until the operating point sits in the middle of its mode, so that
  the next hop is days away rather than minutes. The keeping could use it the same way.
- The model must know what temperature does. That needs a map with temperature steps, which
  *Laser map…* can already make. The simulator already models the temperature controller.
- Rules: off unless chosen for a laser; the temperature only within a small window around where it was
  set, and in small, slow steps, settled before anything else moves; put back on failure.
