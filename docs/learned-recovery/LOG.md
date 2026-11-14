# Learned mode-hop recovery: log

What was tried on this branch, in order, and how each did. Numbers are from the simulator (`wmlock/trials.py`,
`wmlock/fastsim.py`) unless they say otherwise; nothing here has run on a real laser.

## 1. Where the time goes today

The best the existing methods do on the 493-like laser is the model's short way with the ramps ticked
(`--set smooth=true`): about 12 s when it works. A timeline of one such recovery (blue, seed 0):

| s | what |
|---|---|
| 0 to 2 | two plans worked out (a second each on the trials' rig) |
| 2 to 7 | the current out to its turning point and back, 0.1 mA a reading |
| 7 to 12 | the piezo onto the target, the margins checked ±0.5 mA |

When the margins' check finds a hop it starts over, and ends by a map: 48 s (blue, seed 1). By steps, as the
program comes, the median is 46 s.

## 2. How fast could it be? An oracle

With the simulated laser known exactly, a fast manoeuvre (the piezo to its final place, the current to a
turning point at 5 mA/s, held 2.5 of the chip's lags, on to a final current, held again) was tried for every
turning point and final current on a grid:

- blue: back on its right mode with 0.6 to 1.3 mA of margin in 0.7 to 2.1 s (15 lasers, all);
- red: in 0.2 to 0.4 s (its chip lags 40 ms, not 300);
- random lasers of that kind: in 0.6 to 1.8 s where there is a way at all (5 of 8; the others have no place with
  0.3 mA of margin, which is what the existing methods find too).

So seconds are there to be had. What has to be learned is where the manoeuvre must go for a laser the program
knows only by a model that is a little wrong, and by the hop it has just seen.

## 3. Many lasers at once (`wmlock/fastsim.py`)

`LaserModel.step` a point at a time is 0.1 ms; the training wants millions of points on lasers that differ.
`fastsim.Batch` is the same rule for a batch of lasers in lockstep, in numpy. Held against `sim.ModelLaser` along
random paths: the same mode at 48,120 of 48,120 points, the same margins. Kept as a test.

## 4. First try: a manoeuvre flown blind. Not good enough

A network (numpy, two hidden layers) from the situation (where the laser ran, which mode it hopped to) to the
chance of each of 902 blind manoeuvres (turning point, final current) ending on its right mode with margin; trained
on the oracle's word for thousands of lasers randomized about the model (each term of its rule spread 0.5 to 2
times what fits of one map differ by; half of them with their hysteresis by the edge of their mode; their chip's
modes drifting either way).

| 500 simulated hops kept apart (blue) | drifting up | drifting down |
|---|---|---|
| some manoeuvre brings it back (window -1 .. +2 mA) | 100 % | 68 % |
| the best single manoeuvre for each kind of hop | 72 % | 58 % |
| the network's first choice | 68 % | 48 % |

The network was not the limit: it is within a few points of the best any choice can do that knows only the kind
of hop. Where the right mode's range lies varies by half a mA with the laser's hidden hysteresis, and a blind
manoeuvre fits seven lasers in ten. More training would not have helped. Dropped.

Two things this showed that stayed:

- hops by a drift downward want the current well below where it ran (-2 mA) before it comes back up: the 493 nm
  laser's right mode "is come onto only sweeping up", as the README has it. With the window's bottom at -1 mA a
  third of those cannot be brought back by the current at all.
- the outcome depends on where the manoeuvre goes from where the laser ran, not on the window: the window only
  says which manoeuvres are allowed.

## 5. Second try: catching, with the readings in the loop. This is what is on the branch

The manoeuvre reads on the way. The piezo goes along with the current so that the right mode stays on target
all the way; the current goes to a turning point (or none) and is held; then it is ramped one way at a fifth of
a mA a reading until a reading has the laser on its right mode. From that reading, where it came onto the mode is
worked out (the chip lags the current by its lag, the reading came half an interval later), and the current goes
straight to a chosen depth past that edge. The chip itself never overshoots, so the ramp can be fast.

The same test, the best single catching manoeuvre for each kind of hop (blue, 400 hops kept apart):

| | drifting up | drifting down |
|---|---|---|
| window -1 .. +2 mA | 97 % | 63 % (72 % can be brought back at all) |
| window -2 .. +2 mA | 96 % | 87 % |

The policy now chooses among 72 such manoeuvres (12 turning points, 2 ways, 3 depths) and the network says for
each its chance, the margin to expect, and its time. It is learned for one laser from that laser's own model:
1,000 simulated hops looked at by the oracle, then 2,000 more flown by a first network, those it fails looked at
from where they are (what a second manoeuvre must know). About 20 s for a laser like the 650 nm one, a minute
or two for one like the 493 nm one, in numpy alone. No weights are shipped.

Checked that what is flown is what was simulated: the same manoeuvre on the same laser, on the trials' rig
(`Recenter.by_learning`, a `Runner`'s sweeps) and in `learned.fly`: it ends 0.04 mA apart on average (sd 0.04),
inside what the reading's phase is worth. Kept as a test.

Results on the trials' rig are in the README (*Learned recovery*).

## 6. A fault found by a test

`learned.situations` drifts each simulated laser's chip modes until it hops. For lasers with their hysteresis by
the edge of their mode, it looked at each place along the drift with the edge the laser had been at when the drift
began. A test (a thousandth of a chip mode before the hop, the laser must still be on its mode) failed for one
laser in five. Mended; the policies above are from after.

## 7. Not done, or not known

- Nothing has run on a real laser. The simulator's lasers are the model's kind; a real one that is not (the brief:
  the methods fail "even when a sweep has already shown the right mode crossing the target") is what the laser's
  own memory is for (`learned.Memory`: what each manoeuvre came to, added to the network's word), and that is
  tested only in the small.
- Lasers without a model (far, strong, near, shg in the trials) have no policy: `learned` is `auto` for them.
- The margins are not checked by moving the current (the methods of old step ±0.5 mA): the policy leaves the laser
  a chosen depth into its mode, and the trials' margins say how that comes out.
