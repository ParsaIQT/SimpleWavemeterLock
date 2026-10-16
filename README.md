# SimpleWavemeterLock

A small, fast GUI to lock TOPTICA DL pro lasers (DLC pro controllers) to a
HighFinesse wavemeter. It replaces WAND for day-to-day locking: one window,
one row per active wavemeter channel, right-click to pick a laser, click to lock.

![screenshot](docs/screenshot.png)

* **Active channels only**, each with its **vacuum wavelength to 9 significant
  digits** (computed from the measured frequency, so it does not depend on the
  air/vacuum setting of the WLM software), the deviation from the setpoint in MHz
  and the laser's scan offset.
* **Right-click a channel → Link laser** lists every DLC pro on the LAN (found the
  way TOPAS finds them, by UDP broadcast) with its serial number, system label
  and the lasers it drives (`laser1` … `laser4`). Pick one. **Enter IP
  address…** is there for controllers on another subnet.
* **Lock** runs a simple PI loop that only touches `laserN:scan:offset`. For a laser its
  DLC pro locks to a cavity, the button is **Relock** instead: it never touches the piezo, it
  watches the laser and relocks it after a mode hop, when it is locked on the wrong cavity peak,
  or when its lock keeps losing it.
* **Reference laser** (optional): a laser locked to a stable cavity corrects the
  wavemeter's drift for every other channel, and is ignored automatically while it
  mode-hops, unlocks or loses light.
* **Mode-hop recovery** (optional): after a mode hop, the laser is put back on its
  target on a mode that holds, by moving its diode current as well as its piezo, within
  a current window you set, and handed back to its lock: this program's wavemeter lock,
  or the DLC pro's own cavity lock (PDH / FALC), relocked by lockpoint on the right peak
  as you would by hand. For an SHG laser, as by hand: single mode first (its SHG cavity
  shows whether the master is), then the piezo walked slowly onto the target, the current
  trimmed whenever the mode gets worse; the SHG cavity is relocked too. With a TA, its seed
  power shows whether the laser lases: if it stops, its current is jumped up at once and
  brought back down slowly.
* Fast by construction: the wavemeter is read in its own thread, all network I/O
  happens in one worker thread per controller (writes are coalesced, Nagle off),
  and the GUI only repaints numbers that changed. Nothing ever blocks the window.

## Install

Python ≥ 3.8 and one Qt binding (PySide6 preferred; PyQt5 and PyQt6 also work):

```
pip install PySide6 ifaddr
```

`ifaddr` is optional. It is used to broadcast on every network adapter; without it
discovery uses 255.255.255.255 only.

## Run

```
python -m wmlock                 # on the wavemeter PC: reads wlmData.dll directly (fastest)
python -m wmlock --http http://wavemeter-pc:8000   # anywhere else, via server/server.py
python -m wmlock --sim           # demo: simulated wavemeter, two fake DLC pros, a reference laser
```

The source you choose is remembered, so afterwards plain `python -m wmlock` or
double-clicking `wmlock.pyw` (Windows, no console window) is enough. Other options:
`--dll PATH` for a non-standard DLL location, `--channels 1,2,5` to override the
switch's channel list, `--config FILE` for a different settings file (default
`~/.simplewavemeterlock.json`).

The WLM software must be running, with the multichannel switch cycling over the
channels you want: the GUI never switches channels itself. It reads every
channel as soon as the WLM has a new result, and shows the channels ticked
"Use" in the switch settings.

## Use

| | |
|---|---|
| Right-click a channel | Link laser ▸ controller ▸ laserN, Enter IP address…, Rescan LAN, Unlink, Setpoint = current reading, Lock settings…, Mode-hop recovery…, Recenter now, Check setup…, Last recovery…, Use as reference laser, Rename…, Hide channel |
| **Lock** button | Engage or release. Without a setpoint it locks to the current reading. Green means locked; a red bar means it unlocked itself, and the reason is shown under the deviation. Amber *Recentering* means a mode-hop recovery is running: click to stop it (everything goes back). Blue *Continue*: step by step, it waits for you to allow the step shown on the row. |
| **Relock** button | For a laser its DLC pro locks to a cavity (*Mode-hop recovery… ▸ Locked by its DLC pro*), in place of Lock: this program does not lock it to the wavemeter. Click it to have the laser watched (*Watching*, green) and relocked after a mode hop, when it is locked more than 50 MHz from its target (another peak: the wavemeter is never that far off), or when its lock keeps losing it and relocking (its transmission, on average, far below its level locked); if any of that is so when you click, it is relocked there and then. Normally it does nothing at all. Click *Watching* to stop. |
| `set` field | Setpoint as vacuum wavelength in nm. Type and press Enter, or put the cursor next to a digit and use ↑/↓ or the mouse wheel to step that digit. This works while locked, so you can walk the laser. |
| Double-click the name | Rename |
| Right-click empty space | Rescan LAN (F5), Show hidden channels, Always on top, Larger/smaller digits (Ctrl +/−); with `--sim`, Simulate a fault |

Greyed digits mean the channel has had no new reading for 3 s.

## The lock

On every **new** reading of a channel (never on stale data), with
`e = measured − setpoint` in GHz:

```
offset = offset_at_engage − (P·e + I·Σe)
```

* Gains are per reading, so the loop behaves the same whatever the switch cycle
  time is. Defaults: P = 0, I = 0.5 V/GHz. With a tuning coefficient K
  (GHz/V), I = 1/K corrects the whole error in one reading; about 0.3/K is a
  robust choice.
* **Sign**: positive gains assume the frequency rises with the offset. If a lock
  runs away, negate the gains in *Lock settings…*. In the `--sim` demo the
  866 nm laser is deliberately built the other way round, so you can watch the
  lock catch it.
* The lock is bumpless: it reads the present offset when you engage. Each reading
  moves the offset by at most *Max step* (0.25 V). The lock unlocks itself and
  says why if any of these happens:
  * the offset would leave *Min/Max offset* (0–140 V);
  * the offset would move more than *Max excursion* (±10 V) from where it was locked;
  * the error stays outside the *Capture range* (3 GHz, e.g. a mode hop) for 3 readings;
  * the controller stops answering.
* Invalid readings (low signal, overexposed, …) are skipped and the laser is
  left alone. Locks always start released when the program starts.

## Reference laser

If one of the lasers on the wavemeter is locked to a cavity that is more stable
than the wavemeter, right-click it and choose **Use as reference laser**. Its row
turns blue and shows the raw reading, where the reference should be (`ref`), and
the wavemeter's error and drift rate as measured on it. Every other channel is
then corrected for that error before display and before its lock acts; their `nm`
turns blue as a reminder, and the tooltip shows the raw reading. Locked lasers keep
a fixed relationship to the reference laser, however the wavemeter drifts, and
across restarts.

![reference](docs/reference_moved.png)

**How the correction works.** A Fizeau wavemeter's slow error is, to first order, the
same relative error ε on every channel. The reference sits at a fixed frequency F0,
so its reading F0·(1 + ε) measures ε, and each other reading f becomes f / (1 + ε).
(*Correction: the same offset for every laser* is an option in *Reference settings…*.)
F0 is wherever the reference is when you choose it, so nothing jumps. If you know the
reference laser's true wavelength (a comb, an atomic line), type it into *Reference
settings…*: every channel becomes absolutely calibrated, and locked lasers move to keep
their setpoints in that frame. The GUI asks first, and says by how much they will move.

**Why a misbehaving reference is ignored.** A reference that misbehaves jumps, sweeps,
runs away or goes dark. The wavemeter can do none of those: it only drifts, and slowly.
So:

* **Tracking.** The wavemeter's error and its rate are estimated with a critically
  damped α-β tracker (*Averaging time* 30 s). It follows real drift without lag.
* **Ignored readings.** A reading further than 6σ (at least 2 MHz) from the prediction
  is ignored. σ is the reference's own reading-to-reading scatter, which a slow
  runaway cannot inflate.
* **Hold.** Three such readings in a row, missing light, 10 s without readings, or an
  estimated rate above *Wavemeter drifts at most* (2 MHz/min) put the reference on
  **hold**. The correction is frozen there; for a slow runaway it is first wound back
  to before it started. Locks carry on with the frozen correction.
* **Resuming.** To be trusted again, the reference must produce a run of consistent
  readings. A straight-line fit over that run has to show that it is quiet and its
  slope is below the drift limit, within its own uncertainty. That takes about 10 s
  with a quiet wavemeter, longer with a noisy one. The run must also sit where the
  frozen correction expects it, give or take what the wavemeter can have drifted
  meanwhile (*drift limit × time*, at most *Largest drift over an outage*, 50 MHz). The
  difference is then taken up as wavemeter drift.
* **Moved.** If the reference comes back steady but somewhere else, e.g. on another
  cavity mode, the row turns red. The correction stays frozen until you right-click
  and decide:
  * *The reference laser moved: follow it*: nothing else moves.
  * *The wavemeter moved: accept … as drift*: locked lasers follow, and you are told
    by how much first.

Things to know:

* Set *Wavemeter drifts at most* comfortably above your wavemeter's real drift (WS7/WS8
  typically drift well under 0.5 MHz/min). Near the limit, proving "slower than the
  limit" takes long.
* A reference that unlocks but then drifts no faster than a wavemeter cannot be told
  apart from wavemeter drift by its readings alone.
* The WLM software's own auto-calibration steps the wavemeter's error. A step bigger than
  the gate shows up as *moved*: accept it as drift. Better, turn auto-calibration off,
  since the reference now does that job continuously.
* The reference's channel cannot be locked to the wavemeter; it is locked to its cavity.
* In `--sim`, channel 6 (729 nm) is a cavity-locked reference.
  *Right-click empty space ▸ Simulate a fault* gives it mode hops, a cavity unlock or a
  blocked beam, or steps the wavemeter, so you can watch all of the above happen.

## Mode-hop recovery

An external-cavity diode laser lases on the external-cavity mode best aligned with its
selection filter, the grating together with the diode chip's own modes. Temperature
drifts slide the filter until a neighbouring mode wins: a mode hop, typically by a few
GHz. Pulling the frequency back with the piezo alone, as a lock does, can leave the laser
at the edge of its mode, where it runs on two modes at once and soon hops again. The
current moves the filter quickly (the chip modes shift by GHz per mA) and the lasing
frequency only slowly, so a small current change re-centers the filter on the mode, and a
re-centered mode holds.

![mode-hop recovery](docs/modehop.png)

Right-click a channel ▸ **Mode-hop recovery…**:

* **Locked by**: *this program, to the wavemeter*, or *its DLC pro, to a cavity*
  (a DLC pro lock with PDH and FALC, say).
* **It has an SHG stage**: for a TA-SHG / DL-SHG pro. Its SHG cavity is used to see how
  single-mode the master is, and is relocked afterwards (see *SHG lasers* below).
* **Nominal current**: the laser's normal current **as measured** (`current-act`, the DLC pro's
  actual current), read from the controller when you enable recovery. The current always stays
  within **1 mA below and 2 mA above** it (you set both: lower, a laser may stop lasing; see
  *Lasing*), and never above the controller's own `current-clip`. The set current usually
  differs from the measured one (a current offset, feedforward from the piezo, …): a recovery
  writes the set current, offset by the difference it finds when it starts, and every current
  it shows is measured. With the DLC pro's **feedforward** on, the piezo moves the current too
  (`feedforward-factor` mA/V): the window, and the floor kept above where the laser stopped
  lasing, are on the measured current wherever the piezo goes (the set current follows), and
  no piezo move changes the measured current by more than 0.2 mA at once. A current found
  **outside** its window when a recovery starts (left high where a laser that had stopped
  lasing lased again, say) is brought into it first: from above slowly, asking every 0.2 mA
  whether it still lases, and on down, as by hand, until the SHG sweep is single-mode.
* A **hop** is the laser off its target by more than 100 MHz for 2 s, after it has been on
  target (a cavity-locked laser: more than *Wrong peak*, 50 MHz, below). The target is the
  setpoint (`set`). For a cavity-locked laser, type the
  wavelength it has when locked on the right peak (or *Setpoint = current reading* while it
  is), then click **Relock** on its row: it is watched from then on. The target follows the
  wavemeter's slow drift for such a laser. With a reference laser, the corrected reading is
  used. On the reference laser itself, the target is where the reference says it should read.
* **Method**: how it finds a mode on target. **Walk** (the default with an SHG stage), as you
  would by hand: single mode first, then the piezo slowly onto the target, keeping an eye on the
  mode in the SHG cavity and trimming the current when it gets worse. **Map** (the default
  without one): sweep the current over its window, up and down, and pick the mode that holds
  best. The walk keeps the current near nominal; the map needs the window both ways.
* **Lasing**: the parameter that shows the TA's seed power (*auto*: `amp:pd:seed:power`, else
  `amp:seed-limits:power`; *off*: not watched), what fraction of its usual power is not lasing
  (half), and how far the current jumps up when it does not lase (2 to 5 mA, back down at
  0.5 mA/s). See *Lasing* below.
* **Relock by** (cavity only): *by lockpoint*, as you would by hand (the DLC pro is asked to find
  its lock points first, `find-candidates`: it shows none otherwise), or *re-enable the lock* and
  let it find its resonance (*lock without lockpoint* is ticked first: without it, it locks only at
  a selected lockpoint). The default follows the DLC pro's `lock-without-lockpoint` setting. After
  a relock, the setpoint is moved to where the wavemeter puts the laser locked: the lockpoint is the
  target, in the wavemeter's numbers.
* **The right peak is within ± 150 MHz** of where the wavemeter puts the setpoint.
* **Wrong peak**: locked, but more than **50 MHz** from the target: the wavemeter is never that
  far off, so the lock holds the laser on another peak of the cavity (a transverse mode, say),
  and it is relocked. A wavemeter-locked laser keeps the 100 MHz above.
* **Lock slipping**: the cavity transmission (the input the DLC pro's lock detection watches,
  Fine In 1 as a rule; with lock detection off, the lock input) is read twice a second while the
  DLC pro's lock is on, and goes by two levels, in volts:
  * its **zero**: the transmission between the peaks (any DC offset), measured while the master
    scans in a relock by lockpoint (the median of ten looks; 0 V until then);
  * its **level locked**: the median of what it read over the last few minutes while the laser ran
    locked on target (watched or not), and what a relock measured, until then.

  Its average over 10 s below **half way** from zero to locked (0 V and 0.05 V: below 0.025 V)
  means the lock keeps losing the laser and relocking (ReLock): it is taken over, its mode looked
  at, and relocked; the wavelength may well look fine meanwhile. The row shows the transmission in
  volts with its level locked (*transmission 0.0493 V (locked 0.0500)*), and **CAVITY LOCK
  SLIPPING** with the limit when it falls below; *Check setup…* shows which input is watched, both
  levels and the limit. A lock that is off is not watched for this: turning it off and on again by
  hand is not taken for slipping. The same goes for an SHG laser's intra-cavity signal (locked):
  below half its level locked over 10 s, it is recovered (the SHG cavity lost it, or the laser
  stopped lasing).
* **Null the error signal's DC level with the FALC pro's input offset, by at most 0.05 V.**
* **Step by step**: before each step, the recovery waits for you (see *Before the first run*).

### Lasing

A laser can stop lasing altogether when its current drops too far, and setting the current
back does not start it again: it needs a good deal more current first. What tells:

* **its SHG sweep** (with an SHG stage): a laser that lases shows its resonances; one that does not
  sends broadband light (an AR-coated chip without its feedback), and the sweep **flat-lines**. A
  flat sweep where there should be resonances (they have been seen in this recovery, it was locked
  when the recovery began, or its usual look or signal is known) means it does not lase, unless its
  TA is off (then the row says so). The wavemeter has nothing to read then either: whenever it has
  no valid reading of the laser after a move, the SHG sweep is asked.
* **its TA's seed power** (TA pro, TA-SHG pro): below half of what it is while the laser lases on
  target (learned as it is watched), and below the TA's own `power-min` in any case, it is not
  lasing. The broadband light of a laser that does not lase can keep the seed power well up,
  though, so with an SHG stage the sweep has the last word.

Not lasing, at once, without asking (the TA switches itself off after its
`power-min-shutdown-delay` with too little seed):

1. the cavity lock goes off (it would fight), and the current goes up to **2 mA above
   nominal**, then 3, 4, **5 mA**, until it lases again (the SHG sweep shows resonances, or the seed
   power is back): 0.2 mA a write, no reading in between, half a second at each mA (and an SHG
   sweep), so a few seconds;
2. it comes back down to nominal **slowly**, 0.5 mA/s or as fast as an SHG sweep a step allows,
   asked every 0.2 mA whether it still lases;
3. if it stopped lasing on the way somewhere it had been seen lasing, it is never let that low again
   in this recovery (half a mA above where it stopped): up once more, and down only that far.

Then the recovery goes on as after a hop. This happens whenever a watched laser (*Locked*,
*Watching*) stops lasing: by its seed power it is recovered straight away; if the seed power is
fooled, its SHG signal (and cavity transmission) gone on average for a few seconds start the
recovery, and since the wavemeter then has nothing to read either, the hourly limit does not
count. It also happens at the start of any recovery, at every SHG sweep, and after any step that
moves the current or the piezo; a move down is checked every 0.2 mA by the seed power. The row
shows the seed power and the SHG signal (in volts, with its level locked), **NOT LASING** in red
when the seed power is too low, and **SHG SIGNAL GONE** when that is.

If it does not lase even 5 mA up, the current goes back down to where it was (slowly), and the
row says to look at the laser. If its TA switched itself off meanwhile, the laser is left
lasing where it is, and the row says to switch the TA on again: this program never switches a
TA on. Without a TA or an SHG stage nothing tells whether it lases, and nothing is watched: such a
laser gives the wavemeter no valid reading, which is not taken for a hop.

### What happens after a hop

The row turns amber and shows each step; at 10 readings a second of that channel it takes
about half a minute (mapping an SHG laser, a minute), and proportionally longer if the
wavemeter reads it less often.

1. **Take over.** A wavemeter lock is suspended. A cavity lock is disabled
   (`laserN:dl:lock:lock-enabled`), so is the SHG cavity's lock, and if that leaves the
   DLC pro's scan running it is stopped too. How the lock is set up is read first (lock
   input, error signal, FALC pro, the lockpoint you selected, scope channels).
2. **Find a mode on target**: by the walk or by the map (below).
3. **Hand back.** The wavemeter lock is re-engaged at the setpoint (a cavity-locked laser is
   watched again). For a cavity lock, see below; the wavemeter must show the right peak
   (within *Wrong peak*, 50 MHz) for 5 readings in a row, or it unlocks and tries again, up to
   three times. Relocked, the setpoint is moved to where the wavemeter puts it (the middle of
   those readings), and the note says by how much.

**The walk**, as you would by hand (the default with an SHG stage):

1. **Single mode first.** The SHG cavity is scanned and its intra-cavity sweep looked at
   (below). Not single-mode: the current is trimmed 0.1 mA at a time, up first (down, it may
   stop lasing), while the sweep gets no worse, then the other way, until it is single-mode,
   on while its main peaks still grow, and two more steps for good measure.
2. **Near enough?** Within **8 GHz** of the target, the piezo walks it there. Further: the
   current goes the way that brings it nearer (up as far as the window's top, or down as far
   as its bottom) until a hop brings a mode within 8 GHz, and a few steps more the same way, out
   of the new mode's edge. If none does, or the mode it hops to will not stay single-mode, it
   walks from where it was, as far as the piezo may go.
3. **Walk.** The piezo moves onto the target in **0.5 V steps** (its tuning is measured first,
   0.5 V and back, the first time only), and after every step: the frequency (a hop? Back to 1,
   from there), the seed power, and the SHG sweep. Once the mode looks worse than it did when
   the walk began (main peaks 10 % smaller, or multimode), the current is trimmed until it is as
   good again. A reading that says it hopped is confirmed by two more first: on two modes at once
   the wavemeter reads now one, now the other. A step that takes it the wrong way has its tuning
   measured again.

   **Parked high first**, when the feedforward will take the current down on the way (as by
   hand: walking the piezo, the current drops, towards where the laser stops lasing): the
   current goes up 0.1 mA at a time while the SHG sweep stays as good, until it changes (main
   peaks 5 % smaller, a second mode, or a hop), then two steps back, to where it showed no
   change. Whenever the walk has taken it 0.3 mA down from there, it is parked high again.
4. **On target**, single-mode by the SHG sweep there: handed back.

**A laser that is fine is only relocked.** If it is on target and single-mode when it is taken
over (its lock was merely off, slipping, or on another peak; or you pressed *Recenter now*),
nothing is moved but the piezo onto the target, if it is not quite there: the current is never
touched, not even to look either side of it (a laser close to where it stops lasing could stop),
and step by step you are asked only to take it over and then to relock. Two modes of about equal
power can look like one clean mode in a sweep, though: so the SHG cavity, once locked, must show
at least 80 % of its usual intra-cavity signal (learned while the laser runs locked); if not, the
master is not single-mode, the current is trimmed to where the sweep's main peaks are biggest,
and it is handed back again. If that did not bring the signal up, it is handed back all the same,
and the row says so (*SHG signal only … of usual*: a second mode whose peaks fall on the main ones
in the sweep, or a weaker TA).

The walk does not need the current to go far from nominal, and the mode it ends on is the
nearest one that reaches the target, not necessarily the one it was on before the hop. With
the piezo's feedforward to the current set right, a mode stays centered as the piezo walks, and
no trims are needed; but the measured current still moves with the piezo, which is what parking
high is for.

**The map** (the default without an SHG stage). Which way the current comes matters here: a
mode, once lasing, holds on past the point where it
would not have started: sweeping the current up, the laser hops onto a mode later than it hops
off it sweeping down (hysteresis). So where a mode lases sweeping up can be quite different from
where it lases sweeping down, and the same current gives one mode or the other depending on
where the current came from. The map keeps the two directions apart; a range where the target's
mode lases sweeping either way is enough; and the laser is brought to its operating point the
way the map found its mode there.

1. **Quick check.** If the laser is close to its target (the cavity lock had caught a wrong
   peak, say), the piezo tunes it back, and steps of the current check that it is clear of
   the next hop. If so, it is handed back.
2. **Map.** At fixed piezo, the current is ramped to the top of its window and down to the
   bottom (so the laser starts on the mode a sweep down leaves it on), then swept **up**
   across the window and back **down**, one reading per 0.1 mA step. The readings fall into
   straight lines, one per mode, broken by hops: for each mode, where it lases sweeping up and
   where sweeping down. With an SHG stage its cavity is scanned meanwhile, and wherever the
   sweep shows a second mode is cut out of that mode's ranges; so is wherever the readings
   scatter off the mode's line (on two modes at once the wavemeter reads now one, now the
   other).
3. **Choose.** Every range where a mode lases, sweeping up or sweeping down, is a candidate.
   Where to operate in it: in the middle of where the mode holds once it lases, between where
   it hops off sweeping up and where it hops off sweeping down; seen one way only, between
   where it hops on and where it hops off. At least 0.2 mA past where it hops on, clear of
   where it hops off, and inside where it lases both ways if it does there. The best is the
   widest that way that needs the least piezo to reach the target. A range that ends at the
   window's edge is taken to go on a little beyond it.
4. **Park.** The current goes round by the far end of the window to the end the map's sweep
   that way started from, and is swept that way again, a reading a step, up (or down) to the
   operating point: the laser comes to it as it did in the map. It must be on the mode the
   map found there (else it maps again). The piezo tuning (MHz/V) is measured there.
5. **Tune and check.** The piezo brings the laser onto the target, watching for hops. Then
   the current is stepped ±0.5 mA (*Keep a hop at least …*) around the operating point: the
   frequency must stay on one straight line and come back, and with an SHG stage the laser
   must be single-mode at both ends and in the middle. If not, or if the piezo moved a lot
   (the modes move with it), it maps again there, up to three times.

### SHG lasers, relocked by lockpoint

For a TA-SHG pro whose master its DLC pro locks to a reference cavity (lockpoint picked in
the XY display, PDH through a FALC pro) and whose SHG cavity it locks to the master, the
recovery does what you would do by hand:

| By hand | Here |
|---|---|
| Unlock the master and the SHG, stop the master's scan | *Take over* |
| Make sure the master is single-mode-ish before moving the scan offset, then move it slowly towards the target, watching the SHG sweep; trim the current when the mode loses quality | *the walk*: the SHG sweep after every trim and every 0.5 V of piezo, and at the operating point (the map: every map point, both ends of the margin check, and the operating point) |
| Lock the SHG cavity | SHG scan off, `nlo:shg:lock:lock-enabled` on; the intra-cavity signal (`nlo:pd:shg-int:photodiode`) must come up, to at least 80 % of its usual locked (learned while the laser runs, else what it was when the recovery began): if not, the master is not single-mode, and the current is trimmed (once; then it is handed back all the same, and says so) |
| Start the master's scan, bring the right cavity peak into it, show the lock points, pick it in the XY display (scan output against Fine In 1) | the scan runs; the transmission is looked at ten times, unevenly spaced: mostly between the peaks, their median is its zero; the DLC pro is asked to find its lock points (`dl:lock:find-candidates`) before each look at them; from those candidates (`dl:lock:candidates`), the tall peak where the wavemeter puts the target (scan offset − deviation / piezo tuning), within ± 150 MHz, and the only one there; it must stand at least half as tall as the right peak did last time (from the lockpoint you selected), so a transverse mode is never taken for it |
| Correct the FALC's input offset (~0.01 V) so the error signal has no DC | the error signal's level away from the resonances, from the scope; the input offset steps against it, checking on the trace after each step that it does what it should |
| Lock | `dl:lock:select-lockpoint` on that candidate, then `lock-enabled`; the lock must report locked, the wavemeter must agree it is the right peak, and the transmission (the median of a few looks) must reach half the peak's height above zero; it is its level locked from then on |

How a sweep is judged: a single-mode master gives one tall peak per free spectral range of
the SHG cavity, plus small transverse-mode bumps (always the same); a second mode adds its
own family of peaks and takes power from the main ones. A peak is measured by its area, which
does not depend on where the samples fall on a narrow resonance. What single-mode looks like
on your laser is learned from the first recovery (from a map: where the main peaks are
biggest) and kept. A sweep is called multimode if its biggest side peak is more than 10 % (of
a main peak) above the usual, or the main peaks are below 80 % of their usual size (which
also catches two modes of equal power: twice the peaks, each half as big). Until it is known,
a side peak over 20 %, or main peaks below 80 % of the biggest seen in this recovery, is
multimode; two modes of about equal power can pass that, but not the SHG lock's check above. A second mode with less than about 10 % of the power passes; you would hardly see
it by eye either. Where the scan turns, a resonance can show cut short, or as two crossings
run into one: with the SHG piezo's voltage on the scope's other channel (as by default), peaks
there are left out, and a side peak that does not recur in the sweep (each resonance is crossed
sweeping up and again sweeping down) is never taken for a second mode.

![last recovery](docs/last_recovery.png)

*Right-click ▸ Last recovery…* shows what it saw and decided: the steps, the walk (frequency
against scan offset, each reading marked by what the SHG sweep said, and the current, trims and
all), each map (the sweep up, the sweep down and the parking sweep, in measured current; where
the candidates lase; where the laser showed two modes), the last SHG sweeps (and, from a map,
a single-mode and a multimode example), the cavity scan
with the candidates, where the target should be and the peak picked, and the error signal
before and after nulling its DC. Every recovery is also appended to
`~/.simplewavemeterlock-recoveries.jsonl`.

### Before the first run on the real laser

![check setup](docs/setup_check.png)

1. *Right-click ▸ Check setup…* reads (only reads) everything a recovery would use and says
   what looks wrong: the current (measured and set, and what makes them differ: current offset,
   feedforward from the piezo, a limited slew rate) and its window, the seed power and the TA's
   own seed limits, the method, the scan, the lock's state, whether it relocks
   by lockpoint, **the lockpoint you selected by hand** (its type and level are what the
   recovery looks for), the lock input (should be Fine In 1) and the error-signal channel, the
   FALC pro and its input offset, the scope channels, the SHG lock, scan and scope. Nothing is
   written; read it with the laser locked as you like it.
2. Set the setpoint while it is locked on the right peak (*Setpoint = current reading*).
3. Tick **Step by step** and press *Recenter now*. The row turns blue and says what it is
   about to do (*next: …*); **Continue** does it, *right-click ▸ Stop recentering* puts
   everything back. Watch each step on the DLC pro: the SHG sweep, the current trims, the piezo
   walking, the XY display, the lockpoint it selects, the FALC offset. (A laser that stops
   lasing is jumped up without asking.)
4. Then *Last recovery…*: do you agree with its verdicts on the SHG sweeps, with the trims, with
   the peak it picked, with the DC level it saw?
5. Click **Relock** on the row: it is watched from now on (*Watching*).
6. Untick *Step by step* once you trust it.

Safety:

* The current, as measured, never leaves nominal − 1 mA … + 2 mA (*Current stays within*),
  except that a laser that stopped lasing may be taken up to nominal + 5 mA, and that a current
  found outside the window only ever moves towards it (and back there on failure); it never
  changes by more than 0.2 mA per write, nor does the feedforward move it more than that per
  piezo write. Larger moves are ramped. Each write is checked once more just before it is sent,
  whatever the procedure asked for. With a TA, the current comes down slowly (0.5 mA/s) after a
  jump and when it is put back.
* The piezo moves at most ± 25 V per recovery (*Piezo moves at most*), inside the lock's
  offset limits: a walk of 8 GHz takes some 20 V.
* Besides current and offset, a recovery writes only: the cavity lock and SHG lock on/off, the
  cavity lock's *lock without lockpoint* (relocking by re-enabling it), the scan and SHG scan
  on/off, a scope channel's signal (given back), a FALC pro's input offset (within its limit of
  where it was), and runs `find-candidates` and `select-lockpoint`. Anything else is refused. (And
  in this program, a relock moves the setpoint onto the lock.)
* If nothing works, or after 10 minutes, everything it changed goes back where it was and
  the cavity lock stays off, with the reason on the row. Stopping it does the same. A laser
  that had stopped lasing is put back where it lases again, not where it stopped; stopped while
  it is not lasing, the current goes back by way of the top, as in a jump. With hysteresis,
  putting the current back can leave the laser on another mode than it was: so before giving
  up a map brings the current back the way that gives the mode it began on, if the first map
  saw which way that is. If it
  cannot tell which cavity peak is right, or the DLC pro will not take the lockpoint, the
  laser is left at its target, unlocked and scanning, for you to pick the peak. If the lock
  caught a wrong peak three times, the laser is left on its new, good operating point with
  the lock off.
* At most 4 recoveries an hour; beyond that it stops and says so (a laser that stopped lasing
  is recovered all the same). It also does nothing until the laser has been on target again.
* If the controller stops answering, it stops and says what the current was left at.
* A recovery only starts from a current within the window. Set the nominal current again
  after changing it by hand.

*Recenter now* runs a recovery at any time, without a hop (the map skips its quick check): it
re-centers a laser you suspect sits near the edge of its mode. It needs a setpoint and a valid
reading.

Recoveries learn the laser's piezo tuning and mode spacing, the right peak's height, the
lockpoint type and the number `select-lockpoint` wants for it, what single-mode looks like in
the SHG cavity, how the FALC offset shows on the error signal, the cavity transmission's zero;
watching it, its usual seed power and the levels locked of its cavity transmission and SHG
intra-cavity signal are learned too. They are kept
(the dialog shows them, *Forget* clears them) and make later recoveries quicker and surer.

Things to know:

* This has been tested against simulated lasers and a simulated DLC pro only. The walk, on 300
  random SHG masters (modes 3 to 10 GHz apart, the piezo tuning them by 0.2 to 0.8 GHz/V and
  the grating by 60 to 95 % of that, no feedforward), going multimode over 15 % or more of each
  mode, quietly for the wavemeter, hopping 1 or 2 modes, a random DC level of up to ±30 mV on
  the error signal, their usual SHG signal known (as it is once a laser has been watched for a
  minute): of the 245 hops that could be fixed within its windows, 235 were fixed. All 246
  recoveries ended on the right peak with the SHG cavity locked and the DC nulled, 231 strictly
  single-mode and the rest with at most 11 % of the power in a second mode, a median 1.1 mA from
  the next hop; all 54 failures (mostly a target further than the ±25 V of piezo could reach)
  were left unlocked with every setting put back. About 22 s at 10 readings a second (2 s of it
  looking at the cavity transmission, scanning and locked), 33 s at most for nine in ten. With
  nothing learned at all, the same but for at most 12.5 % in a second mode. (The previous
  version, which looked 0.5 mA either side of the operating point: 234 fixed, 239 strictly
  single-mode.) 300 more that also stop lasing 0.7 to 2 mA below their current and start again
  only 1.5 to 4.5 mA above it, a third of them not lasing when the recovery began: 220 of 231
  fixed, 107 jumps of the current in 103 recoveries, none left without lasing, no TA switched
  off; one was handed back with its SHG signal at 0.65 of usual, a second mode whose peaks fell
  on the main ones in the sweep (which its cavity lock then kept losing: watched, it would be
  relocked again). The same 300 again, but sending their TA 70 % of their power as broadband
  light when they do not lase, so that the seed power never tells: 220 of 231 fixed all the
  same, every one of the 103 that stopped lasing caught by its SHG sweep (or the wavemeter
  losing it) and brought back, none left without lasing.
* With the DLC pro's feedforward on: 150 SHG masters as above, their feedforward 0.5 to 3 times
  what keeps a mode aligned as the piezo moves, stopping lasing 0.2 to 1 mA below nominal (and
  starting again only 1.5 to 4 mA above), hopping one mode: 121 fixed, the other 29 out of the
  piezo's reach; 28 stopped lasing once on the way and were brought back, none was left without
  lasing, and the measured current never left its window. Before the feedforward was taken
  into account and the current parked high: 110 fixed, 96 times not lasing in 42 recoveries,
  11 given up as "it stopped lasing 4 times" or "not lasing even at nominal + 5 mA", 2 left
  without lasing.
* The map (with the current ±2 mA and the piezo ±10 V): a Monte Carlo over 3000 random lasers
  (modes 3 to 10 GHz apart, chip modes shifting by 1 to 4 GHz/mA, hysteresis up to 0.15 of the
  mode spacing, noisy readings near a hop) fixed 99.4 % of the hops that could be fixed within
  the windows (2083 of 2095), none off target, restored every other one, and never let a
  current leave its window; about 25 s at 10 readings a second. The same 300 SHG masters as
  above: 180 of the 181 fixable hops were fixed; every one of the 206 recoveries ended on the
  right peak with the SHG cavity locked and the DC nulled, 200 strictly single-mode and the
  other 6 with less than 10 % of the power in a second mode; all 94 failures were left
  unlocked with every setting put back. Mapping with an SHG sweep at every point is slow:
  about a minute.
* Strong hysteresis (0.15 to 0.6 of the mode spacing, more one way than the other), so that
  where a mode lases sweeping up has little to do with where it lases sweeping down: of 300
  random lasers recentered where they were on target, 270 came out on target and clear of a
  hop either way (the previous version, which wanted a mode lasing both ways: 262); of 300
  after a hop, 156 (145); SHG lasers, 151 of 200 (148). On lasers with little hysteresis the
  two do about equally well; the parking sweep makes a recovery some 10 s longer.
* What the simulation cannot settle about a real DLC pro, and how the recovery copes:
  * `select-lockpoint x y type` takes the type as a number. The recovery tries the
    candidate's type character code first, then 0 to 3, checks the lockpoint it got
    (`lockpoint:position`, and its type against yours), and keeps the number that worked.
  * The candidates move with every scan as the free-running laser drifts, so coordinates read
    a moment earlier may no longer name one ("lockpoint not found"). Each try uses candidates
    read just before it, and a "not found" is tried once more with fresh ones.
  * Lockpoint candidates are read while the scan runs, after 3 scan periods, each time after
    `find-candidates` (with its lock points not shown, a DLC pro offers none; the SDK has it as a
    command without arguments, so it is taken to find them once, in the scan running then).
  * Re-enabling a lock that should find its resonance by itself needs `lock-without-lockpoint`;
    it is ticked first, put back if relocking fails, and left ticked once it locked.
  * "Locked" is the lock state 5, or a state text starting with "locked".
  * The FALC pro's input offset moves the error signal straight up and down, so the first
    recovery learns which way it moves, and later ones null the DC in one step. If the trace
    ever did not follow the offset (the scope showing some other signal than the FALC's input),
    the offset is put back and left alone, and the row says so.
  * The wavemeter and the reference cavity get the 739 nm fundamental, picked off directly, so
    the readings do not depend on the SHG cavity's lock, which stays off while the modes are
    mapped.
  * A second mode can hide in the SHG sweep if the master's mode spacing is a whole number of
    SHG cavity free spectral ranges; its main peaks would still shrink, which is caught once
    the single-mode size is known.
* With recovery on, a wavemeter-locked laser is watched while it is locked (*Locked*), a
  cavity-locked one while its row says *Watching* and its cavity lock is on. If you change
  the current by hand, it stands by until the laser is back on target; to work on the laser
  for longer, click *Watching* (or untick *Recover this laser*).
* In `--sim`, channel 7 (739 nm TA-SHG pro) is locked to a cavity by its DLC pro by
  lockpoint, PDH through a FALC pro with a 9 mV DC level on its error signal, and its SHG
  cavity is locked; its measured current is 0.42 mA below the set one (feedforward from the
  piezo). Its master stops lasing 1.3 mA below its current and starts again only 2.4 mA above
  it; its TA switches itself off after 5 s with less than 8 mW of seed. Recovery is on: click
  **Relock** to have it watched. *Right-click empty space ▸ Simulate a fault* makes any laser
  hop a mode, the cavity lock jump a peak (or 60 MHz), the master go multimode (its cavity lock
  then keeps losing it) or stop lasing (also with its seed power hardly dropping: broadband light
  still reaching the TA), or the error signal's DC drift. Like a real DLC pro with its lock points
  not shown, the simulated one offers lockpoint candidates only after `find-candidates`.

## Troubleshooting

* **No controllers in the menu**: discovery replies are UDP, so allow Python
  through the Windows firewall on the lab network (Windows asks on first run).
  Controllers behind a router won't hear the broadcast; use *Enter IP address…*,
  and they are remembered after that.
* **`cannot load wlmData`**: 64-bit Python needs the 64-bit `wlmData.dll`
  (System32), 32-bit Python the 32-bit one (SysWOW64). Point `--dll` at the
  right one if it isn't found.
* **Lock runs away straight after engaging**: the gain sign is wrong for that
  laser; negate P and I.

## Wavemeter server (optional)

`server/server.py` is the FastAPI server you already had, with one fix: it
called `GetUseChannel`, which wlmData does not have. That call always fell
back to "enabled", so every channel was reported. It now reports a channel only
when the switch actually uses it (`GetSwitcherSignalStates`). Same endpoint,
same JSON, so other clients keep working. Run it on the wavemeter PC next to
HighFinesse's `wlmData.py`/`wlmConst.py`:

```
pip install fastapi uvicorn
uvicorn server:app --host 0.0.0.0 --port 8000
```

## Layout

```
wmlock/wavemeter.py  wavemeter sources: wlmData.dll (ctypes), HTTP, push changes only
wmlock/dlcpro.py     DLC pro client (DeCoP command line, TCP 1998) + UDP discovery (60010)
wmlock/lock.py       the PI lock, pure maths
wmlock/reference.py  wavemeter referencing to a cavity-locked laser, pure maths
wmlock/modehop.py    mode-hop recovery: lasing watch and panic, the walk, the current map, driver; pure logic
wmlock/relock.py     taking over and handing back a DLC pro's cavity and SHG locks; setup check
wmlock/cavity.py     reading SHG sweeps, cavity scans and error signals, pure maths
wmlock/engine.py     threads and state: source → lock → per-controller workers
wmlock/gui.py        Qt window
wmlock/sim.py        simulated wavemeter, mode-hopping lasers (and TAs), fake DLC pros (demo and tests)
server/server.py     optional HTTP server for the wavemeter PC
```

The DLC pro protocol and discovery handshake follow TOPTICA's own Python SDK
(`toptica.lasersdk`), but the SDK is not needed.

## Tests

```
pip install pytest fastapi httpx
python -m pytest
```

The tests run the real discovery, protocol, lock and reference code against the
fake controllers, and the GUI headless. The reference is tested on synthetic
mode hops, cavity unlocks, slow runaways, dark periods, wavemeter steps and
restarts. An end-to-end test checks that a locked laser's *true* frequency (known
to the simulator) stays put while the wavemeter drifts and the reference hops,
and moves when referencing is turned off. Mode-hop recovery runs against simulated
external-cavity diode lasers, synchronously (a Monte Carlo over random lasers, checking the
safety rules on every write) and end to end through fake controllers, for wavemeter-locked
and cavity-locked lasers (relocked after a hop, on the wrong peak, or with a lock that keeps
losing it). The simulated lasers have hysteresis (up to 0.6 of the mode spacing,
more one way than the other), follow every write of current and piezo as a real laser does,
and run on two modes just before they hop; they can stop lasing below a current and start again
only well above it. The fake controllers also simulate a TA-SHG pro: its master's mode
content, its TA's seed photodiode and seed protection, the SHG cavity's sweep and lock, the
reference cavity's transmission and PDH error signal with a DC level, a lock that cannot hold a
master on two modes (ReLock catching it again and again), the FALC pro's input
offset, lockpoint candidates and selection; the synchronous tests drive them through the real
client and protocol, without sockets.
