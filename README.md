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
* **Lock** runs a simple PI loop that only touches `laserN:scan:offset`.
* **Reference laser** (optional): a laser locked to a stable cavity corrects the
  wavemeter's drift for every other channel, and is ignored automatically while it
  mode-hops, unlocks or loses light.
* **Mode-hop recovery** (optional): after a mode hop, the laser is put back on its
  target on a mode that holds, by moving its diode current as well as its piezo, within
  a current window you set, and handed back to its lock: this program's wavemeter lock,
  or the DLC pro's own cavity lock (PDH / FALC), relocked by lockpoint on the right peak
  as you would by hand. For an SHG laser, its SHG cavity shows whether the master is
  single-mode, and is relocked too.
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
GHz. Pulling the frequency back with the piezo alone, as a lock does, leaves the laser at
the very edge of its mode, where it soon hops again. The current moves the filter
quickly (the chip modes shift by GHz per mA) and the lasing frequency only slowly, so a
small current change re-centers the filter on a mode that reaches the target, and a
re-centered mode holds.

![mode-hop recovery](docs/modehop.png)

Right-click a channel ▸ **Mode-hop recovery…**:

* **Locked by**: *this program, to the wavemeter*, or *its DLC pro, to a cavity*
  (a DLC pro lock with PDH and FALC, say).
* **It has an SHG stage**: for a TA-SHG / DL-SHG pro. Its SHG cavity is used to see how
  single-mode the master is, and is relocked afterwards (see *SHG lasers* below).
* **Nominal current**: the laser's normal current, read from the controller when you
  enable recovery. The current always stays within **± 2 mA** of it (you set the
  window), and never above the controller's own `current-clip`.
* A **hop** is the laser off its target by more than 100 MHz for 2 s, after it has been on
  target. The target is the setpoint (`set`). For a cavity-locked laser, type the
  wavelength it has when locked on the right peak (or *Setpoint = current reading* while it
  is), and leave it unlocked by this program. The target follows the wavemeter's slow
  drift for such a laser. With a reference laser, the corrected reading is used. On the
  reference laser itself, the target is where the reference says it should read.
* **Relock** (cavity only): *by lockpoint*, as you would by hand, or *re-enable the lock*
  for a lock that finds its resonance by itself (*lock without lockpoint*). The default
  follows the DLC pro's `lock-without-lockpoint` setting.
* **The right peak is within ± 150 MHz** of where the wavemeter puts the setpoint.
* **Null the error signal's DC level with the FALC pro's input offset, by at most 0.05 V.**
* **Step by step**: before each step, the recovery waits for you (see *Before the first run*).

What happens after a hop (the row turns amber and shows each step; it takes 10 to 60 s,
depending on how often the wavemeter reads that channel, about a minute with an SHG stage):

1. **Take over.** A wavemeter lock is suspended. A cavity lock is disabled
   (`laserN:dl:lock:lock-enabled`), so is the SHG cavity's lock, and if that leaves the
   DLC pro's scan running it is stopped too. How the lock is set up is read first (lock
   input, error signal, FALC pro, the lockpoint you selected, scope channels).
2. **Quick check.** If the laser is close to its target (the cavity lock had caught a wrong
   peak, say), the piezo tunes it back, and steps of the current check that it is clear of
   the next hop. If so, go to 6.
3. **Map.** The current is swept across its window (down, up, down) at fixed piezo, one
   reading per 0.1 mA step. The readings fall into straight lines, one per mode, broken
   by hops. A mode's stable range is where it lases in both sweep directions; outside it,
   hysteresis decides which of two modes you get. With an SHG stage, its cavity is scanned
   meanwhile, and wherever the sweep shows a second mode is cut out of the stable ranges.
4. **Choose.** The current is set to the middle of a stable range: the one that is wide
   (up to 1 mA either side counts) and needs the least piezo to reach the target. A range
   that ends at the window's edge is taken to go on a little beyond it. The piezo tuning
   (MHz/V) is measured on the way.
5. **Tune and check.** The piezo brings the laser onto the target, watching for hops. Then
   the current is stepped ±0.5 mA (*Keep a hop at least …*) around the operating point: the
   frequency must stay on one straight line and come back, and with an SHG stage the laser
   must be single-mode at both ends and in the middle. If not, or if the piezo moved a lot
   (the modes move with it), it maps again there, up to three times.
6. **Hand back.** The wavemeter lock is re-engaged at the setpoint. For a cavity lock, see
   below; the wavemeter must show the right peak (within half the tolerance) for 5 readings
   in a row, or it unlocks and tries again, up to three times.

### SHG lasers, relocked by lockpoint

For a TA-SHG pro whose master its DLC pro locks to a reference cavity (lockpoint picked in
the XY display, PDH through a FALC pro) and whose SHG cavity it locks to the master, the
recovery does what you would do by hand:

| By hand | Here |
|---|---|
| Unlock the master and the SHG, stop the master's scan | step 1 |
| Scan the SHG cavity, look at its intra-cavity signal: is the master single-mode? | throughout steps 2 to 5: every map point, both ends of the margin check, and the operating point |
| Lock the SHG cavity | SHG scan off, `nlo:shg:lock:lock-enabled` on; the intra-cavity signal (`nlo:pd:shg-int:photodiode`) must come back to at least half of what it was, locked, when the recovery began |
| Start the master's scan, bring the right cavity peak into it, pick it in the XY display (scan output against Fine In 1) | the scan runs; from the DLC pro's lockpoint candidates (`dl:lock:candidates`), the tall peak where the wavemeter puts the target (scan offset − deviation / piezo tuning), within ± 150 MHz, and the only one there; it must stand at least half as tall as the right peak did last time (from the lockpoint you selected), so a transverse mode is never taken for it |
| Correct the FALC's input offset (~0.01 V) so the error signal has no DC | the error signal's level away from the resonances, from the scope; the input offset steps against it, checking on the trace after each step that it does what it should |
| Lock | `dl:lock:select-lockpoint` on that candidate, then `lock-enabled`; the lock must report locked, the wavemeter must agree it is the right peak, and the transmission must reach half the peak's height |

How a sweep is judged: a single-mode master gives one tall peak per free spectral range of
the SHG cavity, plus small transverse-mode bumps (always the same); a second mode adds its
own family of peaks and takes power from the main ones. A peak is measured by its area, which
does not depend on where the samples fall on a narrow resonance. What single-mode looks like
on your laser is learned from the map itself (where the main peaks are biggest) and kept. A
sweep is called multimode if its biggest side peak is more than 10 % (of a main peak) above
the usual, or the main peaks are below 80 % of their usual size (which also catches two
modes of equal power: twice the peaks, each half as big). A second mode with less than about
10 % of the power passes; you would hardly see it by eye either.

![last recovery](docs/last_recovery.png)

*Right-click ▸ Last recovery…* shows what it saw and decided: the steps, each map (modes,
stable ranges, where the SHG sweep showed two modes), SHG sweeps (a single-mode and a
multimode example from the map, both ends of the check, the operating point), the cavity scan
with the candidates, where the target should be and the peak picked, and the error signal
before and after nulling its DC. Every recovery is also appended to
`~/.simplewavemeterlock-recoveries.jsonl`.

### Before the first run on the real laser

![check setup](docs/setup_check.png)

1. *Right-click ▸ Check setup…* reads (only reads) everything a recovery would use and says
   what looks wrong: the current and its window, the scan, the lock's state, whether it relocks
   by lockpoint, **the lockpoint you selected by hand** (its type and level are what the
   recovery looks for), the lock input (should be Fine In 1) and the error-signal channel, the
   FALC pro and its input offset, the scope channels, the SHG lock, scan and scope. Nothing is
   written; read it with the laser locked as you like it.
2. Set the setpoint while it is locked on the right peak (*Setpoint = current reading*).
3. Tick **Step by step** and press *Recenter now*. The row turns blue and says what it is
   about to do (*next: …*); **Continue** does it, *right-click ▸ Stop recentering* puts
   everything back. Watch each step on the DLC pro: the SHG sweep, the XY display, the
   lockpoint it selects, the FALC offset.
4. Then *Last recovery…*: do you agree with its verdicts on the SHG sweeps, with the peak it
   picked, with the DC level it saw?
5. Untick *Step by step* once you trust it.

Safety:

* The current never leaves nominal ± window and never changes by more than 0.2 mA per
  write. Larger moves are ramped. Each write is checked once more just before it is sent,
  whatever the procedure asked for.
* The piezo moves at most ± 10 V per recovery (*Piezo moves at most*), inside the lock's
  offset limits.
* Besides current and offset, a recovery writes only: the cavity lock and SHG lock on/off, the
  scan and SHG scan on/off, a scope channel's signal (given back), a FALC pro's input offset
  (within its limit of where it was), and runs `select-lockpoint`. Anything else is refused.
* If nothing works, or after 5 minutes, everything it changed goes back where it was and
  the cavity lock stays off, with the reason on the row. Stopping it does the same. If it
  cannot tell which cavity peak is right, or the DLC pro will not take the lockpoint, the
  laser is left at its target, unlocked and scanning, for you to pick the peak. If the lock
  caught a wrong peak three times, the laser is left on its new, good operating point with
  the lock off.
* At most 4 recoveries an hour; beyond that it stops and says so. It also does nothing
  until the laser has been on target again.
* If the controller stops answering, it stops and says what the current was left at.
* A recovery only starts from a current within the window. Set the nominal current again
  after changing it by hand.

*Recenter now* runs steps 1 and 3 to 6 at any time, without a hop: it re-centers a laser
you suspect sits near the edge of its mode. It needs a setpoint and a valid reading.

Recoveries learn the laser's piezo tuning and mode spacing, the right peak's height, the
lockpoint type and the number `select-lockpoint` wants for it, what single-mode looks like in
the SHG cavity, and how the FALC offset shows on the error signal. They are kept (the dialog
shows them, *Forget* clears them) and make later recoveries quicker and surer.

Things to know:

* This has been tested against simulated lasers and a simulated DLC pro only. Mode-hop maps:
  a Monte Carlo over random lasers (modes 3 to 10 GHz apart, chip modes shifting by 1 to
  4 GHz/mA, hysteresis, noisy readings near a hop) fixed 99.6 % of the hops that could be
  fixed within the windows, restored every other one, and never let a current leave its
  window. SHG lasers: 300 random masters that go multimode over 15 % or more of each mode,
  quietly for the wavemeter, with a random DC level of up to ±30 mV on the error signal: 180 of
  the 181 fixable hops were fixed; every one of the 202 recoveries ended on the right peak with
  the SHG cavity locked and the DC nulled, 198 strictly single-mode and the other 4 with less
  than 10 % of the power in a second mode; all 98 failures were left unlocked with every
  setting put back. Waiting for fresh scope traces after each step makes an SHG recovery
  slower: about a minute.
* What the simulation cannot settle about a real DLC pro, and how the recovery copes:
  * `select-lockpoint x y type` takes the type as a number. The recovery tries the
    candidate's type character code first, then 0 to 3, checks the lockpoint it got
    (`lockpoint:position`, and its type against yours), and keeps the number that worked.
  * The candidates move with every scan as the free-running laser drifts, so coordinates read
    a moment earlier may no longer name one ("lockpoint not found"). Each try uses candidates
    read just before it, and a "not found" is tried once more with fresh ones.
  * Lockpoint candidates are read while the scan runs, after 3 scan periods.
  * "Locked" is the lock state 5, or a state text starting with "locked".
  * Whether the error signal on the scope includes the FALC's input offset: checked each time
    (if the trace does not follow the offset, the offset is put back and left alone).
  * The wavemeter is assumed to read the fundamental (the master), which does not need the
    SHG locked.
  * A second mode can hide in the SHG sweep if the master's mode spacing is a whole number of
    SHG cavity free spectral ranges; its main peaks would still shrink, which is caught once
    the single-mode size is known.
* While recovery is on, it acts whenever the laser is off target with its lock on. If you
  change the current by hand, it stands by until the laser is back on target; to work on
  the laser for longer, untick *Recover this laser*.
* In `--sim`, channel 7 (739 nm TA-SHG pro) is locked to a cavity by its DLC pro by
  lockpoint, PDH through a FALC pro with a 9 mV DC level on its error signal, and its SHG
  cavity is locked; recovery is on. *Right-click empty space ▸ Simulate a fault* makes any
  laser hop a mode, the cavity lock jump a peak, the master go multimode, or the error
  signal's DC drift.

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
wmlock/modehop.py    mode-hop recovery: current map, mode choice, recentering, driver; pure logic
wmlock/relock.py     taking over and handing back a DLC pro's cavity and SHG locks; setup check
wmlock/cavity.py     reading SHG sweeps, cavity scans and error signals, pure maths
wmlock/engine.py     threads and state: source → lock → per-controller workers
wmlock/gui.py        Qt window
wmlock/sim.py        simulated wavemeter, mode-hopping lasers, fake DLC pros (demo and tests)
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
and cavity-locked lasers. The fake controllers also simulate a TA-SHG pro: its master's mode
content, the SHG cavity's sweep and lock, the reference cavity's transmission and PDH error
signal with a DC level, the FALC pro's input offset, lockpoint candidates and selection; the
synchronous tests drive them through the real client and protocol, without sockets.
