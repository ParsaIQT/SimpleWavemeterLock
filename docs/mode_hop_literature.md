# Mode hops of an ECDL seen coming, and avoided: what is published

*A survey made in November 2026 for the keeping of a laser off the edge of its mode (README, *Kept off the edge
of its mode*) and the ramps. What was opened and read is marked so; the rest is from abstracts.*

## Hop avoidance under a frequency lock

* **Chiow, Long, Vo, Müller, Chu**, *Extended-cavity diode lasers with tracked resonances*, Appl. Opt. 46(33),
  7997 (2007), arXiv:0710.0636 (read). The piezo keeps the laser on its reference; the current is dithered 0.01
  to 0.1 mA at about 10 Hz, the photodiode's noise level detected synchronously, and an integrator (about 10 s)
  acts on the current (and the temperature). An uncoated diode: under half an hour without a hop became 48 h; an
  AR-coated one: a few hours became 12 days. The noise rises at **both** ends of the hop-free band (without the
  dither it has no sign). Their current corrections stayed within about −0.7 to +0.4 mA over 48 h, only partly
  correlated with the piezo and the baseplate's temperature: a feed-forward alone would not have done.
* **Führer**, PhD thesis, TU Darmstadt 2012 (https://d-nb.info/1106117069/34, German; read), sec. 3.2.5: a
  HighFinesse WS/7's PID on the piezo (about 40 Hz) around an inner loop on the current. The inner error signal
  needs a quarter-wave plate inside the cavity (**Führer, Stang, Walther**, Opt. Express 17, 4991, 2009; 130 GHz
  of tuning with the temperature and the piezo). **Führer, Walther**, Opt. Lett. 33, 372 (2008) is not a live
  loop: the hops of one scan reshape the current's ramp for the next.
* **TOPTICA**: the DLC pro's own options do it inside the controller, for lasers that have them. `laser1:dl:smc:*`
  (DL pro SMC / SAFE: a detection unit in the head and a CC-500 SMC; it finds the plateau and says "mode-hop
  likely", 0.01 to 10 Hz), and `laser1:nlo:auto-nlo:perform-single-mode-optimization` with
  `auto-nlo:master-noise` (the AutoNLO licence; one of its algorithms uses a wavemeter). The firmware's
  documents are in https://www.toptica.com/downloads/software/dlc-pro/DLCpro_v3.5.4.zip (Command Reference,
  Manual M-063). Its pressure compensation (manual 11.12) moves the piezo only, with no current feed-forward.

## What a hop looks like before it happens

* The side mode appears only in the last percent or so of the loss margin and grows hyperbolically there; the
  two-mode zones measured are 5 to 20 % of a hop period, wider on the high-current side (**Takamizawa**,
  arXiv:2309.07393, 2024, read: side modes 10.7 dB below the carrier at their strongest, 20 dB below a few mA
  either side; an uncoated diode's hysteresis 0.54 mA of a 2.76 mA hop period).
* The stable range is not symmetric: it reaches further to the red and grows with power (**Detoma, Tromborg,
  Montrosset**, IEEE J. Quantum Electron. 41, 2005: four-wave mixing between the modes; read).
* A wavemeter's fringes with a second mode a fraction ε of the power, Δ away: their contrast falls by about
  2 ε sin²(π Δ / FSR) in an interferometer of that free spectral range: nothing where Δ is a whole number of
  FSRs. Hence the least over all fringe systems (`keep.Fringes`). HighFinesse's fine and wide FSRs: WS6 16 and
  100 GHz, WS7-60 8 and 32, WS7-30 4 and 32, WS8 2 and 20.
* With an SHG stage the locked SHG cavity rejects the side mode: its intra-cavity signal falls, its rejection
  photodiode (`nlo:pd:shg-pdh-dc:photodiode`) rises.

## The model

* The static rule (the mode of least loss lases) is the standard one: **Saliba, Junker, Turner, Scholten**,
  *Mode stability of external cavity diode lasers*, Appl. Opt. 48(35), 6692 (2009) (read): gain, grating, chip
  etalon and external cavity as one product; chip mode about −30 GHz/K, current about −3 GHz/mA at low frequency.
* A threshold before the running mode gives way: Takamizawa's α_max ≈ 0.02 of the transmittance, taken from the
  hysteresis he measured (as `h` here from a map), with the remark that a theory of an ECDL's hysteresis is
  still wanted.

## Not found in print

The SHG cavity's signals or the wavemeter's interferogram used as a hop's warning; a hop read as a measurement
of where the chip's modes are; the chip's thermal lag as the cause of hysteresis (a test: does it hop at the
same current whatever the ramp's rate?); how long before a hop a second mode shows, in seconds, for a drifting
laser; machine-learnt hop prediction.

## Worth trying on the lab's PCs

* wlmData's `GetMultimodeInfo` (and the `cmiMultimodeLevel`, `cmiMultimodeCount` events): the wavemeter's own
  verdict. What they mean is in HighFinesse's manual only.
* The DLC pro's `dl:cc:output-filter:slew-rate` (mA/s; `slew-rate-enabled`): a ramp made by the controller
  itself, between two writes.
* Whether `laser1:dl:smc:*` or `laser1:nlo:auto-nlo:*` exist on the lab's controllers.
* The DLC pro's recorder, for a keeping: signals 82 (SHG intra-cavity), 81 (SHG rejection), 51 (current), up to
  about 100 kHz.
