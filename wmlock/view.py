"""What a channel's row shows, Qt-free: the window paints it, and share.py sends it to the other PCs, which paint
it the same way.

row(engine, n, now) is a dict:
  n, name
  f         THz in the big digits (the reading; on the reference laser's row its raw reading), None if no valid one...
  bad       ...then this instead ('off', 'no signal', ...), in colour bad_c
  age       s since that reading (greyed after STALE_S)
  ref_unit  the reading is corrected by the reference laser (its unit in the accent colour)
  dev       MHz in the deviation column (from the setpoint; the reference's offset), None for none; colour dev_c
  sp        the setpoint, nm (0: none); None on the reference laser's row, which shows anchor (nm, 0: none) instead
  anchor    see sp: None on any other row
  info      the line under the digits, in colour info_c
  prompt    step by step: what the recovery waits to be allowed ('' if nothing)
  state     the look of the row and its button: off, error, on, engaging, recovering, asking, ref, ref-hold, ref-moved
  relock    the button relocks a laser its DLC pro locks to a cavity (Relock, Watching), else Lock (Locked)
Colours are names: fg, dim, ok, warn, err, accent. tip(engine, n, digits) is the tooltip of its digits.
"""
from __future__ import annotations

import math
import time

from .engine import Engine
from .reference import ACQUIRING, HOLD, MOVED, TRACKING, fmt_duration
from .relock import volts
from .wavemeter import C_NM_THZ

STALE_S = 3.0  # a channel without a new reading for this long is greyed out
NOTE_S = 1800.0  # how long a row mentions its last mode-hop recovery
REMOTE_S = 600.0  # ...and an adjustment made from another PC


def fmt_nm(nm: float, digits: int) -> str:
    """Wavelength with `digits` significant digits."""
    return f"{nm:.{decimals(nm, digits)}f}"


def decimals(nm: float, digits: int) -> int:
    return max(digits - len(str(int(nm))), 0)


def fmt_mhz(x: float) -> str:
    return f"{x / 1e3:+.3f} GHz" if abs(x) >= 1e4 else f"{x:+.1f} MHz"


def row(e, n: int, now: float) -> dict:
    """What channel n's row shows now (now: time.monotonic())."""
    ch = e.channels[n]
    ref = e.reference
    v = {"n": n, "name": ch.name, "age": now - ch.t}
    if ref is not None and ref.channel == n:
        return _reference(ch, ref, now, v)
    valid = ch.freq > 0
    v["f"] = ch.freq if valid else None
    if valid:
        v["bad"], v["bad_c"] = "", "fg"
    elif ch.off:
        v["bad"], v["bad_c"] = "off", "dim"  # (its wavemeter channel: the info line says so)
    else:
        v["bad"], v["bad_c"] = ch.status or "—", "dim" if ch.borrowed else "err"
    v["ref_unit"] = ref is not None and ref.active
    state = ch.state
    if valid and ch.setpoint_nm:
        v["dev"], v["dev_c"] = ch.error_mhz, {"on": "ok", "engaging": "warn", "recovering": "warn"}.get(state, "fg")
    else:
        v["dev"], v["dev_c"] = None, "fg"
    v["sp"], v["anchor"] = ch.setpoint_nm or 0.0, None
    v["info"], v["info_c"], v["prompt"] = _info(e, ch, now)
    if state == "recovering" and ch.hop_prompt:
        state = "asking"
    v["state"] = state if state != "off" else ("error" if ch.message else "off")
    v["relock"] = ch.relock
    return v


def _reference(ch, ref, now: float, v: dict) -> dict:
    """The reference laser's row: its raw reading, where it should be, and how it is doing."""
    v["f"] = ch.raw if ch.raw > 0 else None
    v["bad"], v["bad_c"] = ("", "fg") if ch.raw > 0 else (ch.status or "—", "err")
    v["ref_unit"] = False
    state = ref.state
    color = {TRACKING: "dim", MOVED: "err"}.get(state, "warn")
    v["dev"] = ref.offset if state != ACQUIRING and math.isfinite(ref.offset) else None
    v["dev_c"] = "fg" if state == TRACKING else color
    v["sp"], v["anchor"] = None, C_NM_THZ / ref.anchor if ref.anchor else 0.0
    if state == TRACKING:
        text = f"reference · wavemeter {ref.error:+.2f} MHz, drifting {ref.rate * 60:+.2f} MHz/min"
    elif state == HOLD:
        text = f"hold {fmt_duration(now - ref.since)}: {ref.message}"
    elif state == MOVED:
        text = f"{ref.message} · right-click"
    else:
        text = ref.message
    v["info"], v["info_c"], v["prompt"] = text, color, ""
    v["state"] = {TRACKING: "ref", MOVED: "ref-moved"}.get(state, "ref-hold")
    v["relock"] = False
    if ch.state == "recovering":
        v["info"], v["info_c"], v["prompt"] = _recovery(ch, now)
        v["state"] = "asking" if ch.hop_prompt else "recovering"
    return v


def _recovery(ch, now: float):
    prompt = ch.hop_prompt
    if prompt:
        return f"next: {prompt}   (Continue, or right-click to stop)", "accent", prompt
    parts = [ch.hop_why, ch.hop_stage]
    if ch.current is not None:
        parts.append(f"{ch.current:.2f} mA")
    parts.append(fmt_duration(now - ch.hop_t))
    return " · ".join(p for p in parts if p), "warn", ""


def _info(e, ch, now: float):
    """(text, colour, prompt) of the line under the digits."""
    if ch.state == "recovering":
        return _recovery(ch, now)
    if ch.message:
        return ch.message, "err", ""
    if not ch.device:
        return "right-click to link a laser" + remote_text(ch, now), "dim", ""
    dev = e.devices.get(ch.device)
    ctrl = e.controllers.get(ch.device)
    where = f"laser{ch.laser} @ {dev.name if dev else ch.device}"
    more = hop_text(ch, now) + remote_text(ch, now)
    if ctrl is not None and ctrl.online is False:
        return f"{where}   offline", "err", ""
    if ch.lasing is False and ch.hop.enabled:
        return f"{where}   NOT LASING: seed {ch.seed:.2f} mW{more}", "err", ""
    if slipping(ch):
        return f"{where}   CAVITY LOCK SLIPPING{more}", "warn", ""
    if shg_gone(ch):
        return f"{where}   SHG SIGNAL GONE{more}", "warn", ""
    if ch.look_why:
        return f"{where}   looking, its wavemeter channel switched on for it: {ch.look_why}{more}", "warn", ""
    if ch.off and (ch.state in ("on", "engaging") or ch.relock):
        if ch.relock:
            what = "watched by what its DLC pro says" if ch.state == "on" else \
                "Relock has it watched (and a look, the channel switched on for it)"
        else:
            what = f"lock held at {ch.offset:.3f} {ch.unit}" if ch.offset is not None else "lock held"
        return f"{where}   wavemeter channel off: {what}{more}", "dim", ""
    off = "…" if ch.offset is None else f"{ch.offset:.3f} {ch.unit}"
    return f"{where}   {off}{more}", "dim", ""


def remote_text(ch, now: float) -> str:
    """The last adjustment made from another PC, for a while."""
    if ch.remote_note and now - ch.remote_t < REMOTE_S:
        return f"   ⇄ {ch.remote_note}, {fmt_duration(now - ch.remote_t)} ago"
    return ""


def hop_text(ch, now: float) -> str:
    """The current, the DLC pro lock and the last recovery, for a laser with mode-hop recovery on."""
    if not ch.hop.enabled:
        return ""
    text = f"   {ch.current:.2f} mA" if ch.current is not None else ""
    if ch.seed is not None:
        text += f"   seed {ch.seed:.1f} mW"
    if ch.hop.cavity:
        text += f"   cavity {ch.cavity_txt or ('on' if ch.cavity_on else 'off')}"
        lv = Engine.trans_levels(ch)
        if lv is not None:
            level, usual, _, limit = lv
            text += f" · transmission {volts(level, usual)} V"
            if usual:
                text += f" (locked {volts(usual, usual)}" + (f", relock below {volts(limit, usual)}"
                                                             if slipping(ch) else "") + ")"
    if ch.hop.shg and ch.shg_txt:
        text += f"   SHG {ch.shg_txt}"
        lv = Engine.shg_levels(ch)
        if lv is not None:
            level, usual, _ = lv
            text += f" · {volts(level, usual)} V" + (f" (locked {volts(usual, usual)})" if usual else "")
    if ch.note and now - ch.note_t < NOTE_S:
        text += f"   ⟲ {fmt_duration(now - ch.note_t)} ago"
    if ch.looked and now - ch.looked_t < NOTE_S and ch.looked_t > ch.note_t:
        text += f"   looked {fmt_duration(now - ch.looked_t)} ago"
    if ch.drops:
        wall = time.time()
        k = sum(1 for t, _ in ch.drops if wall - t <= 3600.0)
        if k:
            text += f"   {k} lock dropout{'s' if k != 1 else ''} in the last hour"
    return text


def shg_gone(ch) -> bool:
    """Its SHG signal, armed, far below its level locked (it would be recovered: not lasing, or the SHG cavity
    lost)."""
    lv = Engine.shg_levels(ch)
    return bool(ch.hop.enabled and ch.hop.shg and lv is not None and lv[2] is not None and len(ch.shg_hist) >= 5
                and lv[0] < lv[2])


def slipping(ch) -> bool:
    """Its cavity lock on, but its transmission below its limit on average (armed, it would be relocked: the lock
    keeps losing it)."""
    lv = Engine.trans_levels(ch)
    return bool(ch.hop.enabled and ch.hop.cavity and ch.cavity_on and lv is not None and lv[3] is not None
                and len(ch.trans_hist) >= 5 and lv[0] < lv[3])


def tip(e, n: int, digits: int = 9) -> str:
    """The tooltip of channel n's digits: its reading in full, and what its row has no room for."""
    ch = e.channels[n]
    ref = e.reference
    referenced = ref is not None and ref.channel != n and ref.active and ch.raw > 0 and ch.freq > 0
    return head(n, ch.freq if ch.freq > 0 else None, digits, ref.channel if referenced else None,
                (ch.freq - ch.raw) * 1e6 if referenced else None) + notes(e, n)


def head(n: int, f, digits: int = 9, refch=None, corr=None) -> str:
    """The first lines of a tooltip: the reading f (THz, None: none), and, corrected by the reference laser on channel
    refch by corr (MHz), what the wavemeter read."""
    text = f"channel {n}"
    if f:
        text += f"\n{f:.9g} THz\n{C_NM_THZ / f:.{digits}g} nm (vacuum)"
        if refch is not None and corr is not None:
            text += (f"\nreferenced to channel {refch}: the wavemeter reads "
                     f"{C_NM_THZ / (f - corr * 1e-6):.{digits}g} nm, corrected {corr:+.1f} MHz")
    return text


def notes(e, n: int) -> str:
    """The rest of a tooltip: what does not change with every reading."""
    ch = e.channels[n]
    ref = e.reference
    text = ""
    if ref is not None and ref.channel == n:
        text += f"\nreference laser: {ref.state}" + (f", {ref.message}" if ref.message else "")
    if ch.hop.enabled:
        hop = ch.hop
        text += (f"\nmode-hop recovery on: {hop.i_nom - hop.i_below:.2f} … {hop.i_nom + hop.i_window:.2f} mA "
                 f"(measured), by {hop.how()}"
                 + (", takes over the DLC pro's cavity lock" if hop.cavity else "")
                 + (", checks the mode in the SHG cavity" if hop.shg else "")
                 + (", step by step" if hop.confirm else ""))
    if ch.off:
        text += "\nits wavemeter channel is switched off ('Use' unticked in the WLM software)"
    if ch.note:
        text += f"\nlast recovery: {ch.note}"
    if ch.looked:
        text += f"\nlast look, its wavemeter channel switched on for it: {ch.looked}"
    if ch.drops_total:
        text += f"\nlock dropouts: {ch.drops_total} since the program started; the last at {ch.drop_last}"
    if ch.remote_note:
        at = time.strftime("%H:%M:%S", time.localtime(ch.remote_wall))
        text += f"\nlast adjusted from another PC: {ch.remote_note}, at {at}"
    return text
