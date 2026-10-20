"""Minimal Qt front-end.

Everything slow lives in the engine's threads. This module repaints changed
numbers on a 30 Hz timer and forwards clicks, so the window never stalls.

With other PCs about (share.py), each wavemeter's rows are framed under its
name: this PC's first, then the others', painted from what those PCs send (the
same view.py that paints the rows here) and adjusted by asking them.
"""
from __future__ import annotations

import functools
import math
import signal
import sys
import time
import types
import zlib

from . import dac, view
from .engine import DAC_LOCK, Engine
from .lock import LockParams
from .modehop import HopParams
from .qt import Qt, QtCore, QtGui, QtWidgets
from .reference import ACQUIRING, HOLD, MOVED, TRACKING, RefParams, fmt_duration
from .relock import usable_zero, volts
from .view import NOTE_S, STALE_S, decimals, fmt_mhz, fmt_nm  # noqa: F401  (NOTE_S, STALE_S: as before)
from .wavemeter import C_NM_THZ

REFRESH_MS = 33

C = {
    "bg": "#16171a", "row": "#1f2125", "line": "#2d3036", "fg": "#e8e9eb", "dim": "#80858f",
    "ok": "#38c172", "warn": "#f2b33d", "err": "#ff5a5a", "accent": "#4ea1ff",
}

QSS = f"""
QFrame#row {{ background: {C['row']}; border-radius: 6px; border-left: 4px solid {C['row']}; }}
QFrame#row[state="on"] {{ border-left-color: {C['ok']}; }}
QFrame#row[state="engaging"] {{ border-left-color: {C['warn']}; }}
QFrame#row[state="error"] {{ border-left-color: {C['err']}; }}
QFrame#row[state="ref"] {{ border-left-color: {C['accent']}; }}
QFrame#row[state="ref-hold"] {{ border-left-color: {C['warn']}; }}
QFrame#row[state="ref-moved"] {{ border-left-color: {C['err']}; }}
QFrame#row[state="recovering"] {{ border-left-color: {C['warn']}; }}
QFrame#row[state="asking"] {{ border-left-color: {C['accent']}; }}
QPushButton#lock {{ min-width: 78px; min-height: 30px; border-radius: 5px; font-weight: 600;
    border: 1px solid {C['line']}; background: #2a2d33; }}
QPushButton#lock:hover {{ border-color: {C['accent']}; }}
QPushButton#lock[state="on"] {{ background: #1d6b3e; border-color: {C['ok']}; }}
QPushButton#lock[state="engaging"] {{ background: #6b5418; border-color: {C['warn']}; }}
QPushButton#lock[state="ref"] {{ border-color: {C['accent']}; color: {C['accent']}; }}
QPushButton#lock[state="ref-hold"] {{ border-color: {C['warn']}; color: {C['warn']}; }}
QPushButton#lock[state="ref-moved"] {{ border-color: {C['err']}; color: {C['err']}; }}
QPushButton#lock[state="recovering"] {{ background: #6b5418; border-color: {C['warn']}; }}
QPushButton#lock[state="asking"] {{ background: #1d4f80; border-color: {C['accent']}; }}
QMenu {{ background: #24262b; border: 1px solid {C['line']}; padding: 4px; }}
QMenu::item {{ padding: 4px 22px 4px 18px; }}
QMenu::item:selected {{ background: #363940; }}
QMenu::item:disabled {{ color: {C['dim']}; }}
QMenu::separator {{ height: 1px; background: {C['line']}; margin: 4px 8px; }}
QPushButton#lock:disabled {{ color: {C['dim']}; }}
"""
PANEL_COLORS = ("#b38cff", "#3cc8c8", "#ff7eb6", "#ff9f5a", "#d9c38c", "#c0c4cc")  # (none a state's colour)


def panel_colors(labels) -> dict:
    """Each wavemeter's colour, by its name: the same on every PC that sees the same wavemeters, and no two alike
    while there are colours enough. In the order of their names, each takes the colour its name hashes to, or
    the next one not taken."""
    out, taken, k = {}, set(), len(PANEL_COLORS)
    for label in sorted(set(labels)):
        first = zlib.crc32(label.encode()) % k
        color = next((PANEL_COLORS[(first + j) % k] for j in range(k) if PANEL_COLORS[(first + j) % k] not in taken),
                     PANEL_COLORS[first])
        taken.add(color)
        out[label] = color
    return out


def dark_palette() -> QtGui.QPalette:
    p = QtGui.QPalette()
    R, G = QtGui.QPalette.ColorRole, QtGui.QPalette.ColorGroup
    for role, color in (
        (R.Window, C["bg"]), (R.WindowText, C["fg"]), (R.Base, "#202226"), (R.AlternateBase, C["row"]),
        (R.Text, C["fg"]), (R.Button, "#2a2d33"), (R.ButtonText, C["fg"]), (R.Highlight, C["accent"]),
        (R.HighlightedText, "#ffffff"), (R.ToolTipBase, "#24262b"), (R.ToolTipText, C["fg"]),
    ):
        p.setColor(role, QtGui.QColor(color))
    for role in (R.WindowText, R.Text, R.ButtonText):
        p.setColor(G.Disabled, role, QtGui.QColor(C["dim"]))
    return p


def mono_font() -> QtGui.QFont:
    """A real monospace font, so digits never change width."""
    for family in ("Consolas", "Menlo", "DejaVu Sans Mono", "Liberation Mono"):
        f = QtGui.QFont(family)
        if QtGui.QFontInfo(f).family().lower() == family.lower():
            return f
    f = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
    f.setStyleHint(QtGui.QFont.StyleHint.TypeWriter)
    return f


def event_pos(ev) -> QtCore.QPoint:
    return ev.position().toPoint() if hasattr(ev, "position") else ev.pos()


class Text(QtWidgets.QLabel):
    """QLabel that only touches Qt when its text or colour really changes."""

    def __init__(self, text="", color="fg", font=None):
        super().__init__(text)
        if font is not None:
            self.setFont(font)
        self._color = None
        self.paint(text, color)

    def paint(self, text: str, color: str = "fg"):
        if text != self.text():
            self.setText(text)
        if color != self._color:
            self._color = color
            pal = self.palette()
            pal.setColor(QtGui.QPalette.ColorRole.WindowText, QtGui.QColor(C.get(color, color)))
            self.setPalette(pal)


class DigitSpinBox(QtWidgets.QDoubleSpinBox):
    """Setpoint box: arrow keys / mouse wheel step the digit left of the cursor.

    Typing only takes effect on Enter (keyboard tracking off), so a half-typed
    number never reaches a running lock. The wheel only works with focus, so
    scrolling past the window cannot move a setpoint by accident.
    """

    def __init__(self):
        super().__init__()
        self.setLocale(QtCore.QLocale.c())
        self.setFrame(False)  # styled via the palette: Qt 6 mis-sizes QSS-styled spin boxes
        self._focus_bg(False)
        self.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.setKeyboardTracking(False)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setRange(0.0, 99999.0)
        self.setDecimals(6)
        self.setSpecialValueText("—")  # value 0 = no setpoint
        self.setToolTip("Setpoint, vacuum wavelength (nm).\n"
                        "Put the cursor next to a digit and use ↑/↓ or the wheel to step it.")

    NONE_AT_MINIMUM = True  # (its minimum shows "—": no setpoint, which stepping leaves alone)

    def stepBy(self, steps: int):
        if self.NONE_AT_MINIMUM and self.value() == self.minimum():
            return
        le = self.lineEdit()
        text, pos = le.text(), le.cursorPosition()
        dot = text.find(".")
        dot = len(text) if dot < 0 else dot
        i = pos - 1 if pos > 0 else 0
        if i == dot:
            i -= 1
        if not 0 <= i < len(text) or not text[i].isdigit():
            i = len(text) - 1
        exponent = dot - i - 1 if i < dot else dot - i
        self.setValue(self.value() + steps * 10.0 ** exponent)
        new = le.text()
        new_dot = new.find(".")
        new_dot = len(new) if new_dot < 0 else new_dot
        le.setCursorPosition(max(0, pos + new_dot - dot))

    def wheelEvent(self, ev):
        if self.hasFocus():
            super().wheelEvent(ev)
        else:
            ev.ignore()

    def _focus_bg(self, focused: bool):
        pal = self.palette()
        pal.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor("#2a2d33" if focused else Qt.GlobalColor.transparent))
        self.setPalette(pal)

    def focusInEvent(self, ev):
        self._focus_bg(True)
        super().focusInEvent(ev)

    def focusOutEvent(self, ev):
        self._focus_bg(False)
        super().focusOutEvent(ev)


class VoltBox(DigitSpinBox):
    """An analog output's voltage, set by hand: ↑/↓ or the wheel step the digit left of the cursor; typed, it takes
    effect on Enter."""

    NONE_AT_MINIMUM = False

    def __init__(self, lo: float, hi: float):
        super().__init__()
        self.setSpecialValueText("")
        self.setFrame(True)
        self.setRange(lo, hi)
        self.setDecimals(4)
        self.setToolTip("Volts at the output. Put the cursor next to a digit and use ↑/↓ or the wheel to step it.")


class Row(QtWidgets.QFrame):
    """One channel's row: paints what it shows (a view.row dict); what clicks do is the subclass's."""

    RELOCK_TIP = ("Its DLC pro locks it to a cavity; this does not lock it to the wavemeter.\n"
                  "Relock: watch it, and take it over and relock it after a mode hop, when it is\n"
                  "locked on the wrong peak (further from the target than the wavemeter could be\n"
                  "wrong), or when its lock keeps losing it (its transmission, on average, far\n"
                  "below its level locked);\n"
                  "if it is not locked on target now, relock it now. It does nothing else: the piezo\n"
                  "is its lock's. Watching: click to stop watching.")
    LOCK_TIP = "Lock to the wavemeter: the scan offset follows the reading onto the setpoint."

    def __init__(self, win: "MainWindow", n: int):
        super().__init__()
        self.win, self.n = win, n
        self.setObjectName("row")
        self.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_menu)
        self._state = None
        self._sp = None
        self._menu = None
        self._is_ref = False
        self._relock = None  # the button arms the relock of a cavity-locked laser (else: the wavemeter lock)
        self._prompt = ""
        self._where = ""     # (another PC's row: on which, for the button's tooltip)

        self.num = Text(str(n), "dim", win.bold_font)
        self.name = Text("", "dim")
        self.wl = Text("", "fg", win.big_font)
        self.unit = Text("nm", "dim")
        self.dev = Text("", "fg", win.mid_font)
        self.sp = DigitSpinBox()
        self.sp.setFont(win.mono_font)
        self.sp.valueChanged.connect(self._setpoint_edited)
        self.sp.editingFinished.connect(lambda: self.sp.lineEdit().setModified(False))
        self.sp_label = Text("set", "dim")
        self.anchor_text = Text("", "dim", win.mono_font)  # replaces the setpoint on the reference row
        self.anchor_text.hide()
        self.info = Text("", "dim")
        self.btn = QtWidgets.QPushButton("Lock")
        self.btn.setObjectName("lock")
        self.btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.btn.clicked.connect(self._clicked)
        self.apply_fonts()

        g = QtWidgets.QGridLayout(self)
        g.setContentsMargins(12, 6, 10, 6)
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(0)
        bottom = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignBottom
        g.addWidget(self.num, 0, 0, bottom)
        g.addWidget(self.name, 1, 0)
        wl_box = QtWidgets.QHBoxLayout()
        wl_box.setSpacing(6)
        wl_box.addWidget(self.wl)
        wl_box.addWidget(self.unit, 0, bottom)
        wl_box.addStretch(1)
        g.addLayout(wl_box, 0, 1)
        sp_box = QtWidgets.QHBoxLayout()
        sp_box.setSpacing(4)
        sp_box.addWidget(self.sp_label)
        sp_box.addWidget(self.sp)
        sp_box.addWidget(self.anchor_text)
        sp_box.addStretch(1)
        g.addLayout(sp_box, 1, 1)
        g.addWidget(self.dev, 0, 2, bottom)
        g.addWidget(self.info, 1, 2)
        g.addWidget(self.btn, 0, 3, 2, 1, Qt.AlignmentFlag.AlignVCenter)
        g.setColumnStretch(2, 1)

    def apply_fonts(self):
        w = self.win
        self.wl.setFont(w.big_font)
        fm = QtGui.QFontMetrics(w.big_font)
        self.wl.setMinimumWidth(fm.horizontalAdvance("0" * (w.digits + 1)))
        self.dev.setMinimumWidth(QtGui.QFontMetrics(w.mid_font).horizontalAdvance("+0000.0 MHz"))
        self.sp.setMinimumWidth(QtGui.QFontMetrics(w.mono_font).horizontalAdvance("0" * (w.digits + 4)))
        self.num.setMinimumWidth(QtGui.QFontMetrics(w.bold_font).horizontalAdvance("0000"))

    def paint(self, v: dict, age: float, live: bool = True):
        """Show v (view.row); age: s since its reading; live False: from another PC not heard from lately (greyed)."""
        is_ref = v.get("anchor") is not None
        if is_ref != self._is_ref:
            self._is_ref = is_ref
            self.sp.setVisible(not is_ref)
            self.anchor_text.setVisible(is_ref)
            self.sp_label.paint("ref" if is_ref else "set", "dim")
        digits = self.win.digits
        f = v.get("f")
        if f:
            self.wl.paint(fmt_nm(C_NM_THZ / f, digits), "dim" if age > STALE_S or not live else "fg")
        else:
            self.wl.paint(v.get("bad") or "—", v.get("bad_c", "err") if live else "dim")
        self.unit.paint("nm", "accent" if v.get("ref_unit") else "dim")
        dev = v.get("dev")
        self.dev.paint(fmt_mhz(dev) if dev is not None else "", v.get("dev_c", "fg") if live else "dim")
        if is_ref:
            anchor = v.get("anchor")
            self.anchor_text.paint(fmt_nm(anchor, digits) if anchor else "—", "dim")
        else:
            sp = v.get("sp") or 0.0
            if sp != self._sp and not (self.sp.hasFocus() and self.sp.lineEdit().isModified()):
                self._show_setpoint(sp)
        prompt = v.get("prompt") or ""
        if prompt != self._prompt:
            if prompt:
                QtWidgets.QApplication.alert(self.win)  # flash the task bar: it waits for you
            self._prompt = prompt
            self.info.setToolTip(prompt)
        self.info.paint(v.get("info", ""), v.get("info_c", "dim") if live else "dim")
        self._set_relock(bool(v.get("relock")))
        self._set_state(v.get("state") or "off")
        self.name.paint(v.get("name", ""), "dim")

    def _set_relock(self, relock: bool):
        if relock == self._relock:
            return
        self._relock = relock
        self.btn.setToolTip((self.RELOCK_TIP if relock else self.LOCK_TIP) + self._where)
        state, self._state = self._state, None
        if state is not None:
            self._set_state(state)  # the label

    def _set_state(self, state: str):
        if state == self._state:
            return
        self._state = state
        for w in (self, self.btn):
            w.setProperty("state", state)
            w.style().unpolish(w)
            w.style().polish(w)
        if state.startswith("ref"):
            label = "Ref"
        elif self._relock:
            label = {"on": "Watching", "recovering": "Recentering", "asking": "Continue"}.get(state, "Relock")
        else:
            label = {"on": "Locked", "engaging": "Locking", "recovering": "Recentering",
                     "asking": "Continue"}.get(state, "Lock")
        self.btn.setText(label)

    def _show_setpoint(self, nm: float):
        self._sp = nm
        self.sp.blockSignals(True)
        if nm:
            self.sp.setDecimals(decimals(nm, self.win.digits))
        self.sp.setValue(nm)
        self.sp.blockSignals(False)

    def mouseDoubleClickEvent(self, ev):
        if self.childAt(event_pos(ev)) in (self.num, self.name):
            self.rename_dialog()
        else:
            super().mouseDoubleClickEvent(ev)

    def _setpoint_edited(self, value: float):
        raise NotImplementedError

    def _clicked(self):
        raise NotImplementedError

    def show_menu(self, pos):
        raise NotImplementedError

    def rename_dialog(self):
        raise NotImplementedError


class ChannelRow(Row):
    """A row of this PC's wavemeter."""

    def __init__(self, win: "MainWindow", n: int):
        self.engine = win.engine
        super().__init__(win, n)

    def refresh(self, now: float):
        v = view.row(self.engine, self.n, now)
        self.paint(v, v["age"])

    def _clicked(self):
        ch = self.engine.channels[self.n]
        if ch.state == "recovering" and ch.hop_prompt:
            self.engine.confirm_step(self.n)
        elif ch.state == "recovering":
            self.engine.cancel_recovery(self.n)
        elif self._is_ref:
            self.win.reference_dialog()
        else:
            self.engine.toggle(self.n)

    def update_tooltip(self):
        self.wl.setToolTip(view.tip(self.engine, self.n, self.win.digits))

    def _setpoint_edited(self, value: float):
        nm = round(value, decimals(value, self.win.digits)) if value > 0 else None
        self.engine.set_setpoint(self.n, nm)
        self._show_setpoint(nm or 0.0)

    # ------------------------------------------------------------- actions
    def show_menu(self, pos):
        e = self.engine
        ch = e.channels[self.n]
        ref = e.reference
        m = QtWidgets.QMenu(self)
        m.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self._menu = m
        if ref is not None and ref.channel == self.n:
            m.addAction("Reference settings…", self.win.reference_dialog)
            if ref.state == MOVED:
                m.addAction("The reference laser moved: follow it", e.reference_reanchor)
                m.addAction(f"The wavemeter moved: accept {ref.offset:+.1f} MHz as drift", self.win.accept_drift)
            m.addAction("Stop using as reference", self.win.stop_reference)
            m.addSeparator()
            self._hop_actions(m, ch)
            m.addSeparator()
            m.addAction("Rename…", self.rename_dialog)
            m.addAction("Hide channel", lambda: e.set_hidden(self.n, True))
            m.popup(self.mapToGlobal(pos))
            return
        link = m.addMenu("Link laser")
        devices = e.device_list()
        if e.scanning:
            link.addAction("scanning the LAN…").setEnabled(False)
        elif not devices:
            link.addAction("no controllers found").setEnabled(False)
        for d in devices:
            sub = link.addMenu(d.title())
            for k in range(1, 5):
                if k in d.lasers:
                    text = f"laser{k}    {d.lasers[k]}"
                elif d.probed and not d.error:
                    text = f"laser{k}    (not detected)"
                else:
                    text = f"laser{k}"
                a = sub.addAction(text)
                a.setCheckable(True)
                a.setChecked(ch.device == d.key and ch.laser == k)
                a.triggered.connect(functools.partial(self._link, d.key, k))
        link.addSeparator()
        link.addAction("Enter IP address…", self.link_dialog)
        link.addAction("Analog output (DAC)…", self.dac_dialog)
        link.addAction("Rescan LAN", e.start_discovery)
        if ch.device:
            m.addAction("Unlink", lambda: e.unlink(self.n))
        m.addSeparator()
        m.addAction("Setpoint = current reading", lambda: e.setpoint_to_current(self.n)).setEnabled(ch.freq > 0)
        m.addAction("Lock settings…", self.settings_dialog)
        self._hop_actions(m, ch)
        m.addAction("Use as reference laser" + (f" (instead of channel {ref.channel})" if ref else ""),
                    lambda: self.win.use_as_reference(self.n))
        m.addSeparator()
        m.addAction("Rename…", self.rename_dialog)
        m.addAction("Hide channel", lambda: e.set_hidden(self.n, True))
        m.popup(self.mapToGlobal(pos))

    def _hop_actions(self, m, ch):
        if ch.dac:  # (an analog output: no DLC pro to recover it with)
            m.addAction("Analog output…", self.output_dialog)
            m.addAction("Check setup…", self.setup_dialog)
            return
        m.addAction("Mode-hop recovery…", self.hop_dialog).setEnabled(bool(ch.device))
        if ch.state == "recovering":
            if ch.hop_prompt:
                short = ch.hop_prompt if len(ch.hop_prompt) < 60 else ch.hop_prompt[:57] + "…"
                m.addAction(f"Continue: {short}", lambda: self.engine.confirm_step(self.n))
            m.addAction("Stop recentering", lambda: self.engine.cancel_recovery(self.n))
        else:
            m.addAction("Recenter now", lambda: self.engine.recenter(self.n)).setEnabled(
                bool(ch.device) and ch.state != "engaging")
        m.addAction("Check setup…", self.setup_dialog).setEnabled(bool(ch.device))
        if ch.hop.enabled and ch.hop.shg:
            m.addAction("Take the optimum now (as set by hand)", lambda: self.engine.take_optimum(self.n)).setEnabled(
                bool(ch.device) and ch.state != "recovering")
        if ch.last:
            m.addAction("Last recovery…", self.last_dialog)

    def _link(self, key: str, laser: int, *_):
        self.engine.link(self.n, key, laser)

    def link_dialog(self):
        d = LinkDialog(self, self.engine, self.engine.channels[self.n])
        if d.exec():
            host, laser = d.values()
            if host:
                self.engine.link(self.n, host, laser)

    def dac_dialog(self):
        ch = self.engine.channels[self.n]
        d = DacDialog(self, ch)
        if d.exec():
            v = d.values()
            self.engine.link(self.n, v["device"], v["ao"], v["ff_ao"], v["ff_ratio"])
            p = ch.lock.p.to_dict()
            p.update(v_min=v["v_min"], v_max=v["v_max"], excursion=v["excursion"])
            self.engine.set_params(self.n, LockParams(**p))

    def output_dialog(self):
        d = OutputDialog(self, self.engine, self.n)
        d.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        d.show()

    def settings_dialog(self):
        ch = self.engine.channels[self.n]
        d = LockDialog(self, ch)
        if d.exec():
            self.engine.set_params(self.n, d.params())

    def hop_dialog(self):
        ch = self.engine.channels[self.n]
        d = HopDialog(self, ch, take=lambda: self.engine.take_optimum(self.n))
        if d.exec():
            self.engine.set_hop_params(self.n, d.params())

    def setup_dialog(self):
        d = SetupDialog(self, self.engine, self.n)
        d.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        d.show()

    def last_dialog(self):
        last = self.engine.channels[self.n].last
        if last:
            d = RecoveryDialog(self, last)
            d.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
            d.show()

    def rename_dialog(self):
        ch = self.engine.channels[self.n]
        name, ok = QtWidgets.QInputDialog.getText(self, "Rename", f"Name of channel {self.n}:", text=ch.name)
        if ok:
            self.engine.rename(self.n, name.strip())


class RemoteRow(Row):
    """A row of another PC's wavemeter (share.py), painted from what that PC sends; a click asks that PC to do what
    a click there would."""

    UNLINKED = "right-click to link a laser"

    def __init__(self, win: "MainWindow", panel: "Panel", n: int):
        self.panel = panel
        super().__init__(win, n)
        self.v = {}
        self._sp_until = 0.0  # until then the setpoint typed here is shown, not the one said there (it is on its way)
        self._enabled = None
        self._where = f"\n(on {panel.peer.label}: that PC is asked to)"

    @property
    def peer(self):
        return self.panel.peer

    def can(self) -> bool:
        """Its PC can be asked: heard from lately, and it lets others adjust its lasers."""
        p = self.peer
        return p.control and p.status(time.monotonic()) in ("live", "late")

    def _ask(self, do: str, **args):
        self.win.share.command(self.peer.id, self.n, do, **args)

    def refresh(self, now: float):
        p = self.peer
        v = self.v = p.views.get(self.n) or {}
        live = p.status(now) in ("live", "late")
        age = (v.get("age") or 0.0) + now - p.got.get(self.n, now)
        if now < self._sp_until:
            v = {**v, "sp": self._sp}
        said = p.feedback.get(self.n)
        if said is not None and now < said[1]:
            v = {**v, "info": said[0], "info_c": "err"}
        elif (v.get("info") or "").startswith(self.UNLINKED):  # (linked there, not from here)
            v = {**v, "info": "no laser linked" + v["info"][len(self.UNLINKED):]}
        self.paint(v, age, live)
        enabled = (live and p.control, self._is_ref)
        if enabled != self._enabled:
            self._enabled = enabled
            self.btn.setEnabled(enabled[0] and not enabled[1])
            self.sp.setReadOnly(not enabled[0])

    def update_tooltip(self):
        v, p = self.v, self.peer
        self.wl.setToolTip(view.head(self.n, v.get("f"), self.win.digits, v.get("refch"), v.get("corr"))
                           + (v.get("notes") or "") + f"\non {p.label} ({p.host}, {p.addr[0]})")

    def _clicked(self):
        if not self.can() or self._is_ref:
            return
        state = self.v.get("state") or "off"
        if state == "asking":
            self._ask("continue", prompt=self.v.get("prompt") or "")
        elif state == "recovering":
            self._ask("stop")
        elif state in ("off", "error"):
            self._ask("lock")
        else:
            self._ask("unlock")

    def _setpoint_edited(self, value: float):
        nm = round(value, decimals(value, self.win.digits)) if value > 0 else None
        if not self.can():
            return
        self._ask("setpoint", nm=nm)
        self._show_setpoint(nm or 0.0)
        self._sp_until = time.monotonic() + 2.0

    def show_menu(self, pos):
        p, v = self.peer, self.v
        m = QtWidgets.QMenu(self)
        m.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self._menu = m
        name = f" {v['name']}" if v.get("name") else ""
        m.addAction(f"{p.label}: channel {self.n}{name}").setEnabled(False)
        can = self.can()
        state = v.get("state") or "off"
        busy = state in ("recovering", "asking")
        if not self._is_ref:
            m.addAction("Setpoint = current reading", lambda: self._ask("setpoint_to_reading")).setEnabled(
                can and bool(v.get("f")) and not busy)
            m.addAction("Lock settings…", self.settings_dialog).setEnabled(can and isinstance(v.get("lp"), dict))
        if busy:
            prompt = v.get("prompt") or ""
            if prompt:
                short = prompt if len(prompt) < 60 else prompt[:57] + "…"
                m.addAction(f"Continue: {short}", lambda: self._ask("continue", prompt=prompt)).setEnabled(can)
            m.addAction("Stop recentering", lambda: self._ask("stop")).setEnabled(can)
        else:
            m.addAction("Recenter now", lambda: self._ask("recenter")).setEnabled(
                can and bool(v.get("linked")) and state != "engaging")
        m.addSeparator()
        m.addAction("Rename…", self.rename_dialog).setEnabled(can)
        if not p.control:
            m.addAction("its PC lets the others only watch").setEnabled(False)
        elif not can:
            m.addAction(f"not heard from {p.label} lately").setEnabled(False)
        m.popup(self.mapToGlobal(pos))

    def settings_dialog(self):
        lp = self.v.get("lp")
        if not isinstance(lp, dict):
            return
        d = LockDialog(self, types.SimpleNamespace(n=self.n, lock=types.SimpleNamespace(p=LockParams.from_dict(lp))))
        d.setWindowTitle(f"Lock settings · channel {self.n} on {self.peer.label}")
        if d.exec() and self.can():
            self._ask("lock_params", p=d.params().to_dict())

    def rename_dialog(self):
        if not self.can():
            return
        name, ok = QtWidgets.QInputDialog.getText(self, "Rename", f"Name of channel {self.n} on {self.peer.label}:",
                                                  text=self.v.get("name", ""))
        if ok:
            self._ask("rename", name=name.strip())


class PanelHeader(QtWidgets.QWidget):
    """A wavemeter's name over its rows; another PC's folds away with a click."""

    def __init__(self, panel: "Panel"):
        super().__init__()
        self.panel = panel
        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(4, 2, 4, 0)
        h.setSpacing(10)
        self.arrow = Text("", "dim")
        self.title = Text("", "fg", panel.win.bold_font)
        self.detail = Text("", "dim")
        self.status = Text("", "dim")
        for w in (self.arrow, self.title, self.detail):
            h.addWidget(w)
        h.addStretch(1)
        h.addWidget(self.status)
        self.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.customContextMenuRequested.connect(panel.show_menu)
        if panel.peer is not None:
            self.setCursor(Qt.CursorShape.PointingHandCursor)
            self.setToolTip("Click to fold it away, or back; right-click for more")

    def mousePressEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton and self.panel.peer is not None:
            self.panel.fold(not self.panel.folded)
        super().mousePressEvent(ev)


class Panel(QtWidgets.QFrame):
    """One wavemeter's rows, framed in its colour under its name while other PCs are about: this PC's (peer None),
    or another's (a share.Peer)."""

    WORDS = {"off": "off", "error": "unlocked", "engaging": "locking", "recovering": "recentering",
             "asking": "waiting for you", "ref": "reference", "ref-hold": "reference on hold",
             "ref-moved": "reference moved"}

    def __init__(self, win: "MainWindow", peer=None):
        super().__init__()
        self.win, self.peer = win, peer
        self.rows, self.row_list = {}, []
        self.folded = False
        self.framed = None
        self.color = C["line"]
        self._layout = None
        self._menu = None
        lay = QtWidgets.QVBoxLayout(self)
        lay.setSpacing(6)
        self.header = PanelHeader(self)
        lay.addWidget(self.header)
        self.rows_box = QtWidgets.QVBoxLayout()
        self.rows_box.setSpacing(6)
        lay.addLayout(self.rows_box)
        self.set_framed(peer is not None)

    def set_framed(self, framed: bool):
        if framed == self.framed:
            return
        self.framed = framed
        self.header.setVisible(framed)
        self.layout().setContentsMargins(*((8, 4, 8, 8) if framed else (0, 0, 0, 0)))
        self.update()

    def set_label(self, label: str):
        color = self.win.colors.get(label) or panel_colors([label])[label]
        self.header.title.paint(label, color)
        if color != self.color:
            self.color = color
            self.update()

    def paintEvent(self, ev):
        super().paintEvent(ev)
        if self.framed:
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
            c = QtGui.QColor(self.color)
            c.setAlpha(150)
            p.setPen(QtGui.QPen(c, 1.2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRoundedRect(QtCore.QRectF(self.rect()).adjusted(0.6, 0.6, -0.6, -0.6), 8, 8)

    def show_menu(self, pos):
        m = QtWidgets.QMenu(self)
        m.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self._menu = m
        p = self.peer
        if p is not None:
            m.addAction("Unfold" if self.folded else "Fold away", lambda: self.fold(not self.folded))
            if p.status(time.monotonic()) in ("lost", "closed"):
                m.addAction(f"Forget {p.label}", lambda: self.win.share.forget(p.id))
            m.addSeparator()
        m.addAction("Network…", self.win.share_dialog)
        m.popup(self.header.mapToGlobal(pos))

    # ------------------------------------------------------- another PC's
    def fold(self, folded: bool):
        """Fold its rows away (or back): it sends this PC its rows every few seconds only, meanwhile."""
        self.folded = folded
        for row in self.row_list:
            row.setVisible(not folded)
        label = self.peer.label
        names = self.win.gui.setdefault("folded", [])
        if folded and label not in names:
            names.append(label)
        elif not folded and label in names:
            names.remove(label)
        self.win.engine.dirty = True
        self.win.want(self)
        self.win.fit_later()

    def refresh(self, now: float):
        p = self.peer
        if p.layout != self._layout:
            self._rebuild()
        self.set_label(p.label or p.host)
        h = self.header
        h.arrow.paint("▸" if self.folded else "▾", "dim")
        where = p.host + (f"  {p.addr[0]}" if p.addr else "") + ("" if p.control else "   watch only")
        h.detail.paint(f"{where}   {self.summary()}" if self.folded else where, "dim")
        h.status.paint(*self._status(now))
        if not self.folded:
            for row in self.row_list:
                row.refresh(now)

    def _status(self, now: float):
        p = self.peer
        state, quiet = p.status(now), now - p.heard
        if state == "closed":
            return f"closed at {time.strftime('%H:%M', time.localtime(p.bye_wall))}", "dim"
        if state == "lost":
            return f"not heard from for {fmt_duration(quiet)}", "err"
        if state == "late":
            return f"● {fmt_duration(quiet)} since it was heard from", "warn"
        src = p.src or {}
        if src.get("ok") is False:
            return f"● {src.get('msg') or 'no wavemeter'}", "err"
        return f"● {src.get('rate', 0)} readings/s", "dim"

    def summary(self) -> str:
        """Its rows in a line: how many locked, watched, off..."""
        counts = {}
        for n in self.peer.rows:
            v = self.peer.views.get(n) or {}
            state = v.get("state") or "off"
            word = ("watching" if v.get("relock") else "locked") if state == "on" else self.WORDS.get(state, state)
            counts[word] = counts.get(word, 0) + 1
        k = len(self.peer.rows)
        return f"{k} channel{'s' if k != 1 else ''}" + (": " + ", ".join(f"{c} {w}" for w, c in counts.items())
                                                        if counts else "")

    def _rebuild(self):
        p = self.peer
        self._layout = p.layout
        wanted = list(p.rows)
        for n in list(self.rows):
            if n not in wanted:
                row = self.rows.pop(n)
                row.hide()
                row.deleteLater()
        for row in self.rows.values():
            self.rows_box.removeWidget(row)
        for n in wanted:
            if n not in self.rows:
                self.rows[n] = RemoteRow(self.win, self, n)
            self.rows_box.addWidget(self.rows[n])
            self.rows[n].setVisible(not self.folded)
        self.row_list = [self.rows[n] for n in wanted]
        self.win.fit_later()


class LinkDialog(QtWidgets.QDialog):
    def __init__(self, parent, engine: Engine, ch):
        super().__init__(parent)
        self.setWindowTitle(f"Link channel {ch.n}")
        self.host = QtWidgets.QComboBox()
        self.host.setEditable(True)
        self.host.addItems([d.key for d in engine.device_list()])
        self.host.setEditText(ch.device if ch.device and not ch.dac else "")
        self.host.lineEdit().setPlaceholderText("192.168.1.20")
        self.laser = QtWidgets.QComboBox()
        self.laser.addItems([f"laser{k}" for k in range(1, 5)])
        self.laser.setCurrentIndex(max(0, min(ch.laser, 4) - 1))
        form = QtWidgets.QFormLayout(self)
        form.addRow("Controller IP", self.host)
        form.addRow("Laser", self.laser)
        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def values(self):
        return self.host.currentText().strip(), self.laser.currentIndex() + 1


class DacDialog(QtWidgets.QDialog):
    """A laser without a digital controller: its piezo driver on an analog output (dac.py), and maybe its current on
    a second one, fed forward."""

    def __init__(self, parent, ch):
        super().__init__(parent)
        self.setWindowTitle(f"Analog output · channel {ch.n}")
        form = QtWidgets.QFormLayout(self)

        def row(*widgets):
            box = QtWidgets.QHBoxLayout()
            for w in widgets:
                box.addWidget(Text(w, "dim") if isinstance(w, str) else w)
            box.addStretch(1)
            return box

        def number(value, lo, hi, dec, step):
            sb = QtWidgets.QDoubleSpinBox()
            sb.setLocale(QtCore.QLocale.c())
            sb.setRange(lo, hi)
            sb.setDecimals(dec)
            sb.setSingleStep(step)
            sb.setValue(value)
            return sb

        kind, number_now = dac.parse(ch.device) if ch.dac else ("mcc", 0)
        self.kinds = [("mcc", "Measurement Computing (mcculw, as set up in InstaCal)")]
        if dac.SIM_BOARDS:
            self.kinds.append(("sim", "simulated"))
        self.kind = QtWidgets.QComboBox()
        self.kind.addItems([label for _, label in self.kinds])
        self.kind.setCurrentIndex(next((k for k, (name, _) in enumerate(self.kinds) if name == kind), 0))
        self.board = QtWidgets.QSpinBox()
        self.board.setRange(0, 99)
        self.board.setValue(number_now)
        form.addRow("Board", row(self.kind, "number", self.board, "(InstaCal's: Epimetheus's 'Dev0' is 0)"))
        self.ao = QtWidgets.QSpinBox()
        self.ao.setRange(0, 63)
        self.ao.setValue(ch.laser if ch.dac else 0)
        form.addRow("Output", row(self.ao, "drives its piezo driver (Epimetheus's index_P)"))
        self.ff = QtWidgets.QCheckBox("Feed its moves forward to output")
        self.ff.setChecked(ch.ff_ao is not None)
        self.ff_ao = QtWidgets.QSpinBox()
        self.ff_ao.setRange(0, 63)
        self.ff_ao.setValue(ch.ff_ao if ch.ff_ao is not None else 1)
        self.ff_ratio = number(ch.ff_ratio or 1.0, 0.01, 1e4, 3, 0.1)
        form.addRow(row(self.ff, self.ff_ao, "divided by", self.ff_ratio,
                        "(Epimetheus's current output, index_I, and Feed_forward)"))
        p = ch.lock.p if ch.dac else LockParams(**{**LockParams().to_dict(), **DAC_LOCK})
        self.v_min = number(p.v_min, -10.0, 10.0, 3, 0.1)
        self.v_max = number(p.v_max, -10.0, 10.0, 3, 0.1)
        form.addRow("Never outside", row(self.v_min, "…", self.v_max, "V (Epimetheus: ±1.2 V)"))
        self.excursion = number(p.excursion, 0.0, 20.0, 3, 0.1)
        form.addRow("Locked, at most", row("±", self.excursion, "V from where it locked (Epimetheus: ±0.8 V)"))
        form.addRow(Text("A DAC cannot be read back: before its first lock, set the output by hand (right-click ›\n"
                         "Analog output…) to where the laser is now: in Epimetheus, its offset plus its PID's output\n"
                         "(the 'piezo' it shows). From then on it is kept, and set back after a restart.", "dim"))
        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def values(self) -> dict:
        ff = self.ff.isChecked()
        return {"device": dac.key(self.kinds[self.kind.currentIndex()][0], self.board.value()), "ao": self.ao.value(),
                "ff_ao": self.ff_ao.value() if ff else None, "ff_ratio": self.ff_ratio.value() if ff else 0.0,
                "v_min": min(self.v_min.value(), self.v_max.value()),
                "v_max": max(self.v_min.value(), self.v_max.value()), "excursion": self.excursion.value()}


class OutputDialog(QtWidgets.QDialog):
    """An analog output set by hand, where its laser sits while not locked (Epimetheus's offset); and the output fed
    forward, if any."""

    def __init__(self, parent, engine, n):
        super().__init__(parent)
        self.engine, self.n = engine, n
        ch = engine.channels[n]
        self.setWindowTitle(f"Analog output · channel {n}")
        form = QtWidgets.QFormLayout(self)
        p = ch.lock.p
        self.boxes, self.buttons = [], []
        for ff, ao in ((False, ch.laser), (True, ch.ff_ao)):
            if ao is None:
                continue
            box = VoltBox(p.v_min, p.v_max)
            known = engine.dacs.get(ch.device, ao)
            box.setValue(known if known is not None else 0.0)
            box.valueChanged.connect(functools.partial(self._set, ff))
            put = QtWidgets.QPushButton("Set")
            put.setToolTip("Write the voltage shown (one never set by this program is not known: this makes it so)")
            put.clicked.connect(lambda _=False, ff=ff, box=box: self._set(ff, box.value()))
            last = Text("", "dim")
            row = QtWidgets.QHBoxLayout()
            row.addWidget(box)
            row.addWidget(Text("V", "dim"))
            row.addWidget(put)
            row.addWidget(last)
            row.addStretch(1)
            form.addRow(f"Output {ao}" + (" (fed forward)" if ff else ""), row)
            self.boxes.append((ff, ao, box, last))
            self.buttons.append(put)
        self.note = Text("", "dim")
        form.addRow(self.note)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Close)
        bb.rejected.connect(self.reject)
        form.addRow(bb)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(250)
        self._refresh()

    def _set(self, ff: bool, value: float):
        self.engine.set_output(self.n, value, ff)

    def _refresh(self):
        e = self.engine
        ch = e.channels[self.n]
        free = ch.state == "off"
        for put in self.buttons:
            put.setEnabled(free)
        for _, ao, box, last in self.boxes:
            known = e.dacs.get(ch.device, ao)
            last.paint(f"set to {known:.4f} V" if known is not None else "not set yet: its voltage is not known",
                       "dim" if known is not None else "warn")
            box.setEnabled(free)
            if not free and not box.hasFocus() and known is not None and abs(box.value() - known) > 5e-5:
                box.blockSignals(True)
                box.setValue(known)
                box.blockSignals(False)
        p = ch.lock.p
        self.note.paint(f"Within {p.v_min:g} … {p.v_max:g} V (its lock's limits). " +
                        ("Locked: its lock moves it." if not free else "Each change is written at once."), "dim")


class LockDialog(QtWidgets.QDialog):
    FIELDS = (  # name, label, unit, min, max, decimals
        ("kp", "P gain", "V/GHz", -1e3, 1e3, 3),
        ("ki", "I gain", "V/GHz per reading", -1e3, 1e3, 3),
        ("max_step", "Max step", "V per reading", 0.0, 100.0, 3),
        ("v_min", "Min offset", "V", -1e3, 1e3, 2),
        ("v_max", "Max offset", "V", -1e3, 1e3, 2),
        ("excursion", "Max excursion", "± V from lock point, 0 = off", 0.0, 1e3, 2),
        ("capture", "Capture range", "MHz, 0 = off", 0.0, 1e6, 0),
    )

    def __init__(self, parent, ch):
        super().__init__(parent)
        self.setWindowTitle(f"Lock settings · channel {ch.n}")
        form = QtWidgets.QFormLayout(self)
        self.boxes = {}
        for key, label, unit, lo, hi, dec in self.FIELDS:
            sb = QtWidgets.QDoubleSpinBox()
            sb.setLocale(QtCore.QLocale.c())
            sb.setRange(lo, hi)
            sb.setDecimals(dec)
            sb.setValue(getattr(ch.lock.p, key))
            row = QtWidgets.QHBoxLayout()
            row.addWidget(sb)
            row.addWidget(Text(unit, "dim"))
            form.addRow(label, row)
            self.boxes[key] = sb
        note = Text("Each new reading moves the scan offset by −(P·e + I·Σe), e in GHz.\n"
                    "Positive gains if the frequency rises with the offset: flip the sign\n"
                    "if the lock runs away. I ≈ 0.3 / tuning (GHz/V) is a good start.", "dim")
        form.addRow(note)
        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def params(self) -> LockParams:
        return LockParams(**{k: sb.value() for k, sb in self.boxes.items()})


class HopDialog(QtWidgets.QDialog):
    RELOCK = (("auto", "as its DLC pro is set: by lockpoint, unless it locks without one"),
              ("lockpoint", "pick the right peak and select it as the lockpoint, as by hand"),
              ("plain", "re-enable the lock and let it find its resonance"))
    METHOD = (("auto", "catch with an SHG stage, else map"),
              ("catch", "catch: back onto its own mode by the current (up, then down until its SHG peak is back), "
                        "then the piezo onto the target"),
              ("walk", "walk: single mode first, then the piezo slowly onto the target, as by hand"),
              ("map", "map: sweep the current over the window, up and down, and pick the best mode"))

    def __init__(self, parent, ch, take=None):
        """take: take the optimum now (Engine.take_optimum for this channel), if it can be."""
        super().__init__(parent)
        self.setWindowTitle(f"Mode-hop recovery · channel {ch.n}")
        self.ch = ch
        p = self._p = ch.hop
        form = QtWidgets.QFormLayout(self)

        def row(*widgets):
            box = QtWidgets.QHBoxLayout()
            for w in widgets:
                box.addWidget(Text(w, "dim") if isinstance(w, str) else w)
            box.addStretch(1)
            return box

        def number(value, lo, hi, dec, step=None):
            sb = QtWidgets.QDoubleSpinBox()
            sb.setLocale(QtCore.QLocale.c())
            sb.setRange(lo, hi)
            sb.setDecimals(dec)
            if step:
                sb.setSingleStep(step)
            sb.setValue(value)
            return sb

        def heading(text):
            label = Text(text, "accent")
            label.setContentsMargins(0, 8, 0, 0)
            form.addRow(label)

        self.enabled = QtWidgets.QCheckBox("Recover this laser when it mode-hops")
        self.enabled.setChecked(p.enabled)
        form.addRow(self.enabled)
        self.who = QtWidgets.QComboBox()
        self.who.addItems(["this program, to the wavemeter", "its DLC pro, to a cavity (PDH / FALC)"])
        self.who.setCurrentIndex(1 if p.cavity else 0)
        form.addRow("Locked by", self.who)
        self.shg = QtWidgets.QCheckBox("It has an SHG stage: check its mode in the SHG cavity, and relock that")
        self.shg.setChecked(p.shg)
        form.addRow(self.shg)

        heading("Current and piezo")
        self.i_nom = number(p.i_nom or (ch.current or 0.0), 0, 10000, 3, 0.1)
        use = QtWidgets.QPushButton("Use present")
        use.setEnabled(ch.current is not None)
        use.clicked.connect(lambda: self.i_nom.setValue(ch.current or 0.0))
        now = f"now {ch.current:.3f} mA" if ch.current is not None else "read when enabled"
        form.addRow("Nominal current", row(self.i_nom, "mA measured (current-act)", use, now))
        self.i_below = number(p.i_below, 0.0, 50, 2, 0.1)
        self.i_window = number(p.i_window, 0.1, 50, 2, 0.1)
        form.addRow("Current stays within", row(self.i_below, "mA below and", self.i_window,
                                                "mA above nominal (and the controller's limit)"))
        self.v_window = number(p.v_window, 0.5, 100, 1, 0.5)
        form.addRow("Piezo moves at most", row(self.v_window, "V per recovery, inside the lock's offset limits"))
        self.tolerance = number(p.tolerance, 5, 100000, 0, 10)
        self.grace = number(p.grace, 0.2, 60, 1, 0.5)
        form.addRow("A hop is", row("off target by more than", self.tolerance, "MHz for", self.grace, "s"))
        self.tolerance.setToolTip("For a laser locked to the wavemeter. One its DLC pro locks to a cavity is off "
                                  "target further than Wrong peak, below.")
        self.per_hour = QtWidgets.QSpinBox()
        self.per_hour.setRange(1, 60)
        self.per_hour.setValue(p.max_per_hour)
        form.addRow("At most", row(self.per_hour, "recoveries an hour (not counting a laser that stopped lasing)"))

        heading("Finding a mode on target")
        self.method = QtWidgets.QComboBox()
        self.method.addItems([label for _, label in self.METHOD])
        self.method.setCurrentIndex(next((k for k, (key, _) in enumerate(self.METHOD) if key == p.method), 0))
        form.addRow("Method", self.method)
        self.catch_pct = number(round(100 * (1 - p.catch_min)), 1, 60, 0, 1)
        self.catch_tries = QtWidgets.QSpinBox()
        self.catch_tries.setRange(1, 10)
        self.catch_tries.setValue(p.catch_tries)
        self.pc_try = number(p.pc_try, 0, 100, 1, 5)
        form.addRow("Catch", row("its SHG peak within", self.catch_pct, "% of the optimum's;", self.catch_tries,
                                 "goes, then from the optimum's piezo voltage ±", self.pc_try, "V"))
        self.pc_try.setToolTip("The right mode: the one it was on target on before it hopped. Each go takes the "
                               "current to the top of\nits window, then down a step at a time, until the laser is on "
                               "the right mode and its SHG\npeak (the tallest resonance in the SHG cavity's sweep) is "
                               "within that of the optimum's.\nNot caught in that many goes, it tries from the "
                               "optimum's piezo voltage + and - this\n(the piezo may go that far for it), then walks "
                               "the piezo back onto the target, the\ncurrent parked high in the mode every few steps. "
                               "0: no tries elsewhere.")
        self.optimum = Text("", "dim")
        self._cleared = False
        self._opt_seen = None
        take_btn = QtWidgets.QPushButton("Take now")
        take_btn.setEnabled(take is not None and bool(ch.device))
        take_btn.setToolTip("The laser as it is now is at its optimum, set by hand: its current, piezo voltage and SHG "
                            "signal locked\nare what the catch goes back to (taken by itself the first time Relock is "
                            "clicked).")
        if take is not None:
            take_btn.clicked.connect(take)
        clear = QtWidgets.QPushButton("Clear")
        clear.setToolTip("Forget it: taken again the next time Relock is clicked")
        clear.clicked.connect(self._clear_optimum)
        form.addRow("Optimum", row(self.optimum, take_btn, clear))
        self._show_optimum()
        self._timer = QtCore.QTimer(self)  # (taken now: it comes in a moment later)
        self._timer.timeout.connect(self._show_optimum)
        self._timer.start(300)
        self.reach = number(p.reach, 0.5, 100, 1, 0.5)
        self.walk_step = number(p.walk_step, 0.05, 5, 2, 0.1)
        form.addRow("Walk", row("from a single-mode mode within", self.reach, "GHz (walk), in", self.walk_step,
                                "V steps"))
        self.margin = number(p.margin, 0.05, 5, 2, 0.05)
        form.addRow("Keep a hop at least", row(self.margin, "mA away, both ways (the map checks it)"))
        self.step = number(p.step, 0.01, 1, 2, 0.01)
        form.addRow("Trim and map in steps of", row(self.step, "mA"))

        heading("Lasing")
        self.seed = QtWidgets.QLineEdit(p.seed)
        self.seed.setToolTip("The parameter (below laserN:) that shows the TA's seed power: auto tries "
                             "amp:pd:seed:power, then amp:seed-limits:power; off: do not watch.")
        usual = f"(usually {p.seed_ok:.1f} mW)" if p.seed_ok else "(its usual, learned)"
        self.seed_min = number(p.seed_min, 0.05, 0.95, 2, 0.05)
        form.addRow("Seed power", row(self.seed, "below", self.seed_min, "of usual", usual, "is not lasing"))
        self.jump = number(p.jump, 0.5, 20, 1, 0.5)
        self.jump_max = number(p.jump_max, 0.5, 20, 1, 0.5)
        self.ramp = number(p.ramp, 0.05, 10, 2, 0.1)
        form.addRow("Not lasing", row("current to nominal +", self.jump, "… +", self.jump_max, "mA at once, back at",
                                      self.ramp, "mA/s"))

        heading("Relocking the cavity lock")
        self.relock = QtWidgets.QComboBox()
        self.relock.addItems([label for _, label in self.RELOCK])
        self.relock.setCurrentIndex(next((k for k, (key, _) in enumerate(self.RELOCK) if key == p.relock), 0))
        form.addRow("Relock by", self.relock)
        self.window = number(p.window, 10, 100000, 0, 10)
        form.addRow("The right peak is", row("within ±", self.window, "MHz of where the wavemeter puts the setpoint"))
        self.peak_tol = number(p.peak_tol, 5, 10000, 0, 5)
        form.addRow("Wrong peak", row("locked, but more than", self.peak_tol, "MHz from the target (the wavemeter is "
                                      "never that far off): relock"))
        self.lock_min = number(p.lock_min, 0.05, 0.95, 2, 0.05)
        form.addRow("Lock slipping", row("transmission below", self.lock_min, "of the way from its zero to its level "
                                         "locked, on average over 10 s: relock"))
        self.slip_levels = Text("", "dim")
        form.addRow("", self.slip_levels)
        self.lock_min.valueChanged.connect(self._slip_levels)
        self._slip_levels()
        self.lock_min.setToolTip("The transmission: the input the lock detection watches (Fine In 1, as a rule). Its "
                                 "zero is its level\noff resonance (any DC offset), measured while the master scans in "
                                 "a relock; its level locked,\nthe median of what it read locked on target over the "
                                 "last few minutes. A lock that keeps\nlosing the laser and relocking (ReLock) lets it "
                                 "through only now and then. A lock turned off\n(by hand, say) is not watched for "
                                 "this.")
        self.falc = QtWidgets.QCheckBox("Null the error signal's DC level with the FALC pro's input offset, by at most")
        self.falc.setChecked(p.falc)
        self.falc_max = number(p.falc_max, 0.001, 1.0, 3, 0.005)
        form.addRow(row(self.falc, self.falc_max, "V"))
        self.dropouts = QtWidgets.QCheckBox("Log its lock dropouts (the transmission below the slipping limit), "
                                            "with the DLC pro's recorder, sampled every")
        self.dropouts.setChecked(p.dropouts)
        self.dropouts.setToolTip("Two samples in a row below the limit are a dropout. Each one goes to the console\n"
                                 "(with the time, how long, how low) and to ~/.simplewavemeterlock-dropouts.jsonl;\n"
                                 "once an hour, how many there were. While this program runs it uses the DLC pro's\n"
                                 "recorder (its settings are put back when it closes), reading what it recorded\n"
                                 "every 2 s; without one, it polls the transmission instead, which can miss\n"
                                 "dropouts shorter than the time between reads. Sampling less often asks less of\n"
                                 "the controller (and of TOPAS, which shares it).")
        self.drop_ms = number(p.drop_ms, 0.1, 20.0, 2, 0.25)
        form.addRow(row(self.dropouts, self.drop_ms, "ms"))
        self.confirm = QtWidgets.QCheckBox("Step by step: ask me before each step (for the first runs on a laser)")
        self.confirm.setChecked(p.confirm)
        form.addRow(self.confirm)
        self.forget = False
        learned = self._learned(p)
        if learned:
            fm = QtGui.QFontMetrics(self.font())
            self.learned = Text(fm.elidedText(learned, Qt.TextElideMode.ElideRight, fm.horizontalAdvance("m" * 36)),
                                "dim")
            self.learned.setToolTip(learned.replace(", ", "\n"))
            forget = QtWidgets.QPushButton("Forget")
            forget.clicked.connect(self._forget)
            form.addRow("Learned", row(self.learned, forget))
        form.addRow(Text("On failure everything goes back to where it was (the current down slowly), except that "
                         "the cavity lock stays off,\nand a laser that stopped lasing stays where it lases. "
                         "Right-click the channel: Check setup… reads what a recovery\nwould use; nothing is written.",
                         "dim"))
        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)
        self.who.currentIndexChanged.connect(self._who)
        self._who()

    @staticmethod
    def _learned(p: HopParams) -> str:
        out = []
        if p.kv:
            out.append(f"piezo {p.kv / 1e3:+.3f} GHz/V")
        if p.spacing:
            out.append(f"modes {p.spacing / 1e3:.2f} GHz apart")
        if p.peak_y:
            out.append(f"right peak {p.peak_y:.3f} V high")
        if p.lp_kind:
            out.append(f"lockpoint type {p.lp_kind!r}" + (f" = {p.lp_code}" if p.lp_code >= 0 else ""))
        if p.shg_area:
            out.append(f"single-mode SHG sweep: side peak {p.shg_side * 100:.0f} %")
        if p.falc_gain:
            out.append(f"the FALC offset moves the error signal {p.falc_gain:+.3g} V/V")
        elif p.falc_sign:
            out.append("the FALC offset shows on the error signal")
        if p.seed_ok:
            out.append(f"seed {p.seed_ok:.1f} mW lasing")
        if p.trans_locked:
            out.append(f"transmission {volts(p.trans_locked, p.trans_locked)} V locked")
        if p.trans_zero:
            out.append(f"transmission zero {volts(p.trans_zero, p.trans_locked)} V")
        if p.shg_lock:
            out.append(f"SHG signal {p.shg_lock:.3f} V locked")
        if p.shg_ratio:
            out.append(f"SHG peak in a sweep {p.shg_ratio:.2f} of the signal locked")
        return ", ".join(out)

    def _show_optimum(self):
        """The optimum as taken (it may come in while the dialog is open: then the nominal current with it)."""
        p = self._p
        now = None if self._cleared else (p.opt_i, p.opt_v, p.opt_shg, p.opt_t)
        if now == self._opt_seen:
            return
        if self._opt_seen is not None and now is not None and p.optimum:
            self.i_nom.setValue(p.opt_i)  # (taking it sets the nominal current too)
        self._opt_seen = now
        if now is None or not p.optimum:
            self.optimum.paint("not taken: taken when Relock is first clicked on the laser set up by hand", "dim")
        else:
            self.optimum.paint(f"{p.opt_i:.2f} mA, {p.opt_v:.2f} {self.ch.unit}"
                               + (f", SHG {p.opt_shg:.3f} V locked" if p.opt_shg else ", SHG level not known")
                               + (f" ({p.opt_t})" if p.opt_t else ""), "fg")

    def _clear_optimum(self):
        self._cleared = True
        self._show_optimum()
        self.optimum.paint("cleared: taken again the next time Relock is clicked", "dim")

    def _slip_levels(self):
        """Where the transmission's zero and level locked put the limit, with lock_min as set."""
        p = self._p
        zero, locked = usable_zero(p.trans_zero, p.trans_locked), p.trans_locked
        if locked > zero:
            limit = zero + self.lock_min.value() * (locked - zero)
            self.slip_levels.paint(f"now: zero {volts(zero, locked)} V, locked {volts(locked, locked)} V: relock below "
                                   f"{volts(limit, locked)} V on average", "dim")
        else:
            self.slip_levels.paint("its zero and its level locked are learned (the zero is 0 V until measured)", "dim")

    def _who(self):
        cavity = self.who.currentIndex() == 1
        for w in (self.relock, self.window, self.falc, self.falc_max, self.dropouts, self.drop_ms, self.peak_tol,
                  self.lock_min, self.slip_levels):
            w.setEnabled(cavity)
        self.tolerance.setEnabled(not cavity)

    def _forget(self):
        self.forget = True
        self.learned.paint("forgotten: measured again next time", "dim")

    def params(self) -> HopParams:
        p = self._p
        out = HopParams(
            enabled=self.enabled.isChecked(), cavity=self.who.currentIndex() == 1, shg=self.shg.isChecked(),
            method=self.METHOD[self.method.currentIndex()][0],
            relock=self.RELOCK[self.relock.currentIndex()][0], falc=self.falc.isChecked(),
            dropouts=self.dropouts.isChecked(), drop_ms=self.drop_ms.value(),
            falc_max=self.falc_max.value(), window=self.window.value(), peak_tol=self.peak_tol.value(),
            lock_min=self.lock_min.value(), confirm=self.confirm.isChecked(),
            i_nom=round(self.i_nom.value(), 3), i_window=self.i_window.value(), i_below=self.i_below.value(),
            v_window=self.v_window.value(), reach=self.reach.value(), walk_step=self.walk_step.value(),
            step=self.step.value(), margin=self.margin.value(), tolerance=self.tolerance.value(),
            grace=self.grace.value(), max_per_hour=self.per_hour.value(), seed=self.seed.text().strip() or "auto",
            seed_min=self.seed_min.value(), jump=self.jump.value(), jump_max=self.jump_max.value(),
            ramp=self.ramp.value(), catch_min=round(1 - self.catch_pct.value() / 100, 4),
            catch_tries=self.catch_tries.value(), pc_try=self.pc_try.value())
        if not self.forget:
            for name in HopParams.LEARNED:
                setattr(out, name, getattr(p, name))
        if not self._cleared:
            for name in HopParams.OPTIMUM:
                setattr(out, name, getattr(p, name))
        return out


class Plot(QtWidgets.QWidget):
    """A small, plain plot: lines, dots, vertical and horizontal lines and shaded spans, with the ranges marked."""

    def __init__(self, xlabel: str = "", ylabel: str = "", height: int = 160, ylim=None):
        super().__init__()
        self.xlabel, self.ylabel, self.ylim = xlabel, ylabel, ylim
        self.series = []  # (xs, ys, color, 'line' | 'dots' | 'ring' | 'cross', width)
        self.vlines = []  # (x, color)
        self.spans = []   # (x0, x1, color)
        self.hlines = []  # (y, color)
        self.hspans = []  # (y0, y1, color)
        self.setMinimumHeight(height)
        self.setMinimumWidth(420)

    def add(self, xs, ys, color="accent", style="line", width=1.5):
        pts = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None
               and math.isfinite(x) and math.isfinite(y)]
        if pts:
            self.series.append(([p[0] for p in pts], [p[1] for p in pts], color, style, width))
        return self

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#1b1c20"))
        xs = ([x for s in self.series for x in s[0]] + [v[0] for v in self.vlines]
              + [a for s in self.spans for a in s[:2]])
        ys = ([y for s in self.series for y in s[1]] + [h[0] for h in self.hlines]
              + [a for s in self.hspans for a in s[:2]])
        if not xs or not ys:
            p.setPen(QtGui.QColor(C["dim"]))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "nothing to show")
            return
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        if x1 - x0 < 1e-12:
            x0, x1 = x0 - 1, x1 + 1
        if y1 - y0 < 1e-12:
            y0, y1 = y0 - 1, y1 + 1
        pad = 0.05 * (y1 - y0)
        y0, y1 = (y0 - pad, y1 + pad) if self.ylim is None else self.ylim
        fm = p.fontMetrics()
        left = fm.horizontalAdvance("-0000.000") + 8
        r = QtCore.QRectF(left, 8, self.width() - left - 10, self.height() - 2 * fm.height() - 14)

        def X(x):
            return r.left() + (x - x0) / (x1 - x0) * r.width()

        def Y(y):
            return r.bottom() - (y - y0) / (y1 - y0) * r.height()

        for a, b, color in self.spans:
            c = QtGui.QColor(C.get(color, color))
            c.setAlpha(50)
            p.fillRect(QtCore.QRectF(X(min(a, b)), r.top(), abs(X(b) - X(a)), r.height()), c)
        for a, b, color in self.hspans:
            c = QtGui.QColor(C.get(color, color))
            c.setAlpha(40)
            p.fillRect(QtCore.QRectF(r.left(), Y(max(a, b)), r.width(), abs(Y(a) - Y(b))), c)
        p.setPen(QtGui.QPen(QtGui.QColor(C["line"]), 1))
        p.drawRect(r)
        p.setClipRect(r)
        for y, color in self.hlines:
            p.setPen(QtGui.QPen(QtGui.QColor(C.get(color, color)), 1, Qt.PenStyle.DashLine))
            p.drawLine(QtCore.QPointF(r.left(), Y(y)), QtCore.QPointF(r.right(), Y(y)))
        for x, color in self.vlines:
            p.setPen(QtGui.QPen(QtGui.QColor(C.get(color, color)), 1, Qt.PenStyle.DashLine))
            p.drawLine(QtCore.QPointF(X(x), r.top()), QtCore.QPointF(X(x), r.bottom()))
        for sx, sy, color, style, width in self.series:
            col = QtGui.QColor(C.get(color, color))
            if style == "line":
                p.setPen(QtGui.QPen(col, width))
                p.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(X(x), Y(y)) for x, y in zip(sx, sy)]))
            else:
                p.setPen(QtGui.QPen(col, 1.2 if style == "ring" else width))
                p.setBrush(Qt.BrushStyle.NoBrush if style == "ring" else col)
                for x, y in zip(sx, sy):
                    c = QtCore.QPointF(X(x), Y(y))
                    if style == "ring":
                        p.drawEllipse(c, width + 2.0, width + 2.0)
                    elif style == "cross":
                        p.drawLine(c + QtCore.QPointF(-3, -3), c + QtCore.QPointF(3, 3))
                        p.drawLine(c + QtCore.QPointF(-3, 3), c + QtCore.QPointF(3, -3))
                    else:
                        p.drawEllipse(c, width, width)
        p.setClipping(False)
        p.setPen(QtGui.QColor(C["dim"]))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawText(QtCore.QRectF(0, r.top() - 4, left - 4, fm.height()), Qt.AlignmentFlag.AlignRight, f"{y1:.4g}")
        p.drawText(QtCore.QRectF(0, r.bottom() - fm.height() + 4, left - 4, fm.height()),
                   Qt.AlignmentFlag.AlignRight, f"{y0:.4g}")
        p.drawText(QtCore.QRectF(0, r.center().y() - fm.height() / 2, left - 4, fm.height()),
                   Qt.AlignmentFlag.AlignRight, self.ylabel)
        bottom = QtCore.QRectF(r.left(), r.bottom() + 2, r.width(), fm.height())
        p.drawText(bottom, Qt.AlignmentFlag.AlignLeft, f"{x0:.4g}")
        p.drawText(bottom, Qt.AlignmentFlag.AlignRight, f"{x1:.4g}")
        p.drawText(bottom, Qt.AlignmentFlag.AlignHCenter, self.xlabel)


class RecoveryDialog(QtWidgets.QDialog):
    """What the last recovery saw and did: its steps, its maps, the SHG sweeps, the cavity scan with
    the peak it picked, the error signal; for checking its judgement against yours."""

    def __init__(self, parent, rec: dict):
        super().__init__(parent)
        self.setWindowTitle(f"Last recovery · channel {rec.get('channel', '?')}")
        self.resize(760, 560)
        v = QtWidgets.QVBoxLayout(self)
        head = Text(f"{rec.get('time', '')}   {rec.get('outcome', '')}   ({rec.get('took_s', 0):.0f} s)",
                    "ok" if rec.get("ok") else "err")
        head.setWordWrap(True)
        v.addWidget(head)
        tabs = QtWidgets.QTabWidget()
        v.addWidget(tabs, 1)
        steps = QtWidgets.QPlainTextEdit()
        steps.setReadOnly(True)
        steps.setPlainText("\n".join(f"{t:6.1f} s  {text}" if t is not None else f"{'':8s} {text}"
                                     for t, text in rec.get("notes", [])))
        tabs.addTab(steps, "Steps")
        if rec.get("catch"):
            tabs.addTab(self._scroll([self._catch(c) for c in rec["catch"]]), "Catch")
        if rec.get("walk"):
            tabs.addTab(self._scroll(self._walk(rec["walk"])), "Walk")
        if rec.get("maps"):
            tabs.addTab(self._scroll([self._map(m, rec) for m in rec["maps"]]), "Map")
        if rec.get("sweeps"):
            tabs.addTab(self._scroll([self._sweep(s) for s in rec["sweeps"]]), "SHG sweeps")
        if rec.get("scan"):
            tabs.addTab(self._scroll([self._scan(rec["scan"])]), "Cavity peaks")
        if rec.get("falc"):
            tabs.addTab(self._scroll([self._falc(rec["falc"])]), "Error signal")
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Close)
        bb.rejected.connect(self.reject)
        v.addWidget(bb)

    @staticmethod
    def _scroll(widgets):
        inner = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(inner)
        for w in widgets:
            lay.addWidget(w)
        lay.addStretch(1)
        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        area.setWidget(inner)
        return area

    @staticmethod
    def _box(title: str, plot: QtWidgets.QWidget):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 6)
        label = Text(title, "dim")
        label.setWordWrap(True)
        lay.addWidget(label)
        lay.addWidget(plot)
        return w

    def _map(self, m, rec):
        plot = Plot("current (mA, measured)", "GHz", 200)
        for leg, color, style in ((0, "dim", "dots"), (1, "accent", "dots"), (2, "ok", "dots"), (3, "warn", "ring")):
            pts = [pt for pt in m["points"] if pt[2] == leg and pt[1] is not None]
            plot.add([pt[0] for pt in pts], [pt[1] / 1e3 for pt in pts], color, style, 2)
        bad = [pt for pt in m["points"] if pt[3] and pt[1] is not None]
        plot.add([pt[0] for pt in bad], [pt[1] / 1e3 for pt in bad], "err", "cross", 1.5)
        bands = m.get("bands", [])
        if bands:  # the best candidate: where its mode lases sweeping its way, and the operating point
            plot.spans.append((bands[0][0], bands[0][1], "ok"))
            plot.vlines.append((bands[0][2], "warn"))
        plot.hlines.append((0.0, "dim"))
        best = (f"; the best: {bands[0][0]:.2f}–{bands[0][1]:.2f} mA sweeping {bands[0][4]} (shaded), its operating "
                f"point dashed" if bands and len(bands[0]) > 4 else "")
        title = (f"map at {m['v']:.2f} V: frequency from the target against current, sweeping up (blue dots), then "
                 f"down (green), and parking (amber rings){best}" + ("; × where it showed two modes" if bad else ""))
        return self._box(title, plot)

    def _catch(self, c):
        """One go of the catch: the frequency from the target against the current, coming down; green where the SHG
        peak was within catch_min of the optimum's, red where it was not, grey not looked at; where the right mode
        is shaded."""
        plot = Plot("current (mA, measured)", "GHz", 180)
        least = c.get("min", 0.9)
        for good, color, style in ((True, "ok", "dots"), (False, "err", "cross"), (None, "dim", "dots")):
            sel = [p for p in c["points"] if p[1] is not None and (p[2] is None if good is None else
                                                                   p[2] is not None and (p[2] >= least) == good)]
            plot.add([p[0] for p in sel], [p[1] / 1e3 for p in sel], color, style, 2)
        right, tol = c.get("right", 0.0) / 1e3, c.get("tol", 0.0) / 1e3
        plot.hspans.append((right - tol, right + tol, "ok"))
        plot.hlines.append((0.0, "dim"))
        last = c["points"][-1] if c["points"] else None
        end = (f"caught at {last[0]:.2f} mA" + (f", its SHG peak {last[2]:.2f} of the optimum's" if last[2] is not None
                                                else "")) if c.get("caught") and last else "not caught"
        return self._box(f"go {c.get('go', '?')} at {c.get('v', 0):.2f} V: frequency from the target against the "
                         f"current, coming down from the top of the window; the right mode shaded; green its SHG peak "
                         f"within {100 * (1 - least):.0f} % of the optimum's, × not, grey not looked at: {end}", plot)

    def _walk(self, pts):
        """The walk: frequency from the target against the scan offset, the mode as the SHG sweep had
        it at each reading; and the current, reading by reading."""
        f = Plot("scan offset (V)", "GHz", 200)
        for ok, color, style in ((True, "ok", "dots"), (False, "err", "cross"), (None, "dim", "dots")):
            sel = [p for p in pts if p[3] is ok and p[2] is not None]
            f.add([p[0] for p in sel], [p[2] / 1e3 for p in sel], color, style, 2)
        f.hlines.append((0.0, "dim"))
        i = Plot("reading", "mA", 140)
        i.add(list(range(len(pts))), [p[1] for p in pts], "accent", "line")
        return [self._box("the walk: frequency from the target against the scan offset; green where the SHG sweep "
                          "showed one mode, × two, grey not looked at", f),
                self._box("the current (measured) at each reading: the trims (a laser that was fine: none)", i)]

    def _sweep(self, s):
        plot = Plot("sample", "V", 120)
        plot.add(list(range(len(s["y"]))), s["y"], "ok" if s.get("ok") else ("err" if s.get("ok") is False else "dim"))
        return self._box(f"SHG intra-cavity sweep, {s.get('why', '')} at {s.get('i', 0):.2f} mA: {s.get('text', '')}",
                         plot)

    def _scan(self, sc):
        plot = Plot("scan output (V)", "lock input", 220)
        if sc.get("x"):
            plot.add(sc["x"], sc["y"], "accent")
        cands = sc.get("cands", [])
        plot.add([c[0] for c in cands], [c[1] for c in cands], "dim", "dots", 3)
        w = sc.get("window_v", 0)
        plot.spans.append((sc["x_pred"] - w, sc["x_pred"] + w, "warn"))
        plot.vlines.append((sc["x_pred"], "warn"))
        choice = sc.get("choice") or [None, None, "", ""]
        if choice[0] is not None:
            plot.add([choice[0]], [choice[1]], "ok", "dots", 5)
        return self._box(f"the cavity scan: lockpoint candidates (grey), where the wavemeter puts the target "
                         f"(dashed, ± the window shaded), the peak picked (green). {choice[3]}", plot)

    def _falc(self, f):
        span = max(4 * abs(f["dc0"]), 0.01)
        plot = Plot("sample", "V", 180, ylim=(-span, span))  # the baseline, not the resonances
        plot.add(list(range(len(f["error"]))), f["error"], "accent")
        plot.hlines.append((0.0, "dim"))
        plot.hlines.append((f["dc0"], "warn"))
        after = ""
        if "offset" in f:
            plot.hlines.append((f["dc"], "ok"))
            after = (f"; input offset {f['offset0']:+.4f} → {f['offset']:+.4f} V took it to "
                     f"{f['dc'] * 1e3:+.1f} mV (green)")
            if f.get("gain"):
                after += f" in {f.get('steps', 0)} steps (the offset moves it {f['gain']:+.3g} V/V)"
        return self._box(f"the error signal over the scan before nulling, close up on its baseline: DC level "
                         f"{f['dc0'] * 1e3:+.1f} mV (amber){after}", plot)


class SetupDialog(QtWidgets.QDialog):
    """The setup check: what a recovery would use on this laser, read (only) from its controller."""

    COLORS = {"ok": C["ok"], "info": C["dim"], "warn": C["warn"], "error": C["err"]}

    def __init__(self, parent, engine: Engine, n: int):
        super().__init__(parent)
        self.engine, self.n = engine, n
        self.setWindowTitle(f"Check setup · channel {n}")
        self.resize(720, 560)
        v = QtWidgets.QVBoxLayout(self)
        self.view = QtWidgets.QTextBrowser()
        v.addWidget(self.view, 1)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Close)
        again = bb.addButton("Read again", QtWidgets.QDialogButtonBox.ButtonRole.ActionRole)
        again.clicked.connect(self.read)
        bb.rejected.connect(self.reject)
        v.addWidget(bb)
        self._shown = object()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.read()

    def read(self):
        self.engine.check_setup(self.n)
        self.refresh()

    def refresh(self):
        lines = self.engine.channels[self.n].setup
        if lines is self._shown:
            return
        self._shown = lines
        if lines is None:
            self.view.setHtml(f"<p style='color:{C['dim']}'>reading the controller…</p>")
            return
        html = []
        for level, text in lines:
            text = text.replace("&", "&amp;").replace("<", "&lt;")
            if level == "head":
                html.append(f"<h3 style='color:{C['accent']}; margin:10px 0 2px 0'>{text}</h3>")
            else:
                mark = {"ok": "✓", "info": "·", "warn": "!", "error": "✗"}.get(level, "·")
                body = C["fg"] if level in ("ok", "info") else ""
                html.append(f"<div style='margin:1px 0 1px 8px; color:{self.COLORS.get(level, C['fg'])}'>"
                            f"{mark} <span style='color:{body}'>{text}</span></div>")
        self.view.setHtml("".join(html))


class ReferenceDialog(QtWidgets.QDialog):
    def __init__(self, parent, ref):
        super().__init__(parent)
        self.setWindowTitle(f"Reference laser · channel {ref.channel}")
        form = QtWidgets.QFormLayout(self)
        p = ref.p

        def row(widget, unit):
            box = QtWidgets.QHBoxLayout()
            box.addWidget(widget)
            box.addWidget(Text(unit, "dim"))
            return box

        def number(value, lo, hi, dec):
            sb = QtWidgets.QDoubleSpinBox()
            sb.setLocale(QtCore.QLocale.c())
            sb.setRange(lo, hi)
            sb.setDecimals(dec)
            sb.setValue(value)
            return sb

        digits = getattr(parent, "digits", 9)
        self.anchor = DigitSpinBox()
        self.anchor.setFont(mono_font())
        if ref.anchor:
            nm = C_NM_THZ / ref.anchor
            self.anchor.setDecimals(decimals(nm, digits))
            self.anchor.setValue(nm)
        self._anchor_shown = self.anchor.value()
        form.addRow("Wavelength", row(self.anchor, "nm, vacuum: the laser's true value, if you know it"))
        self.model = QtWidgets.QComboBox()
        self.model.addItems(["relative: scales with each laser's frequency", "the same offset for every laser"])
        self.model.setCurrentIndex(0 if p.scale else 1)
        form.addRow("Correction", self.model)
        self.tau = number(p.tau, 1, 3600, 0)
        form.addRow("Averaging time", row(self.tau, "s"))
        self.max_drift = number(p.max_drift, 0.1, 1000, 1)
        form.addRow("Wavemeter drifts at most", row(self.max_drift, "MHz/min: anything faster is the reference"))
        self.max_jump = number(p.max_jump, 1, 100000, 0)
        form.addRow("Largest drift over an outage", row(self.max_jump, "MHz: beyond it, you decide"))
        form.addRow(Text(
            "The reference laser is ignored while it jumps, sweeps, runs away or has no light;\n"
            "the other lasers keep the last good correction meanwhile. It is trusted again once\n"
            "steady and back where it should be, give or take what the wavemeter can have drifted.", "dim"))
        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def params(self) -> RefParams:
        return RefParams(self.tau.value(), self.max_drift.value(), self.max_jump.value(),
                         scale=self.model.currentIndex() == 0)

    def new_anchor_thz(self):
        """THz if the wavelength was edited, else None."""
        nm = self.anchor.value()
        return C_NM_THZ / nm if nm > 0 and nm != self._anchor_shown else None


class ShareDialog(QtWidgets.QDialog):
    """Network: this PC's wavemeter shared with the other PCs on the LAN, theirs shown here (share.py)."""

    def __init__(self, parent, share):
        super().__init__(parent)
        self.share = share
        self.setWindowTitle("Network")
        s = share.settings
        form = QtWidgets.QFormLayout(self)

        def row(widget, note):
            box = QtWidgets.QHBoxLayout()
            box.addWidget(widget, 1)
            box.addWidget(Text(note, "dim"))
            return box

        self.on = QtWidgets.QCheckBox("Share this PC's wavemeter with the other PCs, and show theirs here")
        self.on.setChecked(share.on)
        form.addRow(self.on)
        self.label = QtWidgets.QLineEdit(str(s.get("label") or ""))
        self.label.setPlaceholderText(share.default_label)
        form.addRow("Its name", row(self.label, "on every PC, over its lasers"))
        self.control = QtWidgets.QCheckBox("Let the other PCs adjust the lasers here: setpoint, Lock, Relock, "
                                           "Continue, Stop, Recenter, names, lock settings")
        self.control.setChecked(share.control)
        form.addRow(self.control)
        self.group = QtWidgets.QLineEdit(str(s.get("group") or ""))
        self.group.setPlaceholderText("none")
        form.addRow("Group", row(self.group, "only PCs in the same group find each other"))
        self.key = QtWidgets.QLineEdit(str(s.get("key") or ""))
        self.key.setEchoMode(QtWidgets.QLineEdit.EchoMode.PasswordEchoOnEdit)
        self.key.setPlaceholderText("none")
        form.addRow("Key", row(self.key, "the same on every PC: all is signed with it, nothing else is taken"))
        self.peers = QtWidgets.QLineEdit(", ".join(s.get("peers") or []))
        self.peers.setPlaceholderText("lab3-pc, 10.1.2.3")
        form.addRow("Also talk to", row(self.peers, "PCs a broadcast does not reach (another subnet)"))
        self.port = QtWidgets.QSpinBox()
        self.port.setRange(1024, 65535)
        self.port.setValue(share.port)
        form.addRow("Port", row(self.port, "UDP, the same on every PC"))
        self.found = Text("", "dim")
        self.found.setWordWrap(True)
        self.found.setMinimumWidth(520)
        form.addRow("Found", self.found)
        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._show_found)
        self._timer.start(500)
        self._show_found()

    def _show_found(self):
        share, now = self.share, time.monotonic()
        lines = [f"{p.label}   {p.host}  {p.addr[0]}:{p.addr[1]}   {p.status(now)}"
                 + ("" if p.control else ", watch only") for p in share.snapshot()]
        if not lines:
            lines = ["no other PC yet" if share.on else "sharing is off"]
        if share.error:
            lines.append(share.error)
        if share.port_note:
            lines.append(share.port_note)
        lines.append(f"traffic: {share.rate_out / 1e3:.1f} kB/s out, {share.rate_in / 1e3:.1f} kB/s in")
        self.found.paint("\n".join(lines), "dim")

    def values(self) -> dict:
        return {"on": self.on.isChecked(), "label": self.label.text().strip(), "control": self.control.isChecked(),
                "group": self.group.text().strip(), "key": self.key.text(),
                "peers": [a.strip() for a in self.peers.text().replace(";", ",").split(",") if a.strip()],
                "port": self.port.value()}


class Scroll(QtWidgets.QScrollArea):
    """The rows of every wavemeter, scrolled up and down only: never narrower than they are (a longer line under
    the digits widens the window, as it would without), as short as a row or two."""

    def __init__(self):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

    def _width(self, hint) -> int:
        return hint.width() + self.verticalScrollBar().sizeHint().width()

    def minimumSizeHint(self):
        w = self.widget()
        if w is None:
            return super().minimumSizeHint()
        hint = w.minimumSizeHint()
        return QtCore.QSize(self._width(hint), min(hint.height(), 120))

    def sizeHint(self):
        w = self.widget()
        return super().sizeHint() if w is None else QtCore.QSize(self._width(w.sizeHint()), w.sizeHint().height())

    def eventFilter(self, obj, ev):
        if obj is self.widget() and ev.type() == QtCore.QEvent.Type.LayoutRequest:
            self.updateGeometry()  # (its rows' sizes changed: the window's layout asks again)
        return super().eventFilter(obj, ev)


class MainWindow(QtWidgets.QWidget):
    def __init__(self, engine: Engine, save, share=None):
        """share: a share.Share, to show the other PCs' wavemeters too (None: this PC's alone)."""
        super().__init__()
        self.engine, self.save, self.share = engine, save, share
        self.gui = engine.config.setdefault("gui", {})
        self.digits = int(self.gui.get("digits", 9))
        self.setWindowTitle("Wavemeter Lock")
        self.mono_font = mono_font()
        self.bold_font = QtGui.QFont(self.font())
        self.bold_font.setBold(True)
        self._make_fonts(int(self.gui.get("font_pt", 26)))

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 8, 8, 4)
        v.setSpacing(6)
        self.area = Scroll()  # (every wavemeter's rows: they may not all fit on the screen)
        inner = QtWidgets.QWidget()
        self.panels_box = QtWidgets.QVBoxLayout(inner)
        self.panels_box.setContentsMargins(0, 0, 0, 0)
        self.panels_box.setSpacing(10)
        self.local = Panel(self)  # this PC's wavemeter
        self.rows_box = self.local.rows_box
        viewer = getattr(engine.source, "viewer", False)
        self.placeholder = Text("no wavemeter on this PC: the other PCs' show here once found" if viewer else
                                "waiting for the wavemeter…", "dim")
        self.placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder.setMinimumHeight(60)
        self.local.layout().addWidget(self.placeholder)
        self.panels_box.addWidget(self.local)
        self.panels_box.addStretch(1)
        self.area.setWidget(inner)
        self.area.viewport().setAutoFillBackground(False)
        inner.setAutoFillBackground(False)
        v.addWidget(self.area, 1)
        status = QtWidgets.QHBoxLayout()
        status.setSpacing(18)
        self.st_wlm = Text("", "dim")
        self.st_ref = Text("", "dim")
        self.st_net = Text("", "dim")
        self.st_lan = Text("", "dim")
        status.addWidget(self.st_wlm)
        status.addWidget(self.st_ref)
        status.addStretch(1)
        status.addWidget(self.st_net)
        status.addWidget(self.st_lan)
        v.addLayout(status)

        self.rows = {}
        self.row_list = []
        self.panels = {}       # another PC's id -> the Panel of its wavemeter
        self.panel_list = []
        self.colors = {}       # a wavemeter's name -> the colour of its frame
        self._peer_ids = None
        self._fit_at = 0.0
        self._layout_version = None
        self._slow_at = 0.0
        self._saved_at = time.monotonic()
        self.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_menu)

        if self.gui.get("geometry"):
            self.restoreGeometry(QtCore.QByteArray.fromHex(self.gui["geometry"].encode()))
        else:
            self.resize(760, 200)
        if self.gui.get("on_top"):
            self.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint, True)

        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(REFRESH_MS)

    def _make_fonts(self, pt: int):
        pt = max(10, min(pt, 96))
        self.gui["font_pt"] = pt
        self.big_font = QtGui.QFont(self.mono_font)
        self.big_font.setPointSize(pt)
        self.mid_font = QtGui.QFont(self.mono_font)
        self.mid_font.setPointSize(max(9, pt // 2))

    def set_font_size(self, pt: int):
        self._make_fonts(pt)
        for row in self.all_rows():
            row.apply_fonts()
        self.engine.dirty = True
        self.fit_later()

    def all_rows(self) -> list:
        """This PC's rows and the other PCs'."""
        return self.row_list + [row for panel in self.panel_list for row in panel.row_list]

    # ------------------------------------------------------------- refresh
    def refresh(self):
        e = self.engine
        if e.layout_version != self._layout_version:
            self._rebuild()
        now = time.monotonic()
        if self.share is not None:
            self._sync_panels()
        if not self.isMinimized():
            for row in self.row_list:
                row.refresh(now)
            for panel in self.panel_list:
                panel.refresh(now)
        if self._fit_at and now >= self._fit_at:
            self._fit()
        if now >= self._slow_at:
            self._slow_at = now + 0.5
            self._slow(now)

    def _slow(self, now: float):
        e = self.engine
        e.update_rate()
        e.tick()
        if e.rows() != [row.n for row in self.row_list]:  # (a channel switched off, watched no more)
            self._rebuild()
        ref = e.reference
        if ref is None:
            self.st_ref.paint("", "dim")
        else:
            text = {TRACKING: f"ref ch{ref.channel}  wavemeter {ref.error:+.1f} MHz",
                    HOLD: f"ref ch{ref.channel}  hold {fmt_duration(now - ref.since)}",
                    MOVED: f"ref ch{ref.channel}  moved: right-click it",
                    ACQUIRING: f"ref ch{ref.channel}  acquiring"}[ref.state]
            self.st_ref.paint(text, {TRACKING: "accent", MOVED: "err"}.get(ref.state, "warn"))
        if getattr(e.source, "viewer", False):
            self.st_wlm.paint("no wavemeter on this PC", "dim")
        elif e.source_ok:
            label = getattr(e.source, "label", "")
            self.st_wlm.paint(f"● {label}   {len(self.row_list)} ch   {e.rate:.0f} readings/s", "dim")
        else:
            self.st_wlm.paint(f"● {e.source_msg}", "err")
        if e.scanning:
            self.st_lan.paint("scanning LAN…", "dim")
        else:
            n = sum(1 for d in e.device_list() if not d.error)
            self.st_lan.paint(f"{n} controller{'s' if n != 1 else ''} on LAN", "dim")
        if self.share is not None:
            self._slow_share(now)
        for row in self.all_rows():
            row.update_tooltip()
        if (e.dirty and now - self._saved_at > 1.0) or (ref is not None and now - self._saved_at > 300):
            self._save()  # the reference's estimate too, now and then

    def _rebuild(self):
        e = self.engine
        with e.lock:
            self._layout_version = e.layout_version
            wanted = e.rows()
        for n in list(self.rows):
            if n not in wanted:
                row = self.rows.pop(n)
                row.hide()
                row.deleteLater()
        for row in self.rows.values():
            self.rows_box.removeWidget(row)
        for n in wanted:
            if n not in self.rows:
                self.rows[n] = ChannelRow(self, n)
            self.rows_box.addWidget(self.rows[n])
        self.row_list = [self.rows[n] for n in wanted]
        self.placeholder.setVisible(not wanted)
        self.fit_later()

    # --------------------------------------------------------- the other PCs
    def _sync_panels(self):
        """A panel for each other PC found (share.py), by name (not one without a wavemeter: --view); this PC's
        framed too while there are any."""
        peers = [p for p in self.share.snapshot() if not p.src.get("viewer")]
        ids = [p.id for p in peers]
        if ids == self._peer_ids:
            return
        self._peer_ids = ids
        for pid in list(self.panels):
            if pid not in ids:
                panel = self.panels.pop(pid)
                panel.hide()
                panel.deleteLater()
        for panel in self.panels.values():
            self.panels_box.removeWidget(panel)
        folded = self.gui.get("folded") or []
        for k, p in enumerate(peers):
            panel = self.panels.get(p.id)
            if panel is None:
                panel = self.panels[p.id] = Panel(self, p)
                if p.label in folded:
                    panel.fold(True)
            self.panels_box.insertWidget(1 + k, panel)
        self.panel_list = [self.panels[pid] for pid in ids]
        self.local.set_framed(bool(peers))
        self.local.setVisible(not (peers and getattr(self.engine.source, "viewer", False)))
        self._slow_share(time.monotonic())
        self.fit_later()

    def _slow_share(self, now: float):
        share = self.share
        if not share.on:
            self.st_net.paint("", "dim")
        elif share.error:
            self.st_net.paint(f"⇄ {share.error}", "err")
        else:
            k = sum(1 for panel in self.panel_list if panel.peer.status(now) in ("live", "late"))
            self.st_net.paint(f"⇄ {k} other PC{'s' if k != 1 else ''} · {(share.rate_out + share.rate_in) / 1e3:.1f} "
                              f"kB/s", "warn" if share.port_note else "dim")
            self.st_net.setToolTip(share.port_note or "The other PCs found on the LAN, and the traffic to and from "
                                                      "them: right-click › Network…")
        self.colors = panel_colors([share.label] + [panel.peer.label for panel in self.panel_list])
        self.local.set_label(share.label)
        self.local.header.detail.paint("this PC" + ("" if share.control else "   the others may only watch"), "dim")
        title = f"Wavemeter Lock · {share.label}" if self.panel_list else "Wavemeter Lock"
        if self.windowTitle() != title:
            self.setWindowTitle(title)
        for panel in self.panel_list:
            self.want(panel)

    def want(self, panel: "Panel"):
        """What this PC wants of that one: its rows as they change, or (folded, minimized) now and then."""
        if self.share is not None:
            self.share.want(panel.peer.id, "slow" if panel.folded or self.isMinimized() else "fast")

    def fit_later(self):
        """Fit a moment from now: rows just added show only then."""
        self._fit_at = time.monotonic() + 0.1

    def _fit(self):
        """Taller if the rows do not all show (as far as the screen goes)."""
        self._fit_at = 0.0
        inner = self.area.widget()
        inner.layout().activate()
        need = inner.sizeHint().height() - self.area.viewport().height()
        if need <= 0 or not self.isVisible() or self.isMaximized() or self.isFullScreen():
            return
        screen = self.screen() if hasattr(self, "screen") else QtGui.QGuiApplication.primaryScreen()
        if screen is None:
            return
        avail = screen.availableGeometry()
        frame = self.frameGeometry()
        extra = frame.height() - self.height()
        height = min(self.height() + need, avail.height() - extra)
        if height > self.height():
            self.resize(self.width(), height)
            if frame.top() + height + extra > avail.bottom():
                self.move(frame.left(), max(avail.top(), avail.bottom() - height - extra))

    def share_dialog(self):
        if self.share is None:
            return
        d = ShareDialog(self, self.share)
        if d.exec():
            self.share.settings.update(d.values())
            self.share.configure()
            self.engine.dirty = True

    def _save(self):
        self.gui["geometry"] = bytes(self.saveGeometry().toHex()).decode()
        self._saved_at = time.monotonic()
        try:
            self.save()
        except OSError as exc:
            self.st_lan.paint(f"cannot save settings: {exc}", "err")

    # ------------------------------------------------------------- actions
    def show_menu(self, pos):
        e = self.engine
        m = QtWidgets.QMenu(self)
        m.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        m.addAction("Rescan LAN   F5", e.start_discovery)
        if self.share is not None:
            m.addAction("Network…", self.share_dialog)
        hidden = [ch for n, ch in sorted(e.channels.items()) if ch.hidden and n in e.active]
        if hidden:
            sub = m.addMenu("Show hidden channel")
            for ch in hidden:
                sub.addAction(f"{ch.n}  {ch.name}", functools.partial(e.set_hidden, ch.n, False))
        a = m.addAction("Always on top")
        a.setCheckable(True)
        a.setChecked(bool(self.gui.get("on_top")))
        a.toggled.connect(self.set_on_top)
        m.addAction("Larger digits   Ctrl +", lambda: self.set_font_size(self.gui["font_pt"] + 2))
        m.addAction("Smaller digits   Ctrl −", lambda: self.set_font_size(self.gui["font_pt"] - 2))
        laser = getattr(e.source, "cavity_laser", lambda: None)()
        if laser is not None:  # --sim only: try the reference and mode-hop logic out
            sim = m.addMenu("Simulate a fault")
            sim.addAction("Reference mode hop +1.5 GHz for 30 s", lambda: laser.mode_hop(1500, 30))
            sim.addAction("Reference mode hop +1.5 GHz, stays", lambda: laser.mode_hop(1500))
            sim.addAction("Reference cavity unlocked for 20 s", lambda: laser.unlock(20))
            sim.addAction("Reference light blocked for 15 s", lambda: laser.dark(15))
            sim.addAction("Wavemeter jumps +20 MHz", lambda: e.source.wavemeter.jump(20e-6 / laser.f0))
            sim.addSeparator()
            for las in e.source.lasers:
                if las.ecdl is None:
                    continue
                name = e.channels[las.ch].name if las.ch in e.channels else ""
                who = f"Channel {las.ch}" + (f" {name}" if name else "")
                sim.addAction(f"{who}: mode hop up", functools.partial(las.hop_mode, 1))
                sim.addAction(f"{who}: mode hop down", functools.partial(las.hop_mode, -1))
                if las.ecdl.i_stop is not None:
                    sim.addAction(f"{who}: stops lasing", functools.partial(las.stop_lasing, 0.0))
                    sim.addAction(f"{who}: stops lasing, its seed power hardly drops (broadband)",
                                  functools.partial(las.stop_lasing, 0.7))
                if e.source.use.get(las.ch, True):
                    sim.addAction(f"{who}: someone unticks it on the wavemeter", functools.partial(e.source.untick,
                                                                                                    las.ch))
                else:
                    sim.addAction(f"{who}: someone ticks it again on the wavemeter", functools.partial(e.source.tick,
                                                                                                        las.ch))
                if las.lock is not None:
                    sim.addAction(f"{who}: cavity lock jumps a peak", functools.partial(las.lock.jump, 1))
                    sim.addAction(f"{who}: cavity lock jumps 60 MHz", functools.partial(las.lock.jump, 0.04))
                    sim.addAction(f"{who}: cavity lock knocked out, ReLock catches the same peak",
                                  functools.partial(las.lock.knock, 1.5))
                    sim.addAction(f"{who}: cavity lock drops out for 3 ms", functools.partial(las.lock.blip, 0.003))
                    sim.addAction(f"{who}: goes multimode, without a hop (its cavity lock keeps losing it)",
                                  functools.partial(las.go_multimode, 1))
                    sim.addAction(f"{who}: error signal DC drifts +20 mV",
                                  functools.partial(setattr, las.lock, "pdh_dc", las.lock.pdh_dc + 0.02))
        self._menu = m
        m.popup(self.mapToGlobal(pos))

    def confirm(self, title: str, text: str) -> bool:
        box = QtWidgets.QMessageBox.question(self, title, text)
        return box == QtWidgets.QMessageBox.StandardButton.Yes

    def use_as_reference(self, n: int):
        if self.engine.channels[n].state != "off" and not self.confirm(
                "Reference laser", f"Channel {n} is locked to the wavemeter. Make it the reference and unlock it?"):
            return
        self.engine.set_reference(n)

    def stop_reference(self):
        shift = self.engine.locked_shift_mhz(lambda f: f)
        if shift > 1.0 and not self.confirm(
                "Stop referencing", f"Without the reference the wavemeter's own drift comes back:\n"
                                    f"locked lasers will move by up to {shift:.1f} MHz. Stop referencing?"):
            return
        self.engine.clear_reference()

    def accept_drift(self):
        ref = self.engine.reference
        if ref is None or ref.state != MOVED:
            return
        shift = self.engine.locked_shift_mhz(lambda f: ref.preview(f, accept=True))
        if self.confirm("Wavemeter moved?",
                        f"Treat the reference's {ref.offset:+.1f} MHz as wavemeter drift?\n"
                        f"Only right if the reference laser is still on the same cavity mode.\n"
                        f"Locked lasers will move by up to {shift:.1f} MHz."):
            self.engine.reference_accept_drift()

    def reference_dialog(self):
        ref = self.engine.reference
        if ref is None:
            return
        d = ReferenceDialog(self, ref)
        if not d.exec():
            return
        params, thz = d.params(), d.new_anchor_thz()
        if thz is not None or params.scale != ref.p.scale:
            shift = self.engine.locked_shift_mhz(lambda f: ref.preview(f, params.scale, thz))
            if shift > 1.0 and not self.confirm(
                    "Reference", f"Locked lasers keep their setpoints in the new frame:\n"
                                 f"they will move by up to {shift:.1f} MHz. Continue?"):
                return
        self.engine.set_reference_params(params)
        if thz is not None:
            self.engine.set_reference_anchor(thz)

    def set_on_top(self, on: bool):
        self.gui["on_top"] = on
        self.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint, on)
        self.show()
        self.engine.dirty = True

    def keyPressEvent(self, ev):
        ctrl = ev.modifiers() & Qt.KeyboardModifier.ControlModifier
        if ctrl and ev.key() in (Qt.Key.Key_Plus, Qt.Key.Key_Equal):
            self.set_font_size(self.gui["font_pt"] + 2)
        elif ctrl and ev.key() == Qt.Key.Key_Minus:
            self.set_font_size(self.gui["font_pt"] - 2)
        elif ev.key() == Qt.Key.Key_F5:
            self.engine.start_discovery()
        else:
            super().keyPressEvent(ev)

    def closeEvent(self, ev):
        self.timer.stop()
        self._save()
        if self.share is not None:  # (first: no adjustments from the others while it closes)
            self.share.stop()
            self.share.join(1.0)
        self.engine.stop()
        super().closeEvent(ev)


def run(engine: Engine, save, share=None) -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName("Wavemeter Lock")
    app.setStyle("Fusion")
    app.setPalette(dark_palette())
    app.setStyleSheet(QSS)
    win = MainWindow(engine, save, share)
    for sig in (signal.SIGINT, signal.SIGTERM):  # Ctrl+C / kill: close cleanly, settings saved
        signal.signal(sig, lambda *_: win.close())
    engine.start()
    if share is not None:
        share.start()
    win.show()
    code = app.exec()
    win.deleteLater()  # the window gone before the interpreter takes Qt down (else PySide6 may crash on exit)
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    if hasattr(app, "shutdown"):  # PySide6
        app.shutdown()
    return code
