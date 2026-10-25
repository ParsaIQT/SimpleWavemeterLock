"""Minimal Qt front-end.

Everything slow lives in the engine's threads. This module repaints changed
numbers on a 30 Hz timer and forwards clicks, so the window never stalls.

With other PCs about (share.py), each wavemeter's rows are framed under its
name: this PC's first, then the others', painted from what those PCs send (the
same view.py that paints the rows here) and adjusted by asking them.
"""
from __future__ import annotations

import functools
import html
import math
import signal
import sys
import time
import types
import zlib
from pathlib import Path

from . import dac, view
from .engine import DAC_LOCK, Engine
from .lasermap import (CLIP, FF, FF_ON, FOLDER, LOCK_ON, PC_MAX, PC_MIN, PD_NAMES, PD_UNITS, STEP_I, STEP_V, TA_ON,
                       TEMP_MAX, TEMP_MIN, TEMP_SET, MapPlan, fmt_bytes)
from .lock import LockParams
from .meters import Meter
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
QFrame#row[state="mapping"] {{ border-left-color: {C['accent']}; }}
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
QPushButton#lock[state="mapping"] {{ background: #1d4f80; border-color: {C['accent']}; }}
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


def scrolling(body: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
    """body in a scroll area of its own, borderless: for a dialog that may not all fit on the screen (its buttons
    below it, not in it, stay in view)."""
    area = QtWidgets.QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
    area.setWidget(body)
    return area


def fit_screen(window: QtWidgets.QWidget, width: int, height: int):
    """Resize a window to width x height, or as much of that as the screen it opens on has room for (its title bar
    and the taskbar left)."""
    parent = window.parentWidget()
    screen = parent.window().screen() if parent is not None and hasattr(parent, "screen") else None
    free = (screen or QtGui.QGuiApplication.primaryScreen()).availableGeometry()
    window.resize(max(min(width, free.width() - 40), 200), max(min(height, free.height() - 80), 200))


def long_items(combo: QtWidgets.QComboBox, chars: int = 50):
    """A combo box of long items no wider than `chars` characters (the one chosen cut short there), each item whole
    in its tooltip and in the list it opens (as wide as the longest, if the screen is)."""
    combo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
    combo.setMinimumContentsLength(chars)
    fm = combo.fontMetrics()
    for k in range(combo.count()):
        combo.setItemData(k, combo.itemText(k), Qt.ItemDataRole.ToolTipRole)
    longest = max((fm.horizontalAdvance(combo.itemText(k)) for k in range(combo.count())), default=0)
    screen = QtGui.QGuiApplication.primaryScreen().availableGeometry()
    combo.view().setMinimumWidth(min(longest + 40, int(0.9 * screen.width())))


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
    LOCK_TIP = "Lock to the wavemeter: the piezo (its scan offset, as a rule) follows the reading onto the setpoint."

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
        elif state == "mapping":
            label = "Mapping"
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
    """A row of a wavemeter of this PC (its engine's)."""

    def __init__(self, win: "MainWindow", n: int, engine: Engine | None = None):
        self.engine = engine if engine is not None else win.engine
        super().__init__(win, n)

    def refresh(self, now: float):
        v = view.row(self.engine, self.n, now)
        self.paint(v, v["age"])

    def _clicked(self):
        ch = self.engine.channels[self.n]
        if ch.state == "mapping":
            self.win.map_window(self.engine, self.n)
            if self.win.confirm("Stop the map", "Stop the laser map? The laser goes back where it began; what it "
                                                "mapped so far stays in its file."):
                self.engine.stop_map(self.n)
        elif ch.state == "recovering" and ch.hop_prompt:
            self.engine.confirm_step(self.n)
        elif ch.state == "recovering":
            self.engine.cancel_recovery(self.n)
        elif self._is_ref:
            self.win.reference_dialog(self.engine)
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
            m.addAction("Reference settings…", lambda: self.win.reference_dialog(e))
            if ref.state == MOVED:
                m.addAction("The reference laser moved: follow it", e.reference_reanchor)
                m.addAction(f"The wavemeter moved: accept {ref.offset:+.1f} MHz as drift",
                            lambda: self.win.accept_drift(e))
            m.addAction("Stop using as reference", lambda: self.win.stop_reference(e))
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
                    lambda: self.win.use_as_reference(self.n, e))
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
        if ch.state == "mapping":
            m.addAction("Laser map (going on)…", lambda: self.win.map_window(self.engine, self.n))
            m.addAction("Stop the laser map", lambda: self.engine.stop_map(self.n))
        else:
            m.addAction("Laser map…", self.map_dialog).setEnabled(bool(ch.device) and ch.state == "off")
            if ch.map_last is not None:
                m.addAction("Last laser map…", lambda: self.win.map_window(self.engine, self.n))
        m.addAction("Its laser's model from a map…", self.model_dialog).setEnabled(bool(ch.device) and not ch.dac)
        if ch.hop.enabled and ch.hop.shg:
            m.addAction("Take the optimum now (as set by hand)", lambda: self.engine.take_optimum(self.n)).setEnabled(
                bool(ch.device) and ch.state != "recovering")
        if ch.last:
            m.addAction("Last recovery…", self.last_dialog)

    def _link(self, key: str, laser: int, *_):
        if self.win.may_link(self.engine, key, laser):
            self.engine.link(self.n, key, laser)

    def link_dialog(self):
        d = LinkDialog(self, self.engine, self.engine.channels[self.n])
        if d.exec():
            host, laser = d.values()
            if host and self.win.may_link(self.engine, host, laser):
                self.engine.link(self.n, host, laser)

    def dac_dialog(self):
        ch = self.engine.channels[self.n]
        d = DacDialog(self, ch)
        if d.exec():
            v = d.values()
            if not self.win.may_link(self.engine, v["device"], v["ao"], v["ff_ao"]):
                return
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

    def map_dialog(self):
        d = MapDialog(self, self.engine, self.n)
        if d.exec():
            self.win.map_window(self.engine, self.n)

    def model_dialog(self):
        """Its laser's model from a map's file (made before, or elsewhere): its recoveries go by it."""
        from .lasermap import FOLDER

        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "A map of its laser", str(FOLDER),
                                                        "Laser maps (*.sqlite);;All files (*)")
        if path:
            QtWidgets.QMessageBox.information(self, "Its laser's model", self.engine.model_from_file(self.n, path))

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
    """A wavemeter's name over its rows; framed, it folds away with a click."""

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
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Click to fold it away, or back; right-click for more")

    def mousePressEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton and self.panel.framed:
            self.panel.fold(not self.panel.folded)
        super().mousePressEvent(ev)


class Panel(QtWidgets.QFrame):
    """One wavemeter's rows, framed in its colour under its name while there are others about (other PCs', this
    PC's others): one of this PC's (meter: a meters.Meter, its engine's rows), or another PC's (peer: a
    share.Peer)."""

    WORDS = {"off": "off", "error": "unlocked", "engaging": "locking", "recovering": "recentering",
             "asking": "waiting for you", "ref": "reference", "ref-hold": "reference on hold",
             "ref-moved": "reference moved"}

    def __init__(self, win: "MainWindow", peer=None, meter=None):
        super().__init__()
        self.win, self.peer, self.meter = win, peer, meter
        self.engine = meter.engine if meter is not None else None
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
        self.placeholder = None
        if meter is not None:
            viewer = getattr(self.engine.source, "viewer", False)
            self.placeholder = Text("no wavemeter on this PC: the other PCs' show here once found" if viewer else
                                    "waiting for the wavemeter…", "dim")
            self.placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.placeholder.setMinimumHeight(60)
            lay.addWidget(self.placeholder)
        self.set_framed(peer is not None)

    @property
    def label(self) -> str:
        """Its wavemeter's name."""
        if self.peer is not None:
            return self.peer.label or self.peer.host
        return self.meter.label

    def set_framed(self, framed: bool):
        if framed == self.framed:
            return
        self.framed = framed
        self.header.setVisible(framed)
        self.layout().setContentsMargins(*((8, 4, 8, 8) if framed else (0, 0, 0, 0)))
        if not framed and self.folded:
            self.fold(False)
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
        if self.framed:
            m.addAction("Unfold" if self.folded else "Fold away", lambda: self.fold(not self.folded))
        if p is not None:
            if p.status(time.monotonic()) in ("lost", "closed"):
                m.addAction(f"Forget {p.label}", lambda: self.win.share.forget(p.id))
        else:  # this PC's: its settings with another of this PC's wavemeters, or hidden
            for other in self.win.locals:
                if other is not self:
                    m.addAction(f"Swap settings with {other.label}…", functools.partial(self.win.swap, self, other))
            if self is not self.win.local and self.win.meters is not None:
                m.addAction("Hide this wavemeter…", functools.partial(self.win.hide_meter, self))
        if not m.isEmpty():
            m.addSeparator()
        m.addAction("Network…", functools.partial(self.win.share_dialog, self.meter.share if self.meter else None))
        m.popup(self.header.mapToGlobal(pos))

    def fold(self, folded: bool):
        """Fold its rows away (or back); another PC's sends this PC its rows every few seconds only, meanwhile."""
        self.folded = folded
        for row in self.row_list:
            row.setVisible(not folded)
        if self.placeholder is not None:
            self.placeholder.setVisible(not folded and not self.row_list)
        label = self.label
        names = self.win.gui.setdefault("folded", [])
        if folded and label not in names:
            names.append(label)
        elif not folded and label in names:
            names.remove(label)
        self.win.engine.dirty = True
        if self.peer is not None:
            self.win.want(self)
        self.win.fit_later()

    def refresh(self, now: float):
        if self.peer is None:
            self._refresh_local(now)
            return
        p = self.peer
        if p.layout != self._layout:
            self._rebuild()
        self.set_label(self.label)
        h = self.header
        h.arrow.paint("▸" if self.folded else "▾", "dim")
        where = p.host + (f"  {p.addr[0]}" if p.addr else "") + ("" if p.control else "   watch only")
        if self.win.share is not None and p.host == self.win.share.host:
            where = "this PC, another copy of the program" + ("" if p.control else "   watch only")
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
        if self.peer is None:
            views = [view.row(self.engine, n, time.monotonic()) for n in self.engine.rows()]
        else:
            views = [self.peer.views.get(n) or {} for n in self.peer.rows]
        counts = {}
        for v in views:
            state = v.get("state") or "off"
            word = ("watching" if v.get("relock") else "locked") if state == "on" else self.WORDS.get(state, state)
            counts[word] = counts.get(word, 0) + 1
        k = len(views)
        return f"{k} channel{'s' if k != 1 else ''}" + (": " + ", ".join(f"{c} {w}" for w, c in counts.items())
                                                        if counts else "")

    def _rebuild(self):
        p = self.peer
        self._layout = p.layout
        self._place(list(p.rows), lambda n: RemoteRow(self.win, self, n))

    def _place(self, wanted: list, make):
        """Rows for the channels wanted, in that order: those there already kept, the others made."""
        for n in list(self.rows):
            if n not in wanted:
                row = self.rows.pop(n)
                row.hide()
                row.deleteLater()
        for row in self.rows.values():
            self.rows_box.removeWidget(row)
        for n in wanted:
            if n not in self.rows:
                self.rows[n] = make(n)
            self.rows_box.addWidget(self.rows[n])
            self.rows[n].setVisible(not self.folded)
        self.row_list = [self.rows[n] for n in wanted]
        self.win.fit_later()

    # ------------------------------------------------------- this PC's
    def rebuild_rows(self):
        """A row for each channel its engine shows."""
        e = self.engine
        with e.lock:
            self._layout = e.layout_version
            wanted = e.rows()
        self._place(wanted, lambda n: ChannelRow(self.win, n, e))
        self.placeholder.setVisible(not wanted and not self.folded)

    def _refresh_local(self, now: float):
        e = self.engine
        if e.layout_version != self._layout:
            self.rebuild_rows()
        if not self.folded and not self.win.isMinimized():
            for row in self.row_list:
                row.refresh(now)

    def paint_header(self, several: bool):
        """Its name, 'this PC', and how its wavemeter is read (with others of this PC's about)."""
        e, share = self.engine, self.meter.share
        h = self.header
        h.arrow.paint("▸" if self.folded else "▾", "dim")
        self.set_label(self.label)
        where = "this PC" + ("" if share is None or share.control else "   the others may only watch")
        h.detail.paint(f"{where}   {self.summary()}" if self.folded else where, "dim")
        if not several:
            h.status.paint("", "dim")
        elif getattr(e.source, "viewer", False):
            h.status.paint("no wavemeter", "dim")
        elif e.source_ok:
            name = getattr(e.source, "name", "")
            name = "" if name in self.label else name + "   "  # (its name says what it is already, unless named)
            h.status.paint(f"● {name}{e.rate:.0f} readings/s", "dim")
        else:
            h.status.paint(f"● {e.source_msg}", "err")


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
                    "Positive gains if the frequency rises with the offset: a lock that runs\n"
                    "away straight after it is engaged reverses them by itself (and says so).\n"
                    "I ≈ 0.3 / tuning (GHz/V) is a good start.", "dim")
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
    METHOD = (("auto", "its model, if it has one and no SHG stage; else catch with an SHG stage, else map"),
              ("catch", "catch: back onto its own mode by the current (up, then down until it is on it: its SHG peak "
                        "back; already on it, its current to the top end of where the mode is at its best; with the "
                        "wavemeter alone, parked in the middle of where its mode holds), then the piezo onto the "
                        "target"),
              ("walk", "walk: single mode first, then the piezo slowly onto the target, as by hand"),
              ("map", "map: sweep the current over the window, up and down, and pick the best mode"),
              ("model", "model: its laser's model, fitted to its last map, says where a mode is on target and holds "
                        "best (its own, or another at another piezo voltage) and which way to come to it; a map here "
                        "first, to see how far its modes moved"))

    def __init__(self, parent, ch, take=None):
        """take: take the optimum now (Engine.take_optimum for this channel), if it can be."""
        super().__init__(parent)
        self.setWindowTitle(f"Mode-hop recovery · channel {ch.n}")
        self.ch = ch
        p = self._p = ch.hop
        outer = QtWidgets.QVBoxLayout(self)
        body = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(body)
        area = scrolling(body)  # (it may not all fit on the screen: OK and Cancel, below it, do)
        outer.addWidget(area, 1)

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
        long_items(self.method)
        form.addRow("Method", self.method)
        md = getattr(ch, "model_dict", None)
        has = md is not None and md.get("laser") == [ch.device, ch.laser]
        self.model_info = Text(f"its laser's model: from {md.get('from', '?')} ({md.get('t', '?')}), "
                               f"{md.get('right', 0):.0%} of that map's points on the right mode" if has else
                               "its laser's model: none yet (map it: Laser map…, its model fitted at the end)", "dim")
        self.model_info.setWordWrap(True)
        form.addRow("", self.model_info)
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
        self.reset_up = number(p.reset_up, 0, 10, 1, 0.5)
        self.reset_slow = number(p.reset_slow, 0, 10, 1, 0.5)
        self.reset_tries = QtWidgets.QSpinBox()
        self.reset_tries.setRange(1, 5)
        self.reset_tries.setValue(p.reset_tries)
        form.addRow("Reset", row("after a hop, first: piezo and current to nominal, up", self.reset_up,
                                 "mA, down 1 mA at a time to +", self.reset_slow, "mA, then a step at a time;",
                                 self.reset_tries, "times"))
        self.reset_up.setToolTip("With an SHG stage, as by hand: the piezo to the optimum's voltage and the current to "
                                 "the optimum's, then the current\nthis far up, down 1 mA at a time, and on down to "
                                 "the optimum's a step at a time, a reading each: coming\nfrom that far above, it "
                                 "lands on its right mode there, as a rule. Then on as ever: the current to the top "
                                 "end\nof where the mode is at its best, the piezo onto the target. Only if that does "
                                 "not bring it back,\nthe goes up and down. 0: no reset.")
        self.ease_step = number(p.ease_step, 0, 0.1, 3, 0.001)
        self.upkeep_back = number(p.upkeep_back, 0, 0.5, 3, 0.005)
        self.upkeep = number(p.upkeep, 0, 1440, 0, 5)
        form.addRow("Top of its mode", row("its SHG signal dipping: down", self.ease_step, "mA at a time, kept",
                                           self.upkeep_back, "mA below; looked for every", self.upkeep,
                                           "min, locked"))
        self.ease_step.setToolTip("At the top of its mode a second mode begins, and its SHG signal (locked) dips "
                                  "below what the mode gives:\na warning of a hop. Relocked, its SHG cavity locked "
                                  "and its signal below usual, the current goes down\nthis much at a time while the "
                                  "signal rises, and is left that much below where it was back up. 0: not.")
        self.upkeep.setToolTip("Every this many minutes, while it is locked and steady (both locks left on): the "
                               "current up in small steps\nuntil its SHG signal dips (the top of its mode), then down "
                               "until it is back up, and kept that much below.\nAnything amiss: the current back "
                               "where it was. 0: never.")
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
        outer.addWidget(bb)
        self.who.currentIndexChanged.connect(self._who)
        self._who()
        want, m = body.sizeHint(), outer.contentsMargins()
        fit_screen(self, want.width() + m.left() + m.right() + area.verticalScrollBar().sizeHint().width(),
                   want.height() + m.top() + m.bottom() + outer.spacing() + bb.sizeHint().height())

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
            catch_tries=self.catch_tries.value(), pc_try=self.pc_try.value(), reset_up=self.reset_up.value(),
            reset_slow=self.reset_slow.value(), reset_tries=self.reset_tries.value(),
            ease_step=round(self.ease_step.value(), 4), upkeep=self.upkeep.value(),
            upkeep_back=round(self.upkeep_back.value(), 4))
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
        fit_screen(self, 760, 560)
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
        lay.addWidget(plot, 1)
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
        shg = any(p[2] is not None for p in c["points"])
        return self._box(f"go {c.get('go', '?')} at {c.get('v', 0):.2f} V: frequency from the target against the "
                         f"current, coming down from the top of the window; the right mode shaded; "
                         + (f"green its SHG peak within {100 * (1 - least):.0f} % of the optimum's, × not, grey not "
                            f"looked at: " if shg else "by the wavemeter alone: ") + end, plot)

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


# ------------------------------------------------------------------ laser maps
def _oklab(hexcolor: str):
    r, g, b = (int(hexcolor[k:k + 2], 16) / 255.0 for k in (1, 3, 5))
    r, g, b = ((c / 12.92) if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in (r, g, b))
    lms = (0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b, 0.2119034982 * r + 0.6806995451 * g
           + 0.1073969566 * b, 0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b)
    l_, m_, s_ = (math.copysign(abs(x) ** (1 / 3), x) for x in lms)
    return (0.2104542553 * l_ + 0.7936177850 * m_ - 0.0040720468 * s_,
            1.9779984951 * l_ - 2.4285922050 * m_ + 0.4505937099 * s_,
            0.0259040371 * l_ + 0.7827717662 * m_ - 0.8086757660 * s_)


def _srgb(lab) -> tuple:
    L, a, b = lab
    l_, m_, s_ = L + 0.3963377774 * a + 0.2158037573 * b, L - 0.1055613458 * a - 0.0638541728 * b, \
        L - 0.0894841775 * a - 1.2914855480 * b
    l3, m3, s3 = l_ ** 3, m_ ** 3, s_ ** 3
    rgb = (4.0767416621 * l3 - 3.3077115913 * m3 + 0.2309699292 * s3,
           -1.2684380046 * l3 + 2.6097574011 * m3 - 0.3413193965 * s3,
           -0.0041960863 * l3 - 0.7034186147 * m3 + 1.7076147010 * s3)
    return tuple(int(round(255 * min(max((12.92 * c) if c <= 0.0031308 else (1.055 * c ** (1 / 2.4) - 0.055), 0.0),
                                         1.0))) for c in rgb)


def diverging(n: int = 129, cold: str = "#3987e5", mid: str = "#383835", warm: str = "#e66767") -> list:
    """n colours (r, g, b) from cold through a neutral grey to warm, evenly in OKLab: below and above a baseline."""
    a, m, b = _oklab(cold), _oklab(mid), _oklab(warm)
    half = (n - 1) / 2
    out = []
    for k in range(n):
        x = (k - half) / half
        p, q = (m, b) if x >= 0 else (m, a)
        out.append(_srgb(tuple(p[c] + (q[c] - p[c]) * abs(x) for c in range(3))))
    return out


def sequential(n: int = 129, low: str = "#0d366b", high: str = "#cde2fb") -> list:
    """n colours (r, g, b) of one hue, evenly in OKLab: the least dark (near the background), the most light."""
    a, b = _oklab(low), _oklab(high)
    return [_srgb(tuple(a[c] + (b[c] - a[c]) * k / (n - 1) for c in range(3))) for k in range(n)]


class MapImage(QtWidgets.QWidget):
    """One way of a laser map (lasermap.MapView: sweeping up, or down), as a picture: the fast axis across, the slow
    one up; each point coloured by one of its layers (MapLegend): its frequency from where the laser began (blue
    below, red above, grey at it), or the power at the wavemeter, or its photodiode (dark blue the least, light the
    most); black where it did not lase, light grey where nothing was read; nothing where it is not measured yet.
    Hovered, a point says what it is."""

    LUT = diverging()
    SEQ = sequential()
    UNMEASURED, DARK, BAD = (0x1b, 0x1c, 0x20), (0x00, 0x00, 0x00), (0xc3, 0xc2, 0xb7)

    def __init__(self, title: str, up: bool):
        super().__init__()
        self.title, self.up = title, up
        self.view = None
        self.layer = "f"     # what the colours show (MapView.LAYERS)...
        self.scale = 1.0     # ...over: GHz either side of where it began, or (lowest, highest)
        self._rows = {}      # row -> (its version, its bytes)
        self._image = None
        self._drawn = None
        self.setMouseTracking(True)
        self.setMinimumSize(260, 200)

    def show_view(self, view, scale, layer: str = "f"):
        if view is not self.view:
            self.view, self._rows, self._image = view, {}, None
        if (layer, scale) != self._drawn:
            self._rows, self._drawn = {}, (layer, scale)
        self.layer, self.scale = layer, scale
        if view is None:
            self.update()
            return
        grid, versions = view.grid(layer, self.up), view.up_v if self.up else view.down_v
        w, changed = len(view.fv), False
        lut = [bytes((c[0], c[1], c[2], 255)) for c in (self.LUT if layer == "f" else self.SEQ)]
        n = len(lut)
        none, dark, bad = (bytes((c[0], c[1], c[2], 255)) for c in (self.UNMEASURED, self.DARK, self.BAD))
        lo, hi = (-scale, scale) if layer == "f" else scale
        width = (hi - lo) or 1.0
        for j, row in enumerate(grid):
            if self._rows.get(j, (None,))[0] == versions[j]:
                continue
            out = []
            for x in row:
                if x is None:
                    out.append(none)
                elif x != x:
                    out.append(bad)
                elif x == -math.inf:
                    out.append(dark)
                else:
                    out.append(lut[min(max(int((x - lo) / width * (n - 1) + 0.5), 0), n - 1)])
            self._rows[j] = (versions[j], b"".join(out))
            changed = True
        if changed or self._image is None:
            data = b"".join(self._rows[j][1] for j in range(len(grid) - 1, -1, -1))  # (the slow axis up)
            self._image = QtGui.QImage(data, w, len(grid), 4 * w, QtGui.QImage.Format.Format_RGBX8888).copy()
            self.update()

    def _area(self) -> QtCore.QRectF:
        fm = self.fontMetrics()
        return QtCore.QRectF(fm.horizontalAdvance("000.0") + 10, fm.height() + 6,
                             self.width() - fm.horizontalAdvance("000.0") - 16, self.height() - 2 * fm.height() - 14)

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor(C["bg"]))
        r, v = self._area(), self.view
        fm = p.fontMetrics()
        p.setPen(QtGui.QColor(C["fg"]))
        p.drawText(QtCore.QRectF(0, 0, self.width(), fm.height() + 2), Qt.AlignmentFlag.AlignHCenter, self.title)
        if v is None or self._image is None:
            p.setPen(QtGui.QColor(C["dim"]))
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, "nothing yet")
            return
        p.drawImage(r, self._image)
        p.setPen(QtGui.QPen(QtGui.QColor(C["line"]), 1))
        p.drawRect(r)
        p.setPen(QtGui.QColor(C["dim"]))
        fast, slow = ("mA", "V") if v.fast == "current" else ("V", "mA")
        bottom = QtCore.QRectF(r.left(), r.bottom() + 2, r.width(), fm.height())
        p.drawText(bottom, Qt.AlignmentFlag.AlignLeft, f"{v.fv[0]:g}")
        p.drawText(bottom, Qt.AlignmentFlag.AlignRight, f"{v.fv[-1]:g}")
        p.drawText(bottom, Qt.AlignmentFlag.AlignHCenter, f"{v.fast} ({fast}) →")
        left = r.left() - 4
        p.drawText(QtCore.QRectF(0, r.top() - 2, left, fm.height()), Qt.AlignmentFlag.AlignRight, f"{v.sv[-1]:g}")
        p.drawText(QtCore.QRectF(0, r.bottom() - fm.height() + 2, left, fm.height()), Qt.AlignmentFlag.AlignRight,
                   f"{v.sv[0]:g}")
        p.drawText(QtCore.QRectF(0, r.center().y() - fm.height() / 2, left, fm.height()), Qt.AlignmentFlag.AlignRight,
                   slow)

    def cell(self, pos):
        """(line, point) under pos, or None."""
        v, r = self.view, self._area()
        if v is None or not r.contains(QtCore.QPointF(pos)):
            return None
        k = min(int((pos.x() - r.left()) / r.width() * len(v.fv)), len(v.fv) - 1)
        j = len(v.sv) - 1 - min(int((pos.y() - r.top()) / r.height() * len(v.sv)), len(v.sv) - 1)
        return j, k

    def mouseMoveEvent(self, ev):
        c = self.cell(event_pos(ev))
        if c is None:
            QtWidgets.QToolTip.hideText()
            return
        v, (j, k) = self.view, c
        x = v.grid("f", self.up)[j][k]
        i, piezo = (v.fv[k], v.sv[j]) if v.fast == "current" else (v.sv[j], v.fv[k])
        what = ("not measured yet" if x is None else "no valid reading" if x != x else "not lasing"
                if x == -math.inf else f"{x:+.3f} GHz from where it began")
        power, pd = v.grid("power", self.up)[j][k], v.grid("pd", self.up)[j][k]
        if isinstance(power, (int, float)) and math.isfinite(power):
            what += f"; {power:.3g} µW at the wavemeter"
        if isinstance(pd, (int, float)) and math.isfinite(pd):
            what += f"; its photodiode {pd:.4g}"
        QtWidgets.QToolTip.showText(self.mapToGlobal(event_pos(ev)),
                                    f"{i:.3f} mA, {piezo:.3f} V, sweeping {'up' if self.up else 'down'}: {what}", self)


class MapLegend(QtWidgets.QWidget):
    """The colours of MapImage: what its layer shows (GHz from where the laser began, µW at the wavemeter, its
    photodiode), and the points that are not that."""

    def __init__(self):
        super().__init__()
        self.layer, self.scale, self.unit = "f", 1.0, ""
        self.setMinimumHeight(2 * self.fontMetrics().height() + 12)

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor(C["bg"]))
        fm = p.fontMetrics()
        bar = QtCore.QRectF(8, 4, min(320.0, self.width() * 0.45), fm.height() * 0.8)
        lut = MapImage.LUT if self.layer == "f" else MapImage.SEQ
        for k, c in enumerate(lut):
            p.fillRect(QtCore.QRectF(bar.left() + bar.width() * k / len(lut), bar.top(), bar.width() / len(lut) + 1,
                                     bar.height()), QtGui.QColor(*c))
        p.setPen(QtGui.QColor(C["dim"]))
        under = QtCore.QRectF(bar.left(), bar.bottom() + 2, bar.width(), fm.height())
        if self.layer == "f":
            p.drawText(under, Qt.AlignmentFlag.AlignLeft, f"−{self.scale:.3g} GHz")
            p.drawText(under, Qt.AlignmentFlag.AlignHCenter, "where it began")
            p.drawText(under, Qt.AlignmentFlag.AlignRight, f"+{self.scale:.3g} GHz")
        else:
            lo, hi = self.scale
            p.drawText(under, Qt.AlignmentFlag.AlignLeft, f"{lo:.3g} {self.unit}".rstrip())
            p.drawText(under, Qt.AlignmentFlag.AlignHCenter, "at the wavemeter" if self.layer == "power"
                       else "its photodiode")
            p.drawText(under, Qt.AlignmentFlag.AlignRight, f"{hi:.3g} {self.unit}".rstrip())
        x = bar.right() + 24
        for color, text in ((MapImage.DARK, "not lasing"), (MapImage.BAD, "no valid reading"),
                            (MapImage.UNMEASURED, "not measured yet")):
            p.fillRect(QtCore.QRectF(x, bar.top(), bar.height(), bar.height()), QtGui.QColor(*color))
            p.setPen(QtGui.QColor(C["line"]))
            p.drawRect(QtCore.QRectF(x, bar.top(), bar.height(), bar.height()))
            p.setPen(QtGui.QColor(C["fg"]))
            p.drawText(QtCore.QPointF(x + bar.height() + 6, bar.bottom()), text)
            x += bar.height() + 14 + fm.horizontalAdvance(text)


class MapDialog(QtWidgets.QDialog):
    """A laser map's plan (lasermap.MapPlan), as you give it: what to sweep and how, with what the map would take and
    what is wrong with it; Start begins it (Engine.start_map)."""

    SWEEP = (("current", "the current, up and down at each piezo voltage"),
             ("piezo", "the piezo, up and down at each current"),
             ("both", "both: a map each, the current's first"))
    PATTERNS = (("hops", "around hops, the dark and fading fringes, and at every Nth point"),
                ("all", "at every point (a large file)"), ("off", "never"))
    FF_END = (("back", "as it was"), ("fitted", "the ideal one the map finds, switched on"))

    def __init__(self, parent, engine: Engine, n: int):
        super().__init__(parent)
        self.engine, self.n = engine, n
        ch = engine.channels[n]
        self.setWindowTitle(f"Laser map · channel {n}" + (f" ({ch.name})" if ch.name else ""))
        p = ch.map_plan or MapPlan()
        self._filled = ch.map_plan is not None
        fit_screen(self, 780, 720)
        outer = QtWidgets.QVBoxLayout(self)
        inner = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(inner)
        outer.addWidget(scrolling(inner), 1)  # (it may not all fit on the screen)

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
            sb.valueChanged.connect(self.check)
            return sb

        def heading(text):
            label = Text(text, "accent")
            label.setContentsMargins(0, 8, 0, 0)
            form.addRow(label)

        self.now = Text("reading the laser…", "dim")
        self.now.setWordWrap(True)
        form.addRow(self.now)
        heading("Current (measured, as its DLC pro shows it)")
        self.i_lo = number(p.i_lo, 0.0, 10000.0, 2, 1.0)
        self.i_hi = number(p.i_hi, 0.0, 10000.0, 2, 1.0)
        self.i_step = number(p.i_step, 0.001, STEP_I, 3, 0.05)
        form.addRow("From", row(self.i_lo, "mA to", self.i_hi, "mA (the most it ever gets) in steps of", self.i_step,
                                "mA"))
        self.dark = QtWidgets.QSpinBox()
        self.dark.setRange(0, 50)
        self.dark.setValue(p.dark)
        self.dark.valueChanged.connect(self.check)
        form.addRow("Below threshold", row("a sweep down ends after", self.dark, "points in a row without lasing "
                                           "(0: never, down to the lowest)"))
        heading("Piezo")
        self.v_lo = number(p.v_lo, -1000.0, 1000.0, 2, 1.0)
        self.v_hi = number(p.v_hi, -1000.0, 1000.0, 2, 1.0)
        self.v_step = number(p.v_step, 0.001, STEP_V, 3, 0.1)
        form.addRow("From", row(self.v_lo, "V to", self.v_hi, "V in steps of", self.v_step, "V"))
        heading("Sweeping")
        self.sweep = QtWidgets.QComboBox()
        self.sweep.addItems([label for _, label in self.SWEEP])
        self.sweep.setCurrentIndex(next((k for k, (key, _) in enumerate(self.SWEEP) if key == p.sweep), 0))
        self.sweep.currentIndexChanged.connect(self.check)
        form.addRow("Sweep", self.sweep)
        self.settle = number(p.settle, 0.0, 60.0, 3, 0.01)
        self.reads = QtWidgets.QSpinBox()
        self.reads.setRange(1, 50)
        self.reads.setValue(p.reads)
        self.reads.valueChanged.connect(self.check)
        form.addRow("Each point", row(self.settle, "s after the step, then", self.reads,
                                      "readings that began after it"))
        self.repeat = QtWidgets.QCheckBox("The first line swept again at the end (how far it drifted meanwhile)")
        self.repeat.setChecked(p.repeat)
        self.repeat.toggled.connect(self.check)
        form.addRow(self.repeat)
        self.model = QtWidgets.QCheckBox("The laser's model fitted to it at the end, its facts said (a minute or two; "
                                         "the laser is free meanwhile)")
        self.model.setChecked(p.model)
        form.addRow(self.model)
        heading("Feedforward")
        ff_note = Text("Off for the map, so that it is square in the current as measured against the piezo: its "
                       "factor taken to 0 in steps that move the current 1 mA at most, the current read after each "
                       "(what it adds learned as it goes), switched off there, and on again the same way after. "
                       "Each map fits the ideal one from where the laser hops.", "dim")
        ff_note.setWordWrap(True)
        form.addRow(ff_note)
        self.ff_end = QtWidgets.QComboBox()
        self.ff_end.addItems([label for _, label in self.FF_END])
        self.ff_end.setCurrentIndex(next((k for k, (key, _) in enumerate(self.FF_END) if key == p.ff_end), 0))
        self.ff_end.currentIndexChanged.connect(self.check)
        form.addRow("At the end, its feedforward", self.ff_end)
        heading("Temperature")
        self.temp_on = QtWidgets.QCheckBox("Step the temperature too: a whole map at each")
        self.temp_on.setChecked(p.t_step > 0)
        self.temp_on.toggled.connect(self.check)
        form.addRow(self.temp_on)
        self.t_lo = number(p.t_lo, -50.0, 100.0, 3, 0.1)
        self.t_hi = number(p.t_hi, -50.0, 100.0, 3, 0.1)
        self.t_step = number(p.t_step or 0.5, 0.001, 20.0, 3, 0.1)
        form.addRow("From", row(self.t_lo, "°C to", self.t_hi, "°C in steps of", self.t_step, "K"))
        self.t_tol = number(p.t_tol, 0.001, 1.0, 3, 0.005)
        self.t_hold = number(p.t_hold, 0.0, 3600.0, 0, 5.0)
        self.t_wait = number(p.t_wait, 1.0, 36000.0, 0, 30.0)
        form.addRow("Settled", row("within", self.t_tol, "K for", self.t_hold, "s, waiting", self.t_wait,
                                   "s at most"))
        heading("Recorded")
        self.patterns = QtWidgets.QComboBox()
        self.patterns.addItems([label for _, label in self.PATTERNS])
        self.patterns.setCurrentIndex(next((k for k, (key, _) in enumerate(self.PATTERNS) if key == p.patterns), 0))
        self.patterns.currentIndexChanged.connect(self.check)
        self.every = QtWidgets.QSpinBox()
        self.every.setRange(0, 100000)
        self.every.setValue(p.every)
        self.every.valueChanged.connect(self.check)
        form.addRow("Wavemeter patterns", row(self.patterns, "N", self.every))
        self.patterns.setToolTip("The wavemeter's interference patterns, as its window shows them: where a second\n"
                                 "mode begins its fringes beat or fade. They take most of the file.")
        self.record = QtWidgets.QLineEdit(p.record)
        self.record.setPlaceholderText("e.g. dl:pd:power, dl:tc:current-act")
        self.record.textChanged.connect(self.check)
        form.addRow("Also read at each point", self.record)
        self.record.setToolTip("DLC pro parameters below laserN:, comma-separated. Its own photodiode\n"
                               "(dl:pd:photodiode) is read at each point anyway, if it has one.")
        folder = QtWidgets.QHBoxLayout()
        self.folder = QtWidgets.QLineEdit(p.folder or str(FOLDER))
        browse = QtWidgets.QPushButton("Browse…")
        browse.clicked.connect(self._browse)
        folder.addWidget(self.folder, 1)
        folder.addWidget(browse)
        form.addRow("Saved in", folder)
        note = Text("Unlock it first; a TA off, its DLC pro's cavity lock off. Its scan is switched off for the map "
                    "and on again after. Stopped, or done, it goes back where it began (another mode, perhaps: "
                    "hysteresis). Untick the other channels in the WLM software for a faster map; with the exposure "
                    "on auto it slows where the laser is dim.", "dim")
        note.setWordWrap(True)
        form.addRow(note)
        self.estimate = Text("", "fg")
        self.estimate.setWordWrap(True)
        outer.addWidget(self.estimate)
        self.problems = Text("", "err")
        self.problems.setWordWrap(True)
        outer.addWidget(self.problems)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        self.start = bb.addButton("Start map", QtWidgets.QDialogButtonBox.ButtonRole.AcceptRole)
        bb.accepted.connect(self._start)
        bb.rejected.connect(self.reject)
        outer.addWidget(bb)
        self._info = object()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._read)
        self.timer.start(150)
        engine.map_look(n)
        self.check()

    def plan(self) -> MapPlan:
        return MapPlan(i_lo=self.i_lo.value(), i_hi=self.i_hi.value(), i_step=self.i_step.value(),
                       v_lo=self.v_lo.value(), v_hi=self.v_hi.value(), v_step=self.v_step.value(),
                       sweep=self.SWEEP[self.sweep.currentIndex()][0], t_lo=self.t_lo.value(),
                       t_hi=self.t_hi.value(), t_step=self.t_step.value() if self.temp_on.isChecked() else 0.0,
                       t_tol=self.t_tol.value(), t_hold=self.t_hold.value(), t_wait=self.t_wait.value(),
                       settle=self.settle.value(), reads=self.reads.value(), dark=self.dark.value(),
                       patterns=self.PATTERNS[self.patterns.currentIndex()][0], every=self.every.value(),
                       record=self.record.text().strip(), repeat=self.repeat.isChecked(), model=self.model.isChecked(),
                       ff_end=self.FF_END[self.ff_end.currentIndex()][0], folder=self.folder.text().strip())

    def limits(self):
        """What take_over would read, from what map_look read (None: not yet)."""
        info = self.engine.channels[self.n].map_info
        if not info or "error" in info:
            return None
        act, i_set, v = info.get("current_act"), info.get("current_set"), info.get("piezo")
        if not all(isinstance(x, (int, float)) for x in (act, i_set, v)):
            return None

        def pair(a, b):
            lo, hi = info.get(a), info.get(b)
            return (lo, hi) if isinstance(lo, (int, float)) and isinstance(hi, (int, float)) and lo < hi else None

        t0 = info.get(TEMP_SET)  # (its feedforward off for the map: read again once off, the map checks again)
        return {"clip": info.get(CLIP) if isinstance(info.get(CLIP), (int, float)) else math.inf,
                "offset": act - i_set, "ff": 0.0, "v0": v,
                "pc": pair(PC_MIN, PC_MAX) or (-math.inf, math.inf), "tc": pair(TEMP_MIN, TEMP_MAX) or
                (-math.inf, math.inf), "t0": t0 if isinstance(t0, (int, float)) else None}

    def _read(self):
        info = self.engine.channels[self.n].map_info
        if info is self._info:
            return
        self._info = info
        if info is None:
            return
        if "error" in info or not info:
            self.now.paint(f"cannot read the laser: {info.get('error', 'not a DLC pro')}", "err")
            self.check()
            return
        act, v, t = info.get("current_act"), info.get("piezo"), info.get(TEMP_SET)
        parts = []
        if isinstance(act, (int, float)):
            parts.append(f"now {act:.3f} mA")
        if isinstance(info.get(CLIP), (int, float)):
            parts.append(f"current-clip {info[CLIP]:.1f} mA set")
        if isinstance(v, (int, float)):
            pc = (f", its limits {info[PC_MIN]:g} … {info[PC_MAX]:g} V" if isinstance(info.get(PC_MIN), (int, float))
                  and isinstance(info.get(PC_MAX), (int, float)) else "")
            parts.append(f"piezo {v:.3f} V{pc}")
        if isinstance(t, (int, float)):
            parts.append(f"{t:.3f} °C set")
        if info.get(FF_ON) is True and isinstance(info.get(FF), (int, float)):
            parts.append(f"feedforward {info[FF]:+.4f} mA/V (off for the map)")
        elif isinstance(info.get(FF), (int, float)):
            parts.append(f"feedforward off ({info[FF]:+.4f} mA/V when on)")
        pd = next((name for name in PD_NAMES if isinstance(info.get(name), (int, float))), "")
        parts.append(f"its photodiode ({pd}) read at each point" if pd else "no photodiode of its own read")
        warn = []
        if info.get(TA_ON) is True:
            warn.append("its TA is on: switch it off first")
        if info.get(LOCK_ON) is True:
            warn.append("its DLC pro's cavity lock is on: switch it off first")
        self.now.paint(" · ".join(parts) + (" — " + "; ".join(warn) if warn else ""), "err" if warn else "dim")
        if not self._filled and isinstance(act, (int, float)) and isinstance(v, (int, float)):
            self._filled = True  # (a first map of this laser: around where it is)
            self.i_lo.setValue(max(round(act - 30.0), 0.0))
            pc = (info.get(PC_MIN), info.get(PC_MAX))
            if all(isinstance(x, (int, float)) for x in pc) and pc[0] < pc[1]:
                self.v_lo.setValue(max(pc[0], v - 30.0))
                self.v_hi.setValue(min(pc[1], v + 30.0))
            else:
                self.v_lo.setValue(v - 30.0)
                self.v_hi.setValue(v + 30.0)
            if isinstance(t, (int, float)):
                self.t_lo.setValue(round(t - 0.5, 3))
                self.t_hi.setValue(round(t + 0.5, 3))
        self.check()

    def check(self, *_):
        p = self.plan()
        bad = p.problems(self.limits())
        info = self.engine.channels[self.n].map_info or {}
        if info.get(TA_ON) is True:
            bad.append("its TA is on")
        if info.get(LOCK_ON) is True:
            bad.append("its cavity lock is on")
        temp = p.t_step > 0
        for w in (self.t_lo, self.t_hi, self.t_step, self.t_tol, self.t_hold, self.t_wait):
            w.setEnabled(temp)
        self.every.setEnabled(p.patterns == "hops")
        s, size = self.engine.map_estimate(self.n, p)
        rate = self.engine.channel_rate(self.n)
        self.estimate.paint(f"{p.points():,} points at most; about {fmt_duration(s)} at the {rate:.0f} readings a "
                            f"second the wavemeter gives it now (slower where it is dim); the file about "
                            f"{fmt_bytes(size)}" if rate else
                            f"{p.points():,} points at most; the file about {fmt_bytes(size)} (no readings now: "
                            f"how long it takes is not known)", "fg")
        self.problems.paint("; ".join(bad), "err")
        self.start.setEnabled(not bad)

    def _browse(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Save laser maps in", self.folder.text())
        if d:
            self.folder.setText(d)

    def _start(self):
        why = self.engine.start_map(self.n, self.plan())
        if why:
            self.problems.paint(why, "err")
            return
        self.accept()


class MapWindow(QtWidgets.QWidget):
    """A laser map as it goes, and once it is over: each way as a picture (MapImage) of what Show picks (its
    frequency, the power at the wavemeter, its photodiode), the last two sweeps of the same, the last interference
    pattern, what it noted (the ideal feedforward each map found among it); Stop puts the laser back where it began.
    Done, a picture of it is saved next to its file."""

    LAYERS = (("f", "frequency"), ("power", "power at the wavemeter"), ("pd", "its photodiode"))

    def __init__(self, win: "MainWindow", engine: Engine, n: int):
        super().__init__(win, Qt.WindowType.Window)
        self.win, self.engine, self.n = win, engine, n
        self.setWindowTitle(f"Laser map · channel {n}")
        fit_screen(self, 980, 760)
        v = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        self.which = QtWidgets.QComboBox()
        self.which.setMinimumWidth(260)
        self.which.activated.connect(self._picked)
        top.addWidget(self.which)
        top.addWidget(Text("Show", "dim"))
        self.show_what = QtWidgets.QComboBox()
        self.show_what.addItems([label for _, label in self.LAYERS])
        self.show_what.activated.connect(self._layer)
        top.addWidget(self.show_what)
        self.bar = QtWidgets.QProgressBar()
        self.bar.setRange(0, 1000)
        self.bar.setTextVisible(False)
        top.addWidget(self.bar, 1)
        v.addLayout(top)
        self.head = Text("", "fg")
        self.head.setWordWrap(True)
        v.addWidget(self.head)
        self.maps = QtWidgets.QWidget()  # (both ways and their legend: the picture saved)
        box = QtWidgets.QVBoxLayout(self.maps)
        box.setContentsMargins(0, 0, 0, 0)
        both = QtWidgets.QHBoxLayout()
        self.up_img, self.down_img = MapImage("sweeping up", True), MapImage("sweeping down", False)
        both.addWidget(self.up_img)
        both.addWidget(self.down_img)
        box.addLayout(both, 1)
        self.legend = MapLegend()
        box.addWidget(self.legend)
        v.addWidget(self.maps, 3)
        lower = QtWidgets.QHBoxLayout()
        self.sweeps = Plot("", "GHz", 150)
        self.pattern = Plot("pixel", "counts", 150)
        self.sweeps_box = RecoveryDialog._box("the last two sweeps: frequency from where it began", self.sweeps)
        self.pattern_box = RecoveryDialog._box("the last interference pattern the wavemeter gave", self.pattern)
        lower.addWidget(self.sweeps_box, 1)
        lower.addWidget(self.pattern_box, 1)
        v.addLayout(lower, 2)
        self.notes = QtWidgets.QPlainTextEdit()
        self.notes.setReadOnly(True)
        self.notes.setMaximumHeight(110)
        v.addWidget(self.notes)
        bb = QtWidgets.QHBoxLayout()
        self.stop = QtWidgets.QPushButton("Stop map")
        self.stop.clicked.connect(self._stop)
        self.folder = QtWidgets.QPushButton("Open folder")
        self.folder.clicked.connect(self._open)
        self.laser = QtWidgets.QPushButton("The laser…")
        self.laser.setToolTip("What the laser's model, fitted to the map at its end, says of it")
        self.laser.clicked.connect(self._laser)
        self.laser.setEnabled(False)
        close = QtWidgets.QPushButton("Close")
        close.clicked.connect(self.close)
        bb.addWidget(self.stop)
        bb.addStretch(1)
        bb.addWidget(self.laser)
        bb.addWidget(self.folder)
        bb.addWidget(close)
        v.addLayout(bb)
        self.follow = True  # show the map filling, until another is picked
        self.view = None
        self._views = 0
        self._shown = (None, -1)
        self._notes = 0
        self._saved = None
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)
        self.refresh()

    def runner(self):
        ch = self.engine.channels[self.n]
        return ch.mapper or ch.map_last

    @property
    def layer(self) -> str:
        return self.LAYERS[max(self.show_what.currentIndex(), 0)][0]

    def _layer(self, _k: int):
        self._shown = (None, -1)
        self.refresh()

    def _unit(self, layer: str, m) -> str:
        return {"f": "GHz", "power": "µW"}.get(layer) or PD_UNITS.get(m.pd, "")

    def _scale(self, view, layer: str):
        """What the colours cover: GHz either side of where it began (most of it), or (lowest, highest) (most of
        those): a round number, so that the picture is drawn again only when it changes much."""
        vals = [x for up in (True, False) for row in view.grid(layer, up) for x in row
                if x is not None and x == x and x != -math.inf]
        if layer == "f":
            vals = sorted(abs(x) for x in vals)
            span = max(vals[int(0.98 * (len(vals) - 1))] if vals else 1.0, 0.05)
            return float(f"{span:.2g}")
        vals.sort()
        if not vals:
            return (0.0, 1.0)
        lo = 0.0 if layer == "power" else vals[int(0.02 * (len(vals) - 1))]
        hi = vals[int(0.98 * (len(vals) - 1))]
        lo, hi = float(f"{lo:.2g}"), float(f"{hi:.2g}")
        return (lo, hi if hi > lo else lo + 1.0)

    def _picked(self, k: int):
        r = self.runner()
        if r is not None and 0 <= k < len(r.mapper.views):
            self.follow = k == len(r.mapper.views) - 1
            self.view = r.mapper.views[k]
            self._shown = (None, -1)
            self.refresh()

    def refresh(self):
        ch = self.engine.channels[self.n]
        r = self.runner()
        if r is None:
            self.head.paint("laser map: reading its settings…" if ch.state == "mapping" else ch.map_note or "", "dim")
            return
        m = r.mapper
        if len(m.views) != self._views:
            self._views = len(m.views)
            self.which.clear()
            for k, view in enumerate(m.views):
                what = f"map {k + 1}: the {view.kind} swept" if not view.repeat else f"its first line again ({k + 1})"
                self.which.addItem(what + (f", {view.temp:.3f} °C" if view.temp is not None else ""))
        if m.views and (self.follow or self.view is None):
            self.view = m.view or m.views[-1]
            self.which.setCurrentIndex(m.views.index(self.view))
        running = r.busy
        self.bar.setValue(int(1000 * m.progress))
        now = time.monotonic()
        took = (now if running else r.now) - r.started
        if running:
            left = f", {fmt_duration(took * (1 - m.progress) / m.progress)} to go" if m.progress > 0.02 else ""
            self.head.paint(f"{r.stage} · {m.points:,} points ({m.lasing:,} lasing), {fmt_duration(took)} so far{left}"
                            + (f" · {fmt_bytes(m.out.size)}" if m.out is not None else ""), "accent")
        else:
            self.head.paint(ch.map_note or r.error or r.result, "ok" if r.state == "done" else "err")
        self.stop.setEnabled(running)
        self.laser.setEnabled(m.model is not None)
        self.laser.setText("The laser…" if m.model is not None or not m.model_note else
                           "Fitting its model…" if m.model_note.startswith("fitting") else "No model")
        self.laser.setToolTip(m.model_note or "What the laser's model, fitted to the map at its end, says of it")
        view = self.view
        if view is not None and (view, view.version) != self._shown:
            self._shown = (view, view.version)
            layer = self.layer
            if layer == "pd" and not m.pd:
                layer = "f"
            scale = self._scale(view, layer)
            self.up_img.show_view(view, scale, layer)
            self.down_img.show_view(view, scale, layer)
            self.legend.layer, self.legend.scale, self.legend.unit = layer, scale, self._unit(layer, m)
            self.legend.update()
            self.sweeps.series.clear()
            col = {"f": 1, "power": 2, "pd": 3}[layer]
            for trace, color in ((m.trace_before, "dim"), (m.trace, "accent")):
                self.sweeps.add([t[0] for t in trace], [t[col] for t in trace], color, "dots", 1.5)
            self.sweeps.xlabel = f"{view.fast} ({'mA' if view.fast == 'current' else 'V'})"
            self.sweeps.ylabel = self._unit(layer, m)
            legs = {1: "up", 2: "down", 0: "down to the dark"}
            what = {"f": "frequency from where it began", "power": "power at the wavemeter",
                    "pd": "its photodiode"}[layer]
            self.sweeps_box.findChild(QtWidgets.QLabel).setText(
                f"the last two sweeps: {what}, now {legs.get(m.trace_legs[0], '')} (blue), "
                f"before that {legs.get(m.trace_legs[1], '…')} (grey)")
            self.sweeps.update()
            self.pattern.series.clear()
            if m.pattern_now is not None:
                values = m.pattern_now[1]
                self.pattern.add(list(range(len(values))), list(values), "ok", "line", 1.0)
            self.pattern.update()
        if len(m.notes) != self._notes:
            self._notes = len(m.notes)
            self.notes.setPlainText("\n".join(f"{t:8.1f} s  {text}" if t is not None else f"{'':10s}{text}"
                                              for t, text in m.notes[-60:]))
            self.notes.verticalScrollBar().setValue(self.notes.verticalScrollBar().maximum())
        if not running and m.out is not None and self._saved is None:
            self._saved = m.out.path.with_suffix(".png")
            try:
                self.maps.grab().save(str(self._saved))  # (a look at it, next to its file)
            except Exception:  # (whatever it is: never in the way)
                pass

    def _stop(self):
        if self.win.confirm("Stop the map", "Stop the map? The laser goes back where it began; what it mapped so far "
                                            "stays in its file."):
            self.engine.stop_map(self.n)

    def _laser(self):
        r = self.runner()
        if r is not None and r.mapper.model is not None:
            ModelDialog(self, r.mapper.model, r.mapper.out.path if r.mapper.out is not None else None).show()

    def _open(self):
        r = self.runner()
        path = r.mapper.out.path.parent if r is not None and r.mapper.out is not None else \
            (self.engine.channels[self.n].map_plan.folder or FOLDER)
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))


class ModelDialog(QtWidgets.QDialog):
    """What a laser's model (lasermodel.LaserModel, fitted to its map) says of it, fact by fact."""

    def __init__(self, parent, model, path=None):
        super().__init__(parent)
        self.setWindowTitle("The laser, as its model has it" + (f" · {Path(path).name}" if path else ""))
        fit_screen(self, 760, 620)
        v = QtWidgets.QVBoxLayout(self)
        browser = QtWidgets.QTextBrowser()
        items = "".join(f"<li style='margin-bottom:6px'>{html.escape(fact)}</li>" for fact in model.facts())
        head = f"{model.f_ref:.6f} THz" + (f" · kept in {html.escape(Path(path).name)}" if path else "")
        browser.setHtml(f"<p style='color:{C['dim']}'>{head}</p><ul>{items}</ul>")
        v.addWidget(browser, 1)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Close)
        bb.rejected.connect(self.reject)
        v.addWidget(bb)


class SetupDialog(QtWidgets.QDialog):
    """The setup check: what a recovery would use on this laser, read (only) from its controller."""

    COLORS = {"ok": C["ok"], "info": C["dim"], "warn": C["warn"], "error": C["err"]}

    def __init__(self, parent, engine: Engine, n: int):
        super().__init__(parent)
        self.engine, self.n = engine, n
        self.setWindowTitle(f"Check setup · channel {n}")
        fit_screen(self, 720, 560)
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

    def __init__(self, parent, share, title: str = "Network"):
        super().__init__(parent)
        self.share = share
        self.setWindowTitle(title)
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
    def __init__(self, engine: Engine, save, share=None, meters=None):
        """share: a share.Share, to show the other PCs' wavemeters too (None: this PC's alone); meters: this PC's
        wavemeters (meters.Meters, the first of them engine's and share's), each framed under its name (None:
        engine's alone)."""
        super().__init__()
        self.engine, self.save, self.share, self.meters = engine, save, share, meters
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
        self.local = Panel(self, meter=meters.items[0] if meters is not None else Meter(engine, share))
        self.locals = [self.local]  # this PC's wavemeters (the first: the settings', engine's)
        self.placeholder = self.local.placeholder
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

        self.panels = {}       # another PC's id -> the Panel of its wavemeter
        self.panel_list = []
        self.colors = {}       # a wavemeter's name -> the colour of its frame
        self._peer_ids = None
        self._meters_version = None
        self._fit_at = 0.0
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

    @property
    def rows(self) -> dict:
        """The first wavemeter's rows (this PC's, the settings'), by channel."""
        return self.local.rows

    @property
    def row_list(self) -> list:
        return self.local.row_list

    def all_rows(self) -> list:
        """This PC's rows and the other PCs'."""
        return [row for panel in self.locals + self.panel_list for row in panel.row_list]

    @property
    def several(self) -> bool:
        """This PC reads more than one wavemeter."""
        return len(self.locals) > 1

    # ------------------------------------------------------------- refresh
    def refresh(self):
        now = time.monotonic()
        self._sync_locals()
        for panel in self.locals:
            panel.refresh(now)
        if self.share is not None:
            self._sync_panels()
        if not self.isMinimized():
            for panel in self.panel_list:
                panel.refresh(now)
        if self._fit_at and now >= self._fit_at:
            self._fit()
        if now >= self._slow_at:
            self._slow_at = now + 0.5
            self._slow(now)

    def _slow(self, now: float):
        if self.meters is not None and self.meters.poll():  # (another wavemeter of this PC's found)
            self._sync_locals()
        for panel in self.locals:
            e = panel.engine
            e.update_rate()
            e.tick()
            if e.rows() != [row.n for row in panel.row_list]:  # (a channel switched off, watched no more)
                panel.rebuild_rows()
        refs = [(panel, panel.engine.reference) for panel in self.locals if panel.engine.reference is not None]
        if not refs:
            self.st_ref.paint("", "dim")
        else:
            texts, colors = [], []
            for panel, ref in refs:
                text = {TRACKING: f"ref ch{ref.channel}  wavemeter {ref.error:+.1f} MHz",
                        HOLD: f"ref ch{ref.channel}  hold {fmt_duration(now - ref.since)}",
                        MOVED: f"ref ch{ref.channel}  moved: right-click it",
                        ACQUIRING: f"ref ch{ref.channel}  acquiring"}[ref.state]
                texts.append(f"{panel.label}: {text}" if self.several else text)
                colors.append({TRACKING: "accent", MOVED: "err"}.get(ref.state, "warn"))
            worst = next((c for c in ("err", "warn") if c in colors), colors[0])
            self.st_ref.paint("    ".join(texts), worst)
        e = self.engine
        if self.several:
            ok = sum(1 for panel in self.locals if panel.engine.source_ok)
            rows = sum(len(panel.row_list) for panel in self.locals)
            rate = sum(panel.engine.rate for panel in self.locals)
            self.st_wlm.paint(f"● {len(self.locals)} wavemeters   {rows} ch   {rate:.0f} readings/s",
                              "dim" if ok == len(self.locals) else "err")
        elif getattr(e.source, "viewer", False):
            self.st_wlm.paint("no wavemeter on this PC", "dim")
        elif e.source_ok:
            label = getattr(e.source, "name", "") or getattr(e.source, "label", "")
            self.st_wlm.paint(f"● {label}   {len(self.row_list)} ch   {e.rate:.0f} readings/s", "dim")
        else:
            self.st_wlm.paint(f"● {e.source_msg}", "err")
        for panel in self.locals:
            if panel.framed:
                panel.paint_header(self.several)
        if e.scanning:
            self.st_lan.paint("scanning LAN…", "dim")
        else:
            n = sum(1 for d in e.device_list() if not d.error)
            self.st_lan.paint(f"{n} controller{'s' if n != 1 else ''} on LAN", "dim")
        if self.share is not None:
            self._slow_share(now)
        for row in self.all_rows():
            row.update_tooltip()
        dirty = any(panel.engine.dirty for panel in self.locals) or (self.meters is not None and self.meters.dirty)
        if (dirty and now - self._saved_at > 1.0) or (refs and now - self._saved_at > 300):
            self._save()  # the reference's estimate too, now and then

    # ------------------------------------------------------- this PC's wavemeters
    def _sync_locals(self):
        """A panel for each of this PC's wavemeters (meters.py), in order, at the top."""
        m = self.meters
        if m is None or m.version == self._meters_version:
            return
        self._meters_version = m.version
        have = {id(panel.meter): panel for panel in self.locals}
        locals_ = [have.get(id(meter)) or Panel(self, meter=meter) for meter in m.items]
        for panel in self.locals:
            if panel not in locals_:
                panel.hide()
                panel.deleteLater()
        folded = self.gui.get("folded") or []
        for k, panel in enumerate(locals_):
            if panel not in self.locals and panel.label in folded:
                panel.fold(True)
            self.panels_box.insertWidget(k, panel)
        self.locals = locals_
        self._peer_ids = None  # (the other PCs' panels after these)
        self._frame_locals()
        self.fit_later()

    def _frame_locals(self):
        framed = self.several or bool(self.panel_list)
        for panel in self.locals:
            panel.set_framed(framed)
            if framed:
                panel.paint_header(self.several)

    def mark_dirty(self):
        self.engine.dirty = True

    def swap(self, a: Panel, b: Panel):
        """The settings under a's wavemeter (its channels, lasers, reference...) go with b's, and b's with a's."""
        if self.meters is None:
            return
        busy = [panel.label for panel in (a, b) if not panel.meter.idle()]
        if busy:
            QtWidgets.QMessageBox.information(
                self, "Swap settings", f"First unlock every laser of {' and '.join(busy)} (Lock, Relock, "
                                       f"Recentering: off).")
            return
        na, nb = a.meter.name or a.label, b.meter.name or b.label
        if self.confirm("Swap settings",
                        f"The channels, lasers, names and reference laser set up under {a.label} go with wavemeter "
                        f"{nb}, and those under {b.label} with {na}: for settings that were given the wrong "
                        f"wavemeter (each laser shown on another's channel). Swap them?"):
            self.meters.swap(a.meter, b.meter)

    def hide_meter(self, panel: Panel):
        """One of this PC's wavemeters (not the first) neither shown nor read any more; its settings kept."""
        if self.meters is None or panel is self.local:
            return
        if not panel.meter.idle():
            QtWidgets.QMessageBox.information(self, "Hide this wavemeter",
                                              f"First unlock every laser of {panel.label}.")
            return
        if self.confirm("Hide this wavemeter",
                        f"Hide {panel.label} ({panel.meter.name}), and stop reading it? Its settings are kept: "
                        f"right-click empty space › Show wavemeter brings it back."):
            self.meters.hide(panel.meter)

    def show_meter(self, serial: int):
        if self.meters is not None:
            self.meters.show(serial)

    def may_link(self, engine, key: str, laser: int, ff_ao=None) -> bool:
        """Link a channel of engine to that laser (or analog output, and the one it feeds forward): fine, unless a
        channel of another of this PC's wavemeters drives it too, and you would rather not."""
        if self.meters is None:
            return True
        mine = {laser} | ({ff_ao} if ff_ao is not None else set())
        for panel in self.locals:
            e = panel.engine
            if e is engine:
                continue
            with e.lock:
                clash = next((ch for ch in e.channels.values() if ch.device == key and mine & (
                    {ch.laser} | ({ch.ff_ao} if ch.ff_ao is not None else set()))), None)
            if clash is not None:
                what = e.where(clash)
                return self.confirm("Link laser", f"Channel {clash.n} of {panel.label} drives it already ({what}): "
                                                  f"two locks would fight over it. Link it here too?")
        return True

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
            self.panels_box.insertWidget(len(self.locals) + k, panel)
        self.panel_list = [self.panels[pid] for pid in ids]
        self._frame_locals()
        self.local.setVisible(not (peers and getattr(self.engine.source, "viewer", False)))
        self._slow_share(time.monotonic())
        self.fit_later()

    def _slow_share(self, now: float):
        share = self.share
        shares = [panel.meter.share for panel in self.locals if panel.meter.share is not None]
        if not share.on:
            self.st_net.paint("", "dim")
        elif share.error:
            self.st_net.paint(f"⇄ {share.error}", "err")
        else:
            live = [panel.peer for panel in self.panel_list if panel.peer.status(now) in ("live", "late")]
            pcs = len({p.host for p in live})
            rate = sum(sh.rate_out + sh.rate_in for sh in shares)
            self.st_net.paint(f"⇄ {pcs} other PC{'s' if pcs != 1 else ''}"
                              + (f" ({len(live)} wavemeters)" if len(live) != pcs else "")
                              + f" · {rate / 1e3:.1f} kB/s", "warn" if share.port_note else "dim")
            self.st_net.setToolTip(share.port_note or "The other PCs found on the LAN, and the traffic to and from "
                                                      "them: right-click › Network…")
        self.colors = panel_colors([panel.label for panel in self.locals] + [panel.label for panel in self.panel_list])
        for panel in self.locals:
            panel.set_label(panel.label)
        if self.several:
            title = f"Wavemeter Lock · {share.host}"
        else:
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

    def share_dialog(self, share=None):
        """The network settings, and the name of a wavemeter of this PC's (share's; the first's if None)."""
        share = share or self.share
        if share is None:
            return
        d = ShareDialog(self, share, f"Network · {share.label}" if self.several else "Network")
        if d.exec():
            share.settings.update(d.values())
            for panel in self.locals:  # (the network's settings are all of theirs)
                if panel.meter.share is not None:
                    panel.meter.share.configure()
            self.engine.dirty = True

    def _save(self):
        self.gui["geometry"] = bytes(self.saveGeometry().toHex()).decode()
        self._saved_at = time.monotonic()
        try:
            self.save()
        except OSError as exc:
            self.st_lan.paint(f"cannot save settings: {exc}", "err")

    # ------------------------------------------------------------- actions
    def rescan(self):
        for panel in self.locals:
            panel.engine.start_discovery()

    def show_menu(self, pos):
        m = QtWidgets.QMenu(self)
        m.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        m.addAction("Rescan LAN   F5", self.rescan)
        if self.share is not None:
            m.addAction("Network…", self.share_dialog)
        hidden = [(panel, ch) for panel in self.locals for n, ch in sorted(panel.engine.channels.items())
                  if ch.hidden and n in panel.engine.active]
        if hidden:
            sub = m.addMenu("Show hidden channel")
            for panel, ch in hidden:
                where = f"{panel.label}: " if self.several else ""
                sub.addAction(f"{where}{ch.n}  {ch.name}", functools.partial(panel.engine.set_hidden, ch.n, False))
        if self.meters is not None and self.meters.hidden():
            sub = m.addMenu("Show wavemeter")
            for serial, name in self.meters.hidden():
                sub.addAction(name, functools.partial(self.show_meter, serial))
        a = m.addAction("Always on top")
        a.setCheckable(True)
        a.setChecked(bool(self.gui.get("on_top")))
        a.toggled.connect(self.set_on_top)
        m.addAction("Larger digits   Ctrl +", lambda: self.set_font_size(self.gui["font_pt"] + 2))
        m.addAction("Smaller digits   Ctrl −", lambda: self.set_font_size(self.gui["font_pt"] - 2))
        sims = [panel for panel in self.locals if hasattr(panel.engine.source, "lasers")]
        if sims:  # --sim only: try the reference and mode-hop logic out
            sim = m.addMenu("Simulate a fault")
            ref = next(((p.engine, p.engine.source.cavity_laser()) for p in sims if p.engine.source.cavity_laser()),
                       None)
            if ref is not None:
                re_, laser = ref
                sim.addAction("Reference mode hop +1.5 GHz for 30 s", lambda: laser.mode_hop(1500, 30))
                sim.addAction("Reference mode hop +1.5 GHz, stays", lambda: laser.mode_hop(1500))
                sim.addAction("Reference cavity unlocked for 20 s", lambda: laser.unlock(20))
                sim.addAction("Reference light blocked for 15 s", lambda: laser.dark(15))
                sim.addAction("Wavemeter jumps +20 MHz", lambda: re_.source.wavemeter.jump(20e-6 / laser.f0))
            for panel in sims:  # (each of this PC's simulated wavemeters' lasers)
                sim.addSeparator()
                if self.several:
                    sim.addAction(panel.label).setEnabled(False)
                self._sim_laser_actions(sim, panel.engine)
        self._menu = m
        m.popup(self.mapToGlobal(pos))

    @staticmethod
    def _sim_laser_actions(sim, e):
        """--sim: the faults its simulated lasers can be given."""
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

    def confirm(self, title: str, text: str) -> bool:
        box = QtWidgets.QMessageBox.question(self, title, text)
        return box == QtWidgets.QMessageBox.StandardButton.Yes

    def map_window(self, engine: Engine, n: int) -> "MapWindow":
        """Channel n's laser map, as it goes or as it ended: its window, shown (one a channel, kept)."""
        windows = self.__dict__.setdefault("map_windows", {})
        w = windows.get((id(engine), n))
        if w is None:
            w = windows[(id(engine), n)] = MapWindow(self, engine, n)
        w.show()
        w.raise_()
        w.activateWindow()
        return w

    def use_as_reference(self, n: int, engine: Engine | None = None):
        e = engine or self.engine
        if e.channels[n].state != "off" and not self.confirm(
                "Reference laser", f"Channel {n} is locked to the wavemeter. Make it the reference and unlock it?"):
            return
        e.set_reference(n)

    def stop_reference(self, engine: Engine | None = None):
        e = engine or self.engine
        shift = e.locked_shift_mhz(lambda f: f)
        if shift > 1.0 and not self.confirm(
                "Stop referencing", f"Without the reference the wavemeter's own drift comes back:\n"
                                    f"locked lasers will move by up to {shift:.1f} MHz. Stop referencing?"):
            return
        e.clear_reference()

    def accept_drift(self, engine: Engine | None = None):
        e = engine or self.engine
        ref = e.reference
        if ref is None or ref.state != MOVED:
            return
        shift = e.locked_shift_mhz(lambda f: ref.preview(f, accept=True))
        if self.confirm("Wavemeter moved?",
                        f"Treat the reference's {ref.offset:+.1f} MHz as wavemeter drift?\n"
                        f"Only right if the reference laser is still on the same cavity mode.\n"
                        f"Locked lasers will move by up to {shift:.1f} MHz."):
            e.reference_accept_drift()

    def reference_dialog(self, engine: Engine | None = None):
        e = engine or self.engine
        ref = e.reference
        if ref is None:
            return
        d = ReferenceDialog(self, ref)
        if not d.exec():
            return
        params, thz = d.params(), d.new_anchor_thz()
        if thz is not None or params.scale != ref.p.scale:
            shift = e.locked_shift_mhz(lambda f: ref.preview(f, params.scale, thz))
            if shift > 1.0 and not self.confirm(
                    "Reference", f"Locked lasers keep their setpoints in the new frame:\n"
                                 f"they will move by up to {shift:.1f} MHz. Continue?"):
                return
        e.set_reference_params(params)
        if thz is not None:
            e.set_reference_anchor(thz)

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
            self.rescan()
        else:
            super().keyPressEvent(ev)

    def closeEvent(self, ev):
        self.timer.stop()
        self._save()
        if self.meters is not None:
            self.meters.stop()
        else:
            if self.share is not None:  # (first: no adjustments from the others while it closes)
                self.share.stop()
                self.share.join(1.0)
            self.engine.stop()
        super().closeEvent(ev)


def refuse(title: str, text: str) -> int:
    """Say why the program does not start (in a message box too: it may have no console)."""
    print(text, file=sys.stderr)
    try:
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
        app.setApplicationName("Wavemeter Lock")
        QtWidgets.QMessageBox.warning(None, title, text)
    except Exception:  # (no display, say: the console has it)
        pass
    return 1


def run(engine: Engine, save, share=None, meters=None) -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName("Wavemeter Lock")
    app.setStyle("Fusion")
    app.setPalette(dark_palette())
    app.setStyleSheet(QSS)
    win = MainWindow(engine, save, share, meters)
    for sig in (signal.SIGINT, signal.SIGTERM):  # Ctrl+C / kill: close cleanly, settings saved
        signal.signal(sig, lambda *_: win.close())
    if meters is not None:
        meters.start()
    else:
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
