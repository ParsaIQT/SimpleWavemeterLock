"""Minimal Qt front-end.

Everything slow lives in the engine's threads. This module repaints changed
numbers on a 30 Hz timer and forwards clicks, so the window never stalls.
"""
from __future__ import annotations

import functools
import math
import signal
import sys
import time

from .engine import Engine
from .lock import LockParams
from .modehop import HopParams
from .qt import Qt, QtCore, QtGui, QtWidgets
from .reference import ACQUIRING, HOLD, MOVED, TRACKING, RefParams, fmt_duration
from .wavemeter import C_NM_THZ

REFRESH_MS = 33
STALE_S = 3.0  # a channel without a new reading for this long is greyed out
NOTE_S = 1800.0  # how long a row mentions its last mode-hop recovery

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
"""


def fmt_nm(nm: float, digits: int) -> str:
    """Wavelength with `digits` significant digits."""
    return f"{nm:.{decimals(nm, digits)}f}"


def decimals(nm: float, digits: int) -> int:
    return max(digits - len(str(int(nm))), 0)


def fmt_mhz(x: float) -> str:
    return f"{x / 1e3:+.3f} GHz" if abs(x) >= 1e4 else f"{x:+.1f} MHz"


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
            pal.setColor(QtGui.QPalette.ColorRole.WindowText, QtGui.QColor(C[color]))
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

    def stepBy(self, steps: int):
        if self.value() == self.minimum():
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


class ChannelRow(QtWidgets.QFrame):
    def __init__(self, win: "MainWindow", n: int):
        super().__init__()
        self.win, self.engine, self.n = win, win.engine, n
        self.setObjectName("row")
        self.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_menu)
        self._state = None
        self._sp = None
        self._menu = None
        self._is_ref = False
        self._prompt = ""

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

    # -------------------------------------------------------------- refresh
    def refresh(self, now: float):
        ch = self.engine.channels[self.n]
        ref = self.engine.reference
        is_ref = ref is not None and ref.channel == self.n
        if is_ref != self._is_ref:
            self._is_ref = is_ref
            self.sp.setVisible(not is_ref)
            self.anchor_text.setVisible(is_ref)
            self.sp_label.paint("ref" if is_ref else "set", "dim")
        if is_ref:
            self._refresh_reference(ch, ref, now)
            if ch.state == "recovering":
                self._show_recovery(ch, now)
                self._set_state("asking" if ch.hop_prompt else "recovering")
            return
        self.unit.paint("nm", "accent" if ref is not None and ref.active else "dim")  # referenced
        valid = ch.freq > 0
        if valid:
            self.wl.paint(fmt_nm(C_NM_THZ / ch.freq, self.win.digits), "dim" if now - ch.t > STALE_S else "fg")
        else:
            self.wl.paint(ch.status or "—", "err")
        state = ch.state
        if valid and ch.setpoint_nm:
            color = {"on": "ok", "engaging": "warn", "recovering": "warn"}.get(state, "fg")
            self.dev.paint(fmt_mhz(ch.error_mhz), color)
        else:
            self.dev.paint("", "fg")
        sp = ch.setpoint_nm or 0.0
        if sp != self._sp and not (self.sp.hasFocus() and self.sp.lineEdit().isModified()):
            self._show_setpoint(sp)
        if state == "recovering":
            self._show_recovery(ch, now)
        elif ch.message:
            self.info.paint(ch.message, "err")
        elif ch.device:
            dev = self.engine.devices.get(ch.device)
            ctrl = self.engine.controllers.get(ch.device)
            where = f"laser{ch.laser} @ {dev.name if dev else ch.device}"
            if ctrl is not None and ctrl.online is False:
                self.info.paint(f"{where}   offline", "err")
            else:
                off = "…" if ch.offset is None else f"{ch.offset:.3f} {ch.unit}"
                self.info.paint(f"{where}   {off}{self._hop_text(ch, now)}", "dim")
        else:
            self.info.paint("right-click to link a laser", "dim")
        if state == "recovering" and ch.hop_prompt:
            state = "asking"
        self._set_state(state if state != "off" else ("error" if ch.message else "off"))
        self.name.paint(ch.name, "dim")

    def _refresh_reference(self, ch, ref, now: float):
        """The reference row: raw reading, where the reference should be, and how it is doing."""
        digits = self.win.digits
        if ch.raw > 0:
            self.wl.paint(fmt_nm(C_NM_THZ / ch.raw, digits), "dim" if now - ch.t > STALE_S else "fg")
        else:
            self.wl.paint(ch.status or "—", "err")
        state = ref.state
        color = {TRACKING: "dim", MOVED: "err"}.get(state, "warn")
        offset = state != ACQUIRING and math.isfinite(ref.offset)
        self.dev.paint(fmt_mhz(ref.offset) if offset else "", "fg" if state == TRACKING else color)
        self.anchor_text.paint(fmt_nm(C_NM_THZ / ref.anchor, digits) if ref.anchor else "—", "dim")
        if state == TRACKING:
            text = f"reference · wavemeter {ref.error:+.2f} MHz, drifting {ref.rate * 60:+.2f} MHz/min"
        elif state == HOLD:
            text = f"hold {fmt_duration(now - ref.since)}: {ref.message}"
        elif state == MOVED:
            text = f"{ref.message} · right-click"
        else:
            text = ref.message
        self.info.paint(text, color)
        self.unit.paint("nm", "dim")
        self._set_state({TRACKING: "ref", MOVED: "ref-moved"}.get(state, "ref-hold"))
        self.name.paint(ch.name, "dim")

    def _show_recovery(self, ch, now: float):
        prompt = ch.hop_prompt
        if prompt:
            if prompt != self._prompt:
                QtWidgets.QApplication.alert(self.win)  # flash the task bar: it waits for you
            self._prompt = prompt
            self.info.paint(f"next: {prompt}   (Continue, or right-click to stop)", "accent")
            self.info.setToolTip(prompt)
            return
        if self._prompt:
            self._prompt = ""
            self.info.setToolTip("")
        parts = [ch.hop_why, ch.hop_stage]
        if ch.current is not None:
            parts.append(f"{ch.current:.2f} mA")
        parts.append(fmt_duration(now - ch.hop_t))
        self.info.paint(" · ".join(p for p in parts if p), "warn")

    @staticmethod
    def _hop_text(ch, now: float) -> str:
        """The current, the DLC pro lock and the last recovery, for a laser with mode-hop recovery on."""
        if not ch.hop.enabled:
            return ""
        text = f"   {ch.current:.2f} mA" if ch.current is not None else ""
        if ch.hop.cavity:
            text += f"   cavity {ch.cavity_txt or ('on' if ch.cavity_on else 'off')}"
        if ch.hop.shg and ch.shg_txt:
            text += f"   SHG {ch.shg_txt}"
        if ch.note and now - ch.note_t < NOTE_S:
            text += f"   ⟲ {fmt_duration(now - ch.note_t)} ago"
        return text

    def _set_state(self, state: str):
        if state == self._state:
            return
        self._state = state
        for w in (self, self.btn):
            w.setProperty("state", state)
            w.style().unpolish(w)
            w.style().polish(w)
        label = "Ref" if state.startswith("ref") else {
            "on": "Locked", "engaging": "Locking", "recovering": "Recentering", "asking": "Continue"}.get(state, "Lock")
        self.btn.setText(label)

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
        ch = self.engine.channels[self.n]
        ref = self.engine.reference
        tip = f"channel {self.n}"
        if ch.freq > 0:
            tip += f"\n{ch.freq:.9g} THz\n{C_NM_THZ / ch.freq:.{self.win.digits}g} nm (vacuum)"
            if ref is not None and ref.channel != self.n and ref.active and ch.raw > 0:
                tip += (f"\nreferenced to channel {ref.channel}: the wavemeter reads "
                        f"{C_NM_THZ / ch.raw:.{self.win.digits}g} nm, corrected {(ch.freq - ch.raw) * 1e6:+.1f} MHz")
        if ref is not None and ref.channel == self.n:
            tip += f"\nreference laser: {ref.state}" + (f", {ref.message}" if ref.message else "")
        if ch.hop.enabled:
            hop = ch.hop
            tip += (f"\nmode-hop recovery on: {hop.i_nom:.2f} ± {hop.i_window:g} mA"
                    + (", takes over the DLC pro's cavity lock" if hop.cavity else "")
                    + (", checks the mode in the SHG cavity" if hop.shg else "")
                    + (", step by step" if hop.confirm else ""))
        if ch.note:
            tip += f"\nlast recovery: {ch.note}"
        self.wl.setToolTip(tip)

    def _show_setpoint(self, nm: float):
        self._sp = nm
        self.sp.blockSignals(True)
        if nm:
            self.sp.setDecimals(decimals(nm, self.win.digits))
        self.sp.setValue(nm)
        self.sp.blockSignals(False)

    def _setpoint_edited(self, value: float):
        nm = round(value, decimals(value, self.win.digits)) if value > 0 else None
        self.engine.set_setpoint(self.n, nm)
        self._show_setpoint(nm or 0.0)

    # ------------------------------------------------------------- actions
    def mouseDoubleClickEvent(self, ev):
        if self.childAt(event_pos(ev)) in (self.num, self.name):
            self.rename_dialog()
        else:
            super().mouseDoubleClickEvent(ev)

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

    def settings_dialog(self):
        ch = self.engine.channels[self.n]
        d = LockDialog(self, ch)
        if d.exec():
            self.engine.set_params(self.n, d.params())

    def hop_dialog(self):
        ch = self.engine.channels[self.n]
        d = HopDialog(self, ch)
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


class LinkDialog(QtWidgets.QDialog):
    def __init__(self, parent, engine: Engine, ch):
        super().__init__(parent)
        self.setWindowTitle(f"Link channel {ch.n}")
        self.host = QtWidgets.QComboBox()
        self.host.setEditable(True)
        self.host.addItems([d.key for d in engine.device_list()])
        self.host.setEditText(ch.device or "")
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

    def __init__(self, parent, ch):
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
        form.addRow("Nominal current", row(self.i_nom, "mA", use, now))
        self.i_window = number(p.i_window, 0.1, 50, 2, 0.1)
        form.addRow("Current stays within", row(self.i_window, "mA of nominal, and below the controller's limit"))
        self.v_window = number(p.v_window, 0.5, 100, 1, 0.5)
        form.addRow("Piezo moves at most", row(self.v_window, "V per recovery, inside the lock's offset limits"))
        self.tolerance = number(p.tolerance, 5, 100000, 0, 10)
        self.grace = number(p.grace, 0.2, 60, 1, 0.5)
        form.addRow("A hop is", row("off target by more than", self.tolerance, "MHz for", self.grace, "s"))
        self.margin = number(p.margin, 0.05, 5, 2, 0.05)
        form.addRow("Keep a hop at least", row(self.margin, "mA away, both ways"))
        self.step = number(p.step, 0.01, 1, 2, 0.01)
        form.addRow("Map in steps of", row(self.step, "mA"))
        self.per_hour = QtWidgets.QSpinBox()
        self.per_hour.setRange(1, 60)
        self.per_hour.setValue(p.max_per_hour)
        form.addRow("At most", row(self.per_hour, "recoveries an hour"))

        heading("Relocking the cavity lock")
        self.relock = QtWidgets.QComboBox()
        self.relock.addItems([label for _, label in self.RELOCK])
        self.relock.setCurrentIndex(next((k for k, (key, _) in enumerate(self.RELOCK) if key == p.relock), 0))
        form.addRow("Relock", self.relock)
        self.window = number(p.window, 10, 100000, 0, 10)
        form.addRow("The right peak is", row("within ±", self.window, "MHz of where the wavemeter puts the setpoint"))
        self.falc = QtWidgets.QCheckBox("Null the error signal's DC level with the FALC pro's input offset, by at most")
        self.falc.setChecked(p.falc)
        self.falc_max = number(p.falc_max, 0.001, 1.0, 3, 0.005)
        form.addRow(row(self.falc, self.falc_max, "V"))
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
        form.addRow(Text("On failure everything goes back to where it was, except that the cavity lock stays off.\n"
                         "Right-click the channel: Check setup… reads what a recovery would use; nothing is written.",
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
        if p.falc_sign:
            out.append("the FALC offset shows on the error signal")
        return ", ".join(out)

    def _who(self):
        cavity = self.who.currentIndex() == 1
        for w in (self.relock, self.window, self.falc, self.falc_max):
            w.setEnabled(cavity)

    def _forget(self):
        self.forget = True
        self.learned.paint("forgotten: measured again next time", "dim")

    def params(self) -> HopParams:
        p = self._p
        out = HopParams(
            enabled=self.enabled.isChecked(), cavity=self.who.currentIndex() == 1, shg=self.shg.isChecked(),
            relock=self.RELOCK[self.relock.currentIndex()][0], falc=self.falc.isChecked(),
            falc_max=self.falc_max.value(), window=self.window.value(), confirm=self.confirm.isChecked(),
            i_nom=round(self.i_nom.value(), 3), i_window=self.i_window.value(), v_window=self.v_window.value(),
            step=self.step.value(), margin=self.margin.value(), tolerance=self.tolerance.value(),
            grace=self.grace.value(), max_per_hour=self.per_hour.value())
        if not self.forget:
            for name in HopParams.LEARNED:
                setattr(out, name, getattr(p, name))
        return out


class Plot(QtWidgets.QWidget):
    """A small, plain plot: lines, dots, vertical lines and shaded spans, with the ranges marked."""

    def __init__(self, xlabel: str = "", ylabel: str = "", height: int = 160, ylim=None):
        super().__init__()
        self.xlabel, self.ylabel, self.ylim = xlabel, ylabel, ylim
        self.series = []  # (xs, ys, color, 'line' | 'dots' | 'cross', width)
        self.vlines = []  # (x, color)
        self.spans = []   # (x0, x1, color)
        self.hlines = []  # (y, color)
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
        ys = [y for s in self.series for y in s[1]] + [h[0] for h in self.hlines]
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
                p.setPen(QtGui.QPen(col, width))
                p.setBrush(col)
                for x, y in zip(sx, sy):
                    c = QtCore.QPointF(X(x), Y(y))
                    if style == "cross":
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
        plot = Plot("current (mA)", "GHz", 200)
        for leg, color in ((0, "dim"), (1, "accent"), (2, "ok")):
            pts = [pt for pt in m["points"] if pt[2] == leg and pt[1] is not None]
            plot.add([pt[0] for pt in pts], [pt[1] / 1e3 for pt in pts], color, "dots", 2)
        bad = [pt for pt in m["points"] if pt[3] and pt[1] is not None]
        plot.add([pt[0] for pt in bad], [pt[1] / 1e3 for pt in bad], "err", "cross", 1.5)
        for lo, hi, *_ in m.get("bands", [])[:3]:
            plot.spans.append((lo, hi, "ok"))
        if m.get("bands"):
            plot.vlines.append((m["bands"][0][2], "warn"))
        plot.hlines.append((0.0, "dim"))
        title = (f"map at {m['v']:.2f} V: frequency from the target against current, sweeping down (grey), up "
                 f"(blue), down (green); stable ranges shaded, the best's operating point dashed"
                 + ("; × where the SHG sweep showed two modes" if bad else ""))
        return self._box(title, plot)

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


class MainWindow(QtWidgets.QWidget):
    def __init__(self, engine: Engine, save):
        super().__init__()
        self.engine, self.save = engine, save
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
        self.rows_box = QtWidgets.QVBoxLayout()
        self.rows_box.setSpacing(6)
        v.addLayout(self.rows_box)
        self.placeholder = Text("waiting for the wavemeter…", "dim")
        self.placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder.setMinimumHeight(60)
        v.addWidget(self.placeholder)
        v.addStretch(1)
        status = QtWidgets.QHBoxLayout()
        status.setSpacing(18)
        self.st_wlm = Text("", "dim")
        self.st_ref = Text("", "dim")
        self.st_lan = Text("", "dim")
        status.addWidget(self.st_wlm)
        status.addWidget(self.st_ref)
        status.addStretch(1)
        status.addWidget(self.st_lan)
        v.addLayout(status)

        self.rows = {}
        self.row_list = []
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
        for row in self.rows.values():
            row.apply_fonts()
        self.engine.dirty = True

    # ------------------------------------------------------------- refresh
    def refresh(self):
        e = self.engine
        if e.layout_version != self._layout_version:
            self._rebuild()
        now = time.monotonic()
        if not self.isMinimized():
            for row in self.row_list:
                row.refresh(now)
        if now >= self._slow_at:
            self._slow_at = now + 0.5
            self._slow(now)

    def _slow(self, now: float):
        e = self.engine
        e.update_rate()
        e.tick()
        ref = e.reference
        if ref is None:
            self.st_ref.paint("", "dim")
        else:
            text = {TRACKING: f"ref ch{ref.channel}  wavemeter {ref.error:+.1f} MHz",
                    HOLD: f"ref ch{ref.channel}  hold {fmt_duration(now - ref.since)}",
                    MOVED: f"ref ch{ref.channel}  moved: right-click it",
                    ACQUIRING: f"ref ch{ref.channel}  acquiring"}[ref.state]
            self.st_ref.paint(text, {TRACKING: "accent", MOVED: "err"}.get(ref.state, "warn"))
        if e.source_ok:
            label = getattr(e.source, "label", "")
            self.st_wlm.paint(f"● {label}   {len(self.row_list)} ch   {e.rate:.0f} readings/s", "dim")
        else:
            self.st_wlm.paint(f"● {e.source_msg}", "err")
        if e.scanning:
            self.st_lan.paint("scanning LAN…", "dim")
        else:
            n = sum(1 for d in e.device_list() if not d.error)
            self.st_lan.paint(f"{n} controller{'s' if n != 1 else ''} on LAN", "dim")
        for row in self.row_list:
            row.update_tooltip()
        if (e.dirty and now - self._saved_at > 1.0) or (ref is not None and now - self._saved_at > 300):
            self._save()  # the reference's estimate too, now and then

    def _rebuild(self):
        e = self.engine
        with e.lock:
            self._layout_version = e.layout_version
            wanted = [n for n in e.active if not e.channels[n].hidden]
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
                if las.lock is not None:
                    sim.addAction(f"{who}: cavity lock jumps a peak", functools.partial(las.lock.jump, 1))
                    sim.addAction(f"{who}: goes multimode, without a hop", functools.partial(las.go_multimode, 1))
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
        self.engine.stop()
        super().closeEvent(ev)


def run(engine: Engine, save) -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName("Wavemeter Lock")
    app.setStyle("Fusion")
    app.setPalette(dark_palette())
    app.setStyleSheet(QSS)
    win = MainWindow(engine, save)
    for sig in (signal.SIGINT, signal.SIGTERM):  # Ctrl+C / kill: close cleanly, settings saved
        signal.signal(sig, lambda *_: win.close())
    engine.start()
    win.show()
    code = app.exec()
    win.deleteLater()  # the window gone before the interpreter takes Qt down (else PySide6 may crash on exit)
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    if hasattr(app, "shutdown"):  # PySide6
        app.shutdown()
    return code
