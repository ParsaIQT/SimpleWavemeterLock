"""Minimal Qt front-end.

Everything slow lives in the engine's threads. This module repaints changed
numbers on a 30 Hz timer and forwards clicks, so the window never stalls.
"""
from __future__ import annotations

import functools
import signal
import sys
import time

from .engine import Engine
from .lock import LockParams
from .qt import Qt, QtCore, QtGui, QtWidgets
from .wavemeter import C_NM_THZ

REFRESH_MS = 33
STALE_S = 3.0  # a channel without a new reading for this long is greyed out

C = {
    "bg": "#16171a", "row": "#1f2125", "line": "#2d3036", "fg": "#e8e9eb", "dim": "#80858f",
    "ok": "#38c172", "warn": "#f2b33d", "err": "#ff5a5a", "accent": "#4ea1ff",
}

QSS = f"""
QFrame#row {{ background: {C['row']}; border-radius: 6px; border-left: 4px solid {C['row']}; }}
QFrame#row[state="on"] {{ border-left-color: {C['ok']}; }}
QFrame#row[state="engaging"] {{ border-left-color: {C['warn']}; }}
QFrame#row[state="error"] {{ border-left-color: {C['err']}; }}
QPushButton#lock {{ min-width: 78px; min-height: 30px; border-radius: 5px; font-weight: 600;
    border: 1px solid {C['line']}; background: #2a2d33; }}
QPushButton#lock:hover {{ border-color: {C['accent']}; }}
QPushButton#lock[state="on"] {{ background: #1d6b3e; border-color: {C['ok']}; }}
QPushButton#lock[state="engaging"] {{ background: #6b5418; border-color: {C['warn']}; }}
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

        self.num = Text(str(n), "dim", win.bold_font)
        self.name = Text("", "dim")
        self.wl = Text("", "fg", win.big_font)
        self.unit = Text("nm", "dim")
        self.dev = Text("", "fg", win.mid_font)
        self.sp = DigitSpinBox()
        self.sp.setFont(win.mono_font)
        self.sp.valueChanged.connect(self._setpoint_edited)
        self.sp.editingFinished.connect(lambda: self.sp.lineEdit().setModified(False))
        self.info = Text("", "dim")
        self.btn = QtWidgets.QPushButton("Lock")
        self.btn.setObjectName("lock")
        self.btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.btn.clicked.connect(lambda: self.engine.toggle(n))
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
        sp_box.addWidget(Text("set", "dim"))
        sp_box.addWidget(self.sp)
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
        valid = ch.freq > 0
        if valid:
            self.wl.paint(fmt_nm(C_NM_THZ / ch.freq, self.win.digits), "dim" if now - ch.t > STALE_S else "fg")
        else:
            self.wl.paint(ch.status or "—", "err")
        state = ch.state
        if valid and ch.setpoint_nm:
            self.dev.paint(fmt_mhz(ch.error_mhz), {"on": "ok", "engaging": "warn"}.get(state, "fg"))
        else:
            self.dev.paint("", "fg")
        sp = ch.setpoint_nm or 0.0
        if sp != self._sp and not (self.sp.hasFocus() and self.sp.lineEdit().isModified()):
            self._show_setpoint(sp)
        if ch.message:
            self.info.paint(ch.message, "err")
        elif ch.device:
            dev = self.engine.devices.get(ch.device)
            ctrl = self.engine.controllers.get(ch.device)
            where = f"laser{ch.laser} @ {dev.name if dev else ch.device}"
            if ctrl is not None and ctrl.online is False:
                self.info.paint(f"{where}   offline", "err")
            else:
                off = "…" if ch.offset is None else f"{ch.offset:.3f} {ch.unit}"
                self.info.paint(f"{where}   {off}", "dim")
        else:
            self.info.paint("right-click to link a laser", "dim")
        self._set_state(state if state != "off" else ("error" if ch.message else "off"))
        self.name.paint(ch.name, "dim")

    def _set_state(self, state: str):
        if state == self._state:
            return
        self._state = state
        for w in (self, self.btn):
            w.setProperty("state", state)
            w.style().unpolish(w)
            w.style().polish(w)
        self.btn.setText({"on": "Locked", "engaging": "Locking"}.get(state, "Lock"))

    def update_tooltip(self):
        ch = self.engine.channels[self.n]
        tip = f"channel {self.n}"
        if ch.freq > 0:
            tip += f"\n{ch.freq:.9g} THz\n{C_NM_THZ / ch.freq:.{self.win.digits}g} nm (vacuum)"
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
        m = QtWidgets.QMenu(self)
        m.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
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
        m.addAction("Rename…", self.rename_dialog)
        m.addAction("Hide channel", lambda: e.set_hidden(self.n, True))
        self._menu = m
        m.popup(self.mapToGlobal(pos))

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
        self.st_wlm = Text("", "dim")
        self.st_lan = Text("", "dim")
        status.addWidget(self.st_wlm)
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
        if e.dirty and now - self._saved_at > 1.0:
            self._save()

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
        self._menu = m
        m.popup(self.mapToGlobal(pos))

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
    return app.exec()
