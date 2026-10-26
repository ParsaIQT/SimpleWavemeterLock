import importlib
import re
import socket
import time

import pytest

from conftest import wait_for

qt = pytest.importorskip("wmlock.qt", exc_type=ImportError)
from wmlock import gui  # noqa: E402
from wmlock.engine import Engine  # noqa: E402
from wmlock.qt import QtCore, QtWidgets  # noqa: E402


@pytest.fixture(scope="module")
def app():
    a = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    a.setStyle("Fusion")
    a.setPalette(gui.dark_palette())
    a.setStyleSheet(gui.QSS)
    yield a
    # every window gone before the interpreter takes Qt down (else PySide6 may crash on exit)
    for w in a.topLevelWidgets():
        w.close()
        w.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    a.processEvents()
    if hasattr(a, "shutdown"):  # PySide6: destroy the application now, in order
        a.shutdown()


def spin(app, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.005)


def test_formatting():
    assert gui.fmt_nm(396.95912534, 9) == "396.959125"
    assert gui.fmt_nm(1550.1234567, 9) == "1550.12346"
    assert gui.fmt_nm(85.123456789, 9) == "85.1234568"
    assert gui.fmt_mhz(1.234) == "+1.2 MHz"
    assert gui.fmt_mhz(-25000) == "-25.000 GHz"


def test_digit_stepping(app):
    sb = gui.DigitSpinBox()
    sb.setValue(396.959125)
    le = sb.lineEdit()
    le.setCursorPosition(len(le.text()))  # after the last digit: 1e-6
    sb.stepBy(1)
    assert sb.value() == pytest.approx(396.959126)
    le.setCursorPosition(3)  # left of the dot: units
    sb.stepBy(-2)
    assert sb.value() == pytest.approx(394.959126)
    assert le.cursorPosition() == 3
    le.setCursorPosition(4)  # right after the dot: still units
    sb.stepBy(1)
    assert sb.value() == pytest.approx(395.959126)
    le.setCursorPosition(6)  # second decimal
    sb.stepBy(1)
    assert sb.value() == pytest.approx(395.969126)
    sb.setValue(0)
    sb.stepBy(1)
    assert sb.value() == 0  # no setpoint: stepping does nothing


def test_window_shows_channels_and_links_from_menu(app, demo):
    e = Engine({"channels": {"1": {"name": "397"}}}, demo.make_source, discovery_targets=["127.0.0.1"],
               extra_devices=[d.key for d in demo.dlcs])
    saved = []
    win = gui.MainWindow(e, lambda: saved.append(e.to_config()))
    e.start()
    win.show()
    try:
        spin(app, 0.3)
        assert wait_for(lambda: (app.processEvents(), not e.scanning)[1], 3)
        spin(app, 0.3)
        assert list(win.rows) == [1, 2, 3, 4, 5, 6, 7]
        row = win.rows[1]
        assert len(row.wl.text()) == 10 and row.wl.text().startswith("396.959")
        assert win.rows[5].wl.text() == "no signal"
        assert row.name.text() == "397"

        row.show_menu(row.rect().center())
        link = next(a for a in row._menu.actions() if a.text() == "Link laser").menu()
        ctrl = next(a for a in link.actions() if "Ca+ blue" in a.text()).menu()
        laser1 = ctrl.actions()[0]
        assert "DL pro 397" in laser1.text()
        laser1.trigger()
        row._menu.close()
        assert e.channels[1].device == demo.dlcs[0].key and e.channels[1].laser == 1

        row.btn.click()  # lock at the present reading
        spin(app, 0.5)
        assert e.channels[1].state == "on" and row.btn.text() == "Locked"
        assert row.dev.text().endswith("MHz")
        row.btn.click()
        spin(app, 0.1)
        assert e.channels[1].state == "off" and row.btn.text() == "Lock"

        e.set_hidden(5, True)
        spin(app, 0.1)
        assert 5 not in win.rows
        win.set_font_size(30)
        assert win.gui["font_pt"] == 30
        spin(app, 1.2)
        assert saved  # settings were saved while running
    finally:
        win.close()
        spin(app, 0.1)


def test_a_stuck_wlm_restarted_is_said_and_can_be_left_alone(app):
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.05), exposure=0.01)
    e = Engine({"wlm_restart": False}, demo.make_source, discovery_targets=["127.0.0.1"])
    assert e.source.restart is False  # (as its settings say)
    win = gui.MainWindow(e, lambda: None)
    e.start()
    win.show()
    try:
        assert wait_for(lambda: (app.processEvents(), 1 in win.rows)[1], 3)
        win.show_menu(win.rect().center())
        tick = next(a for a in win._menu.actions() if a.text() == "Restart the WLM software if it gets stuck")
        assert tick.isCheckable() and not tick.isChecked()
        tick.trigger()  # (on)
        win._menu.close()
        assert e.source.restart and e.config["wlm_restart"] is True and e.to_config()["wlm_restart"] is True
        e.source.STALL_S, e.source.RESTART_S = 0.5, 0.3
        win.show_menu(win.rect().center())
        sim = next(a.menu() for a in win._menu.actions() if a.text() == "Simulate a fault")
        next(a for a in sim.actions() if a.text().startswith("WLM software stuck")).trigger()
        win._menu.close()
        assert wait_for(lambda: (app.processEvents(), bool(e.source.restarted))[1], 5)
        spin(app, 0.6)  # (the status bar, twice a second)
        assert "   ⟲ WLM " in win.st_wlm.text() and win.st_wlm.toolTip().startswith("the WLM software restarted")
    finally:
        win.close()
        spin(app, 0.1)
        demo.close()


def test_reference_row(app):
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.05), exposure=0.01)
    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
    win = gui.MainWindow(e, lambda: None)
    e.start()
    win.show()
    try:
        assert wait_for(lambda: (app.processEvents(), 6 in win.rows)[1], 3)
        row = win.rows[6]
        row.show_menu(row.rect().center())
        use = next(a for a in row._menu.actions() if a.text().startswith("Use as reference"))
        use.trigger()
        row._menu.close()
        spin(app, 0.6)  # the status bar updates twice a second
        assert row.btn.text() == "Ref" and row.sp_label.text() == "ref" and not row.sp.isVisible()
        assert "acquiring" in row.info.text() and "acquiring" in win.st_ref.text()
        assert wait_for(lambda: (app.processEvents(), e.reference.state == "tracking")[1], 20)
        spin(app, 0.6)
        assert row.info.text().startswith("reference · wavemeter") and row.anchor_text.text().startswith("729.3475")
        assert win.rows[1].unit._color == "accent"  # other channels are referenced now
        demo.laser(6).mode_hop(1500, 30)
        assert wait_for(lambda: (app.processEvents(), e.reference.state == "hold")[1], 3)
        spin(app, 0.6)
        assert row.info.text().startswith("hold") and row._state == "ref-hold" and "hold" in win.st_ref.text()
        row.show_menu(row.rect().center())
        items = [a.text() for a in row._menu.actions() if a.text()]
        assert items[:2] == ["Reference settings…", "Stop using as reference"]  # nothing to decide while unsteady
        row._menu.close()
        e.clear_reference()
        spin(app, 0.6)
        assert row.btn.text() == "Lock" and row.sp.isVisible() and win.st_ref.text() == ""
    finally:
        win.close()
        spin(app, 0.1)
        demo.close()


def test_reference_dialog_round_trip(app):
    from wmlock.reference import Reference, RefParams

    ref = Reference(6, RefParams(tau=20, max_drift=1.5, max_jump=40, scale=False), anchor=411.0425, error=3.0)
    d = gui.ReferenceDialog(None, ref)
    assert d.params() == ref.p and d.new_anchor_thz() is None
    d.anchor.setValue(729.0)
    assert d.new_anchor_thz() == pytest.approx(299792.458 / 729.0)


def test_dialogs_read_back_values(app, demo):
    from wmlock.engine import Channel

    ch = Channel(3, {"link": {"device": "10.0.0.7", "laser": 2}, "lock": {"ki": 0.4}})
    e = Engine({}, demo.make_source, extra_devices=["10.0.0.7"])
    assert gui.LinkDialog(None, e, ch).values() == ("10.0.0.7", 2)
    d = gui.LockDialog(None, ch)
    assert d.boxes["ki"].value() == 0.4
    d.boxes["ki"].setValue(-0.8)
    p = d.params()
    assert p.ki == -0.8 and p.capture == ch.lock.p.capture


def test_hop_dialog_round_trip(app):
    from wmlock.engine import Channel
    from wmlock.modehop import HopParams

    ch = Channel(7, {"hop": {"enabled": True, "cavity": True, "shg": True, "i_nom": 151.25, "kv": 402.0,
                             "spacing": 6010.0, "lp_kind": "t", "lp_code": 116, "peak_y": 0.8, "relock": "lockpoint",
                             "trans_locked": 0.79, "trans_zero": 0.004, "shg_lock": 1.21}})
    ch.current = 151.4
    d = gui.HopDialog(None, ch)
    assert d.params() == ch.hop
    assert "lockpoint type 't' = 116" in d.learned.toolTip() and "transmission 0.790 V locked" in d.learned.toolTip()
    assert "transmission zero 0.004 V" in d.learned.toolTip()
    assert d.slip_levels.text() == "now: zero 0.004 V, locked 0.790 V: relock below 0.397 V on average"
    assert not d.tolerance.isEnabled() and d.peak_tol.isEnabled()  # locked to a cavity: off target is the wrong peak
    d.i_window.setValue(1.5)
    d.confirm.setChecked(True)
    d.falc_max.setValue(0.02)
    d.peak_tol.setValue(40.0)
    d.lock_min.setValue(0.3)
    assert d.slip_levels.text() == "now: zero 0.004 V, locked 0.790 V: relock below 0.240 V on average"
    d.who.setCurrentIndex(0)
    assert not d.relock.isEnabled() and not d.falc.isEnabled() and not d.lock_min.isEnabled()  # only for a cavity lock
    assert not d.slip_levels.isEnabled()
    assert d.tolerance.isEnabled()
    d._forget()
    p = d.params()
    assert p == HopParams(enabled=True, cavity=False, shg=True, relock="lockpoint", i_nom=151.25, i_window=1.5,
                          confirm=True, falc_max=0.02, peak_tol=40.0, lock_min=0.3)  # measured values forgotten


def test_hop_dialog_fits_the_screen_with_ok_and_cancel_in_view(app):
    from wmlock.engine import Channel
    from wmlock.qt import Qt, QtGui

    ch = Channel(5, {"link": {"device": "192.168.1.30", "laser": 1}, "hop": {"enabled": True, "method": "catch"}})
    d = gui.HopDialog(None, ch)
    free = QtGui.QGuiApplication.primaryScreen().availableGeometry()  # (offscreen: 800 x 800, too small for it all)
    area = d.findChild(QtWidgets.QScrollArea)
    bb = d.findChild(QtWidgets.QDialogButtonBox)
    assert area.widget().sizeHint().height() > free.height()  # (its settings do not all fit: they scroll...)
    assert d.width() <= free.width() and d.height() <= free.height() and d.minimumSizeHint().height() < 300
    assert bb.parent() is d and not area.isAncestorOf(bb)  # (...and OK and Cancel, below them, never scroll away)
    d.show()
    app.processEvents()
    assert bb.geometry().bottom() <= d.height() and bb.isVisible()
    longest = max(d.method.fontMetrics().horizontalAdvance(label) for _, label in gui.HopDialog.METHOD)
    assert d.method.sizeHint().width() < 0.5 * longest  # (once as wide as its longest item: the dialog 1732 px wide)
    whole = gui.HopDialog.METHOD[1][1]
    assert d.method.currentText() == whole and d.method.itemData(1, Qt.ItemDataRole.ToolTipRole) == whole
    assert d.method.view().minimumWidth() > d.method.sizeHint().width()  # (its list as wide as its items)
    d.close()


def test_hop_dialog_offers_the_lasers_model(app):
    from wmlock.engine import Channel
    from wmlock.lasermodel import Comb, LaserModel

    ch = Channel(5, {"link": {"device": "192.168.1.30", "laser": 1}, "hop": {"enabled": True, "method": "model"}})
    d = gui.HopDialog(None, ch)
    assert d.method.currentText().startswith("model: its laser's model, fitted to its last map")
    assert d.model_info.text().startswith("its laser's model: none yet (map it")
    assert d.params().method == "model"
    model = LaserModel(f_ref=607.42, i0=64.0, v0=67.0, t0=0.0, ext=Comb(8.26), chip=Comb(87.4), right=0.77)
    ch.adopt_model(model, "map 3.sqlite")
    ch.model_dict["t"] = "2026-10-25 10:00"
    d = gui.HopDialog(None, ch)
    assert d.model_info.text() == ("its laser's model: from map 3.sqlite (2026-10-25 10:00), 77% of that map's points "
                                   "on the right mode")
    d.method.setCurrentIndex(0)
    assert d.params().method == "auto" and d.method.currentText().startswith("its model, if it has one and no SHG")


def test_hop_dialog_shows_the_optimum_and_takes_it(app):
    from wmlock.engine import Channel

    ch = Channel(7, {"hop": {"enabled": True, "cavity": True, "shg": True, "i_nom": 150.0, "shg_ratio": 0.98}})
    ch.device = "192.168.1.20"
    taken = []
    d = gui.HopDialog(None, ch, take=lambda: taken.append(1))
    assert d.optimum.text().startswith("not taken: taken when Relock is first clicked")
    assert "SHG peak in a sweep 0.98 of the signal locked" in d.learned.toolTip()
    take = next(b for b in d.findChildren(QtWidgets.QPushButton) if b.text() == "Take now")
    take.click()
    assert taken == [1]
    ch.hop.opt_i, ch.hop.opt_v, ch.hop.opt_shg, ch.hop.opt_t = 150.31, 70.12, 1.231, "2026-10-19 21:03"
    ch.hop.i_nom = 150.31  # (as the engine takes it, a moment later)
    spin(app, 0.5)
    assert d.optimum.text() == "150.31 mA, 70.12 V, SHG 1.231 V locked (2026-10-19 21:03)"
    assert d.i_nom.value() == pytest.approx(150.31)
    d.catch_pct.setValue(15)
    d.pc_try.setValue(20.0)
    p = d.params()
    assert p.catch_min == 0.85 and p.pc_try == 20.0 and p.opt_shg == 1.231 and p.i_nom == 150.31
    assert (p.reset_up, p.reset_slow, p.reset_tries, p.ease_step, p.upkeep, p.upkeep_back) == (3.0, 1.0, 2, 0.001, 0.0,
                                                                                              0.02)
    d.reset_up.setValue(2.5)
    d.reset_tries.setValue(3)
    d.ease_step.setValue(0.002)
    d.upkeep.setValue(60)
    d.upkeep_back.setValue(0.03)
    p = d.params()
    assert (p.reset_up, p.reset_slow, p.reset_tries, p.ease_step, p.upkeep, p.upkeep_back) == (2.5, 1.0, 3, 0.002, 60.0,
                                                                                              0.03)
    d._clear_optimum()
    assert d.params().opt_i == 0 and not d.params().optimum and d.params().shg_ratio == 0.98


def test_recovery_and_setup_dialogs(app, demo):
    rec = {"time": "2026-10-13 12:00:00", "channel": 7, "ok": True, "outcome": "jumped +6.000 GHz: relocked",
           "took_s": 42.0, "notes": [[1.0, "map 1"], [None, "locked"]],
           "maps": [{"v": 70.0, "points": [[150.0, 6000.0, 0, False], [149.9, 6025.0, 0, True],
                                           [150.1, -10.0, 1, False]],
                     "bands": [[150.6, 151.8, 151.2, -300.0]]}],
           "sweeps": [{"why": "operating point", "i": 151.2, "ok": True, "text": "single mode", "y": [0.0, 1.0, 0.0]}],
           "scan": {"x_pred": 70.0, "window_v": 0.375, "cands": [[70.01, 0.8, "t"], [71.4, 0.15, "t"]],
                    "choice": [70.01, 0.8, "t", "peak at 70.010 V"], "x": [69.0, 70.0, 71.0], "y": [0.0, 0.8, 0.0]},
           "falc": {"name": "falc1:input:offset", "offset0": 0.0, "dc0": 0.009, "error": [0.009, 0.01, 0.008],
                    "offset": -0.009, "dc": 0.0001},
           "walk": [[70.0, 149.6, 6000.0, True], [69.5, 149.6, 5800.0, False], [69.5, 149.7, 5780.0, True],
                    [55.0, 149.7, 1.0, None]],
           "catch": [{"v": 70.0, "go": 1, "caught": False, "right": 0.0, "tol": 880.0, "min": 0.9,
                      "points": [[152.0, -2200.0, None], [151.9, -150.0, 0.7], [151.8, 2200.0, None]]},
                     {"v": 100.0, "go": 2, "caught": True, "right": 12000.0, "tol": 880.0, "min": 0.9,
                      "points": [[152.0, 9800.0, None], [151.9, 11950.0, 0.95]]}]}
    d = gui.RecoveryDialog(None, rec)
    d.show()
    spin(app, 0.1)
    tabs = d.findChild(QtWidgets.QTabWidget)
    assert [tabs.tabText(k) for k in range(tabs.count())] == ["Steps", "Catch", "Walk", "Map", "SHG sweeps",
                                                              "Cavity peaks", "Error signal"]
    for k in range(tabs.count()):
        tabs.setCurrentIndex(k)
        spin(app, 0.05)  # every plot paints
    d.close()
    empty = gui.Plot()
    empty.resize(300, 100)
    empty.grab()  # nothing to show, no trouble

    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
    e.start()
    try:
        assert wait_for(lambda: 7 in e.channels, 5)
        e.link(7, demo.dlcs[2].key, 1)
        from wmlock.modehop import HopParams

        e.set_hop_params(7, HopParams(enabled=True, cavity=True, shg=True))
        s = gui.SetupDialog(None, e, 7)
        assert wait_for(lambda: (app.processEvents(), e.channels[7].setup is not None)[1], 5)
        spin(app, 0.2)
        html = s.view.toPlainText()
        assert "Cavity lock" in html and "relock by lockpoint" in html and "SHG" in html
        s.close()
    finally:
        e.stop()


def test_row_shows_a_recovery(app):
    from wmlock.modehop import HopParams
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
    demo.laser(1).drift = 0.0
    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
    win = gui.MainWindow(e, lambda: None)
    e.start()
    win.show()
    try:
        assert wait_for(lambda: (app.processEvents(), 1 in win.rows and e.channels[1].freq > 0)[1], 5)
        e.link(1, demo.dlcs[0].key, 1)
        e.set_hop_params(1, HopParams(enabled=True, step=0.2))
        e.setpoint_to_current(1)
        e.engage(1)
        ch, row = e.channels[1], win.rows[1]
        assert wait_for(lambda: (app.processEvents(), ch.guard.armed and ch.current is not None)[1], 5)
        spin(app, 0.1)
        assert row.btn.text() == "Locked" and row.info.text().endswith("150.00 mA")
        row.show_menu(row.rect().center())
        items = [a.text() for a in row._menu.actions()]
        assert "Mode-hop recovery…" in items and "Recenter now" in items
        row._menu.close()
        demo.laser(1).hop_mode(+1)
        assert wait_for(lambda: (app.processEvents(), ch.state == "recovering")[1], 3)
        spin(app, 0.3)
        assert row.btn.text() == "Recentering" and row._state == "recovering"
        assert row.info.text().startswith("jumped +") and "mapping" in row.info.text() and "mA" in row.info.text()
        row.show_menu(row.rect().center())
        assert "Stop recentering" in [a.text() for a in row._menu.actions()]
        row._menu.close()
        assert wait_for(lambda: (app.processEvents(), ch.state == "on")[1], 30), ch.message
        spin(app, 0.6)
        assert row.btn.text() == "Locked" and "⟲" in row.info.text()
        assert "last recovery: jumped" in row.wl.toolTip()
    finally:
        win.close()
        spin(app, 0.1)
        demo.close()


def test_row_asks_before_each_step(app):
    from wmlock.modehop import HopParams
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
    win = gui.MainWindow(e, lambda: None)
    e.start()
    win.show()
    try:
        assert wait_for(lambda: (app.processEvents(), 7 in win.rows and e.channels[7].freq > 0)[1], 5)
        e.link(7, demo.dlcs[2].key, 1)
        e.setpoint_to_current(7)
        e.set_hop_params(7, HopParams(enabled=True, cavity=True, shg=True, confirm=True))
        ch, row = e.channels[7], win.rows[7]
        spin(app, 0.1)
        assert row.btn.text() == "Relock" and "does not lock it to the wavemeter" in row.btn.toolTip()
        row._clicked()  # Relock: watch it (it is locked, on target: nothing else happens)
        assert wait_for(lambda: (app.processEvents(), ch.guard.armed)[1], 5)
        assert row.btn.text() == "Watching" and row._state == "on" and not ch.lock.active
        demo.laser(7).lock.jump(1)
        assert wait_for(lambda: (app.processEvents(), bool(ch.hop_prompt))[1], 5)
        spin(app, 0.1)
        assert row.btn.text() == "Continue" and row._state == "asking"
        assert row.info.text().startswith("next: Take the laser over")
        row.show_menu(row.rect().center())
        items = [a.text() for a in row._menu.actions()]
        assert any(t.startswith("Continue: Take the laser over") for t in items) and "Stop recentering" in items
        row._menu.close()
        row._clicked()  # Continue
        assert wait_for(lambda: (app.processEvents(), ch.hop_prompt.startswith((  # (the current parked high first)
            "On the right mode: the current up a step at a time while its SHG peak stays at its best",)))[1], 5)
        e.cancel_recovery(7)
        assert wait_for(lambda: (app.processEvents(), ch.state == "off")[1], 5)
        assert "recovery stopped" in ch.message and "cavity lock left off" in ch.message
        spin(app, 0.1)
        assert row.btn.text() == "Relock"  # click to relock it
    finally:
        win.close()
        spin(app, 0.1)
        demo.close()


def test_row_says_when_it_does_not_lase(app):
    """With recovery on, the TA's seed power is shown; too little, and the row says NOT LASING (not
    armed, nothing is done about it)."""
    from wmlock.modehop import HopParams
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
    win = gui.MainWindow(e, lambda: None)
    e.start()
    win.show()
    try:
        assert wait_for(lambda: (app.processEvents(), 7 in win.rows and e.channels[7].freq > 0)[1], 5)
        e.link(7, demo.dlcs[2].key, 1)
        e.setpoint_to_current(7)
        e.set_hop_params(7, HopParams(enabled=True, cavity=True, shg=True, seed_ok=20.0))
        ch, row = e.channels[7], win.rows[7]
        assert wait_for(lambda: (app.processEvents(), ch.seed is not None and ch.lasing is True)[1], 5)
        spin(app, 0.1)
        assert re.search(r"seed (19\.9|20\.[01]) mW", row.info.text()) and "NOT LASING" not in row.info.text()
        demo.laser(7).stop_lasing()
        assert wait_for(lambda: (app.processEvents(), ch.lasing is False)[1], 5)
        spin(app, 0.1)
        assert "NOT LASING: seed 0." in row.info.text() and row.info._color == "err"
        assert ch.state == "off"  # not armed: left alone
    finally:
        win.close()
        spin(app, 0.1)
        demo.close()


def test_row_says_when_its_cavity_lock_keeps_losing_it(app):
    """A cavity-locked laser's transmission is shown in volts, against its level locked; far below on average (its
    lock keeps losing a master on two modes), the row says so, and where the limit is (not armed, nothing is done
    about it)."""
    from wmlock.modehop import HopParams
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
    e.LEVEL_S = 3.0
    win = gui.MainWindow(e, lambda: None)
    e.start()
    win.show()
    try:
        assert wait_for(lambda: (app.processEvents(), 7 in win.rows and e.channels[7].freq > 0)[1], 5)
        e.link(7, demo.dlcs[2].key, 1)
        e.setpoint_to_current(7)
        e.set_hop_params(7, HopParams(enabled=True, cavity=True, shg=True, trans_locked=0.8))
        ch, row = e.channels[7], win.rows[7]
        assert wait_for(lambda: (app.processEvents(), len(ch.trans_hist) >= 5 and ch.cavity_on is True)[1], 5)
        spin(app, 0.1)
        assert re.search(r"cavity locked · transmission 0\.[78]\d\d V \(locked 0\.[78]\d\d\)", row.info.text()), \
            row.info.text()
        demo.laser(7).go_multimode(1)
        assert wait_for(lambda: (app.processEvents(), "CAVITY LOCK SLIPPING" in row.info.text())[1], 10)
        assert re.search(r"transmission 0\.[0-3]\d\d V \(locked 0\.[78]\d\d, relock below 0\.[34]\d\d\)",
                         row.info.text()), row.info.text()
        assert row.info._color == "warn" and ch.state == "off"  # not armed: left alone
    finally:
        win.close()
        spin(app, 0.1)
        demo.close()


def test_row_of_a_channel_switched_off_stays_while_watched(app):
    """Someone unticks the channel in the WLM software: the row of a laser that is watched stays, and says so; so
    does one that Relock could watch; any other goes (the channel is not measured)."""
    from wmlock.modehop import HopParams
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004)
    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
    win = gui.MainWindow(e, lambda: None)
    e.start()
    win.show()
    try:
        assert wait_for(lambda: (app.processEvents(), 7 in win.rows and e.channels[7].freq > 0)[1], 5)
        e.link(7, demo.dlcs[2].key, 1)
        e.setpoint_to_current(7)
        e.set_hop_params(7, HopParams(enabled=True, cavity=True, shg=True))
        ch, row = e.channels[7], win.rows[7]
        row._clicked()  # Relock: watched
        assert wait_for(lambda: (app.processEvents(), ch.guard.armed)[1], 5)
        e.source.untick(7)
        assert wait_for(lambda: (app.processEvents(), ch.off)[1], 2)
        spin(app, 0.7)
        assert 7 in win.rows and win.rows[7] is row and row.btn.text() == "Watching"
        assert row.wl.text() == "off" and "wavemeter channel off: watched by what its DLC pro says" in \
            row.info.text()
        row._clicked()  # stop watching: still shown, for Relock
        spin(app, 0.7)
        assert win.rows.get(7) is row and row.btn.text() == "Relock" and "Relock has it watched" in row.info.text()
        e.set_hop_params(7, HopParams(enabled=False, cavity=True, shg=True))  # nothing to watch it for: not shown
        assert wait_for(lambda: (app.processEvents(), 7 not in win.rows)[1], 2)
        e.source.tick(7)
        assert wait_for(lambda: (app.processEvents(), 7 in win.rows)[1], 2)
    finally:
        win.close()
        spin(app, 0.1)
        demo.close()


def test_another_pcs_wavemeter_is_framed_under_its_name_and_adjusted_from_here(app):
    """Another PC found: its rows show in a frame of their own under its wavemeter's name (and this PC's under its
    own), as they show there; clicks and setpoints typed here are done there; folded away, it sends less."""
    from test_share import PC

    a, b = PC("Lab A"), None
    win = gui.MainWindow(a.engine, lambda: None, a.share)
    a.start()
    win.show()
    try:
        spin(app, 0.3)
        assert not win.local.framed and not win.local.header.isVisible()  # (alone: as ever)
        b = PC("Lab B", peers=[a]).start()
        assert wait_for(lambda: (app.processEvents(), bool(win.panel_list) and 7 in win.panel_list[0].rows)[1], 5)
        spin(app, 0.6)
        panel = win.panel_list[0]
        assert panel.header.title.text() == "Lab B" and panel.framed and win.local.framed
        colors = gui.panel_colors(["Lab A", "Lab B"])
        assert panel.header.title._color == colors["Lab B"] != colors["Lab A"] == win.local.header.title._color
        here = "this PC   " + socket.gethostname() + "".join(f"  {', '.join(gui.lan_addresses())}"
                                                            for _ in [1] if gui.lan_addresses())
        assert win.local.header.title.text() == "Lab A" and win.local.header.detail.text() == here  # (its name and
        #                                                           addresses on the LAN, as another PC's shows its own)
        assert win.windowTitle() == "Wavemeter Lock · Lab A" and win.st_net.text().startswith("⇄ 1 other PC")
        assert panel.header.status.text().endswith("readings/s")
        row = panel.rows[1]
        assert row.wl.text().startswith("396.959") and row.name.text() == "397"
        assert row.btn.text() == "Lock" and row.info.text() == "no laser linked" and row.btn.isEnabled()
        assert "on Lab B" in row.wl.toolTip()

        e = b.engine
        e.link(1, b.demo.dlcs[0].key, 1)
        row._clicked()  # Lock, there
        assert wait_for(lambda: (app.processEvents(), e.channels[1].state == "on")[1], 3)
        assert wait_for(lambda: (app.processEvents(), row.btn.text() == "Locked")[1], 2)
        assert e.channels[1].remote_note == f"locked from {a.share.who}" and row.info.text().startswith("laser1 @ ")
        row.sp.setValue(396.9592)  # (as typed, Enter)
        assert wait_for(lambda: e.channels[1].setpoint_nm == 396.9592, 2)
        spin(app, 0.5)
        assert row.sp.value() == pytest.approx(396.9592) and row.dev.text().endswith("MHz")
        row.show_menu(row.rect().center())
        items = [x.text() for x in row._menu.actions() if x.text()]
        assert items == ["Lab B: channel 1 397", "Setpoint = current reading", "Lock settings…", "Recenter now",
                         "Rename…"]
        row._menu.close()
        row._clicked()  # Locked: unlock, there
        assert wait_for(lambda: (app.processEvents(), e.channels[1].state == "off")[1], 3)

        QtTest = importlib.import_module(qt.BINDING + ".QtTest")
        QtTest.QTest.mouseClick(panel.header, QtCore.Qt.MouseButton.LeftButton)  # fold it away: it sends less
        spin(app, 0.3)
        assert panel.folded and not row.isVisible() and win.gui["folded"] == ["Lab B"]
        assert panel.peer.wanted == "slow" and panel.header.detail.text().endswith("7 channels: 7 off")
        assert wait_for(lambda: b.share.peers[a.share.id].want == "slow", 3)
        panel.fold(False)
        spin(app, 0.2)
        assert row.isVisible() and win.gui["folded"] == [] and panel.peer.wanted == "fast"

        b.share.settings["control"] = False  # there, the others may only watch
        assert wait_for(lambda: (app.processEvents(), not row.btn.isEnabled() and row.sp.isReadOnly())[1], 4)
        assert panel.header.detail.text().endswith("watch only")
        b.close()
        assert wait_for(lambda: (app.processEvents(), panel.header.status.text().startswith("closed at"))[1], 3)
        assert row.wl._color == "dim"
    finally:
        win.close()
        spin(app, 0.1)
        if b is not None:
            b.close()
        a.demo.close()


def test_network_dialog_round_trip(app):
    from wmlock.share import Share

    e = Engine({}, lambda sink: None)
    settings = {"label": "Lab 2 · WS7", "group": "ions", "key": "k", "peers": ["lab3-pc", "10.1.2.3:5000"],
                "control": False, "port": 47000}
    d = gui.ShareDialog(None, Share(e, settings))
    assert d.values() == {"on": True, **settings}
    assert d.found.text().startswith("no other PC yet")
    d.label.setText("")
    assert d.values()["label"] == "" and d.label.placeholderText() == socket.gethostname()  # (its default name)
    d.peers.setText("a; b ,c")
    assert d.values()["peers"] == ["a", "b", "c"]


def test_network_dialog_says_and_sets_how_artiq_calls(app):
    from wmlock.rpc import Api, Server
    from wmlock.share import Share

    e = Engine({}, lambda sink: None)
    settings = {"label": "Lab 2", "artiq_from": ["artiq-master"]}
    share = Share(e, settings)
    api = Server(Api(lambda: []), settings)
    d = gui.ShareDialog(None, share, api=api)
    v = d.values()
    assert v["artiq"] is True and v["artiq_port"] == 3280 and v["artiq_from"] == ["artiq-master"]
    assert "ARTIQ's calls: answered on nothing yet" in d.found.text()  # (not started)
    d.artiq.setChecked(False)
    d.artiq_port.setValue(4000)
    d.artiq_from.setText("10.0.0.5; artiq-master")
    assert {k: d.values()[k] for k in ("artiq", "artiq_port", "artiq_from")} == {
        "artiq": False, "artiq_port": 4000, "artiq_from": ["10.0.0.5", "artiq-master"]}
    assert not hasattr(gui.ShareDialog(None, share), "artiq")  # (no server: no such settings)


def test_a_pc_without_a_wavemeter_shows_the_others(app):
    """--view: no wavemeter here, the others' wavemeters only (this PC's empty frame not shown)."""
    from test_share import PC, free_port

    from wmlock.share import Share
    from wmlock.wavemeter import NullSource

    a = PC("Lab A").start()
    e = Engine({}, NullSource)
    share = Share(e, {"label": "office", "port": free_port(), "lan": False, "peers": [f"127.0.0.1:{a.port}"]})
    win = gui.MainWindow(e, lambda: None, share)
    e.start()
    share.start()
    win.show()
    try:
        spin(app, 0.6)
        assert win.st_wlm.text() == "no wavemeter on this PC"
        assert wait_for(lambda: (app.processEvents(), bool(win.panel_list) and win.panel_list[0].rows)[1], 5)
        spin(app, 0.2)
        assert not win.local.isVisible() and win.panel_list[0].header.title.text() == "Lab A"
        assert win.panel_list[0].rows[1].wl.text().startswith("396.959")
        assert a.peer("office").src["viewer"]  # (no frame for it on the others: nothing in it)
    finally:
        win.close()
        spin(app, 0.1)
        a.close()


def test_a_laser_on_an_analog_output(app):
    """Linked to an analog output (Link laser › Analog output (DAC)…): its row says which, its menu has the output set
    by hand in place of mode-hop recovery, and the output dialog writes the board (not while locked)."""
    from wmlock.lock import LockParams
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004, dac=True)
    e = Engine({}, demo.make_source, discovery_targets=["127.0.0.1"])
    win = gui.MainWindow(e, lambda: None)
    e.start()
    win.show()
    try:
        assert wait_for(lambda: (app.processEvents(), 8 in win.rows and e.channels[8].freq > 0)[1], 5)
        row, ch = win.rows[8], e.channels[8]
        row.show_menu(row.rect().center())
        link = next(a for a in row._menu.actions() if a.text() == "Link laser").menu()
        assert "Analog output (DAC)…" in [a.text() for a in link.actions()]
        row._menu.close()
        d = gui.DacDialog(row, ch)
        assert d.values() == {"device": "dac:mcc:0", "ao": 0, "ff_ao": None, "ff_ratio": 0.0, "v_min": -1.2,
                              "v_max": 1.2, "excursion": 0.8}  # (Epimetheus's: its board, its limits)
        d.kind.setCurrentIndex(1)
        d.board.setValue(demo.board.number)
        d.ao.setValue(2)
        d.ff.setChecked(True)
        d.ff_ao.setValue(3)
        d.ff_ratio.setValue(5.0)
        d.v_max.setValue(1.0)
        v = d.values()
        assert v == {"device": demo.dac_key, "ao": 2, "ff_ao": 3, "ff_ratio": 5.0, "v_min": -1.2, "v_max": 1.0,
                     "excursion": 0.8}
        e.link(8, v["device"], v["ao"], v["ff_ao"], v["ff_ratio"])
        e.set_params(8, LockParams(**{**ch.lock.p.to_dict(), "v_max": v["v_max"]}))
        spin(app, 0.6)
        assert row.info.text().startswith("AO 2 (+ AO 3 fed forward) @ simulated DAC")
        row.show_menu(row.rect().center())
        items = [a.text() for a in row._menu.actions() if a.text()]
        assert "Analog output…" in items and "Check setup…" in items and "Mode-hop recovery…" not in items
        assert "Recenter now" not in items
        row._menu.close()
        out = gui.OutputDialog(row, e, 8)
        out.show()
        spin(app, 0.3)
        (_, ao, box, last), (_, ff_ao, ff_box, _) = out.boxes
        assert (ao, ff_ao) == (2, 3) and last.text() == "not set yet: its voltage is not known"
        box.setValue(0.3)  # (as typed, Enter)
        out.buttons[1].click()  # 0 V, as shown: set (else not known: no lock)
        assert wait_for(lambda: (app.processEvents(), demo.board.volts[2] == 0.3)[1], 2)
        assert wait_for(lambda: e.dacs.get(demo.dac_key, 3) == 0.0, 2)
        spin(app, 0.4)
        assert last.text() == "set to 0.3000 V" and box.maximum() == 1.0
        e.set_setpoint(8, 935.17976)
        row._clicked()  # Lock
        assert wait_for(lambda: (app.processEvents(), ch.state == "on")[1], 3), ch.message
        spin(app, 0.4)
        assert not box.isEnabled() and out.note.text().endswith("Locked: its lock moves it.")
        assert row.btn.text() == "Locked"
        out.close()
    finally:
        win.close()
        spin(app, 0.1)
        demo.close()


def test_a_pc_with_two_wavemeters_frames_each_and_swaps_their_settings(app, monkeypatch):
    """Two wavemeters on this PC (meters.py): each framed under its name, with its own rows and status; their settings
    swapped, one hidden and shown again; a laser driven from one asked about before it is linked on the other."""
    from wmlock.meters import Meter, Meters
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, wavemeter=SimWavemeter(noise=0.3, amplitude=0.0, walk=0.0), exposure=0.004,
                meters=2, serials=[1234, 5678])
    devices = [d.key for d in demo.dlcs]
    config = {"channels": {"1": {"name": "397"}}}
    first = Engine(config, demo.make_source, discovery_targets=["127.0.0.1"], extra_devices=devices)

    def make(section, serial, model):
        return Meter(Engine(section, lambda sink: demo.make_source(sink, serial, model),
                            discovery_targets=["127.0.0.1"], extra_devices=devices))

    meters = Meters(config, Meter(first), demo, make)
    asked = []
    monkeypatch.setattr(gui.MainWindow, "confirm", lambda self, title, text: asked.append((title, text)) or True)
    win = gui.MainWindow(first, lambda: None, None, meters)
    meters.start()
    win.show()
    try:
        assert wait_for(lambda: (app.processEvents(), len(win.locals) == 2 and 3 in win.locals[1].rows)[1], 5)
        a, b = win.locals
        assert a is win.local and a.framed and b.framed and a.header.isVisible()
        assert a.header.title.text() == "simulated WS7 1234" and b.header.title.text() == "simulated WS6 5678"
        assert a.header.title._color != b.header.title._color
        assert list(b.rows) == [1, 2, 3] and 7 in win.rows and b.rows[1].engine is meters.items[1].engine
        assert wait_for(lambda: (app.processEvents(), b.rows[1].wl.text().startswith("369.5245")
                                 and a.rows[1].wl.text().startswith("396.959"))[1], 3)
        spin(app, 0.6)
        assert win.st_wlm.text().startswith("● 2 wavemeters   10 ch")
        assert a.header.detail.text().startswith(f"this PC   {socket.gethostname()}")
        assert a.header.status.text().endswith("readings/s")
        QtTest = importlib.import_module(qt.BINDING + ".QtTest")
        QtTest.QTest.mouseClick(b.header, QtCore.Qt.MouseButton.LeftButton)  # fold it away
        assert b.folded and not b.rows[1].isVisible() and "simulated WS6 5678" in win.gui["folded"]
        b.fold(False)

        a.engine.link(1, demo.dlcs[0].key, 1)  # 397 on this one: linked on the other too, it is asked first
        asked.clear()
        assert win.may_link(b.engine, demo.dlcs[0].key, 1)
        assert asked[0][0] == "Link laser" and asked[0][1].startswith("Channel 1 of simulated WS7 1234 drives it")
        asked.clear()
        assert win.may_link(b.engine, demo.dlcs[0].key, 2) and not asked  # (another laser: nothing to ask)
        a.engine.unlink(1)

        win.swap(a, b)  # the settings under each go with the other's wavemeter
        assert wait_for(lambda: (app.processEvents(), a.rows.get(1) is not None and
                                 a.rows[1].wl.text().startswith("369.5245") and list(a.rows) == [1, 2, 3]
                                 and b.rows.get(1) is not None and b.rows[1].wl.text().startswith("396.959")
                                 and list(b.rows) == [1, 2, 3, 4, 5, 6, 7])[1], 5)
        assert asked and asked[-1][0] == "Swap settings" and a.rows[1].name.text() == "397"
        spin(app, 0.6)
        assert a.header.title.text() == "simulated WS6 5678"  # (named by what it reads, until named)

        win.show_menu(QtCore.QPoint(5, 5))  # (each one's simulated lasers can be given a fault)
        sim = next(x for x in win._menu.actions() if x.text() == "Simulate a fault").menu()
        texts = [x.text() for x in sim.actions()]
        assert "simulated WS6 5678" in texts and "simulated WS7 1234" in texts
        assert "Channel 1 397: mode hop up" in texts and "Channel 7: stops lasing" in texts  # (as swapped)
        win._menu.close()

        win.hide_meter(b)
        assert wait_for(lambda: (app.processEvents(), win.locals == [a] and not a.framed)[1], 3)
        assert meters.hidden() == [(1234, "simulated WS7 1234")]
        win.show_menu(QtCore.QPoint(5, 5))
        shown = next(x for x in win._menu.actions() if x.text() == "Show wavemeter").menu()
        assert [x.text() for x in shown.actions()] == ["simulated WS7 1234"]
        win._menu.close()
        win.show_meter(1234)
        assert wait_for(lambda: (app.processEvents(), len(win.locals) == 2 and win.locals[1].rows)[1], 5)
        assert win.locals[1].framed and win.locals[1].label == "simulated WS7 1234"
    finally:
        win.close()
        spin(app, 0.1)
        demo.close()


def test_a_second_copy_with_the_same_settings_says_so_and_closes(tmp_path, monkeypatch):
    from wmlock import __main__ as entry
    from wmlock.config import FileLock, lock_path

    said = []
    monkeypatch.setattr(gui, "refuse", lambda title, text: said.append(text) or 1)
    path = tmp_path / "settings.json"
    first = FileLock(lock_path(path))  # (the copy running)
    assert first.acquire()
    try:
        assert entry.main(["--view", "--config", str(path)]) == 1
    finally:
        first.release()
    assert said and said[0].startswith("The program is running already, with these settings")
    assert str(path) in said[0] and not path.exists()  # (nothing written: they are the other's)


def test_a_laser_map_from_its_dialog_to_its_window_and_picture(app, demo, tmp_path):
    from wmlock.lasermap import load

    e = Engine({"channels": {"3": {"name": "854"}}}, demo.make_source, discovery_targets=["127.0.0.1"],
               extra_devices=[d.key for d in demo.dlcs])
    win = gui.MainWindow(e, lambda: None)
    e.start()
    win.show()
    try:
        spin(app, 0.3)
        e.link(3, demo.dlcs[1].key, 1)
        for n in (1, 2, 4, 5, 6, 7):
            e.source.untick(n)
        assert wait_for(lambda: (app.processEvents(), e.channel_rate(3) > 5)[1], 5)
        d = gui.MapDialog(win, e, 3)
        d.show()
        assert wait_for(lambda: (app.processEvents(), "current-clip 200.0 mA" in d.now.text())[1], 5), d.now.text()
        assert "dl:pd:photodiode" in d.now.text() and d.i_lo.value() == 120.0  # (a first map: 30 mA below it)
        d.i_hi.setValue(0.0)
        assert not d.start.isEnabled() and "highest current" in d.problems.text()
        d.i_hi.setValue(152.0)
        d.i_lo.setValue(145.0)
        d.i_step.setValue(1.0)
        d.v_lo.setValue(68.0)
        d.v_hi.setValue(72.0)
        d.v_step.setValue(2.0)
        d.reads.setValue(1)
        d.settle.setValue(0.0)
        d.repeat.setChecked(False)
        d.patterns.setCurrentIndex(1)  # (every point)
        d.folder.setText(str(tmp_path))
        assert d.start.isEnabled() and d.problems.text() == "" and "points at most" in d.estimate.text()
        d.i_hi.setValue(250.0)  # (above its current-clip)
        assert not d.start.isEnabled() and "current-clip" in d.problems.text()
        d.i_hi.setValue(152.0)
        p = d.plan()
        assert (p.i_lo, p.i_hi, p.v_lo, p.v_hi, p.patterns, p.repeat) == (145.0, 152.0, 68.0, 72.0, "all", False)
        assert p.model and d.model.isChecked()  # (the laser's model at the end, unless unticked)
        d.model.setChecked(False)
        assert not d.plan().model
        d.model.setChecked(True)
        assert p.ff_end == "back"
        d.ff_end.setCurrentIndex(1)
        assert d.plan().ff_end == "fitted" and d.start.isEnabled()
        d.ff_end.setCurrentIndex(0)
        d._start()
        assert d.result() == QtWidgets.QDialog.DialogCode.Accepted
        w = win.map_window(e, 3)
        assert win.map_window(e, 3) is w  # (one a channel)
        row = win.rows[3]
        assert wait_for(lambda: (app.processEvents(), row.btn.text() == "Mapping")[1], 5)
        ch = e.channels[3]
        assert wait_for(lambda: (app.processEvents(), ch.state == "off")[1], 60), ch.map_note
        spin(app, 1.2)
        assert ch.map_note.startswith("map done") and w.head.text().startswith("map done")
        assert w.up_img._image is not None and w.up_img._image.width() == 8 and w.up_img._image.height() == 3
        png = ch.map_last.mapper.out.path.with_suffix(".png")
        assert png.exists() and png.stat().st_size > 1000
        assert w.which.count() == 1 and not w.stop.isEnabled()
        assert w.pattern.series and w.sweeps.series
        drawn = w.up_img._image.copy()
        w.show_what.setCurrentIndex(1)  # the power at the wavemeter
        w._layer(1)
        assert w.up_img.layer == "power" and w.legend.layer == "power" and w.legend.unit == "µW"
        lo, hi = w.legend.scale
        assert lo == 0.0 and hi > 0 and w.sweeps.ylabel == "µW" and w.up_img._image != drawn
        w.show_what.setCurrentIndex(2)  # its photodiode
        w._layer(2)
        assert w.up_img.layer == "pd" and w.legend.unit == "µA" and w.legend.scale[1] > w.legend.scale[0]
        w.grab()  # (painted with each, without trouble)
        with load(ch.map_last.mapper.out.path) as f:
            assert len(f.points) > 40
        m = ch.map_last.mapper
        assert m.model is None and not w.laser.isEnabled()  # (too few points for the laser's model)
        lasermodel = pytest.importorskip("wmlock.lasermodel")
        m.model = lasermodel.LaserModel(384.2, 150.0, 70.0, 0.0, lasermodel.Comb(6.0, p_i=-0.25, p_v=0.4),
                                        lasermodel.Comb(0.0, p_i=-2.0, p_v=0.32), right=0.9, points=500)
        m.model_note = "its model: right mode at 90% of the points"
        spin(app, 0.7)
        assert w.laser.isEnabled() and w.laser.text() == "The laser…"
        w._laser()
        shown = w.findChildren(gui.ModelDialog)
        assert shown and "its external cavity's modes are 6.0000 GHz apart" in \
            shown[0].findChild(QtWidgets.QTextBrowser).toPlainText()
        shown[0].close()
        acts = {a.text(): a for a in _menu_actions(row)}
        assert "Last laser map…" in acts and "Laser map…" in acts
        assert acts["Its laser's model from a map…"].isEnabled()
    finally:
        for w in list(getattr(win, "map_windows", {}).values()):
            w.close()
        win.close()
        e.stop()


def _menu_actions(row):
    """The actions of the row's menu (shown, then closed)."""
    row.show_menu(QtCore.QPoint(5, 5))
    m = row._menu
    acts = [a for a in m.actions()]
    m.close()
    return acts


def test_sls_dialog_round_trip_and_its_test(app):
    """SLS lock…: what locks the laser, its unit, a PLL's master and guard; Test asks the unit what it is."""
    from wmlock.sim import Demo
    from wmlock.sls import SlsParams

    demo = Demo(discovery_port=0, beat=True)
    try:
        e = Engine({"channels": {"9": {"name": "739 master"}, "10": {"name": "739 slave"}, "3": {}}},
                   demo.make_source)
        ch = e.channels[10]
        d = gui.SlsDialog(None, e, ch)
        assert d.params() == SlsParams() and not d.host.isEnabled() and not d.master.isEnabled()
        d.kind.setCurrentIndex(2)
        assert d.host.isEnabled() and d.master.isEnabled() and d.guard.isChecked()
        d.host.setText(" 127.0.0.1 ")
        d.port.setValue(demo.pll.sls.port)
        d.master.setCurrentIndex(d.masters.index(9))
        d.capture.setValue(0.8)
        p = d.params()
        assert p == SlsParams(kind="pll", host="127.0.0.1", port=demo.pll.sls.port, master=9, capture=0.8)
        d._test()
        assert wait_for(lambda: (app.processEvents(), "manual lock (MANUAL)" in d.found.text())[1], 5), d.found.text()
        ch.sls = SlsParams(kind="pll", host="P1029F", servo=2, master=9, beat=80.1, amp_usual=0.9)
        d = gui.SlsDialog(None, e, ch)
        assert d.servo.currentIndex() == 2 and d.beat.value() == 80.1 and d.params() == ch.sls
        d.servo.setCurrentIndex(3)
        assert d.params().amp_usual == 0.0  # (another PLL: learned afresh)
    finally:
        demo.close()


def test_rows_of_lasers_sls_units_lock(app, tmp_path):
    """The master's row and the slave's: Relock and Watching, the unit's state on the line under the digits, and
    Relocking while the guard has the slave."""
    from wmlock.sim import Demo, SimWavemeter

    demo = Demo(discovery_port=0, beat=True, wavemeter=SimWavemeter(noise=0.05), exposure=0.004)
    cfg = {"channels": {
        "9": {"name": "739 master", "sls": {"kind": "pdh", "host": "127.0.0.1", "port": demo.pdh.sls.port}},
        "10": {"name": "739 slave", "link": {"device": demo.sister.key, "laser": 1},
               "hop": {"enabled": True, "i_nom": 150.0},
               "sls": {"kind": "pll", "host": "127.0.0.1", "port": demo.pll.sls.port, "master": 9, "beat": 80.0}}}}
    e = Engine(cfg, demo.make_source, discovery_targets=["127.0.0.1"])
    win = gui.MainWindow(e, lambda: None)
    e.start()
    win.show()
    try:
        assert wait_for(lambda: (app.processEvents(), 10 in win.rows and e.channels[10].sls_view is not None)[1], 5)
        spin(app, 0.3)
        row = win.rows[10]
        assert row.btn.text() == "Relock" and "SLS unit locks it" in row.btn.toolTip()
        assert "PLL 127.0.0.1: manual lock (MANUAL)" in row.info.text() and "not guarded" in row.info.text()
        assert "SLS 127.0.0.1: Locked: 1" in win.rows[9].info.text()
        row.btn.click()
        win.rows[9].btn.click()
        assert wait_for(lambda: (app.processEvents(), row.btn.text() == "Watching")[1], 3)
        demo.pdh.drop(0.5)
        assert wait_for(lambda: (app.processEvents(), row.btn.text() == "Relocking")[1], 3)
        assert "its master's lock was lost" in row.info.text()
        assert wait_for(lambda: (app.processEvents(), row.btn.text() == "Watching")[1], 25), row.info.text()
        assert "relocked: its PLL holds it again" in row.info.text()
    finally:
        win.close()
        e.stop()
        demo.close()
