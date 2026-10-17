import re
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
                    [55.0, 149.7, 1.0, None]]}
    d = gui.RecoveryDialog(None, rec)
    d.show()
    spin(app, 0.1)
    tabs = d.findChild(QtWidgets.QTabWidget)
    assert [tabs.tabText(k) for k in range(tabs.count())] == ["Steps", "Walk", "Map", "SHG sweeps", "Cavity peaks",
                                                              "Error signal"]
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
        assert wait_for(lambda: (app.processEvents(), ch.hop_prompt.startswith(("Walk the piezo onto the target",
                                                                               "Lock the SHG cavity")))[1], 5)
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
