"""python -m wmlock [--sim | --http URL | --dll [PATH] | --view] [--name NAME] [--config FILE]"""
from __future__ import annotations

import argparse
import faulthandler
import logging
import logging.handlers
import sys
import threading
import zlib
from pathlib import Path

from . import config as cfgfile
from . import rpc
from .engine import DAC_LOCK, Engine
from .lock import LockParams
from .meters import Elsewhere, Meter, Meters
from .share import SIM_PORT, Settings, Share
from .wavemeter import Claims, DllSource, HttpMeters, NullSource, WlmHub


def log_paths(path, serial=None):
    """Where recoveries and lock dropouts are logged (lines of JSON): next to the settings file, by its name; a
    wavemeter of this PC's other than the first, with its serial number in the names."""
    path = Path(path)
    tag = f"-wlm{serial}" if serial else ""
    rec = path.with_name(path.stem.lstrip(".") + tag + "-recoveries.jsonl")
    if path.name.startswith("."):
        rec = rec.with_name("." + rec.name)
    return rec, rec.with_name(rec.name.replace("-recoveries", "-dropouts"))


_crash_file = None  # (kept open: where faulthandler writes)


def keep_errors(path):
    """Errors kept, next to the settings file: warnings and every error nothing caught (in the window, in a thread),
    with its traceback, in ...-errors.log (1 MB, and the two before), the program going on where it can; the
    interpreter itself dying (a crash in Qt, say), where each thread was, in ...-crash.log. Their paths, or None."""
    global _crash_file
    path = Path(path)
    dot = "." if path.name.startswith(".") else ""
    errors = path.with_name(dot + path.stem.lstrip(".") + "-errors.log")
    crash = errors.with_name(errors.name.replace("-errors.log", "-crash.log"))
    try:
        handler = logging.handlers.RotatingFileHandler(errors, maxBytes=1_000_000, backupCount=2, encoding="utf-8")
        _crash_file = open(crash, "a", encoding="utf-8")
    except OSError:
        return None
    handler.setLevel(logging.WARNING)
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s %(threadName)s: %(message)s",
                                           "%Y-%m-%d %H:%M:%S"))
    logging.getLogger().addHandler(handler)
    faulthandler.enable(file=_crash_file, all_threads=True)
    log = logging.getLogger("wmlock")

    def uncaught(kind, value, tb):
        if issubclass(kind, KeyboardInterrupt):
            sys.__excepthook__(kind, value, tb)
            return
        log.critical("uncaught %s (the program goes on)", kind.__name__, exc_info=(kind, value, tb))

    def uncaught_in_thread(a):
        if a.exc_type is SystemExit:
            return
        log.critical("uncaught %s in thread %s", a.exc_type.__name__, a.thread.name if a.thread else "?",
                     exc_info=(a.exc_type, a.exc_value, a.exc_traceback))

    sys.excepthook = uncaught  # (PyQt5 would end the program at an error in the window's code: logged instead)
    threading.excepthook = uncaught_in_thread
    return errors, crash


def sim_serials(path) -> list:
    """The demo's simulated wavemeters' serial numbers: the same each time with the same settings file, others with
    another (two demos on a PC: no two alike)."""
    base = 100 + 2 * (zlib.crc32(str(Path(path).resolve()).encode()) % 450)
    return [base, base + 1]


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m wmlock",
                                description="Minimal wavemeter lock for TOPTICA DLC pro lasers.")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--sim", action="store_true",
                     help="demo: two simulated wavemeters, four fake DLC pros on localhost")
    src.add_argument("--http", metavar="URL",
                     help="read the wavemeter(s) through server/server.py, e.g. http://wlm-pc:8000")
    src.add_argument("--dll", metavar="PATH", nargs="?", const="",
                     help="read wlmData.dll directly (the default), optionally from PATH")
    src.add_argument("--view", action="store_true",
                     help="no wavemeter on this PC: show, and adjust, the other PCs' wavemeters only")
    p.add_argument("--name", metavar="NAME",
                   help="this wavemeter's name (the first's, on a PC with several), shown over its lasers on every PC "
                        "(remembered)")
    p.add_argument("--channels", metavar="LIST",
                   help="with --dll: show these channels instead of asking the switch, e.g. 1,2,5")
    p.add_argument("--config", metavar="FILE", help="settings file (default: ~/.simplewavemeterlock.json)")
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(levelname)s %(name)s: %(message)s",
                        datefmt="%Y-%m-%d %H:%M:%S")
    logging.getLogger("wmlock.dropouts").setLevel(logging.INFO)  # (its hourly summaries too)
    logging.getLogger("wmlock.share").setLevel(logging.INFO)  # (the other PCs found, their adjustments here)
    logging.getLogger("wmlock.meters").setLevel(logging.INFO)  # (this PC's wavemeters, which settings are whose)
    logging.getLogger("wmlock.rpc").setLevel(logging.INFO)  # (what ARTIQ's calls adjust)

    path = args.config or cfgfile.default_path(args.sim)
    held = cfgfile.FileLock(cfgfile.lock_path(path))  # one copy of the program at a time with these settings
    try:
        if not held.acquire():
            from .gui import refuse

            return refuse("Wavemeter Lock", f"The program is running already, with these settings ({path}): its "
                                            f"window shows every wavemeter of this PC. Another copy needs settings of "
                                            f"its own (--config FILE).")
    except OSError:  # (no lock file can be made there: as ever)
        pass
    kept = keep_errors(path)
    if kept:
        logging.getLogger("wmlock.errors").setLevel(logging.INFO)
        logging.getLogger("wmlock.errors").info("warnings and errors are kept in %s (a crash in %s)", *kept)
    config = cfgfile.load(path)
    meter = config.get("wavemeter") if isinstance(config.get("wavemeter"), dict) else {}
    demo, targets, extra, backend = None, None, (), None

    if args.sim:
        from .sim import Demo

        demo = Demo(dac=True, meters=2, serials=sim_serials(path), beat=True)
        for ch, name in demo.names.items():
            config.setdefault("channels", {}).setdefault(str(ch), {"name": name})
        config.setdefault("reference", {"channel": demo.REFERENCE})
        shg = config["channels"][str(demo.SHG)]  # cavity-locked SHG laser, with mode-hop recovery on
        shg["link"] = {"device": demo.dlcs[2].key, "laser": 1}  # its port is new every time
        shg.setdefault("setpoint_nm", demo.wavelength(demo.SHG))
        shg.setdefault("hop", {"enabled": True, "cavity": True, "shg": True, "i_nom": 149.58})  # measured
        old = config["channels"][str(demo.DAC)]  # a laser without a digital controller, on a simulated DAC
        _, _, _, ao, ff, _, _ = demo.DAC_LASER
        old["link"] = {"device": demo.dac_key, "ao": ao, "ff_ao": ff, "ff_ratio": 5.0}
        old.setdefault("setpoint_nm", demo.wavelength(demo.DAC))
        old.setdefault("lock", {**LockParams().to_dict(), **DAC_LOCK})
        last = config.setdefault("dac", {}).setdefault("last", {})  # (the simulated board starts at 0 V: where it is)
        last.setdefault(f"{demo.dac_key[4:]}/{ao}", 0.3)
        last.setdefault(f"{demo.dac_key[4:]}/{ff}", 0.0)
        master = config["channels"][str(demo.MASTER)]  # an SLS Laser Servo locks it to the cavity...
        master.setdefault("sls", {"kind": "pdh"}).update(host=demo.pdh.sls.host, port=demo.pdh.sls.port)  # (new ports)
        slave = config["channels"][str(demo.SLAVE)]  # ...and an SLS PLL beat-locks this one to it, its manual lock
        slave["link"] = {"device": demo.sister.key, "laser": 1}
        slave.setdefault("hop", {"enabled": True, "i_nom": 150.0})  # (recovered if it stops lasing)
        pll = slave.setdefault("sls", {"kind": "pll", "master": demo.MASTER})
        pll.update(host=demo.pll.sls.host, port=demo.pll.sls.port)
        second = config.setdefault("other_wavemeters", {}).setdefault(str(demo.serials[1]), {})
        for ch, name in demo.second_names.items():  # the other simulated wavemeter on this simulated PC
            second.setdefault("channels", {}).setdefault(str(ch), {"name": name})
        backend, targets, extra = demo, demo.targets, demo.extra_devices
        serial = meter.get("serial") if meter.get("serial") in demo.serials else None

        def make_source(sink):
            return demo.make_source(sink, serial)
    else:
        if args.http:
            config["source"], config["http_url"] = "http", args.http
        elif args.dll is not None:
            config["source"] = "dll"
            if args.dll:
                config["dll_path"] = args.dll
        elif args.view:
            config["source"] = "view"
        if config.get("source") == "view":
            make_source = NullSource
        elif config.get("source") == "http" and config.get("http_url"):
            backend = HttpMeters(config["http_url"], config.get("poll_ms", 20) / 1e3)

            def make_source(sink):
                return backend.make_source(sink, meter.get("serial"), meter.get("model", ""))
        else:
            config["source"] = "dll"
            dll = config.get("dll_path") or None
            chans = [int(c) for c in args.channels.split(",")] if args.channels else None
            interval = config.get("poll_ms", 2) / 1e3
            backend = WlmHub(dll, Claims(Path.home()))  # (another copy of the program: none of the same wavemeters)

            def make_source(sink):
                return DllSource(sink, dll, interval, chans, serial=meter.get("serial"), model=meter.get("model", ""),
                                 hub=backend, choose=sink.choose_wavemeter)

    rec, drops = log_paths(path)
    engine = Engine(config, make_source, discovery_targets=targets, extra_devices=extra, log_path=rec,
                    drop_path=drops)
    shared = config.setdefault("share", {})  # the other PCs (share.py)
    if args.name is not None:
        shared["label"] = args.name.strip()
    if args.sim:  # demos find each other on this PC only, never the real ones
        shared.setdefault("port", SIM_PORT)
        shared.setdefault("lan", False)
    siblings = set()  # (this PC's wavemeters' instances on the network: they ignore each other)
    share = Share(engine, shared, siblings=siblings)
    found = []  # (this PC's wavemeters, once known)
    elsewhere = Elsewhere(lambda: list(found[0].items) if found else [], share)  # (a beat lock's master: anywhere)
    engine.elsewhere = elsewhere

    def make(section, serial, model):
        """Another wavemeter of this PC's: its engine (its settings: section) and its instance on the network."""
        rec, drops = log_paths(path, serial)
        e = Engine(section, lambda sink: backend.make_source(sink, serial, model), discovery_targets=targets,
                   extra_devices=[*extra, *engine.known], log_path=rec, drop_path=drops)
        e.elsewhere = elsewhere
        return Meter(e, Share(e, Settings(section.setdefault("share", {}), shared), siblings=siblings,
                              subscribe=False))

    meters = Meters(config, Meter(engine, share), backend, make)
    found.append(meters)
    api = rpc.Server(rpc.Api(lambda: list(meters.items), share), shared, rpc.SIM_PORT if args.sim else rpc.PORT)
    try:
        from .gui import run

        return run(engine, lambda: cfgfile.save(path, meters.to_config()), share, meters, api)
    finally:
        if demo:
            demo.close()
        if isinstance(backend, HttpMeters):
            backend.stop()
        held.release()


if __name__ == "__main__":
    raise SystemExit(main())
