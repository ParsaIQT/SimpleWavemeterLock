"""python -m wmlock [--sim | --http URL | --dll [PATH] | --view] [--name NAME] [--config FILE]"""
from __future__ import annotations

import argparse
import logging
from pathlib import Path

from . import config as cfgfile
from .engine import Engine
from .share import SIM_PORT, Share
from .wavemeter import DllSource, HttpSource, NullSource


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m wmlock",
                                description="Minimal wavemeter lock for TOPTICA DLC pro lasers.")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--sim", action="store_true",
                     help="demo: simulated wavemeter and two fake DLC pros on localhost")
    src.add_argument("--http", metavar="URL",
                     help="read the wavemeter through server/server.py, e.g. http://wlm-pc:8000")
    src.add_argument("--dll", metavar="PATH", nargs="?", const="",
                     help="read wlmData.dll directly (the default), optionally from PATH")
    src.add_argument("--view", action="store_true",
                     help="no wavemeter on this PC: show, and adjust, the other PCs' wavemeters only")
    p.add_argument("--name", metavar="NAME",
                   help="this wavemeter's name, shown over its lasers on every PC (remembered)")
    p.add_argument("--channels", metavar="LIST",
                   help="with --dll: show these channels instead of asking the switch, e.g. 1,2,5")
    p.add_argument("--config", metavar="FILE", help="settings file (default: ~/.simplewavemeterlock.json)")
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(levelname)s %(name)s: %(message)s",
                        datefmt="%Y-%m-%d %H:%M:%S")
    logging.getLogger("wmlock.dropouts").setLevel(logging.INFO)  # (its hourly summaries too)
    logging.getLogger("wmlock.share").setLevel(logging.INFO)  # (the other PCs found, their adjustments here)

    path = args.config or cfgfile.default_path(args.sim)
    config = cfgfile.load(path)
    demo, targets, extra = None, None, ()

    if args.sim:
        from .sim import Demo

        demo = Demo()
        for ch, name in demo.names.items():
            config.setdefault("channels", {}).setdefault(str(ch), {"name": name})
        config.setdefault("reference", {"channel": demo.REFERENCE})
        shg = config["channels"][str(demo.SHG)]  # cavity-locked SHG laser, with mode-hop recovery on
        shg["link"] = {"device": demo.dlcs[2].key, "laser": 1}  # its port is new every time
        shg.setdefault("setpoint_nm", demo.wavelength(demo.SHG))
        shg.setdefault("hop", {"enabled": True, "cavity": True, "shg": True, "i_nom": 149.58})  # measured
        make_source, targets, extra = demo.make_source, demo.targets, demo.extra_devices
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
            url = config["http_url"]
            interval = config.get("poll_ms", 20) / 1e3

            def make_source(sink):
                return HttpSource(sink, url, interval)
        else:
            config["source"] = "dll"
            dll = config.get("dll_path") or None
            chans = [int(c) for c in args.channels.split(",")] if args.channels else None
            interval = config.get("poll_ms", 2) / 1e3

            def make_source(sink):
                return DllSource(sink, dll, interval, chans)

    log_path = Path(path).with_name(Path(path).stem.lstrip(".") + "-recoveries.jsonl")
    if Path(path).name.startswith("."):
        log_path = log_path.with_name("." + log_path.name)
    drop_path = log_path.with_name(log_path.name.replace("-recoveries", "-dropouts"))
    engine = Engine(config, make_source, discovery_targets=targets, extra_devices=extra, log_path=log_path,
                    drop_path=drop_path)
    shared = config.setdefault("share", {})  # the other PCs (share.py)
    if args.name is not None:
        shared["label"] = args.name.strip()
    if args.sim:  # demos find each other on this PC only, never the real ones
        shared.setdefault("port", SIM_PORT)
        shared.setdefault("lan", False)
    share = Share(engine, shared)
    try:
        from .gui import run

        return run(engine, lambda: cfgfile.save(path, engine.to_config()), share)
    finally:
        if demo:
            demo.close()


if __name__ == "__main__":
    raise SystemExit(main())
