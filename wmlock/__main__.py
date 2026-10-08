"""python -m wmlock [--sim | --http URL | --dll [PATH]] [--config FILE]"""
from __future__ import annotations

import argparse
import logging

from . import config as cfgfile
from .engine import Engine
from .wavemeter import DllSource, HttpSource


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
    p.add_argument("--channels", metavar="LIST",
                   help="with --dll: show these channels instead of asking the switch, e.g. 1,2,5")
    p.add_argument("--config", metavar="FILE", help="settings file (default: ~/.simplewavemeterlock.json)")
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    path = args.config or cfgfile.default_path(args.sim)
    config = cfgfile.load(path)
    demo, targets, extra = None, None, ()

    if args.sim:
        from .sim import Demo

        demo = Demo()
        for ch, name in demo.names.items():
            config.setdefault("channels", {}).setdefault(str(ch), {"name": name})
        make_source, targets, extra = demo.make_source, demo.targets, demo.extra_devices
    else:
        if args.http:
            config["source"], config["http_url"] = "http", args.http
        elif args.dll is not None:
            config["source"] = "dll"
            if args.dll:
                config["dll_path"] = args.dll
        if config.get("source") == "http" and config.get("http_url"):
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

    engine = Engine(config, make_source, discovery_targets=targets, extra_devices=extra)
    try:
        from .gui import run

        return run(engine, lambda: cfgfile.save(path, engine.to_config()))
    finally:
        if demo:
            demo.close()


if __name__ == "__main__":
    raise SystemExit(main())
