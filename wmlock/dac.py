"""Analog outputs (DACs), for lasers without a digital controller: a channel's lock moves an output's voltage, which
drives the laser's piezo driver (and, optionally, a second output its current, fed forward).

How Epimetheus (the lab's LabVIEW program) drove these lasers, decoded from its VIs:
  * one Measurement Computing board through ULx for LabVIEW: a task on 'Dev0/Ao0:15' (InstaCal's board 0, 16
    outputs, made with -4 … 4 V), all 16 outputs written every 4 ms;
  * a laser's piezo output (index_P) = an offset set by hand (last_offset_P) + its PID's output, that clipped to
    ±0.8 V and the sum to ±1.2 V; its current output (index_I, -1: none) = last_offset_I + the PID's output divided
    by Feed_forward (0: no feed-forward), clipped the same;
  * its PID (NI PID.vi) acted on the wavelength in nm, once for each new reading while locked: Kc = -1 (the
    frequency rises with the voltage), Ti = 0.0005 min; unlocked, the outputs stay where they were.

Here an output takes the place of a DLC pro's scan offset (engine.Controller talks to it through DacClient, as to a
DLC pro: 'laser{ao}:scan:offset' is output ao's voltage, 'laser{ao}:scan:unit' 'V'); the lock (lock.py) moves it
within its limits (for an output, by default ±1.2 V, and ±0.8 V from where it locked). A DAC cannot be read back:
what was last written to each output is kept (Memory, in the settings), so that the lock starts from there, and so
that whenever the board is opened (at the start, plugged in again; a power cut leaves it at 0 V) each output in use
is put back where it was. An output this program never wrote is not known: it must be set by hand first
(Engine.set_output), never jumped to by a lock.

Boards (keys 'dac:<kind>:<number>'):
  mcc   a Measurement Computing board, through mcculw (Windows; InstaCal's board number: ULx's 'Dev0' is 0), each
        output written with the range InstaCal has for it
  sim   a simulated board (sim.py), for the demo and the tests
"""
from __future__ import annotations

import logging
import threading
import time

from .dlcpro import DecopError

log = logging.getLogger(__name__)

SIM_BOARDS = {}  # number -> SimBoard, made by sim.py
PARAM = "scan:offset"  # (below laser{ao}: as for a DLC pro)


def key(kind: str, number: int) -> str:
    return f"dac:{kind}:{int(number)}"


def is_dac(device) -> bool:
    return isinstance(device, str) and device.startswith("dac:")


def parse(device: str):
    """'dac:mcc:0' -> ('mcc', 0)."""
    _, kind, number = device.split(":")
    return kind, int(number)


def title(device: str) -> str:
    """A board, as said on a row: 'MCC board 0'."""
    try:
        kind, number = parse(device)
    except ValueError:
        return device
    return {"mcc": f"MCC board {number}", "sim": f"simulated DAC {number}"}.get(kind, device)


class MccBoard:
    """A Measurement Computing analog output board, through MCC's mcculw (its Universal Library, as configured in
    InstaCal). Nothing is written but what write() is asked: mcculw's AoInfo probes ranges by writing 0 V to output
    0, a laser's, so the range of each output is read from InstaCal's configuration instead."""

    def __init__(self, number: int):
        self.number = number
        self.name = f"MCC board {number}"
        self.channels = 0
        self._ul = self._ranges = None
        self._range = {}  # output -> ULRange

    def open(self):
        try:
            from mcculw import enums, ul
        except (ImportError, OSError) as e:  # (OSError: no cbw64.dll, MCC's library: InstaCal not installed)
            raise OSError(f"mcculw cannot be used ({e}): pip install mcculw, and install InstaCal") from None
        self._ul, self._enums = ul, enums
        try:
            product = ul.get_board_name(self.number)
            self.channels = ul.get_config(enums.InfoType.BOARDINFO, self.number, 0, enums.BoardInfo.NUMDACHANS)
        except ul.ULError as e:
            raise OSError(f"MCC board {self.number}: {e} (is it in InstaCal?)") from None
        if not self.channels:
            raise OSError(f"MCC board {self.number} ({product}) has no analog outputs")
        self.name = f"{product} (MCC board {self.number})"

    def range_of(self, ao: int):
        """The range InstaCal has for output ao (ULRange); ±10 V if it does not say."""
        r = self._range.get(ao)
        if r is None:
            enums = self._enums
            try:
                r = enums.ULRange(self._ul.get_config(enums.InfoType.BOARDINFO, self.number, ao,
                                                      enums.BoardInfo.DACRANGE))
            except (self._ul.ULError, ValueError):
                r = enums.ULRange.BIP10VOLTS
            self._range[ao] = r
        return r

    def write(self, ao: int, volts: float):
        if not 0 <= ao < self.channels:
            raise DecopError(f"{self.name} has no output {ao} (0 … {self.channels - 1})")
        try:
            self._ul.v_out(self.number, ao, self.range_of(ao), float(volts))
        except self._ul.ULError as e:
            raise DecopError(f"{self.name}, output {ao}: {e}") from None

    def describe(self, ao: int) -> str:
        r = self.range_of(ao)
        return f"{self.name}: {self.channels} outputs; output {ao} in the range {r.name} (as InstaCal has it)"

    def close(self):
        pass  # (its outputs keep their voltages)


class SimBoard:
    """A simulated analog output board: its outputs move simulated lasers (sim.py). Writes outside its range, ±10 V,
    are clipped, as a real one's."""

    RANGE = 10.0

    def __init__(self, number: int = 0, channels: int = 16):
        self.number, self.channels = number, channels
        self.name = f"simulated DAC {number}"
        self.volts = [0.0] * channels
        self.writes = []  # (output, volts), as written (for the tests)
        self.fail = None  # an exception the next write raises (for the tests)

    def open(self):
        pass

    def write(self, ao: int, volts: float):
        if not 0 <= ao < self.channels:
            raise DecopError(f"{self.name} has no output {ao} (0 … {self.channels - 1})")
        if self.fail is not None:
            e, self.fail = self.fail, None
            raise e
        self.volts[ao] = max(-self.RANGE, min(self.RANGE, float(volts)))
        self.writes.append((ao, float(volts)))

    def describe(self, ao: int) -> str:
        return f"{self.name}: {self.channels} outputs, ±{self.RANGE:g} V"

    def close(self):
        pass


def open_board(kind: str, number: int):
    if kind == "mcc":
        return MccBoard(number)
    if kind == "sim":
        board = SIM_BOARDS.get(number)
        if board is None:
            raise OSError(f"no simulated DAC {number}")
        return board
    raise OSError(f"unknown kind of DAC: {kind!r}")


class Memory:
    """What was last written to each output (a DAC cannot be read back), kept in the settings ('dac': {'last':
    {'mcc:0/3': 0.512}})."""

    def __init__(self, cfg=None):
        last = (cfg or {}).get("last") or {}
        self._last = {k: float(v) for k, v in last.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}
        self._lock = threading.Lock()
        self.changed = 0.0  # monotonic: when it changed and was not yet saved (0: saved)

    @staticmethod
    def _name(device: str, ao: int) -> str:
        return f"{device[4:]}/{int(ao)}"

    def get(self, device: str, ao: int):
        with self._lock:
            return self._last.get(self._name(device, ao))

    def put(self, device: str, ao: int, volts: float):
        with self._lock:
            name = self._name(device, ao)
            if self._last.get(name) != volts:
                self._last[name] = round(float(volts), 6)
                if not self.changed:
                    self.changed = time.monotonic()

    def to_config(self) -> dict:
        with self._lock:
            self.changed = 0.0
            return {"last": dict(sorted(self._last.items()))}


class DacClient:
    """An analog output board as engine.Controller's thread sees a DLC pro: 'laser{ao}:scan:offset' is output ao's
    voltage (as last written: unknown, an error, until it is written), 'laser{ao}:scan:unit' 'V'. Opened on its first
    use; it cannot be had (mcculw missing, no such board): OSError, the controller is offline and tries again."""

    def __init__(self, device: str, memory: Memory):
        self.device, self.memory = device, memory
        self.kind, self.number = parse(device)
        self.board = None
        self.name = title(device)
        self.used = set()  # outputs in use: set back to what they were last set to whenever the board is opened

    @property
    def is_open(self) -> bool:
        return self.board is not None

    def use(self, ao: int):
        self.used.add(int(ao))

    def open(self):
        """The board, opened; each output in use set back to what it was last set to (a board reset by a power cut,
        or plugged in again, outputs 0 V)."""
        board = open_board(self.kind, self.number)
        board.open()
        for ao in sorted(self.used):
            v = self.memory.get(self.device, ao)
            if v is not None:
                try:
                    board.write(ao, v)
                except DecopError as e:
                    log.warning("%s: cannot set output %d back to %.4f V: %s", board.name, ao, v, e)
                else:
                    log.info("%s: output %d set back to %.4f V, where it was left", board.name, ao, v)
        self.board, self.name = board, board.name

    def close(self):
        if self.board is not None:
            self.board.close()
        self.board = None

    @staticmethod
    def _output(param: str):
        """'laser3:scan:offset' -> (3, 'scan:offset')."""
        head, _, name = param.partition(":")
        if not head.startswith("laser") or not head[5:].isdigit():
            raise DecopError(f"{param}: not on an analog output")
        return int(head[5:]), name

    def get(self, param: str) -> str:
        ao, name = self._output(param)
        if name == "scan:unit":
            return '"V"'
        if name != PARAM:
            raise DecopError(f"{param}: not on an analog output")
        if self.board is None:
            self.open()
        v = self.memory.get(self.device, ao)
        if v is None:
            raise DecopError(f"output {ao}: its voltage is not known (a DAC cannot be read back): set it first")
        return f"{v:.6f}"

    def get_float(self, param: str) -> float:
        return float(self.get(param))

    def get_str(self, param: str) -> str:
        r = self.get(param)
        return r[1:-1] if len(r) >= 2 and r[0] == r[-1] == '"' else r

    def set(self, param: str, value) -> int:
        ao, name = self._output(param)
        if name != PARAM:
            raise DecopError(f"{param}: not on an analog output")
        if self.board is None:
            self.open()
        self.board.write(ao, float(value))
        self.memory.put(self.device, ao, float(value))
        return 0

    def report(self, ao: int, ff_ao=None) -> list:
        """For the setup check: the board, its outputs, what they were last set to."""
        if self.board is None:
            self.open()
        out = [("head", "Analog output"), ("ok", self.board.describe(ao))]
        for k, what in ((ao, "piezo"), (ff_ao, "feed-forward")):
            if k is None:
                continue
            v = self.memory.get(self.device, k)
            out.append(("info", f"output {k} ({what}): last set to {v:.4f} V") if v is not None else
                       ("warn", f"output {k} ({what}): not set by this program yet, its voltage is not known: set it "
                                f"by hand (right-click › Analog output…) before locking"))
        out.append(("info", "nothing reads an output back: do not let another program (Epimetheus, Atlas) drive this "
                            "board at the same time; Epimetheus rewrites all 16 outputs every 4 ms"))
        return out
