"""WLM monitoring server (FastAPI), for reading the wavemeter from another PC.

Run on the wavemeter PC, next to HighFinesse's wlmData.py and wlmConst.py
(they ship with the WLM software, under Examples/Python):

    pip install fastapi uvicorn
    uvicorn server:app --host 0.0.0.0 --port 8000

then on any PC:  python -m wmlock --http http://<wavemeter-pc>:8000

Changes from the original: wlmData has no GetUseChannel, so the old code always
fell back to "enabled" and reported every channel. Whether the switch measures
a channel is the 'Use' flag of GetSwitcherSignalStates: a channel with it
unticked is reported "Disabled" with "use": false, and

    POST /api/v1/channel/<n>/use   {"use": true}

ticks it again (false unticks it), as SimpleWavemeterLock does to relock a
laser whose channel someone switched off, before switching it off again.

A PC with several wavemeters (each with its WLM software running): each answer
says whose it is ("wavemeter": {"model": "WS7", "serial": 1234}), ?wlm=1234 asks
for that one (404 if it is not running), the first one without it; and

    GET /api/v1/wavemeters   {"wavemeters": [{"index": 0, "model": "WS7", "serial": 1234}, ...]}

lists them. wlmData addresses one wavemeter at a time (PresetWLMIndex), for the
whole server: each request sets it, one at a time.
"""
import ctypes
import sys
import threading
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from pydantic import BaseModel

# HighFinesse imports
import wlmData
import wlmConst

DLL_PATH = "wlmData.dll"
MAX_CHANNELS = 17  # the real count comes from GetChannelsCount
DLL_LOCK = threading.Lock()  # one request at a time talks to wlmData: it addresses one wavemeter at a time
PRESET = [False]  # PresetWLMIndex has been called (there were several): set it for every request since


def load_wlm_dll():
    """Loads the DLL once when the server starts."""
    try:
        wlmData.LoadDLL(DLL_PATH)
        print("wlmData.dll loaded successfully.")
    except Exception as e:
        print(f"CRITICAL ERROR: Couldn't find or load DLL on path {DLL_PATH}. Details: {e}")
        sys.exit(1)


@asynccontextmanager
async def lifespan(app):
    load_wlm_dll()
    yield


# Initialize FastAPI
app = FastAPI(title="WLM Monitoring Server", description="Fast, lightweight multichannel wavemeter API",
              lifespan=lifespan)


def get_status_message(freq):
    """Helper function to decode error states."""
    if freq == wlmConst.ErrWlmMissing: return "WLM inactive"
    if freq == wlmConst.ErrNoSignal: return "No Signal"
    if freq == wlmConst.ErrBadSignal: return "Bad Signal"
    if freq == wlmConst.ErrLowSignal: return "Low Signal"
    if freq == wlmConst.ErrBigSignal: return "High Signal"
    if freq == wlmConst.ErrOutOfRange: return "Out of Range"
    if freq <= 0: return f"Error Code: {freq}"
    return "Valid"


def _call(name, *args, default=0):
    f = getattr(wlmData.dll, name, None)
    return f(*args) if f is not None else default


def _preset(index):
    if getattr(wlmData.dll, "PresetWLMIndex", None) is not None:
        wlmData.dll.PresetWLMIndex(index)
        PRESET[0] = True


def _meter():
    """The model and serial number of the wavemeter addressed."""
    t, serial = _call("GetWLMVersion", 0), _call("GetWLMVersion", 1)
    return (f"WS{t}" if 4 <= t <= 30 else ""), (serial if serial > 0 else 0)


def wavemeters():
    """The wavemeters running on this PC: [(index, model, serial)], found as HighFinesse's own example finds them."""
    n = wlmData.dll.GetWLMCount(0)
    if n <= 0:
        return []
    if n == 1 or getattr(wlmData.dll, "PresetWLMIndex", None) is None:
        if PRESET[0]:
            _preset(0)
        return [(0, *_meter())]
    out, seen = [], set()
    for i in range(n + 4):
        _preset(i)
        model, serial = _meter()
        if serial and serial not in seen:
            seen.add(serial)
            out.append((i, model, serial))
            if len(out) == n:
                break
    return out


def select(wlm):
    """Address the wavemeter asked for (None: the first): its (index, model, serial), or the answer to give if it
    cannot be."""
    found = wavemeters()
    if not found:
        return JSONResponse(status_code=503, content={"detail": "wlmServer is not running on the host machine."})
    if wlm is None:
        f = found[0]
    else:
        f = next((f for f in found if f[2] == wlm), None)
        if f is None:
            here = ", ".join(f"{m} {s}".strip() for _, m, s in found)
            return JSONResponse(status_code=404, content={"detail": f"wavemeter {wlm} is not running there (it has "
                                                                    f"{here})"})
    if PRESET[0]:
        _preset(f[0])
    return f


def channel_count():
    n = wlmData.dll.GetChannelsCount(0)
    return min(n, MAX_CHANNELS) if n > 0 else 1


def channel_use(ch, n):
    """True if the switch measures this channel ('Use' ticked in the WLM software); False if it does not
    because 'Use' is unticked (the switch cycling: ticking it would have it measured); None if it does not
    for another reason (the switch not cycling)."""
    if n <= 1:
        return True if ch == 1 else None
    if wlmData.dll.GetSwitcherMode(0) == 0:  # switch not cycling: only the selected channel
        return True if ch == max(wlmData.dll.GetSwitcherChannel(0), 1) else None
    use, show = ctypes.c_long(), ctypes.c_long()
    if wlmData.dll.GetSwitcherSignalStates(ch, ctypes.byref(use), ctypes.byref(show)) < 0:
        return True  # not available: report the channel
    return bool(use.value)


def channel_in_use(ch, n):
    """True if the switch measures this channel ('Use' ticked in the WLM software)."""
    return channel_use(ch, n) is True


@app.get("/api/v1/wavemeters", tags=["Metrics"])
def get_wavemeters():
    """The wavemeters running on this PC (each with its WLM software)."""
    with DLL_LOCK:
        return {"wavemeters": [{"index": i, "model": m, "serial": s} for i, m, s in wavemeters()]}


@app.get("/api/v1/wavelength", tags=["Metrics"])
def get_wavelength_data(wlm: Optional[int] = None):
    """Fetches real-time data for all channels statelessly (of wavemeter wlm, by its serial number: on a PC with
    several)."""
    with DLL_LOCK:
        return _wavelength_data(wlm)


def _wavelength_data(wlm):
    # 1. Check if WLM Server is actually running (the one asked for)
    if wlmData.dll.GetWLMCount(0) == 0:
        return JSONResponse(
            status_code=503,
            content={"error": "wlmServer is not running on the host machine."}
        )
    meter = select(wlm)
    if isinstance(meter, JSONResponse):
        return meter

    # 2. Read Global Environment Metrics
    temp_raw = wlmData.dll.GetTemperature(0.0)
    temperature = None if temp_raw <= wlmConst.ErrTemperature else round(temp_raw, 2)

    pressure_raw = wlmData.dll.GetPressure(0.0)
    pressure = None if pressure_raw <= wlmConst.ErrTemperature else round(pressure_raw, 2)

    # 3. Read All Channels
    channels_data = {}
    n = channel_count()
    for ch in range(1, n + 1):
        use = channel_use(ch, n)
        if use is not True:
            channels_data[f"channel_{ch}"] = {
                "status": "Disabled",
                "is_valid": False,
                "frequency_thz": None,
                "wavelength_nm": None
            }
            if use is False:  # 'Use' unticked: POST /api/v1/channel/<n>/use can tick it
                channels_data[f"channel_{ch}"]["use"] = False
            continue  # Skip trying to read frequencies for this channel

        freq = wlmData.dll.GetFrequencyNum(ch, 0.0)
        wl = wlmData.dll.GetWavelengthNum(ch, 0.0)

        is_valid = (freq > 0)

        channels_data[f"channel_{ch}"] = {
            "status": get_status_message(freq),
            "is_valid": is_valid,
            "frequency_thz": freq if is_valid else None,
            "wavelength_nm": wl if is_valid else None
        }

    # Return structured JSON payload
    return {
        "server_status": "Online",
        "wavemeter": {"model": meter[1], "serial": meter[2]},
        "environment": {
            "temperature_c": temperature,
            "pressure_mbar": pressure
        },
        "channels": channels_data
    }


class Use(BaseModel):
    use: bool


@app.post("/api/v1/channel/{ch}/use", tags=["Control"])
def set_channel_use(ch: int, body: Use, wlm: Optional[int] = None):
    """Tick (use: true) or untick a channel's 'Use' in the WLM software (its 'Show' left as it is); of wavemeter
    wlm, by its serial number, on a PC with several."""
    with DLL_LOCK:
        return _set_channel_use(ch, body, wlm)


def _set_channel_use(ch, body, wlm):
    if wlmData.dll.GetWLMCount(0) == 0:
        return JSONResponse(status_code=503, content={"detail": "wlmServer is not running on the host machine."})
    meter = select(wlm)
    if isinstance(meter, JSONResponse):
        return meter
    n = channel_count()
    if not 1 <= ch <= n:
        return JSONResponse(status_code=404, content={"detail": f"no channel {ch} (the switch has {n})"})
    if n <= 1 or wlmData.dll.GetSwitcherMode(0) == 0:
        return JSONResponse(status_code=409, content={"detail": "the switch is not cycling through its channels"})
    use, show = ctypes.c_long(), ctypes.c_long()
    if wlmData.dll.GetSwitcherSignalStates(ch, ctypes.byref(use), ctypes.byref(show)) < 0:
        show.value = 1
    r = wlmData.dll.SetSwitcherSignalStates(ch, int(body.use), show.value)
    if r < 0:
        return JSONResponse(status_code=500, content={"detail": f"SetSwitcherSignalStates: error {r}"})
    return {"channel": ch, "use": body.use}
