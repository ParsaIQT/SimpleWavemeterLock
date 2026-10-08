"""WLM monitoring server (FastAPI), for reading the wavemeter from another PC.

Run on the wavemeter PC, next to HighFinesse's wlmData.py and wlmConst.py
(they ship with the WLM software, under Examples/Python):

    pip install fastapi uvicorn
    uvicorn server:app --host 0.0.0.0 --port 8000

then on any PC:  python -m wmlock --http http://<wavemeter-pc>:8000

Change from the original: wlmData has no GetUseChannel, so the old code always
fell back to "enabled" and reported every channel. Whether the switch measures
a channel is the 'Use' flag of GetSwitcherSignalStates.
"""
import ctypes
import sys
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.responses import JSONResponse

# HighFinesse imports
import wlmData
import wlmConst

DLL_PATH = "wlmData.dll"
MAX_CHANNELS = 17  # the real count comes from GetChannelsCount


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


def channel_count():
    n = wlmData.dll.GetChannelsCount(0)
    return min(n, MAX_CHANNELS) if n > 0 else 1


def channel_in_use(ch, n):
    """True if the switch measures this channel ('Use' ticked in the WLM software)."""
    if n <= 1:
        return ch == 1
    if wlmData.dll.GetSwitcherMode(0) == 0:  # switch not cycling: only the selected channel
        return ch == max(wlmData.dll.GetSwitcherChannel(0), 1)
    use, show = ctypes.c_long(), ctypes.c_long()
    if wlmData.dll.GetSwitcherSignalStates(ch, ctypes.byref(use), ctypes.byref(show)) < 0:
        return True  # not available: report the channel
    return bool(use.value)


@app.get("/api/v1/wavelength", tags=["Metrics"])
def get_wavelength_data():
    """Fetches real-time data for all channels statelessly."""

    # 1. Check if WLM Server is actually running
    if wlmData.dll.GetWLMCount(0) == 0:
        return JSONResponse(
            status_code=503,
            content={"error": "wlmServer is not running on the host machine."}
        )

    # 2. Read Global Environment Metrics
    temp_raw = wlmData.dll.GetTemperature(0.0)
    temperature = None if temp_raw <= wlmConst.ErrTemperature else round(temp_raw, 2)

    pressure_raw = wlmData.dll.GetPressure(0.0)
    pressure = None if pressure_raw <= wlmConst.ErrTemperature else round(pressure_raw, 2)

    # 3. Read All Channels
    channels_data = {}
    n = channel_count()
    for ch in range(1, n + 1):
        if not channel_in_use(ch, n):
            channels_data[f"channel_{ch}"] = {
                "status": "Disabled",
                "is_valid": False,
                "frequency_thz": None,
                "wavelength_nm": None
            }
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
        "environment": {
            "temperature_c": temperature,
            "pressure_mbar": pressure
        },
        "channels": channels_data
    }
