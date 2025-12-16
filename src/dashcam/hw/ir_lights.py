
# 1 = full power, 0.75 = low power PWM, 0 = off.

import os, sys, time
from typing import Dict
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")
from gpiozero import PWMLED

# BCM pin map
PINS: Dict[str, int] = {"cam0": 1, "cam1": 14, "usb0": 23}

VERBOSE     = os.getenv("IR_VERBOSE", "1").lower() in ("1","true","yes","on")
FORCE_SIM   = 1

_values = {n: 0.0 for n in PINS}
SIM = FORCE_SIM
_devs: Dict[str, object] = {}

# --- minimal add-ons for boot priming ---

def reinit():
    """Close and reopen all PWM devices """
    global _devs, SIM
    # Turn everything off and close current devices
    try:
        cleanup()
    except Exception:
        pass
    # Re-open fresh
    try:
        if FORCE_SIM:
            raise ImportError("forced sim")
        from gpiozero import PWMLED
        _devs = {n: PWMLED(pin=p, active_high=True, initial_value=0.0) for n, p in PINS.items()}
        SIM = False
        _log("reinit: backend=gpiozero PWM")
    except Exception as e:
        SIM = True
        _devs = {}
        print(f"[ir] reinit fallback PRINT_ONLY ({e})", file=sys.stderr)

def prime(start_flash_ms: int = 60):
    """
    Reclaim pins after boot, drive them to a known-low state,
    then flash each channel briefly to guarantee direction/ownership.
    """
    reinit()
    # known-low settle
    try:
        all_off()
    except Exception:
        pass
    time.sleep(0.05)
    # visible flash: cam0 -> cam1 -> usb0
    for n in PINS:
        try:
            set(n, 1.0); time.sleep(max(0, start_flash_ms)/1000.0)
            set(n, 0.0); time.sleep(0.02)
        except Exception as e:
            _log(f"prime: {n} flash error: {e}")

def _log(msg: str):
    if VERBOSE:
        print(f"[ir] {msg}")

# Try gpiozero; fall back to print-only if unavailable or forced
try:
    if FORCE_SIM:
        raise ImportError("forced sim")
    from gpiozero import PWMLED
    _devs = {n: PWMLED(pin=p, active_high=True, initial_value=0.0) for n, p in PINS.items()}
    SIM = False
    _log("backend=gpiozero PWM")
except Exception as e:
    SIM = True
    _devs = {}
    print(f"[ir] backend=PRINT_ONLY (gpiozero unavailable: {e})", file=sys.stderr)

def set(name: str, val: float):
    """Set power: 0=off, 1=full, fractional=PWM (e.g., 0.75)."""
    if name not in PINS:
        raise KeyError(f"unknown IR '{name}'")
    v = max(0.0, min(1.0, float(val)))
    _values[name] = v
    if SIM:
        _log(f"{name}={v:.2f} (sim)")
        return
    _devs[name].value = v  
    _log(f"{name}={v:.2f}")

def on(name: str):                 set(name, 1.0)
def off(name: str):                set(name, 0.0)
def on_lp(name, v=None): set(name, 0.75 if v is None else v)  

def all_set(v: float):
    for n in PINS: set(n, v)

def all_on():                      all_set(1.0)
def all_off():                     all_set(0.0)
def all_on_lp(v=None): all_set(0.75 if v is None else float(v))

def flash(name: str, ms: int = 40):
    """Full power briefly, then restore previous value."""
    prev = _values[name]
    set(name, 1.0)
    time.sleep(max(0, ms) / 1000.0)
    set(name, prev)

def state() -> Dict[str, float]:
    return dict(_values)

def cleanup():
    """Turn everything off and close devices."""
    all_off()
    if not SIM:
        for d in _devs.values():
            try: d.close()
            except: pass
    _log("cleanup")
