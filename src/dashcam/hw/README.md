# Hardware helpers (`dashcam.hw`)

GPIO and platform telemetry utilities.

---

## `ir_lights.py`

IR illumination control (PWM) using `gpiozero.PWMLED`.

### Channels and pins
Default BCM pin map:
- `cam0` -> 1
- `cam1` -> 14
- `usb0` -> 23

### Power model
- `1.0`  = full power
- `0.75` = low power (typical)
- `0.0`  = off

### Simulation mode
- `FORCE_SIM = 1` forces simulation (no real GPIO activity).
- Simulation keeps internal state only (`state()` reflects requested values).

### Configuration
- `IR_VERBOSE` : enable logging (`1/0`, `true/false`)

### Public API
Lifecycle:
- `reinit()`   : (re)initialize devices and reset state
- `cleanup()`  : close devices and reset state
- `prime()`    : optional boot priming behavior

Per-channel control:
- `set(name, val)`
- `on(name)`
- `off(name)`
- `on_lp(name, v)` : low-power helper

Global control:
- `all_set(v)`
- `all_on()`
- `all_off()`
- `all_on_lp(v)`

Effects:
- `flash(ms)`  : brief full-on flash for all channels
- `state()`    : return current requested values

---

## `health.py`

Raspberry Pi health / telemetry logger (CSV).

### What it logs
- timestamp (ISO)
- uptime (seconds)
- CPU utilization (approx)
- load averages (1/5/15)
- temperature (C) via `vcgencmd`
- clocks (MHz): `arm`, `core`, `v3d`
- throttling flags (hex) via `vcgencmd get_throttled`
- memory used/total (MB)

### Usage model
- CLI program that writes/append CSV at a fixed interval.
- Designed to run alongside recording/streaming for performance characterization.

### Runtime requirements
- `vcgencmd` available on target Pi image
- read access to `/proc/uptime`

