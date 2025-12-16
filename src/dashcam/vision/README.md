# Vision (`dashcam.vision`)

Software motion detection and ambient “dark meter” used for day/night behavior.

This module is designed to be robust under mode transitions where cameras are repeatedly acquired/released (motion verify, recording, streaming).

---

## `motion.py`

CSI + UVC motion detector + camera-dark-meter.

### Key concepts

Capture reservations (conflict avoidance):
- **CAPTURE reservation** blocks BOTH USB + CSI capture while streaming is being brought up or torn down.
- **CSI reservation** blocks CSI (Picamera2) capture independently (useful when opening/closing MIPI devices).

Serialization:
- A single CSI open/close/start/stop lock is used to serialize Picamera2 operations.

Profiles:
- Motion thresholds are “light-aware”:
  - DAY profile (defaults tuned for fewer false positives)
  - NIGHT profile (defaults more sensitive)
- Profiles can be overridden manually.

Warm-up / reseed:
- Baselines can be reseeded after mode transitions to reduce false triggers.

Capture hook:
- A user-provided hook can be called before each capture (e.g., IR priming).

---

## Public API

Resource reservations:
- `capture_reserve()`
- `capture_release()`
- `capture_is_reserved()`
- `csi_reserve()`
- `csi_release()`
- `csi_is_reserved()`

Configuration hooks:
- `set_capture_hook(fn)`
- `set_force_profile(mode="")` where mode is `"" | "day" | "night"`

Dark meter:
- `dark_meter_start()`
- `dark_meter_stop(block=True)`
- `dark_meter_running()`
- `dark_state()` -> dict with keys like `ema`, `is_dark`, `ts`
- `ambient_is_dark_camera()` -> boolean convenience
- `sample_dark_once()` -> one-shot sample

Motion detector:
- `start()`
- `stop()`
- `wait_stopped(timeout_s)`
- `motion_active()` -> boolean
- `reset_last_motion(ts)`
- `reseed_baselines(n_rounds)`

Discovery / helpers:
- `discover()`
- `camera_names()`
- `process_capture(name, Y)` -> run detector on a provided luma frame
- `rotation_loop()` -> internal worker loop

---

## Configuration (environment variables)

Core timing:
- `INTERVAL_S` : detector capture interval
- `QUIET_SECONDS` : quiet window after motion triggers
- `MOTION_WARMUP_ROUNDS` : baseline warm-up cycles after start/reseed

Enable sources:
- `MOTION_ENABLE_MIPI` : enable CSI sources
- `MOTION_ENABLE_USB`  : enable UVC sources

MIPI open robustness:
- `MOTION_MIPI_OPEN_RETRIES`
- `MOTION_MIPI_OPEN_BACKOFF_S`
- `MOTION_MIPI_REUSE_DELAY_S`

Per-camera overrides (optional):
- `MOTION_PCT_MIPI0`, `MOTION_AREA_MIPI0`
- `MOTION_PCT_MIPI1`, `MOTION_AREA_MIPI1`
- `MOTION_PCT_VIDEO0`, `MOTION_AREA_VIDEO0`

Day/night profiles:
- `MD_DAY_PCT`, `MD_DAY_AREA`, `MD_DAY_FLOOR`
- `MD_NIGHT_PCT`, `MD_NIGHT_AREA`, `MD_NIGHT_FLOOR`

Dark meter:
- `DARK_METER_PERIOD`
- `DARK_EMA_ALPHA`
- `DARK_ON_LUMA`
- `DARK_OFF_LUMA`
- `DARK_STALE_S`
- `DARK_METER_USE_MIPI` : prefer CSI for dark samples (if enabled)
- `DARK_VERBOSE`
- `DARKMETER_VERBOSE`

---

## Outputs
- Motion state is exposed via `motion_active()`.
- Dark meter state is exposed via `dark_state()`.
- Proof/overlay outputs are produced by scripts in `scripts/` (not by this core module).

---

## Runtime expectations
- The control layer should stop the detector and dark meter before starting streaming or full processing.
- Use CAPTURE/CSI reservation functions to prevent race conditions during camera mode transitions.

