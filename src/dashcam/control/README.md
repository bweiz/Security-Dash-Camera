# Control (`dashcam.control`)

Top-level orchestration for the dashcam system:
- coordinates motion detection vs Full Processing Mode (recording)
- coordinates IR behavior (prime/flash/low-power/full) based on ambient darkness
- exposes a simple local socket protocol to toggle per-camera streaming
- performs safe device handoff (avoids CSI/USB conflicts during stream bring-up/tear-down)
- handles user “record” / “motion” intent via GPIO bridge switches

---

## `app.py`

Wrapper / state machine + stream control server.

### Responsibilities
- Start/stop motion detector and dark meter at appropriate times
- Start/stop Full Processing Mode (recording)
- Handle streaming requests per camera (`cam0`, `cam1`, `usb0`)
- Ensure camera resources are released before switching modes
- Optionally restart MediaMTX when the final active stream is stopped
- Optional vehicle “shutdown hold” behavior (request poweroff)

### Stream control socket
- Socket path: `/tmp/streamctl.sock`
- Request format: one line ASCII command

Supported commands:
- `PING` -> `PONG`
- `QUIT` -> `OK` (no shutdown; just a polite ack)
- `STREAM_ON <cam>`  where `<cam>` is `cam0 | cam1 | usb0`
- `STREAM_OFF <cam>` where `<cam>` is `cam0 | cam1 | usb0`

Behavior notes:
- On `STREAM_ON`, the wrapper reserves capture globally (`motion.capture_reserve()`) and stops motion + dark meter before opening cameras.
- If streaming a MIPI source (`cam0`/`cam1`), the wrapper also reserves CSI (`motion.csi_reserve()`) and may stop the other MIPI stream first.
- On `STREAM_OFF`, when the final active stream is removed, MediaMTX may be restarted and the detector baselines are reseeded.

### GPIO bridge switches
`GpioBridgeSwitch` drives a SOURCE pin high and reads a SENSE pin. This is used for “record” and “motion” intent inputs.

Default pins (BCM):
- Record switch: `RECORD_SRC=5`, `RECORD_SNS=6`
- Motion switch: `MOTION_SRC=16`, `MOTION_SNS=20`

Polling:
- `POLL_SEC=0.2`

### Ambient darkness policy
`camera_dark(state)` uses `motion.dark_state()` to decide whether it is dark, with:
- `IR_FORCE` override (force dark / not dark)
- stale handling with optional “hold last known” behavior

### MediaMTX control
- `restart_mediamtx()` restarts the RTSP server service via `systemctl`
- `wait_for_mediamtx()` blocks until the service is reachable again

---

## Configuration (environment variables)

Recording / output:
- `RECORD_DIR` : recording root directory (default `/mnt/dashcam/recordings`)

IR / darkness behavior:
- `IR_FORCE` : override darkness detection (`1/0`, `true/false`)
- `IR_DARK_STALE_S` : max age of dark-meter value before considered stale
- `IR_HOLD_ON_STALE` : if stale, keep last known dark state
- `IR_BOOT_FORCE_S` : force IR on briefly at boot (seconds)
- `IR_PRIME_MS` : IR prime duration (milliseconds)
- `IR_FLASH_MS` : flash duration (milliseconds)
- `IR_FLASH_GAP_S` : cooldown between flashes (seconds)
- `IR_STREAM_FULL` : streaming IR policy (use full power)
- `IR_STREAM_LP_DUTY` : streaming IR low-power duty
- `IR_RECORD_LP_DUTY` : recording IR low-power duty
- `IR_FULL_CAP_S` : max continuous “full power” time (seconds)
- `NIGHT_STORAGE_VF` : preferred `ffmpeg -vf` filter for night storage

Device handoff:
- `MIPI_REUSE_DELAY_S` : delay before reacquiring CSI devices (seconds)

Switch / poweroff behavior:
- `SWITCH_CLOSED_ON` : switch polarity (`1` means closed => ON)
- `SHUTDOWN_HOLD_S` : if both switches held ON for this long, request poweroff (seconds)

---

## Integration note (imports)
This file was migrated from a versioned prototype and may still reference versioned module names (e.g., `motion_detection_V11`, `IR_lights_V2`, `Full_Proc_V5`). When wiring the package, update imports to the in-repo modules:
- `dashcam.vision.motion`
- `dashcam.hw.ir_lights`
- `dashcam.media.full_proc`

---

## Dependencies
- `gpiozero` (GPIO bridge switches + IR control module)
- `systemctl` access if MediaMTX is managed as a systemd service
- `dashcam.vision.motion` and `dashcam.media.full_proc` for runtime operation

