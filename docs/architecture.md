# Security Dashcam — Architecture (High Level)

## Goal
A multi-camera dashcam that can:
- Detect motion (low-cost “pre-filter” stage)
- Transition into Full Processing Mode to record multi-camera video
- Optionally stream a lower-bitrate feed to a mobile app on request
- Operate reliably in a vehicle environment (power events, lighting changes, intermittent connectivity)

## System modes (conceptual)
1. **Idle / Parked**
   - Cameras may be OFF for power savings (optional IR off)
   - Motion trigger may come from PIR (future/optional) and/or periodic software checks

2. **Motion Verify**
   - Lightweight software motion detection validates that motion is “real”
   - If verified, transition into Full Processing Mode

3. **Full Processing Mode**
   - Record from all cameras
   - Create storage stream (higher quality)
   - Create streaming stream (lower quality) only when requested
   - Periodically re-check motion to decide whether to keep recording

4. **Shutdown / Safe Stop**
   - Stop encoders/workers cleanly
   - Release camera devices and GPIO resources

## Major software modules (repo mapping)
- `dashcam/control/app.py`
  - Top-level wrapper / state machine
  - Owns transitions between modes
  - Coordinates motion detection vs recording vs streaming control

- `dashcam/vision/motion.py`
  - Software motion detection pipeline
  - Responsible for:
    - acquiring frames from camera sources
    - running a lightweight motion algorithm
    - emitting “motion active / inactive” state + debug/proof artifacts

- `dashcam/media/full_proc.py`
  - Full processing engine (workers per camera source)
  - Responsible for:
    - capture/encode/record pipelines
    - stream pipeline (when enabled)
    - managing concurrent camera use (avoid device conflicts)

- `dashcam/hw/ir_lights.py`
  - IR illumination control abstraction
  - Supports real GPIO PWM and a simulation fallback

- `dashcam/hw/health.py`
  - Pi health telemetry logging (temp, throttling, etc.)

## Key interfaces (what talks to what)
- `App (control)` calls:
  - `MotionDetector.start()/stop()/is_motion_active()`
  - `FullProc.start()/stop()`
  - `StreamCtlServer` or socket message to toggle streaming

- `FullProc` uses:
  - camera sources (CSI / USB)
  - `ffmpeg` encoding/segmenting for storage + optional RTSP push

- `MotionDetector` may use:
  - IR control during night/dark conditions (optional)

## Data outputs
- **Recordings** (local storage)
  - segmented files (e.g., 30s chunks)
  - timestamp overlay may be applied in the pipeline

- **Streaming**
  - RTSP endpoint served by MediaMTX (or equivalent)
  - lower bitrate, bounded latency target

- **Debug artifacts**
  - proof frames / motion overlays / logs
  - useful for tuning thresholds and demonstrating correctness

## Constraints / targets (engineering)
- Latency from capture to stream: <= ~20 seconds (target)
- Streaming bitrate: ideally <= ~10 Mbps
- Must tolerate:
  - day/night lighting changes
  - USB camera enumeration changes (real world)
  - service restarts and resource cleanup

## Future extensions (planned)
- PIR integration for parked-mode wake-up
- Configuration profiles per vehicle / per camera
- Health watchdog + auto-recover flows
- Motion deadspots to help "overactive" motion detection
