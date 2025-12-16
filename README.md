# Security Dash Camera (Senior Design)

A multi-camera security dashcam that detects motion, records evidence, and optionally streams a low-bitrate live feed to a mobile app.

## Why this exists
Typical dashcams only record while the vehicle is running. This project targets **parked-vehicle incidents** (break-ins, theft, vandalism) where evidence is most often missing.

## What it does
- **Motion gating**: lightweight motion detection to avoid recording 24/7
- **Full Processing Mode**: multi-camera capture + recording
- **On-demand streaming**: lower-quality stream can be enabled only when requested

## How it’s organized
See:
- `docs/architecture.md` — module breakdown and interfaces
- `docs/state_machine.md` — modes, transitions, timing knobs
- `docs/repo_guide.md` — where to find what

## Repo layout
- `src/dashcam/control/` — top-level orchestration / state machine
- `src/dashcam/vision/` — motion detection + proof artifacts
- `src/dashcam/media/` — recording/encoding/stream pipelines
- `src/dashcam/hw/` — IR lighting and device telemetry helpers
- `scripts/` — test harnesses and experiments
- `configs/` — RTSP server configs (e.g., MediaMTX)
- `systemd/` — service definitions (deployment)

## High-level pipeline (conceptual)

```text
[Motion Trigger]
      |
      v
[VERIFY_MOTION] ----(no motion)----> [IDLE]
      |
   (confirmed)
      v
[FULL_PROCESS]
  |    |      |
  |    |      +--> (optional) RTSP push to MediaMTX -> mobile app
  |    +--> Storage-quality record segments (local)
  +--> Periodic motion checks to decide whether to keep recording

