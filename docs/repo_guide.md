# Repo Guide

## Where to look
- `src/dashcam/control/` — high-level orchestration
- `src/dashcam/vision/`  — motion detection + proof artifacts
- `src/dashcam/media/`   — recording/encoding/stream pipelines
- `src/dashcam/hw/`      — GPIO/IR + telemetry utilities
- `scripts/`             — proof-of-concept runners and experiments
- `configs/`             — server configs (MediaMTX) and profiles
- `systemd/`             — service definitions for deployment

## Conventions
- No hardcoded paths inside `src/dashcam/*`
- Configuration comes from env/config layer (future: config.py)
- Keep “test harness scripts” in `scripts/` (not mixed into library code)

