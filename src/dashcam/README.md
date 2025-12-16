# Dashcam package (`dashcam`)

Python package for the Security Dash Camera project.

## Subpackages
- `dashcam.control` — top-level orchestration / state machine
- `dashcam.vision`  — software motion detection + dark meter
- `dashcam.media`   — full processing mode (record + optional RTSP)
- `dashcam.hw`      — IR control and device telemetry helpers

## Philosophy
- `src/dashcam/*` should be reusable modules (avoid hardcoded paths where possible)
- proof-of-concept runners belong in `scripts/`
- deployment-related files live in `systemd/` and `configs/`

