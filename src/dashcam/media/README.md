# Media (`dashcam.media`)

Full Processing Mode (recording + optional RTSP streaming).

This module owns the capture/encode pipelines and is designed for clean start/stop behavior under repeated transitions.

---

## `full_proc.py`

Robust capture/record + RTSP streaming for:
- MIPI (Picamera2) sources
- USB (v4l2) sources

### Design goals (as implemented)
- Hardened start/stop ordering so MIPI re-acquires cleanly
- Threads exit cleanly when publisher is absent
- When streaming stops and not recording, cameras are closed to release media nodes
- Idempotent starts (safe to call start multiple times)

---

## Key configuration (module constants)
- `FPS`, `W`, `H`
- `SEG_SECONDS` : MP4 segment duration (exported)
- `STORAGE_CRF` : storage quality (lower is higher quality)
- `STREAM_CRF`  : stream quality
- `ROOT_DIR`    : default storage root (`/mnt/dashcam/recordings`)
- `USB_DEV` and `USB_INPUT_FMT` (USB capture device + input format)

---

## RTSP URL helpers
Environment variables:
- `RTSP_URL_ROOT_CLIENT` : what clients open (defaults to `rtsp://<pi-ip>:8554`)
- `RTSP_URL_ROOT_LOCAL`  : what the publisher uses locally (defaults to `rtsp://127.0.0.1:8554`)

Functions:
- `rtsp_url_for(name)`      -> client URL (view)
- `rtsp_publish_url(name)`  -> publish URL (ffmpeg push)

---

## ffmpeg command builders
Storage:
- `ffmpeg_storage_from_stdin(cam_name)`
- `ffmpeg_storage_from_v4l2(dev, cam_name)`

Streaming:
- `ffmpeg_stream_from_stdin()`
- `ffmpeg_stream_from_v4l2(dev)`
- `ffmpeg_rtsp_publish(url)`

Combined USB path:
- `ffmpeg_usb_record_and_stdout(dev, cam_name)` (record + tee/stdout use)

---

## Classes

### `MipiWorker`
Owns one CSI camera index and manages:
- recording pipeline
- optional streaming queue pipeline
- safe stop/close behavior

Methods:
- `start()`
- `start_queue()`
- `stop_queue()`
- `stop_all()`

### `UsbWorker`
Owns one `/dev/video*` device and manages:
- record-only
- queue-only
- upgrade-to-combined mode

Methods:
- `start_record()`
- `start_queue_only()`
- `upgrade_to_combined()`
- `stop_all()`

### `FullProc`
Top-level service used by the wrapper.

Methods:
- `start_record()`, `stop_record()`
- `start_queue(cam_name)`, `stop_queue(cam_name)`

---

## Storage layout
- Per-camera directory under `ROOT_DIR`
- Filename pattern includes timestamp (see `out_pattern(cam_name)`)

---

## Notes
- Storage video filter:
  - The module currently initializes `STORAGE_VF` at import time.
  - The wrapper sets `os.environ["STORAGE_VF"]` based on ambient light; if dynamic switching is desired, the module should read `os.getenv("STORAGE_VF", "null")` at ffmpeg command build time (or update the module variable before starting record).

- Known issue in prototype:
  - `STORAGE_VF` is currently defined using an incorrect environment key string.
  - Intended behavior is typically: `STORAGE_VF = os.getenv("STORAGE_VF", "null")`.

