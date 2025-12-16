# Risk Register (Top Engineering Risks)

Format:
- Risk: what can go wrong
- Impact: why it matters
- Mitigation: how we reduce likelihood/impact
- Verification: how we prove mitigation works

---

## R1 — Camera device conflicts / resource locking
- Risk: motion pipeline and full processing pipeline fight over CSI/USB devices; cameras fail to open or hang.
- Impact: missed recordings, system instability, need manual reboot.
- Mitigation:
  - single owner (control wrapper) manages mode transitions
  - explicit stop/release before switching pipelines
  - timeout + fallback path (SAFE_STOP) on failures
- Verification:
  - repeated transitions IDLE→VERIFY→FULL→IDLE for N cycles
  - confirm zero “device busy” errors and no leaked processes

---

## R2 — Encoder/ffmpeg failure under load
- Risk: ffmpeg crashes, stalls, or can’t keep up (thermal throttling, I/O stalls, format mismatch).
- Impact: corrupted recordings, dropped segments, stream outages.
- Mitigation:
  - isolate ffmpeg command construction + logging
  - watchdog checks for stalled pipelines
  - bounded bitrate/resolution profiles (separate storage vs stream)
- Verification:
  - stress test for 30–60 minutes while monitoring segment output continuity
  - inject a failure (kill ffmpeg) and confirm auto-recovery to a valid state

---

## R3 — Lighting transitions / night performance
- Risk: false positives/negatives due to low light, headlights, IR reflections, exposure shifts.
- Impact: missed events or constant triggering.
- Mitigation:
  - “dark meter” / profile selection for thresholds
  - optional IR prime/brightness control
  - minimum motion confirmation window before FULL_PROCESS
- Verification:
  - test scenarios: daylight, dusk, night, headlights sweep
  - record false trigger rate and missed detection rate across scenarios

---

## R4 — Storage and I/O constraints
- Risk: SD card/SSD can’t sustain write rates; fragmentation or full disk causes failure.
- Impact: dropped/corrupted recording segments.
- Mitigation:
  - segmented recording files (fixed duration chunks)
  - keep storage stream separate from stream quality
  - (future) disk space monitoring + retention policy
- Verification:
  - run at target bitrate, verify segment creation rate and file integrity
  - run near-full disk test and confirm graceful behavior (log + stop/retain)

---

## R5 — Process lifecycle and shutdown safety
- Risk: unsafe shutdown leaves camera locked, orphan ffmpeg processes, or GPIO left on (IR).
- Impact: requires reboot; drains power; unreliable in vehicle power events.
- Mitigation:
  - SAFE_STOP state that always executes cleanup
  - ensure IR off + child processes terminated on exit
  - systemd restart policy (deployment)
- Verification:
  - send SIGTERM repeatedly during recording/streaming
  - confirm no orphan processes, no locked devices, IR returns to OFF state

---

## R6 — Latency exceeds target for live streaming
- Risk: end-to-end delay grows beyond target (buffering, encoder queue, RTSP server settings).
- Impact: stream becomes unusable for “live” monitoring.
- Mitigation:
  - low-latency ffmpeg settings for stream profile
  - use lower bitrate/resolution for stream pipeline
  - keep stream disabled unless requested
- Verification:
  - measure glass-to-glass latency under normal and stressed CPU conditions
  - confirm <= target threshold (e.g., 20s) under defined operating profile

