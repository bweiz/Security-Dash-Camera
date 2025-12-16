# Scripts

This folder contains one-off test harnesses and proof tools used during development.
These are not the “production” entrypoints; they exist to validate subsystems quickly.

## Files
- `motion_detection_test.py`
  - Quick motion detector run loop
  - Used for tuning thresholds, cadence, and validating camera acquisition behavior

- `motion_detection_proof.py`
  - Produces proof artifacts (frames/overlays/logs) to show motion detection behavior
  - Useful for debugging false positives and validating day/night transitions

- `record_on_motion_test.py`
  - Integration harness: motion detection triggers Full Processing Mode recording
  - Used to validate transitions and “safe handoff” between motion pipeline and record pipeline

