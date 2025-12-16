# Security Dashcam — State Machine

## States
### 1) IDLE
- Cameras may be off
- Motion detector may run periodically or be gated by PIR
Transitions:
- -> VERIFY_MOTION when trigger occurs

### 2) VERIFY_MOTION
- Enable minimal camera pipeline to verify motion
- Optionally enable IR for a short “prime” during dark conditions
Transitions:
- -> FULL_PROCESS if motion confirmed
- -> IDLE if no motion for N checks

### 3) FULL_PROCESS
- Start FullProc (multi-camera workers)
- Always record storage-quality stream
- Streaming pipeline enabled only when requested (mobile app)
Transitions:
- -> FULL_PROCESS (stay) if periodic motion check passes
- -> IDLE if no motion for timeout
- -> SAFE_STOP if shutdown event / error

### 4) SAFE_STOP
- Stop ffmpeg/workers
- Release camera handles
- Turn off IR
Transitions:
- -> IDLE

## Events / inputs
- Motion trigger (software detection)
- PIR trigger (optional future)
- Stream request (socket / control interface)
- Error events (camera failure, ffmpeg failure)
- Shutdown event (service stop / power event)

## Timing parameters (tunable)
- Verify motion window
- Full mode motion check period (e.g., every 20s)
- Full mode idle timeout
- IR prime duration

