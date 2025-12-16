# record_on_motion_test.py 

import time
from FullProc_V2 import FullProc
import motion_detection_V5 as motion

POLL_SEC        = 0.25
QUIET_GRACE_SEC = 8.0
MIN_RECORD_SEC  = 10.0


RELEASE_WAIT_SEC = 0.8          
ACQUIRE_RETRIES  = 5            
RETRY_SLEEP_SEC  = 0.8

def start_record_with_retry(svc):
    """Start recorder with short backoff if libcamera still releasing."""
    last_err = None
    for attempt in range(1, ACQUIRE_RETRIES + 1):
        try:
            svc.start_record()
            return True
        except Exception as e:
            last_err = e
            msg = str(e)
            # Retry on common handoff errors
            if ("Configured state" in msg) or ("acquire()" in msg) or ("Camera __init__" in msg):
                time.sleep(RETRY_SLEEP_SEC)
                continue
            else:
                break
    print(f"[test] start_record error after retries: {last_err}")
    return False

def main():
    print("[test] starting motion detector…")
    motion.start()

    svc = FullProc(mipi_indices=[0, 1], usb_dev="/dev/video0")

    recording = False
    last_hit_ts = 0.0
    rec_start_ts = 0.0

    try:
        while True:
            active = False
            try:
                active = motion.motion_active()
            except Exception as e:
                print(f"[test] motion check error: {e}")

            now = time.time()

            if not recording:
                if active:
                    print("[test] MOTION → start recording")
                    # Pause detector to release cameras
                    try: motion.stop()
                    except: pass

                    # >>> waitt to let libcamera release <<<
                    time.sleep(RELEASE_WAIT_SEC)

                    if start_record_with_retry(svc):
                        recording = True
                        rec_start_ts = now
                        last_hit_ts = now
                    else:
                        # Recorder failed to start; re-arm detector
                        try: motion.start()
                        except: pass
                # else: keep waiting

            else:
                if active:
                    last_hit_ts = now

                quiet_for = now - last_hit_ts
                rec_for   = now - rec_start_ts

                if quiet_for >= QUIET_GRACE_SEC and rec_for >= MIN_RECORD_SEC:
                    print(f"[test] quiet {quiet_for:.1f}s → stop recording "
                          f"(recorded {rec_for:.1f}s)")
                    try:
                        svc.stop_record()
                    except Exception as e:
                        print(f"[test] stop_record error: {e}")
                    recording = False
                    # re-arm the detector
                    try: motion.start()
                    except Exception as e:
                        print(f"[test] motion restart error: {e}")

            time.sleep(POLL_SEC)

    except KeyboardInterrupt:
        pass
    finally:
        if recording:
            try: svc.stop_record()
            except: pass
        try: motion.stop()
        except: pass
        print("[test] done.")

if __name__ == "__main__":
    main()
