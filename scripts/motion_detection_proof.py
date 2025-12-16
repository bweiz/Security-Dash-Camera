#!/usr/bin/env python3
# motion_detection_proof.py — run detector; print metrics for frame/absdiff/threshold/clean/overlay

import os, time, argparse, signal, sys

def parse_args():
    p = argparse.ArgumentParser(description="Run motion detector; print stage metrics (proofs optional).")
    p.add_argument("--interval-s", type=float, default=2.0, help="Round-robin interval between camera grabs")
    p.add_argument("--print-period", type=float, default=0.5, help="Console print interval")
    p.add_argument("--proof", action="store_true", help="Enable proof montages (still prints metrics)")
    p.add_argument("--motion-only", action="store_true", help="Save proofs only when motion is detected")
    p.add_argument("--min-secs", type=float, default=6.0, help="Min seconds between proofs per camera")
    p.add_argument("--proof-dir", default="/home/pi/proof", help="Directory to save proofs")
    p.add_argument("--ext", default="png", choices=["png","jpg"], help="Image format for proofs")
    p.add_argument("--jpg-q", type=int, default=90, help="JPEG quality (if --ext=jpg)")
    p.add_argument("--enable-mipi", action="store_true", help="Force-enable MIPI cameras")
    p.add_argument("--disable-mipi", action="store_true", help="Force-disable MIPI cameras")
    p.add_argument("--enable-usb", action="store_true", help="Force-enable USB cameras")
    p.add_argument("--disable-usb", action="store_true", help="Force-disable USB cameras")
    p.add_argument("--debug", action="store_true", help="Verbose logs")
    return p.parse_args()

def set_env_from_args(a):
    os.environ["INTERVAL_S"] = str(a.interval_s)

    if a.proof:
        os.environ["MOTION_PROOF"] = "1"
        os.environ["MOTION_PROOF_DIR"] = a.proof_dir
        os.environ["MOTION_PROOF_MIN_SECS"] = str(a.min_secs)
        os.environ["MOTION_PROOF_EXT"] = a.ext
        os.environ["MOTION_PROOF_JPG_Q"] = str(a.jpg_q)
        os.environ["MOTION_PROOF_ON_MOTION"] = "1" if a.motion_only else "0"

    if a.enable_mipi:
        os.environ["MOTION_ENABLE_MIPI"] = "1"
    if a.disable_mipi:
        os.environ["MOTION_ENABLE_MIPI"] = "0"
    if a.enable_usb:
        os.environ["MOTION_ENABLE_USB"] = "1"
    if a.disable_usb:
        os.environ["MOTION_ENABLE_USB"] = "0"

    if a.debug:
        os.environ["MOTION_DEBUG"] = "1"

def main():
    args = parse_args()
    set_env_from_args(args)

    try:
        import motion_detection_test as md
    except Exception as e:
        print("[wrapper] Failed to import motion_detection_test:", e)
        sys.exit(1)

    stop_flag = {"stop": False}
    def handle_sigint(signum, frame):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    md.start()

    try:
        print("[wrapper] Running. Ctrl+C to stop. Proof dir:",
              os.getenv("MOTION_PROOF_DIR", "./proof"))
        last_names = []
        while not stop_flag["stop"]:
            names = md.camera_names()
            if names != last_names:
                print("[wrapper] Cameras:", ", ".join(names) if names else "<none>")
                last_names = names

            printed = False
            for nm in names:
                try:
                    if hasattr(md, "metrics_line"):
                        line = md.metrics_line(nm)
                    else:
                        s = md.debug_snapshot(nm)
                        if s:
                            m = s["metrics"]
                            w = s.get("w", 0); h = s.get("h", 0)
                            line = (f"[md] cam={s['cam']} {w}x{h}  "
                                    f"abs.mean={m['abs_mean']:.1f} abs.p95={m['abs_p95']:.1f}  "
                                    f"thr.on={m['thr_on']} clean.on={m['clean_on']}  "
                                    f"blobs={m['num_blobs']} max_area={m['max_area']}  "
                                    f"thr={m['thr_val']:.1f} pct={m['pct']:.1f} floor={m['thr_floor']:.1f}  "
                                    f"motion={m['is_motion']}")
                    if line:
                        print(line)
                        printed = True
                except Exception:
                    # ignore until snapshot exists
                    pass

            if not printed and not names:
                print("[md] no cameras detected")

            time.sleep(args.print_period)
    finally:
        md.stop()
        print("[wrapper] Stopped.")

if __name__ == "__main__":
    main()
