
# Full_Proc.py — robust capture/record + RTSP streaming for MIPI (Picamera2) and USB (v4l2)
# - Hardened start/stop ordering so MIPI re-acquires cleanly 
# - Threads exit if publisher is None; no AttributeError on .stdin/.stdout
# - On stream stop (and not recording), camera is closed to release media nodes
# - Idempotent starts

import os, time, threading, subprocess, argparse, signal, socket, gc
from pathlib import Path

# ==================== CONFIG ====================
FPS, (W, H)   = 10, (1920, 1080)
SEG_SECONDS   = 30               # length of each MP4 segment (exported for wrapper)
STORAGE_CRF   = 23               # higher = smaller files (lower quality)
STREAM_CRF    = 30
X264_THREADS  = 0                # 0 lets x264 use all cores
ROOT_DIR      = os.path.expanduser("/mnt/dashcam/recordings")
#USB_INPUT_FMT = "yuyv422"

USB_DEV = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._USB_Camera_SN0001-video-index0"
USB_INPUT_FMT = "mjpeg"     
W, H = 1920, 1080           

STORAGE_VF = os.environ.get("eq=brightness=0.3:contrast=1.5", "null")

# Kill/teardown timings
PROC_KILL_GRACE_S     = 1.5
CAM_RELEASE_SLEEP_S   = 0.10
PUBLISHER_START_DELAY = 0.02

# Ensure root directory exists
Path(ROOT_DIR).mkdir(parents=True, exist_ok=True)
# =================================================

# ---------- RTSP URL helpers ----------
def _get_pi_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

RTSP_URL_ROOT_CLIENT = os.environ.get("RTSP_URL_ROOT_CLIENT", f"rtsp://{_get_pi_ip()}:8554")
RTSP_URL_ROOT_LOCAL  = os.environ.get("RTSP_URL_ROOT_LOCAL",  "rtsp://127.0.0.1:8554")

def rtsp_url_for(name: str) -> str:        # what users open
    return f"{RTSP_URL_ROOT_CLIENT}/{name}"

def rtsp_publish_url(name: str) -> str:    # where ffmpeg publishes
    return f"{RTSP_URL_ROOT_LOCAL}/{name}"
# ---------------------------------------

# ----- File Management -----
def ensure_dir(path: str): os.makedirs(path, exist_ok=True)
def out_pattern(cam_name: str):
    cam_dir = os.path.join(ROOT_DIR, cam_name); ensure_dir(cam_dir)
    return os.path.join(cam_dir, f"{cam_name}_%Y%m%d-%H%M%S.mp4")

# ==================== FFmpeg builders ====================
def ffmpeg_storage_from_stdin(cam_name: str):
    gop = FPS * 2
    return [
        "ffmpeg","-hide_banner","-loglevel","warning","-y","-nostdin",
        "-f","rawvideo","-pix_fmt","yuv420p","-s",f"{W}x{H}","-r",str(FPS),"-i","-",
        #"-vf","eq=brightness=0.3:contrast=1.5",
        "-vf",STORAGE_VF,
        "-an","-c:v","libx264","-preset","ultrafast","-tune","zerolatency",
        "-crf",str(STORAGE_CRF),"-g",str(gop),"-bf","0","-profile:v","baseline",
        "-threads",str(X264_THREADS),"-pix_fmt","yuv420p",
        "-force_key_frames",f"expr:gte(t,n_forced*{SEG_SECONDS})",
        "-f","segment","-segment_time",str(SEG_SECONDS),"-reset_timestamps","1",
        "-strftime","1",
        "-segment_format","mp4", out_pattern(cam_name)
    ]

def ffmpeg_storage_from_v4l2(dev: str, cam_name: str):
    gop = FPS * 2
    return [
        "ffmpeg","-hide_banner","-loglevel","warning","-y","-nostdin",
        "-f","v4l2","-thread_queue_size","64","-input_format",USB_INPUT_FMT,
        "-framerate",str(FPS),"-video_size",f"{W}x{H}","-i",dev,
        "-vf",STORAGE_VF,
        #"-vf","eq=brightness=0.3:contrast=1.5",
        #"-filter_complex","extractplanes=y+u+v[y][u][v]; [y]histeq=strength=0.2:intensity=0.3[lumaeq]; [lumaeq][u][v]mergeplanes=0x001020:yuv422p[out]","-map","[out]",
        "-an","-c:v","libx264","-preset","ultrafast","-tune","zerolatency",
        "-crf",str(STORAGE_CRF),"-g",str(gop),"-bf","0","-profile:v","baseline",
        "-threads",str(X264_THREADS),"-pix_fmt","yuv420p",
        "-force_key_frames",f"expr:gte(t,n_forced*{SEG_SECONDS})",
        "-f","segment","-segment_time",str(SEG_SECONDS),"-reset_timestamps","1",
        "-strftime","1",
        "-segment_format","mp4", out_pattern(cam_name)
    ]


def ffmpeg_stream_from_stdin():
    gop = FPS
    return [
        "ffmpeg","-hide_banner","-loglevel","warning","-y","-nostdin",
        "-f","rawvideo","-pix_fmt","yuv420p","-s","1280x720","-r",str(FPS),"-i","-",
        "-an","-c:v","libx264","-preset","ultrafast","-tune","zerolatency",
        "-crf",str(STREAM_CRF),"-g",str(gop),"-bf","0","-profile:v","baseline",
        "-threads",str(X264_THREADS),
        "-x264-params","scenecut=0:repeat-headers=1",
        "-bsf:v","h264_metadata=aud=insert",
        "-f","h264","-"  # -> stdout
    ]

def ffmpeg_stream_from_v4l2(dev: str):
    gop = FPS
    return [
        "ffmpeg","-hide_banner","-loglevel","warning","-y","-nostdin",
        "-f","v4l2","-thread_queue_size","64","-input_format",USB_INPUT_FMT,
        "-framerate",str(FPS),"-video_size",f"{W}x{H}","-i",dev,
        "-vf","scale=1280:720:flags=fast_bilinear,format=yuv420p,hue=s=0",
        "-an","-c:v","libx264","-preset","ultrafast","-tune","zerolatency",
        "-crf",str(STREAM_CRF),"-g",str(gop),"-bf","0","-profile:v","baseline",
        "-threads",str(X264_THREADS),
        "-x264-params","scenecut=0:repeat-headers=1",
        "-bsf:v","h264_metadata=aud=insert",
        "-f","h264","-"
    ]

def ffmpeg_usb_record_and_stdout(dev: str, cam_name: str):
    return [
        "ffmpeg","-hide_banner","-loglevel","warning","-y","-nostdin",
        "-fflags","+genpts","-use_wallclock_as_timestamps","1",
        "-f","v4l2","-thread_queue_size","64","-input_format",USB_INPUT_FMT,
        "-framerate",str(FPS),"-video_size",f"{W}x{H}","-i",dev,
        "-pix_fmt","yuv420p",
        "-c:v","libx264","-preset","ultrafast","-tune","zerolatency",
        "-crf",str(STORAGE_CRF),"-g",str(FPS),"-bf","0","-profile:v","baseline",
        "-threads",str(X264_THREADS),"-movflags","+faststart",
        "-x264-params","scenecut=0:repeat-headers=1",
        "-bsf:v","h264_metadata=aud=insert",
        "-f","tee",
        f"[f=segment:segment_time={SEG_SECONDS}:reset_timestamps=1:strftime=1]{out_pattern(cam_name)}|[f=h264]pipe:1"
    ]

def ffmpeg_rtsp_publish(url: str):
    return [
        "ffmpeg","-hide_banner","-loglevel","warning","-y","-nostdin",
        "-fflags","+genpts","-use_wallclock_as_timestamps","1",
        "-f","h264","-i","-",
        "-c:v","copy","-rtsp_transport","tcp",
        "-muxdelay","0","-muxpreload","0",
        "-f","rtsp", url
    ]
# ===========================================================

# ==================== MIPI camera (Picamera2) ====================
from picamera2 import Picamera2

class MipiWorker:
    def __init__(self, index: int, name: str):
        self.index, self.name = index, name
        self.cam = None
        self.proc = None      # storage ffmpeg (stdin = raw 1080p)
        self.sproc = None     # stream encoder (stdin = raw 720p, stdout = h264)
        self.rtsp = None      # RTSP publisher (stdin = h264)
        self._cap_thr = None
        self._pump_thr = None
        self._stop_evt = threading.Event()
        self._lock = threading.RLock()

    # ---------- camera lifecycle ----------
    def _ensure_camera(self):
        if self.cam is not None:
            return
        info = Picamera2.global_camera_info()
        if self.index < 0 or self.index >= len(info):
            raise RuntimeError(f"{self.name}: MIPI index {self.index} not found (have {len(info)})")
        self.cam = Picamera2(self.index)
        cfg = self.cam.create_video_configuration(
            main={"size": (1920, 1080), "format": "YUV420"},    # feeds storage
            lores={"size": (1280, 720), "format": "YUV420"},    # feeds stream
            controls={"FrameDurationLimits": (int(1e6/FPS), int(1e6/FPS))},
            queue=False,
        )
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(0.03)  # small settle

    def _close_camera(self):
        cam = self.cam
        self.cam = None
        if cam is None:
            return
        try:
            cam.stop()
        except Exception:
            pass
        time.sleep(CAM_RELEASE_SLEEP_S)
        try:
            cam.close()
        except Exception:
            pass
        gc.collect()
        time.sleep(CAM_RELEASE_SLEEP_S)

    def _has_any_consumer(self):
        return (self.proc is not None) or (self.sproc is not None)

    # ---------- capture thread ----------
    def _start_capture_thread(self):
        if self._cap_thr and self._cap_thr.is_alive():
            return
        self._stop_evt.clear()
        self._cap_thr = threading.Thread(target=self._capture_loop, daemon=True)
        self._cap_thr.start()

    def _capture_loop(self):
        EXP_LORES = 1280*720*3//2  # YUV420p bytes
        EXP_MAIN  = 1920*1080*3//2
        try:
            while not self._stop_evt.is_set():
                did = False

                # STREAM feed (lores -> sproc.stdin)
                sp = self.sproc
                if sp is not None and sp.stdin is not None:
                    try:
                        lores_buf = self.cam.capture_buffer("lores")
                        if len(lores_buf) != EXP_LORES:
                            # Trim to expected
                            lores_buf = memoryview(lores_buf)[:EXP_LORES]
                        sp.stdin.write(lores_buf)
                        did = True
                    except Exception:
                        with self._lock:
                            self.sproc = None

                # STORAGE feed (main -> proc.stdin)
                pr = self.proc
                if pr is not None and pr.stdin is not None:
                    try:
                        main_buf = self.cam.capture_buffer("main")
                        if len(main_buf) != EXP_MAIN:
                            main_buf = memoryview(main_buf)[:EXP_MAIN]
                        pr.stdin.write(main_buf)
                        did = True
                    except Exception:
                        with self._lock:
                            self.proc = None

                if not did:
                    time.sleep(0.005)
        finally:
            # close stdin pipes to flush ffmpeg gracefully if still open
            try:
                if self.sproc and self.sproc.stdin:
                    self.sproc.stdin.flush()
                    self.sproc.stdin.close()
            except Exception:
                pass
            try:
                if self.proc and self.proc.stdin:
                    self.proc.stdin.flush()
                    self.proc.stdin.close()
            except Exception:
                pass

    # ---------- RTSP publisher ----------
    def _start_rtsp_if_needed(self):
        if (self.rtsp is None) or (self.rtsp.poll() is not None):
            url_pub = rtsp_publish_url(self.name)
            self.rtsp = subprocess.Popen(
                ffmpeg_rtsp_publish(url_pub),
                stdin=subprocess.PIPE, bufsize=0, start_new_session=True
            )
            time.sleep(PUBLISHER_START_DELAY)
            print(f"[RTSP] {self.name}: publishing @ {rtsp_url_for(self.name)}")

    # ---------- for use ----------
    def start(self):
        """Start 1080p segmented recording."""
        with self._lock:
            self._ensure_camera()
            self._start_capture_thread()
            if self.proc is None:
                self.proc = subprocess.Popen(
                    ffmpeg_storage_from_stdin(self.name),
                    stdin=subprocess.PIPE, start_new_session=True
                )
            print(f"[INFO] {self.name}: recording 1080p segments…")

    def start_queue(self):
        """Start RTSP streaming (1280x720)."""
        with self._lock:
            if self.sproc is not None and self.sproc.poll() is None:
                print(f"[INFO] {self.name}: stream already running")
                return

            self._ensure_camera()
            self._start_capture_thread()

            # start stream encoder (stdin raw yuv; stdout h264)
            self.sproc = subprocess.Popen(
                ffmpeg_stream_from_stdin(),
                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                bufsize=0, start_new_session=True
            )

            # pump h264 -> RTSP
            def pump():
                try:
                    while not self._stop_evt.is_set():
                        sp = self.sproc
                        if sp is None or sp.stdout is None:
                            return
                        if sp.poll() is not None:
                            return
                        chunk = sp.stdout.read(65536)
                        if not chunk:
                            time.sleep(0.002)
                            continue
                        self._start_rtsp_if_needed()
                        try:
                            if self.rtsp and self.rtsp.stdin:
                                self.rtsp.stdin.write(chunk)
                        except BrokenPipeError:
                            try:
                                if self.rtsp and self.rtsp.stdin:
                                    self.rtsp.stdin.close()
                            except Exception:
                                pass
                            self.rtsp = None
                        except Exception:
                            self.rtsp = None
                finally:
                    # close publisher stdin on exit
                    try:
                        if self.rtsp and self.rtsp.stdin:
                            self.rtsp.stdin.close()
                    except Exception:
                        pass

            self._pump_thr = threading.Thread(target=pump, daemon=True)
            self._pump_thr.start()
            print(f"[RTSP] {self.name}: encoder live (-> {rtsp_url_for(self.name)})")

    def stop_queue(self):
        """Stop RTSP streaming. If not recording, also close camera."""
        with self._lock:
            # stop publisher
            if self.rtsp:
                try:
                    if self.rtsp.stdin: self.rtsp.stdin.close()
                except Exception:
                    pass
                try:
                    self.rtsp.terminate()
                    self.rtsp.wait(timeout=PROC_KILL_GRACE_S)
                except Exception:
                    try: self.rtsp.kill()
                    except Exception: pass
                self.rtsp = None

            # stop stream encoder
            sp = self.sproc
            self.sproc = None
            if sp:
                try:
                    if sp.stdin: sp.stdin.close()
                except Exception:
                    pass
                try:
                    sp.terminate()
                    sp.wait(timeout=PROC_KILL_GRACE_S)
                except Exception:
                    try: sp.kill()
                    except Exception: pass

            # wait pump thread down
            if self._pump_thr and self._pump_thr.is_alive():
                self._stop_evt.set()
                self._pump_thr.join(timeout=1.0)
            self._pump_thr = None
            print(f"[RTSP] {self.name}: stopped.")

            # If not recording, stop capture thread & close camera to release MIPI
            if self.proc is None:
                self._stop_evt.set()
                if self._cap_thr and self._cap_thr.is_alive():
                    self._cap_thr.join(timeout=1.0)
                self._cap_thr = None
                self._close_camera()
                self._stop_evt.clear()  # ready for next start

    def stop_all(self):
        """Stop streaming + recording and close camera."""
        with self._lock:
            # stop streaming side
            self.stop_queue()

            # stop storage encoder
            pr = self.proc
            self.proc = None
            if pr:
                try:
                    if pr.stdin:
                        try: pr.stdin.flush()
                        except Exception: pass
                        try: pr.stdin.close()
                        except Exception: pass
                except Exception:
                    pass
                try:
                    pr.wait(timeout=5)
                except Exception:
                    try:
                        pr.terminate()
                        pr.wait(timeout=2)
                    except Exception:
                        try: pr.kill()
                        except Exception: pass

            # stop capture thread
            self._stop_evt.set()
            if self._cap_thr and self._cap_thr.is_alive():
                self._cap_thr.join(timeout=1.0)
            self._cap_thr = None
            self._stop_evt.clear()

            # close camera
            self._close_camera()
# =============================================================

# ==================== USB camera (v4l2) ======================
class UsbWorker:
    def __init__(self, dev: str, name: str):
        self.dev, self.name = dev, name
        self.proc = None      # ffmpeg process
        self.mode = "idle"    # "idle" | "record" | "stream" | "combined"
        self.rtsp = None
        self._pump_thr = None
        self._stop_evt = threading.Event()

    def _spawn(self, argv, stdout=False):
        return subprocess.Popen(argv,
                                stdout=(subprocess.PIPE if stdout else None),
                                bufsize=0 if stdout else -1,
                                start_new_session=True)

    def _pump_to_rtsp(self, proc_with_stdout):
        def pump():
            try:
                while not self._stop_evt.is_set():
                    if proc_with_stdout.poll() is not None:
                        return
                    chunk = proc_with_stdout.stdout.read(65536)
                    if not chunk:
                        time.sleep(0.002); continue
                    if (self.rtsp is None) or (self.rtsp.poll() is not None):
                        url_pub = rtsp_publish_url(self.name)
                        self.rtsp = subprocess.Popen(
                            ffmpeg_rtsp_publish(url_pub),
                            stdin=subprocess.PIPE, bufsize=0, start_new_session=True
                        )
                        time.sleep(PUBLISHER_START_DELAY)
                        print(f"[RTSP] {self.name}: publishing @ {rtsp_url_for(self.name)}")
                    try:
                        self.rtsp.stdin.write(chunk)
                    except Exception:
                        try:
                            if self.rtsp and self.rtsp.stdin: self.rtsp.stdin.close()
                        except Exception: pass
                        self.rtsp = None
            finally:
                try:
                    if self.rtsp and self.rtsp.stdin:
                        self.rtsp.stdin.close()
                except Exception:
                    pass
        self._stop_evt.clear()
        self._pump_thr = threading.Thread(target=pump, daemon=True)
        self._pump_thr.start()

    def _stop_rtsp(self):
        if self.rtsp:
            try:
                if self.rtsp.stdin: self.rtsp.stdin.close()
            except Exception: pass
            try:
                self.rtsp.terminate()
                self.rtsp.wait(timeout=PROC_KILL_GRACE_S)
            except Exception:
                try: self.rtsp.kill()
                except Exception: pass
            self.rtsp = None

    def start_record(self):
        self.stop_all()
        self.proc = self._spawn(ffmpeg_storage_from_v4l2(self.dev, self.name))
        self.mode = "record"
        print(f"[INFO] {self.name}: recording 1080p segments…")

    def start_queue_only(self):
        self.stop_all()
        self.proc = self._spawn(ffmpeg_stream_from_v4l2(self.dev), stdout=True)
        self._pump_to_rtsp(self.proc)
        self.mode = "stream"
        print(f"[RTSP] {self.name}: encoder live (-> {rtsp_url_for(self.name)})")

    def upgrade_to_combined(self):
        self.stop_all()
        self.proc = self._spawn(ffmpeg_usb_record_and_stdout(self.dev, self.name), stdout=True)
        self._pump_to_rtsp(self.proc)
        self.mode = "combined"
        print(f"[RTSP] {self.name}: combined record+stream (-> {rtsp_url_for(self.name)})")

    def stop_all(self):
        # stop RTSP publisher / pump thread
        self._stop_evt.set()
        if self._pump_thr and self._pump_thr.is_alive():
            self._pump_thr.join(timeout=1.0)
        self._pump_thr = None
        self._stop_rtsp()

        # stop ffmpeg
        proc = self.proc
        self.proc = None
        self.mode = "idle"
        if not proc:
            return

        try:
            pgid = os.getpgid(proc.pid)
        except Exception:
            pgid = None

        def send(sig, wait_s):
            try:
                (os.killpg(pgid, sig) if pgid is not None else os.kill(proc.pid, sig))
            except ProcessLookupError:
                return True
            try:
                proc.wait(timeout=wait_s)
                return True
            except Exception:
                return proc.poll() is not None

        if not send(signal.SIGINT, 6):
            if not send(signal.SIGTERM, 3):
                send(signal.SIGKILL, 1)
# =============================================================

# ==================== FullProc orchestrator ==================
class FullProc:
    def __init__(self, mipi_indices=[0], usb_dev: str | None = None):
        # build camera map
        self.cams = {}
        try:
            from picamera2 import Picamera2 as _P2
            count = len(_P2.global_camera_info())
        except Exception:
            count = 0
        m = [i for i in mipi_indices if 0 <= i < count]
        if len(m) > 0: self.cams["cam0"] = MipiWorker(m[0], "cam0")
        if len(m) > 1: self.cams["cam1"] = MipiWorker(m[1], "cam1")
        if usb_dev: self.cams["usb0"] = UsbWorker(usb_dev, "usb0")

    # ---- recording ----
    def start_record(self):
        os.makedirs(ROOT_DIR, exist_ok=True)
        for name in ("cam0","cam1","usb0"):
            p = os.path.join(ROOT_DIR, name); ensure_dir(p)
        for cam in self.cams.values():
            if isinstance(cam, MipiWorker):
                cam.start()
            elif isinstance(cam, UsbWorker):
                cam.start_record()

    def stop_record(self):
        for cam in self.cams.values():
            if isinstance(cam, MipiWorker):
                cam.stop_all()
            elif isinstance(cam, UsbWorker):
                cam.stop_all()

    # ---- stream on/off ----
    def start_queue(self, cam_name: str):
        cam = self.cams.get(cam_name)
        if not cam:
            print(f"[WARN] Unknown cam '{cam_name}'"); return
        if isinstance(cam, MipiWorker):
            cam.start_queue()
        elif isinstance(cam, UsbWorker):
            if cam.mode == "record":
                cam.upgrade_to_combined()
            else:
                cam.start_queue_only()

    def stop_queue(self, cam_name: str):
        cam = self.cams.get(cam_name)
        if not cam: return
        if isinstance(cam, MipiWorker):
            cam.stop_queue()
        elif isinstance(cam, UsbWorker):
            if cam.mode == "combined":
                cam.start_record()  # drop back to record-only
            else:
                cam.stop_all()
# =============================================================

# ===================== CLI for testing =======================
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Full Processing (record + RTSP)")
    ap.add_argument("--mipi", type=int, nargs="*", default=[0], help="Picamera2 indices, e.g., 0 1")
    ap.add_argument("--usb", type=str, default=None, help="USB v4l2 device, e.g., /dev/video0")
    ap.add_argument("--record", action="store_true", help="start 1080p segmented recording")
    ap.add_argument("--queue", nargs="*", default=[], help="cams to start RTSP: cam0 cam1 usb0")
    args = ap.parse_args()

    svc = FullProc(mipi_indices=args.mipi, usb_dev=args.usb)

    if args.record:
        svc.start_record()
    for cam in args.queue:
        svc.start_queue(cam)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for cam in args.queue:
            svc.stop_queue(cam)
        if args.record:
            svc.stop_record()
        print("[INFO] Stopped.")
