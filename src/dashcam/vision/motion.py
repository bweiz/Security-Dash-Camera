#!/usr/bin/env python3
# motion_detection_V11.py — CSI+UVC motion detector + camera-dark-meter (race-hardened)
# - Global CAPTURE reservation blocks both USB and CSI re-acquires during streaming bring-up
# - Separate CSI reservation for detector lifetime and MIPI streams
# - Single CSI_OPEN_LOCK serializes all Picamera2 open/close/start/stop
# - Warm-up/reseed, per-cam thresholds, capture hook preserved
# - Small cleanups and clearer logs

import os, glob, time, threading, subprocess
from typing import Optional, List, Dict, Callable
import numpy as np
import cv2

# =================== Globals ===================
_stopped_evt = threading.Event()
_stopped_evt.set()

_detector_stopped = threading.Event()
_detector_stopped.set()

CSI_OPEN_LOCK = threading.Lock()   # serialize any Picamera2 open/close
CSI_IN_USE    = threading.Event()  # held while detector or a MIPI stream uses CSI

# Global capture reservation - used by Proc_Flow
CAPTURE_IN_USE = threading.Event()

def capture_reserve(): CAPTURE_IN_USE.set()
def capture_release(): CAPTURE_IN_USE.clear()
def capture_is_reserved() -> bool: return CAPTURE_IN_USE.is_set()

# CSI reservation helpers
def csi_reserve(): CSI_IN_USE.set()
def csi_release(): CSI_IN_USE.clear()
def csi_is_reserved() -> bool: return CSI_IN_USE.is_set()

# Hook called before each detector capture
_capture_hook: Optional[Callable[[str], None]] = None
def set_capture_hook(fn: Optional[Callable[[str], None]]) -> None:
    global _capture_hook
    _capture_hook = fn

# =================== Knobs ===================
INTERVAL_S     = float(os.getenv("INTERVAL_S", "2.0"))     # one capture every N s
WORK_W, WORK_H = 480, 270
PERCENTILE     = 90.0
AREA_THRESHOLD = 3700
QUIET_SECONDS  = float(os.getenv("QUIET_SECONDS", "10"))
USB_REQ_SIZE   = (1280, 720)

# Per-camera tuning
PER_CAM: Dict[str, Dict[str, float]] = {}

def _per_cam_from_env(label: str, pct_key: str, area_key: str):
    pct  = os.getenv(pct_key)
    area = os.getenv(area_key)
    if pct or area:
        cfg = {}
        if pct:  cfg["pct"]  = float(pct)
        if area: cfg["area"] = int(area)
        PER_CAM[label] = cfg

_per_cam_from_env("mipi0",  "MOTION_PCT_MIPI0",  "MOTION_AREA_MIPI0")
_per_cam_from_env("mipi1",  "MOTION_PCT_MIPI1",  "MOTION_AREA_MIPI1")
_per_cam_from_env("video8", "MOTION_PCT_VIDEO0", "MOTION_AREA_VIDEO0")

# ------- Light-aware threshold profiles (day/night) -------

DAY = {
    "pct":   float(os.getenv("MD_DAY_PCT",   "90")),   
    "area":  int  (os.getenv("MD_DAY_AREA",  "3700")),
    "floor": float(os.getenv("MD_DAY_FLOOR",  "8")),
}
NIGHT = {
    "pct":   float(os.getenv("MD_NIGHT_PCT",   "90")), # more sensitive
    "area":  int  (os.getenv("MD_NIGHT_AREA",  "1500")),
    "floor": float(os.getenv("MD_NIGHT_FLOOR", "5")),
}

# profile override: "", "day", or "night"
_force_profile: str = ""

def set_force_profile(mode: str = ""):
    """
    Override profile:
      "night" -> always use NIGHT
      "day"   -> always use DAY
      ""      -> auto based on dark_state()
    """
    global _force_profile
    m = (mode or "").strip().lower()
    if m in ("day", "night"):
        _force_profile = m
    else:
        _force_profile = ""

def _choose_profile():
    """Return (profile_dict, profile_name) based on override or global dark meter."""
    if _force_profile == "night":
        return NIGHT, "night"
    if _force_profile == "day":
        return DAY, "day"

    try:
        ds = dark_state()
        if ds.get("is_dark", False):
            return NIGHT, "night"
        else:
            return DAY, "day"
    except Exception:
        return DAY, "day"


# Darkness EMA
DARK_EMA_ALPHA  = float(os.getenv("DARK_EMA_ALPHA", "0.2"))
DARK_ON_LUMA    = float(os.getenv("DARK_ON_LUMA",   "30"))
DARK_OFF_LUMA   = float(os.getenv("DARK_OFF_LUMA",  "40"))
DARK_STALE_S    = float(os.getenv("DARK_STALE_S",   "30"))


ENABLE_MIPI = os.getenv("MOTION_ENABLE_MIPI", "1").lower() not in ("0","false","off","no")
ENABLE_USB  = os.getenv("MOTION_ENABLE_USB",  "1").lower() not in ("0","false","off","no")
DARK_METER_USE_MIPI = os.getenv("DARK_METER_USE_MIPI", "0").lower() in ("1","true","yes","on")

# Reuse delay + busy retry
MIPI_REUSE_DELAY_S   = float(os.getenv("MOTION_MIPI_REUSE_DELAY_S", "1.2"))
MIPI_OPEN_RETRIES    = int(os.getenv("MOTION_MIPI_OPEN_RETRIES", "3"))
MIPI_OPEN_BACKOFF_S  = float(os.getenv("MOTION_MIPI_OPEN_BACKOFF_S", "0.5"))

# Debug
DEBUG             = 0
DARK_VERBOSE      = os.getenv("DARK_VERBOSE", "0") in ("1","true","True")
DARKMETER_VERBOSE = os.getenv("DARKMETER_VERBOSE", "0") in ("1","true","True")

_dark_ema: Optional[float] = None
_dark_is_dark: bool = False
_dark_last_ts: float = 0.0

print(f"[motion] loaded from: {__file__}")


def _update_dark_from_frame(Y: np.ndarray) -> None:
    global _dark_ema, _dark_is_dark, _dark_last_ts
    luma = float(np.mean(Y))
    _dark_ema = luma if _dark_ema is None else (1.0 - DARK_EMA_ALPHA) * _dark_ema + DARK_EMA_ALPHA * luma
    if not _dark_is_dark and _dark_ema <= DARK_ON_LUMA: _dark_is_dark = True
    elif _dark_is_dark and _dark_ema >= DARK_OFF_LUMA:  _dark_is_dark = False
    _dark_last_ts = time.time()
    if DARK_VERBOSE: print(f"[dark] mean={luma:.1f} ema={_dark_ema:.1f} is_dark={_dark_is_dark}")

def dark_state() -> Dict[str, Optional[float]]:
    return {"is_dark": _dark_is_dark, "ema": _dark_ema, "ts": _dark_last_ts,
            "stale_after_s": DARK_STALE_S, "on_thr": DARK_ON_LUMA, "off_thr": DARK_OFF_LUMA}

def ambient_is_dark_camera() -> bool:
    return _dark_is_dark


cv2.setUseOptimized(True)
cv2.setNumThreads(1)
os.environ["OPENCV_OPENCL_RUNTIME"] = "disabled"
_KER = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

def _cleanup(mask_u8: np.ndarray) -> np.ndarray:
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN,  _KER, iterations=1)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, _KER, iterations=1)
    return mask_u8

def _largest_blob_area(mask_u8: np.ndarray) -> int:
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
    return 0 if n <= 1 else int(np.max(stats[1:, cv2.CC_STAT_AREA]))

def _resize_keep_width_center_crop(Y: np.ndarray, w: int, h: int) -> np.ndarray:
    H, W = Y.shape[:2]
    if W != w:
        new_h = max(1, int(round(H * (w / float(W)))))
        Y = cv2.resize(Y, (w, new_h), interpolation=cv2.INTER_AREA)
        H, W = Y.shape[:2]
    if H > h:
        top = (H - h) // 2
        Y = Y[top:top+h, :]
    elif H < h:
        pad = h - H
        tp = pad // 2; bp = pad - tp
        Y = np.pad(Y, ((tp, bp), (0, 0)), mode="edge")
    return Y

# =================== Helpers ===================
def _is_uvc(dev: str) -> bool:
    try:
        out = subprocess.run(["v4l2-ctl","-D","-d",dev], capture_output=True, text=True, check=False).stdout
        if "Driver name" in out:
            return "uvcvideo" in out
    except Exception:
        pass
    try:
        node = os.path.basename(dev)
        link = os.path.realpath(f"/sys/class/video4linux/{node}/device/driver")
        return "uvcvideo" in link
    except Exception:
        return False

# =================== Picamera2 (CSI) ===================
try:
    from picamera2 import Picamera2
except Exception:
    Picamera2 = None
if not ENABLE_MIPI:
    Picamera2 = None

if DEBUG:
    print(f"[motion] flags: ENABLE_MIPI={ENABLE_MIPI}, ENABLE_USB={ENABLE_USB}, Picamera2_available={Picamera2 is not None}, DARK_METER_USE_MIPI={DARK_METER_USE_MIPI}")

CSI_MODEL_ALLOWLIST = ("imx", "ov", "ar", "sony", "s5k", "gc", "hm", "os")

class MipiSource:
    def __init__(self, index: int, label: str):
        if Picamera2 is None:
            raise RuntimeError("Picamera2 not available")
        self.index = index
        self.name  = label
        self.cam   = None

        # Busy-retry
        for attempt in range(1, MIPI_OPEN_RETRIES + 1):
            try:
                with CSI_OPEN_LOCK:
                    self.cam = Picamera2(self.index)
                    cfg = self.cam.create_video_configuration(
                        main={"size": (WORK_W, WORK_H), "format": "YUV420"},
                        queue=False, buffer_count=2
                    )
                    self.cam.configure(cfg)
                    self.cam.start()
                time.sleep(0.08)
                return
            except Exception as e:
                msg = (str(e) or "").lower()
                if "acquire" in msg or "busy" in msg or "resource" in msg:
                    if DEBUG: print(f"[motion] MIPI open attempt {attempt} failed (busy). Retrying...")
                    try:
                        with CSI_OPEN_LOCK:
                            if self.cam:
                                try: self.cam.stop()
                                except: pass
                                try: self.cam.close()
                                except: pass
                                self.cam = None
                    except: pass
                    time.sleep(MIPI_OPEN_BACKOFF_S * attempt)
                else:
                    raise
        # Final attempt
        with CSI_OPEN_LOCK:
            self.cam = Picamera2(self.index)
            cfg = self.cam.create_video_configuration(
                main={"size": (WORK_W, WORK_H), "format": "YUV420"},
                queue=False, buffer_count=2
            )
            self.cam.configure(cfg)
            self.cam.start()
        time.sleep(0.08)

    def grab_Y(self) -> Optional[np.ndarray]:
        try:
            buf = self.cam.capture_buffer("main")
            if buf is None: return None
            return np.frombuffer(buf, dtype=np.uint8, count=WORK_W*WORK_H).reshape(WORK_H, WORK_W)
        except Exception as e:
            if DEBUG: print(f"[motion] {self.name} grab error: {e}")
            return None

    def close(self) -> None:
        try:
            with CSI_OPEN_LOCK:
                try: self.cam.stop()
                except: pass
                try: self.cam.close()
                except: pass
        except: pass
        time.sleep(0.10)

# =================== USB ===================
class UsbSource:
    def __init__(self, dev: str):
        self.dev  = dev
        self.name = os.path.basename(dev)  # "video8"
        self.cap  = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        try: self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except: pass
        try: self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except: pass
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  USB_REQ_SIZE[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, USB_REQ_SIZE[1])
        for _ in range(2): self.cap.read()

    def grab_Y(self) -> Optional[np.ndarray]:
        ok, frame = self.cap.read()
        if not ok or frame is None: return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return _resize_keep_width_center_crop(gray, WORK_W, WORK_H)

    def close(self) -> None:
        try: self.cap.release()
        except: pass

# =================== Detector state ===================
sources: List[object]            = []
baselines: Dict[str, np.ndarray] = {}
last_motion_ts: float            = 0.0
idx: int                         = 0
running: bool                    = False
thr: Optional[threading.Thread]  = None
lock = threading.RLock()

# warm-up / reseed
WARMUP_ROUNDS = int(os.getenv("MOTION_WARMUP_ROUNDS", "1"))
_warmup_counts: Dict[str, int] = {}

def reseed_baselines(n_rounds: int = 1) -> None:
    global _warmup_counts
    with lock:
        baselines.clear()
        _warmup_counts = {getattr(s, "name", "src"): int(n_rounds) for s in sources}

def reset_last_motion(ts: float = 0.0) -> None:
    global last_motion_ts
    with lock:
        last_motion_ts = float(ts)

def discover() -> List[object]:
    found: List[object] = []

    # USB first
    if ENABLE_USB:
        for dev in sorted(glob.glob("/dev/video*")):
            if not _is_uvc(dev):
                if DEBUG: print(f"[motion] skip non-uvc node {dev}")
                continue
            try:
                s = UsbSource(dev)
                if s.grab_Y() is None: s.close(); continue
                found.append(s)
                print(f"[motion] USB {dev} ready")
            except Exception as e:
                print(f"[motion] USB {dev} skipped: {e}")

    # CSI sensors
    if Picamera2 is not None and ENABLE_MIPI:
        try:
            info = Picamera2.global_camera_info()
            for i, ci in enumerate(info):
                model = str(ci.get("Model", "")).strip()
                model_l = model.lower()
                if DEBUG: print(f"[motion] Picamera2[{i}] model='{model}'")
                if not any(tok in model_l for tok in CSI_MODEL_ALLOWLIST):
                    if DEBUG: print(f"[motion] skip Picamera2[{i}] (not CSI): '{model}'")
                    continue
                try:
                    s = MipiSource(i, f"mipi{i}")
                    if s.grab_Y() is None: s.close(); continue
                    found.append(s)
                    print(f"[motion] MIPI {i} ready ({model})")
                except Exception as e:
                    if DEBUG: print(f"[motion] MIPI {i} skipped: {e}")
        except Exception as e:
            print(f"[motion] MIPI enumerate failed: {e}")

    if not found:
        print("[motion] no cameras found; detector idle")
    return found

def process_capture(name: str, Y: Optional[np.ndarray]) -> bool:
    if Y is None: return False

    _update_dark_from_frame(Y)

    cur = cv2.GaussianBlur(Y, (3, 3), 0).astype(np.float32)

    cnt = int(_warmup_counts.get(name, 0))
    base = baselines.get(name)
    if base is None or cnt > 0:
        baselines[name] = cur
        if cnt > 0:
            _warmup_counts[name] = cnt - 1
        else:
            if name not in _warmup_counts and WARMUP_ROUNDS > 0:
                _warmup_counts[name] = max(0, WARMUP_ROUNDS - 1)
        return False

    
    #cfg = PER_CAM.get(name, {})
    #pct  = float(cfg.get("pct",  PERCENTILE))
    #ath  = int(cfg.get("area", AREA_THRESHOLD))
    #thr_floor = float(THR_FLOOR_PER_CAM.get(name, THR_FLOOR_GLOBAL))
    

    # Light-aware defaults chosen from day/night, but still allow per-camera overrides.
    prof, prof_name = _choose_profile()
    cfg = PER_CAM.get(name, {})

    # precedence: per-camera override -> light profile -> global constants
    pct       = float(cfg.get("pct",  prof["pct"]))
    ath       = int  (cfg.get("area", prof["area"]))
    thr_floor = float(cfg.get("floor", prof["floor"]))

    diff = cv2.absdiff(cur, base).astype(np.uint8)
    thr_val = max(float(np.percentile(diff, pct)), thr_floor)
    _, mask = cv2.threshold(diff, thr_val, 255, cv2.THRESH_BINARY)
    area = _largest_blob_area(_cleanup(mask))

    baselines[name] = cur

    if DEBUG:
        print(f"[motion] {name} area={area} thr={thr_val:.1f} pct={pct} A>{ath}")

    return area > ath

def rotation_loop():
    global idx, running, last_motion_ts
    next_tick = time.perf_counter()
    try:
        while running:
            if not sources:
                next_tick += INTERVAL_S
                time.sleep(max(0.0, next_tick - time.perf_counter()))
                continue

            s = sources[idx % len(sources)]
            idx = (idx + 1) % max(1, len(sources))
            name = getattr(s, "name", "src")

            if _capture_hook is not None:
                try: _capture_hook(name)
                except Exception as e:
                    if DEBUG: print(f"[motion] hook err: {e}")

            t0 = time.perf_counter()
            try:
                Y = s.grab_Y()
                if process_capture(name, Y):
                    with lock:
                        last_motion_ts = time.time()
                    if DEBUG: print(f"[motion] hit: {name}")
            except Exception as e:
                if DEBUG: print(f"[motion] capture err on {name}: {e}")
            t1 = time.perf_counter()

            print(f"[motion] {name} capture+proc {(t1-t0)*1000:.1f} ms")

            next_tick += INTERVAL_S
            time.sleep(max(0.0, next_tick - time.perf_counter()))
    finally:
        for s in sources:
            try: s.close()
            except: pass
        _detector_stopped.set()

# ===== 1-fps tiny-frame dark meter =====
class _DarkMeter:
    """Samples ONE camera at low res to update dark EMA. """
    def __init__(self, period_s: float = 1.0):
        self.period = period_s
        self._stop = threading.Event()
        self._thr: Optional[threading.Thread] = None
        self._src = None  # ("mipi", Picamera2) or ("uvc", cv2.VideoCapture)

    def _open(self):
        # Never open anything while detector is running or capture is reserved
        if running or capture_is_reserved():
            return

        # Prefer USB; it doesn't contend with libcamera
        if ENABLE_USB:
            for dev in sorted(glob.glob("/dev/video*")):
                if not _is_uvc(dev): continue
                try:
                    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                    try: cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    except: pass
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  160)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
                    self._src = ("uvc", cap); return
                except Exception:
                    continue

        # Only if explicitly enabled AND CSI not reserved
        if DARK_METER_USE_MIPI and Picamera2 is not None and ENABLE_MIPI:
            if CSI_IN_USE.is_set() or capture_is_reserved():
                return
            with CSI_OPEN_LOCK:
                if CSI_IN_USE.is_set() or capture_is_reserved():
                    return
                try:
                    cam = Picamera2(0)
                    cfg = cam.create_video_configuration(
                        main={"size": (96, 54), "format": "YUV420"},
                        queue=False, buffer_count=1
                    )
                    cam.configure(cfg); cam.start(); time.sleep(0.03)
                    self._src = ("mipi", cam); return
                except Exception:
                    self._src = None

    def _close(self):
        if not self._src: return
        kind, obj = self._src
        try:
            if kind == "mipi":
                with CSI_OPEN_LOCK:
                    try: obj.stop()
                    except: pass
                    try: obj.close()
                    except: pass
            else:
                obj.release()
        except: pass
        self._src = None
        time.sleep(0.05)

    def _grab_update(self):
        kind, obj = self._src
        if kind == "mipi":
            buf = obj.capture_buffer("main")
            if buf is None: return
            Y = np.frombuffer(buf, dtype=np.uint8, count=96*54).reshape(54, 96)
            _update_dark_from_frame(Y)
        else:
            ok, frame = obj.read()
            if not ok or frame is None: return
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            Y = cv2.resize(gray, (96, 54), interpolation=cv2.INTER_AREA)
            _update_dark_from_frame(Y)

    def start(self):
        if self._thr and self._thr.is_alive(): return
        self._stop.clear()
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()
        if DARKMETER_VERBOSE: print(f"[darkmeter] start period={self.period}s")

    def stop(self):
        if not self._thr: return
        self._stop.set()
        self._thr.join(timeout=self.period + 1.0)
        if DARKMETER_VERBOSE: print("[darkmeter] stop")
        self._thr = None
        self._close()

    def running(self) -> bool:
        return self._thr is not None and self._thr.is_alive()

    def _run(self):
        while not self._stop.is_set():
            if running or capture_is_reserved():
                self._close()
                self._stop.wait(self.period)
                continue
            if self._src is None:
                self._open()
                if self._src is None:
                    self._stop.wait(self.period)
                    continue
            try:
                self._grab_update()
            except Exception as e:
                if DARKMETER_VERBOSE: print(f"[darkmeter] grab err: {e}")
                self._close()
            self._stop.wait(self.period)

_darkmeter = _DarkMeter(period_s=float(os.getenv("DARK_METER_PERIOD", "1.0")))
def dark_meter_start(): _darkmeter.start()
def dark_meter_stop(block: bool=True, timeout: float=2.0):
    _darkmeter.stop()
    if block:
        t0 = time.time()
        while _darkmeter.running() and time.time() - t0 < timeout:
            time.sleep(0.05)
def dark_meter_running() -> bool: return _darkmeter.running()

# =================== one-time sampler ===================
def sample_dark_once() -> Optional[float]:
    if running or capture_is_reserved(): return None
    Y = None
    # Prefer UVC snapshot
    if ENABLE_USB:
        for dev in sorted(glob.glob("/dev/video*")):
            if not _is_uvc(dev): continue
            try:
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
                ok, frame = cap.read(); cap.release()
                if ok and frame is not None:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    Y = cv2.resize(gray, (96,54), interpolation=cv2.INTER_AREA)
                    break
            except Exception:
                continue
    # Only if explicitly enabled and CSI not in use
    if Y is None and DARK_METER_USE_MIPI and Picamera2 is not None and not CSI_IN_USE.is_set():
        with CSI_OPEN_LOCK:
            try:
                cam = Picamera2(0)
                cfg = cam.create_video_configuration(main={"size": (96, 54), "format": "YUV420"},
                                                     queue=False, buffer_count=1)
                cam.configure(cfg); cam.start(); time.sleep(0.03)
                buf = cam.capture_buffer("main")
                if buf is not None: Y = np.frombuffer(buf, dtype=np.uint8, count=96*54).reshape(54, 96)
                cam.stop(); cam.close()
            except Exception:
                Y = None
    if Y is None: return None
    _update_dark_from_frame(Y)
    return float(np.mean(Y))

# =================== for use ===================
def start() -> None:
    global running, thr, sources
    with lock:
        if running:
            return
        # Kill if streaming is bringing things up
        if capture_is_reserved():
            print("[motion] capture reserved; detector start skipped")
            return
        # Guard CSI for detector
        csi_reserve()
        # Grace before opening cams
        time.sleep(MIPI_REUSE_DELAY_S)
        dark_meter_stop(block=True)
        sources = discover()
        reseed_baselines(WARMUP_ROUNDS)
        _detector_stopped.clear()
        running = True
        thr = threading.Thread(target=rotation_loop, daemon=True)
        thr.start()
        print("[motion] detector started")

def stop() -> None:
    global running, thr
    with lock:
        running = False
    if thr:
        thr.join()
    _detector_stopped.set()
    # small grace + release CSI
    time.sleep(0.10)
    csi_release()
    print("[motion] detector stopped")

def wait_stopped(timeout: float = 1.5) -> bool:
    return _detector_stopped.wait(timeout)

def motion_active() -> bool:
    with lock:
        ts = last_motion_ts
    return (time.time() - ts) <= QUIET_SECONDS

def camera_names() -> List[str]:
    return [getattr(s, "name", "src") for s in sources]

if __name__ == "__main__":
    dark_meter_start()
    try:
        while True:
            ds = dark_state()
            print("motion:", motion_active(), "dark:", ds["is_dark"], "ema:", ds["ema"])
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        dark_meter_stop()
        stop()
