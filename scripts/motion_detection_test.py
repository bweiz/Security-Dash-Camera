#!/usr/bin/env python3
# motion_detection_test.py — CSI+UVC motion detector with (optional) proofs + debug metrics

import os, glob, time, threading, subprocess, json
from datetime import datetime
from typing import Optional, List, Dict, Callable
import numpy as np
import cv2

# ===== optional IR support =====
try:
    import IR_lights_V2 as ir
except Exception:
    ir = None

# ===== capture hook (called before each detector capture) =====
_capture_hook: Optional[Callable[[str], None]] = None
def set_capture_hook(fn: Optional[Callable[[str], None]]) -> None:
    global _capture_hook
    _capture_hook = fn

# =================== Knobs ===================
INTERVAL_S     = float(os.getenv("INTERVAL_S", "2.0"))  # one capture every N s (round-robin)
WORK_W, WORK_H = 480, 270
PERCENTILE     = 90.0
AREA_THRESHOLD = 12000
QUIET_SECONDS  = 20.0
USB_REQ_SIZE   = (1280, 720)

# Per-camera tuning (env-tweakable)
PER_CAM = {
    "mipi0":  {"pct": float(os.getenv("MOTION_PCT_MIPI0", 90)), "area": int(os.getenv("MOTION_AREA_MIPI0", 1000))},
    "mipi1":  {"pct": float(os.getenv("MOTION_PCT_MIPI1", 90)), "area": int(os.getenv("MOTION_AREA_MIPI1", 1000))},
    "video0": {"pct": float(os.getenv("MOTION_PCT_VIDEO0", 90)), "area": int(os.getenv("MOTION_AREA_VIDEO0", 1000))},
}

THR_FLOOR_GLOBAL = 8.0
THR_FLOOR_PER_CAM = {
    "mipi0": float(os.getenv("MOTION_THRFLOOR_MIPI0", 5.0)),
    "mipi1": float(os.getenv("MOTION_THRFLOOR_MIPI1", 5.0)),
    "video0": float(os.getenv("MOTION_THRFLOOR_VIDEO0", 5.0)),
}

# Verbosity
DEBUG = os.environ.get("MOTION_DEBUG", "0") in ("1","true","True")

# Env feature flags
ENABLE_MIPI = os.environ.get("MOTION_ENABLE_MIPI", "1") not in ("0","false","False","off","OFF")
ENABLE_USB  = os.environ.get("MOTION_ENABLE_USB",  "1") not in ("0","false","False","off","OFF")
print(f"[motion] test loaded from: {__file__}")
if DEBUG:
    print(f"[motion] flags: ENABLE_MIPI={ENABLE_MIPI}, ENABLE_USB={ENABLE_USB}")

# =================== IR mapping + default hook ===================

# Map detector camera labels -> IR channels
CAMMAP = {
    "mipi0": "cam1",
    "mipi1": "usb0",
    # any "video*" will be treated as usb0 below
}

def _default_ir_hook(cam_label: str):
    """
    Simple IR flash per capture so test mode behaves like night motion:
      - mipi0  -> cam0
      - mipi1  -> cam1
      - video* -> usb0
    Only used if no custom hook is set via set_capture_hook().
    """
    if ir is None:
        return

    name = CAMMAP.get(cam_label)
    if name is None and cam_label.startswith("video"):
        name = "cam0"
    if not name:
        return

    try:
        ms = int(os.getenv("IR_FLASH_MS", "80"))
        ir.flash(name, ms=ms)
    except Exception as e:
        if DEBUG:
            print(f"[ir] default hook error for {name}: {e}")

# =================== OpenCV settings ===================
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

def _blob_count_and_max(mask_u8: np.ndarray):
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
    num = max(0, n - 1)
    max_area = 0 if n <= 1 else int(np.max(stats[1:, cv2.CC_STAT_AREA]))
    return num, max_area

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
        out = subprocess.run(["v4l2-ctl","-D","-d",dev],
                             capture_output=True, text=True, check=False).stdout
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

CSI_MODEL_ALLOWLIST = ("imx", "ov", "ar", "sony", "s5k", "gc", "hm", "os")

class MipiSource:
    def __init__(self, index: int, label: str):
        if Picamera2 is None:
            raise RuntimeError("Picamera2 not available")
        self.index = index
        self.name  = label
        self.cam   = Picamera2(self.index)
        cfg = self.cam.create_video_configuration(
            main={"size": (WORK_W, WORK_H), "format": "YUV420"},
            queue=False, buffer_count=2
        )
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(0.08)
        # Try to discover stride for zero-copy buffer access
        self._stride = None
        try:
            sc = self.cam.stream_configuration("main")
            if isinstance(sc, dict) and "stride" in sc:
                self._stride = int(sc["stride"])
        except Exception:
            self._stride = None

    def grab_Y(self) -> Optional[np.ndarray]:
        try:
            if self._stride:
                buf = self.cam.capture_buffer("main")
                if buf is None:
                    return None
                plane = np.frombuffer(buf, dtype=np.uint8, count=self._stride * WORK_H)\
                          .reshape(WORK_H, self._stride)
                return plane[:, :WORK_W].copy()
            else:
                arr = self.cam.capture_array("main")
                if arr is None:
                    return None
                return arr[:WORK_H, :WORK_W].copy()
        except Exception as e:
            if DEBUG:
                print(f"[motion] {self.name} grab error: {e}")
            return None

    def close(self) -> None:
        try: self.cam.stop()
        except: pass
        try: self.cam.close()
        except: pass

# =================== USB (UVC) ===================
class UsbSource:
    def __init__(self, dev: str):
        self.dev  = dev
        self.name = os.path.basename(dev)  # "video0"
        self.cap  = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        try: self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except: pass
        try: self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except: pass
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  USB_REQ_SIZE[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, USB_REQ_SIZE[1])
        for _ in range(2):
            self.cap.read()  # warmup

    def grab_Y(self) -> Optional[np.ndarray]:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return _resize_keep_width_center_crop(gray, WORK_W, WORK_H)

    def close(self) -> None:
        try: self.cap.release()
        except: pass

# =================== Visual Proof (8-panel montage) ===================
PROOF_ENABLE       = os.getenv("MOTION_PROOF", "0") in ("1","true","True","yes","on","ON")
PROOF_ON_MOTION    = os.getenv("MOTION_PROOF_ON_MOTION", "1") in ("1","true","True","yes","on","ON")
PROOF_MIN_SECS     = float(os.getenv("MOTION_PROOF_MIN_SECS", "6.0"))   # per-camera rate limit
PROOF_DIR          = os.getenv("MOTION_PROOF_DIR", "./proof")
PROOF_EXT          = os.getenv("MOTION_PROOF_EXT", "png")               # png or jpg
PROOF_JPG_QUALITY  = int(os.getenv("MOTION_PROOF_JPG_Q", "90"))
PROOF_SHOW_HIST    = False

_last_proof_ts: Dict[str, float] = {}
_last_prevY: Dict[str, np.ndarray] = {}

if PROOF_ENABLE:
    try:
        os.makedirs(PROOF_DIR, exist_ok=True)
    except Exception as _e:
        print(f"[proof] could not create dir '{PROOF_DIR}': {_e}")

def _to_bgr(img: np.ndarray) -> np.ndarray:
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img

def _fit_panel(img: np.ndarray, w: int = WORK_W, h: int = WORK_H) -> np.ndarray:
    vis = _to_bgr(img)
    if vis.dtype != np.uint8:
        vis = np.clip(vis, 0, 255).astype(np.uint8)
    if vis.shape[0] != h or vis.shape[1] != w:
        vis = cv2.resize(vis, (w, h), interpolation=cv2.INTER_AREA)
    return vis

def _panel(img: np.ndarray) -> np.ndarray:
    return _fit_panel(img)

def _blank_panel(label: str = "") -> np.ndarray:
    p = np.zeros((WORK_H, WORK_W, 3), dtype=np.uint8)
    if label:
        _text(p, label, (6,18))
    return p

def _text(img, s, xy=(6,16)):
    cv2.putText(img, s, (xy[0]+1, xy[1]+1), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0,0,0), 2, cv2.LINE_AA)
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255,255,255), 1, cv2.LINE_AA)

def _diff_hist_panel(diff_u8: np.ndarray, thr_val: float, thr_floor: float) -> np.ndarray:
    hist = cv2.calcHist([diff_u8], [0], None, [256], [0,256]).flatten()
    H0, W0 = 120, 256
    canvas = np.zeros((H0, W0, 3), dtype=np.uint8)
    hmax = max(1.0, hist.max())
    pts = []
    for x in range(256):
        y = int(round((hist[x]/hmax) * (H0-1)))
        pts.append((x, H0-1 - y))
    for i in range(1, 256):
        cv2.line(canvas, pts[i-1], pts[i], (200,200,200), 1, cv2.LINE_AA)
    tx = min(max(int(round(thr_val)), 0), 255)
    fx = min(max(int(round(thr_floor)), 0), 255)
    cv2.line(canvas, (tx,0), (tx,H0-1), (255,0,0), 1, cv2.LINE_AA)
    cv2.line(canvas, (fx,0), (fx,H0-1), (0,255,0), 1, cv2.LINE_AA)
    _text(canvas, f"diff hist | thr={thr_val:.1f} | floor={thr_floor:.1f}", (6,16))
    canvas = cv2.resize(canvas, (WORK_W, WORK_H), interpolation=cv2.INTER_AREA)
    return canvas

def _largest_bbox(clean_u8: np.ndarray):
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(clean_u8, connectivity=8)
    if n <= 1:
        return None
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x = int(stats[idx, cv2.CC_STAT_LEFT])
    y = int(stats[idx, cv2.CC_STAT_TOP])
    w = int(stats[idx, cv2.CC_STAT_WIDTH])
    h = int(stats[idx, cv2.CC_STAT_HEIGHT])
    area = int(stats[idx, cv2.CC_STAT_AREA])
    return (x,y,w,h,area)

def _overlay_mask(gray_u8: np.ndarray, mask_u8: np.ndarray, bbox=None) -> np.ndarray:
    base = _fit_panel(gray_u8)
    overlay = base.copy()
    m = _fit_panel(mask_u8)[:,:,0]
    overlay[m > 0] = (0, 0, 255)
    out = cv2.addWeighted(overlay, 0.45, base, 0.55, 0.0)
    if bbox is not None:
        x,y,w,h,area = bbox
        cv2.rectangle(out, (x,y), (x+w, y+h), (0,255,0), 2)
        _text(out, f"largest blob: {area}px", (6, WORK_H-8))
    return out

# ===== NEW: snapshots for debug printing =====
_snapshots: Dict[str, Dict] = {}
_last_name: Optional[str] = None

def debug_snapshot(name: Optional[str] = None) -> Dict:
    if not _snapshots:
        return {}
    if name is None or name not in _snapshots:
        name = _last_name or next(iter(_snapshots))
    return dict(_snapshots[name])

def metrics_line(name: Optional[str] = None) -> str:
    s = debug_snapshot(name)
    if not s:
        return ""
    m = s["metrics"]
    w = s.get("w", 0); h = s.get("h", 0)
    return (f"[md] cam={s['cam']} {w}x{h}  "
            f"abs.mean={m['abs_mean']:.1f} abs.p95={m['abs_p95']:.1f}  "
            f"thr.on={m['thr_on']} clean.on={m['clean_on']}  "
            f"blobs={m['num_blobs']} max_area={m['max_area']}  "
            f"thr={m['thr_val']:.1f} pct={m['pct']:.1f} floor={m['thr_floor']:.1f}  "
            f"motion={m['is_motion']}")

def _compose_and_save_proof(name: str,
                            prev_u8: Optional[np.ndarray],
                            Y_u8: np.ndarray,
                            base_u8: np.ndarray,
                            cur_u8: np.ndarray,
                            diff_u8: np.ndarray,
                            thr_val: float,
                            mask_u8: np.ndarray,
                            clean_u8: np.ndarray,
                            area: int,
                            pct: float,
                            thr_floor: float,
                            is_motion: bool,
                            ath: int):
    try:
        prev_vis = prev_u8 if prev_u8 is not None else Y_u8
        bbox = _largest_bbox(clean_u8)
        overlay = _overlay_mask(Y_u8, clean_u8, bbox)

        p0 = _panel(prev_vis); _text(p0, "[0] prev (Y)", (6,18))
        p1 = _panel(Y_u8);     _text(p1, "[1] current (Y)", (6,18))
        p2 = _panel(diff_u8);  _text(p2, "[2] abs diff", (6,18))
        top = cv2.hconcat([p0, p1, p2])

        p3 = _panel(mask_u8);   _text(p3, f"[3] thr ≥ {thr_val:.1f} (pct={pct:.1f}, floor={thr_floor:.1f})", (6,18))
        p4 = _panel(clean_u8);  _text(p4, "[4] cleaned", (6,18))
        p5 = _panel(overlay);   _text(p5, "[5] overlay", (6,18))
        bot = cv2.hconcat([p3, p4, p5])

        grid = cv2.vconcat([top, bot])

        header_h = 24
        header = np.zeros((header_h, grid.shape[1], 3), dtype=np.uint8)
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        meta = (f"{ts} | cam={name} | motion={is_motion} | largest_area={area} (A>{ath}) | "
                f"thr={thr_val:.1f} | pct={pct:.1f} | floor={thr_floor:.1f} | work={WORK_W}x{WORK_H}")
        _text(header, meta, (8,16))

        viz = cv2.vconcat([header, grid])

        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        base_fn = f"{stamp}_{name}_proof"
        img_path = os.path.join(PROOF_DIR, f"{base_fn}.{PROOF_EXT}")
        json_path = os.path.join(PROOF_DIR, f"{base_fn}.json")

        if PROOF_EXT.lower() == "jpg":
            cv2.imwrite(img_path, viz, [int(cv2.IMWRITE_JPEG_QUALITY), PROOF_JPG_QUALITY])
        else:
            cv2.imwrite(img_path, viz)

        with open(json_path, "w") as f:
            json.dump({
                "ts": ts,
                "camera": name,
                "motion": bool(is_motion),
                "largest_blob_area": int(area),
                "thr_val": float(thr_val),
                "pct": float(pct),
                "thr_floor": float(thr_floor),
                "work_size": [int(WORK_W), int(WORK_H)],
                "area_threshold": int(ath)
            }, f, indent=2)

        print(f"[proof] saved {img_path}")
    except Exception as e:
        print(f"[proof] compose error {name}: {e}")

def _maybe_save_proof(name: str,
                      Y: np.ndarray,
                      base_f32: np.ndarray,
                      cur_f32: np.ndarray,
                      diff_u8: np.ndarray,
                      thr_val: float,
                      mask_u8: np.ndarray,
                      clean_u8: np.ndarray,
                      area: int,
                      pct: float,
                      thr_floor: float,
                      is_motion: bool,
                      ath: int):
    if not PROOF_ENABLE:
        return
    if PROOF_ON_MOTION and not is_motion:
        return
    now = time.time()
    last = _last_proof_ts.get(name, 0.0)
    if (now - last) < PROOF_MIN_SECS:
        return
    _last_proof_ts[name] = now

    prev_u8 = _last_prevY.get(name)
    _compose_and_save_proof(
        name=name,
        prev_u8=prev_u8 if prev_u8 is not None else None,
        Y_u8=Y,
        base_u8=base_f32,
        cur_u8=cur_f32,
        diff_u8=diff_u8,
        thr_val=thr_val,
        mask_u8=mask_u8,
        clean_u8=clean_u8,
        area=area,
        pct=pct,
        thr_floor=thr_floor,
        is_motion=is_motion,
        ath=ath
    )

# =================== Detector state ===================
sources: List[object]            = []
baselines: Dict[str, np.ndarray] = {}
last_motion_ts: float            = 0.0
idx: int                         = 0
running: bool                    = False
thr: Optional[threading.Thread]  = None
lock = threading.Lock()

def discover() -> List[object]:
    found: List[object] = []

    # Open USB first
    if ENABLE_USB:
        for dev in sorted(glob.glob("/dev/video*")):
            if not _is_uvc(dev):
                if DEBUG:
                    print(f"[motion] skip non-uvc node {dev}")
                continue
            try:
                s = UsbSource(dev)
                if s.grab_Y() is None:
                    s.close()
                    continue
                found.append(s)
                print(f"[motion] USB {dev} ready")
            except Exception as e:
                print(f"[motion] USB {dev} skipped: {e}")

    # Open real CSI sensors
    if Picamera2 is not None and ENABLE_MIPI:
        try:
            info = Picamera2.global_camera_info()
            for i, ci in enumerate(info):
                model = str(ci.get("Model", "")).strip()
                model_l = model.lower()
                if DEBUG:
                    print(f"[motion] Picamera2[{i}] model='{model}'")
                if not any(tok in model_l for tok in CSI_MODEL_ALLOWLIST):
                    if DEBUG:
                        print(f"[motion] skip Picamera2[{i}] (not CSI): '{model}'")
                    continue
                try:
                    s = MipiSource(i, f"mipi{i}")
                    if s.grab_Y() is None:
                        s.close()
                        continue
                    found.append(s)
                    print(f"[motion] MIPI {i} ready ({model})")
                except Exception as e:
                    print(f"[motion] MIPI {i} skipped: {e}")
        except Exception as e:
            print(f"[motion] MIPI enumerate failed: {e}")

    if not found:
        print("[motion] no cameras found; detector idle")
    return found

def process_capture(name: str, Y: Optional[np.ndarray]) -> bool:
    global _last_name
    if Y is None:
        return False

    cur = cv2.GaussianBlur(Y, (3, 3), 0).astype(np.float32)
    base = baselines.get(name)
    if base is None:
        baselines[name] = cur
        _last_prevY[name] = Y.copy()
        return False

    cfg = PER_CAM.get(name, {})
    pct       = float(cfg.get("pct",  PERCENTILE))
    ath       = int(cfg.get("area", AREA_THRESHOLD))
    thr_floor = float(THR_FLOOR_PER_CAM.get(name, THR_FLOOR_GLOBAL))

    diff = cv2.absdiff(cur, base).astype(np.uint8)
    thr_val = max(float(np.percentile(diff, pct)), thr_floor)
    _, mask = cv2.threshold(diff, thr_val, 255, cv2.THRESH_BINARY)
    clean = _cleanup(mask)
    num_blobs, max_area = _blob_count_and_max(clean)

    is_motion = max_area > ath

    bbox = _largest_bbox(clean)
    overlay = _overlay_mask(Y, clean, bbox)
    abs_mean = float(diff.mean())
    abs_p95  = float(np.percentile(diff, 95))
    thr_on   = int((mask  > 0).sum())
    clean_on = int((clean > 0).sum())

    _snapshots[name] = {
        "cam": name, "w": WORK_W, "h": WORK_H,
        "frame": Y, "absdiff": diff, "thresh": mask, "clean": clean, "overlay": overlay,
        "metrics": {
            "abs_mean": abs_mean, "abs_p95": abs_p95,
            "thr_on": thr_on, "clean_on": clean_on,
            "num_blobs": num_blobs, "max_area": max_area,
            "thr_val": thr_val, "pct": pct, "thr_floor": thr_floor,
            "is_motion": bool(is_motion), "A_thresh": ath,
        },
    }
    _last_name = name

    _maybe_save_proof(
        name=name, Y=Y, base_f32=base, cur_f32=cur,
        diff_u8=diff, thr_val=thr_val, mask_u8=mask, clean_u8=clean,
        area=max_area, pct=pct, thr_floor=thr_floor,
        is_motion=is_motion, ath=ath
    )

    baselines[name] = cur
    _last_prevY[name] = Y.copy()

    if DEBUG:
        print(f"[motion] {name} area={max_area} thr={thr_val:.1f} pct={pct} A>{ath}")

    return is_motion

def rotation_loop():
    """One capture every INTERVAL_S seconds across all sources."""
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

            # IR / hook behavior:
            # - If caller set a hook via set_capture_hook, use that.
            # - Else, use default IR flash so this behaves like night mode visually.
            if _capture_hook is not None:
                try:
                    _capture_hook(name)
                except Exception as e:
                    if DEBUG:
                        print(f"[motion] hook err: {e}")
            else:
                _default_ir_hook(name)

            t0 = time.perf_counter()
            try:
                Y = s.grab_Y()
                if process_capture(name, Y):
                    with lock:
                        last_motion_ts = time.time()
                    if DEBUG:
                        print(f"[motion] hit: {name}")
            except Exception as e:
                if DEBUG:
                    print(f"[motion] capture err on {name}: {e}")
            t1 = time.perf_counter()
            if DEBUG:
                print(f"[motion] {name} capture+proc {(t1-t0)*1000:.1f} ms")

            next_tick += INTERVAL_S
            time.sleep(max(0.0, next_tick - time.perf_counter()))
    finally:
        for s in sources:
            try: s.close()
            except: pass

# =================== Public API ===================
def start() -> None:
    global running, thr, sources
    with lock:
        if running:
            return
        sources = discover()
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
    print("[motion] detector stopped")

def motion_active() -> bool:
    with lock:
        ts = last_motion_ts
    return (time.time() - ts) <= 2

def camera_names() -> List[str]:
    return [getattr(s, "name", "src") for s in sources]

if __name__ == "__main__":
    try:
        start()
        while True:
            print("motion:", motion_active())
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
