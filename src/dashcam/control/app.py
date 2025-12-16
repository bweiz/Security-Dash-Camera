#!/usr/bin/env python3
# proc_flow_V14.py — wrapper with IR cooldown, motion reseed, and safe device handoff
# - STREAM_ON/OFF
# - Defers MediaMTX restart until stream stops 
# - Uses motion.capture_reserve()/release() to block BOTH USB & CSI during STREAM_ON/OFF
# - Uses motion.csi_reserve()/release() specifically for MIPI
# - Adds busy retry for start_queue()

import os, socket, threading, time, subprocess
from typing import Set
from gpiozero import DigitalInputDevice, OutputDevice
from pathlib import Path


# ---------- Recording dir ----------
RECORD_DIR = Path(os.getenv("RECORD_DIR", "/mnt/dashcam/recordings"))
RECORD_DIR.mkdir(parents=True, exist_ok=True)

# ---------- Modules ----------
import IR_lights_V2 as ir
import motion_detection_V11 as motion
from Full_Proc_V5 import FullProc, SEG_SECONDS

# ---------- MediaMTX ----------
def restart_mediamtx():
    subprocess.Popen(
        ["/usr/bin/sudo", "/bin/systemctl", "restart", "mediamtx.service"],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
    )

def wait_for_mediamtx(port=8554, timeout=6.0):
    end = time.time() + timeout
    while time.time() < end:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False
# ----------- VF adjustments for light -----------------------------------
def _set_storage_vf_for_light(state):
    """Choose storage filter based on ambient light. Night => boost, Day => none."""
    if camera_dark(state):
        os.environ["STORAGE_VF"] = os.getenv("NIGHT_STORAGE_VF", "eq=brightness=0.3:contrast=1.5")
    else:
        os.environ["STORAGE_VF"] = "null"

# ====================== SWITCHES: GPIO-BRIDGE MODE ======================
RECORD_SRC, RECORD_SNS = 5, 6
MOTION_SRC, MOTION_SNS = 16, 20
POLL_SEC = 0.2
CLOSED_IS_ON = os.getenv("SWITCH_CLOSED_ON", "1") not in ("0","false","False")

def make_source_high(pin: int) -> OutputDevice:
    return OutputDevice(pin=pin, active_high=True, initial_value=True)
def make_sense(pin: int, bounce=0.05) -> DigitalInputDevice:
    return DigitalInputDevice(pin=pin, pull_up=False, bounce_time=bounce)

class GpioBridgeSwitch:
    def __init__(self, src_pin: int, sns_pin: int, closed_is_on: bool = True, bounce=0.05):
        self.src = make_source_high(src_pin)
        self.sns = make_sense(sns_pin, bounce)
        self.closed_is_on = closed_is_on
    def is_on(self) -> bool:
        v = self.sns.is_active
        return v if self.closed_is_on else (not v)

# ===================== CAMERA-ONLY DARKNESS POLICY =====================
CAMERA_DARK_STALE_S = float(os.getenv("IR_DARK_STALE_S", "120"))
HOLD_ON_STALE       = os.getenv("IR_HOLD_ON_STALE", "1").lower() in ("1","true","yes","on")

def camera_dark(state_dict) -> bool:
    """Return dark boolean using camera EMA, quiet (no prints)."""
    force = os.getenv("IR_FORCE")
    if force is not None:
        return force.lower() in ("1","on","true","yes")
    try:
        ds = motion.dark_state()
        ema = ds.get("ema"); ts = ds.get("ts") or 0.0
        age = time.time() - ts
        if ema is not None and age <= CAMERA_DARK_STALE_S:
            val = bool(ds.get("is_dark", False))
            state_dict["_dark_cache"] = (val, "")
            return val
        if HOLD_ON_STALE and "_dark_cache" in state_dict:
            return bool(state_dict["_dark_cache"][0])
        return False
    except Exception:
        return False

# ===================== RTSP URL HELPERS =====================
SOCKET_PATH = "/tmp/streamctl.sock"
def get_pi_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80)); ip = s.getsockname()[0]; s.close(); return ip
    except Exception:
        return "127.0.0.1"
RTSP_URL_ROOT_CLIENT = f"rtsp://{get_pi_ip()}:8554"
def rtsp_url_for(name: str) -> str: return f"{RTSP_URL_ROOT_CLIENT}/{name}"
RTSP_URL_ROOT_LOCAL = "rtsp://127.0.0.1:8554"
def rtsp_publish_url(name: str) -> str: return f"{RTSP_URL_ROOT_LOCAL}/{name}"
os.environ["RTSP_URL_ROOT_CLIENT"] = RTSP_URL_ROOT_CLIENT
os.environ["RTSP_URL_ROOT_LOCAL"]  = RTSP_URL_ROOT_LOCAL

# ===================== MIPI helpers =====================
MIPI_REUSE_DELAY_S = float(os.getenv("MIPI_REUSE_DELAY_S", "1.2"))
def _is_mipi(cam: str) -> bool: return cam in ("cam0", "cam1")

# ---------- simple UDS control server ----------
class StreamCtlServer:
    def __init__(self, svc: FullProc, state):
        self.svc  = svc
        self.state = state
        self._thr = None
        self._stop = False

    # safe send to avoid BrokenPipe noise
    def _safe_send(self, conn, b: bytes):
        try: conn.sendall(b)
        except BrokenPipeError: pass

    def start(self):
        try: os.unlink(SOCKET_PATH)
        except FileNotFoundError: pass
        self._stop = False
        self._thr = threading.Thread(target=self._serve, daemon=True)
        self._thr.start()
        print(f"[streamctl] listening on {SOCKET_PATH}")

    def stop(self):
        self._stop = True
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                s.connect(SOCKET_PATH); s.sendall(b"QUIT\n")
        except Exception: pass
        try: os.unlink(SOCKET_PATH)
        except Exception: pass

    def _serve(self):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as srv:
            srv.bind(SOCKET_PATH)
            try: os.chmod(SOCKET_PATH, 0o666)
            except Exception: pass
            srv.listen(4)
            while not self._stop:
                try:
                    srv.settimeout(0.5)
                    conn, _ = srv.accept()
                except socket.timeout:
                    continue
                threading.Thread(target=self._handle, args=(conn,), daemon=True).start()

    def _handle(self, conn: socket.socket):
        with conn:
            try: data = conn.recv(1024).decode("utf-8","replace").strip()
            except Exception: return
            if not data: return
            if data == "QUIT": self._safe_send(conn, b"OK\n"); return
            if data == "PING": self._safe_send(conn, b"PONG\n"); return

            parts = data.split()
            if len(parts)!=2 or parts[0] not in ("STREAM_ON","STREAM_OFF"):
                self._safe_send(conn, b"ERR: bad command\n"); return
            cmd, cam = parts
            if cam not in ("cam0","cam1","usb0"):
                self._safe_send(conn, b"ERR: unknown cam\n"); return

            try:
                if cmd == "STREAM_ON":
                    # if already streaming, don't restart anything
                    if cam in self.state["streaming_cams"]:
                        print(f"[INFO] {cam}: stream already running")
                        self._safe_send(conn, b"OK\n"); return

                    # Reserve ALL capture (USB+CSI) so detector/dark-meter cannot rearm during bring-up
                    try: motion.capture_reserve()
                    except Exception as e: print(f"[streamctl] capture_reserve error: {e}")

                    # Ensure detector/dark meter are fully down before opening anything
                    self._stop_motion_blocking(wait_s=1.0)

                    # If opening a MIPI, stop the other MIPI first and reserve CSI
                    if _is_mipi(cam):
                        try: motion.csi_reserve()
                        except Exception as e: print(f"[streamctl] csi_reserve error: {e}")
                        for c in list(self.state["streaming_cams"]):
                            if c != cam and _is_mipi(c):
                                try: self.svc.stop_queue(c)
                                except Exception as e: print(f"[streamctl] stop_queue({c}) error: {e}")
                                self.state["streaming_cams"].discard(c)
                        time.sleep(MIPI_REUSE_DELAY_S)

                    # Start requested stream with one retry on "busy"
                    try:
                        self._start_queue_with_retry(cam)
                    except Exception as e:
                        # On failure, release reservations we took
                        if _is_mipi(cam):
                            try: motion.csi_release()
                            except Exception: pass
                        try: motion.capture_release()
                        except Exception: pass
                        self._safe_send(conn, f"ERR: {e}\n".encode("utf-8","replace")); return

                    self.state["streaming_cams"].add(cam)
                    print(f"[streamctl] STREAM_ON {cam} -> open {rtsp_url_for(cam)}")
                    self._safe_send(conn, b"OK\n"); return

                if cmd == "STREAM_OFF":
                    if cam not in self.state["streaming_cams"]:
                        print(f"[INFO] {cam}: stream not running")
                        self._safe_send(conn, b"OK\n"); return

                    # Stop the specific stream
                    self.svc.stop_queue(cam)
                    self.state["streaming_cams"].discard(cam)
                    print(f"[streamctl] STREAM_OFF {cam}")
                    self._safe_send(conn, b"OK\n")

                    # Give libcamera/uvc a moment to release
                    time.sleep(MIPI_REUSE_DELAY_S)

                    # Release CSI reservation if no MIPI stream remains
                    if not any(_is_mipi(c) for c in self.state["streaming_cams"]):
                        try: motion.csi_release()
                        except Exception as e: print(f"[streamctl] csi_release error: {e}")

                    # If no streams at all, do *one* MediaMTX restart and fully re-arm environment
                    if not self.state["streaming_cams"]:
                        # Restart MediaMTX only when *last* stream is gone
                        restart_mediamtx()
                        if wait_for_mediamtx():
                            print("MediaMTX is back online!")
                        else:
                            print("Still restarting, continuing anyway…")

                        # Release global capture reservation and re-enable dark meter
                        try: motion.capture_release()
                        except Exception as e: print(f"[streamctl] capture_release error: {e}")

                        try:
                            motion.reseed_baselines(2)
                            motion.reset_last_motion(0.0)
                        except Exception as e:
                            print(f"[streamctl] reseed/reset error: {e}")

                        if not self.state.get("recording", False):
                            try: motion.dark_meter_start()
                            except Exception as e:
                                print(f"[streamctl] dark_meter_start error: {e}")
                    return
            except Exception as e:
                self._safe_send(conn, f"ERR: {e}\n".encode("utf-8","replace")); return

    def _start_queue_with_retry(self, cam: str, retries: int = 2):
        try:
            self.svc.start_queue(cam)
        except Exception as e:
            msg = (str(e) or "").lower()
            if ("busy" in msg or "resource" in msg or "device" in msg) and retries > 0:
                print(f"[streamctl] start_queue({cam}) busy; retry after {MIPI_REUSE_DELAY_S}s")
                time.sleep(MIPI_REUSE_DELAY_S)
                self.svc.start_queue(cam)
            else:
                raise

    def _stop_motion_blocking(self, wait_s: float = 1.0):
        # Stop detector if running
        if self.state.get("detector_running"):
            try: motion.stop()
            except Exception as e:
                print(f"[streamctl] motion.stop error: {e}")
            self.state["detector_running"] = False

        # Stop dark meter (can hold CSI/USB briefly)
        try:
            motion.dark_meter_stop(block=True)
        except Exception as e:
            print(f"[streamctl] dark_meter_stop error: {e}")

        # If motion exposes a blocking wait, use it; otherwise sleep a bit
        try:
            motion.wait_stopped(wait_s)
        except Exception:
            time.sleep(wait_s)

        # final grace for device reuse
        time.sleep(MIPI_REUSE_DELAY_S)

# ===================== SHUTDOWN VIA SWITCH COMBO =====================
SHUTDOWN_HOLD_S = float(os.getenv("SHUTDOWN_HOLD_S", "3"))
def request_poweroff():
    print("[shutdown] requesting system poweroff...")
    try: subprocess.run(["systemctl","poweroff"], check=False)
    except Exception:
        try: subprocess.run(["sudo","/sbin/shutdown","-h","now"], check=False)
        except Exception:
            try: subprocess.run(["sudo","poweroff"], check=False)
            except Exception as e:
                print(f"[shutdown] failed to exec poweroff: {e}")

def graceful_cleanup(svc: FullProc, state, ctl: StreamCtlServer):
    try: ir.all_off()
    except: pass
    if state.get("recording"):
        try: 
            svc.stop_record() 
        except: 
            pass
        state["ir_full_until"] = 0.0
        state["recording"] = False; state["started_by"] = None
    try: motion.dark_meter_stop(block=True)
    except: pass
    if state.get("detector_running"):
        try: motion.stop()
        except: pass
        state["detector_running"] = False
    try: ctl.stop()
    except: pass
    try: motion.capture_release()
    except: pass
    try: os.sync()
    except: pass

# ===================== IR helpers =====================
IR_STREAM_FULL = os.getenv("IR_STREAM_FULL", "0").lower() in ("1","true","yes","on")
IR_STREAM_LP_DUTY = float(os.getenv("IR_STREAM_LP_DUTY", "0.75"))
IR_RECORD_LP_DUTY = float(os.getenv("IR_RECORD_LP_DUTY", "0.75"))  # LP duty while recording
IR_FULL_CAP_S     = float(os.getenv("IR_FULL_CAP_S", "180"))        # max seconds of full IR per recording

def _ir_for_motion_with_cooldown(state):
    """Full IR unless within 3 min of last motion stop; then low power (0.75)."""
    if not camera_dark(state):
        if state.get("ir_locked"): ir.all_off(); state["ir_locked"] = False
        try: motion.set_force_profile("")
        except AttributeError: pass
        return
    if time.monotonic() < state.get("cooldown_until", 0.0):
        ir.all_on_lp(IR_RECORD_LP_DUTY)
        state["_ir_mode"] = "lp"
    else:
        ir.all_on()
        state["_ir_mode"] = "full"
    state["ir_locked"] = True
    try: motion.set_force_profile("night")
    except AttributeError: pass

def _ir_for_streaming(state):
    """IR policy while streaming: follow ambient; choose full or LP."""
    if not camera_dark(state):
        if state.get("ir_locked"):
            ir.all_off(); state["ir_locked"] = False
        try: motion.set_force_profile("")
        except AttributeError: pass
        return
    if IR_STREAM_FULL:
        ir.all_on()
    else:
        ir.all_on_lp(IR_STREAM_LP_DUTY)
    state["ir_locked"] = True
    try: motion.set_force_profile("")
    except AttributeError: pass

# ===================== MAIN LOOP =====================
def main():
    rec_sw = GpioBridgeSwitch(RECORD_SRC, RECORD_SNS, closed_is_on=CLOSED_IS_ON, bounce=0.05)
    mot_sw = GpioBridgeSwitch(MOTION_SRC, MOTION_SNS, closed_is_on=CLOSED_IS_ON, bounce=0.05)
    
    # NOTE: adjust indices/device to your rig
    svc = FullProc(mipi_indices=[0, 1],
                   usb_dev="/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._USB_Camera_SN0001-video-index0")

    state = {
        "recording": False,
        "started_by": None,           # "record" | "motion" | None
        "detector_running": False,
        "streaming_cams": set(),      # type: Set[str]
        "ir_locked": False,
        "_dark_cache": (False, ""),
        "_shutdown_armed": False,
        "_shutdown_deadline": 0.0,
        "record_started_at": 0.0,
        "cooldown_until": 0.0,        # IR LP for 3 minutes after a motion stop
        "ir_full_until":0.0
    }


    

    # Map detector camera labels - IR channel names
    CAMMAP = {"mipi0": "cam1", "mipi1": "usb0"}   

    # Flash width + boot warm-up window 
    IR_FLASH_MS       = int(os.getenv("IR_FLASH_MS", "120"))       
    IR_BOOT_FORCE_S   = float(os.getenv("IR_BOOT_FORCE_S", "8.0"))  # force-flash window after start
    IR_FLASH_GAP_S    = float(os.getenv("IR_FLASH_GAP_S", "0.8"))   # min time between flashes per channel
    _BOOT_FORCE_END   = time.monotonic() + IR_BOOT_FORCE_S
    _last_flash_ts    = {}  # per IR channel throttle

    def on_motion_capture(cam_label: str):
        # Robust mapping: mipi* to cam{0,1}, any "video*" to usb0
        name = CAMMAP.get(cam_label)
        if name is None and cam_label.startswith("video"):
            name = "cam0"
        if not name:
            return

        now = time.monotonic()
        if now - _last_flash_ts.get(name, 0.0) < IR_FLASH_GAP_S:
            return

        # For a brief window after start, ALWAYS flash (ignore darkness/recording gates)
        if now < _BOOT_FORCE_END:
            try:
                ir.set(name, 1.0)
                time.sleep(IR_FLASH_MS / 1000.0)
            finally:
                ir.set(name, 0.0)
                _last_flash_ts[name] = now
            return

        # Normal behavior after warm-up
        if state["recording"]:
            return

        # Use dark state, but allow a "probing" flash
        ds   = motion.dark_state()
        ema  = ds.get("ema")
        ts   = ds.get("ts") or 0.0
        age  = time.time() - ts
        is_dark = bool(ds.get("is_dark", False))

        # If it's dark OR we don't have a fresh/known reading yet, flash to seed the baseline.
        if is_dark or ema is None or age > CAMERA_DARK_STALE_S:
            ir.flash(name, ms=IR_FLASH_MS)
            _last_flash_ts[name] = now
            try: motion.set_force_profile("night")
            except AttributeError: pass
            return
        # else: bright and fresh → no flash
        try: motion.set_force_profile("night")
        except AttributeError: pass


    motion.set_capture_hook(on_motion_capture)


    ctl = StreamCtlServer(svc, state)
    ctl.start()

    print("[rtsp] client URLs (open these in VLC / your app):")
    for cam in ("cam0", "cam1", "usb0"):
        print(f"   {cam}: {rtsp_url_for(cam)}")
    print(f"[wiring] RECORD(src,sns)=({RECORD_SRC},{RECORD_SNS})  MOTION(src,sns)=({MOTION_SRC},{MOTION_SNS})")
    print(f"[ir] pins={getattr(ir, 'PINS', {})}  dark=CAMERA_ONLY")
    ir.all_off(); print("[ir] init -> all_off()")
    print("[wrapper] Ctrl+C to exit.  (Hold BOTH switches for "
          f"{int(SHUTDOWN_HOLD_S)}s to power off)")

    try:
        ir.prime(start_flash_ms=int(os.getenv("IR_PRIME_MS", "80")))
        # Start dark meter
        motion.dark_meter_start()

        while True:
            rec_on = rec_sw.is_on()
            mot_on = mot_sw.is_on()
            streaming = len(state["streaming_cams"]) > 0

            # Gate on global capture reservation to prevent re-arming during STREAM_ON bring-up
            capture_busy = False
            try: capture_busy = motion.capture_is_reserved()
            except Exception: pass

            # ===== Manage detector vs. dark meter (block to avoid races) =====
            want_detector = (not state["recording"] and not streaming and not capture_busy and mot_on)
            want_meter    = (not state["recording"] and not streaming and not capture_busy and not mot_on)

            if want_detector and not state["detector_running"]:
                motion.dark_meter_stop(block=True)
                try:
                    motion.start()
                    ir.prime(start_flash_ms=int(os.getenv("IR_PRIME_MS", "80")))
                    print("[wrapper] detector ARMED")
                except Exception as e:
                    print(f"[wrapper] motion start error: {e}")
                else:
                    state["detector_running"] = True
            elif state["detector_running"] and not want_detector:
                try: motion.stop()
                except: pass
                state["detector_running"] = False
                print("[wrapper] detector DISARMED")

            if want_meter and not motion.dark_meter_running():
                motion.dark_meter_start()
            elif not want_meter and motion.dark_meter_running():
                motion.dark_meter_stop(block=True)

            # ===== React to motion hits -> start recording =====
            if mot_on and state["detector_running"] and not state["recording"] and not streaming:
                try:
                    if motion.motion_active():
                   
                        
                        print("[wrapper] MOTION HIT -> start recording")
                        try: motion.stop()
                        except: pass
                        state["detector_running"] = False
                        motion.dark_meter_stop(block=True)
                        try:
                            _set_storage_vf_for_light(state)
                            svc.start_record()
                            state["ir_full_until"] = time.monotonic() + IR_FULL_CAP_S
                            state["recording"]         = True
                            state["started_by"]        = "motion"
                            state["record_started_at"] = time.monotonic()
                            _ir_for_motion_with_cooldown(state)
                        except Exception as e:
                            print(f"[wrapper] start_record error: {e}")
                except Exception as e:
                    print(f"[wrapper] motion check error: {e}")

            # ===== IR policy while recording =====
            if state["recording"]:
                if camera_dark(state):
                    if not state["ir_locked"]:
                        _ir_for_motion_with_cooldown(state)
                    if time.monotonic() > state.get("ir_full_until", 0.0) and state.get("_ir_mode") == "full":
                        ir.all_on_lp(IR_RECORD_LP_DUTY)
                        state["_ir_mode"] = "lp"
                        state["ir_locked"] = True
                else:
                    if state["ir_locked"]:
                        ir.all_off(); state["ir_locked"] = False
            else:
                if state["ir_locked"]:
                    ir.all_off(); state["ir_locked"] = False

            # ===== IR policy while streaming (not recording) =====
            if streaming and not state["recording"]:
                _ir_for_streaming(state)

            # ===== RECORD switch priority =====
            if rec_on:
                if not state["recording"]:
                    print("[wrapper] RECORD ON -> start recording")
                    if state["detector_running"]:
                        try: motion.stop()
                        except: pass
                        state["detector_running"] = False
                    motion.dark_meter_stop(block=True)
                    try:
                        _set_storage_vf_for_light(state)
                        svc.start_record()
                        state["ir_full_until"] = time.monotonic() + IR_FULL_CAP_S
                        state["recording"]         = True
                        state["started_by"]        = "record"
                        state["record_started_at"] = time.monotonic()
                       
                        if camera_dark(state):
                            ir.all_on(); 
                            state["ir_locked"] = True
                            state["_ir_mode"] = "full"
                        else:
                            if state["ir_locked"]:
                                ir.all_off(); 
                                state["ir_locked"] = False
                    except Exception as e:
                        print(f"[wrapper] start_record error: {e}")
                else:
                    if state["started_by"] != "record":
                        state["started_by"] = "record"
            else:
                if state["recording"] and state["started_by"] == "record":
                    print("[wrapper] RECORD OFF -> stop recording")
                    try:
                        svc.stop_record()
                        state["ir_full_until"] = 0.0
                    finally:
                        try:
                            # wait at least 1s, at most 3s, or SEG_SECONDS if smaller
                            time.sleep(min(max(1.0, SEG_SECONDS), 3.0))
                        except Exception:
                            pass
                        try:
                            os.sync()
                        except Exception:
                            pass
                    state["recording"]  = False
                    state["started_by"] = None
                    if state["ir_locked"]:
                        ir.all_off(); state["ir_locked"] = False

            # If started by MOTION, stop when MOTION switch turns OFF
            if state["recording"] and state["started_by"] == "motion" and not mot_on:
                print("[wrapper] MOTION OFF -> stop recording")
                try: 
                    svc.stop_record()
                except: 
                    pass
                state["ir_full_until"] = 0.0
                state["recording"]  = False
                state["started_by"] = None
                state["cooldown_until"] = time.monotonic() + 180.0  # 3-min LP cooldown
                if state["ir_locked"]:
                    ir.all_off(); state["ir_locked"] = False
                try: motion.reseed_baselines(1)
                except: pass
                try: motion.reset_last_motion(0.0)
                except: pass
                time.sleep(0.5)

            # stop after exactly one segment if started by MOTION
            if state["recording"] and state["started_by"] == "motion":
                if time.monotonic() - state["record_started_at"] >= SEG_SECONDS:
                    print("[wrapper] segment complete -> stop & re-arm motion")
                    try: 
                        svc.stop_record() 
                    except: 
                        pass
                    state["ir_full_until"] = 0.0
                    state["recording"]  = False
                    state["started_by"] = None
                    state["cooldown_until"] = time.monotonic() + 180.0
                    if state["ir_locked"]:
                        ir.all_off(); state["ir_locked"] = False
                    try: motion.reseed_baselines(1)
                    except: pass
                    try: motion.reset_last_motion(0.0)
                    except: pass
                    time.sleep(0.5)

            # ===== BOTH SWITCHES -> shutdown after hold =====
            both_on = rec_on and mot_on
            now = time.time()
            if both_on:
                if not state["_shutdown_armed"]:
                    state["_shutdown_armed"] = True
                    state["_shutdown_deadline"] = now + SHUTDOWN_HOLD_S
                    print(f"[shutdown] both switches ON -> hold for {int(SHUTDOWN_HOLD_S)}s to power off")
                else:
                    if now >= state["_shutdown_deadline"]:
                        print("[shutdown] hold confirmed -> powering off")
                        graceful_cleanup(svc, state, ctl)
                        request_poweroff()
                        time.sleep(0.5)
                        break
            else:
                if state["_shutdown_armed"]:
                    print("[shutdown] cancelled (switches changed)")
                    state["_shutdown_armed"] = False

            time.sleep(POLL_SEC)

    except KeyboardInterrupt:
        pass
    finally:
        try: motion.dark_meter_stop(block=True)
        except: pass
        try: ctl.stop()
        except: pass
        if state.get("recording"):
            try: 
                svc.stop_record() 
            except: 
                pass
            
        if state.get("detector_running"):
            try: motion.stop()
            except: pass
        try: motion.capture_release()
        except: pass
        try: ir.all_off()
        except: pass
        print("[wrapper] bye")

if __name__ == "__main__":
    main()
