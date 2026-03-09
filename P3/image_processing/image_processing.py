import os
import subprocess
import time
import sys
from multiprocessing import shared_memory
import warnings
warnings.filterwarnings("ignore", category=UserWarning,
module="multiprocessing")
from multiprocessing.resource_tracker import unregister
import numpy as np
import cv2

# ─── CONFIG ───────────────────────────────────────────────────────────────────
SHM_NAME      = "camera_shm_hd"
WIDTH, HEIGHT, CHANNELS = 1280, 720, 4
SENDER_PATH   = "/home/marjoe/TSBB18/P3/image_processing/image_sender.py"
SYSTEM_PYTHON = "/usr/bin/python3"

COLOR_RANGES = {
    "Red":    [(np.array([  0, 150,  50]), np.array([ 10, 255, 255])),
               (np.array([170, 150,  50]), np.array([180, 255, 255]))],
    "Blue":   [(np.array([100, 200, 150]), np.array([140, 255, 255]))],
    "Green":  [(np.array([ 35,  50,  50]), np.array([ 90, 255, 255]))],
    "Yellow": [(np.array([ 20, 170, 150]), np.array([ 35, 255, 255]))],
}
MIN_CONTOUR_AREA = 400

# ─── GLOBALS ──────────────────────────────────────────────────────────────────
_sender_process = None
_shm            = None
_frame_buffer   = None


# ─── CAMERA ───────────────────────────────────────────────────────────────────
def start_camera():
    global _sender_process, _shm, _frame_buffer
    print("Launching Camera Sender...")
    _sender_process = subprocess.Popen([SYSTEM_PYTHON, SENDER_PATH])
    while True:
        try:
            _shm = shared_memory.SharedMemory(name=SHM_NAME)
            try: unregister(_shm.name, "shared_memory")
            except KeyError: pass
            print("Connected to shared memory.")
            break
        except FileNotFoundError:
            if _sender_process.poll() is not None:
                print("Camera sender exited unexpectedly.")
                sys.exit(1)
            time.sleep(0.5)
    _frame_buffer = np.ndarray((HEIGHT, WIDTH, CHANNELS), dtype=np.uint8, buffer=_shm.buf)


def stop_camera():
    global _sender_process, _shm, _frame_buffer
    if _shm:
        try:
            from multiprocessing.resource_tracker import unregister
            unregister(f"/{_shm.name}", "shared_memory")
        except Exception:
            pass
        _shm.close()
        _shm = None
    _frame_buffer = None
    if _sender_process and _sender_process.poll() is None:
        _sender_process.terminate()
        _sender_process = None
    print("Camera stopped.")


def get_frame():
    """Return a copy of the current frame as a BGR image."""
    if _frame_buffer is None:
        raise RuntimeError("Call start_camera() first.")
    return cv2.cvtColor(_frame_buffer.copy(), cv2.COLOR_BGRA2BGR)


# ─── DETECTION ────────────────────────────────────────────────────────────────
def detect_bricks():
    bgr_frame = get_frame()
    hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
    detections = []
    for color_name, ranges in COLOR_RANGES.items():
        mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv_frame, lower, upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            detections.append({"color": color_name, "pixel": (cx, cy)})
    return detections


# ─── STANDALONE VIEWER ────────────────────────────────────────────────────────
if __name__ == "__main__":
    start_camera()
    print("Live feed running. Press 'q' to quit.")
    try:
        while True:
            display = get_frame()
            for b in detect_bricks():
                cx, cy = b["pixel"]
                cv2.circle(display, (cx, cy), 8, (0, 255, 0), -1)
                cv2.putText(display, b["color"], (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Lego Detector", display)
            if cv2.waitKey(10) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("^C detected.")
    finally:
        cv2.destroyAllWindows()
        stop_camera()
