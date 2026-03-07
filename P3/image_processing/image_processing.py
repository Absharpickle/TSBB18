import os
import json
import subprocess
import time
import sys
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister
import numpy as np
import cv2

# ─── CONFIG ───────────────────────────────────────────────────────────────────

SHM_NAME      = "camera_shm_hd"
WIDTH, HEIGHT, CHANNELS = 1280, 720, 4
SENDER_PATH   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "image_sender.py")
SYSTEM_PYTHON = "/usr/bin/python3"
CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "homography.json")

COLOR_RANGES = {
    "Red":    [( np.array([  0, 150,  50]), np.array([ 10, 255, 255]) ),
               ( np.array([170, 150,  50]), np.array([180, 255, 255]) )],
    "Blue":   [( np.array([100, 200, 150]), np.array([140, 255, 255]) )],
    "Green":  [( np.array([ 35,  50,  50]), np.array([ 90, 255, 255]) )],
    "Yellow": [( np.array([ 20, 170, 150]), np.array([ 35, 255, 255]) )],
}

MIN_CONTOUR_AREA = 5000

# ─── GLOBALS ──────────────────────────────────────────────────────────────────

_sender_process = None
_shm            = None
_frame_buffer   = None

# ─── HOMOGRAPHY ───────────────────────────────────────────────────────────────

def _load_homography():
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE) as f:
            data = json.load(f)
        print("Homography loaded from calibration file.")
        return np.array(data["H"], dtype=np.float32)
    else:
        print("WARNING: No calibration file found — using hardcoded homography.")
        sys.exit(1)

_H = _load_homography()


def pixel_to_world(px, py):
    """Convert a pixel coordinate to world (X, Y) in metres using the homography."""
    pt = np.array([[[px, py]]], dtype=np.float32)
    world = cv2.perspectiveTransform(pt, _H)
    return float(world[0][0][0]), float(world[0][0][1])


# ─── CAMERA CONTROL ───────────────────────────────────────────────────────────

def start_camera():
    """Launch image_sender.py and connect to shared memory."""
    global _sender_process, _shm, _frame_buffer

    print(f"Launching Camera Sender: {SENDER_PATH}")
    _sender_process = subprocess.Popen([SYSTEM_PYTHON, SENDER_PATH])

    _shm = None
    while True:
        try:
            _shm = shared_memory.SharedMemory(name=SHM_NAME)
            try:
                unregister(_shm.name, "shared_memory")
            except KeyError:
                pass
            print(f"Connected to Shared Memory '{SHM_NAME}'.")
            break
        except FileNotFoundError:
            if _sender_process.poll() is not None:
                print("Camera sender exited unexpectedly.")
                sys.exit(1)
            time.sleep(0.5)

    _frame_buffer = np.ndarray(
        (HEIGHT, WIDTH, CHANNELS), dtype=np.uint8, buffer=_shm.buf
    )


def stop_camera():
    """Terminate sender and release shared memory."""
    global _sender_process, _shm, _frame_buffer

    if _shm is not None:
        _shm.close()
        _shm = None
    _frame_buffer = None

    if _sender_process is not None and _sender_process.poll() is None:
        _sender_process.terminate()
        _sender_process = None

    print("Camera stopped.")


# ─── DETECTION ────────────────────────────────────────────────────────────────

def detect_bricks():
    """
    Capture one frame and return detected bricks as a list of dicts:
        [{"color": "Red", "pixel": (cx, cy), "world": (X, Y)}, ...]
    """
    if _frame_buffer is None:
        raise RuntimeError("Camera not started — call start_camera() first.")

    frame     = _frame_buffer.copy()
    bgr_frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
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
            wx, wy = pixel_to_world(cx, cy)

            detections.append({
                "color": color_name,
                "pixel": (cx, cy),
                "world": (wx, wy),
            })

    return detections


# ─── DEBUG / STANDALONE ───────────────────────────────────────────────────────

if __name__ == "__main__":
    start_camera()
    print("Detector running. Press 'q' to quit.")
    try:
        while True:
            frame    = _frame_buffer.copy()
            display  = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            bricks   = detect_bricks()

            for b in bricks:
                cx, cy   = b["pixel"]
                wx, wy   = b["world"]
                label    = f"{b['color']} ({wx:.2f}, {wy:.2f})"
                cv2.circle(display, (cx, cy), 8, (0, 255, 0), -1)
                cv2.putText(display, label, (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Lego Detector", display)
            if cv2.waitKey(10) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("^C detected.")
    finally:
        cv2.destroyAllWindows()
        stop_camera()
