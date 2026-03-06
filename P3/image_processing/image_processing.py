import time
import sys
import subprocess
import os
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister
import numpy as np
import cv2

# --- CONFIGURATION ---
SHM_NAME      = "camera_shm_hd"
WIDTH         = 1280
HEIGHT        = 720
CHANNELS      = 4  # BGRA — XRGB8888 from PiCamera2

SENDER_PATH   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "image_sender.py")
SYSTEM_PYTHON = "/usr/bin/python3"

# --- HSV COLOR RANGES (tune per your lighting conditions) ---
COLOR_RANGES = {
    "Red": [
        (np.array([0,   120,  50]), np.array([10,  255, 255])),
        (np.array([170, 120,  50]), np.array([180, 255, 255]))
    ],
    "Blue":   [(np.array([100, 150, 100]), np.array([140, 255, 255]))],
    "Green":  [(np.array([35,   50,  50]), np.array([90,  255, 255]))],
    "Yellow": [(np.array([20,  140, 100]), np.array([35,  255, 255]))]
}

# --- HOMOGRAPHY ---
# Calibrate these 4 image↔world point pairs to your real camera setup
_IMAGE_POINTS = np.array([
    [317, 150],
    [316, 364],
    [150, 366],
    [207, 150]
], dtype=np.float32)

_WORLD_POINTS = np.array([
    [2.0,  2.0],
    [2.0, -2.0],
    [0.0, -2.0],
    [0.0,  2.0]
], dtype=np.float32)

_H, _ = cv2.findHomography(_IMAGE_POINTS, _WORLD_POINTS)

# --- MODULE STATE ---
_sender_process = None
_shm            = None
_frame_buffer   = None


def start_camera():
    """Launch image_sender.py via system Python and connect to shared memory."""
    global _sender_process, _shm, _frame_buffer

    print("Launching camera sender...")
    _sender_process = subprocess.Popen([SYSTEM_PYTHON, SENDER_PATH])

    while True:
        try:
            _shm = shared_memory.SharedMemory(name=SHM_NAME)
            try:
                unregister(_shm._name, "shared_memory")
            except KeyError:
                pass
            print(f"Connected to shared memory '{SHM_NAME}'")
            break
        except FileNotFoundError:
            if _sender_process.poll() is not None:
                print("image_sender.py exited unexpectedly.")
                sys.exit(1)
            time.sleep(0.5)

    _frame_buffer = np.ndarray(
        (HEIGHT, WIDTH, CHANNELS), dtype=np.uint8, buffer=_shm.buf
    )


def stop_camera():
    """Disconnect from shared memory and terminate image_sender.py."""
    global _shm, _sender_process
    if _shm is not None:
        _shm.close()
        _shm = None
    if _sender_process is not None and _sender_process.poll() is None:
        _sender_process.terminate()
        _sender_process = None
    print("Camera stopped.")


def pixel_to_world(cX, cY):
    """Convert pixel coordinates to world coordinates via homography."""
    point = np.array([[[cX, cY]]], dtype=np.float32)
    world = cv2.perspectiveTransform(point, _H)
    return float(world[0][0][0]), float(world[0][0][1])


def detect_bricks(convert_to_world=True):
    """
    Capture one frame and detect all Lego bricks by color.

    Returns:
        List of dicts: [{"color": "Red", "px": 320, "py": 240,
                         "world_x": 1.2, "world_y": -0.5}, ...]
        world_x/world_y only included if convert_to_world=True.
    """
    if _frame_buffer is None:
        raise RuntimeError("Camera not started — call start_camera() first.")

    frame = _frame_buffer.copy()
    bgr   = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    hsv   = cv2.cvtColor(bgr,   cv2.COLOR_BGR2HSV)

    detected = []

    for color_name, ranges in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            if cv2.contourArea(cnt) > 5000:
                x, y, w, h = cv2.boundingRect(cnt)
                cX = x + w // 2
                cY = y + h // 2

                brick = {"color": color_name, "px": cX, "py": cY}

                if convert_to_world:
                    wx, wy = pixel_to_world(cX, cY)
                    brick["world_x"] = wx
                    brick["world_y"] = wy

                detected.append(brick)

    return detected


def preview(seconds=10):
    """
    Show a live detection preview window for calibration/debugging.
    Press 'q' to quit early.
    """
    if _frame_buffer is None:
        raise RuntimeError("Camera not started — call start_camera() first.")

    print(f"Preview running for {seconds}s — press 'q' to quit.")
    deadline = time.time() + seconds

    while time.time() < deadline:
        frame = _frame_buffer.copy()
        bgr   = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        hsv   = cv2.cvtColor(bgr,   cv2.COLOR_BGR2HSV)

        for color_name, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for cnt in contours:
                if cv2.contourArea(cnt) > 5000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(bgr, color_name, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        cv2.imshow("Lego Detector — press q to quit", bgr)
        if cv2.waitKey(10) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()