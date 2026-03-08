import json
import os
import sys
import numpy as np
import cv2
from rustypot import Sts3215PyController
import setup
from image_processing.image_processing import start_camera, stop_camera, detect_bricks, get_frame

HOMOGRAPHY_FILE = "/home/marjoe/TSBB18/P3/homography.json"
ANGLE_SCALE = 1

# ─── SAFETY WRAPPER ───────────────────────────────────────────────────────────
_original_move_arm_physical = setup.move_arm_physical

def _safe_move_arm_physical(arm, q):
    scaled = [a * ANGLE_SCALE for a in q]
    print(f"[SAFE MODE] Original: {[f'{a:.3f}' for a in q]}")
    print(f"[SAFE MODE] Scaled:   {[f'{a:.3f}' for a in scaled]}")
    _original_move_arm_physical(arm, scaled)

setup.move_arm_physical = _safe_move_arm_physical
from setup import *

# ─── HELPERS ──────────────────────────────────────────────────────────────────
def load_homography():
    if not os.path.exists(HOMOGRAPHY_FILE):
        print(f"ERROR: No homography file found at {HOMOGRAPHY_FILE}")
        sys.exit(1)
    with open(HOMOGRAPHY_FILE) as f:
        data = json.load(f)
    return np.array(data["H"], dtype=np.float32)

def pixel_to_world(px, py, H):
    pt = np.array([[[px, py]]], dtype=np.float32)
    world = cv2.perspectiveTransform(pt, H)
    return float(world[0][0][0]), float(world[0][0][1])

# ─── MAIN ─────────────────────────────────────────────────────────────────────
print("=" * 50)
print("  SAFE MODE — angles scaled to 1/4")
print("=" * 50)

H = load_homography()

arm = Sts3215PyController(serial_port=PORT, baudrate=BAUDRATE, timeout=0.2)
arm_limits()
home_arm(arm)
start_camera()

try:
    bricks = detect_bricks()
    print(f"Detected {len(bricks)} brick(s).")
    

    for brick in bricks:
            
        color = brick["color"]
        px, py = brick["pixel"]
        world_x, world_y = pixel_to_world(px, py, H)
        drop_x, drop_y = DROP_ZONES.get(color, DROP_ZONES["Red"])

        print(f"\n→ Picking up {color} at ({world_x:.2f}, {world_y:.2f})")
        pick_up_lego(arm, world_x, world_y)

        print(f"→ Dropping at ({drop_x:.2f}, {drop_y:.2f})")
        drop_lego(arm, drop_x, drop_y)

finally:
    stop_camera()
    home_arm(arm)
    relax_arm(arm)
    print("\nDone.")
