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
    
def detect_move(original_bricks, H, threshold=0.15):
    
    current_bricks = detect_bricks()
    moved = []
    
    for orig in original_bricks:
        ox, oy = pixel_to_world(*orig["pixel"], H)
        
        match = min(
        (abs(pixel_to_world(*b["pixel"], H)[0] - ox) +
        abs(pixel_to_world(*b["pixel"], H)[1] - oy))
        
        for b in current_bricks if b["color"] == orig["color"]
        ) if any(b["color"] == orig["color"] for b in current_bricks) else 999
        
        if match > threshold:
            moved.append(orig)
    return moved
    
def near_drop(wx, wy, margin=0.3):
    return any(
    abs(wx - dx) < margin and abs(wy - dy) < margin
    for dx, dy in DROP_ZONES.values()
    )
    
def get_active_bricks(H):
    return [
        b for b in detect_bricks()
        if not near_drop(*pixel_to_world(*b["pixel"], H))
    ]

# ─── MAIN ─────────────────────────────────────────────────────────────────────

H = load_homography()

arm = Sts3215PyController(serial_port=PORT, baudrate=BAUDRATE, timeout=0.2)
arm_limits(arm)
home_arm(arm)
start_camera()
#move_arm_physical(arm, [0.0, 0.0, 1.0, -1.0])

try:
    bricks = get_active_bricks(H)
    
    print(f"Detected {len(bricks)} brick(s).")
    

    while bricks:
        brick = bricks[0]
        color = brick["color"]
        world_x, world_y = pixel_to_world(*brick["pixel"], H)

        print(f"\n→ Picking up {color} at ({world_x:.2f}, {world_y:.2f})")
        pick_up_lego(arm, world_x, world_y)

        print(f"→ Dropping at {DROP_ZONES[color]}")
        drop_lego(arm, *DROP_ZONES[color])
        
        bricks = get_active_bricks(H)

finally:
    stop_camera()
    home_arm(arm)
    for s_id in ALL_SERVO_IDS:
        try:
            arm.write_torque_enable(s_id, False)
        except Exception:
            pass
    print("\nDone.")
