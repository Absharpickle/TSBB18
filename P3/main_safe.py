import setup
from image_processing import detect_bricks, start_camera, stop_camera

# ─── SAFETY WRAPPER ───────────────────────────────────────────────────────────

ANGLE_SCALE = 0.25  # Move at 1/4 of computed angles

_original_move_arm_physical = setup.move_arm_physical

def _safe_move_arm_physical(angles, port=setup.PORT, baudrate=setup.BAUDRATE):
    """Scale all joint angles to ANGLE_SCALE before sending to the robot."""
    scaled = [a * ANGLE_SCALE for a in angles]
    print(f"[SAFE MODE] Original angles: {[f'{a:.3f}' for a in angles]}")
    print(f"[SAFE MODE] Scaled  angles: {[f'{a:.3f}' for a in scaled]}")
    _original_move_arm_physical(scaled, port=port, baudrate=baudrate)

# Patch setup module so all downstream calls (pick_up_lego, drop_lego, etc.) use it
setup.move_arm_physical = _safe_move_arm_physical

# Now import everything else — they will use the patched version
from setup import *

# ─── MAIN ─────────────────────────────────────────────────────────────────────

print("=" * 50)
print("  SAFE MODE — angles scaled to 1/4")
print("=" * 50)

home_arm()
start_camera()

try:
    bricks = detect_bricks()
    print(f"Detected {len(bricks)} brick(s).")

    for brick in bricks:
        color    = brick["color"]
        wx, wy   = brick["world"]
        drop_x, drop_y = DROP_ZONES.get(color, DROP_ZONES["default"])

        print(f"\n→ Picking up {color} brick at world ({wx:.2f}, {wy:.2f})")
        pick_up_lego(wx, wy)

        print(f"→ Dropping at ({drop_x:.2f}, {drop_y:.2f})")
        drop_lego(drop_x, drop_y)

finally:
    stop_camera()
    home_arm()
    print("\nDone.")
