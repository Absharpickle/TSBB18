import cv2
import numpy as np
import json
import os
from image_processing import start_camera, stop_camera

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "homography.json")

# ─── STEP 1: Define your 4 world positions ────────────────────────────────────
# Place 4 bricks at these exact XY positions on the table (metres)
# Adjust these to wherever you physically place your calibration bricks
WORLD_POINTS = [
    [2.0,  2.0],   # brick 1 — far right,  far positive Y
    [2.0, -2.0],   # brick 2 — far right,  far negative Y
    [0.0, -2.0],   # brick 3 — near robot, far negative Y
    [0.0,  2.0],   # brick 4 — near robot, far positive Y
]

# ─── CLICK HANDLER ────────────────────────────────────────────────────────────

clicked_points = []

def on_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append([x, y])
        cv2.circle(param, (x, y), 8, (0, 255, 0), -1)
        cv2.putText(param, f"{len(clicked_points)}", (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Calibration", param)
        print(f"  Point {len(clicked_points)}: pixel ({x}, {y})  "
              f"→ world {WORLD_POINTS[len(clicked_points)-1]}")


# ─── MAIN ─────────────────────────────────────────────────────────────────────

def main():
    print("=== Homography Calibration ===\n")
    print("Place 4 bricks at these world positions (X, Y) in metres:")
    for i, (wx, wy) in enumerate(WORLD_POINTS, 1):
        print(f"  Brick {i}: ({wx:.1f}, {wy:.1f})")
    print("\nStarting camera...")

    start_camera()

    try:
        from image_processing import _frame_buffer
        import time
        time.sleep(1.0)  # let camera warm up

        frame = _frame_buffer.copy()
        display = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        print("\nClick each brick in the image IN ORDER (1→2→3→4).")
        print("Press 'r' to reset clicks, 'q' to quit without saving.\n")

        cv2.imshow("Calibration", display)
        cv2.setMouseCallback("Calibration", on_click, display)

        while True:
            key = cv2.waitKey(20) & 0xFF

            if key == ord('r'):
                clicked_points.clear()
                # Redraw clean frame
                frame   = _frame_buffer.copy()
                display = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                cv2.imshow("Calibration", display)
                print("Reset — click the 4 bricks again.")

            elif key == ord('q'):
                print("Quit — no calibration saved.")
                break

            elif len(clicked_points) == 4:
                # All 4 points collected — compute homography
                image_pts = np.array(clicked_points, dtype=np.float32)
                world_pts = np.array(WORLD_POINTS,   dtype=np.float32)

                H, mask = cv2.findHomography(image_pts, world_pts)

                if H is None:
                    print("Homography computation failed — try again.")
                    clicked_points.clear()
                    continue

                # Save to JSON
                data = {
                    "H":            H.tolist(),
                    "image_points": image_pts.tolist(),
                    "world_points": world_pts.tolist()
                }
                with open(CALIBRATION_FILE, "w") as f:
                    json.dump(data, f, indent=2)

                print(f"\nHomography saved to {CALIBRATION_FILE}")
                print("H =")
                print(np.round(H, 4))

                # Show reprojection to verify accuracy
                print("\nReprojection check (world coords from clicked pixels):")
                for i, (px, py) in enumerate(clicked_points):
                    pt    = np.array([[[px, py]]], dtype=np.float32)
                    world = cv2.perspectiveTransform(pt, H)
                    wx, wy = world[0][0]
                    print(f"  Brick {i+1}: clicked ({px}, {py})  "
                          f"→ ({wx:.3f}, {wy:.3f})  "
                          f"target {WORLD_POINTS[i]}")
                break

    finally:
        cv2.destroyAllWindows()
        stop_camera()


if __name__ == "__main__":
    main()
