import os
import json
import sys
import numpy as np
import cv2
from image_processing.image_processing import start_camera, stop_camera, get_frame, detect_bricks

# ─── CONFIG ───────────────────────────────────────────────────────────────────
CALIBRATION_FILE = "/home/marjoe/TSBB18/P3/homography.json"

# Place 4 bricks at these exact positions on the table (metres)
WORLD_POINTS = [
    [1.0,  2.0],   # brick 1
    [0.0, 2.0],
    [-1.0, 2.0],   # brick 2
    [-1.0, 3.0],   # brick 3
    [0.0, 3.0],
    [1.0,  3.0],   # brick 4
]

# ─── CLICK HANDLER ────────────────────────────────────────────────────────────
_clicked = []

def _on_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(_clicked) < len(WORLD_POINTS):
        _clicked.append([x, y])
        print(f"  Point {len(_clicked)}: pixel ({x}, {y})  "
              f"→ world {WORLD_POINTS[len(_clicked) - 1]}")


# ─── MAIN ─────────────────────────────────────────────────────────────────────
def main():
    print("=== Homography Calibration ===")
    print("Place 4 bricks at these world positions (metres):")
    for i, (wx, wy) in enumerate(WORLD_POINTS, 1):
        print(f"  Brick {i}: ({wx}, {wy})")

    start_camera()
    print("\nClick the 4 bricks IN ORDER on the live feed.")
    print("Press 'r' to reset, 'q' to quit without saving.\n")

    cv2.namedWindow("Calibration")
    cv2.setMouseCallback("Calibration", _on_click)

    saved = False
    try:
        while True:
            display = get_frame()

            # Draw clicked points
            for i, (px, py) in enumerate(_clicked):
                cv2.circle(display, (px, py), 8, (0, 255, 0), -1)
                cv2.putText(display, str(i + 1), (px + 10, py - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Status overlay
            if saved:
                status, color = "Saved! Press 'q' to close.", (0, 255, 0)
            elif len(_clicked) < len(WORLD_POINTS):
                status, color = f"Click brick {len(_clicked) + 1} of {len(WORLD_POINTS)}", (0, 200, 255)
            else:
                status, color = "All 4 clicked — saving...", (0, 200, 255)
            cv2.putText(display, status, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            cv2.imshow("Calibration", display)
            key = cv2.waitKey(10) & 0xFF

            if key == ord('r'):
                _clicked.clear()
                saved = False
                print("Reset — click the 4 bricks again.")

            elif key == ord('q'):
                if not saved:
                    print("Quit — no calibration saved.")
                break

            elif len(_clicked) == len(WORLD_POINTS) and not saved:
                image_pts = np.array(_clicked,     dtype=np.float32)
                world_pts = np.array(WORLD_POINTS, dtype=np.float32)
                H, _ = cv2.findHomography(image_pts, world_pts)

                if H is None:
                    print("Homography computation failed — resetting.")
                    _clicked.clear()
                    continue

                with open(CALIBRATION_FILE, "w") as f:
                    json.dump({
                        "H":            H.tolist(),
                        "image_points": image_pts.tolist(),
                        "world_points": world_pts.tolist(),
                    }, f, indent=2)

                print(f"\nHomography saved to: {CALIBRATION_FILE}")
                print("\nReprojection check:")
                for i, (px, py) in enumerate(_clicked):
                    pt = np.array([[[px, py]]], dtype=np.float32)
                    world = cv2.perspectiveTransform(pt, H)
                    wx, wy = world[0][0]
                    print(f"  Brick {i+1}: clicked ({px}, {py})  "
                          f"→ ({wx:.3f}, {wy:.3f})  target {WORLD_POINTS[i]}")
                saved = True

    finally:
        cv2.destroyAllWindows()
        stop_camera()


if __name__ == "__main__":
    main()
