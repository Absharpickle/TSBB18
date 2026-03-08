import os
import json
import sys
import numpy as np
import cv2
from rustypot import Sts3215PyController
from setup import forward_kinematics, ALL_SERVO_IDS, ARM_SERVO_IDS, PORT, BAUDRATE
from image_processing.image_processing import start_camera, stop_camera, get_frame, detect_bricks

HOMOGRAPHY_FILE = "/home/marjoe/TSBB18/P3/homography.json"

def load_homography():
    with open(HOMOGRAPHY_FILE) as f:
        data = json.load(f)
    return np.array(data["H"], dtype=np.float32)

def pixel_to_world(px, py, H):
    pt = np.array([[[px, py]]], dtype=np.float32)
    world = cv2.perspectiveTransform(pt, H)
    return float(world[0][0][0]), float(world[0][0][1])

def main():
    H = load_homography()

    # Connect and relax arm
    arm = Sts3215PyController(serial_port=PORT, baudrate=BAUDRATE, timeout=0.2)
    print("Relaxing arm — move it freely by hand.")
    for s_id in ALL_SERVO_IDS:
        try:
            arm.write_torque_enable(s_id, False)
        except Exception:
            pass

    start_camera()
    print("\nLive feed running.")
    print("Move arm tip over a brick and compare TCP vs camera world coords.")
    print("Press 'q' to quit.\n")

    try:
        while True:
            # Read arm TCP
            try:
                angles = [arm.read_present_position(i)[0] for i in ARM_SERVO_IDS]
                tcp = forward_kinematics(angles)
                tcp_str = f"ARM TCP  →  X: {tcp[0]:.3f}  Y: {tcp[1]:.3f}  Z: {tcp[2]:.3f}"
            except Exception:
                tcp_str = "ARM TCP  →  (read error)"

            # Detect bricks and draw world coords
            frame = get_frame()
            bricks = detect_bricks()
            for b in bricks:
                cx, cy = b["pixel"]
                wx, wy = pixel_to_world(cx, cy, H)
                label = f"{b['color']} ({wx:.2f}, {wy:.2f})"
                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
                cv2.putText(frame, label, (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Overlay TCP on frame
            cv2.putText(frame, tcp_str, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

            cv2.imshow("Homography Check", frame)

            # Also print to terminal
            print(f"\r{tcp_str}", end="", flush=True)

            if cv2.waitKey(100) & 0xFF == ord('q'):
                break

    finally:
        print("\n")
        cv2.destroyAllWindows()
        stop_camera()

if __name__ == "__main__":
    main()
