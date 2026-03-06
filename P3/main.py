from rustypot import Sts3215PyController
from setup import *
from image_processing import start_camera, stop_camera, detect_bricks


def main():
    arm = Sts3215PyController(serial_port=PORT, baudrate=BAUDRATE, timeout=0.2)
    arm_limits(port=PORT)
    print("Physical arm connected.")

    start_camera()
    print("Camera ready.")

    try:
        bricks = detect_bricks(convert_to_world=True)
        print(f"Detected {len(bricks)} bricks: {bricks}")

        if not bricks:
            print("No bricks detected — exiting.")
            return

        drop_counts = {"Red": 0, "Green": 0, "Blue": 0, "Yellow": 0}

        for brick in bricks:
            color   = brick.get("color", "Red")
            world_x = brick["world_x"]
            world_y = brick["world_y"]

            print(f"\nPicking {color} brick at ({world_x:.2f}, {world_y:.2f})")
            pick_up_lego(arm, world_x, world_y)

            zone_x, zone_y = DROP_ZONES.get(color, DROP_ZONES["Red"])
            offset_y = zone_y + drop_counts.get(color, 0) * 0.15

            print(f"Dropping {color} brick at ({zone_x:.2f}, {offset_y:.2f})")
            drop_lego(arm, zone_x, offset_y)

            drop_counts[color] = drop_counts.get(color, 0) + 1

        print("\nAll bricks sorted!")

    finally:
        stop_camera()
        home_arm(arm)
        relax_arm(port=PORT)
        print("Done — arm relaxed, camera stopped.")


if __name__ == "__main__":
    main()
