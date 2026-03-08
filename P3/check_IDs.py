from rustypot import Sts3215PyController
from setup import forward_kinematics, relax_arm, PORT, BAUDRATE

def scan_servos(port=PORT, baudrate=BAUDRATE, max_id=20):
    print(f"Scanning for servos on {port}...")

    try:
        controller = Sts3215PyController(serial_port=port, baudrate=baudrate, timeout=0.05)
    except Exception as e:
        print(f"Failed to open {port}. Check connection and port name.\nError: {e}")
        return

    found_ids = []

    for servo_id in range(1, max_id + 1):
        try:
            pos = controller.read_present_position(servo_id)
            print(f"[+] Found servo ID {servo_id} | Position: {pos:.4f} rad")
            found_ids.append(servo_id)
        except Exception:
            pass

    print("\n--- Scan Complete ---")
    if found_ids:
        print(f"Found {len(found_ids)} servos. Active IDs: {found_ids}")

        if all(i in found_ids for i in [1, 2, 3, 4]):
            angles = [controller.read_present_position(i) for i in [1, 2, 3, 4]]
            tcp = forward_kinematics(angles)
            print(f"\n--- TCP Position ---")
            print(f"  Angles : {[f'{a:.4f}' for a in angles]}")
            print(f"  X: {tcp[0]:.4f} m")
            print(f"  Y: {tcp[1]:.4f} m")
            print(f"  Z: {tcp[2]:.4f} m  ← use this as FLOOR_Z")
    else:
        print("No servos found.")

if __name__ == "__main__":
    scan_servos()
    input("Press Enter to relax arm, then move claw to table surface and run again...")
    relax_arm()
