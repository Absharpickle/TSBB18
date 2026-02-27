import subprocess
import time
import sys
import signal
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister
import numpy as np
import cv2

# --- CONFIG ---
SHM_NAME = "camera_shm_hd"
WIDTH = 1280
HEIGHT = 720
CHANNELS = 4
SENDER_PATH = "/home/marjoe/tsbb18/P3/image_processing/image_sender.py"
SYSTEM_PYTHON = "/usr/bin/python3"

# --- COLOR DEF (HSV) ---
# Hue: 0-180, Sat: 0-255, Val: 0-255
color_ranges = {
    "Red": [
        (np.array([0, 150, 50]), np.array([10, 255, 255])),
        (np.array([170, 150, 50]), np.array([180, 255, 255]))
    ],
    "Blue":  [(np.array([100, 200, 150]), np.array([140, 255, 255]))],
    "Green": [(np.array([35, 50, 50]), np.array([90,  255, 255]))],
    "Yellow":[(np.array([20,  170, 150]), np.array([35, 255, 255]))]
}

print("------------------------------------------------")
print(f"Launching Camera Sender...")
sender_process = subprocess.Popen([SYSTEM_PYTHON, SENDER_PATH])

# Wait for Shared Memory
shm = None
while True:
    try:
        shm = shared_memory.SharedMemory(name=SHM_NAME)
        try:
            unregister(shm._name, 'shared_memory')
        except KeyError:
            pass 
        print(f"Connected to Shared Memory '{SHM_NAME}'")
        break
    except FileNotFoundError:
        if sender_process.poll() is not None:
            sys.exit(1)
        time.sleep(0.5)

frame_buffer = np.ndarray((HEIGHT, WIDTH, CHANNELS), dtype=np.uint8, buffer=shm.buf)

print("\nLego Detector Running.")
print("Press 'q' to quit.\n")

try:
    while True:
        frame = frame_buffer.copy()
        
        # 1. Convert to BGR then HSV
        bgr_frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

        # 2. Color processing
        for color_name, ranges in color_ranges.items():
            mask = np.zeros(hsv_frame.shape[:2], dtype="uint8")
            for (lower, upper) in ranges:
                sub_mask = cv2.inRange(hsv_frame, lower, upper)
                mask = cv2.bitwise_or(mask, sub_mask)

            # Noise cleanup
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

            # Find Contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                if cv2.contourArea(cnt) > 5000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    # Draw on original frame
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, color_name, (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        cv2.imshow("Lego Detector", frame)
        
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nCtrl+C detected.")

finally:
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    if 'shm' in locals() and shm is not None:
        shm.close()
    if sender_process.poll() is None:
        sender_process.terminate()
