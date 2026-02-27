import signal
import sys
import time
from multiprocessing import shared_memory
import numpy as np
from picamera2 import Picamera2

# --- CONFIGURATION ---
SHM_NAME = "camera_shm_hd"
WIDTH = 1280
HEIGHT = 720
CHANNELS = 4 

# --- SIGNAL HANDLING ---
# This flag controls the main loop
running = True

def handle_stop_signals(signum, frame):
    """Handles SIGTERM (from terminate()) and SIGINT (Ctrl+C)"""
    global running
    print(f"\nSignal {signum} received. Stopping loop...")
    running = False

# Register the handlers
signal.signal(signal.SIGINT, handle_stop_signals)  # Ctrl+C
signal.signal(signal.SIGTERM, handle_stop_signals) # process.terminate()

# --- CAMERA SETUP ---
print("Initializing Camera...")
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (WIDTH, HEIGHT), "format": "XRGB8888"})
picam2.configure(config)
picam2.start()

# --- SHARED MEMORY SETUP ---
try:
    shm = shared_memory.SharedMemory(create=True, size=WIDTH*HEIGHT*CHANNELS, name=SHM_NAME)
except FileExistsError:
    # If it exists from a previous crash, connect to it
    shm = shared_memory.SharedMemory(name=SHM_NAME)

# Create buffer wrapper
frame_buffer = np.ndarray((HEIGHT, WIDTH, CHANNELS), dtype=np.uint8, buffer=shm.buf)

print(f"Shared Memory '{SHM_NAME}' ready.")
print("Broadcasting...")

# --- MAIN LOOP ---
try:
    while running:
        # 1. Capture to local array
        frame = picam2.capture_array()
        
        # 2. Copy to shared buffer
        frame_buffer[:] = frame
        
        # Optional: Add a tiny sleep if you want to reduce CPU usage
        # time.sleep(0.001)

except Exception as e:
    print(f"Error in main loop: {e}")

finally:
    # --- CLEANUP ---
    print("Cleaning up resources...")
    picam2.stop()
    shm.close()
    try:
        shm.unlink()
        print("Shared memory unlinked.")
    except FileNotFoundError:
        pass # Already gone
    print("Sender Stopped.")