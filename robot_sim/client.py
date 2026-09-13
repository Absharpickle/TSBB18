import time
from turtle import position
import cvl_robot
import pybullet as p
import cv2
import numpy as np
import math
import random
from cvl_robot import *
import rustypot


DROP_ZONES = {
    "Red":      (1.8, 1.5),
    "Green":    (1.8, 0.5),
    "Blue":     (1.8, -0.5),
    "Yellow":   (1.8, -1.5)
}

# --- MAIN FUNCTION ---

def main():
    # Initialize the simulation
    initialize_simulation(gui=True)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Create the Braccio robotic arm
    robot_id = create_braccio_arm(position=(0, 0, 0.02))
      
    # Create Lego bricks      
    
    brick_id = spawn_random_legos()

    for _ in range(120):
        p.stepSimulation()

    rgb_image = capture_image()

    pixel_coords = find_all_bricks(rgb_image)

    drop_counts = {"Red": 0, "Green": 0, "Blue": 0, "Yellow": 0}

    for index, (cX, cY, color) in enumerate(pixel_coords):

        world_x, world_y = pixel_to_world_coordinates(cX, cY)

        pick_up_lego(robot_id, world_x, world_y)

        if color not in DROP_ZONES:
            color = "Red"   # Fallback

        zone_x, zone_y = DROP_ZONES[color]
        offset_y = zone_y + drop_counts[color] * 0.15

        drop_lego_brick(robot_id, zone_x, offset_y)

        drop_counts[color] += 1

    while(True):
        p.stepSimulation()
        time.sleep(0.01)
    p.disconnect()

if __name__ == "__main__":
    main()
