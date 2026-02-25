import time
from turtle import position
import cvl_robot
import pybullet as p
import cv2
import numpy as np
import math
import random
from cvl_robot import *


# --- MAIN FUNCTION ---

def main():
    # Initialize the simulation
    cvl_robot.initialize_simulation(gui=True)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Create the Braccio robotic arm
    robot_id = cvl_robot.create_braccio_arm(position=(0, 0, 0.02))
      
    # Create Lego bricks      
    
    brick_id = spawn_random_legos()

    for _ in range(120):
        p.stepSimulation()

    rgb_image = cvl_robot.capture_image()

    pixel_coords = find_all_bricks(rgb_image)

    for index, (cX, cY, color) in enumerate(pixel_coords):

        world_x, world_y = pixel_to_world_coordinates(cX, cY)

        pick_up_lego(robot_id, world_x, world_y)

        standard_pose(robot_id)

        lager_x = 1.8
        lager_y = 0.0

        offset_y = lager_y + index * 0.15

        drop_lego_brick(robot_id, lager_x, offset_y)

        standard_pose(robot_id)

    while(True):
        p.stepSimulation()
        time.sleep(0.01)
    p.disconnect()

if __name__ == "__main__":
    main()
