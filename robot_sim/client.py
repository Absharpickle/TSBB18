import time
from turtle import position
import cvl_robot
import pybullet as p
import cv2
import numpy as np
import math
import random

# --- FIND BRICKS ---

def find_all_bricks(rgb_image):
    if rgb_image.shape[2] == 4:
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGRA2BGR)
    else:
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    color_ranges = {
        "Red": [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [180, 255, 255])],
        "Green": [([40, 50, 50], [90, 255, 255])],
        "Blue": [([100, 150, 50], [140, 255, 255])],
        "Yellow": [([20, 100, 100], [40, 255, 255])]
    }

    found_bricks = []

    for color_name, ranges in color_ranges.items():
        color_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        for lower, upper in ranges:
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            color_mask = cv2.bitwise_or(color_mask, mask)

        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            # Ignore small countours
            if 20 < area < 200:
                x, y, w, h = cv2.boundingRect(contour)
                cX = x + w // 2
                cY = y + int(h * 0.25)
                # Add brick to list
                found_bricks.append((cX, cY, color_name))

    return found_bricks



# --- PIXEL TO WORLD COORDINATES ---

def get_pixel_coordinates(image):
    if image.shape[2] == 4:
        display_image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    else:
        display_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    coords = []
    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            coords.append([x, y])
            cv2.circle(display_image, (x, y), 5, (255, 0, 0), -1)
            cv2.imshow("Image", display_image)
            print(f"Clicked at: ({x}, {y})")

    cv2.imshow("Image", display_image)
    cv2.setMouseCallback("Image", click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return coords

def pixel_to_world_coordinates(cX, cY):
    image_points = np.array([
        [317, 150],     # Values from calibration
        [316, 364], 
        [150, 366], 
        [207, 150]
    ], dtype=np.float32)

    world_points = np.array([
        [2.0, 2.0],
        [2.0, -2.0],
        [0.0, -2.0],
        [0.0, 2.0]
    ], dtype=np.float32)

    H, _ = cv2.findHomography(image_points, world_points)

    point_homogeneous = np.array([[[cX, cY]]], dtype=np.float32)
    world_point_homogeneous = cv2.perspectiveTransform(point_homogeneous, H)

    world_point_x = world_point_homogeneous[0][0][0]
    world_point_y = world_point_homogeneous[0][0][1]
    return world_point_x, world_point_y

def standard_pose(robot_id):
    print("Lifting arm to upright position")
    
    base_rad = p.getJointState(robot_id, 0)[0]
    shoulder_rad = p.getJointState(robot_id, 1)[0]
    elbow_rad = p.getJointState(robot_id, 2)[0]
    wrist_rad = p.getJointState(robot_id, 3)[0]
    
    # 2. Konvertera till grader
    current_base = np.rad2deg(base_rad)
    current_shoulder = np.rad2deg(shoulder_rad)
    current_elbow = np.rad2deg(elbow_rad)
    current_wrist = np.rad2deg(wrist_rad)
    
    if current_wrist < -10:
        upright_shoulder = current_shoulder + 30
    elif current_wrist > 10:
        upright_shoulder = current_shoulder - 30
    else:
        upright_shoulder = current_shoulder
    
    cvl_robot.move_arm(robot_id, 
                       base=current_base, 
                       shoulder=upright_shoulder, 
                       elbow=current_elbow, 
                       wrist=current_wrist)

    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./120.)
    
    print("Arm is in upright position")


# --- MOVE ROBOT TO XYZ ---

def move_to_xyz(robot_id, target_x, target_y, target_z, stop_on_contact=False):
    tip_index = 4
    yaw = math.atan2(target_y, target_x)


    # NEEDS CALIBRATION
    if target_x > 1:
        tcp_offset = 0.55
        claw_length = 0.25
    else:
        tcp_offset = 0.35
        claw_length = 0.35 #   # DISTANCE BEHIND TIP
    
    #tcp_offset = 0.40
    #claw_length = 0.35  # VERTICAL HEIGHT FOR WRIST
    
    adjusted_x = target_x - tcp_offset * math.cos(yaw)
    adjusted_y = target_y - tcp_offset * math.sin(yaw)

    final_wrist_z = target_z + claw_length

    # --- DYNAMIC MOVEMENT ---
    if stop_on_contact:
        current_state = p.getLinkState(robot_id, tip_index)
        current_wrist_z = current_state[0][2]
        
        # 2. CALCULATE Z DISTANCE AND STEPS
        z_distance = current_wrist_z - final_wrist_z
        num_steps = max(1, int(z_distance * 100))
        
        for s in range(1, num_steps + 1):
            # GRADUAL APPROACH
            step_z = current_wrist_z - (z_distance * (s / num_steps))
            
            joint_poses = p.calculateInverseKinematics(
                robot_id, tip_index, [adjusted_x, adjusted_y, step_z]
            )
            
            # FAST RESPONSE
            for i in range(4):
                p.setJointMotorControl2(
                    bodyUniqueId=robot_id, jointIndex=i,
                    controlMode=p.POSITION_CONTROL, targetPosition=joint_poses[i],
                    force=500.0, maxVelocity=2.0
                )
            
            for _ in range(5):
                p.stepSimulation()
                time.sleep(1/240)
            
            # --- SENSORY CHECK ---
            c_l = p.getContactPoints(bodyA=robot_id, linkIndexA=5)
            c_r = p.getContactPoints(bodyA=robot_id, linkIndexA=7)

            c_l = c_l if c_l is not None else []
            c_r = c_r if c_r is not None else []

            if any(c[2] != robot_id for c in (c_l + c_r)):
                print(f"Contact at: {step_z:.3f}")
                break

        for _ in range(60):
            p.stepSimulation()
            time.sleep(1./240.)
                
    # --- NORMAL MOVEMENT ---
    else:
        target_pos = [adjusted_x, adjusted_y, final_wrist_z]
        joint_poses = p.calculateInverseKinematics(robot_id, tip_index, target_pos)
        
        cvl_robot.move_arm(robot_id, 
                           base=np.rad2deg(joint_poses[0]), 
                           shoulder=np.rad2deg(joint_poses[1]), 
                           elbow=np.rad2deg(joint_poses[2]), 
                           wrist=np.rad2deg(joint_poses[3]))
    
        for _ in range(240):
            p.stepSimulation()
            time.sleep(1./240.)

def pick_up_lego(robot_id, target_x, target_y):
    
    cvl_robot.control_claw(robot_id, open_claw=True)
    move_to_xyz(robot_id, target_x, target_y, target_z=0.60)
    move_to_xyz(robot_id, target_x, target_y, target_z=0.05, stop_on_contact=True) 
    cvl_robot.control_claw(robot_id, open_claw=False)

    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./120.)

def drop_lego_brick(robot_id, storage_x, storage_y):
    move_to_xyz(robot_id, storage_x, storage_y, target_z=0.80)
    move_to_xyz(robot_id, storage_x, storage_y, target_z=0.60)
    
    cvl_robot.control_claw(robot_id, open_claw=True)
    
    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./120.)

def spawn_random_legos():
    
    colors = {
        "red": [1, 0, 0],
        "green": [0, 1, 0],
        "blue": [0, 0, 1],
        "yellow": [1, 1, 0]
    }
    color_choices = list(colors.values())
    
    spawned_bricks = []
    spawned_positions = [] 
    
    min_distance_between_bricks = 0.4
    min_distance_from_robot = 0.8      

    for i in range(5):
        valid_position = False
        
        while not valid_position:
            rand_x = random.uniform(0.5, 1.9) 
            rand_y = random.uniform(-1.8, 1.9)
            
            valid_position = True
        
            distance_to_robot = math.sqrt(rand_x**2 + rand_y**2)
            if distance_to_robot < min_distance_from_robot:
                valid_position = False
                continue
            
            for (px, py) in spawned_positions:
                distance = math.sqrt((rand_x - px)**2 + (rand_y - py)**2)
                if distance < min_distance_between_bricks:
                    valid_position = False 
                    break
                    
        spawned_positions.append((rand_x, rand_y)) 
        
        rand_z = 0.1 
        rand_color = random.choice(color_choices)
        
        brick_id = cvl_robot.create_lego_brick(
            color=rand_color,
            position=(rand_x, rand_y, rand_z)
        )
        spawned_bricks.append(brick_id)
        
        for _ in range(50):
            p.stepSimulation()
            
    return spawned_bricks


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

    #pixel_points = get_pixel_coordinates(rgb_image)

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
    
    # Print the world coordinates (position) of each joint
    
    
    # cvl_robot.state(robot_id)

    while(True):
        p.stepSimulation()
        time.sleep(0.01)
    p.disconnect()

if __name__ == "__main__":
    main()
