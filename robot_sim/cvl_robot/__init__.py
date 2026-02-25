import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import random
import math

__all__ = ["initialize_simulation", "create_braccio_arm", "move_arm", "control_claw", "create_lego_brick", "move_brick_to_position", "remove_brick", "capture_image", "find_all_bricks", "get_pixel_coordinates", "move_to_xyz", "spawn_random_legos", "drop_lego_brick", "pick_up_lego", "standard_pose", "pixel_to_world_coordinates"]

def initialize_simulation(gui=True):
    """
    Initialize the PyBullet simulation environment.

    Parameters:
        gui (bool): Whether to use the GUI (True) or direct mode (False).

    Returns:
        int: The client ID of the PyBullet connection.
    """
    client_id = p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")  
    p.setGravity(0, 0, -9.81)
    return client_id


def create_braccio_arm(position=(0, 0, 0.2), scale=10):
    """
    Create a Braccio robotic arm in the PyBullet simulation.

    Parameters:
        position (tuple): The (x, y, z) position of the robot.
        scale (float): The scaling factor for the robot model.

    Returns:
        int: The unique ID of the created robot.
    """
    robot_id = p.loadURDF("cvl_robot/cvl-arm.urdf", position, useFixedBase=True, globalScaling=scale)

    #for i in range(p.getNumJoints(robot_id)):
    #    info = p.getJointInfo(robot_id, i)
    #    print(f"{i}: parent={info[16]}, name={info[1].decode()}")
    #for i in range(1, 4):
    #    info = p.getJointInfo(robot_id, i)
    #    axis = info[13]
    #    print(f"joint{i} axis = {axis}")
#
    #return
    return robot_id


def move_arm(robot_id, base=0, shoulder=0, elbow=0, wrist=0, stop_on_contact=False):

    targets = [np.deg2rad(base), np.deg2rad(shoulder), np.deg2rad(elbow), np.deg2rad(wrist)]
    max_vel = 1.3
    forces = 500.0 

    for i in range(4):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=targets[i],
            force=forces,
            maxVelocity=max_vel
        )

    for _ in range(120): 
        p.stepSimulation()
        time.sleep(1./120.)
        
        if stop_on_contact:
            contacts_left = p.getContactPoints(bodyA=robot_id, linkIndexA=5)
            contacts_right = p.getContactPoints(bodyA=robot_id, linkIndexA=7)
            
            hit = False
            for contact in contacts_left + contacts_right:
                if contact[2] != robot_id:
                    hit = True
                    break
                    
            if hit:
                print("Contact detected!")
                for i in range(4):
                    current_angle = p.getJointState(robot_id, i)[0]
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id, 
                        jointIndex=i, 
                        controlMode=p.POSITION_CONTROL, 
                        targetPosition=current_angle, 
                        force=forces
                    )
                break

def control_claw(robot_id, open_claw=True):
    GRIPPER_MIN = 0.0      # fully closed
    GRIPPER_MAX = 0.3     # fully open
    target = GRIPPER_MAX if open_claw else GRIPPER_MIN

    max_vel = 0.4
    force = 400.0

    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=5,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target,
        force=force,
        maxVelocity=max_vel
    )
    
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=7,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target,
        force=force,
        maxVelocity=max_vel
    )

    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./120.)

def state(robot_id):
    """
    Print the world coordinates (position) of each joint of a robot.

    Args:
        robot_id (int): The ID of the robot loaded in PyBullet.
    """
    
    controlled_joints = {
        0: "base",
        1: "shoulder",
        2: "elbow",
        3: "wrist",
        5: "gripper_left",
        7: "gripper_right"
    }

    print("\n=== JOINT STATE ===")
    for j, name in controlled_joints.items():
        angle = p.getJointState(robot_id, j)[0]
        link_state = p.getLinkState(robot_id, j)
        pos, orn = link_state[0], link_state[1]

        print(f"{name:14s}")
        print(f"  angle:       {np.rad2deg(angle):7.2f}°")
        print(f"  world pos:   {np.array(pos)}")

    # end effector = wrist tip
    ee = p.getLinkState(robot_id, 3)[0]
    print("\n=== END EFFECTOR (tip) ===")
    print(f"  world pos:   {np.array(ee)}\n")


def create_lego_brick(color, position, scale=(0.6, 0.3, 0.3)):
    """
    Create a Lego brick in the PyBullet simulation.

    Parameters:
        color (list): RGB values for the brick's color (e.g., [1, 0, 0] for red).
        position (tuple): The (x, y, z) position of the brick.
        scale (tuple): Scaling factors for the brick's dimensions.

    Returns:
        int: The unique ID of the created Lego brick.
    """

    offset = (-0.016, -0.016, -0.0115)

    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName="cvl_robot/lego.obj",
        meshScale=scale,
        collisionFramePosition=offset
    )
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName="cvl_robot/lego.obj",
        meshScale=scale,
        rgbaColor=color + [1]
    )
    brick_id = p.createMultiBody(
        baseMass=0.2,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=position,
        baseOrientation=[0, 0, 0, 1]
    )
    return brick_id

def move_brick_to_position(brick_id, target_position):
    """
    Move a Lego brick to a target position in the simulation.

    Parameters:
        brick_id (int): The unique ID of the Lego brick to move.
        target_position (tuple or list): The (x, y, z) target position for the brick.

    Returns:
        None
    """
    try:
        # Validate target_position
        if len(target_position) != 3:
            raise ValueError("Target position must be a tuple or list of three floats (x, y, z).")
        
        # Get the current orientation of the brick
        current_orientation = p.getBasePositionAndOrientation(brick_id)[1]
        
        # Move the brick to the target position while keeping its current orientation
        p.resetBasePositionAndOrientation(brick_id, target_position, current_orientation)

        for _ in range(240):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    except Exception as e:
        print(f"Error moving Lego brick with ID {brick_id}: {e}")

def remove_brick(brick_id):
    """
    Remove a Lego brick from the simulation.

    Parameters:
        brick_id (int): The unique ID of the Lego brick to remove.

    Returns:
        None
    """
    try:
        p.removeBody(brick_id)
    except Exception as e:
        print(f"Error removing Lego brick with ID {brick_id}: {e}")


def capture_image(image_width=640, image_height=480, camera_position=(2, -4, 5), target_position=(2, 0, 0), output_filename="captured_image.png"):
    """
    Capture an image from the simulation, save it as a PNG, and return the image as OpenCV data.

    Parameters:
        image_width (int): Width of the captured image (default is 640).
        image_height (int): Height of the captured image (default is 480).
        camera_position (tuple): (x, y, z) position of the camera (default is above and behind the workspace).
        target_position (tuple): (x, y, z) position the camera is looking at (default is looking at the center).
        output_filename (str): The filename to save the captured image (default is "captured_image.png").

    Returns:
        numpy.ndarray: The captured image as OpenCV data (RGB format).
    """
    try:
        # Set up the view matrix for the camera
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=[0, 1, 0]  # The "up" direction for the camera
        )

        # Set up the projection matrix
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=float(image_width) / image_height,
            nearVal=0.01,
            farVal=100
        )

        # Capture the image
        _, _, rgb_image, _, _ = p.getCameraImage(
            width=image_width,
            height=image_height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        rgb_image = np.array(rgb_image, dtype=np.uint8).reshape((image_height, image_width, 4))[:, :, :3]  # RGB

        # Save the image as a PNG file
        cv2.imwrite(output_filename, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))  # OpenCV expects BGR format
        print(f"Image saved as {output_filename}")
        return rgb_image

    except Exception as e:
        print(f"Error capturing and saving image: {e}")
        return None

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
            
            joint_poses = calculate_jacobian_ik(
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
        joint_poses = calculate_jacobian_ik(robot_id, tip_index, target_pos)
        
        move_arm(robot_id, 
                           base=np.rad2deg(joint_poses[0]), 
                           shoulder=np.rad2deg(joint_poses[1]), 
                           elbow=np.rad2deg(joint_poses[2]), 
                           wrist=np.rad2deg(joint_poses[3]))
    
        for _ in range(240):
            p.stepSimulation()
            time.sleep(1./240.)

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
        
        brick_id = create_lego_brick(
            color=rand_color,
            position=(rand_x, rand_y, rand_z)
        )
        spawned_bricks.append(brick_id)
        
        for _ in range(50):
            p.stepSimulation()
            
    return spawned_bricks

def drop_lego_brick(robot_id, storage_x, storage_y):
    move_to_xyz(robot_id, storage_x, storage_y, target_z=0.80)
    move_to_xyz(robot_id, storage_x, storage_y, target_z=0.60)
    
    control_claw(robot_id, open_claw=True)
    
    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./120.)

def pick_up_lego(robot_id, target_x, target_y):
    
    control_claw(robot_id, open_claw=True)
    move_to_xyz(robot_id, target_x, target_y, target_z=0.60)
    move_to_xyz(robot_id, target_x, target_y, target_z=0.05, stop_on_contact=True) 
    control_claw(robot_id, open_claw=False)

    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./120.)
                
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
    
    move_arm(robot_id, 
                       base=current_base, 
                       shoulder=upright_shoulder, 
                       elbow=current_elbow, 
                       wrist=current_wrist)

    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./120.)
    
    print("Arm is in upright position")

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

def get_numerical_jacobian(robot_id, tip_index, q_current, arm_dof_indices, delta=1e-4):
    """
    Calculates the Jacobian matrix manually using finite differences.
    """
    # Number of Degrees of Freedom (DoF) we are controlling (should be 4)
    num_dof = len(arm_dof_indices)
    
    # Create an empty 3x4 matrix (3 spatial dimensions X, Y, Z by 4 joint angles)
    J_arm = np.zeros((3, num_dof))
    
    # 1. Get the baseline position f(q)
    for i, j_idx in enumerate(arm_dof_indices):
        p.resetJointState(robot_id, j_idx, q_current[i])
        
    ee_state = p.getLinkState(robot_id, tip_index, computeForwardKinematics=True)
    base_pos = np.array(ee_state[0])
    
    # 2. Perturb each joint one by one to find its effect on the end-effector
    for i, j_idx in enumerate(arm_dof_indices):
        # Nudge the joint by 'delta'
        q_current[i] += delta
        p.resetJointState(robot_id, j_idx, q_current[i])
        
        # Get the new position f(q + delta)
        ee_state_new = p.getLinkState(robot_id, tip_index, computeForwardKinematics=True)
        new_pos = np.array(ee_state_new[0])
        
        # Calculate the derivative (change in position / delta)
        # This becomes column 'i' in our Jacobian matrix
        J_arm[:, i] = (new_pos - base_pos) / delta
        
        # Revert the joint to its original angle for the next loop
        q_current[i] -= delta
        p.resetJointState(robot_id, j_idx, q_current[i])
        
    return J_arm

def calculate_jacobian_ik(robot_id, tip_index, target_pos, max_iter=500, tol=1e-3, alpha=0.5):
    """
    Robust Inverse Kinematics using Damped Least Squares (DLS), Numerical Jacobian,
    and Strict Joint Limits to prevent 180-degree flips.
    """
    num_joints = p.getNumJoints(robot_id)
    movable_joints = [i for i in range(num_joints) if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]
            
    arm_dof_indices = [movable_joints.index(i) for i in [0, 1, 2, 3]]
    saved_states = [p.getJointState(robot_id, i)[0] for i in range(num_joints)]
    
    # Extract the current angles of the 4 arm joints
    q = [p.getJointState(robot_id, i)[0] for i in arm_dof_indices]
    
    # --- NEW: Fetch joint limits from the URDF ---
    joint_limits = []
    for j_idx in arm_dof_indices:
        info = p.getJointInfo(robot_id, j_idx)
        lower_limit, upper_limit = info[8], info[9]
        
        # If the URDF doesn't specify limits, give it standard physical limits (-180 to 180 degrees)
        if lower_limit == 0 and upper_limit == 0:
            lower_limit, upper_limit = -np.pi, np.pi
            
        joint_limits.append((lower_limit, upper_limit))
    
    # Override the wrist limit (index 3) to strictly prevent it from flipping backwards.
    # Depending on how your URDF axes are set up, you may need to change this to (-np.pi/2, 0)
    joint_limits[3] = (-np.pi/2, 0)  # Wrist can only bend downwards, never upwards
    
    lambda_sq = 0.04 
    
    for _ in range(max_iter):
        for i, j_idx in enumerate(arm_dof_indices):
            p.resetJointState(robot_id, j_idx, q[i])
            
        ee_state = p.getLinkState(robot_id, tip_index, computeForwardKinematics=True)
        current_pos = np.array(ee_state[0])
        
        error = np.array(target_pos) - current_pos
        if np.linalg.norm(error) < tol:
            break
            
        J_arm = get_numerical_jacobian(robot_id, tip_index, q, arm_dof_indices)
        
        J_arm_T = J_arm.T
        J_pinv = J_arm_T @ np.linalg.inv(J_arm @ J_arm_T + lambda_sq * np.eye(3))
        
        dq = alpha * (J_pinv @ error)
        dq = np.clip(dq, -0.15, 0.15)
        
        for i in range(len(arm_dof_indices)):
            q[i] += dq[i]
            # --- NEW: Clamp the joint angle to its physical limits ---
            q[i] = np.clip(q[i], joint_limits[i][0], joint_limits[i][1])

    for i in range(num_joints):
        p.resetJointState(robot_id, i, saved_states[i])
        
    return q

 # --- IGNORE ---

 # --- USED IN CALIBRATION ---
    """ cvl_robot.create_lego_brick(color=[0, 1, 0], position=( 2.0,  2.0, 0.1)) # 1. Positiv X, Positiv Y
    time.sleep(0.5)
    cvl_robot.create_lego_brick(color=[0, 1, 0], position=( 2.0, -2.0, 0.1)) # 2. Positiv X, Negativ Y
    time.sleep(0.5)
    cvl_robot.create_lego_brick(color=[0, 1, 0], position=(0, -2.0, 0.1)) # 3. Negativ X, Negativ Y
    time.sleep(0.5)
    cvl_robot.create_lego_brick(color=[0, 1, 0], position=(0,  2.0, 0.1)) # 4. Negativ X, Positiv Y """

    """ print(f"Hittade pixlar: X={cX}, Y={cY}")
    print(f"Beräknade världskoordinater: X={world_x:.3f}, Y={world_y:.3f}")
    print(f"FACIT (där vi skapade biten): X=1.32, Y=-0.9") """