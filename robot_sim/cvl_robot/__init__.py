import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2

__all__ = ["initialize_simulation", "create_braccio_arm", "move_arm_to_position", "control_claw", "create_lego_brick", "move_brick_to_position", "remove_brick", "capture_image"]

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