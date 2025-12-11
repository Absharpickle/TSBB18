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
    return robot_id


def move_arm(robot_id, base=90, shoulder=90, elbow=90, wrist_pitch=90):
    """
    Move the robot arm. base values are in degrees.

    Parameters:
        robot_id (int): The unique ID of the robot.
        base (float): Base joint angle in degrees.
        shoulder (float): Shoulder joint angle in degrees.
        elbow (float): Elbow joint angle in degrees.
        wrist_pitch (float): Wrist pitch joint angle in degrees.

    Returns:
        None
    """
    p.resetJointState(robot_id, 0, np.deg2rad(base))
    p.resetJointState(robot_id, 1, np.deg2rad(shoulder))
    p.resetJointState(robot_id, 2, np.deg2rad(elbow))
    p.resetJointState(robot_id, 3, np.deg2rad(wrist_pitch))

    for _ in range(240):  # Simulate steps for the motion
        p.stepSimulation()
        time.sleep(1./240.)

def control_claw(robot_id, open_claw=True):
    claw_joint_indices = 7
    angle = 0.17 if open_claw else 1.27
    p.resetJointState(robot_id, claw_joint_indices, angle)
    #p.setJointMotorControl2(robot_id, claw_joint_indices, p.POSITION_CONTROL, angle)
    for _ in range(60):
        p.stepSimulation()
        time.sleep(1./240.)

def state(robot_id):
    """
    Print the world coordinates (position and orientation) of each joint of a robot.

    Args:
        robot_id (int): The ID of the robot loaded in PyBullet.
    """
    
    joint_names = ["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper_base", "gripper_fix", "gripper_movable"]
    num_joints = p.getNumJoints(robot_id) - 1

    for joint_index in range(num_joints):        
        link_state = p.getLinkState(robot_id, joint_index)
        world_position = link_state[0]  # [0] is the world position (x, y, z)
        world_orientation = link_state[1]  # [1] is the orientation (quaternion)

        joint_name = joint_names[joint_index] if joint_index < len(joint_names) else f"Joint {joint_index}"

        print(f"Joint {joint_index} ({joint_name}):")
        print(f"  World Position: {world_position}")
        print(f"  World Orientation (Quaternion): {world_orientation}")


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
    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName="cvl_robot/lego.obj",
        meshScale=scale
    )
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName="cvl_robot/lego.obj",
        meshScale=scale,
        rgbaColor=color + [1]  # Add alpha channel
    )
    brick_id = p.createMultiBody(
        baseMass=0,
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

        # Optional: Step simulation to visualize the change
        for _ in range(240):  # Simulate a short period
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


def capture_image(image_width=640, image_height=480, camera_position=(2, -3, 1), target_position=(2, 0, 0), output_filename="captured_image.png"):
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