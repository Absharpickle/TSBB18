import time
import cvl_robot
import pybullet as p

def main():
    # Initialize the simulation
    cvl_robot.initialize_simulation(gui=True)

    # Create the Braccio robotic arm
    robot_id = cvl_robot.create_braccio_arm(position=(0, 0, 0.2))
      
    # Create a Lego brick        
    brick_id = cvl_robot.create_lego_brick(
        color=[1, 0, 0],
        position=(1.82, 1.82, 0.1)
    )

    # Print the world coordinates (position and orientation) of each joint
    cvl_robot.state(robot_id)
    
    cvl_robot.move_arm(robot_id, base=0.7, shoulder=0.6, elbow=1.0, wrist_pitch=0.8)
    cvl_robot.control_claw(robot_id,open_claw=False)
    cvl_robot.control_claw(robot_id,open_claw=True)
    cvl_robot.state(robot_id)

    cvl_robot.move_brick_to_position(brick_id,[1.82, 1.82, 1])

    # Capture an image
    rgb_image = cvl_robot.capture_image()
    cvl_robot.remove_brick(brick_id)

    while(True):
        p.stepSimulation()
        time.sleep(0.01)
    p.disconnect()

if __name__ == "__main__":
    main()
