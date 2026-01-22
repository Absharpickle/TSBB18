import time
import cvl_robot
import pybullet as p

def main():
    # Initialize the simulation
    cvl_robot.initialize_simulation(gui=True)

    # Create the Braccio robotic arm
    robot_id = cvl_robot.create_braccio_arm(position=(0, 0, 0.02))
      
    # Create a Lego brick        
    brick_id = cvl_robot.create_lego_brick(
        color=[1, 0, 0],
        position=(1.82, 1.82, 0.1)
    )
    
    # Print the world coordinates (position) of each joint
    cvl_robot.state(robot_id)
    
    # test base
    cvl_robot.move_arm(robot_id, base=0, shoulder=45, elbow=0, wrist=0)
    cvl_robot.move_arm(robot_id, base=90, shoulder=45, elbow=0, wrist=0)
    cvl_robot.move_arm(robot_id, base=180, shoulder=45, elbow=0, wrist=0)
    cvl_robot.move_arm(robot_id, base=270, shoulder=45, elbow=0, wrist=0)

    # test shoulder
    cvl_robot.move_arm(robot_id, base=0, shoulder=0,  elbow=0, wrist=0)
    cvl_robot.move_arm(robot_id, base=0, shoulder=45, elbow=0, wrist=0)
    cvl_robot.move_arm(robot_id, base=0, shoulder=90, elbow=0, wrist=0)

    # test elbow
    cvl_robot.move_arm(robot_id, base=0, shoulder=0, elbow=0, wrist=0)
    cvl_robot.move_arm(robot_id, base=0, shoulder=0, elbow=45, wrist=0)
    cvl_robot.move_arm(robot_id, base=0, shoulder=0, elbow=90, wrist=0)
    cvl_robot.move_arm(robot_id, base=0, shoulder=0, elbow=115, wrist=0)

    # test wrist
    cvl_robot.move_arm(robot_id, base=0, shoulder=0, elbow=0, wrist=0)
    cvl_robot.move_arm(robot_id, base=0, shoulder=0, elbow=0, wrist=45)
    cvl_robot.move_arm(robot_id, base=0, shoulder=0, elbow=0, wrist=90)
    cvl_robot.move_arm(robot_id, base=0, shoulder=0, elbow=0, wrist=135)

    # test claw
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
