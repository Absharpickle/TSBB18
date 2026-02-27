from rustypot import Sts3215PyController
import time
import math
from limits import *

def set_arm(port='/dev/ttyUSB0', baudrate=1_000_000):
	# Base, Shoulder, Elbow, Wrist, Twist, Claw
	servo_ids = [1, 2, 3, 4, 5, 6]
	
	target_pos = [math.pi/6, 0.0, 0.0, 0.0, 0.0, 0.0]
	
	try:
		print(f"Connecting to servos on {port}")
		arm = Sts3215PyController(serial_port=port, baudrate=baudrate, timeout=0.1)
		print(f"Connection successfull.")
		
		print(f"Moving servos...")
		arm.sync_write_goal_position(servo_ids, target_pos)
		
		time.sleep(2.0)
		
		actual_pos = arm.sync_read_present_position(servo_ids)
		
		print("\n--- Final Positions ---")
		for s_id, pos in zip(servo_ids, actual_pos):
			print(f"Servo {s_id}: {pos:.3f} rad")
			
	except Exception as e:
		print(f"An error occured: {e}")
			
if __name__ == "__main__":
	arm_limits(port='/dev/ttyUSB0')
	set_arm(port='/dev/ttyUSB0')
	#relax_arm(port='/dev/ttyUSB0')
