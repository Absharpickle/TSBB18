from rustypot import Sts3215PyController

__all__ = ["arm_limits", "relax_arm"]

def arm_limits(port='/dev/ttyUSB0', baudrate=1_000_000):
	servo_ids = [1, 2, 3, 4, 5, 6]
	
	try:
		arm = Sts3215PyController(serial_port=port, baudrate=baudrate, timeout=0.2)
		print("Connected to arm, configuring limits...")
		
		conf_torque = 400
		for s_id in servo_ids:
			arm.write_torque_limit(s_id, conf_torque)
			
		conf_speed = 500
		for s_id in servo_ids:
			arm.write_goal_speed(s_id, conf_speed)
			
		conf_acc = 50
		for s_id in servo_ids:
			arm.write_goal_acceleration(s_id, conf_acc)
			
		print("Successfully configured limits for all servos.")
		print(f"-> Max Torque: {conf_torque}")
		print(f"-> Max Speed: {conf_speed}")
		print(f"-> Max Acceleration: {conf_acc}")
		
	except Exception as e:
		print(f"Error configuring arm: {e}")
		
def relax_arm(port='/dev/ttyUSB0', baudrate=1_000_000):
	servo_ids = [1, 2, 3, 4, 5, 6]
	
	try:
		arm = Sts3215PyController(serial_port=port, baudrate=baudrate, timeout=0.1)
		
		for s_id in servo_ids:
			arm.write_torque_enable(s_id, False)
			
	except Exception as e:
		print(f"Error: {e}")
