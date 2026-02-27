from rustypot import Sts3215PyController

def scan_servos(port='/dev/ttyUSB0', baudrate=1_000_000, max_id=20):
	print(f"Scanning for servos on {port}...")
	
	try:
		controller = Sts3215PyController(serial_port=port, baudrate=baudrate, timeout=0.05)
		
	except Exception as e:
		print(f"Failed to open {port}. Check connection and port name.{nError: {e}}")
		return
		
	found_ids = []
	
	for servo_id in range(1, max_id+1):
		try:
			pos = controller.read_present_position(servo_id)
			print(f"[+] Found active servo at ID {servo_id} | Current position: {pos}")
			found_ids.append(servo_id)
		except Exception:
			pass
			
	print("\n--- Scan Complete ---")
	if found_ids:
		print(f"Found {len(found_ids)} servos. Active IDs: {found_ids}")
		
	else:
		print(f"No servos found.")
		
if __name__ == "__main__":
	scan_servos(port='/dev/ttyUSB0')
