import time
import math
import numpy as np
from rustypot import Sts3215PyController
from image_processing.image_processing import detect_bricks

# ─── CONFIGURATION ────────────────────────────────────────────────────────────

PORT     = "/dev/ttyUSB0"
BAUDRATE = 1_000_000

SAFE_TRANSIT_Z    = 1.6
FLOOR_Z_Y_MAX     = 0.35
FLOOR_Z_Y_MIN     = 0.15
PARALLAX_PULLBACK = -0.1

Y_MIN = 2.0
Y_MAX = 3.0

ARM_SERVO_IDS = [1, 2, 3, 4]
ALL_SERVO_IDS = [1, 2, 3, 4, 5, 6]
CLAW_ID       = 6
CLAW_OPEN     = -0.4
CLAW_CLOSED   = 0.6
MAX_VEL       = 1.4

DROP_ZONES = {
    "Red":    (2.5,  -0.5),
    "Green":  (-2.5, 1.2),
    "Blue":   (2.5, 1.2),
    "Yellow": (-2.5, -0.5),
}

# ─── FORWARD KINEMATICS ───────────────────────────────────────────────────────

L1 = 0.063 * 10
L2 = 0.103 * 10
L3 = 0.103 * 10
L4 = 0.143 * 10

def forward_kinematics(q):
    base, shoulder, elbow, wrist = q
    shoulder = -shoulder
    wrist = -wrist
    vertical   = (L2 * math.cos(shoulder)
                + L3 * math.cos(shoulder + elbow)
                + L4 * math.cos(shoulder + elbow + wrist))
    horizontal = (L2 * math.sin(shoulder)
                + L3 * math.sin(shoulder + elbow)
                + L4 * math.sin(shoulder + elbow + wrist))
    y = horizontal * math.cos(base)
    x = horizontal * math.sin(base)
    z = L1 + vertical
    return np.array([x, y, z])

# ─── IK ───────────────────────────────────────────────────────────────────────

def get_numerical_jacobian(q, delta=1e-4):
    J = np.zeros((3, 4))
    for i in range(4):
        q_fwd = list(q); q_fwd[i] += delta
        q_bwd = list(q); q_bwd[i] -= delta
        J[:, i] = (forward_kinematics(q_fwd) - forward_kinematics(q_bwd)) / (2 * delta)
    return J


def calculate_jacobian_ik(target_pos, max_iter=500, tol=1e-3, alpha=0.5):
    seed = math.atan2(target_pos[1], target_pos[0])
    
    q = list(current_q)

    joint_limits = [
        (-math.pi, math.pi),
        (-1.4,     1.4),
        (-2.6,     2.6),
        (-math.pi, math.pi),
    ]

    for _ in range(max_iter):
        error      = np.array(target_pos) - forward_kinematics(q)
        error_norm = np.linalg.norm(error)
        if error_norm < tol:
            break

        J              = get_numerical_jacobian(q)
        manipulability = np.sqrt(max(0.0, np.linalg.det(J @ J.T)))
        lambda_sq      = 0.04 if manipulability > 0.01 else 0.1
        J_pinv         = J.T @ np.linalg.inv(J @ J.T + lambda_sq * np.eye(3))

        dq = np.clip(alpha * (J_pinv @ error), -0.15, 0.15)
        for i in range(4):
            q[i] = np.clip(q[i] + dq[i], joint_limits[i][0], joint_limits[i][1])

    return q

# ─── HELPING FUNCTIONS ────────────────────────────────────────────────────────

def get_floor_z(world_y):
    t = (world_y - Y_MIN) / (Y_MAX - Y_MIN)
    t = max(0.0, min(1.0, t))
    return FLOOR_Z_Y_MIN + t * (FLOOR_Z_Y_MAX-FLOOR_Z_Y_MIN)
    
def check_z(floor_z, world_x, world_y):
    #if world_y > 2.5:
        #floor_z += 0.1
        
    if 0.5 < world_x < 1.0:
        floor_z += 0.1
        
    elif -1.0 < world_x < -0.5:
        floor_z += 0.15
        
    elif -1.5 < world_x < -1.0 or 1.0 < world_x < 1.5:
        floor_z += 0.15
        
    elif -0.5 < world_x < 0.5:
        floor_z += 0.03 * world_y
    
    return floor_z
    
# ─── ARM CONTROL ──────────────────────────────────────────────────────────────

current_q = [0.0, 1.0, 1.8, 0.0]


def move_arm_physical(arm, q):
    global current_q
    current   = arm.sync_read_present_position(ARM_SERVO_IDS)
    max_delta = max(abs(q[i] - current[i]) for i in range(4))
    wait_time = max(0.1, (max_delta / MAX_VEL) * 1.15)
    arm.sync_write_goal_position(ARM_SERVO_IDS, q)
    time.sleep(wait_time)
    current_q = q


def open_claw(arm):
    arm.sync_write_goal_position([CLAW_ID], [CLAW_OPEN])
    time.sleep(0.1)


def close_claw(arm):
    arm.sync_write_goal_position([CLAW_ID], [CLAW_CLOSED])
    time.sleep(0.6)


def move_to_xyz(arm, target_x, target_y, target_z, apply_parallax=False):
    global current_q

    if apply_parallax:
        yaw       = math.atan2(target_y, target_x)
        target_x -= PARALLAX_PULLBACK * math.cos(yaw)
        target_y -= PARALLAX_PULLBACK * math.sin(yaw)

    current_tcp = forward_kinematics(current_q)
    current_z   = current_tcp[2]

    if current_z < SAFE_TRANSIT_Z:
        lift_q = calculate_jacobian_ik([current_tcp[0], current_tcp[1], SAFE_TRANSIT_Z])
        move_arm_physical(arm, lift_q)
        current_tcp = forward_kinematics(current_q)

    xy_distance = math.sqrt((target_x - current_tcp[0])**2 + (target_y - current_tcp[1])**2)
    if xy_distance > 0.01:
        transit_z = max(target_z, SAFE_TRANSIT_Z)
        swing_q   = calculate_jacobian_ik([target_x, target_y, transit_z])
        move_arm_physical(arm, swing_q)

    if target_z < SAFE_TRANSIT_Z:
        descent_q = calculate_jacobian_ik([target_x, target_y, target_z])
        move_arm_physical(arm, descent_q)


def pick_up_lego(arm, world_x, world_y):
    open_claw(arm)
    floor_z = get_floor_z(world_y)
    floor_z = check_z(floor_z, world_x, world_y)
    #move_to_xyz(arm, world_x, world_y, target_z=SAFE_TRANSIT_Z, apply_parallax=True)
    move_arm_physical(arm, [0.0, 0.0, 1.0, -1.0])
    move_to_xyz(arm, world_x, world_y, target_z=floor_z, apply_parallax=True)
    close_claw(arm)


def drop_lego(arm, storage_x, storage_y):
    #move_to_xyz(arm, storage_x, storage_y, target_z=SAFE_TRANSIT_Z)
    move_to_xyz(arm, storage_x, storage_y, target_z=0.3)
    open_claw(arm)


def home_arm(arm):
    print("Returning to home position...")
    current_tcp = forward_kinematics(current_q)
    if current_tcp[2] < SAFE_TRANSIT_Z:
        lift_q = calculate_jacobian_ik([current_tcp[0], current_tcp[1], SAFE_TRANSIT_Z])
        move_arm_physical(arm, lift_q)
    move_arm_physical(arm, [0.0, 0.0, 0.0, 0.0])
    print("Home position reached.")


def arm_limits(arm):
    try:
        print("Connected to arm, configuring limits...")
        for s_id in ALL_SERVO_IDS:
            arm.write_torque_limit(s_id, 400)
            #arm.write_goal_speed(s_id, 10)
            #arm.write_goal_acceleration(s_id, 20)
    except Exception as e:
        print(f"Error configuring arm: {e}")


def relax_arm(port=PORT, baudrate=BAUDRATE):
    try:
        arm = Sts3215PyController(serial_port=port, baudrate=baudrate, timeout=0.1)
        for s_id in ALL_SERVO_IDS:
            arm.write_torque_enable(s_id, False)
        print("Arm relaxed.")
    except Exception as e:
        print(f"Error relaxing arm: {e}")
