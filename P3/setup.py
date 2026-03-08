import time
import math
import numpy as np
from rustypot import Sts3215PyController

# ─── CONFIGURATION ────────────────────────────────────────────────────────────

PORT     = "/dev/ttyUSB0"
BAUDRATE = 1_000_000

SAFE_TRANSIT_Z    = 0.75
FLOOR_Z           = 0.3694
PARALLAX_PULLBACK = 0.18

ARM_SERVO_IDS = [1, 2, 3, 4]
ALL_SERVO_IDS = [1, 2, 3, 4, 5, 6]
CLAW_ID       = 6
CLAW_OPEN     =  0.5
CLAW_CLOSED   = -0.5
MAX_VEL       = 0.5

DROP_ZONES = {
    "Red":    (1.8,  1.5),
    "Green":  (1.8,  0.5),
    "Blue":   (1.8, -0.5),
    "Yellow": (1.8, -1.5),
}

# ─── FORWARD KINEMATICS ───────────────────────────────────────────────────────

L1 = 0.063
L2 = 0.103
L3 = 0.103
L4 = 0.143

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
    x = horizontal * math.cos(base)
    y = horizontal * math.sin(base)
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
    q    = [seed * 0.5, 1.0, 1.8, seed * -0.5]

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

# ─── ARM CONTROL ──────────────────────────────────────────────────────────────

current_q = [0.0, 1.0, 1.8, 0.0]


def move_arm_physical(arm, q):
    global current_q
    current   = arm.sync_read_present_position(ARM_SERVO_IDS)
    max_delta = max(abs(q[i] - current[i]) for i in range(4))
    wait_time = max(1.0, (max_delta / MAX_VEL) * 1.25)
    arm.sync_write_goal_position(ARM_SERVO_IDS, q)
    time.sleep(wait_time)
    current_q = q


def open_claw(arm):
    arm.sync_write_goal_position([CLAW_ID], [CLAW_OPEN])
    time.sleep(0.8)


def close_claw(arm):
    arm.sync_write_goal_position([CLAW_ID], [CLAW_CLOSED])
    time.sleep(0.8)


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
    move_to_xyz(arm, world_x, world_y, target_z=0.60, apply_parallax=True)
    move_to_xyz(arm, world_x, world_y, target_z=FLOOR_Z, apply_parallax=True)
    close_claw(arm)
    time.sleep(0.5)


def drop_lego(arm, storage_x, storage_y):
    move_to_xyz(arm, storage_x, storage_y, target_z=0.80)
    move_to_xyz(arm, storage_x, storage_y, target_z=FLOOR_Z)
    open_claw(arm)
    move_to_xyz(arm, storage_x, storage_y, target_z=SAFE_TRANSIT_Z)


def home_arm(arm):
    print("Returning to home position...")
    current_tcp = forward_kinematics(current_q)
    if current_tcp[2] < SAFE_TRANSIT_Z:
        lift_q = calculate_jacobian_ik([current_tcp[0], current_tcp[1], SAFE_TRANSIT_Z])
        move_arm_physical(arm, lift_q)
    move_arm_physical(arm, [0.0, 0.0, 0.0, 0.0])
    print("Home position reached.")


def arm_limits(port=PORT, baudrate=BAUDRATE):
    try:
        arm = Sts3215PyController(serial_port=port, baudrate=baudrate, timeout=0.2)
        print("Connected to arm, configuring limits...")
        for s_id in ALL_SERVO_IDS:
            arm.write_torque_limit(s_id, 400)
            arm.write_goal_speed(s_id, 500)
            arm.write_goal_acceleration(s_id, 50)
        print("-> Max Torque: 400 | Max Speed: 500 | Max Acceleration: 50")
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
