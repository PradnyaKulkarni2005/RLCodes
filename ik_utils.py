# ik_utils.py

import pybullet as p
import time
import math

def move_to_pose(robot_id, end_effector_index, target_position):
    # Optional: specify orientation (e.g., gripper facing down)
    orientation = p.getQuaternionFromEuler([0, math.pi, 0])

    # Calculate inverse kinematics to get joint angles
    joint_angles = p.calculateInverseKinematics(
        robot_id,
        end_effector_index,
        target_position,
        targetOrientation=orientation
    )

    print("[DEBUG] Applying IK solution to joints:")
    for i, angle in enumerate(joint_angles):
        print(f" - Joint {i}: {round(angle, 3)}")

    # Apply joint angles to robot
    num_joints = p.getNumJoints(robot_id)

    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_type = joint_info[2]

        # Only move revolute or prismatic joints
        if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            p.setJointMotorControl2(
                robot_id,
                i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_angles[i]
            )

    # Step simulation to let robot move
    for _ in range(200):
        p.stepSimulation()
        time.sleep(1./240.)

    # Print resulting end-effector position
    state = p.getLinkState(robot_id, end_effector_index)
    actual_pos = state[4]
    print("[DEBUG] Final end-effector position:", [round(x, 3) for x in actual_pos])
