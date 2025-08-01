import pybullet as p
import pybullet_data
import os
import numpy as np
import time

def setup_pybullet(gui=True):
    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0.4, 0, 0])
    return p.loadURDF("plane.urdf")

def load_so100(base_position=(0, 0, 0), urdf_path="robot/so100.urdf"):
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at {urdf_path}")
    return p.loadURDF(urdf_path, base_position, useFixedBase=True)

def load_object(position=(0.5, 0, 0.02)):
    # Replace with your own object URDF if needed
    return p.loadURDF("cube_small.urdf", basePosition=position)

def get_joint_names(robot_id):
    return [p.getJointInfo(robot_id, i)[1].decode('utf-8') for i in range(p.getNumJoints(robot_id))]

def get_end_effector_index(robot_id):
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        link_name = info[12].decode('utf-8')
        if link_name == "jaw":
            print(f"End effector index: {i}")
            return i
    print("End effector 'jaw' not found!")
    return -1


def move_to_pose(robot_id, end_effector_index, target_pos, target_ori=None):
    if target_ori is None:
        target_ori = p.getQuaternionFromEuler([0, 0, 0])

    joint_positions = p.calculateInverseKinematics(robot_id, end_effector_index, target_pos, target_ori)

    num_joints = p.getNumJoints(robot_id)
    movable_joints = [i for i in range(num_joints) if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]

    print(f"[DEBUG] Applying IK solution to joints:")
    for i, joint_index in enumerate(movable_joints):
        print(f" - Joint {joint_index}: {joint_positions[i]:.3f}")
        p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, joint_positions[i])


def step_sim(n=100):
    for _ in range(n):
        p.stepSimulation()
        time.sleep(1. / 240.)
