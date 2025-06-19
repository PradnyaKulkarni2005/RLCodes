import gym
import pybullet as p
import pybullet_data
import numpy as np
import time
from gym import spaces
import math
import os

class SO100PickEnv(gym.Env):
    def __init__(self, render_mode=False):
        super(SO100PickEnv, self).__init__()
        self.render_mode = render_mode
        self.time_step = 1.0 / 240.0
        self.max_steps = 200
        self.step_counter = 0
        self.urdf_path = "D:/robotics/so100.urdf"

        self._connect()

        # Action: delta x, y, z for gripper
        self.action_space = spaces.Box(low=-0.05, high=0.05, shape=(3,), dtype=np.float32)

        # Observation: gripper_pos (3), cube_pos (3)
        obs_low = np.array([-1, -1, 0, -1, -1, 0])
        obs_high = np.array([1, 1, 1, 1, 1, 1])
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)

    def _connect(self):
        if self.render_mode:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)

        self.plane = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(self.urdf_path, useFixedBase=True)

        self.joint_indices = [0, 1, 2, 3]
        self.gripper_left = 4
        self.gripper_right = 5
        self.end_effector_index = 3

        # Random cube position
        x = np.random.uniform(0.20, 0.30)
        y = np.random.uniform(-0.05, 0.05)
        self.cube_pos = [x, y, 0.02]
        self.cube_id = p.loadURDF("cube_small.urdf", basePosition=self.cube_pos)
        p.changeDynamics(self.cube_id, -1, lateralFriction=1.0, rollingFriction=0.01)

        # Open gripper and move to hover
        self._control_gripper(open=True)
        self._move_arm_to([x, y, 0.1])

        self.step_counter = 0
        self.has_grasped = False
        return self._get_obs()

    def _get_obs(self):
        gripper_pos = np.array(p.getLinkState(self.robot, self.end_effector_index)[0])
        cube_pos = np.array(p.getBasePositionAndOrientation(self.cube_id)[0])
        
        obs = np.concatenate([gripper_pos, cube_pos])
        return obs.astype(np.float32)


    
    def step(self, action):
        self.step_counter += 1

        # Clip the action to limit movement per step
        action = np.clip(action, -0.05, 0.05)

        # Get current end-effector position
        current_pos = np.array(p.getLinkState(self.robot, self.end_effector_index)[0])
        new_pos = current_pos + action
        new_pos = np.clip(new_pos, [0.1, -0.2, 0.02], [0.4, 0.2, 0.3])
        self._move_arm_to(new_pos)

        # Get cube position
        cube_pos = np.array(p.getBasePositionAndOrientation(self.cube_id)[0])
        xy_dist = np.linalg.norm(new_pos[:2] - cube_pos[:2])
        z_dist = abs(new_pos[2] - cube_pos[2])

        reward = -xy_dist  # penalize distance in XY
        done = False

        if not self.has_grasped and xy_dist < 0.03 and z_dist < 0.05:
            # Try to grasp
            self._move_arm_to([cube_pos[0], cube_pos[1], cube_pos[2] + 0.005])
            self._control_gripper(open=False)
            time.sleep(0.2)

            # Attempt to lift
            self._move_arm_to([cube_pos[0], cube_pos[1], 0.2])
            time.sleep(0.2)

            # Check if cube was lifted
            lifted_z = p.getBasePositionAndOrientation(self.cube_id)[0][2]
            if lifted_z > 0.05:  # cube lifted from table
                reward += 10.0
                self.has_grasped = True
                done = True

        # Episode end condition
        if self.step_counter >= self.max_steps:
            done = True

        return self._get_obs(), reward, done, {}

    def _move_arm_to(self, position):
        orn = p.getQuaternionFromEuler([0, math.pi, 0])
        joint_angles = p.calculateInverseKinematics(self.robot, self.end_effector_index, position, orn)
        for i, j in enumerate(self.joint_indices):
            p.setJointMotorControl2(self.robot, j, p.POSITION_CONTROL, joint_angles[i])
        for _ in range(30):
            p.stepSimulation()
            if self.render_mode:
                time.sleep(self.time_step)

    def _control_gripper(self, open=True):
        target = 0.04 if open else 0.0  # Adjust if your gripper closes tighter
        p.setJointMotorControl2(self.robot, self.gripper_left, p.POSITION_CONTROL, targetPosition=target, force=100)
        p.setJointMotorControl2(self.robot, self.gripper_right, p.POSITION_CONTROL, targetPosition=target, force=100)
        for _ in range(10):  # allow some time for fingers to move
            p.stepSimulation()
            if self.render_mode:
                time.sleep(self.time_step)

    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()

