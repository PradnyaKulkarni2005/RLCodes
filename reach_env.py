import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import os

class ReachEnv(gym.Env):
    def __init__(self, render=False):
        super(ReachEnv, self).__init__()

        self.render_sim = render
        self.end_effector_index = 5  # Change based on your robot
        self.max_steps = 100
        self.current_step = 0

        if self.render_sim:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.robot_id = None
        self.target_position = [0.4, 0.0, 0.3]
        self.target_uid = None

        # 6 joints control, angle limits: (-π, π)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=np.float32)

        # Observation: joint angles + end-effector position + target position
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(6 + 3 + 3,), dtype=np.float32
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        p.resetSimulation()
        self.current_step = 0
        p.setGravity(0, 0, -9.81)

        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("robot/so100.urdf", useFixedBase=True)

        # Randomize initial joint angles
        self.initial_joint_positions = np.random.uniform(low=-0.5, high=0.5, size=6)
        for i in range(6):
            p.resetJointState(self.robot_id, i, self.initial_joint_positions[i])

        # Place a visual target marker
        self._load_target_visual()

        for _ in range(10):
            p.stepSimulation()

        return self._get_obs(), {}  # Observation, info

    def _load_target_visual(self):
        # Make sure to use a valid URDF file for the target object
        target_urdf_path = os.path.join("robot", "sphere_small.urdf")
        self.target_uid = p.loadURDF(target_urdf_path, basePosition=self.target_position, useFixedBase=True)

    def _get_obs(self):
        joint_states = [p.getJointState(self.robot_id, i)[0] for i in range(6)]
        ee_state = p.getLinkState(self.robot_id, self.end_effector_index)
        ee_pos = ee_state[4]  # world position
        return np.array(joint_states + list(ee_pos) + self.target_position, dtype=np.float32)

    def step(self, action):
        self.current_step += 1

        # Scale action to valid joint angles
        scaled_action = np.clip(action, -1, 1) * np.pi

        for i in range(6):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=scaled_action[i],
                force=200
            )

        for _ in range(10):  # More simulation steps for smoother motion
            p.stepSimulation()
            if self.render_sim:
                time.sleep(1./240.)

        # Get reward
        ee_pos = p.getLinkState(self.robot_id, self.end_effector_index)[4]
        dist = np.linalg.norm(np.array(ee_pos) - np.array(self.target_position))
        reward = -dist

        terminated = dist < 0.05
        truncated = self.current_step >= self.max_steps

        return self._get_obs(), reward, bool(terminated), bool(truncated), {}

    def close(self):
        p.disconnect()
