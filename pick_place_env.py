import numpy as np
from reach_env import ReachEnv
import gymnasium
from gymnasium import spaces

class PickPlaceEnv(ReachEnv):
    def __init__(self, render=True):
        super(PickPlaceEnv, self).__init__()
        self.n_joints = 6  # Set based on your robot arm (not including gripper)
        
        # Then define action_space
        self.action_space = spaces.Box(
            low=np.array([-1.0] * self.n_joints + [-1.0]),  # last -1.0 for gripper open/close
            high=np.array([1.0] * self.n_joints + [1.0]),
            dtype=np.float32
        )


        # Add cube state (position only for now) to observation
        obs_dim = self.observation_space.shape[0] + 3  # 3D pos of cube
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        self.cube_pos = np.array([0.1, 0.0, 0.03])  # initial cube location

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # Randomize cube and goal positions
        self.cube_pos = np.array([0.1, 0.0, 0.03]) + 0.02 * (np.random.rand(3) - 0.5)
        self.goal_pos = np.array([0.2, 0.1, 0.05]) + 0.02 * (np.random.rand(3) - 0.5)

        obs = self._get_obs()
        return obs, {}  # gymnasium expects a tuple: (obs, info)


    def step(self, action):
        arm_action = action[:-1]
        gripper_action = action[-1]

        # Call parent step and unpack all 5 values
        obs, _, terminated, truncated, info = super().step(arm_action)

        # Control gripper
        self._control_gripper(gripper_action)

        # Compute custom reward and success flag
        reward, success = self.custome_reward()

        # Repack values properly for Gymnasium (use terminated + truncated)
        return self._get_obs(), reward, terminated, truncated, {"success": success}


    def _get_obs(self):
        base_obs = super()._get_obs()
        return np.concatenate([base_obs, self.cube_pos])

    def _control_gripper(self, grip_cmd):
        """
        Converts grip_cmd into a command for the gripper joint.
        For example: -1 = open, 1 = close.
        You need to implement this in your Mujoco/PyBullet sim.
        """
        pass  # You'll control the 'gripper' joint defined in your URDF here

    def custome_reward(self):
        ee_pos = self._get_ee_position()
        cube = self.cube_pos
        goal = self.goal_pos

        dist_to_cube = np.linalg.norm(ee_pos - cube)
        cube_to_goal = np.linalg.norm(cube - goal)

        reward = -dist_to_cube  # Encourage reaching
        success = False

        if dist_to_cube < 0.05:
            reward += 1  # Bonus for touching cube

            if self._gripper_closed_near_cube():
                reward += 2  # Bonus for grasping

                if cube[2] > 0.08:  # lifted
                    reward += 3

                    if cube_to_goal < 0.05:
                        reward += 5
                        success = True

        done = False
        return reward, success

    def _get_ee_position(self):
        """Fetch end-effector position from sim"""
        # This should return the 3D position of the robot's end effector
        return np.array([0, 0, 0])  # replace with real implementation

    def _gripper_closed_near_cube(self):
        """Checks if gripper is closed and near cube"""
        # Replace with logic based on joint state / contact sensors
        return True  # placeholder
