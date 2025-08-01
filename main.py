# main.py

from reach_env import ReachEnv
from stable_baselines3 import PPO
import time

# Load environment with GUI
env = ReachEnv(render=True)

# Load trained model
model = PPO.load("ppo_reach_robot")

# Reset environment (Gymnasium returns obs, info)
obs, _ = env.reset()
terminated = False
truncated = False

while not (terminated or truncated):
    action, _ = model.predict(obs)
    
    # Gymnasium step returns 5 values
    obs, reward, terminated, truncated, _ = env.step(action)

    time.sleep(1. / 60.)  # Adjust sim speed for visibility

env.close()
