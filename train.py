# from pick_place_env import PickPlaceEnv
# from stable_baselines3 import PPO
# from stable_baselines3.common.vec_env import DummyVecEnv
# from stable_baselines3.common.monitor import Monitor
# import os

# def make_env():
#     env = PickPlaceEnv(render=False)
#     env = Monitor(env)
#     return env

# vec_env = DummyVecEnv([make_env])

# # Load the previously trained reach model if available
# model_path = "ppo_reach_robot.zip"
# if os.path.exists(model_path):
#     print("Loading pre-trained reach model...")
#     model = PPO.load(model_path, env=vec_env)
# else:
#     print("Training from scratch...")
#     model = PPO("MlpPolicy", vec_env, verbose=1)

# # Fine-tune on pick-and-place
# model.learn(total_timesteps=500_000)
# model.save("ppo_pick_place_robot")

# vec_env.close()
from pick_place_env import PickPlaceEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
import os

def make_env():
    env = PickPlaceEnv(render=True)
    env = Monitor(env)
    return env

# Wrap the environment in a dummy vectorized environment
vec_env = DummyVecEnv([make_env])

# === Train from scratch ===
model = PPO("MlpPolicy", vec_env, verbose=1)

# Train the agent for 500,000 timesteps
model.learn(total_timesteps=500_000)

# Save the model to disk
model.save("ppo_pick_place_robot")

# Clean up
vec_env.close()
