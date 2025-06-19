
from so100_env import SO100PickEnv
from stable_baselines3 import PPO

# Create environment
env = SO100PickEnv(render_mode=True)

# Train the agent
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=50000)

# Test the trained agent
obs = env.reset()
done = False
while not done:
    action, _ = model.predict(obs)
    obs, reward, done, info = env.step(action)
    print(f"Reward: {reward:.3f}")

env.close()