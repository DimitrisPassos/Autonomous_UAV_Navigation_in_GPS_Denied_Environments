
import sys
import os

sys.path.append('~/thesis/src/navigation/src')
print(sys.path)
import gymnasium as gym
from stable_baselines3 import PPO
from model_train import UAVNavigationEnv
from stable_baselines3.common.vec_env import DummyVecEnv


from stable_baselines3 import PPO
import gymnasium as gym  # This is your custom Gazebo-ROS environment

# # Load the trained 2D model
# model = PPO.load("ppo_simple_2d")

# # Create the 3D environment
# env = UAVNavigationEnv()

# # Test the model in the new environment
# obs = env.reset()
# done = False

# while not done:
#     action, _ = model.predict(obs)
#     obs, reward, done, _, _ = env.step(action)
#     env.render()

# env.close()
# Create the environment
env = DummyVecEnv([lambda: UAVNavigationEnv()])

# Define the RL model (using PPO as an example)
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
model.learn(total_timesteps=10000)

# Save the model
model.load("ppo_simple_2d.zip")

# Test the model after training
obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs)
    obs, reward, done, info = env.step(action)
    if done:
        obs = env.reset()

# Close the environment
env.close()
