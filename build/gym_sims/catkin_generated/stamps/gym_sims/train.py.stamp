import gymnasium as gym
from stable_baselines3 import PPO
import torch

# Step 1: Train the model in a simple environment
env = gym.make("MountainCarContinuous-v0")  # A simple continuous control environment

# Initialize the PPO model
model = PPO("MlpPolicy", env, verbose=1)

# Train the model for 10,000 timesteps
model.learn(total_timesteps=10000)

# Save the trained model
model.save("ppo_uav_model")
print("Model trained and saved to ppo_uav_model.zip")

# Step 2: Extract the trained model's policy parameters (weights)
policy_parameters = model.policy.state_dict()

# Save the parameters to a file
torch.save(policy_parameters, "ppo_uav_policy_params.pth")
print("Policy parameters saved to ppo_uav_policy_params.pth")

# Step 3: Reload the trained model for UAV
# You don't need to redefine the model, just reload the trained model

# Reload the model
loaded_model = PPO.load("ppo_uav_model")

# Alternatively, you can load the parameters directly
loaded_parameters = torch.load("ppo_uav_policy_params.pth")

# Check if parameters loaded correctly
loaded_model.policy.load_state_dict(loaded_parameters)
print("Model parameters loaded from ppo_uav_policy_params.pth")

# Step 4: Test the loaded model in the same environment
obs, _ = env.reset()

for step in range(1000):
    # Predict action using the loaded model
    action, _ = loaded_model.predict(obs, deterministic=True)

    # Take the action in the environment
    obs, reward, done, info = env.step(action)

    # Render the environment (optional for visualization)
    env.render()

    if done:
        obs, _ = env.reset()

env.close()

print("Test run with the loaded model is complete.")
