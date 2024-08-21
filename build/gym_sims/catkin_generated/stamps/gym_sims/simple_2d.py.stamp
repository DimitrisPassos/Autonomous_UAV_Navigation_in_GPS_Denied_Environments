import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math

class SimpleUAVEnv(gym.Env):
    def __init__(self, obstacle_count=3):
        super(SimpleUAVEnv, self).__init__()
        
        # Define action space (move forward, turn left, turn right)
        self.action_space = spaces.Discrete(3)  # 3 actions: forward, turn left, turn right
        
        # Observation space (UAV position and angle)
        self.observation_space = spaces.Box(low=np.array([-10, -10, -np.pi]), 
                                            high=np.array([10, 10, np.pi]), 
                                            dtype=np.float32)
        
        # Define the goal and obstacles
        self.goal = np.array([5.0, 5.0])  # Example goal position
        self.obstacles = [np.array([2.0, 2.0]), np.array([-2.0, -2.0])]  # Static obstacles
        self.obstacle_count = obstacle_count
        self.obstacles = self._generate_random_obstacles(self.obstacle_count)  
        # UAV's position and angle
        self.uav_position = np.array([0.0, 0.0])
        self.uav_angle = 0.0  # Angle in radians
        
        self.max_steps = 100
        self.current_step = 0

    def _generate_random_obstacles(self, count):
        obstacles = []
        for _ in range(count):
            x = np.random.uniform(-9, 9)  # Random x between -9 and 9
            y = np.random.uniform(-9, 9)  # Random y between -9 and 9
            obstacles.append(np.array([x, y]))
        return obstacles

    def reset(self, seed = None, options = None):
        # Reset UAV's position, angle, and steps
        self.uav_position = np.array([0.0, 0.0])
        self.uav_angle = 0.0
        self.current_step = 0
        
        # Regenerate random obstacles at the start of each episode
        self.obstacles = self._generate_random_obstacles(self.obstacle_count)
        
        # Return initial observation and an empty info dictionary
        return self._get_observation(), {}

    def step(self, action):
        self.current_step += 1
        
        # Take action: 0=move forward, 1=turn left, 2=turn right
        if action == 0:
            self._move_forward()
        elif action == 1:
            self.uav_angle -= 0.1  # Turn left
        elif action == 2:
            self.uav_angle += 0.1  # Turn right

        # Get the new observation (UAV's position, angle)
        observation = self._get_observation()

        # Compute reward
        reward = self._compute_reward()

        # Check if the episode is done
        done = self._check_done()

        # Return observation, reward, done, and an empty info dictionary
        return observation, reward, done, {}

    def _move_forward(self):
        # Move the UAV forward in the direction of its current angle
        self.uav_position[0] += np.cos(self.uav_angle) * 0.1
        self.uav_position[1] += np.sin(self.uav_angle) * 0.1

    def _get_observation(self):
        # Return UAV's position and angle
        return np.array([self.uav_position[0], self.uav_position[1], self.uav_angle])

    def _compute_reward(self):
        # Reward is based on the distance to the goal
        distance_to_goal = np.linalg.norm(self.uav_position - self.goal)
        reward = -distance_to_goal  # Negative reward for being farther from the goal

        # Penalty for collisions with obstacles
        for obstacle in self.obstacles:
            if np.linalg.norm(self.uav_position - obstacle) < 0.5:  # Collision radius
                reward -= 100  # Big penalty for collision

        # Reward for reaching the goal
        if distance_to_goal < 0.5:
            reward += 100  # Large reward for reaching the goal

        return reward

    def _check_done(self):
        # Done if UAV reaches the goal or the maximum number of steps is reached
        if np.linalg.norm(self.uav_position - self.goal) < 0.5:
            return True
        if self.current_step >= self.max_steps:
            return True
        return False

    def render(self, mode='human'):
        # Simple text rendering
        print(f"UAV Position: {self.uav_position}, UAV Angle: {self.uav_angle}, Obstacles: {self.obstacles}")

# To use this environment, register it with Gymnasium
gym.envs.registration.register(
    id='SimpleUAV-v0',
    entry_point='path.to.this.file:SimpleUAVEnv',  # Change this path to your file's path
    max_episode_steps=100,
)
