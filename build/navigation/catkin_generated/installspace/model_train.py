#!/usr/bin/env python3

import gym
from gym import spaces
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from octomap_msgs.msg import Octomap
import numpy as np

class UAVNavigationEnv(gym.Env):
    def __init__(self):
        super(UAVNavigationEnv, self).__init__()
        
        # Initialize ROS node
        rospy.init_node('uav_rl_environment')

        # Action space: Move in [x, y, z] directions and yaw rotation
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)  # [x, y, z, yaw]

        # Observation space: From OctoMap or any sensor data
        self.observation_space = spaces.Box(low=0, high=1, shape=(64, 64, 64), dtype=np.float32)  # Example grid shape

        # Publishers to control the UAV
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Subscriber for OctoMap or any 3D mapping
        self.octomap_sub = rospy.Subscriber("/octomap_binary", Octomap, self.octomap_callback)
        
        # UAV's current state
        self.current_state = None

        # Rate for publishing commands
        self.rate = rospy.Rate(10)
        
        # Initialize UAV's starting position and reset the environment
        self.reset()

    def octomap_callback(self, msg):
        # Process the octomap data and store it as the current state
        # For simplicity, we use the occupancy grid (3D array)
        self.current_state = self.process_octomap(msg)

    def process_octomap(self, octomap_msg):
        # Convert OctoMap message to a 3D numpy array (simplified for the example)
        # You may need to process it more depending on the actual data format
        return np.random.random((64, 64, 64))  # Placeholder, use actual octomap data

    def reset(self):
        # Reset UAV to initial state
        rospy.loginfo("Resetting the environment")
        self.current_state = np.zeros((64, 64, 64))  # Reset the observation space
        return self.current_state

    def step(self, action):
        # Take action based on the command (e.g., move in x, y, z, or rotate)
        twist = Twist()
        twist.linear.x = action[0]  # Move forward/backward
        twist.linear.y = action[1]  # Move sideways
        twist.linear.z = action[2]  # Move up/down
        twist.angular.z = action[3]  # Rotate (yaw)
        self.cmd_vel_pub.publish(twist)

        # Simulate time delay
        self.rate.sleep()

        # Assume we have access to some reward function and terminal condition
        reward = self.compute_reward()
        done = self.is_done()

        # Return the next observation (OctoMap data)
        observation = self.current_state

        return observation, reward, done, {}

    def compute_reward(self):
        # Simplified reward: You can base this on exploration, distance to goal, etc.
        reward = np.random.random()  # Placeholder reward calculation
        return reward

    def is_done(self):
        # Check if the UAV has reached the goal or hit an obstacle
        done = False  # Placeholder for terminal condition
        return done

    def close(self):
        rospy.signal_shutdown("Closing UAV RL environment")
