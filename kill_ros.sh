#!/bin/bash

# Kill ROS and Gazebo processes
echo "Killing roscore..."
killall -9 roscore

echo "Killing rosmaster..."
killall -9 rosmaster

echo "Killing gzclient..."
killall -9 gzclient

echo "Killing gzserver..."
killall -9 gzserver

echo "ROS and Gazebo processes terminated."
