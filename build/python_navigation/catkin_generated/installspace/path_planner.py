#!/usr/bin/env python3

import rospy
from octomap_msgs.msg import Octomap
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
import numpy as np
import heapq

# Node class for A* algorithm
class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to node
        self.h = 0  # Heuristic cost to goal
        self.f = 0  # Total cost

    def __lt__(self, other):
        return self.f < other.f

# A* Algorithm
def astar(grid, start, goal):
    open_list = []
    closed_list = set()

    start_node = Node(start)
    goal_node = Node(goal)

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == goal_node.position:
            # Path found, trace back to start
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        closed_list.add(current_node.position)

        # 6 possible directions in 3D space (x, y, z)
        neighbors = [
            (1, 0, 0), (-1, 0, 0),
            (0, 1, 0), (0, -1, 0),
            (0, 0, 1), (0, 0, -1)
        ]

        for offset in neighbors:
            neighbor_pos = (
                current_node.position[0] + offset[0],
                current_node.position[1] + offset[1],
                current_node.position[2] + offset[2]
            )

            # Check bounds and obstacles
            if (
                0 <= neighbor_pos[0] < grid.shape[0] and
                0 <= neighbor_pos[1] < grid.shape[1] and
                0 <= neighbor_pos[2] < grid.shape[2] and
                grid[neighbor_pos] == 0  # Free cell
            ):
                if neighbor_pos in closed_list:
                    continue

                neighbor_node = Node(neighbor_pos, current_node)
                neighbor_node.g = current_node.g + 1
                neighbor_node.h = np.linalg.norm(np.array(neighbor_pos) - np.array(goal_node.position))  # Euclidean dist
                neighbor_node.f = neighbor_node.g + neighbor_node.h

                # Check if this path is better than the existing one
                if any(open_node for open_node in open_list if open_node.position == neighbor_pos and open_node.f <= neighbor_node.f):
                    continue

                heapq.heappush(open_list, neighbor_node)

    return None  # No path found

# Path Planner class
class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)

        # Subscriptions and publishers
        self.octomap_sub = rospy.Subscriber("/octomap_binary", Octomap, self.octomap_callback)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Placeholder for the UAV's state
        self.grid = None
        self.start_position = None
        self.goal_position = None

        rospy.spin()

    def octomap_callback(self, msg):
        rospy.loginfo("Received Octomap data")
        self.grid = self.process_octomap(msg)

    def process_octomap(self, msg):
        # Convert Octomap to 3D grid; this is a placeholder, actual logic needed
        grid = np.zeros((10, 10, 10))  # Empty grid for simplicity
        grid[3:5, 3:5, 3:5] = 1  # Example obstacle, replace with Octomap processing
        return grid

    def pose_callback(self, msg):
        self.start_position = (int(msg.pose.pose.position.x), int(msg.pose.pose.position.y), int(msg.pose.pose.position.z))
        rospy.loginfo(f"Start position: {self.start_position}")

    def goal_callback(self, msg):
        self.goal_position = (int(msg.pose.position.x), int(msg.pose.position.y), int(msg.pose.position.z))
        rospy.loginfo(f"Goal position: {self.goal_position}")

        if self.grid is not None and self.start_position is not None and self.goal_position is not None:
            self.plan_path()

    def plan_path(self):
        path = astar(self.grid, self.start_position, self.goal_position)

        if path:
            rospy.loginfo(f"Path found: {path}")
            self.follow_path(path)
        else:
            rospy.loginfo("No path found")

    def follow_path(self, path):
        for position in path:
            twist = Twist()
            twist.linear.x = position[0] - self.start_position[0]
            twist.linear.y = position[1] - self.start_position[1]
            twist.linear.z = position[2] - self.start_position[2]
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(1)

# Main function
if __name__ == "__main__":
    try:
        planner = PathPlanner()
    except rospy.ROSInterruptException:
        pass
