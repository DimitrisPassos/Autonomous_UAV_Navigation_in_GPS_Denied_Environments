import numpy as np
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
import heapq
import time

COLLISION_DIST = 1.3
TIME_DELTA = 0.3
REPLAN_THRESHOLD = 1.3  # Distance from obstacles to trigger replanning

class AStarPlanner:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.open_list = []
        self.closed_list = set()
        self.parent_map = {}
        self.cost_map = {start: 0}

    def heuristic(self, a, b):
        # Ensure both a and b are valid before calculating the distance
        if None in a or None in b:
            raise ValueError("Invalid values in positions for heuristic calculation")
        return np.linalg.norm(np.array(a) - np.array(b))

    def get_neighbors(self, node):
        neighbors = []
        directions = [
            (-1, 0, 0), (1, 0, 0),  # x-axis
            (0, -1, 0), (0, 1, 0),  # y-axis
            (0, 0, -1), (0, 0, 1)   # z-axis
        ]

        for direction in directions:
            new_node = (int(node[0] + direction[0]), int(node[1] + direction[1]), int(node[2] + direction[2]))
            if (0 <= new_node[0] < self.grid.shape[0] and
                0 <= new_node[1] < self.grid.shape[1] and
                0 <= new_node[2] < self.grid.shape[2]):
                if self.grid[new_node[0], new_node[1], new_node[2]] == 0:  # Free space
                    neighbors.append(new_node)
        return neighbors

    def plan(self):
        heapq.heappush(self.open_list, (0, self.start))
        while self.open_list:
            _, current = heapq.heappop(self.open_list)
            if current in self.closed_list:
                continue
            self.closed_list.add(current)

            if current == self.goal:
                return self.reconstruct_path()

            for neighbor in self.get_neighbors(current):
                new_cost = self.cost_map[current] + 1
                if neighbor not in self.cost_map or new_cost < self.cost_map[neighbor]:
                    self.cost_map[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, self.goal)
                    heapq.heappush(self.open_list, (priority, neighbor))
                    self.parent_map[neighbor] = current

        return None

    def reconstruct_path(self):
        path = []
        current = self.goal
        while current != self.start:
            path.append(current)
            current = self.parent_map[current]
        path.append(self.start)
        return path[::-1]

class GazeboEnv:
    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.laser_scan_data = None
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.octomap_marker = rospy.Subscriber("/occupied_cells_vis_array", MarkerArray, self.occupied_cells)
        self.odom_x = None
        self.odom_y = None
        self.odom_z = None
        self.GRID_SIZE_X = 20
        self.GRID_SIZE_Y = 20
        self.GRID_SIZE_Z = 10
        self.grid = np.zeros((self.GRID_SIZE_X, self.GRID_SIZE_Y, self.GRID_SIZE_Z))
        self.current_path = []  # Initialize current_path to store the planned path

    def occupied_cells(self, marker_array):
        for marker in marker_array.markers:
            for point in marker.points:
                x, y, z = int(point.x), int(point.y), int(point.z)
                self.grid[x][y][z] = 1  # Mark as occupied

    def laser_scan_callback(self, scan):
        self.laser_scan_data = scan.ranges


    def object_proximity_by_laser(self):       
        # Check if any distance in the laser scan data is below the threshold
        for distance in self.laser_scan_data:
            if distance < COLLISION_DIST and distance > 0.1:  # Ignore invalid zero values
                #print(f"Found one and it is {distance}")
                return True
        return False

    def odometry_callback(self, od_data):
        self.odom_x = od_data.pose.pose.position.x
        self.odom_y = od_data.pose.pose.position.y
        self.odom_z = od_data.pose.pose.position.z

    def stop_drone(self):
        velocity_cmd = Twist()
        velocity_cmd.linear.x = 0
        velocity_cmd.linear.y = 0
        velocity_cmd.linear.z = 0
        velocity_cmd.angular.z = 0
        self.vel_pub.publish(velocity_cmd)
        print("Drone stopped. Goal reached.")

    def move_along_path(self):
        """Move along the current planned path with continuous obstacle checking"""
        for waypoint in self.current_path:
            print(f"Moving to waypoint {waypoint}")
            self.move_to_point(waypoint)
            
            # Continuously check for obstacles while moving along the path
            if self.check_trajectory_for_obstacles():
                print("Obstacle detected during movement. Stopping and replanning.")
                self.stop_drone()
                self.replan_path()
                break


    def replan_path(self):
        """Replan the path if an obstacle is detected"""
        current_pos = (self.odom_x, self.odom_y, self.odom_z)
        planner = AStarPlanner(self.grid, current_pos, self.goal)
        self.current_path = planner.plan()

        if self.current_path is None:
            print("Replanning failed, no valid path found.")
            return False
        print("Replanning successful. New path:", self.current_path)
        return True

    def move_to_point(self, point):
        """Move towards the specified point with obstacle checking"""
        target_x, target_y, target_z = point
        while not self.is_goal_reached([self.odom_x, self.odom_y, self.odom_z], point, threshold=0.2):
            velocity_cmd = Twist()
            velocity_cmd.linear.x = target_x - self.odom_x
            velocity_cmd.linear.y = target_y - self.odom_y
            velocity_cmd.linear.z = target_z - self.odom_z

            norm = np.linalg.norm([velocity_cmd.linear.x, velocity_cmd.linear.y, velocity_cmd.linear.z])
            if norm > 1:
                velocity_cmd.linear.x /= norm
                velocity_cmd.linear.y /= norm
                velocity_cmd.linear.z /= norm

            self.vel_pub.publish(velocity_cmd)
            
            # Check for obstacles during movement
            if self.check_trajectory_for_obstacles():
                print(f"Obstacle detected at {self.odom_x}, {self.odom_y}, {self.odom_z}. Stopping.")
                self.stop_drone()
                return False  # Indicate a collision
            rospy.sleep(TIME_DELTA)

        return True 

    def is_goal_reached(self, current_position, goal_position, threshold=COLLISION_DIST + 0.2):
        distance_to_goal = np.linalg.norm(np.array(current_position) - np.array(goal_position))
        print(f"Distance to goal is {distance_to_goal}")
        return distance_to_goal < threshold

    def step(self, goal_pos):
        """Plan and execute a step towards the goal"""
        if self.odom_x is None or self.odom_y is None or self.odom_z is None:
            print("Odometry data is not available yet. Waiting...")
            return

        self.goal = goal_pos  # Set the goal

        current_pos = (self.odom_x, self.odom_y, self.odom_z)
        planner = AStarPlanner(self.grid, current_pos, self.goal)

        while True:
            # Plan the path
            self.current_path = planner.plan()
            print(self.current_path)

            if self.current_path:
                while self.current_path:
                    # Check for obstacles in the path
                    if self.check_trajectory_for_obstacles():
                        print("Replanning due to detected collision.")
                        # Replan the path from the current position
                        current_pos = (self.odom_x, self.odom_y, self.odom_z)
                        goal_x, goal_y, goal_z = self.goal
                        goal_z += 1
                        planner = AStarPlanner(self.grid, current_pos, self.goal)
                        self.current_path = planner.plan()

                        # If no valid path is found after replanning
                        if self.current_path is None:
                            print("Replanning failed, no valid path found.")
                            break
                    else:
                        self.move_along_path()  # No obstacles, proceed with the current path
                        break
            else:
                print("Failed to find a path. Replanning...")
                rospy.sleep(1)



    def check_trajectory_for_obstacles(self):
        for point in self.current_path:
            x, y, z = point
            t_x, t_y, t_z = int(x), int(y), int(z)
            for i in range(int(self.odom_x), t_x + 1):
                for j in range(int(self.odom_y), t_y + 1):
                    for k in range(int(self.odom_z), t_z + 1):
                        if 0 <= i < self.GRID_SIZE_X and 0 <= j < self.GRID_SIZE_Y and 0 <= k < self.GRID_SIZE_Z:
                            if self.grid[i, j, k] == 1:
                                print(f"COLLISION at ({i}, {j}, {k})")
                                return True
        return False

    def replan_if_needed(self):
        if self.check_trajectory_for_obstacles():
            print("Obstacle detected! Replanning...")
            self.stop_drone()
            
            # Update the grid with the new obstacles and replan
            planner = AStarPlanner(self.grid, (self.odom_x, self.odom_y, self.odom_z), self.goal)
            self.current_path = planner.plan()
            
            if self.current_path is None:
                print("Failed to replan a path.")
                return False
            return True
        return False


    def detect_obstacle_nearby(self):
        for point in self.current_path:
            x, y, z = map(int, point)  # Ensure that point coordinates are integers
            if self.grid[x, y, z] == 1:  # Check if grid cell is occupied
                return True
        return False

def main():
    rospy.init_node("uav_navigation", anonymous=True)

    env = GazeboEnv()
    rospy.sleep(2)
    
    try:
        while True:
            user_input = input("Do you want to plan a new path? (y/n): ").strip().lower()
            if user_input == 'n':
                print("Exiting...")
                break
            else:
                x = float(input("Enter target x coordinate: "))
                y = float(input("Enter target y coordinate: "))
                z = float(input("Enter target z coordinate: "))
                goal_pos = (x, y, z)
                
                env.step(goal_pos)  # Start the planning and moving
                rospy.sleep(1)
                
                #env.replan_if_needed()  # Check and replan if needed
            
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected, stopping the UAV.")
        env.stop_drone()
    finally:
        env.stop_drone()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
