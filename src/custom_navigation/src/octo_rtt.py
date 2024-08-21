#! /usr/bin/env python

import rospy
from ompl import base as ob
from ompl import geometric as og
from geometry_msgs.msg import PoseStamped, Twist, Transform
from visualization_msgs.msg import MarkerArray
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import numpy as np
import fcl
from fcl import Box
import faulthandler
faulthandler.enable()
import time
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Empty
from scipy.interpolate import CubicSpline
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PoseStamped
from nav_msgs.msg import Path

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        if dt == 0:  # Prevent division by zero
            dt = 0.001  # Assign a small time step to avoid zero division

        # Proportional term
        proportional = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.Ki * self.integral
        
        # Derivative term
        derivative = self.Kd * (error - self.prev_error) / dt
        self.prev_error = error

        # Return the total control action
        return proportional + integral + derivative





class Planner:
    def __init__(self):
        rospy.init_node('path_planning_node')
        self.pid_x = PID(0.8, 0.0, 0.2); self.pid_y = PID(0.8, 0.0, 0.4); self.pid_z = PID(1.0, 0.0, 0.2); self.pid_yaw = PID(0.5, 0.0, 1.2)

        self.prev_goal = [0.0] * 7
        self.min_bounds = [-30.0, -30.0, 0.0]
        self.max_bounds = [30.0, 20.0, 10.0]
        self.uav_position = [0, 0, 0]
        self.quadrotor = Box(0.80, 0.80, 0.2)
        self.sonar_height = None
        self.odom_rotate = None
        self.last_time = rospy.Time.now().to_sec()
        self.obstacle_list = set()
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        rospy.Subscriber("/occupied_cells_vis_array", MarkerArray, self.occupied_cells_callback)
        self.trajectory_pub = rospy.Publisher('/trajectory', Path, queue_size=10)
        rospy.Subscriber("/sonar_height", Range, self.sonar_height_callback, queue_size=20)
        
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.objects = set()
        self.space = ob.SE3StateSpace()
        bounds = ob.RealVectorBounds(3)

        for i in range(3):
            bounds.setLow(i, self.min_bounds[i])
            bounds.setHigh(i, self.max_bounds[i])

        self.space.setBounds(bounds)

        self.si = ob.SpaceInformation(self.space)
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        self.pdef = ob.ProblemDefinition(self.si)
        if self.uav_position[2] == None or self.uav_position[2] < 0.5:
            self.takeoff()

    def sonar_height_callback(self, msg):
        self.sonar_height = msg.range


    def should_add_collision_object(self):
        if self.objects is None:
            return False
        if len(self.objects) == len(self.obstacle_list):
            return False
        return True

    def add_obstacle(self, x, y, z):
        """Add an obstacle and precompute its FCL collision object."""
        if self.should_add_collision_object():
            obstacle_geometry = fcl.Box(1.0, 1.0, 1.0)
            obstacle_transform = fcl.Transform([x, y, z])
            obstacle_collision_object = fcl.CollisionObject(obstacle_geometry, obstacle_transform)

            self.obstacle_list.add(obstacle_collision_object)
    
    def odometry_callback(self, odometry_data):
        self.uav_position[0] = odometry_data.pose.pose.position.x
        self.uav_position[1] = odometry_data.pose.pose.position.y
        self.uav_position[2] = odometry_data.pose.pose.position.z
        quaternion = (
            odometry_data.pose.pose.orientation.x,
            odometry_data.pose.pose.orientation.y,
            odometry_data.pose.pose.orientation.z,
            odometry_data.pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.odom_rotate = yaw


    def takeoff(self):
        takeoff_pub = rospy.Publisher('/takeoff', Empty, queue_size=1)
        rospy.sleep(1)
        takeoff_msg = Empty()
        rospy.loginfo("Sending takeoff message...")
        takeoff_pub.publish(takeoff_msg)
        rospy.sleep(1)

    def calculate_distance_angle(self, goal_x, goal_y):

        theta = self.odom_rotate  # Assuming this is the yaw
        delta_x = goal_x - self.uav_position[0]
        delta_y = goal_y - self.uav_position[1]

        angle_to_goal = math.atan2(delta_y, delta_x) - theta
        angle_to_goal = (angle_to_goal + np.pi) % (2 * np.pi) - np.pi
        return angle_to_goal


    def occupied_cells_callback(self, msg):
        if msg is not None:
            for marker in msg.markers:
                for point in marker.points:
                    x, y, z = point.x, point.y, point.z
                    self.objects.add((int(x), int(y), int(z)))
                    if self.should_add_collision_object():
                        self.add_obstacle(x, y, z)


    def check_future_points_for_collisions(self, path, start_index):
        """Check if there are obstacles in the future points of the path."""
        num_states = path.getStateCount()

        for i in range(start_index, num_states):
            state = path.getState(i)
            x, y, z = state.getX(), state.getY(), state.getZ()

            # Perform collision checking for future points
            if not self.is_state_valid(state):
                self.stop_drone()
                rospy.loginfo(f"Collision detected at future path point ({x}, {y}, {z})")
                return False

        return True

    def is_state_valid(self, state):
        """Check if the current state is valid using FCL for collision detection."""
        if len(self.objects) == 0 or self.objects == None:
            return True
        x, y, z = int(state.getX()), int(state.getY()), int(state.getZ())
        translation = fcl.Transform([x, y, z])

        uav_collision_object = fcl.CollisionObject(self.quadrotor, translation)

        collision_objects_copy = self.obstacle_list.copy()
        for obstacle_object in collision_objects_copy:

            request = fcl.CollisionRequest()
            result = fcl.CollisionResult()

            if fcl.collide(uav_collision_object, obstacle_object, request, result):
                return False

        return True


    def target_is_obstacle(self, target_x, target_y, target_z, tolerance=0.5):
        """Check if the target point is near any obstacle, allowing for some tolerance."""
        
        collision_objects_copy = self.objects.copy()
        for ena in collision_objects_copy:
            if ena == (target_x, target_y, target_z):
                return True
        return False
    

    #def dummy_collision_check(self, path):


    def check_future_path_for_obstacles(self, path):
        """Check if there are obstacles in the future points of the path."""
        num_states = path.getStateCount()
        for i in range(num_states):
            state = path.getState(i)
            x, y, z = state.getX(), state.getY(), state.getZ()
            if not self.is_state_valid(state):
                self.stop_drone()
                return False

        return True


    def set_start(self):
        if None in self.uav_position:
            rospy.loginfo("Waiting for UAV position...")
            return False

        # Create start state
        start = ob.State(self.space)
        start().rotation().setIdentity()  # Set default rotation
        start().setXYZ(self.uav_position[0], self.uav_position[1], self.uav_position[2])

        # Clear previous start states and add the new start state
        self.pdef.clearStartStates()
        self.pdef.addStartState(start)

        self.kaka = start  # Storing the start state if needed later
        return True


    def set_goal(self, x, y, z, qx=0, qy=0, qz=0, qw=1):
        # Create goal state
        goal = ob.State(self.space)
        goal().setXYZ(x, y, z)
        goal().rotation().x = qx
        goal().rotation().y = qy
        goal().rotation().z = qz
        goal().rotation().w = qw

        # Clear previous goal states and set the new goal state
        self.pdef.clearGoal()
        self.pdef.setGoalState(goal)

        # Store the goal values if needed later
        self.the_goal = (x, y, z, qx, qy, qz, qw)
        rospy.loginfo(f"Goal set to: {x}, {y}, {z}")



    def print_path(self, path):
        """Print the path that the drone will follow."""
        rospy.loginfo("Printing the path...")
        for i in range(path.getStateCount()):
            state = path.getState(i)
            x, y, z = state.getX(), state.getY(), state.getZ()
            print(f"Waypoint {i + 1}: x = {x}, y = {y}, z = {z}")


    def create_trajectory(self, path):
        # Create a trajectory message
        trajectory_msg = Path()
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.header.frame_id = "map"  # Adjust this according to your setup

        num_states = path.getStateCount()

        # Loop through the path to create waypoints
        for i in range(num_states):
            state = path.getState(i)
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "map"

            pose_stamped.pose.position.x = state.getX()
            pose_stamped.pose.position.y = state.getY()
            pose_stamped.pose.position.z = state.getZ()

            # Compute yaw for the current waypoint (assuming next waypoint exists)
            if i < num_states - 1:
                next_state = path.getState(i + 1)
                dx = next_state.getX() - state.getX()
                dy = next_state.getY() - state.getY()
                yaw = math.atan2(dy, dx)  # Yaw is the angle to the next waypoint

                # Ensure yaw is normalized to [-π, π] to avoid sudden jumps in direction
                if yaw > math.pi:
                    yaw -= 2 * math.pi
                elif yaw < -math.pi:
                    yaw += 2 * math.pi

                quaternion = quaternion_from_euler(0, 0, yaw)  # Convert yaw to quaternion
                pose_stamped.pose.orientation = Quaternion(*quaternion)
            else:
                pose_stamped.pose.orientation.w = 1.0  # End of path, default orientation
            rospy.loginfo(f"Waypoint {i}: Yaw={yaw}, Quaternion={quaternion}")
            trajectory_msg.poses.append(pose_stamped)

        # Publish the trajectory
        self.trajectory_pub.publish(trajectory_msg)
        rospy.loginfo("Trajectory published with %d waypoints.", num_states)

    def get_max_distance_point(self):
        if self.kaka is None or self.the_goal is None:
            return
        x,y,z = self.uav_position[0], self.uav_position[1], self.uav_position[2] 
        target_x, target_y, target_z, _,_,_,_ = self.the_goal
        x,y,z = int(x), int(y), int(z)
        target_x, target_y, target_z = int(target_x), int(target_y), int(target_z)
        max_points = max(abs(x - target_x), (y-target_y), (z- target_y))
        print(max_points)
        return 8 

    def plan(self, is_replan = False):
        if not self.set_start():
            return
        #self.pdef.setStartAndGoalStates(self.kaka, self.kako)
        self.pdef.clearSolutionPaths()
        # if is_replan:
        #     self.set_goal(22, 8, 8)
        #     self.set_start()
        planner = og.InformedRRTstar(self.si)
        #planner = og.PRM(self.si)
        planner.setProblemDefinition(self.pdef)
        planner.setGoalBias(0.05)  # 5% of samples will be directed toward the goal
        planner.setRange(5.0)  # Set a smaller range to increase sampling density
        opt = ob.PathLengthOptimizationObjective(self.si)
        self.pdef.setOptimizationObjective(opt)
        #self.pdef.setCostToGoHeuristic(opt)
        #opt.setCostThreshold(ob.Cost(1.51))
        planner.setup()
        time_for_calculation = self.get_max_distance_point()
        print(f"Solving in {time_for_calculation}")
        solved = planner.solve(time_for_calculation)

        if solved:
            rospy.loginfo("Path found, executing...")
            path = planner.getProblemDefinition().getSolutionPath()
            path_simplifier = og.PathSimplifier(self.si)
            path_simplifier.smoothBSpline(path)
            #path_simplifier.reduceVertices(path, 50, 100, rangeRatio=0.33)
            self.print_path(path)
            print(f"Oh and the path is {path.getStateCount()}")
            self.execute_path(path)


    def navigate_smoothly(self, path):
        # Generate cubic splines for smooth transitions in X, Y, and Z
        num_states = path.getStateCount()
        waypoints_x = []
        waypoints_y = []
        waypoints_z = []
        for i in range(num_states):
            state = path.getState(i)
            waypoints_x.append(state.getX())
            waypoints_y.append(state.getY())
            waypoints_z.append(state.getZ()) 
        cs_x = CubicSpline(range(len(waypoints_x)), waypoints_x)
        cs_y = CubicSpline(range(len(waypoints_y)), waypoints_y)
        cs_z = CubicSpline(range(len(waypoints_z)), waypoints_z)
        final_waypoints_x = []
        final_waypoints_y = []
        final_waypoints_z = []
        # Generate smooth path between waypoints
        
        for t in np.linspace(0, len(waypoints_x) - 1, 20):  # 100 steps of smooth traversal
            x = cs_x(t)
            final_waypoints_x.append(x)
            y = cs_y(t)
            final_waypoints_y.append(y)
            z = cs_z(t)
            final_waypoints_z.append(z)
        return final_waypoints_x, final_waypoints_y, final_waypoints_z
            #print(f"Interpolated with x = {x}, y = {y}, z = {z}")
            #self.move_to(x, y, z)  


    def rotate_in_place(self):
        """Rotate the UAV 360 degrees around itself to update obstacle info."""
        rospy.loginfo("Rotating in place for obstacle scan...")
        
        total_rotation = 0
        current_yaw = self.odom_rotate
        target_rotation = 2 * math.pi  # 360 degrees

        rate = rospy.Rate(10)

        while total_rotation < target_rotation:
            yaw_velocity = 0.7  # Adjust this speed based on your needs
            velocity_cmd = Twist()
            velocity_cmd.angular.z = yaw_velocity  # Apply yaw rotation
            self.vel_pub.publish(velocity_cmd)

            rate.sleep()

            # Update the total rotation completed
            total_rotation += yaw_velocity * (1 / 10.0)  # Based on 10 Hz rate

        # Stop rotating after completing the full 360 degrees
        self.stop_drone()
        rospy.loginfo("Rotation complete.")

    def execute_path(self, path):
        num_states = path.getStateCount()
        self.create_trajectory(path)
        el_patho_x, el_patho_y, el_patho_z = self.navigate_smoothly(path)
        rate = rospy.Rate(10)

        for i in range(len(el_patho_x)):  # Only use interpolated points
            x, y, z = el_patho_x[i], el_patho_y[i], el_patho_z[i]

            # Check for collisions
            if not self.check_future_path_for_obstacles(path):
                rospy.loginfo("Collision detected, stopping...")
                self.stop_drone()
                self.set_start()
                self.set_goal(self.the_goal[0], self.the_goal[1], self.the_goal[2])
                self.plan(is_replan=True)
                return

            # Calculate yaw between consecutive waypoints
            if i < len(el_patho_x) - 1:
                next_x, next_y = el_patho_x[i + 1], el_patho_y[i + 1]
                dx = next_x - x
                dy = next_y - y
                target_yaw = math.atan2(dy, dx)

                # Normalize yaw to [-pi, pi]
                if target_yaw > math.pi:
                    target_yaw -= 2 * math.pi
                elif target_yaw < -math.pi:
                    target_yaw += 2 * math.pi
            else:
                target_yaw = None  # No need to update yaw on the last waypoint

            # Move to the interpolated point with correct yaw
            self.move_to_point(x, y, z, target_yaw)
        self.rotate_in_place()
        self.stop_drone()




    def rotate_to_goal(self, target_x, target_y):
        angle_to_goal = np.arctan2(target_y - self.uav_position[1], target_x - self.uav_position[0])
        current_yaw = self.odom_rotate

        # Calculate the difference in angles
        yaw_difference = angle_to_goal - current_yaw

        # Normalize the angle to the range [-pi, pi]
        yaw_difference = (yaw_difference + np.pi) % (2 * np.pi) - np.pi

        # Rotate until the yaw difference is small
        while abs(yaw_difference) > 0.1:  # Adjust threshold as needed
            velocity_cmd = Twist()
            velocity_cmd.angular.z = np.clip(yaw_difference, -1.0, 1.0)
            self.vel_pub.publish(velocity_cmd)

            # Recalculate yaw difference
            current_yaw = self.odom_rotate
            yaw_difference = angle_to_goal - current_yaw
            yaw_difference = (yaw_difference + np.pi) % (2 * np.pi) - np.pi
        velocity_cmd = Twist()
        # Stop rotating
        velocity_cmd.angular.z = 0
        self.vel_pub.publish(velocity_cmd)

    def move_forward_to_goal(self, target_x, target_y, target_z):
        distance_to_goal = np.linalg.norm([target_x - self.uav_position[0], target_y - self.uav_position[1]])
        
        while distance_to_goal > 0.2:  # Set a small threshold for stopping at the goal
            
            velocity_cmd = Twist()
            # Move forward (positive x-direction relative to UAV's orientation)
            velocity_cmd.linear.x = 1.0  # Constant forward velocity
            velocity_cmd.linear.z = target_z - self.uav_position[2]  # Adjust altitude
            
            # Publish the velocity command
            self.vel_pub.publish(velocity_cmd)
            
            # Recalculate the distance to the goal
            distance_to_goal = np.linalg.norm([target_x - self.uav_position[0], target_y - self.uav_position[1]])
        velocity_cmd = Twist()
        # Stop moving
        velocity_cmd.linear.x = 0
        velocity_cmd.linear.z = 0
        self.vel_pub.publish(velocity_cmd)



    def move_to_point(self, target_x, target_y, target_z, target_yaw=None):
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
            if None in self.uav_position:
                rospy.sleep(1)
                continue

            current_time = rospy.Time.now().to_sec()
            dt = current_time - self.last_time
            self.last_time = current_time

            # Calculate the positional error in world coordinates
            error_x = target_x - self.uav_position[0]
            error_y = target_y - self.uav_position[1]
            error_z = target_z - self.uav_position[2]

            # Transform error_x and error_y into local frame (based on current yaw)
            yaw = self.odom_rotate  # The UAV's current yaw
            local_error_x = error_x * math.cos(yaw) + error_y * math.sin(yaw)
            local_error_y = -error_x * math.sin(yaw) + error_y * math.cos(yaw)

            # Compute velocity commands in local frame
            vel_x = self.pid_x.compute(local_error_x, dt)
            vel_y = self.pid_y.compute(local_error_y, dt)
            vel_z = self.pid_z.compute(error_z, dt)  # No need to transform Z

            # Compute yaw error and control (if target_yaw is provided)
            if target_yaw is not None:
                yaw_error = target_yaw - self.odom_rotate  # Assuming current yaw is in odom_rotate
                yaw_velocity = self.pid_yaw.compute(yaw_error, dt)
            else:
                yaw_velocity = 0

            # Normalize velocities (cap them to prevent extreme values)
            norm = np.linalg.norm([vel_x, vel_y, vel_z])
            if norm > 1:
                vel_x /= norm
                vel_y /= norm
                vel_z /= norm

            # Publish the velocity command
            velocity_cmd = Twist()
            velocity_cmd.linear.x = vel_x  # Move forward/backward in the UAV's local frame
            velocity_cmd.linear.y = vel_y  # Move sideways in the UAV's local frame
            velocity_cmd.linear.z = vel_z  # Move up/down
            velocity_cmd.angular.z = yaw_velocity  # Adjust yaw

            self.vel_pub.publish(velocity_cmd)

            # Break loop when close enough to the target
            if np.linalg.norm([error_x, error_y, error_z]) < 0.5:
                break
            
            rate.sleep()





    def input_goal(self):
        x = float(input("Enter target x coordinate: "))
        y = float(input("Enter target y coordinate: "))
        z = float(input("Enter target z coordinate: "))
        self.set_goal(x, y, z)

    def stop_drone(self):
        velocity_cmd = Twist()
        velocity_cmd.linear.x = 0
        velocity_cmd.linear.y = 0       
        velocity_cmd.linear.z = 0       
        self.vel_pub.publish(velocity_cmd)
        rospy.sleep(0.3)

if __name__ == '__main__':
    

    try:
        planner = Planner()
        rospy.sleep(2)
        while not rospy.is_shutdown():
            
            planner.input_goal()
            planner.plan()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")