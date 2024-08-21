import rospy
from ompl import base as ob
from ompl import geometric as og
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import MarkerArray
from trajectory_msgs.msg import MultiDOFJointTrajectory
from nav_msgs.msg import Odometry
from fcl import Box
import fcl
import numpy as np
from ompl import util as ou

ou.setLogLevel(ou.LOG_DEBUG)  # Enable debug logging


class Planner:
    def __init__(self):
        rospy.init_node('path_planning_node')

        self.prev_goal = [0.0] * 7
        self.min_bounds = [-10.0, -10.0, 0.0]  # Set bounds based on environment
        self.max_bounds = [15.0, 10.0, 5.0]
        self.uav_position = [None, None, None]  # x, y, z

        self.objects = set()  # Store occupied points

        # Subscribers
        rospy.Subscriber("/ground_truth/state", Odometry, self.odometry_callback)
        rospy.Subscriber("/occupied_cells_vis_array", MarkerArray, self.occupied_cells_callback)

        # Publishers
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Define the robot size for FCL collision checking
        self.quadrotor = Box(0.80, 0.80, 0.2)

        # OMPL setup
        self.space = ob.SE3StateSpace()
        bounds = ob.RealVectorBounds(3)  # 3D bounds for the SE3 space (x, y, z)
        bounds.setLow(0, self.min_bounds[0])
        bounds.setLow(1, self.min_bounds[1])
        bounds.setLow(2, self.min_bounds[2])
        bounds.setHigh(0, self.max_bounds[0])
        bounds.setHigh(1, self.max_bounds[1])
        bounds.setHigh(2, self.max_bounds[2])
        self.space.setBounds(bounds)

        

        self.si = ob.SpaceInformation(self.space)
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        #self.si.setup() 
        self.pdef = ob.ProblemDefinition(self.si)

        # UAV position
        self.uav_position = [None, None, None]  # x, y, z

    def odometry_callback(self, odometry_data):
        """Update the UAV position based on odometry data."""
        self.uav_position[0] = odometry_data.pose.pose.position.x
        self.uav_position[1] = odometry_data.pose.pose.position.y
        self.uav_position[2] = odometry_data.pose.pose.position.z


    def get_path_length_objective(self, si):
        """Return an optimization objective which attempts to minimize path length."""
        return ob.PathLengthOptimizationObjective(si)

    def occupied_cells_callback(self, msg):
        """Update the objects set with new obstacle points from MarkerArray."""
        for marker in msg.markers:
            for point in marker.points:
                x, y, z = int(point.x), int(point.y), int(point.z)
                self.objects.add((x, y, z))  # Add obstacle points to the set

    def is_state_valid(self, state):
        """Simplified validity check without FCL."""
        # Extract position
        x, y, z = state.getX(), state.getY(), state.getZ()

        # Check if position is within bounds
        return (self.min_bounds[0] <= x <= self.max_bounds[0] and 
                self.min_bounds[1] <= y <= self.max_bounds[1] and 
                self.min_bounds[2] <= z <= self.max_bounds[2])


    def set_start(self):
        """Set the UAV start position."""
        if None in self.uav_position:
            rospy.loginfo("Waiting for UAV position...")
            return False

        x, y, z = self.uav_position
        min_x, min_y, min_z = self.min_bounds
        max_x, max_y, max_z = self.max_bounds

        # Check if the UAV's position is within the bounds
        if not (min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z):
            rospy.logwarn(f"Start position ({x}, {y}, {z}) is out of bounds.")
            return False

        start = ob.State(self.space)
        start().setXYZ(x, y, z)
        start().rotation().setIdentity()  # Set default orientation
        self.pdef.addStartState(start)
        rospy.loginfo(f"Start set at: ({x}, {y}, {z})")
        return True


    def set_goal(self, x, y, z):
        """Set the goal position."""
        min_x, min_y, min_z = self.min_bounds
        max_x, max_y, max_z = self.max_bounds

        # Check if the goal position is within the bounds
        if not (min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z):
            rospy.logwarn(f"Goal position ({x}, {y}, {z}) is out of bounds.")
            return

        goal = ob.State(self.space)
        goal().setXYZ(x, y, z)
        goal().rotation().setIdentity()  # Set default identity rotation (or set specific orientation)
        self.pdef.setGoalState(goal)
        rospy.loginfo(f"Goal set to: {x}, {y}, {z}")




    def plan(self):
        """Run the RRTstar planner to plan a path."""
        if not self.set_start():
            rospy.logwarn("Invalid start state. Aborting planning.")
            return  # Exit if start position is not set

        planner = og.RRTstar(self.si)
        planner.setProblemDefinition(self.pdef)
        objective = self.get_path_length_objective(self.si)
        self.pdef.setOptimizationObjective(objective)
        planner.setup()

        rospy.loginfo("Starting planner.solve()...")
        solved = planner.solve(10.0)  # Set a 10-second planning time limit

        if solved:
            rospy.loginfo("Found solution, executing path...")
            path = planner.getProblemDefinition().getSolutionPath()

            if path is not None:
                rospy.loginfo(f"Solution path found with {path.asGeometric().getStateCount()} states.")
                self.execute_path(path)
            else:
                rospy.logwarn("No valid path returned from the planner.")
        else:
            rospy.logwarn("No solution found.")



    def execute_path(self, path):
        """Move the UAV along the planned path."""
        path_geometric = path.asGeometric()
        num_states = path_geometric.getStateCount()

        rospy.loginfo(f"Executing path with {num_states} states...")

        rate = rospy.Rate(10)  # Frequency of sending velocity commands (10 Hz)
        for i in range(num_states):
            state = path_geometric.getState(i)
            x, y, z = state.getX(), state.getY(), state.getZ()

            rospy.loginfo(f"Moving to point ({x}, {y}, {z})")
            self.move_to_point(x, y, z)

            if rospy.is_shutdown():
                break
            rate.sleep()


    def move_to_point(self, target_x, target_y, target_z):
        """Move the drone toward the target point."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if None in self.uav_position:
                rospy.loginfo("Waiting for UAV position...")
                rospy.sleep(1)
                continue

            # Log the current position and target position
            rospy.loginfo(f"Current position: {self.uav_position}, moving to ({target_x}, {target_y}, {target_z})")

            velocity_cmd = Twist()
            velocity_cmd.linear.x = target_x - self.uav_position[0]
            velocity_cmd.linear.y = target_y - self.uav_position[1]
            velocity_cmd.linear.z = target_z - self.uav_position[2]

            # Normalize the velocities
            norm = np.linalg.norm([velocity_cmd.linear.x, velocity_cmd.linear.y, velocity_cmd.linear.z])
            if norm > 1:
                velocity_cmd.linear.x /= norm
                velocity_cmd.linear.y /= norm
                velocity_cmd.linear.z /= norm

            # Log the velocity command being sent
            rospy.loginfo(f"Velocity command: x={velocity_cmd.linear.x}, y={velocity_cmd.linear.y}, z={velocity_cmd.linear.z}")
            
            # Publish velocity command
            self.vel_pub.publish(velocity_cmd)

            # Stop moving when the goal is reached
            if np.linalg.norm([target_x - self.uav_position[0], target_y - self.uav_position[1], target_z - self.uav_position[2]]) < 0.2:
                self.stop_drone()
                rospy.loginfo("Target reached. Stopping drone.")
                break

            rate.sleep()


    def stop_drone(self):
        """Stop the drone by publishing zero velocities."""
        velocity_cmd = Twist()
        velocity_cmd.linear.x = 0
        velocity_cmd.linear.y = 0
        velocity_cmd.linear.z = 0
        self.vel_pub.publish(velocity_cmd)

    def input_goal(self):
        """Get goal coordinates from user input."""
        x = float(input("Enter target x coordinate: "))
        y = float(input("Enter target y coordinate: "))
        z = float(input("Enter target z coordinate: "))
        self.set_goal(x, y, z)
        self.plan()


if __name__ == '__main__':
    planner = Planner()
    rospy.sleep(2)  # Wait for ROS to initialize

    try:
        while not rospy.is_shutdown():
            planner.input_goal()

    except KeyboardInterrupt:
        rospy.loginfo("Stopping drone and exiting...")
        planner.stop_drone()

    finally:
        planner.stop_drone()
