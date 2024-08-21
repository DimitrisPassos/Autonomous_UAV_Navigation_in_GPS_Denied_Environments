#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class UAVHeightAdjuster:
    def __init__(self):
        rospy.init_node('uav_height_adjuster')

        self.sonar_sub = rospy.Subscriber('/sonar_height', Range, self.sonar_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.sonar_height = None

    def sonar_callback(self, msg):
        """Update the sonar height reading."""
        self.sonar_height = msg.range 

    def adjust_height(self):
        if self.sonar_height is None or self.sonar_height > 1.0:
            rospy.logwarn("No sonar height data available.")
            return


        while self.sonar_height < 1.5:
            vel_msg = Twist()
            vel_msg.linear.z = 1.0
            self.cmd_vel_pub.publish(vel_msg)
        empty = Twist()

        self.cmd_vel_pub.publish(empty)

if __name__ == '__main__':
    adjuster = UAVHeightAdjuster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        adjuster.adjust_height()
        rate.sleep()
