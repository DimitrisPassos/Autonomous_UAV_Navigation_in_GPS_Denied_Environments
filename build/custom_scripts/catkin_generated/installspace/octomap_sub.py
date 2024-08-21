#!/usr/bin/env python3

import rospy
from octomap_msgs.msg import Octomap

def octomap_callback(msg):
    rospy.loginfo("Received an OctoMap message!")
    # Process the OctoMap data here
    # For example, print the size of the OctoMap data
    rospy.loginfo("OctoMap data size: %d", len(msg.data))

def octomap_subscriber():
    rospy.init_node('octomap_subscriber', anonymous=True)
    rospy.Subscriber("/octomap_full", Octomap, octomap_callback)
    rospy.loginfo("OctoMap Subscriber Node Started. Listening to /octomap_binary...")
    rospy.spin()

if __name__ == '__main__':
    try:
        octomap_subscriber()
    except rospy.ROSInterruptException:
        pass
