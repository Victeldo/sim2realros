#!/usr/bin/env python

import rospy
from robosuite_publisher_pkg.msg import Array  # Import custom message type

def array_callback(msg):
    rospy.loginfo("Received array: %s", msg.data)

def array_subscriber():
    rospy.init_node('array_subscriber', anonymous=True)
    rospy.Subscriber('array_topic', Array, array_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        array_subscriber()
    except rospy.ROSInterruptException:
        pass
