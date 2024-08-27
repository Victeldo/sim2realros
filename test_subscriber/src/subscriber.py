#!/usr/bin/env python

import rospy
from robosuite_publisher_pkg.msg import Array  # Import custom message type

# from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

movement_arr = None

def array_callback(msg):
    global movement_arr
    movement_arr = msg
    # rospy.loginfo("Received array: %s", msg.data)
    rospy.loginfo("Received array: %s", movement_arr)

def array_subscriber():
    # rospy.init_node('array_subscriber', anonymous=True)
    # rospy.Subscriber('array_topic', Array, array_callback)
    # rospy.spin()
    rospy.init_node('pose_subscriber', anonymous=True)
    # rospy.Subscriber('pose_topic', PoseStamped, array_callback)
    # rospy.Subscriber('/my_gen3/robot_deets_topic', Array, array_callback)
    rospy.spin()

if __name__ == '__main__':
    
    # print(robot_deets)
    try:
        robot_deets = rospy.wait_for_message('/my_gen3/robot_deets_topic', Array)
        print(robot_deets)
        array_subscriber()
    except rospy.ROSInterruptException:
        pass
