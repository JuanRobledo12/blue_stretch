#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received message: %s", data.data)

def subscriber():
    rospy.init_node('subscriber_node', anonymous=True)
    rospy.Subscriber('test_topic', String, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
