#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    rospy.init_node('publisher_node', anonymous=True)
    pub = rospy.Publisher('test_topic', String, queue_size=10)
    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        message = "Hello, world!"
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
