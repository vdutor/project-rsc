#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg

import time

x = 0

"""
This is just a driver file.
It broadcasts fake transformations in order to test the gmapping node.

"""

def broadcast_location():
    global x
    x += 0.01
    rate = rospy.Rate(10) # 10hz
    br = tf.TransformBroadcaster()
    br.sendTransform((x, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "odom")
    rate.sleep();

if __name__ == '__main__':
    rospy.init_node('tf_tester')
    while not rospy.is_shutdown():
            broadcast_location()

