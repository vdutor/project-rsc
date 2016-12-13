#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import geometry_msgs.msg

import time


def broadcast_location():
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.transform.translation.x = 1.0
    t.transform.translation.y = 2.0
    t.transform.translation.z = 3.0
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf_tester')
    while not rospy.is_shutdown():
            broadcast_location()
            time.sleep(1)
    # rospy.spin()

