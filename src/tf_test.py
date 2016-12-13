#!/usr/bin/env python
import rospy
import tf
# import tf2_ros
import geometry_msgs.msg

import time

def broadcast_location2():
    print "broadcast location 2"
    rate = rospy.Rate(10) # 10hz
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "odom")
    rate.sleep();

# def broadcast_location():
    # print "broadcasting location"
    # # rospy.INFO("BROADCASTING LOCATION in TF_TEST")
    # br = tf2_ros.TransformBroadcaster()
    # t = geometry_msgs.msg.TransformStamped()

    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "base_link"
    # t.child_frame_id = "odom"
    # t.transform.translation.x = 1.0
    # t.transform.translation.y = 2.0
    # t.transform.translation.z = 0
    # q = tf.transformations.quaternion_from_euler(0, 0, 0)
    # t.transform.rotation.x = q[0]
    # t.transform.rotation.y = q[1]
    # t.transform.rotation.z = q[2]
    # t.transform.rotation.w = q[3]

    # br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf_tester')
    while not rospy.is_shutdown():
            broadcast_location2()

