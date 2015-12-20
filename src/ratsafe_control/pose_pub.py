#! /usr/bin/env python

import numpy as np
import rospy
import tf

from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('pose_pub')
    tf_list = tf.TransformListener()
    pose_pub = rospy.Publisher('/ardrone/drone_pose', PoseStamped)
    ps = PoseStamped()
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            # tf_list.waitForTransform('/world', '/drone', now, rospy.Duration(4.0))
            (pos,quat) = tf_list.lookupTransform('/world', '/drone', now)
            # ps.header.seq += 1
            # ps.header.stamp = now
            # ps.header.frame_id = '/world'
            # ps.pose.position.x = pos[0]
            # ps.pose.position.y = pos[1]
            # ps.pose.position.z = pos[2]
            # ps.pose.orientation.x = quat[0]
            # ps.pose.orientation.y = quat[1]
            # ps.pose.orientation.z = quat[2]
            # ps.pose.orientation.w = quat[3]
            pose_pub.publish(ps)
        except Exception as e:
            print e


if __name__ == "__main__":
    main()
