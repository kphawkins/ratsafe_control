#! /usr/bin/env python

import numpy as np
import rospy
import tf

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

class DroneSub(object):
    def __init__(self, topic='/ardrone'):
        self._pose_sub = rospy.Subscriber(topic + '/drone_pose', PoseStamped, self._pose_cb)
        self._cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_cb)
        self._navdata_sub = rospy.Subscriber(topic + '/navdata', Navdata, self._navdata_cb)
        self.pose = PoseStamped()
        self.cmd_vel = Twist()
        self.navdata = Navdata()

    def _pose_cb(self, msg):
        self.pose = msg

    def _cmd_vel_cb(self, msg):
        self.cmd_vel = msg

    def _navdata_cb(self, msg):
        self.navdata = msg

def main():
    rospy.init_node('drone_repub')
    dronesub = DroneSub()
    repub = rospy.Publisher('/ardrone/repub', Float64MultiArray)
    f = Float64MultiArray()
    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        f.data = []
        now = rospy.Time.now()
        f.data.append(now.to_sec())
        f.data.append(dronesub.pose.header.seq)
        f.data.append(dronesub.pose.header.stamp.to_sec())
        f.data.append(dronesub.pose.pose.position.x)
        f.data.append(dronesub.pose.pose.position.y)
        f.data.append(dronesub.pose.pose.position.z)
        f.data.append(dronesub.pose.pose.orientation.x)
        f.data.append(dronesub.pose.pose.orientation.y)
        f.data.append(dronesub.pose.pose.orientation.z)
        f.data.append(dronesub.pose.pose.orientation.w)
        f.data.append(dronesub.cmd_vel.linear.x)
        f.data.append(dronesub.cmd_vel.linear.y)
        f.data.append(dronesub.cmd_vel.linear.z)
        f.data.append(dronesub.cmd_vel.angular.x)
        f.data.append(dronesub.cmd_vel.angular.y)
        f.data.append(dronesub.cmd_vel.angular.z)
        f.data.append(dronesub.navdata.header.seq)
        f.data.append(dronesub.navdata.header.stamp.to_sec())
        f.data.append(dronesub.navdata.batteryPercent)
        f.data.append(dronesub.navdata.state)
        f.data.append(dronesub.navdata.magX)
        f.data.append(dronesub.navdata.magY)
        f.data.append(dronesub.navdata.magZ)
        f.data.append(dronesub.navdata.pressure)
        f.data.append(dronesub.navdata.temp)
        f.data.append(dronesub.navdata.wind_speed)
        f.data.append(dronesub.navdata.wind_angle)
        f.data.append(dronesub.navdata.wind_comp_angle)
        f.data.append(dronesub.navdata.rotX)
        f.data.append(dronesub.navdata.rotY)
        f.data.append(dronesub.navdata.rotZ)
        f.data.append(dronesub.navdata.altd)
        f.data.append(dronesub.navdata.vx)
        f.data.append(dronesub.navdata.vy)
        f.data.append(dronesub.navdata.vz)
        f.data.append(dronesub.navdata.ax)
        f.data.append(dronesub.navdata.ay)
        f.data.append(dronesub.navdata.az)
        f.data.append(dronesub.navdata.motor1)
        f.data.append(dronesub.navdata.motor2)
        f.data.append(dronesub.navdata.motor3)
        f.data.append(dronesub.navdata.motor4)
        f.data.append(dronesub.navdata.tm)
        repub.publish(f)
        r.sleep()

if __name__ == "__main__":
    main()
