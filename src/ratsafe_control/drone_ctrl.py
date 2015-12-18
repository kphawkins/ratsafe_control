#! /usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist

class DroneInterface(object):
    def __init__(self, topic='/ardrone'):
        self._topic = topic
        self._land_pub = rospy.Publisher(topic + '/land', EmptyMsg)
        self._takeoff_pub = rospy.Publisher(topic + '/takeoff', EmptyMsg)
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)
        self._flattrim_srv = rospy.ServiceProxy(topic + '/flattrim', EmptySrv)
        self._vel_twist = Twist()

    def land(self):
        self._land_pub.publish(EmptyMsg())

    def takeoff(self):
        self._takeoff_pub.publish(EmptyMsg())

    def flattrim(self):
        self._flattrim_srv()

    def get_nav_data(self):
        return rospy.wait_for_message('/ardrone/navdata',Navdata,1)

    def cmd_vel(self, lin_v=[0.0,0.0,0.0], rot=0.0, auto_hover=True):
        self._vel_twist.linear.x = lin_v[0]
        self._vel_twist.linear.y = lin_v[1]
        self._vel_twist.linear.z = lin_v[2]
        if not auto_hover:
            self._vel_twist.angular.x = 1.0
            self._vel_twist.angular.y = 1.0

        self._vel_twist.angular.z = rot
        self._cmd_vel_pub.publish(self._vel_twist)

def main():
    rospy.init_node("drone_ctrl")
    drone = DroneInterface()
    print "Connected to drone"
    rospy.sleep(1.)
    print drone.get_nav_data()
    print "Doing a flattrim..."
    drone.flattrim()
    raw_input('Press any key to takeoff.')
    print "Taking off!"
    drone.takeoff()

    raw_input('Press any key to move forward.')
    drone.cmd_vel([0.1,0.0,0.0],0.0)
    # drone.cmd_vel([-0.1,0.0,0.0],0.0)

    raw_input('Press any key to stop.')
    drone.cmd_vel([0.0,0.0,0.0],0.0)

    raw_input('Press any key to land.')
    print "Landing..."
    drone.land()

if __name__ == "__main__":
    main()
