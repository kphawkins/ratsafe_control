#! /usr/bin/env python

import numpy as np
import rospy

from drone_ctrl import DroneInterface
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4

LSTICK_X = 0
LSTICK_Y = 1
RSTICK_X = 3
RSTICK_Y = 4
LTRIGGER = 2
RTRIGGER = 5

class DroneTeleop(object):
    def __init__(self, topic='/ardrone'):
        self._joy_sub = rospy.Subscriber('/joy', Joy, self._joy_cb, queue_size=1)
        self._teleop_override_pub = rospy.Publisher(topic + '/teleop_override', Bool, latch=True)
        self._teleop_override_pub.publish(Bool(False))
        self._last_msg = None
        self.drone = DroneInterface(topic, is_teleop=True)

    def _joy_cb(self, msg):
        if self._last_msg is None:
            self._last_msg = msg
            return
        lbutts = self._last_msg.buttons
        cbutts = msg.buttons
        if lbutts[BUTTON_A] == 0 and cbutts[BUTTON_A] == 1:
            self.drone.land()
        if lbutts[BUTTON_B] == 0 and cbutts[BUTTON_B] == 1:
            self.drone.flattrim()
        if lbutts[BUTTON_X] == 0 and cbutts[BUTTON_X] == 1:
            self.drone.reset()
        if lbutts[BUTTON_Y] == 0 and cbutts[BUTTON_Y] == 1:
            self.drone.takeoff()
        if lbutts[BUTTON_LB] == 0 and cbutts[BUTTON_LB] == 1:
            self._teleop_override_pub.publish(Bool(False))

        if msg.axes[LTRIGGER] < 0.9 or msg.axes[RTRIGGER] < 0.9:
            lin_v = [0.0, 0.0, 0.0]
            rot_v = 0.0
            self._teleop_override_pub.publish(Bool(True))
        else:
            lin_v = [msg.axes[LSTICK_Y], msg.axes[LSTICK_X], msg.axes[RSTICK_Y]]
            # lin_v = [msg.axes[LSTICK_Y], 0.0*msg.axes[LSTICK_X], msg.axes[RSTICK_Y]]
            rot_v = -msg.axes[RSTICK_X]
        self.drone.cmd_vel(lin_v, rot_v)

        self._last_msg = msg

def main():
    rospy.init_node("drone_teleop")
    teleop = DroneTeleop()

    def land_robot():
        print "Killing velocity..."
        teleop.drone.cmd_vel([0.0,0.0,0.0],0.0)

        print "Landing..."
        teleop.drone.land()
        rospy.sleep(1.)

    rospy.on_shutdown(land_robot)

    r = rospy.Rate(1.)
    while not rospy.is_shutdown():
        print teleop.drone.get_nav_data()
        r.sleep()

if __name__ == "__main__":
    main()
