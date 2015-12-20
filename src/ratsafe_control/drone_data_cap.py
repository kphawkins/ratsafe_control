#! /usr/bin/env python

import numpy as np

import rospy

from drone_ctrl import DroneInterface

def main():
    rospy.init_node("drone_data_cap")
    drone = DroneInterface()
    print "Connected to drone"

    if True:
        def land_robot():
            print "Killing velocity..."
            drone.cmd_vel([0.0,0.0,0.0],0.0)

            if False:
                print "Landing..."
                drone.land()
            rospy.sleep(1.)
        rospy.on_shutdown(land_robot)

    if False:
        raw_input('Press any key to move forward full speed.')
        drone.cmd_vel([1.0,0.0,0.0],0.0)

        raw_input('Press any key to stop.')
        drone.cmd_vel([0.0,0.0,0.0],0.0)

    if True:
        raw_input('Press any key to move with Sine.')
        ampli = -0.8
        freq = 1/1.5

        r = rospy.Rate(100)
        is_first = True
        while not rospy.is_shutdown():
            if is_first:
                stime = rospy.get_time()
                ctime = 0.0
                is_first = False
            else:
                ctime = rospy.get_time() - stime
            cmdx = ampli * np.sin(freq * ctime * (2*np.pi))
            drone.cmd_vel([cmdx,0.0,0.0],0.0)
            r.sleep()


if __name__ == "__main__":
    main()
