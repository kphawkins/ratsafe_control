#! /usr/bin/env python

import numpy as np
import rospy
import tf
from drone_ctrl import DroneInterface

class OptiDrone(DroneInterface):
    def __init__(self, drone_topic='/ardrone'):
        super(OptiDrone,self).__init__(drone_topic)
        self._tf_list = tf.TransformListener()
        self.last_pos = None
        self.last_quat = None

    ##
    # returns (pos, quat), quat = (x,y,z,w)
    def get_pose(self):
        try:
            (pos, quat) = self._tf_list.lookupTransform('/world','/drone', rospy.Time(0))
            self.last_pos = pos
            self.last_quat = quat
            return (pos, quat)
        except:
            return (self.last_pos, self.last_quat)

def main():
    rospy.init_node("optidrone")
    drone = OptiDrone()
    rospy.sleep(0.3)

    def land_robot():
        print "Killing velocity..."
        drone.cmd_vel([0.0,0.0,0.0],0.0)

        print "Landing..."
        drone.land()
        rospy.sleep(1.)

    rospy.on_shutdown(land_robot)

    print "Connected to drone"
    rospy.sleep(1.)
    print drone.get_nav_data()
    print "Doing a flattrim..."
    drone.flattrim()
    raw_input('Press any key to takeoff.')
    print "Taking off!"
    drone.takeoff()

    raw_input('Press any key to get target pose.')
    (first_pos, first_quat) = drone.get_pose()
    first_pos = np.array(first_pos)

    raw_input('Press any key to move forward.')
    drone.cmd_vel([0.1,0.0,0.0],0.0)
    raw_input('Press any key to stop.')
    drone.cmd_vel()

    raw_input('Press any key to move back.')
    r = rospy.Rate(100.0)
    gains = np.array([0.20, 0.20, 0.0])
    while not rospy.is_shutdown():
        (pos, quat) = drone.get_pose()
        rot = tf.transformations.quaternion_matrix(quat)
        rot = np.mat(rot[:3,:3])
        vel = -gains * np.array((rot.T*np.mat(pos - first_pos).T).T)[0]
        # print vel
        drone.cmd_vel(vel)
        r.sleep()


if __name__ == "__main__":
    main()
