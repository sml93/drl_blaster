#!/usr/bin/env python2.7

from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

PKG = 'px4'

import math
import time
import rospy
import numpy as np

# from six.moves import xrange
# from threading import Thread
# from pymavlink import mavutil
# from std_msgs.msg import Bool
# from std_msgs.msg import Header
# from mavros_msgs.msg import ParamValue
from mavros_msgs.msg import AttitudeTarget
# from mavros_test_common import MavrosTestCommon
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion

def thruster():
    pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    rospy.init_node('thruster', anonymous=True)
    rate = rospy.Rate(10)
    uav_thruster = AttitudeTarget()

    roll = np.deg2rad(0)
    pitch = np.deg2rad(20)
    yaw = np.deg2rad(0)
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    quaternion = Quaternion(*quaternion)

    while not rospy.is_shutdown():
      for i in range(201):
        uav_thruster.header.stamp = rospy.Time.now()
        uav_thruster.orientation.x = quaternion.x
        uav_thruster.orientation.y = -quaternion.y
        uav_thruster.orientation.z = quaternion.z
        uav_thruster.orientation.w = quaternion.w*0.6
        uav_thruster.thrust = 0.5
        print('curr_seq: ', i)
        print(uav_thruster)
        pub.publish(uav_thruster)
        rate.sleep()
        # rospy.loginfo("\n")
      break

  
if __name__=='__main__':
  try:
    thruster()
  except rospy.ROSInterruptException:
    pass