#!/usr/bin/env python
import argparse

import rospy
import roslib; roslib.load_manifest("pronto_helper")

import math
import copy
import threading
import tf

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from atlas_hardware_interface import *

class ProntoHelper:

    def __init__(self):

        self.atlas_data_sub = rospy.Subscriber('/atlas_hardware/data/full', AtlasControlDataFromRobot, self.atlas_data_cb)

        footstep_pub = rospy.Publisher('/foot_contact_service/foot_sensor', AtlasFootSensor, queue_size=10)


    def atlas_data_cb(self, data) :
        print data.walk_feedback

        fs = AtlasFootSensor()       
        fs.left_fz = 0.0
        fs.left_mx = 0.0
        fs.left_my = 0.0
        fs.right_fz = 0.0
        fs.right_mx = 0.0
        fs.right_my = 0.0
        footstep_pub.pubish(fs)

        
if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Pronto Helper')
    # parser.add_argument('-r, --robot', dest='robot', help='e.g. r2')
    
    args = parser.parse_args()

    rospy.init_node("pronto_helper")

    robot = ProntoHelper()

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        r.sleep()
