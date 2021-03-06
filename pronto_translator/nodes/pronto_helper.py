#!/usr/bin/env python
import argparse

import rospy
import roslib; roslib.load_manifest("pronto_translator")
roslib.load_manifest("atlas_hardware_interface")
import math
import copy
import threading
import tf

from std_msgs.msg import Int32
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from atlas_hardware_interface.msg import AtlasControlDataFromRobot, AtlasOdometry

from pronto_translator_msgs.msg import *

class ProntoHelper:

    def __init__(self):

        self.imu_pub = rospy.Publisher('/pronto_helper/imu', Imu, queue_size=10)
        self.raw_imu_pub = rospy.Publisher('/pronto_helper/raw_imu', CachedRawIMUData, queue_size=10)
        self.behavior_pub = rospy.Publisher('/pronto_helper/behavior', Int32, queue_size=10)
        self.footsensor_pub = rospy.Publisher('/pronto_helper/foot_sensor', FootSensor, queue_size=10)
        self.odometry_pub = rospy.Publisher('/pronto_helper/odom', Odometry, queue_size=10)

        self.atlas_data_sub = rospy.Subscriber('/atlas_hardware/data/full', AtlasControlDataFromRobot, self.atlas_data_cb)
        self.atlas_odom_sub = rospy.Subscriber('/atlas_hardware/data/odometry', AtlasOdometry, self.atlas_odom_cb)

    def atlas_data_cb(self, data) :

        # publish foot sensor
        fs = FootSensor()       
        fs.left_fz = data.foot_sensors[0].force.z
        fs.left_mx = data.foot_sensors[0].torque.x
        fs.left_my = data.foot_sensors[0].torque.y
        fs.right_fz = data.foot_sensors[1].force.z
        fs.right_mx = data.foot_sensors[1].torque.x
        fs.right_my = data.foot_sensors[1].torque.y
        self.footsensor_pub.publish(fs)

        # publish behavior
        behavior = Int32()
        behavior.data = data.current_behavior.state
        self.behavior_pub.publish(behavior)
       
        # publish Imu
        filtered_imu = Imu()
        filtered_imu = data.filtered_imu
        self.imu_pub.publish(filtered_imu)

        # publish Raw Imu
        raw_imu = CachedRawIMUData()
        raw_imu.header = data.header
        for i in range(15) :        
            raw_imu.data[i].imu_timestamp = math.floor(data.raw_imu[i].imu_timestamp.to_nsec()/1000)
            raw_imu.data[i].packet_count = data.raw_imu[i].packet_count;
            raw_imu.data[i].dax = data.raw_imu[i].da.x;
            raw_imu.data[i].day = data.raw_imu[i].da.y;
            raw_imu.data[i].daz = data.raw_imu[i].da.z;
            raw_imu.data[i].ddx = data.raw_imu[i].dd.x;
            raw_imu.data[i].ddy = data.raw_imu[i].dd.y;
            raw_imu.data[i].ddz = data.raw_imu[i].dd.z;
        self.raw_imu_pub.publish(raw_imu)


    def atlas_odom_cb(self, data) :

        # odometry
        odom = Odometry()
        odom.header = data.header

        odom.child_frame_id = data.header.frame_id
        odom.pose.pose.position.x = data.pos_est.position.x
        odom.pose.pose.position.y = data.pos_est.position.y
        odom.pose.pose.position.z = data.pos_est.position.z
        odom.pose.pose.orientation.x = data.filtered_imu.orientation.x
        odom.pose.pose.orientation.y = data.filtered_imu.orientation.y
        odom.pose.pose.orientation.z = data.filtered_imu.orientation.z
        odom.pose.pose.orientation.w = data.filtered_imu.orientation.w
        odom.twist.twist.linear.x = data.pos_est.velocity.x
        odom.twist.twist.linear.y = data.pos_est.velocity.y
        odom.twist.twist.linear.z = data.pos_est.velocity.z
        odom.twist.twist.angular.x = data.filtered_imu.angular_velocity.x
        odom.twist.twist.angular.y = data.filtered_imu.angular_velocity.y
        odom.twist.twist.angular.z = data.filtered_imu.angular_velocity.z
        self.odometry_pub.publish(odom)



if __name__=="__main__":
    rospy.init_node("pronto_helper")
    robot = ProntoHelper()
    rospy.spin()
