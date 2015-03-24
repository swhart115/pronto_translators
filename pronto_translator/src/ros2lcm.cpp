// Selective ros2lcm translator
// mfallon
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <atlas_hardware_interface/AtlasControlDataFromRobot.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/atlas_behavior_t.hpp"
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/atlas_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
#include "lcmtypes/pronto/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/pronto/multisense_state_t.hpp"

using namespace std;

class ROS_2_LCM{

public:
  
  ROS_2_LCM(ros::NodeHandle node_);
  
  ~ROS_2_LCM();

private:

  lcm::LCM lcm_publish_ ;
  
  ros::NodeHandle node_;

  // data subscribers
  ros::Subscriber atlas_data_sub_; 
  ros::Subscriber head_joint_states_sub_;  
  ros::Subscriber lidar_sub_;

  // All the data from Atlas:
  void atlas_data_cb(const atlas_hardware_interface::AtlasControlDataFromRobotConstPtr& msg);

  // Multisense Joint Angles:
  void head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 
  
  // LIDAR data:
  void lidar_cb(const sensor_msgs::LaserScanConstPtr& msg);  
};

ROS_2_LCM::ROS_2_LCM(ros::NodeHandle node_) :
    node_(node_){
  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // Atlas Data:
  atlas_data_sub_ = node_.subscribe(string("/atlas_hardware/data/full"), 1, &ROS_2_LCM::atlas_data_cb,this); // NEED THIS

  // Laser:
  lidar_sub_ = node_.subscribe(string("/multisense/lidar_scan"), 1, &ROS_2_LCM::lidar_cb,this); // NEED THIS

  // Multisense Joint Angles:
  head_joint_states_sub_ = node_.subscribe(string("/multisense/joint_states"), 1, &ROS_2_LCM::head_joint_states_cb,this); // NEED THIS
  
};

ROS_2_LCM::~ROS_2_LCM()  { }


void ROS_2_LCM::lidar_cb(const sensor_msgs::LaserScanConstPtr& msg){
  bot_core::planar_lidar_t scan_out;
  scan_out.ranges = msg->ranges;
  scan_out.intensities = msg->intensities;
  scan_out.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  scan_out.nranges =msg->ranges.size();
  scan_out.nintensities=msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;
  lcm_publish_.publish("SCAN", &scan_out);
}
void ROS_2_LCM::head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  pronto::multisense_state_t msg_out;
  msg_out.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  msg_out.num_joints = 1;
  msg_out.joint_position.push_back(msg->position[0]);
  msg_out.joint_velocity.push_back(msg->velocity[0]);
  msg_out.joint_name.push_back("hokuyo_joint");
  lcm_publish_.publish("MULTISENSE_STATE", &msg_out);  
}

void ROS_2_LCM::atlas_data_cb(const atlas_hardware_interface::AtlasControlDataFromRobotConstPtr& msg) {
  // Time
  int64_t utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  pronto::utime_t utime_msg;
  utime_msg.utime = utime;
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg);

  // Behavior
  pronto::atlas_behavior_t behavior;
  behavior.utime = utime;
  behavior.behavior = msg->current_behavior.state;
  lcm_publish_.publish("ATLAS_BEHAVIOR", &behavior);

  // BDI Pose
  bot_core::pose_t pose_msg;
  pose_msg.utime =  utime;

  pose_msg.pos[0] = msg->pos_est.position.x;
  pose_msg.pos[1] = msg->pos_est.position.y;
  pose_msg.pos[2] = msg->pos_est.position.z;

  pose_msg.orientation[0] =  msg->filtered_imu.orientation.w;
  pose_msg.orientation[1] =  msg->filtered_imu.orientation.x;
  pose_msg.orientation[2] =  msg->filtered_imu.orientation.y;
  pose_msg.orientation[3] =  msg->filtered_imu.orientation.z;

  pose_msg.vel[0] = msg->pos_est.velocity.x;
  pose_msg.vel[1] = msg->pos_est.velocity.y;
  pose_msg.vel[2] = msg->pos_est.velocity.z;

  pose_msg.rotation_rate[0] = msg->filtered_imu.angular_velocity.x;
  pose_msg.rotation_rate[1] = msg->filtered_imu.angular_velocity.y;
  pose_msg.rotation_rate[2] = msg->filtered_imu.angular_velocity.z;

  pose_msg.accel[0] = msg->filtered_imu.linear_acceleration.x;
  pose_msg.accel[1] = msg->filtered_imu.linear_acceleration.y;
  pose_msg.accel[2] = msg->filtered_imu.linear_acceleration.z;

  lcm_publish_.publish("POSE_BDI", &pose_msg);   

  // Raw IMU
  pronto::atlas_raw_imu_batch_t imu;
  imu.utime = utime;
  imu.num_packets = 15;
  for (size_t i=0; i < 15 ; i++){
    pronto::atlas_raw_imu_t raw;
    raw.utime = (int64_t) floor(msg->raw_imu[i].imu_timestamp.toNSec()/1000);
    raw.packet_count = msg->raw_imu[i].packet_count;
    raw.delta_rotation[0] = msg->raw_imu[i].da.x;
    raw.delta_rotation[1] = msg->raw_imu[i].da.y;
    raw.delta_rotation[2] = msg->raw_imu[i].da.z;
    
    raw.linear_acceleration[0] = msg->raw_imu[i].dd.x;
    raw.linear_acceleration[1] = msg->raw_imu[i].dd.y;
    raw.linear_acceleration[2] = msg->raw_imu[i].dd.z;
    imu.raw_imu.push_back( raw );
  }
  lcm_publish_.publish( ("ATLAS_IMU_BATCH") , &imu);

  // Atlas State
  pronto::atlas_state_t atlas_state_msg;
  atlas_state_msg.utime = utime;
  atlas_state_msg.num_joints = 28;
  for (uint i = 0; i < 28; i++)  { //jfeed[*].*_filt contains the raw unfiltered measurements 
    atlas_state_msg.joint_position.push_back(msg->jfeed[i].q_filt);      
    atlas_state_msg.joint_velocity.push_back(msg->jfeed[i].qd_filt);
    atlas_state_msg.joint_effort.push_back(msg->jfeed[i].f_filt);
  }
  atlas_state_msg.force_torque.l_foot_force_z   =  msg->foot_sensors[0].force.z;
  atlas_state_msg.force_torque.l_foot_torque_x  =  msg->foot_sensors[0].torque.x;
  atlas_state_msg.force_torque.l_foot_torque_y  =  msg->foot_sensors[0].torque.y;
  atlas_state_msg.force_torque.r_foot_force_z   =  msg->foot_sensors[1].force.z;
  atlas_state_msg.force_torque.r_foot_torque_x  =  msg->foot_sensors[1].torque.x;
  atlas_state_msg.force_torque.r_foot_torque_y  =  msg->foot_sensors[1].torque.y;
  atlas_state_msg.force_torque.l_hand_force[0] =  msg->wrist_sensors[0].force.x;
  atlas_state_msg.force_torque.l_hand_force[1] =  msg->wrist_sensors[0].force.y;
  atlas_state_msg.force_torque.l_hand_force[2] =  msg->wrist_sensors[0].force.z;
  atlas_state_msg.force_torque.l_hand_torque[0] = msg->wrist_sensors[0].torque.x;
  atlas_state_msg.force_torque.l_hand_torque[1] = msg->wrist_sensors[0].torque.y;
  atlas_state_msg.force_torque.l_hand_torque[2] = msg->wrist_sensors[0].torque.z;
  atlas_state_msg.force_torque.r_hand_force[0] =  msg->wrist_sensors[1].force.x;
  atlas_state_msg.force_torque.r_hand_force[1] =  msg->wrist_sensors[1].force.y;
  atlas_state_msg.force_torque.r_hand_force[2] =  msg->wrist_sensors[1].force.z;
  atlas_state_msg.force_torque.r_hand_torque[0] =  msg->wrist_sensors[1].torque.x;
  atlas_state_msg.force_torque.r_hand_torque[1] =  msg->wrist_sensors[1].torque.y;
  atlas_state_msg.force_torque.r_hand_torque[2] =  msg->wrist_sensors[1].torque.z;

  lcm_publish_.publish("ATLAS_STATE", &atlas_state_msg);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "ros2lcm");
  
  ros::NodeHandle nh;

  new ROS_2_LCM(nh);
  
  ROS_INFO("ROS2LCM Translator Ready");
  
  ros::spin();
  
  return 0;
}
