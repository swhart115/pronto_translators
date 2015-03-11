// Selective ros2lcm translator
// mfallon
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>

#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
//#include <atlas_msgs/ForceTorqueSensors.h>

#include <pronto_translator_msgs/FootSensor.h>
#include <pronto_translator_msgs/CachedRawIMUData.h>
#include <pronto_translator_msgs/RawIMUData.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/atlas_behavior_t.hpp"
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/atlas_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
#include "lcmtypes/pronto/atlas_raw_imu_batch_t.hpp"

#include "lcmtypes/pronto/multisense_state_t.hpp"
//#include <lcmtypes/multisense.hpp>
//#include "lcmtypes/pronto/imu_t.hpp"

using namespace std;

class ROS_2_LCM{

public:
  
  ROS_2_LCM(ros::NodeHandle node_, bool simulation_);
  
  ~ROS_2_LCM();

private:

  bool simulation_; 

  lcm::LCM lcm_publish_ ;
  
  ros::NodeHandle node_;

  // data subscribers
  ros::Subscriber behavior_sub_; 
  ros::Subscriber foot_sensor_sub_; 
  ros::Subscriber imu_sub_; 
  ros::Subscriber raw_imu_sub_; 
  ros::Subscriber joint_states_sub_;
  ros::Subscriber head_joint_states_sub_;  
  ros::Subscriber pose_bdi_sub_;
  ros::Subscriber lidar_sub_;

  // Behavior mode
  void behavior_cb(const std_msgs::Int32ConstPtr& msg);

  // foot sensor data
  void foot_sensor_cb(const pronto_translator_msgs::FootSensorConstPtr& msg);

  // filtered IMU data
  void imu_cb(const sensor_msgs::ImuConstPtr& msg);

  // raw filtered IMU data
  void raw_imu_cb(const pronto_translator_msgs::CachedRawIMUDataConstPtr& msg);

   // Joint Angles:
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 

  // The position and orientation from BDI's own estimator:
  void pose_bdi_cb(const nav_msgs::OdometryConstPtr& msg);

  // Multisense Joint Angles:
  void head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 
  
  // LIDAR data:
  void lidar_cb(const sensor_msgs::LaserScanConstPtr& msg);  

  // data storage 
  sensor_msgs::Imu imu_data_;
  pronto_translator_msgs::FootSensor foot_sensor_data_;
  int64_t last_joint_state_utime_;


  // LCM publishers
  void publish_joint_state();
  void publish_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
  void publish_multisense_state(int64_t utime, float position, float velocity);

  void store_imu_raw();
  void append_foot_sensor_data(pronto::force_torque_t& msg_out);
  
  bool verbose_;

  boost::mutex imu_data_mutex_;
  boost::mutex foot_data_mutex_;

};

ROS_2_LCM::ROS_2_LCM(ros::NodeHandle node_, bool simulation_) :
    simulation_(simulation_), 
    node_(node_),
    verbose_(false) {

  ROS_INFO("Initializing Translator");

  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // The position and orientation from BDI's own estimator (or GT from Gazebo):
  if (simulation_) {
    pose_bdi_sub_ = node_.subscribe(string("/ground_truth_odom"), 100, &ROS_2_LCM::pose_bdi_cb,this);  
  } else {
    pose_bdi_sub_ = node_.subscribe(string("/atlas_hardware/data/odometry"), 100, &ROS_2_LCM::pose_bdi_cb,this); // NEED THIS
  }

  // Foot sensor:
  foot_sensor_sub_ = node_.subscribe(string("/pronto_helper/foot_sensor"), 100, &ROS_2_LCM::foot_sensor_cb,this); // NEED THIS

  // Behavior:
  behavior_sub_ = node_.subscribe(string("/pronto_helper/behavior"), 100, &ROS_2_LCM::behavior_cb,this); // NEED THIS

  // IMU:
  imu_sub_ = node_.subscribe(string("/pronto_helper/imu"), 100, &ROS_2_LCM::imu_cb,this); // NEED THIS

  // Raw IMU:
  raw_imu_sub_ = node_.subscribe(string("/pronto_helper/raw_imu"), 100, &ROS_2_LCM::raw_imu_cb,this); // NEED THIS

  // Joint Data:
  joint_states_sub_ = node_.subscribe(string("/atlas/joint_states"), 100, &ROS_2_LCM::joint_states_cb,this); // NEED THIS

  // Laser:
  lidar_sub_ = node_.subscribe(string("/multisense/lidar_scan"), 100, &ROS_2_LCM::lidar_cb,this); // NEED THIS

  // Multisense Joint Angles:
  head_joint_states_sub_ = node_.subscribe(string("/multisense/joint_states"), 100, &ROS_2_LCM::head_joint_states_cb,this); // NEED THIS
  
};

ROS_2_LCM::~ROS_2_LCM()  { }


int behavior_counter=0;  
void ROS_2_LCM::behavior_cb(const std_msgs::Int32ConstPtr& msg) {
  
  if (behavior_counter%100 ==0){ 
    ROS_INFO("BEHAVIOR ID: %d", (int) msg->data);
  }
  behavior_counter++;

  pronto::atlas_behavior_t msg_out;
  msg_out.utime = last_joint_state_utime_;
  msg_out.behavior = (int) msg->data;
 
  lcm_publish_.publish("ATLAS_BEHAVIOR", &msg_out);

}


int scan_counter=0;
void ROS_2_LCM::lidar_cb(const sensor_msgs::LaserScanConstPtr& msg){

  if (scan_counter%100 ==0){ // 80
    ROS_INFO("LSCAN [%d]", scan_counter );
  }  
  scan_counter++;

  publish_lidar(msg, "SCAN");

}

int imu_store_counter=0;
void ROS_2_LCM::imu_cb(const sensor_msgs::ImuConstPtr& msg) {

  if (imu_store_counter%100 ==0){ // 80
    ROS_INFO("IMU STORE [%d]", imu_store_counter );
  }  
  imu_store_counter++;

  // copy filtered IMU Data
  boost::unique_lock<boost::mutex> scoped_lock(imu_data_mutex_);
  imu_data_.header                          = msg->header;
  imu_data_.orientation                     = msg->orientation;
  imu_data_.orientation_covariance          = msg->orientation_covariance;
  imu_data_.angular_velocity                = msg->angular_velocity;
  imu_data_.angular_velocity_covariance     = msg->angular_velocity_covariance;
  imu_data_.linear_acceleration             = msg->linear_acceleration;
  imu_data_.linear_acceleration_covariance  = msg->linear_acceleration_covariance;

}

int foot_sensor_counter=0;  
void ROS_2_LCM::foot_sensor_cb(const pronto_translator_msgs::FootSensorConstPtr& msg) {
  
  if (foot_sensor_counter%100 ==0){ 
    ROS_INFO("FOOT SENSOR[%d]", (int) foot_sensor_counter);
  }
  foot_sensor_counter++;

  boost::unique_lock<boost::mutex> scoped_lock(foot_data_mutex_);
  foot_sensor_data_ = *msg;
}


int gt_counter = 0;
void ROS_2_LCM::pose_bdi_cb(const nav_msgs::OdometryConstPtr& msg) {

  if (gt_counter%100 ==0){
    ROS_INFO("BDI POSE  [%d]", gt_counter );
  }  
  
  gt_counter++;

  bot_core::pose_t pose_msg;
  pose_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);

  pose_msg.pos[0] = msg->pose.pose.position.x;
  pose_msg.pos[1] = msg->pose.pose.position.y;
  pose_msg.pos[2] = msg->pose.pose.position.z;

  pose_msg.orientation[0] =  msg->pose.pose.orientation.w;
  pose_msg.orientation[1] =  msg->pose.pose.orientation.x;
  pose_msg.orientation[2] =  msg->pose.pose.orientation.y;
  pose_msg.orientation[3] =  msg->pose.pose.orientation.z;

  pose_msg.vel[0] = msg->twist.twist.linear.x;
  pose_msg.vel[1] = msg->twist.twist.linear.y;
  pose_msg.vel[2] = msg->twist.twist.linear.z;

  pose_msg.rotation_rate[0] = msg->twist.twist.angular.x;
  pose_msg.rotation_rate[1] = msg->twist.twist.angular.y;
  pose_msg.rotation_rate[2] = msg->twist.twist.angular.z;

  pose_msg.accel[0] = imu_data_.linear_acceleration.x;
  pose_msg.accel[1] = imu_data_.linear_acceleration.y;
  pose_msg.accel[2] = imu_data_.linear_acceleration.z;

  lcm_publish_.publish("POSE_BDI", &pose_msg);   
  lcm_publish_.publish("POSE_BODY", &pose_msg);    // for now

}

int head_joint_counter =0;
void ROS_2_LCM::head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){

  if (head_joint_counter%100 ==0){
    ROS_INFO("HEAD JOINT STATE[%d]", head_joint_counter);
  }  
  head_joint_counter++;

  int64_t utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  float position = msg->position[0];
  float velocity = msg->velocity[0];
  publish_multisense_state(utime, position, velocity);
}



inline int getIndex(const std::vector<std::string> &vec, const std::string &str) {
  return std::find(vec.begin(), vec.end(), str) - vec.begin();
}


int js_counter=0;
void ROS_2_LCM::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg) {

  if (js_counter%500 ==0){
    ROS_INFO("JOINT STATE[%d]", js_counter);
  }  
  js_counter++;

  std::vector< std::pair<int,int> > jm;

/*  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_shx") , 16  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_elx") , 17  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_akx") , 15  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "back_bkx") , 2  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_wry") , 18  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_hpy") , 12  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_wry") , 19  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_kny") , 7  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_elx") , 20  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_aky") , 14  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_shy") , 21  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_kny") , 13  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_wrx") , 22  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_akx") , 9  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_ely") , 23  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_wrx") , 24  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_hpx") , 5  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_hpy") , 6  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_hpz") , 4  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_hpx") , 11  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_shx") , 25  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "back_bky") , 1  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_shy") , 26  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "neck_ry") , 3  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_hpz") , 10  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "back_bkz") , 0  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_aky") , 8  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_ely") , 27  ));
*/

  jm.push_back (  std::make_pair( getIndex(msg->name, "back_bkz")  , 0  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "back_bky")  , 1  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "back_bkx")  , 2  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "neck_ay")   , 3  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_hpz") , 4  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_hpx") , 5  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_hpy") , 6  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_kny") , 7  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_aky") , 8  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_leg_akx") , 9  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_hpz") , 10  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_hpx") , 11  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_hpy") , 12  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_kny") , 13  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_aky") , 14  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_leg_akx") , 15  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_shz") , 16  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_shx") , 17  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_ely") , 18  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_elx") , 19  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_uwy") , 20  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "l_arm_mwx") , 21  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_shz") , 22  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_shx") , 23  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_ely") , 24  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_elx") , 25  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_uwy") , 26  ));
  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_mwx") , 27  ));

  int n_joints = jm.size();
  //int n_joints = msg->name.size();
  
  pronto::atlas_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec  

  if (verbose_)
    std::cout << msg_out.utime << " jnt\n";

  double elapsed_utime = (msg_out.utime - last_joint_state_utime_)*1E-6;
  if (elapsed_utime>0.004)
    std::cout << elapsed_utime << "   is elapsed_utime in sec\n";

  msg_out.joint_position.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_velocity.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_effort.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.num_joints = n_joints;

  for (std::vector<int>::size_type i = 0; i < n_joints; i++)  {
    //std::cout << jm[i].first << " and " << jm[i].second << "\n";
    msg_out.joint_position[ jm[i].second ] = msg->position[ jm[i].first ];      
    msg_out.joint_velocity[ jm[i].second ] = msg->velocity[ jm[i].first ];
    msg_out.joint_effort[ jm[i].second ]   = msg->effort[ jm[i].first ];
    //msg_out.joint_position[i] = msg->position[i];
    //msg_out.joint_velocity[i] = msg->velocity[i];
    //msg_out.joint_effort[i] = msg->effort[i];
  }

  // App end FT sensor info
  pronto::force_torque_t force_torque;
  append_foot_sensor_data(force_torque);
  msg_out.force_torque = force_torque;
  
  lcm_publish_.publish("ATLAS_STATE", &msg_out);
  
  pronto::utime_t utime_msg;
  int64_t joint_utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  utime_msg.utime = joint_utime;
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg); 

  last_joint_state_utime_ = joint_utime;
}


int raw_imu_counter =0;
void ROS_2_LCM::raw_imu_cb(const pronto_translator_msgs::CachedRawIMUDataConstPtr& msg){

  if (raw_imu_counter%100 ==0){
    ROS_INFO("RAW IMU [%d]", raw_imu_counter );
  }  
  raw_imu_counter++;

  pronto::atlas_raw_imu_batch_t imu;
  imu.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);

  imu.num_packets = 15;
  for (size_t i=0; i < 15 ; i++){
    
   /* std::cout << i
      << " | " <<  msg->data[i].imu_timestamp
      << " | " <<  msg->data[i].packet_count
      << " | " <<  msg->data[i].da.x << " " << msg->data[i].da.y << " " << msg->data[i].da.z
      << " | " <<  msg->data[i].dd.x << " " << msg->data[i].dd.y << " " << msg->data[i].dd.z << "\n";*/
    
    pronto::atlas_raw_imu_t raw;
    //raw.utime = msg->data[i].imu_timestamp;
    raw.utime = msg->data[i].imu_timestamp;
    raw.packet_count = msg->data[i].packet_count;
    raw.delta_rotation[0] = msg->data[i].dax;
    raw.delta_rotation[1] = msg->data[i].day;
    raw.delta_rotation[2] = msg->data[i].daz;
    
    raw.linear_acceleration[0] = msg->data[i].ddx;
    raw.linear_acceleration[1] = msg->data[i].ddy;
    raw.linear_acceleration[2] = msg->data[i].ddz;
    imu.raw_imu.push_back( raw );
  }
  lcm_publish_.publish( ("ATLAS_IMU_BATCH") , &imu);

}


int li_counter =0;
void ROS_2_LCM::publish_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel ){

  if (li_counter%10 ==0){
    ROS_INFO("LIDAR  [%d]", li_counter );
  }  
  li_counter++;

  bot_core::planar_lidar_t scan_out;
  scan_out.ranges = msg->ranges;
  scan_out.intensities = msg->intensities;
  scan_out.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  scan_out.nranges =msg->ranges.size();
  scan_out.nintensities=msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;
  lcm_publish_.publish(channel.c_str(), &scan_out);

}


void ROS_2_LCM::publish_multisense_state(int64_t utime, float position, float velocity){
  pronto::multisense_state_t msg_out;
  msg_out.utime = utime;
  for (std::vector<int>::size_type i = 0; i < 13; i++)  {
    msg_out.joint_name.push_back("z");      
    msg_out.joint_position.push_back(0);      
    msg_out.joint_velocity.push_back(0);
    msg_out.joint_effort.push_back(0);
  }  
  msg_out.num_joints = 1;

  msg_out.joint_position[0] = position;
  msg_out.joint_velocity[0] = velocity;
  msg_out.joint_name[0] = "motor_joint";

  lcm_publish_.publish("MULTISENSE_STATE", &msg_out);  
}


void ROS_2_LCM::append_foot_sensor_data(pronto::force_torque_t& msg_out){
  
  msg_out.l_foot_force_z   =  foot_sensor_data_.left_fz;
  msg_out.l_foot_torque_x  =  foot_sensor_data_.left_mx;
  msg_out.l_foot_torque_y  =  foot_sensor_data_.left_my;
  msg_out.r_foot_force_z   =  foot_sensor_data_.right_fz;
  msg_out.r_foot_torque_x  =  foot_sensor_data_.right_mx;
  msg_out.r_foot_torque_y  =  foot_sensor_data_.right_my;

  msg_out.l_hand_force[0] =  0;
  msg_out.l_hand_force[1] =  0;
  msg_out.l_hand_force[2] =  0;
  msg_out.l_hand_torque[0] = 0;
  msg_out.l_hand_torque[1] = 0;
  msg_out.l_hand_torque[2] = 0;
  msg_out.r_hand_force[0] =  0;
  msg_out.r_hand_force[1] =  0;
  msg_out.r_hand_force[2] =  0;
  msg_out.r_hand_torque[0] =  0;
  msg_out.r_hand_torque[1] =  0;
  msg_out.r_hand_torque[2] =  0;

}


int main(int argc, char **argv){
  
  bool simulation = false;  

  ros::init(argc, argv, "ros2lcm");
  
  ros::NodeHandle nh;

  new ROS_2_LCM(nh, simulation);
  
  ROS_INFO("ROS2LCM Translator Ready");
  
  ros::spin();
  
  return 0;
}
