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
#include <atlas_msgs/ForceTorqueSensors.h>

#include <atlas_hardware_interface/AtlasFootSensor.h>
#include <atlas_hardware_interface/AtlasOdometry.h>
#include <atlas_hardware_interface/AtlasControlDataFromRobot.h>
#include <atlas_hardware_interface/AtlasRobotBehavior.h>

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
  ros::Subscriber atlas_control_data_sub_; 
  ros::Subscriber joint_states_sub_;
  ros::Subscriber head_joint_states_sub_;  
  ros::Subscriber pose_bdi_sub_;
  ros::Subscriber lidar_sub_;

  // Basic Atals Control data:
  void atlas_control_data_cb(const atlas_hardware_interface::AtlasControlDataFromRobotConstPtr& msg);

  // Joint Angles:
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 

  // The position and orientation from BDI's own estimator:
  void pose_bdi_cb(const atlas_hardware_interface::AtlasOdometryConstPtr& msg);

  // Multisense Joint Angles:
  void head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 
  
  // LIDAR data:
  void lidar_cb(const sensor_msgs::LaserScanConstPtr& msg);  

  // data storage 
  atlas_hardware_interface::AtlasControlDataFromRobot atlas_control_data_;
  sensor_msgs::Imu imu_data_;
  int64_t last_joint_state_utime_;

  // LCM publishers
  void publish_imu_batch(); 
  void publish_behavior();
  void publish_joint_state();
  void publish_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
  void publish_multisense_state(int64_t utime, float position, float velocity);

  void store_imu_raw();
  void append_foot_sensor_data(pronto::force_torque_t& msg_out);
  
  bool verbose_;

  boost::mutex control_data_mutex_;
  boost::mutex imu_data_mutex_;

};

ROS_2_LCM::ROS_2_LCM(ros::NodeHandle node_, bool simulation_) :
    simulation_(simulation_), node_(node_){

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

  // Atlas Data:
  atlas_control_data_sub_ = node_.subscribe(string("/atlas_hardware/data/full"), 100, &ROS_2_LCM::atlas_control_data_cb,this); // NEED THIS

  // Joint Data:
  joint_states_sub_ = node_.subscribe(string("/atlas/joint_states"), 100, &ROS_2_LCM::joint_states_cb,this); // NEED THIS

  // Laser:
  lidar_sub_ = node_.subscribe(string("/multisense/lidar_scan"), 100, &ROS_2_LCM::lidar_cb,this); // NEED THIS

  // Multisense Joint Angles:
  head_joint_states_sub_ = node_.subscribe(string("/spindle_state"), 100, &ROS_2_LCM::head_joint_states_cb,this); // NEED THIS
  
  verbose_ = false;
};

ROS_2_LCM::~ROS_2_LCM()  { }



int atlas_counter=0;
void ROS_2_LCM::atlas_control_data_cb(const atlas_hardware_interface::AtlasControlDataFromRobotConstPtr& msg){

//  if (verbose_)
  if (atlas_counter%100 ==0){
      ROS_INFO("Got AtlasControlDataFromRobot[%d]", atlas_counter);
  }
  atlas_counter++;

  boost::unique_lock<boost::mutex> scoped_lock(control_data_mutex_);
  atlas_control_data_ = *msg;  // not thread safe, FIXME

  store_imu_raw();
  publish_imu_batch();
  publish_behavior();

}

void ROS_2_LCM::store_imu_raw() {

  // copy filtered IMU Data
  boost::unique_lock<boost::mutex> scoped_lock(imu_data_mutex_);
  imu_data_.header                          = atlas_control_data_.filtered_imu.header;
  imu_data_.orientation                     = atlas_control_data_.filtered_imu.orientation;
  imu_data_.orientation_covariance          = atlas_control_data_.filtered_imu.orientation_covariance;
  imu_data_.angular_velocity                = atlas_control_data_.filtered_imu.angular_velocity;
  imu_data_.angular_velocity_covariance     = atlas_control_data_.filtered_imu.angular_velocity_covariance;
  imu_data_.linear_acceleration             = atlas_control_data_.filtered_imu.linear_acceleration;
  imu_data_.linear_acceleration_covariance  = atlas_control_data_.filtered_imu.linear_acceleration_covariance;

}

int behavior_counter=0;  
void ROS_2_LCM::publish_behavior() {
  
  if (behavior_counter%100 ==0){ 
    ROS_INFO("BEHAVIOR ID: %d", (int) atlas_control_data_.current_behavior.state);
  }
  behavior_counter++;

  pronto::atlas_behavior_t msg_out;
  msg_out.utime = last_joint_state_utime_;
  msg_out.behavior = (int) atlas_control_data_.current_behavior.state;
 
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

int gt_counter = 0;
void ROS_2_LCM::pose_bdi_cb(const atlas_hardware_interface::AtlasOdometryConstPtr& msg) {

  if (gt_counter%100 ==0){
    ROS_INFO("BDI POSE  [%d]", gt_counter );
  }  
  
  gt_counter++;

  bot_core::pose_t pose_msg;
  pose_msg.utime = (int64_t) floor(atlas_control_data_.header.stamp.toNSec()/1000);

  pose_msg.pos[0] = atlas_control_data_.pos_est.position.x;
  pose_msg.pos[1] = atlas_control_data_.pos_est.position.y;
  pose_msg.pos[2] = atlas_control_data_.pos_est.position.z;

  pose_msg.orientation[0] =  imu_data_.orientation.w;
  pose_msg.orientation[1] =  imu_data_.orientation.x;
  pose_msg.orientation[2] =  imu_data_.orientation.y;
  pose_msg.orientation[3] =  imu_data_.orientation.z;

  pose_msg.vel[0] = atlas_control_data_.pos_est.velocity.x;
  pose_msg.vel[1] = atlas_control_data_.pos_est.velocity.y;
  pose_msg.vel[2] = atlas_control_data_.pos_est.velocity.z;

  pose_msg.rotation_rate[0] = imu_data_.angular_velocity.x;
  pose_msg.rotation_rate[1] = imu_data_.angular_velocity.y;
  pose_msg.rotation_rate[2] = imu_data_.angular_velocity.z;

  pose_msg.accel[0] = imu_data_.linear_acceleration.x;
  pose_msg.accel[1] = imu_data_.linear_acceleration.y;
  pose_msg.accel[2] = imu_data_.linear_acceleration.z;

  lcm_publish_.publish("POSE_BDI", &pose_msg);   
  lcm_publish_.publish("POSE_BODY", &pose_msg);    // for now

}

void ROS_2_LCM::publish_imu_batch(){

  pronto::atlas_raw_imu_batch_t imu;
  imu.utime = (int64_t) floor(atlas_control_data_.header.stamp.toNSec()/1000);

  imu.num_packets = 15;
  for (size_t i=0; i < 15 ; i++){
    
   /* std::cout << i
      << " | " <<  atlas_control_data_.raw_imu[i].imu_timestamp
      << " | " <<  atlas_control_data_.raw_imu[i].packet_count
      << " | " <<  atlas_control_data_.raw_imu[i].da.x << " " << atlas_control_data_.raw_imu[i].da.y << " " << atlas_control_data_.raw_imu[i].da.z
      << " | " <<  atlas_control_data_.raw_imu[i].dd.x << " " << atlas_control_data_.raw_imu[i].dd.y << " " << atlas_control_data_.raw_imu[i].dd.z << "\n";*/
    
    pronto::atlas_raw_imu_t raw;
    //raw.utime = atlas_control_data_.raw_imu[i].imu_timestamp;
    raw.utime = (int64_t) floor(atlas_control_data_.raw_imu[i].imu_timestamp.toNSec()/1000); //atlas_control_data_.raw_imu[i].imu_timestamp;
    raw.packet_count = atlas_control_data_.raw_imu[i].packet_count;
    raw.delta_rotation[0] = atlas_control_data_.raw_imu[i].da.x;
    raw.delta_rotation[1] = atlas_control_data_.raw_imu[i].da.y;
    raw.delta_rotation[2] = atlas_control_data_.raw_imu[i].da.z;
    
    raw.linear_acceleration[0] = atlas_control_data_.raw_imu[i].dd.x;
    raw.linear_acceleration[1] = atlas_control_data_.raw_imu[i].dd.y;
    raw.linear_acceleration[2] = atlas_control_data_.raw_imu[i].dd.z;
    imu.raw_imu.push_back( raw );
  }
  lcm_publish_.publish( ("ATLAS_IMU_BATCH") , &imu);


}


void ROS_2_LCM::head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  int64_t utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  float position = msg->position[0];
  float velocity = msg->velocity[0];
  publish_multisense_state(utime, position, velocity);
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
  msg_out.num_joints = 13;

  msg_out.joint_position[0] = position;
  msg_out.joint_velocity[0] = velocity;
  msg_out.joint_name[0] = "hokuyo_joint";

  msg_out.joint_name[1] = "pre_spindle_cal_x_joint";
  msg_out.joint_name[2] = "pre_spindle_cal_y_joint";
  msg_out.joint_name[3] = "pre_spindle_cal_z_joint";

  msg_out.joint_name[4] = "pre_spindle_cal_roll_joint";
  msg_out.joint_name[5] = "pre_spindle_cal_pitch_joint";
  msg_out.joint_name[6] = "pre_spindle_cal_yaw_joint";

  msg_out.joint_name[7] = "post_spindle_cal_x_joint";
  msg_out.joint_name[8] = "post_spindle_cal_x_joint";
  msg_out.joint_name[9] = "post_spindle_cal_x_joint";

  msg_out.joint_name[10] = "post_spindle_cal_roll_joint";
  msg_out.joint_name[11] = "post_spindle_cal_pitch_joint";
  msg_out.joint_name[12] = "post_spindle_cal_yaw_joint";

  lcm_publish_.publish("MULTISENSE_STATE", &msg_out);  
}


inline int getIndex(const std::vector<std::string> &vec, const std::string &str) {
  return std::find(vec.begin(), vec.end(), str) - vec.begin();
}

int js_counter=0;
void ROS_2_LCM::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg) {

  if (js_counter%500 ==0){
    ROS_INFO("Got JointState[%d]", js_counter);
  }  
  js_counter++;

  std::vector< std::pair<int,int> > jm;

  jm.push_back (  std::make_pair( getIndex(msg->name, "r_arm_shx") , 16  ));
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

  int n_joints = jm.size();
  
  pronto::atlas_state_t msg_out;
  msg_out.utime = (int64_t) atlas_control_data_.header.stamp.toNSec()/1000; // from nsec to usec  

  if (verbose_)
    std::cout << msg_out.utime << " jnt\n";

  double elapsed_utime = (msg_out.utime - last_joint_state_utime_)*1E-6;
  if (elapsed_utime>0.004)
    std::cout << elapsed_utime << "   is elapsed_utime in sec\n";

  msg_out.joint_position.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_velocity.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_effort.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.num_joints = n_joints;

  for (std::vector<int>::size_type i = 0; i < jm.size(); i++)  {
    //std::cout << jm[i].first << " and " << jm[i].second << "\n";
    msg_out.joint_position[ jm[i].second ] = msg->position[ jm[i].first ];      
    msg_out.joint_velocity[ jm[i].second ] = msg->velocity[ jm[i].first ];
    msg_out.joint_effort[ jm[i].second ] = msg->effort[ jm[i].first ];
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


void ROS_2_LCM::append_foot_sensor_data(pronto::force_torque_t& msg_out){
  
  msg_out.l_foot_force_z   =  atlas_control_data_.foot_sensors[0].force.z;
  msg_out.l_foot_torque_x  =  atlas_control_data_.foot_sensors[0].torque.x;
  msg_out.l_foot_torque_y  =  atlas_control_data_.foot_sensors[0].torque.y;
  msg_out.r_foot_force_z   =  atlas_control_data_.foot_sensors[1].force.z;
  msg_out.r_foot_torque_x  =  atlas_control_data_.foot_sensors[1].torque.x;
  msg_out.r_foot_torque_y  =  atlas_control_data_.foot_sensors[1].torque.y;

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
