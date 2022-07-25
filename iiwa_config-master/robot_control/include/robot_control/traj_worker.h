#pragma once

#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Eigen>
#include "ros/ros.h"

#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include <robot_control/moveit_service.h>
//#include "tangential_smooth.h"
using namespace Eigen;

class TrajWorker
{
public:
  TrajWorker(ros::NodeHandle &nh);
  ~TrajWorker(void);

  int32_t generate(ros::Time t, Eigen::VectorXd& configNow, geometry_msgs::Pose& pose_final,std::string arm_name,std::string action_type);
  bool get_setpt(ros::Time t_now, Eigen::VectorXd& configNext);

  VectorXd configFinal_;

  VectorXd configInit_;

  ros::Time time_init_;
  ros::Duration traj_duration;
  ros::NodeHandle nh_;
  ros::ServiceClient moveit_client_;
  robot_control::moveit_service moveit_service;

  bool using_obstacle_avoidance_;


  trajectory_msgs::MultiDOFJointTrajectoryPoint traj_vel_ff;

  //Param for normalization
  std::string jointNameTemp; //temp param for config name generation different between robots
  // panda: _joint  iiwa: _joint_
  std::string armNameleftTemp, armNameRightTemp; // temp param for config name generation different between robots
  // panda: panda_left iiwa: left
  int numJoints; // Number of joints:
                      // panda and iiwa: 7
                      // comau: 6
};
