#pragma once

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
//#include <ail_mav/SetPose.h>
//#include <ail_mav/mav_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include "traj_worker.h"
#include <robot_control/control_service.h>
#include <robot_control/dispatch_result.h>
class TrajCoordinator
{
public:
  TrajCoordinator(std::string name);
  ~TrajCoordinator(void);

  void run(double freq);
  void iteration(const ros::TimerEvent& e);

//  void goalCB();
//  void preemptCB();
  bool control_service_cb(
    robot_control::control_service::Request & req, 
    robot_control::control_service::Response & res);
 // ros::AsyncSpinner async_spinner_;
  // define callback queue to avoid getting stucked in callback
 // ros::CallbackQueue connect_srv_cb_queue_;
  ros::ServiceServer control_service_handle_;
  void posCB(const sensor_msgs::JointState::ConstPtr& msg);
protected:
  ros::NodeHandle nh_;
//  actionlib::SimpleActionServer<iiwa_control::trackingAction> as_;
  std::string action_name_;

  ros::Subscriber sub_pos_;
  ros::Publisher dispatch_result_pub_;
  std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> > client_traj_execute_;
  ros::ServiceClient client_pos_control;
  TrajWorker traj_worker_left_,traj_worker_right_;
  bool serving_left_,serving_right_;
  Eigen::VectorXd configNow_;
  double freq_;

  // robot param for system normalization
  std::string armNameLeft, armNameRight, bothArmName;
  std::string robotLeftJoints, robotRightJoints;
  int numJoints;
};
