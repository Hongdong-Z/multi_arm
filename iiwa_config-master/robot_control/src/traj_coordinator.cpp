#include <eigen_conversions/eigen_msg.h>
#include "traj_coordinator.h"


TrajCoordinator::TrajCoordinator(std::string name)
  : traj_worker_left_(nh_),traj_worker_right_(nh_),
  serving_left_(false),serving_right_(false)
{

  nh_.getParam("/nonblocking_dual_driver/left_arm_name", armNameLeft);
  nh_.getParam("/nonblocking_dual_driver/right_arm_name", armNameRight);
  nh_.getParam("/nonblocking_dual_driver/both_arm_name", bothArmName);
  nh_.getParam("/nonblocking_dual_driver/left_joint", robotLeftJoints);
  nh_.getParam("/nonblocking_dual_driver/right_joint", robotRightJoints);
  nh_.getParam("/nonblocking_dual_driver/num_joints", numJoints);

  configNow_.resize(numJoints*2);
  control_service_handle_ = nh_.advertiseService("/arm_control", &TrajCoordinator::control_service_cb, this);
  // get the parameters
//  ros::NodeHandle nn("~");
  dispatch_result_pub_=nh_.advertise<robot_control::dispatch_result>("/dispatch_result", 1);
  // subscribe to the data topic of interest
  sub_pos_ = nh_.subscribe("/joint_states", 1, &TrajCoordinator::posCB, this);
//  client_traj_execute_ = nh_.advertise<moveit_msgs::ExecuteTrajectoryActionGoal>("/execute_trajectory/goal", 1);
//  client_traj_execute_ = nh_.advertise<moveit_msgs::ExecuteTrajectoryActionGoal>("/move_group/fake_controller_joint_states", 1);
  client_traj_execute_.reset(new actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>(
         nh_, "/execute_trajectory", false));
//     waitForAction(execute_action_client_, move_group::EXECUTE_ACTION_NAME, timeout_for_servers, allotted_time);
  ros::NodeHandle n;

}

TrajCoordinator::~TrajCoordinator()
{
}

void TrajCoordinator::run(double freq)
{
  ros::NodeHandle node;
  freq_=freq;
  ros::Timer timer = node.createTimer(ros::Duration(1.0 / freq), &TrajCoordinator::iteration, this);
  
  ros::spin();
}
bool TrajCoordinator::control_service_cb(
    robot_control::control_service::Request & req, 
    robot_control::control_service::Response & res)
{
  if(serving_left_||serving_right_)
    client_traj_execute_->cancelGoal(); 
    // calls function from TrajectoryGeneration,
  // a traj will be generated from current real pos (not from last given pos) to given pos
  Eigen::VectorXd configNow_single_rob(numJoints);
  int32_t start_index;

  
  if (req.arm_name.compare(armNameLeft) ==0){ 
    start_index=0;
    for (int32_t jj=start_index;jj<start_index+numJoints;jj++){
      configNow_single_rob(jj-start_index)=configNow_(jj);
    }
    int32_t gen_result = traj_worker_left_.generate(ros::Time::now(), configNow_single_rob, req.goal_pose,req.arm_name,req.action_type);
    if (gen_result)
      serving_left_ = true;
    else
      serving_left_ = false;
  }
  else if(req.arm_name.compare(armNameRight) ==0){
    start_index=numJoints;
    for (int32_t jj=start_index;jj<start_index+numJoints;jj++){
      configNow_single_rob(jj-start_index)=configNow_(jj);
    }
    int32_t gen_result = traj_worker_right_.generate(ros::Time::now(), configNow_single_rob, req.goal_pose,req.arm_name,req.action_type);
    if (gen_result)
      serving_right_ = true;
    else
      serving_right_ = false;
  }
  else if (req.arm_name.compare(bothArmName) ==0){
    ROS_INFO("Alive");
    start_index=0;
    for (int32_t jj=start_index;jj<start_index+numJoints;jj++){
      configNow_single_rob(jj-start_index)=configNow_(jj);
    }
    int32_t gen_result_l = traj_worker_left_.generate(ros::Time::now(), configNow_single_rob, req.goal_pose,armNameLeft,req.action_type);
    start_index=numJoints;
    for (int32_t jj=start_index;jj<start_index+numJoints;jj++){
      configNow_single_rob(jj-start_index)=configNow_(jj);
    }
    int32_t gen_result_r = traj_worker_right_.generate(ros::Time::now(), configNow_single_rob, req.goal_pose,armNameRight,req.action_type);
    if (gen_result_l&&gen_result_r){
      serving_left_ = true;
      serving_right_ = true;
    }
      
    else{
      serving_left_ = false;
      serving_right_ = false;
    }
    
  }
  // serving_ set true when a new goal comes, set false when the drone arrives with distance smaller than 0.1m, 
  // will be checked if the action is resting or working, if working, action can give setpoint to mavros
 // serving_ = true;

  return true;

}
void TrajCoordinator::iteration(const ros::TimerEvent& e)
{
  static double time_elapse = 0;
  static bool using_nominal_height = false;
 // geometry_msgs::Vector3 nominal_pos_sp;

  double dt = e.current_real.toSec() - e.last_real.toSec();
  time_elapse += dt;
  ros::Time time_now = ros::Time::now();
 
  Eigen::VectorXd configNext(numJoints*2), configNextLeft(numJoints),configNextRight(numJoints);

  // difference between current position and target pos

  // calls function from TrajectoryGeneration, 
  //returns true if the setpoint arrives at target,
  //regardless of real drone
  bool finished_left =true,finished_right=true;
  if(serving_left_){
    
    finished_left= !traj_worker_left_.get_setpt(ros::Time::now(), configNextLeft);
  }
  else{
    for (int jj=0;jj<numJoints;jj++){
      configNextLeft(jj)=configNow_(jj);
    }
  }
  if(serving_right_){
    
    finished_right = !traj_worker_right_.get_setpt(ros::Time::now(), configNextRight);
  }
  else{
    for (int jj=0;jj<numJoints;jj++){
      configNextRight(jj)=configNow_(jj+numJoints);
    }
  }
  for (int jj=0;jj<numJoints;jj++){
    configNext(jj)=configNextLeft(jj);
    configNext(jj+numJoints)=configNextRight(jj);
  }
  //ROS_INFO("timer iteration2");
  //***************************
  moveit_msgs::ExecuteTrajectoryGoal configNext_actionGoal;
  trajectory_msgs::JointTrajectoryPoint configNext_trajmsg,configNow_trajmsg;
  configNext_trajmsg.positions.resize(numJoints*2);
  for (int jj=0;jj<numJoints*2;jj++){
    configNext_trajmsg.positions[jj] = configNext(jj);
  }
  configNext_trajmsg.time_from_start=ros::Duration(1/freq_);
  configNow_trajmsg.positions.resize(numJoints*2);
  for (int jj=0;jj<numJoints*2;jj++){
    configNow_trajmsg.positions[jj] = configNow_(jj);
  }
  configNow_trajmsg.time_from_start=ros::Duration(0);
  configNext_actionGoal.trajectory.joint_trajectory.points.push_back(configNow_trajmsg);
  configNext_actionGoal.trajectory.joint_trajectory.points.push_back(configNext_trajmsg);
  for (int jj=0;jj<numJoints;jj++){
    configNext_actionGoal.trajectory.joint_trajectory.joint_names.push_back(robotLeftJoints + std::to_string(jj+1));
  }
  for (int jj=0;jj<numJoints;jj++){
    configNext_actionGoal.trajectory.joint_trajectory.joint_names.push_back(robotRightJoints + std::to_string(jj+1));
  }
  if(finished_left&&finished_right){
    //do nothing if both are finished

  }
  else{
    //do something
    if(serving_left_||serving_right_)
      client_traj_execute_->cancelGoal(); //cancel the last goal and send this goal
    client_traj_execute_->sendGoal(configNext_actionGoal);
  }
  if(finished_left&&serving_left_){
    robot_control::dispatch_result dispatch_result_msg;
    dispatch_result_msg.arm_name = armNameLeft;
    dispatch_result_msg.result="succeed";
    serving_left_=false;
    dispatch_result_pub_.publish(dispatch_result_msg);
  }
  if(finished_right&&serving_right_){
    robot_control::dispatch_result dispatch_result_msg;
    dispatch_result_msg.arm_name = armNameRight;
    dispatch_result_msg.result="succeed";
    serving_right_=false;
    dispatch_result_pub_.publish(dispatch_result_msg);
  }

  

  for (int jj=0;jj<numJoints*2;jj++){
    configNow_(jj) = configNext(jj) ;
  }
}


void TrajCoordinator::posCB(const sensor_msgs::JointState::ConstPtr& msg)
{
  
  for (int jj=0;jj<numJoints*2;jj++){
    configNow_(jj) = msg->position[jj] ;
  }
}
