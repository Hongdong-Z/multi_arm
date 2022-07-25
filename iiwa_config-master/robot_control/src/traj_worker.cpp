#include "traj_worker.h"
#include <eigen_conversions/eigen_msg.h>

int32_t TrajWorker::generate(ros::Time t, Eigen::VectorXd& configNow, geometry_msgs::Pose& pose_final,std::string arm_name,std::string action_type)
{

  moveit_service.request.goal_pose.position.x = pose_final.position.x;
  moveit_service.request.goal_pose.position.y = pose_final.position.y;
  moveit_service.request.goal_pose.position.z = pose_final.position.z;
  moveit_service.request.goal_pose.orientation.x = pose_final.orientation.x;
  moveit_service.request.goal_pose.orientation.y = pose_final.orientation.y;
  moveit_service.request.goal_pose.orientation.z = pose_final.orientation.z;
  moveit_service.request.goal_pose.orientation.w = pose_final.orientation.w;
  moveit_service.request.start_config.position.resize(numJoints); //panda has 7
  moveit_service.request.start_config.name.resize(numJoints);
  for (int32_t jj=0;jj<numJoints;jj++){
    moveit_service.request.start_config.position[jj]=configNow(jj);
  }
  std::string lr_string="no";
  if(arm_name[0]=='l')
    lr_string = armNameleftTemp;
  else if(arm_name[0]=='r')
    lr_string = armNameRightTemp;
  std::cout << lr_string << std::endl;
  for (int jj=0;jj<numJoints;jj++){
 //   std::string s = std::to_string(jj+1);
    moveit_service.request.start_config.name[jj] = lr_string + jointNameTemp + std::to_string(jj+1);
  }
  moveit_service.request.arm_name=arm_name;
  moveit_service.request.action_type=action_type;
  // moveit service called
  if(moveit_client_.call(moveit_service) && moveit_service.response.succeeded){
    ROS_INFO("moveit planning succeeded");
  }


  time_init_ = ros::Time::now();
 // std::cout<<moveit_service.response.succeeded<<"hey"<<std::endl;
  return moveit_service.response.succeeded;
}

bool TrajWorker::get_setpt(ros::Time t_now, Eigen::VectorXd& configNext)
 // geometry_msgs::Vector3& pos_sp, 
 // geometry_msgs::Vector3& nominal_pos_sp, 
 // double& orientation_sp, Eigen::Vector3d& pos)
{
  ros::Duration t_tracking = t_now - time_init_;
  bool not_finished;
  Vector3d pos_sp_eigen;
  Vector3d nominal_pos_sp_eigen;
  
  size_t end_index_traj_pts = moveit_service.response.traj_pts.size();
  double traj_end_time_from_start = moveit_service.response.traj_pts[end_index_traj_pts-1].time_from_start.toSec();
  double time_step = traj_end_time_from_start / end_index_traj_pts;
  double time_index_double=t_tracking.toSec() / time_step;
  size_t traj_index_cnt = std::floor(time_index_double);//rounded!
  double time_percentage_of_latter=time_index_double-(double)traj_index_cnt;

  if(traj_index_cnt >= end_index_traj_pts-1){
    traj_index_cnt = end_index_traj_pts - 2;
  }
  if(t_tracking.toSec() >= traj_end_time_from_start){
    if(t_tracking.toSec() >= traj_end_time_from_start+3.0){
      not_finished = false;
    }else
    not_finished = true;
    for (int32_t jj=0;jj<numJoints;jj++){
      configNext(jj)=moveit_service.response.traj_pts[end_index_traj_pts-1].positions[jj];
    }
  }
  else{
    not_finished = true;
    for (int32_t jj=0;jj<numJoints;jj++){
      configNext(jj)=moveit_service.response.traj_pts[traj_index_cnt].positions[jj]*(1-time_percentage_of_latter)
      +moveit_service.response.traj_pts[traj_index_cnt+1].positions[jj]*(time_percentage_of_latter);
    }// todo interpolation
  }
  return not_finished;
}

TrajWorker::TrajWorker(ros::NodeHandle &nh)
:nh_(nh)
{

  nh_.getParam("/nonblocking_dual_driver/joint_name_temp", jointNameTemp);
  nh_.getParam("/nonblocking_dual_driver/arm_name_left_temp", armNameleftTemp);
  nh_.getParam("/nonblocking_dual_driver/arm_name_right_temp", armNameRightTemp);
  nh_.getParam("/nonblocking_dual_driver/num_joints", numJoints);

  //subscribe moveit server
  moveit_client_ = nh_.serviceClient<robot_control::moveit_service>("moveit_service");

}

TrajWorker::~TrajWorker(void)
{
}
