#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from shutil import move
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import os
import yaml

import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from robot_control.srv import moveit_service, moveit_serviceResponse
from moveit_msgs.msg import RobotState
from robot_control.msg import dispatch_result
from sensor_msgs.msg import JointState
from copy import deepcopy
from robot_control.msg import dispatch_non_traj

class MoveitServer:
  def __init__(self,moveit_commander):
    yaml_path = rospy.get_param("moveit_server" + "/yaml_path")
    with open(yaml_path,"r") as f:
          temp = yaml.load(f.read(), Loader=yaml.FullLoader)

    self.left_robot_name = temp['left_arm_name']
    self.right_robot_name = temp['right_arm_name']
    self.left_end_effector = temp['left_end_effector']
    self.right_end_effector = temp['right_end_effector']

    self.left_group_name = self.left_robot_name
    self.left_move_group = moveit_commander.MoveGroupCommander(self.left_group_name)
    self.left_move_group.allow_replanning(True)
    self.left_move_group.set_end_effector_link(self.left_end_effector)
    self.left_move_group.set_pose_reference_frame("world")
    #self.left_move_group.set_joint_value_target('left_joint_3',0)   
    self.right_group_name = self.right_robot_name
    self.right_move_group = moveit_commander.MoveGroupCommander(self.right_group_name)
    self.right_move_group.allow_replanning(True)
    self.right_move_group.set_end_effector_link(self.right_end_effector)
    self.right_move_group.set_pose_reference_frame("world")
    #self.right_move_group.set_joint_value_target('right_joint_3',0)  
    self.s1 = rospy.Service('/moveit_service', moveit_service, self.single_rob_planning_Callback)
    self.s2 = rospy.Subscriber('/dispatch_non_traj', dispatch_non_traj, self.non_traj_Callback)
    self.attach_result_pub = rospy.Publisher('/dispatch_result', dispatch_result, queue_size=10)
    print ("Ready to show arm control informtion.")
    rospy.spin()

  def non_traj_Callback(self, data):
    print('data.action_type')
    #write your code here
    if data.arm_name.find("left")==0:
      self.left_move_group.attach_object('k_modul', self.left_end_effector)
    elif data.arm_name.find("right")==0:
      self.right_move_group.attach_object('k_modul', self.right_end_effector)
    print('write your code here')
    result_msg=dispatch_result()
    result_msg.action_type=data.action_type
    result_msg.arm_name=data.arm_name
    result_msg.result='success'
    self.attach_result_pub.publish(result_msg)
    

  def single_rob_planning_Callback(self, req):
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = deepcopy(req.start_config)
    
    print('req.action_type')
    print(req.action_type)
    if req.action_type.find("coupl")==0:#coupled goto waypoint or coupled machining, use tcp and cart move
      if req.arm_name.find("left")==0:
        
        self.left_move_group.set_end_effector_link("tcp_l")
        self.left_move_group.set_start_state(moveit_robot_state)
        waypoints = []
     #   self.left_move_group.set_joint_value_target('left_joint_3',0)      #this does not work
        waypoints.append(copy.deepcopy(req.goal_pose))
        
        (goal_pose_plan, fraction) = self.left_move_group.compute_cartesian_path(
                                           waypoints,   # 路径点
                                           0.01,        # 步长
                                           0.0,avoid_collisions = False)         # 跳转阈值（jump_threshold）
        error_code=True
      elif req.arm_name.find("right")==0:
        self.right_move_group.set_end_effector_link("tcp_r")
        self.right_move_group.set_start_state(moveit_robot_state)
        waypoints = []
     #   self.right_move_group.set_joint_value_target('right_joint_3',0)   #this does not work
        waypoints.append(copy.deepcopy(req.goal_pose))
        
        (goal_pose_plan, fraction) = self.right_move_group.compute_cartesian_path(
                                           waypoints,   # 路径点
                                           0.01,        # 步长
                                           0.0,avoid_collisions = False)         # 跳转阈值（jump_threshold）
        error_code=True
    elif req.action_type.find("machin")==0:#in this case, it is single machining, use link_ee and cart move
      if req.arm_name.find("left")==0:
        
        self.left_move_group.set_end_effector_link(self.left_end_effector)
        self.left_move_group.set_start_state(moveit_robot_state)
        waypoints = []
     #   self.left_move_group.set_joint_value_target('left_joint_3',0)      #this does not work
        waypoints.append(copy.deepcopy(req.goal_pose))
        
        (goal_pose_plan, fraction) = self.left_move_group.compute_cartesian_path(
                                           waypoints,   # 路径点
                                           0.01,        # 步长
                                           0.0,avoid_collisions = False)         # 跳转阈值（jump_threshold）
        error_code=True
      elif req.arm_name.find("right")==0:
        self.right_move_group.set_end_effector_link(self.right_end_effector)
        self.right_move_group.set_start_state(moveit_robot_state)
        waypoints = []
     #   self.right_move_group.set_joint_value_target('right_joint_3',0)   #this does not work
        waypoints.append(copy.deepcopy(req.goal_pose))
        
        (goal_pose_plan, fraction) = self.right_move_group.compute_cartesian_path(
                                           waypoints,   # 路径点
                                           0.01,        # 步长
                                           0.0,avoid_collisions = False)         # 跳转阈值（jump_threshold）
        error_code=True
    
    elif req.action_type.find("attac")==0:
      pass # should not be here
    elif req.action_type.find("approaching")==0:#in this case, use tcp and joint move
      if req.arm_name.find("left")==0:
        self.left_move_group.set_end_effector_link("tcp_l")
        #print("end effector: ", self.left_move_group.get_end_effector_link())
        self.left_move_group.set_start_state(moveit_robot_state)
        #self.left_move_group.set_joint_value_target('left_joint_3',0)      #this does not work
        #self.left_move_group.set_pose_reference_frame("TCP_L")
        self.left_move_group.set_pose_target(req.goal_pose)
        if '3'==sys.version[0]:
          (error_code, goal_pose_plan, planning_time, err2) = self.left_move_group.plan()
        else:
          goal_pose_plan = self.left_move_group.plan()
        error_code=True
        self.left_move_group.clear_pose_targets()
      elif req.arm_name.find("right")==0:
        self.right_move_group.set_end_effector_link("tcp_r")
        self.right_move_group.set_start_state(moveit_robot_state)
       # self.right_move_group.set_joint_value_target('right_joint_3',0)      #this does not work
        #self.left_move_group.set_pose_reference_frame("TCP_R")
        self.right_move_group.set_pose_target(req.goal_pose)
        if '3'==sys.version[0]:
          (error_code, goal_pose_plan, planning_time, err2) = self.right_move_group.plan()
        else:
          goal_pose_plan = self.right_move_group.plan()
        error_code=True
        self.right_move_group.clear_pose_targets()
    else:#in this case, use link_ee and joint move

      if req.arm_name.find("left")==0:
        self.left_move_group.set_end_effector_link(self.left_end_effector)
        #print("end effector: ", self.left_move_group.get_end_effector_link())
        self.left_move_group.set_start_state(moveit_robot_state)
        #self.left_move_group.set_joint_value_target('left_joint_3',0)      #this does not work
        #self.left_move_group.set_pose_reference_frame("TCP_L")
        self.left_move_group.set_pose_target(req.goal_pose)
        if '3'==sys.version[0]:
          (error_code, goal_pose_plan, planning_time, err2) = self.left_move_group.plan()
        else:
          goal_pose_plan = self.left_move_group.plan()
        error_code=True
        self.left_move_group.clear_pose_targets()
      elif req.arm_name.find("right")==0:
        self.right_move_group.set_end_effector_link(self.right_end_effector)
        self.right_move_group.set_start_state(moveit_robot_state)
       # self.right_move_group.set_joint_value_target('right_joint_3',0)      #this does not work
        #self.left_move_group.set_pose_reference_frame("TCP_R")
        self.right_move_group.set_pose_target(req.goal_pose)
        if '3'==sys.version[0]:
          (error_code, goal_pose_plan, planning_time, err2) = self.right_move_group.plan()
        else:
          goal_pose_plan = self.right_move_group.plan()
        
        error_code=True
        self.right_move_group.clear_pose_targets()
        
        print(goal_pose_plan)
    return moveit_serviceResponse(goal_pose_plan.joint_trajectory.points,error_code)

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

if __name__ == "__main__":
      # ROS节点初始化
  rospy.init_node('moveit_server')
  moveit_commander.roscpp_initialize(sys.argv)

  robot = moveit_commander.RobotCommander()

  scene = moveit_commander.PlanningSceneInterface()

  server = MoveitServer(moveit_commander)
