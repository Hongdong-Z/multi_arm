#!/usr/bin/env python3
from ast import Load
import rospy
import os
import yaml
import math
import tf2_ros
import geometry_msgs.msg
from robot_control.srv import control_service
from robot_control.srv import symbolic_maneuver, symbolic_maneuverResponse
from robot_control.msg import dispatch_non_traj
class Converter:
    def __init__(self):
        #param
        rospy.init_node('symb_maneuver_converter')
        yaml_path = rospy.get_param("symb_maneuver_converter" + "/yaml_path")
        with open(yaml_path,"r") as f:
            temp = yaml.load(f.read(), Loader=yaml.FullLoader)

        self.left_arm_name = temp['left_arm_name']
        self.right_arm_name = temp['right_arm_name']
        self.both_arm_name = temp['both_arm_name']

        self.robot_endeffector_frame_name_r=''
        self.start_wypt=''
        self.goal_wypt=''
        self.action_type=''
        self.case_open_l=False
        self.case_open_r=False
    def run(self):
        
        self.tfBuffer = tf2_ros.Buffer()
        tflistener = tf2_ros.TransformListener(self.tfBuffer)
        symbServer = rospy.Service('/symbolic_maneuver', symbolic_maneuver,self.symbActionServerHandlingCallback)
        self.numericManeuverClientCaller=rospy.ServiceProxy('/arm_control',control_service)        
        self.attach_pub = rospy.Publisher('/dispatch_non_traj', dispatch_non_traj, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/10.0), self.timerCallback)
        rospy.spin()
    def timerCallback(self, event=None):
        pass


    def symbActionServerHandlingCallback(self,req):
        print(req.robot_endeffector_frame_name)
        print(req.start_wypt)
        print(req.goal_wypt)
        print(req.action_type)
        if req.action_type.find('attach')==0:
            msg=dispatch_non_traj()
            msg.action_type=req.action_type
            msg.arm_name=req.robot_endeffector_frame_name
            self.attach_pub.publish(msg)
            return symbolic_maneuverResponse(True)
        self.case_open=True
        self.robot_endeffector_frame_name=req.robot_endeffector_frame_name
        self.start_wypt=req.start_wypt
        self.goal_wypt=req.goal_wypt
        self.action_type=req.action_type
        tf_not_found=False
        try:
            trans_start_wypt = self.tfBuffer.lookup_transform('world', self.start_wypt, rospy.Time())
       #     print(trans_start_wypt.transform.translation.x)
       #     print(trans_start_wypt.transform.translation.x)
       #     print(trans_start_wypt.transform.translation.x)
       #     print(trans_start_wypt.transform.rotation.x)
       #     print(trans_start_wypt.transform.rotation.y)
       #     print(trans_start_wypt.transform.rotation.z)
       #     print(trans_start_wypt.transform.rotation.w)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            tf_not_found=True
        try:
            trans_goal_wypt = self.tfBuffer.lookup_transform('world',self.goal_wypt,  rospy.Time())
            print(trans_goal_wypt.transform.translation.x)
            print(trans_goal_wypt.transform.translation.y)
            print(trans_goal_wypt.transform.translation.z)
            print(trans_goal_wypt.transform.rotation.x)
            print(trans_goal_wypt.transform.rotation.y)
            print(trans_goal_wypt.transform.rotation.z)
            print(trans_goal_wypt.transform.rotation.w)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            tf_not_found=True
        
        if tf_not_found:
            pass
        else:
            start_pose = geometry_msgs.msg.Pose()
            start_pose.position = trans_start_wypt.transform.translation
            start_pose.orientation=trans_start_wypt.transform.rotation
            goal_pose = geometry_msgs.msg.Pose()
            goal_pose.position = trans_goal_wypt.transform.translation
            goal_pose.orientation=trans_goal_wypt.transform.rotation
            #srv_action_type=
            if 'coupled' in self.action_type:
                arm_name = self.both_arm_name
                try:
                    both_response=self.numericManeuverClientCaller(arm_name, start_pose, goal_pose,self.action_type)
                except rospy.ServiceException as e:
                    print ("Service call failed: %s"%e)
            else:
                if 'left' in self.robot_endeffector_frame_name:
                    arm_name = self.left_arm_name
                    try:
                        left_response=self.numericManeuverClientCaller(arm_name, start_pose, goal_pose,self.action_type)
                    except rospy.ServiceException as e:
                        print ("Service call failed: %s"%e)
                elif 'right' in self.robot_endeffector_frame_name:
                    arm_name = self.right_arm_name
                    try:
                        right_response=self.numericManeuverClientCaller(arm_name, start_pose, goal_pose,self.action_type)
                    except rospy.ServiceException as e:
                        print ("Service call failed: %s"%e)
            
                
        
        self.case_open_l=False
    #else:
        #    self.case_open_r=True#including coupled case
         #   self.robot_endeffector_frame_name_r=req.robot_endeffector_frame_name
        #    self.start_wypt_r=req.start_wypt
        #    self.goal_wypt_r=req.goal_wypt
        #    self.action_type_r=req.action_type

        return symbolic_maneuverResponse(True)

if __name__ == '__main__':
    
    
    converter_instance=Converter()
    converter_instance.run()

    
