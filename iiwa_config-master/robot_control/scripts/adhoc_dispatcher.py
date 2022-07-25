#!/usr/bin/env python3
import rospy
import os
import yaml

from robot_control.srv import symbolic_maneuver#, symbolic_maneuverResponse
from robot_control.msg import dispatch_result
class Dispatcher:   
    def __init__(self):
        rospy.init_node('adhoc_disp')
        yaml_path = rospy.get_param("moveit_server" + "/yaml_path")
        # Param 
        with open(yaml_path,"r") as f:
            temp = yaml.load(f.read(), Loader=yaml.FullLoader)

        self.left_arm_name = temp['left_arm_name']
        self.right_arm_name = temp['right_arm_name']
        self.arm_name = temp['arm_name']


        self.step_kin_l=0
        self.step_kin_r=0
      #  self.action_dict_l=['goto_waypoint','machining','goto_waypoint','coupled_goto_waypoint','coupled_machining','coupled_goto_waypoint','goto_waypoint','coupled_goto_waypoint','coupled_machining','coupled_goto_waypoint']
      #  self.action_dict_r=['goto_waypoint',''         ,''          ,'coupled_goto_waypoint','coupled_machining','coupled_goto_waypoint','goto_waypoint','coupled_goto_waypoint','coupled_machining','coupled_goto_waypoint']
      #  self.goal_wypt_dict_l=['wypt_l_seg1_start','wypt_m_seg1_end','wypt_m_coupler_coupling','wypt_m_seg2_start','wypt_m_seg2_end','wypt_m_manuf_end','wypt_m_manuf_end','wypt_m_seg2_end','wypt_m_seg2_start','wypt_m_manuf_end']
      #  self.goal_wypt_dict_r=['wypt_m_coupler_coupling','empty','empty','wypt_m_seg2_start','wypt_m_seg2_end','wypt_m_manuf_end','wypt_m_manuf_end','wypt_m_seg2_end','wypt_m_seg2_start','wypt_m_manuf_end']
      #  self.action_dict_l={0:'goto_waypoint',1:'machining',2:'goto_waypoint',3:'coupled_goto_waypoint',4:'coupled_machining',5:'coupled_goto_waypoint',6:'machining',7:'goto_waypoint',8:'coupled_goto_waypoint',9:'coupled_machining',10:'coupled_goto_waypoint'}
      #  self.action_dict_r={0:'goto_waypoint',1:'coupled_goto_waypoint',2:''          ,3:'coupled_goto_waypoint',4:'coupled_machining',5:'coupled_goto_waypoint',6:'goto_waypoint',7:'',8:'coupled_goto_waypoint',9:'coupled_machining',10:'coupled_goto_waypoint'}
      #  self.goal_wypt_dict_l={0:'wypt_l_seg1_start',1:'wypt_m_seg1_end',2:'wypt_m_coupler_coupling',3:'wypt_m_seg2_start',4:'wypt_m_seg2_end',5:'wypt_m_manuf_end',6:'wypt_m_coupler_coupling',7:'wypt_m_manuf_end',8:'wypt_m_seg2_end',9:'wypt_m_seg2_start',10:'wypt_m_manuf_end'}
      #  self.goal_wypt_dict_r={0:'wypt_m_coupler_coupling',1:'wypt_m_coupler_mag',2:'empty',3:'wypt_m_seg2_start',4:'wypt_m_seg2_end',5:'wypt_m_manuf_end',6:'wypt_m_manuf_end',7:'empty',8:'wypt_m_seg2_end',9:'wypt_m_seg2_start',10:'wypt_m_manuf_end'}
        #self.action_dict_l={0:'goto_waypoint',1:'machining',2:'approaching',3:'coupled_goto_waypoint',4:'coupled_machining',5:'coupled_goto_waypoint',6:'approaching',7:'coupled_goto_waypoint',8:'coupled_machining'}
        #self.goal_wypt_dict_l={0:'wypt_l_seg1_start',1:'wypt_m_seg1_end',2:'wypt_m_coupler_coupling',3:'wypt_m_seg2_start',4:'wypt_m_seg2_end',5:'wypt_m_manuf_end',6:'wypt_m_manuf_end',7:'wypt_m_seg2_end',8:'wypt_m_seg2_start'}
       
        if self.arm_name == "panda_arm":
            self.action_dict_l={0:'approaching',1:'attach',2:'approaching',3:'approaching',4:'coupled_goto_waypoint',5:'coupled_goto_waypoint'}
            self.goal_wypt_dict_l={0:'wypt_m_coupler_mag',1:'empty',2:'wypt_m_coupler_coupling',3:'wypt_l_seg1_start',4:'wypt_m_seg1_end',5:'wypt_m_manuf_end'}
    
            self.action_dict_r={0:'approaching',1:'attach'  ,2:'approaching'        ,3:'approaching',4:'coupled_goto_waypoint',5:'coupled_goto_waypoint'}
            self.goal_wypt_dict_r={0:'wypt_m_coupler_mag',1:'empty',2:'wypt_m_coupler_coupling',3:'wypt_l_seg1_start',4:'wypt_m_seg1_end',5:'wypt_m_manuf_end'}

        elif self.arm_name == "comau_arm":
            self.action_dict_l={0:'approaching',1:'attach',2:'approaching',3:'approaching',4:'coupled_goto_waypoint',5:'coupled_goto_waypoint'}
            self.goal_wypt_dict_l={0:'wypt_m_coupler_mag',1:'empty',2:'wypt_m_coupler_coupling',3:'wypt_l_seg1_start',4:'wypt_m_seg1_end',5:'wypt_m_manuf_end'}
    
            self.action_dict_r={0:'approaching',1:'attach'  ,2:'approaching'        ,3:'approaching',4:'coupled_goto_waypoint',5:'coupled_goto_waypoint'}
            self.goal_wypt_dict_r={0:'wypt_m_coupler_mag',1:'empty',2:'wypt_m_coupler_coupling',3:'wypt_l_seg1_start',4:'wypt_m_seg1_end',5:'wypt_m_manuf_end'}


        elif self.arm_name == "iiwa_arm":
            self.action_dict_l={0:'goto_waypoint',1:'machining',2:'approaching',3:'coupled_goto_waypoint',4:'coupled_machining',5:'coupled_goto_waypoint',6:'approaching',7:'coupled_goto_waypoint',8:'coupled_machining'}
            self.goal_wypt_dict_l={0:'wypt_l_seg1_start',1:'wypt_m_seg1_end',2:'wypt_m_coupler_coupling',3:'wypt_m_seg2_start',4:'wypt_m_seg2_end',5:'wypt_m_manuf_end',6:'wypt_m_manuf_end',7:'wypt_m_seg2_end',8:'wypt_m_seg2_start'}
            self.action_dict_r={0:'approaching',1:'attach'  ,2:'approaching'        ,3:'coupled_goto_waypoint',4:'coupled_machining',5:'coupled_goto_waypoint',6:'',7:'coupled_goto_waypoint',8:'coupled_machining'}
            self.goal_wypt_dict_r={0:'wypt_m_coupler_mag',1:'empty',2:'wypt_m_coupler_coupling',3:'wypt_m_seg2_start',4:'wypt_m_seg2_end',5:'wypt_m_manuf_end',6:'empty',7:'wypt_m_seg2_end',8:'wypt_m_seg2_start'}

        
        #self.action_dict_l={0:'goto_waypoint',1:'goto_waypoint',2:'goto_waypoint',3:'goto_waypoint',4:'goto_waypoint',5:'goto_waypoint'}
        #self.goal_wypt_dict_l={0:'lh0',1:'lh1',2:'lh2',3:'lh3',4:'lh4',5:'lh5'}

       # self.action_dict_r={0:'goto_waypoint',1:'goto_waypoint'  ,2:'goto_waypoint',3:'goto_waypoint',4:'goto_waypoint',5:'goto_waypoint'}
        #self.goal_wypt_dict_r={0:'rh0',1:'rh1',2:'rh2',3:'rh3',4:'rh4',5:'rh5'}
       
        #self.action_dict_r={0:'approaching',1:'attach'  ,2:'approaching'        ,3:'coupled_goto_waypoint',4:'coupled_machining',5:'coupled_goto_waypoint',6:'',7:'coupled_goto_waypoint',8:'coupled_machining'}
        #self.goal_wypt_dict_r={0:'wypt_m_coupler_mag',1:'empty',2:'wypt_m_seg2_start',3:'wypt_m_seg2_end',4:'wypt_m_seg2_end',5:'wypt_m_manuf_end',6:'empty',7:'wypt_m_seg2_end',8:'wypt_m_seg2_start'}
        #self.prevent_dupl=False
        self.ready_for_next_coupled_action_l=False
        self.ready_for_next_coupled_action_r=False
        
    def run(self):
        #rospy.init_node('adhoc_disp')
        #symbServer = rospy.Service('/symbolic_maneuver', symbolic_maneuver,self.symbActionServerHandlingCallback)
        self.symbClientCaller=rospy.ServiceProxy('/symbolic_maneuver',symbolic_maneuver)    
        rospy.Subscriber("/dispatch_result", dispatch_result, self.dispatchResultCallback)    
        rospy.Timer(rospy.Duration(1.0), self.timerCallback)
        try:
            symb_direct_response=self.symbClientCaller(self.left_arm_name, 'wypt_m_coupler_coupling', self.goal_wypt_dict_l[self.step_kin_l],self.action_dict_l[self.step_kin_l])
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)
        try:
            symb_direct_response=self.symbClientCaller(self.right_arm_name, 'wypt_m_coupler_coupling', self.goal_wypt_dict_r[self.step_kin_r],self.action_dict_r[self.step_kin_r])
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)
        rospy.spin()
        
    def timerCallback(self, event=None):
        if self.ready_for_next_coupled_action_l and self.ready_for_next_coupled_action_r:
            self.step_kin_l=max(self.step_kin_l,self.step_kin_r)
            self.step_kin_r=max(self.step_kin_l,self.step_kin_r)
            try:
                symb_direct_response=self.symbClientCaller(self.arm_name, 'wypt_m_coupler_coupling', self.goal_wypt_dict_l[self.step_kin_l],self.action_dict_l[self.step_kin_l])
            except rospy.ServiceException as e:
                print ("Service call failed: %s"%e)
            self.ready_for_next_coupled_action_l=False
            self.ready_for_next_coupled_action_r=False

    def dispatchResultCallback(self,data):
        print(data.action_type)
        print(data.arm_name)
        print(data.result)

        if 'coupled' in data.action_type:#last action is coupled, succeed
            
            self.step_kin_l+=1
            self.step_kin_r+=1
            if 'coupled' in self.action_dict_l[self.step_kin_l]:
                if 'left' in data.arm_name:#not self.prevent_dupl:
                    self.ready_for_next_coupled_action_l=True
                elif 'right' in data.arm_name:#not self.prevent_dupl:
                    self.ready_for_next_coupled_action_r=True
                #self.prevent_dupl=True
            else:#next action not coupled
                try:
                    symb_direct_response=self.symbClientCaller(self.left_arm_name, 'wypt_m_coupler_coupling', self.goal_wypt_dict_l[self.step_kin_l],self.action_dict_l[self.step_kin_l])
                except rospy.ServiceException as e:
                    print ("Service call failed: %s"%e)
                try:
                    symb_direct_response=self.symbClientCaller(self.right_arm_name, 'wypt_m_coupler_coupling', self.goal_wypt_dict_r[self.step_kin_r],self.action_dict_r[self.step_kin_r])
                except rospy.ServiceException as e:
                    print ("Service call failed: %s"%e)
        else:
            if 'left' in data.arm_name:
                self.step_kin_l+=1
                next_action=''
                action_ind=self.step_kin_l
                while next_action=='':
                	next_action=self.action_dict_l[action_ind]
                	action_ind += 1
                if 'coupled' in next_action:
                    self.ready_for_next_coupled_action_l=True
                else:
                    try:
                        symb_direct_response=self.symbClientCaller(self.left_arm_name, 'wypt_m_coupler_coupling', self.goal_wypt_dict_l[self.step_kin_l],self.action_dict_l[self.step_kin_l])
                    except rospy.ServiceException as e:
                        print ("Service call failed: %s"%e)
            elif 'right' in data.arm_name:
                self.step_kin_r+=1
                next_action=''
                action_ind=self.step_kin_r
                while next_action=='':
                    next_action=self.action_dict_r[action_ind]
                    action_ind+=1
                if 'coupled' in next_action:
                    self.ready_for_next_coupled_action_r=True
                else:
                    try:
                        symb_direct_response=self.symbClientCaller(self.right_arm_name, 'wypt_m_coupler_coupling', self.goal_wypt_dict_r[self.step_kin_r],self.action_dict_r[self.step_kin_r])
                    except rospy.ServiceException as e:
                        print ("Service call failed: %s"%e)
        print(self.step_kin_l)
        print(self.step_kin_r)

if __name__ == '__main__':
    dispatcher_instance=Dispatcher()
    dispatcher_instance.run()

