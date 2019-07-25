#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import subprocess
from std_msgs.msg import String,Bool,Float64
from ti_gpsr.msg import array

class GPSRNode:
    def __init__(self):
        self.command_list_sub = rospy.Subscriber('/command_list',array,self.Command)
        self.navi_result_sub = rospy.Subscriber('/navigation/result',String,self.navigateResult)
        self.mani_result_sub = rospy.Subscriber('/object/grasp_res',Bool,self.manipulateResult)
        self.search_result_sub = rospy.Subscriber('/object/recog_res',Bool,self.searchResult)
        self.change_pose_res_sub = rospy.Subscriber('/arm/changing_pose_res',Bool,self.changePoseResult)
        
        self.destination_pub = rospy.Publisher('/navigation/destination',String,queue_size=1)
        self.search_pub = rospy.Publisher('/object/recog_req',String,queue_size=10)
        self.manipulation_pub = rospy.Publisher('/object/grasp_req',String,queue_size=10)
        self.changing_pose_req_pub = rospy.Publisher('/arm/changing_pose_req',String,queue_size=1)
        self.m6_reqest_pub = rospy.Publisher('/m6_controller/command',Float64,queue_size=1)
        self.gpsrAPI_pub = rospy.Publisher('/gpsrface',Bool,queue_size=10)#APIのON、OFF切り替え
        self.action_res_pub = rospy.Publisher('/command_res',Bool,queue_size=1)#動作の終了を知らせる

        #最低限必要な変数
        self.sub_state = 0
        self.task_count = 0
        self.action = 'none'
        self.location = 'none'
        self.obj = 'none'
        self.answer = 'none'
        #各result
        self.navigation_result = 'null'
        self.search_result = False
        self.manipulation_result = False
        self.place_result = False
        
        #実行可能な動作リスト
        self.com_list = ['navi','mani','search','answer','bring','place','operater']
       
    def Command(self,command_list):
        #print 'action:{action} location:{location} obj:{obj} answer:{answer}'.format(action=command_list.action,location=command_list.location,obj=command_list.obj,answer=command_list.answer)#test
        #rospy.sleep(3)#test
        for num in range(len(self.com_list)):
            if self.com_list[num] in command_list.action:
                self.action = self.com_list[num]#command_list.action
        print 'action',self.action
        self.location = command_list.location
        self.obj = command_list.obj
        self.answer = command_list.answer
    
    def navi(self):
        if self.sub_state == 0:
            self.destination_pub.publish(self.location)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'navi'
            if self.navigation_result == 'succsess':
                self.navigation_result = 'null'
                self.location = 'none'
                self.action = 'none'
                self.sub_state = 0
                self.action_res_pub(True)

    def mani(self):
        if self.sub_state == 0:
            self.manipulation_pub.publish(self.obj)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'mani'
            if self.manipulation_result == True:
                self.manipulation_result = False
                self.obj = 'none'
                self.action = 'none'
                self.sub_state = 0
                self.changing_pose_req_pub.publish('carry')
                rospy.sleep(3)
                self.action_res_pub(True)
        
    def search(self):
        if self.sub_state == 0:
            self.search_pub.publish(self.obj)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'search'
            if self.search_result == True:
                self.search_result = False
                self.obj = 'none'
                self.action = 'none'
                self.sub_state = 0
                self.action_res_pub(True)
            
    def Answer(self):
        CMD ='/usr/bin/picospeaker {ans}'.format(ans=self.answer)
        subprocess.call(CMD.strip().split(" "))
        rospy.sleep(3)#sentenceの長さによって変更
        self.answer = 'none'
        self.action = 'none'
        self.action_res_pub(True)
            
    def place(self):
        if self.sub_state == 0:
            self.changing_pose_req_pub.publish('place')
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'place'
            if self.place_result == True:
                self.place_result = False
                self.action = 'none'
                self.sub_state = 0
                self.action_res_pub(True)

    def bring(self):#仕様を確認する
        if self.sub_state == 0:
            self.changing_pose_req_pub.publish('pass')
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'pass'
            if self.place_result == True:
                self.place_result = False
                self.action = 'none'
                self.sub_state = 0
                self.action_res_pub(True)
                    
    def operater(self):
        self.task_count+=1
        self.gpsrAPI_pub.publish(True)
        self.action ='none' 
        if self.task_count == 3:
            self.finishState()
                
    def finishState(self):
        self.gpsrAPI_pub.publish(False)
        self.action = 'finish'
        self.destination_pub.publish('entrance')
        print 'navi entrance'
        if self.navigation_result == 'succsess':
            CMD = '/usr/bin/picospeaker %s' % 'Finished gpsr'
            subprocess.call(CMD.strip().split(" "))
            print ''
            print 'finish GPSR'
            exit()
            
    def navigateResult(self,result):
        self.navigation_result = result.data

    def searchResult(self,result):
        self.search_result = result.data

    def manipulateResult(self,result):
        self.manipulation_result = result.data

    def changePoseResult(self,result):
        self.place_result = result.data

    def loopMain(self):
        print '///start GPSR//'
        while not rospy.is_shutdown():
            try:
                print ''
                print '--{action}-- [task_count:{count}]'.format(action=self.action,count=self.task_count)   
                if self.action == 'none':
                    print "command waiting.."
                elif self.action != 'none':
                    self.gpsrAPI_pub.publish(False)
                    self.API_state = 0  
                    if self.action == "operater":
                        self.operater()
                    elif self.action == "navi":
                        self.navi()
                    elif self.action == "mani":
                        self.mani()
                    elif self.action == "search":
                        self.search()
                    elif self.action == "answer":
                        self.Answer()              
                    elif self.action == "place":
                        self.place()              
                    elif self.action == "bring":
                        self.bring()        
            except IndexError:
                pass
            rospy.sleep(0.5)    
    
if __name__ == '__main__':
    rospy.init_node('gpsr_node')
    gpsr = GPSRNode()
    rospy.sleep(1)
    self.gpsrAPI_pub.publish(True)
    gpsr.loopMain()
    rospy.spin()
