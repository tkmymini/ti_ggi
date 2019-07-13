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
        self.arm_change_result_sub = rospy.Subscriber('/arm_change/result',String,self.arm_changeResult)#未使用
        
        self.destination_pub = rospy.Publisher('/navigation/destination',String,queue_size=1)
        self.search_pub = rospy.Publisher('/object/recog_req',String,queue_size=10)
        self.manipulation_pub = rospy.Publisher('/object/grasp_req',String,queue_size=10)
        self.changing_pose_req_pub = rospy.Publisher('/arm/changing_pose_req',String,queue_size=1)#未使用
        self.m6_reqest_pub = rospy.Publisher('/m6_controller/command',Float64,queue_size=1)
        self.API_pub = rospy.Publisher('/face',Bool,queue_size=10)#未使用
        self.API_pub = rospy.Publisher('/face',Bool,queue_size=10)#未使用

        self.sub_state = 0
        self.command = 'command'
        self.navigate = 'go'
        self.search_object = 'find'
        self.manipulate_object = 'manipulate'
        self.return_operator = 'return'
        self.finish = 'finish'
        self.task_count = 0
        self.answer_result = 'Null'
        self.navigation_result = 'Null'
        self.search_result = 'Null'
        self.manipulation_result = 'Null'
        self.arm_change_result = 'Null'

        self.action = 'none'
        self.location = 'none'
        self.obj = 'none'
        self.answer = 'none'

    def Command(self,command_list):
        self.action = command_list.action
        self.location = command_list.location
        self.obj = command_list.obj
        self.answer = command_list.answer
        
    def locate(self):
        if self.sub_state == 0:
            self.destination_pub.publish(self.location)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'navi'
            if self.navigation_result == 'succsess':
                self.search_pub.publish(self.obj)
                self.sub_state = 2
        elif self.sub_state == 2:
            print 'search'
            if self.search_result == True:
                self.destination_pub.publish("operator")
                self.sub_state = 3
        elif self.sub_state == 3:
            print 'return'
            if self.navigation_result == 'return_op':
                self.sub_state = 0
                self.action = 'none'
                self.location = 'none'
                self.obj = 'none'
                self.answer = 'none'
                self.task_count+=1

    def tell(self):
        if self.sub_state == 0:
            self.destination_pub.publish(self.location)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'navi'
            if self.navigation_result == 'succsess':
                rospy.sleep(2)
                CMD = '/usr/bin/picospeaker {answer}'.format(answer=self.answer)
                rospy.sleep(1)#sentenceの長さによって変更
                self.sub_state = 2
        elif self.sub_state == 2:
             self.destination_pub.publish("operator")
             self.sub_state = 3
        elif self.sub_state == 3:
            print 'return'
            if self.navigation_result == 'return_op':
                self.sub_state = 0
                self.action = 'none'
                self.location = 'none'
                self.obj = 'none'
                self.answer = 'none'
                self.task_count+=1
                
    def find(self):
        if self.sub_state == 0:
            self.destination_pub.publish(self.location)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'navi'
            if self.navigation_result == 'succsess':
                self.search_pub.publish(self.obj)
                self.sub_state = 2
        elif self.sub_state == 2:
            print 'search'
            if self.search_result == True:
                self.destination_pub.publish("operator")
                self.sub_state = 3
        elif self.sub_state == 3:
            print 'return'
            if self.navigation_result == 'return_op':
                self.sub_state = 0
                self.action = 'none'
                self.location = 'none'
                self.obj = 'none'
                self.answer = 'none'
                self.task_count+=1
     
    def bring(self):
        if self.sub_state == 0:
            self.destination_pub.publish(self.location)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'navi'
            if self.navigation_result == 'succsess':
                self.manipulation_pub.publish(self.obj)
                self.sub_state =2
        elif self.sub_state == 2:
            print 'mani'
            if self.manipulation_result == True:
                self.destination_pub.publish("operator")
                self.sub_state = 3
        elif self.sub_state == 3:
            print 'return'
            if self.navigation_result == 'return_op':
                self.sub_state = 0
                self.action = 'none'
                self.location = 'none'
                self.obj = 'none'
                self.answer = 'none'
                self.task_count+=1
                
    def take(self):
        if self.sub_state == 0:
            self.destination_pub.publish(self.location)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'navi'
            if self.navigation_result == 'succsess':
                self.manipulation_pub.publish(self.obj)
                self.sub_state = 2
        elif self.sub_state == 2:
            print 'mani'
            if self.manipulation_result == True:
                self.destination_pub.publish("operator")
                self.sub_state = 3
        elif self.sub_state == 3:
            print 'return'
            if self.navigation_result == 'return_op':
                self.sub_state = 0
                self.action = 'none'
                self.location = 'none'
                self.obj = 'none'
                self.answer = 'none'
                self.task_count+=1
    #それぞれの関数の最後にgpsr.return_opratet()をつける

    def finishState(self):
        #self.API_pub.publish('False')
        self.action = 'fin'
        self.destination_pub.publish('entrance')
        print 'navi entrance'
        if self.navigation_result == 'succsess':
            print 'おわり。'
            rospy.sleep(1)
            CMD = '/usr/bin/picospeaker %s' % 'I finished gpsr'
            subprocess.call(CMD.strip().split(" "))
            print ''
            print "I finished GPSR"
            exit()
            
    def navigateResult(self,result):
        self.navigation_result = result.data

    def searchResult(self,result):
        self.search_result = result.data

    def manipulateResult(self,result):
        self.manipulation_result = result.data

    def arm_changeResult(self,result):
        self.arm_change_result = result.data

    def loopMain(self):
        print '///start GPSR//'
        while not rospy.is_shutdown():
            try:
                print ''
                print '--{action}-- [task_count:{count}]'.format(action=self.action,count=self.task_count)
                if self.task_count == 3:#task_count == 繰り返したい回数
                    self.finishState()    
                if self.action == "Tell":
                    self.tell()
                elif self.action == "Locate":
                    self.locate()
                elif self.action == "Find":
                    self.find()
                elif self.action == "Bring":
                    self.bring()
                elif self.action == "Take":
                    self.take()
                elif self.action == "none":
                    print "command waiting.."
            except IndexError:
                pass
            rospy.sleep(0.5)    
    
if __name__ == '__main__':
    rospy.init_node('gpsr_node')
    gpsr = GPSRNode()
    gpsr.loopMain()
    rospy.spin()
