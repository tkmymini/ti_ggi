#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String,Bool,Float64
from ti_gpsr.msg import array

class GPSRNode:
    def __init__(self):
        self.command_list_sub = rospy.Subscriber('/command_list',array,self.Command)
        self.navi_result_sub = rospy.Subscriber('/navigation/result',String,self.navigateResult)
        self.mani_result_sub = rospy.Subscriber('/object/grasp_res',Bool,self.manipulateResult)
        self.search_result_sub = rospy.Subscriber('/object/recog_res',Bool,self.searchResult)
        self.change_pose_res_sub = rospy.Subscriber('/arm/changing_pose_res',Bool,self.changePoseResult)
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.getLaserCB)#start
              
        self.destination_pub = rospy.Publisher('/navigation/destination',String,queue_size=1)
        self.search_pub = rospy.Publisher('/object/recog_req',String,queue_size=1)
        self.manipulation_pub = rospy.Publisher('/object/grasp_req',String,queue_size=1)
        self.changing_pose_req_pub = rospy.Publisher('/arm/changing_pose_req',String,queue_size=1)
        self.m6_reqest_pub = rospy.Publisher('/m6_controller/command',Float64,queue_size=1)
        self.gpsrAPI_pub = rospy.Publisher('/gpsrface',Bool,queue_size=1)#APIのON、OFF切り替え
        self.action_res_pub = rospy.Publisher('/command_res',Bool,queue_size=1)#動作の終了を知らせる
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)#start
        self.tts_pub = rospy.Publisher('/tts',String,queue_size = 1)

        #最低限必要な変数
        self.sub_state = 0
        self.voice_state = 0
        self.task_count = 0
        self.action = 'none'
        self.location = 'none'
        self.obj = 'none'
        self.answer = 'none'
        #各result
        self.navigation_result = 'null'
        self.search_result = False
        self.manipulation_result = False
        self.arm_change_result = False
        #startに必要な関数
        self.min_laser_dist = 999.9
        self.front_laser_dist = 999.9
        #実行可能な動作リスト#このリストはcommand_listのループが必要なければこのリストはいらない
        self.com_list = ['go','grasp','search','speak','give','place','end']
        #start()の条件のみ使用しています
        self.start_flg = 0
       
    def Command(self,command_list):
        """for num in range(len(self.com_list)):#音声処理が正しくできていれば必要ない可能性大
            if self.com_list[num] in command_list.action:
                self.action = self.com_list[num]#command_list.action"""
        self.action = command_list.action
        self.location = command_list.location
        self.obj = command_list.obj
        self.answer = command_list.answer
    
    def go(self):
        if self.sub_state == 0:
            self.m6_reqest_pub.publish(0.3)
            self.destination_pub.publish(self.location)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'navigation'
            if self.navigation_result == 'succsess':
                self.navigation_result = 'none'
                self.location = 'none'
                self.action = 'none'
                self.sub_state = 0
                self.action_res_pub.publish(True)
                self.start_flg = 1

    def grasp(self):
        if self.sub_state == 0:
            self.m6_reqest_pub.publish(-0.07)
            self.manipulation_pub.publish(self.obj)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'manipulation'
            if self.manipulation_result == True:
                self.manipulation_result = False
                self.obj = 'none'
                self.action = 'none'
                self.sub_state = 0
                self.changing_pose_req_pub.publish('carry')
                rospy.sleep(3)
                self.action_res_pub.publish(True)
        
    def search(self):
        if self.sub_state == 0:
            self.m6_reqest_pub.publish(-0.07)
            self.search_pub.publish(self.obj)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'search'
            if self.search_result == True:
                text ='I find {obj}'.format(obj=self.obj)
                self.tts_pub.publish(text)
                rospy.sleep(3)#sentenceの長さによって変更
                self.search_result = False
                self.obj = 'none'
                self.action = 'none'
                self.sub_state = 0
                self.action_res_pub.publish(True)
            
    def speak(self):
        text ='{ans}'.format(ans=self.answer)
        self.tts_pub.publish(text)
        rospy.sleep(3)#sentenceの長さによって変更
        self.answer = 'none'
        self.action = 'none'
        self.action_res_pub.publish(True)
            
    def place(self):
        if self.sub_state == 0:
            self.changing_pose_req_pub.publish('place')
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'put'
            if self.arm_change_result == True:
                self.arm_change_result = False
                self.action = 'none'
                self.sub_state = 0
                self.action_res_pub.publish(True)

    def Give(self):
        if self.sub_state == 0:
            self.changing_pose_req_pub.publish('give')
            rospy.sleep(3)
            self.sub_state = 1
        elif self.sub_state == 1:
            self.tts_pub.publish('Please pull it up')
            rospy.sleep(2)#時間の調整あり
            self.sub_state = 2
        elif self.sub_state == 2:
            print 'give'
            if self.arm_change_result == True:
                rospy.sleep(2)
                self.arm_change_result = False
                self.action = 'none'
                self.sub_state = 0
                self.action_res_pub.publish(True)
    
    def start(self):
        try:
            while not rospy.is_shutdown() and self.front_laser_dist == 999.9:
                rospy.sleep(1.0)
            initial_distance = self.front_laser_dist
            self.tts_pub.publish('Please open the door')
            while not rospy.is_shutdown() and self.front_laser_dist <= initial_distance + 0.88:#試走場のドアの幅を参考
                rospy.loginfo(" Waiting for door open")
                rospy.sleep(2.0)
            rospy.sleep(2.0)
            self.tts_pub.publish('Thank you')
            while not rospy.is_shutdown() and not self.front_laser_dist < 2.0:
                twist_cmd = Twist()
                twist_cmd.linear.x = 0.25
                self.cmd_vel_pub.publish(twist_cmd)
            rospy.sleep(0.5)
            rospy.loginfo(" Enter the room")
            self.location = 'shelf'
            while not rospy.is_shutdown() and self.start_flg == 0:
                self.go()
                rospy.sleep(2.0)
        except rospy.ROSInterruptException:
            rospy.loginfo(" Interrupted")
            pass
    
    def end(self):
        self.task_count+=1
        self.gpsrAPI_pub.publish(True)
        #self.action_res_pub.publish(True)#test用
        self.action ='none' 
        self.voice_state = 0

    def finishState(self):
        if self.sub_state == 0:
            self.gpsrAPI_pub.publish(False)
            self.action = 'finish'
            self.destination_pub.publish('entrance')
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'navi entrance'
            if self.navigation_result == 'succsess':
                self.tts_pub.publish('Finished gpsr')
                print ''
                print 'finish GPSR'
                exit()
            
    def navigateResult(self,result):
        self.navigation_result = result.data
        print self.navigation_result

    def searchResult(self,result):
        self.search_result = result.data

    def manipulateResult(self,result):
        self.manipulation_result = result.data

    def changePoseResult(self,result):
        self.arm_change_result = result.data
        
    def getLaserCB(self,laser_scan):
        self.laser_dist = laser_scan.ranges
        self.min_laser_dist = min(laser_scan.ranges[180:540])
        self.front_laser_dist = laser_scan.ranges[359]
        
    def loopMain(self):
        print '///start GPSR//'
        while not rospy.is_shutdown():
            try:
                print ''
                print '--{action}-- [task_count:{count}]'.format(action=self.action,count=self.task_count)
                if self.task_count == 2:
                    self.finishState()
                if self.action == 'none':
                    if self.voice_state == 0:
                        self.tts_pub.publish('command waiting')
                        self.voice_state = 1
                    elif self.voice_state == 1:
                        print "command waiting.."
                elif self.action != 'none':
                    self.gpsrAPI_pub.publish(False)
                    if self.action == "end":
                        self.end()
                    elif self.action == "go":
                        self.go()
                    elif self.action == "grasp":
                        self.grasp()
                    elif self.action == "search":
                        self.search()
                    elif self.action == "speak":
                        self.speak()              
                    elif self.action == "place":
                        self.place()              
                    elif self.action == "give":
                        self.Give()        
            except IndexError:
                pass
            rospy.sleep(0.5)    
    
if __name__ == '__main__':
    rospy.init_node('gpsr_node')
    gpsr = GPSRNode()
    rospy.sleep(1)
    gpsr.start()
    gpsr.gpsrAPI_pub.publish(True)
    #gpsr.action_res_pub.publish(True)#test用
    gpsr.loopMain()
    rospy.spin()
