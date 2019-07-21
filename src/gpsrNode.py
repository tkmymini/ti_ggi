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
        
        self.destination_pub = rospy.Publisher('/navigation/destination',String,queue_size=1)
        self.search_pub = rospy.Publisher('/object/recog_req',String,queue_size=10)
        self.manipulation_pub = rospy.Publisher('/object/grasp_req',String,queue_size=10)
        self.changing_pose_req_pub = rospy.Publisher('/arm/changing_pose_req',String,queue_size=1)
        self.m6_reqest_pub = rospy.Publisher('/m6_controller/command',Float64,queue_size=1)
        self.gpsrAPI_pub = rospy.Publisher('/gpsrface',Bool,queue_size=10)

        #最低限必要な変数
        self.sub_state = 0
        self.API_state = 0
        self.task_count = 0
        self.action = 'none'
        self.location = 'none'
        self.obj = 'none'
        self.answer = 'none'
        #タスクをするために必要な動作
        self.navigate = 'go'
        self.search_object = 'find'
        self.manipulate_object = 'manipulate'
        self.return_operator = 'return'
        self.finish = 'finish'
        #各result
        self.navigation_result = 'null'
        self.search_result = False
        self.manipulation_result = False
        #実行可能な動作リスト
        self.com_list = ['Tell','Locate','Find','Bring','Take']
       
    def Command(self,command_list):
        print 'action:{action} location:{location} obj:{obj} answer:{answer}'.format(action=command_list.action,location=command_list.location,obj=command_list.obj,answer=command_list.answer)#test
        rospy.sleep(3)#test
        for num in range(len(self.com_list)):
            if self.com_list[num] in command_list.action:
                self.action = self.com_list[num]#command_list.action
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
                self.navigation_result = 'null'
                self.sub_state = 2
        elif self.sub_state == 2:
            print 'search'
            if self.search_result == True:
                self.destination_pub.publish("operator")
                self.search_result = False
                self.sub_state = 3
        elif self.sub_state == 3:
            print 'return'
            if self.navigation_result == 'succsess':
                self.navigation_result = 'null'
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
                CMD ='/usr/bin/picospeaker {answer}'.format(answer=self.answer)
                subprocess.call(CMD.strip().split(" "))
                rospy.sleep(1)#sentenceの長さによって変更
                self.navigation_result = 'null'
                self.sub_state = 2
        elif self.sub_state == 2:
             self.destination_pub.publish("operator")
             self.sub_state = 3
        elif self.sub_state == 3:
            print 'return'
            if self.navigation_result == 'succsess':
                self.navigation_result = 'null'
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
                self.navigation_result = 'null'
                self.sub_state = 2
        elif self.sub_state == 2:
            print 'search'
            if self.search_result == True:
                self.destination_pub.publish("operator")
                self.search_result = False
                self.sub_state = 3
        elif self.sub_state == 3:
            print 'return'
            if self.navigation_result == 'succsess':
                self.navigation_result = 'null'
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
                self.navigation_result = 'null'
                self.sub_state =2
        elif self.sub_state == 2:
            print 'mani'
            if self.manipulation_result == True:
                print 'state:arm change'
                self.changing_pose_req_pub.publish('carry')
                rospy.sleep(3)#かかる時間によって変更
                self.sub_state = 3
                self.manipulation_result = False
        elif self.sub_state == 3: 
                self.destination_pub.publish("operator")
                self.sub_state = 4
        elif self.sub_state == 4:
            print 'return'
            if self.navigation_result == 'succsess':
                self.navigation_result = 'null'
                self.changing_pose_req_pub.publish('pass')#人にオブジェクトを渡すためにアームの角度を変える必要があれば。
                rospy.sleep(3)#かかる時間によって変更
                self.sub_state = 5
        elif self.sub_state == 5:
            CMD = '/usr/bin/picospeaker %s' % 'Here you are'
            subprocess.call(CMD.strip().split(" "))
            rospy.sleep(2)#時間の調整あり
            self.sub_state = 6
        elif self.sub_state == 6:
            CMD = '/usr/bin/picospeaker %s' % 'You are welcome'
            subprocess.call(CMD.strip().split(" "))
            rospy.sleep(1.5)#時間の調整あり 
            self.sub_state = 0
            self.action = 'none'
            self.location = 'none'
            self.obj = 'none'
            self.answer = 'none'
            self.task_count+=1
                
    #基本bringと同じ処理。把持したものをオペレーター以外の場所に持って行くなら違う処理になるので、
    #把持した後にnaviして、naviした場所のどこかに物体を置いてオペレーターに戻って終了
    def take(self):
        if self.sub_state == 0:
            self.destination_pub.publish(self.location)
            self.sub_state = 1
        elif self.sub_state == 1:
            print 'navi'
            if self.navigation_result == 'succsess':
                self.manipulation_pub.publish(self.obj)
                self.navigation_result = 'null'
                self.sub_state =2
        elif self.sub_state == 2:
            print 'mani'
            if self.manipulation_result == True:
                print 'state:arm change'
                self.changing_pose_req_pub.publish('carry')
                rospy.sleep(3)#かかる時間によって変更
                self.sub_state = 3
                self.manipulation_result = False
        elif self.sub_state == 3: 
                self.destination_pub.publish("arg")#変更必須!argは物体を置く場所
                self.sub_state = 4
        elif self.sub_state == 4:
            print 'navi'
            if self.navigation_result == 'succsess':
                self.navigation_result = 'null'
                self.changing_pose_req_pub.publish('put')#ある場所のテーブルに物体を置く。もしかしたら時間でなく、成功したか判断しないといけないかも。
                rospy.sleep(60)#かかる時間によって変更または、実行結果を受け取る必要がある
                self.sub_state = 5
        elif self.sub_state == 5:
            self.destination_pub.publish("operator")
            self.sub_state = 6
        elif self.sub_state == 6:
            print 'return'
            if self.navigation_result == 'succsess':
                self.navigation_result = 'null'
                self.sub_state = 0
                self.action = 'none'
                self.location = 'none'
                self.obj = 'none'
                self.answer = 'none'
                self.task_count+=1
                
    """def sample(self):#新たな命令を追加する場合はこれを参考にして下さい
        self.gpsrAPI_pub.publish(False)
        self.com = self.action
        self.API_state = 0
        #ここに処理(navi,mani..)を書く
        self.sub_state = 0
        self.com = 'null'
        self.action = 'none'
        self.location = 'none'
        self.obj = 'none'
        self.answer = 'none'
        self.task_count+=1"""

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

    def loopMain(self):
        print '///start GPSR//'
        while not rospy.is_shutdown():
            try:
                print ''
                print '--{action}-- [task_count:{count}]'.format(action=self.action,count=self.task_count)
                if self.task_count == 3:#task_count == 繰り返したい回数
                    self.finishState()    
                if self.action == 'none':
                    if self.API_state == 0:
                        self.gpsrAPI_pub.publish(True)
                        self.API_state = 1
                    elif self.API_state == 1:
                        print "command waiting.."
                elif self.action != 'none':
                    self.gpsrAPI_pub.publish(False)
                    self.API_state = 0  
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
            except IndexError:
                pass
            rospy.sleep(0.5)    
    
if __name__ == '__main__':
    rospy.init_node('gpsr_node')
    gpsr = GPSRNode()
    rospy.sleep(1)
    gpsr.loopMain()
    rospy.spin()
