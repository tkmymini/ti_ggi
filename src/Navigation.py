#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import actionlib
import subprocess
import std_srvs.srv
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from std_msgs.msg import String,Bool,Float64
from geometry_msgs.msg import Twist,Quaternion,PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
import time
import sys

class Navigation:
    def __init__(self):
        self.request_sub = rospy.Subscriber('navigation/input',String,self.NavigateToDestination)
        self.pose_sub = rospy.Subscriber('/tf',TFMessage,self.BaseCB)
        self.location_word_sub = rospy.Subscriber('location/voice/word',String,self.ReceiveLocation)
        self.object_word_sub = rospy.Subscriber('object/voice/word',String,self.ReceiveObject)
        self.cancel_voice_sub = rospy.Subscriber('voice/cancel',String,self.CancelVoice)

        self.result_pub = rospy.Publisher('navigation/result',String,queue_size=10)
        self.pose_pub = rospy.Publisher('amcl_pose',PoseWithCovarianceStamped,queue_size=1)
        self.search_list_pub = rospy.Publisher('search/list',String,queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
        self.clear_costmap = rospy.ServiceProxy('move_base/clear_costmaps',std_srvs.srv.Empty)
        self.reset_pub = rospy.Publisher('reconfiguration/input',Bool,queue_size=10)
        self.follow_result_pub = rospy.Publisher('follow/result',String,queue_size=1)
        self.voice_cancel_pub = rospy.Publisher('voice/cancel',String,queue_size=1)
        
        self.location_list = []
        
        self.robot_pose_x = 999
        self.robot_pose_y = 999
        self.robot_pose_w = 999
        self.location_pose_x = 0
        self.location_pose_y = 0
        self.location_pose_w = 0
        self.cmd_vel = Twist()
        self.cmd_vel.angular.z = 0.0
        self.location_append_num = 0
        self.location_word = 'Null'
        self.object_word = 'Null'
        self.effective_location_word  = 'Null'
        self.effective_object_word  = 'Null'
        self.receive_location_num  = 0
        self.receive_object_num  = 0
        self.cancel_flg = 0
        self.sub_state = 0

    def BaseCB(self,pose):
        try:
            if pose.transforms[0].header.frame_id == 'odom':
                print 'rotation z is: ',pose.transforms[0].transform.rotation.z
                self.robot_pose_x = pose.transforms[0].transform.translation.x
                self.robot_pose_y = pose.transforms[0].transform.translation.y
                self.robot_pose_w = pose.transforms[0].transform.rotation.z
                if self.cancel_flg == 1 and self.location_append_num > 0:
                    if self.sub_state == 0:
                        rospy.sleep(0.5)
                        self.location_append_num = self.location_append_num -1
                        print 'state is: 0'
                        rospy.sleep(0.5)
                        self.sub_state = 1
                    if self.sub_state == 1:
                        print 'state is: 1'
                        print 'location append num is: ',self.location_append_num
                        rospy.sleep(1)
                        del self.location_list[self.location_append_num]
                        self.location_word = 'Null'
                        self.object_word = 'Null'
                        self.effective_location_word = 'Null'
                        self.effective_object_word = 'Null'
                        self.receive_location_num = 0
                        self.receive_object_num = 0
                        print 'location list is:'
                        print self.location_list
                        print ''
                        print 'cancel ok'
                        self.voice_cancel_pub.publish('success')
                        rospy.sleep(0.5)
                        self.sub_state = 2
                    if self.sub_state == 2:
                        self.cancel_flg = 0
                        pass
                elif self.cancel_flg == 1 and self.location_append_num == 0:
                    if self.sub_state == 0:
                        self.voice_cancel_pub.publish('failed')
                        rospy.sleep(0.5)
                        self.sub_state = 1
                    if self.sub_state == 1:
                        self.cancel_flg = 0
                        pass
                if self.location_word != 'Null':
                    if self.receive_location_num == 0:
                        self.effective_location_word = self.location_word
                        if self.effective_location_word == 'finish':
                            pass
                        else:
                            print 'effective location is:',self.effective_location_word
                            CMD = '/usr/bin/picospeaker %s' % 'location is %s' % self.effective_location_word
                            subprocess.call(CMD.strip().split(" "))
                            rospy.sleep(1)
                        self.location_pose_x = self.robot_pose_x
                        self.location_pose_y = self.robot_pose_y
                        self.location_pose_w = self.robot_pose_w
                        self.receive_location_num = 1
                        
                if self.object_word != 'Null':
                    if self.receive_object_num == 0:
                        self.effective_object_word = self.object_word
                        if self.effective_object_word == 'hoge':
                            print 'effective object is:',self.effective_location_word
                        else:
                            print 'effective object is:',self.effective_object_word
                            CMD = '/usr/bin/picospeaker %s' % 'object is %s' % self.effective_object_word
                            subprocess.call(CMD.strip().split(" "))
                            rospy.sleep(1)
                        self.receive_object_num = 1
                        
                if self.effective_location_word != 'Null' and self.effective_object_word != 'Null':
                    location_list = (self.effective_location_word + ',' + self.effective_object_word)
                    self.search_list_pub.publish(location_list)
                    self.location_list.append([self.effective_location_word,self.location_pose_x,self.location_pose_y,self.location_pose_w,self.effective_object_word])
                    print self.location_list[self.location_append_num]
                    if self.effective_location_word == 'finish':
                        print 'location list is:'
                        print self.location_list
                        CMD = '/usr/bin/picospeaker %s' % 'finish following'
                        subprocess.call(CMD.strip().split(" "))
                        rospy.sleep(0.5)
                        self.follow_result_pub.publish('succsess')
                    elif self.effective_location_word == 'entrance':
                        CMD = '/usr/bin/picospeaker %s' % 'append list'
                        subprocess.call(CMD.strip().split(" "))
                        rospy.sleep(1)
                        CMD = '/usr/bin/picospeaker %s' % 'word is entrance'
                        subprocess.call(CMD.strip().split(" "))
                        rospy.sleep(1)
                    else:
                        CMD = '/usr/bin/picospeaker %s' % 'append list'
                        subprocess.call(CMD.strip().split(" "))
                        rospy.sleep(1)
                        CMD = '/usr/bin/picospeaker %s' % 'words are %(location)s and %(object)s' % {'location':self.effective_location_word,'object':self.effective_object_word}
                        subprocess.call(CMD.strip().split(" "))
                        rospy.sleep(3)
                        print 'location list is:'
                        print self.location_list
                        print ''
                    self.location_append_num += 1
                    self.location_word = 'Null'
                    self.object_word = 'Null'
                    self.effective_location_word = 'Null'
                    self.effective_object_word = 'Null'
                    self.receive_location_num = 0
                    self.receive_object_num = 0
        except AttributeError:
            print 'error'
            pass

    def ReceiveLocation(self,location):
        self.location_word = location.data
        print self.location_word

    def ReceiveObject(self,object_word):
        self.object_word = object_word.data
        print self.object_word

    def CancelVoice(self,cansel):
        print 'receive cancel'
        self.sub_state = 0
        self.cancel_flg = 1
        
    def NavigateToDestination(self,destination):
        location_num = -1
        for location_num_i in range(len(self.location_list)):    
            if self.location_list[location_num_i][0] in destination.data:
                location_num = location_num_i
                print 'distination is:',destination.data
        if location_num == -1:
            print "not exist such object"
            return
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if ac.wait_for_server(rospy.Duration(5)) == 1:
            print "wait for action client rising up 0"
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'          # 地図座標系
        goal.target_pose.header.stamp = rospy.Time.now() # 現在時刻
        goal.target_pose.pose.position.x =  self.location_list[location_num][1]
        goal.target_pose.pose.position.y =  self.location_list[location_num][2]        
        q = tf.transformations.quaternion_from_euler(0, 0, self.location_list[location_num][3])
        goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        ac.send_goal(goal);        
        while not rospy.is_shutdown():
            print 'get state is',ac.get_state()
            if ac.get_state == 1 and reset_flg == 1:                
                print 'Got out of the obstacle'
                self.reset_pub.publish(0)
                reset_flg = 0
            if ac.get_state() == 3:
                print "goal"
                result = String()
                result = "succsess"
                if destination.data != 'entrance' or destination.data != 'oprater':
                    self.result_pub.publish(result)
                    #rospy.sleep(1)
                if destination.data == 'entrance':
                    exit()
                if destination.data == 'oprater':
                    self.result_pub.publish("return_op")
                break
            if ac.get_state() == 4 and reset_flg == 0:
                print 'Buried in obstacles'
                self.clear_costmap()
                self.reset_pub.publish(1)
                reset_flg = 1
                ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                if ac.wait_for_server(rospy.Duration(5)) == 1:
                    print "wait for action client rising up 1"
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'          # 地図座標系
                goal.target_pose.header.stamp = rospy.Time.now() # 現在時刻
                goal.target_pose.pose.position.x =  self.location_list[location_num][1]
                goal.target_pose.pose.position.y =  self.location_list[location_num][2]                
                q = tf.transformations.quaternion_from_euler(0, 0, self.location_list[location_num][3])
                goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
                ac.send_goal(goal);
        print "finish"
        
if __name__ == '__main__':
    rospy.init_node('ggi_navigation',anonymous=True)
    navigation = Navigation()
    rospy.spin()
    
