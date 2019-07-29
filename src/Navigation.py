#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import math
import time
import tf
import actionlib
import std_srvs.srv
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from std_msgs.msg import String,Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Quaternion

class Navigation:
    def __init__(self):
        self.request_sub = rospy.Subscriber('/navigation/destination',String,self.NavigateToDestination)
   
        self.result_pub = rospy.Publisher('/navigation/result',String,queue_size= 10)
   
        self.location_list = [["kichen",-0.35,-0.532,-3.088],
                              ["entrance",2.52,-6,1.55],
                              ["living",2.77,-3,0.015],
                              ["shelf",2.062,-0.217,0.015]]
        
    def NavigateToDestination(self,destination):
        location_num = -1
        for location_num_i in range(len(self.location_list)):
            if self.location_list[location_num_i][0] in destination.data:
                location_num = location_num_i
        if location_num == -1:
            print "not exist such location"
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
            print 'get satate is',ac.get_state()
            if ac.get_state() == 1 and reset_flg == 1:
                print 'Got out of the obstacle'
                self.reset_pub.publish(0)
                reset_flg = 0
            if ac.get_state() == 3:
                print "goal"
                result = String()
                result = "succsess"
                self.result_pub.publish(result)
                time.sleep(3)
                break
            if ac.get_state() == 4 and reset_flg == 0:
                print 'Buried in obstacles'
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
    rospy.init_node('navigation',anonymous=True)
    navigation = Navigation()
    rospy.spin()
