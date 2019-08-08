## GPSR setup
$ sh mimi_setup.sh
$ roslaunch e_manipulation motor_setup.launch  
$ rosrun ti_gpsr gpsr_navi.sh  
$ rosrun chaser chaser  
$ rosrun e_object_recognizer object_recognizer.py  
$ rosrun e_grasping_position_detector e_grasping_position_detector  
$ rosrun manipulation manipulation.py  
$ roslaunch darknet_ros darknet_ros.launch    
$ rosrun ti_gpsr Navigation.py  
$ rosrun ti_gpsr gpsrNode.py
## How to use  
/command_listのaction,location,obj,answerにそれぞれ文字列を入れて送って下さい  
gpsrNodeでこのトピックを受け取り、代入された文字列にしたがって処理を行います  
処理が終わり次第、Trueを返します
