xterm -geometry 80x5+0+450 -e "/opt/ros/kinetic/bin/roslaunch camera_tf start_camera_tf.launch" &
sleep 3s
xterm -geometry 80x5+0+560 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/athome/map/sisoujyou2.yaml" &
sleep 7s
xterm -geometry 80x5+0+670 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_rviz_launchers view_navigation.launch"
