#!/bin/sh
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/homeOffice.world" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/my_robot.yaml" &
sleep 5
xterm -e "rosrun add_markers display_markers" &
sleep 10
xterm -e "rosrun add_markers add_markers"