#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/homeOffice.world" &
sleep 15
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/my_robot.yaml" &
sleep 15
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 20
xterm -e "rosrun add_markers add_markers" &
sleep 3
xterm -e "rosrun pick_objects pick_objects"