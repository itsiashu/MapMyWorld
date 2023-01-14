#!/bin/sh
#deploy turtlebot with in turtlebot_world(homeOffice world)
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/homeOffice.world" &
sleep 5
# Localize through amcl
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/my_robot.yaml" &
sleep 5
# Navigate through Rviz
xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch"