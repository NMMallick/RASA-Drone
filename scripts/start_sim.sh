#!/usr/bin/env bash

DONT_RUN=1 make px4_sitl_default gazebo
source ~/ras/RASA-Drone/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

## launch the gazebo simulator 
gnome-terminal --tab \ --title=newTab --working-directory=`pwd` -- bash -c "roslaunch px4 posix_sitl.launch; exec bash -i" \

## launch the roscore (with mavros) 
gnome-terminal --tab \ --title=newTab --working-directory=`pwd` -- bash -c "sleep 2 && roslaunch mavros px4.launch fcu_url:='udp://:14540@127.0.0.1:14557'; exec bash -i" \
