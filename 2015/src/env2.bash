#!/bin/bash
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  /bin/echo "Calling env.sh without arguments is not supported anymore. Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# ensure to not use different shell type which was set before
CATKIN_SHELL=sh

# setting up ROS sourcing and IP addreses
export ROS_IP=`hostname -I`
export ROS_WS=/home/rachel/catkin_ws
export VREP_ROOT_DIR=/home/rachel/V-REP_PRO_EDU_V3_2_0_rev6_64_Linux
source /opt/ros/indigo/setup.bash
source /home/rachel/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.245:11311
export PATH=$ROS_ROOT/bin:$PATH

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(cd "`dirname "$0"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"
exec "$@"

