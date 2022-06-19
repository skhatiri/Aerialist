#!/bin/bash
source /src/catkin_ws/devel/setup.bash &&
DONT_RUN=1 make -C /src/PX4-Autopilot/ px4_sitl_default gazebo &&
. /src/PX4-Autopilot/Tools/setup_gazebo.bash /src/PX4-Autopilot/ /src/PX4-Autopilot/build/px4_sitl_default &&
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/src/PX4-Autopilot &&
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/src/catkin_ws/src/avoidance/avoidance/sim/models:/src/catkin_ws/src/avoidance/avoidance/sim/worlds" >> ~/.bashrc &&
exec roslaunch resources/simulation/collision_prevention.launch gui:=false rviz:=false world_file_name:=collision_prevention obst:=false