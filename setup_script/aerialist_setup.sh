#!/bin/bash

cd ~
git clone https://github.com/skhatiri/Aerialist.git




source ~/anaconda3/bin/activate Aerialist
cd ~/Aerialist
pip3 install -r ~/Aerialist/requirements.txt
pip3 install kconfiglib
pip3 install empy
pip3 install future
pip3 install packaging
pip3 install toml
pip3 install numpy
pip3 install jinja2
pip3 install cython
pip3 install pyyaml
pip3 install future
pip3 install python-decouple
pip3 install --user pyros-genmsg
pip3 install --user jsonschema

cd ~
PWD=$(pwd)
echo $PWD
CATKIN_HOME=$PWD"/catkin_ws/"
PX4_HOME=$PWD"/Firmware/"
ROS_HOME=$PWD"/.ros/"
echo $PX4_HOME
cd ~/Aerialist
cp template.env .env
SEARCH_STRING="PX4_HOME=/src/PX4-Autopilot/"
REPLACE_STRING="PX4_HOME="$PX4_HOME
sed -i "s|$SEARCH_STRING|$REPLACE_STRING|g" .env
SEARCH_STRING="CATKIN_HOME=/src/catkin_ws/"
REPLACE_STRING="CATKIN_HOME="$CATKIN_HOME
sed -i "s|$SEARCH_STRING|$REPLACE_STRING|g" .env
SEARCH_STRING="ROS_HOME=/root/.ros/"
REPLACE_STRING="ROS_HOME="$ROS_HOME
sed -i "s|$SEARCH_STRING|$REPLACE_STRING|g" .env
SEARCH_STRING="SIMULATOR=gazebo"
REPLACE_STRING="SIMULATOR=ros"
sed -i "s|$SEARCH_STRING|$REPLACE_STRING|g" .env
SEARCH_STRING="DRONE=sitl"
REPLACE_STRING="DRONE=ros"
sed -i "s|$SEARCH_STRING|$REPLACE_STRING|g" .env
sed -i 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Firmware' "~/.bashrc"


make -C /home/aerialist/Firmware px4_sitl_default gazebo &

# Capture the process ID (PID) of the make command
make_pid=$!

# Wait for the build to complete (you can adjust the timeout as needed)
# You can also check the build process output for specific completion messages
wait $make_pid

# Check if the make command is still running
if ps -p $make_pid > /dev/null; then
    echo "Terminating the build process..."
    # Terminate the make command
    kill $make_pid
else
    echo "Build completed successfully."
fi

# Continue with other commands in your script
# ...

# Optionally, you can use an exit code to indicate success or failure
# Exit with 0 if the build completed successfully, or a different code if it failed
if ps -p $make_pid > /dev/null; then
    exit 1  # Build failed
else
    exit 0  # Build succeeded
fi


exit 0
