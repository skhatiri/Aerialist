#!/usr/bin/env python

import subprocess
import time
import rospy


def run_disparity_subprocess():
    command = 'bash -c "source /src/catkin_ws/devel/setup.bash; rosrun topic_tools transform /stereo/disparity /stereo/disparity_image sensor_msgs/Image \'m.image\'"'

    # Initialize the subprocess.Popen object
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output, errors = process.communicate()
    rospy.spin()


def launch_node():
    rospy.init_node("disparity_image_launcher")
    run_disparity_subprocess()


if __name__ == '__main__':  # Parse the box size from the command-line argument
    try:
        # obstacle_list = [[13, 5, 0, 'tree', 'tree_1'], [23, 15, 0, 'tree', 'tree_2']]
        launch_node()
    except rospy.ROSInterruptException:
        pass
