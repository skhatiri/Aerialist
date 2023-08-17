import subprocess, time, shlex, os
from decouple import config

histogram_file = "get_histogram_image.py"
pointcloud_file = "get_pointcloud_image.py"
disparity_file = "get_disparity_image.py"
raw_file = "get_raw_image.py"

CATKIN_DIR = config("CATKIN_HOME")
path = CATKIN_DIR+"src/avoidance/histogram_image/scripts/"
AERIALIST_DIR = config("AERIALIST_HOME")
path2 = AERIALIST_DIR+"aerialist/"


def run_subprocess(histogram_folder, pointcloud_folder, disparity_folder, raw_folder):
    # histogram_pro = subprocess.Popen(["python2", path2 + histogram_file, histogram_folder])
    # pointcloud_pro = subprocess.Popen(["python2", path2 + pointcloud_file, pointcloud_folder])
    disparity_m_pro = subprocess.Popen(["python3", path2 + "disparity_image_launch.py"])
    return disparity_m_pro
    # disparity_pro = subprocess.Popen(["python2", path2 + disparity_file, disparity_folder])
    # raw_pro = subprocess.Popen(["python2", path2 + raw_file, raw_folder])
    # return histogram_pro, disparity_pro, raw_pro, disparity_m_pro
