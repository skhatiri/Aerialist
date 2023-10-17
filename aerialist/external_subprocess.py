import subprocess, time, shlex, os
from decouple import config

histogram_file = "get_histogram_image.py"
pointcloud_file = "get_pointcloud_image.py"
disparity_file = "get_disparity_image.py"
raw_file = "get_raw_image.py"
box_file = "modify_box.py"
box_pattern_file = "modify_pattern_box.py"
tree_file = "modify_tree.py"
wind_file = "wind_handler.py"
light_file = "light_handler.py"

CATKIN_DIR = config("CATKIN_HOME")
path = CATKIN_DIR + "src/avoidance/histogram_image/scripts/"
AERIALIST_DIR = config("AERIALIST_HOME")
path2 = AERIALIST_DIR + "aerialist/"


def run_subprocess(histogram_folder, pointcloud_folder, disparity_folder, raw_folder):
    # histogram_pro = subprocess.Popen(["python2", path2 + histogram_file, histogram_folder])
    pointcloud_pro = subprocess.Popen(["python2", path2 + pointcloud_file, pointcloud_folder])
    # box_1 = subprocess.Popen(["python2", path2 + box_pattern_file, "box_5", "1", "1", "1", "5", "8", "0"])
    # box_2 = subprocess.Popen(["python2", path2 + box_file, "box_4", "1", "1", "1", "-20", "50", "0"])
    # box_2 = subprocess.Popen(["python2", path2 + box_file, "box_4", "1", "1", "1", "5", "8", "0"])
    # tree_1 = subprocess.Popen(["python2", path2 + tree_file, "tree_1", "10", "0", "0"])
    # tree_2 = subprocess.Popen(["python2", path2 + tree_file, "tree_2", "15", "-2", "0"])
    # wind = subprocess.Popen(["python2", path2 + wind_file])
    disparity_m_pro = subprocess.Popen(["python3", path2 + "disparity_image_launch.py"])
    return disparity_m_pro
    # disparity_pro = subprocess.Popen(["python2", path2 + disparity_file, disparity_folder])
    # raw_pro = subprocess.Popen(["python2", path2 + raw_file, raw_folder])
    # return histogram_pro, disparity_pro, raw_pro, disparity_m_pro


def run_tree_subprocess(tree_count, x, y, z):
    tree_proc = subprocess.Popen(["python2", path2 + tree_file, tree_count, x, y, z])
    # tree_2 = subprocess.Popen(["python2", path2 + tree_file, "tree_2", "15", "-2", "0"])


def run_box_subprocess(box_count, l, w, h, x, y, z):
    subprocess.Popen(["python2", path2 + box_file, box_count, l, w, h, x, y, z])


# def spawn_box_with_pattern(box_count, l, w, h, x, y):
#     subprocess.Popen(["python2", path2 + box_pattern_file, "box_5", "6", "6", "6", "-20", "50", "0"])