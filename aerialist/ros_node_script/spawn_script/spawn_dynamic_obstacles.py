#!/usr/bin/env python
import subprocess
import rospy
import roslaunch


def run_tree_subprocess(tree_name, x, y, z):
    subprocess.Popen(
        ["python2", "/src/catkin_ws/src/avoidance/spawn_obstacles/src/nodes/spawn_tree.py", tree_name, x, y, z])


def run_box_subprocess(box_name, l, w, h, x, y, z):
    subprocess.Popen(
        ["python2", "/src/catkin_ws/src/avoidance/spawn_obstacles/src/nodes/spawn_box.py", box_name, l, w, h, x, y, z])


def run_apartment_subprocess(apartment_name, x, y, z):
    subprocess.Popen(
        ["python2", "/src/catkin_ws/src/avoidance/spawn_obstacles/src/nodes/spawn_apartment.py", apartment_name, x, y,
         z])


def launch_node():
    # Iterate over the array and launch nodes
    rospy.init_node("dynamic_obstacle_launcher")
    list1 = rospy.get_param('/dynamic_obstacle_launcher/obstacle_string').split(',end,')
    obstacle_list = []
    for temp in list1:
        if len(temp) > 0:
            obstacle_list.append(temp.split(','))

    for temp in obstacle_list:
        node_name = temp[4]
        if temp[3] == "tree":
            x, y, z, node_type, node_name_arg = temp[0], temp[1], temp[2], temp[3], temp[4]
            run_tree_subprocess(node_name, str(x), str(y), str(z))
        elif temp[3] == "apartment":
            x, y, z, node_type, node_name_arg = temp[0], temp[1], temp[2], temp[3], temp[4]
            run_apartment_subprocess(node_name, str(x), str(y), str(z))
        elif temp[3] == "box":
            x, y, z, node_type, node_name_arg, l, w, h = temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7]
            run_box_subprocess(node_name, str(l), str(w), str(h), str(x), str(y), str(z))

    rospy.spin()


if __name__ == '__main__':  # Parse the box size from the command-line argument
    try:
        # obstacle_list = [[13, 5, 0, 'tree', 'tree_1'], [23, 15, 0, 'tree', 'tree_2']]
        launch_node()
    except rospy.ROSInterruptException:
        pass
