#!/usr/bin/env python
import subprocess
import rospy
import roslaunch


def run_tree_subprocess(tree_name, x, y, z, r):
    subprocess.Popen(
        ["python2", "/src/catkin_ws/src/avoidance/spawn_obstacles/src/nodes/spawn_tree.py", tree_name, x, y, z, r])


def run_box_subprocess(box_name, l, w, h, x, y, z, r):
    subprocess.Popen(
        ["python2", "/src/catkin_ws/src/avoidance/spawn_obstacles/src/nodes/spawn_box.py", box_name, l, w, h, x, y, z, r])


def run_apartment_subprocess(apartment_name, x, y, z, r):
    subprocess.Popen(
        ["python2", "/src/catkin_ws/src/avoidance/spawn_obstacles/src/nodes/spawn_apartment.py", apartment_name, x, y,
         z, r])


def launch_node():
    # Iterate over the array and launch nodes
    rospy.init_node("dynamic_obstacle_launcher")
    list1 = rospy.get_param('/dynamic_obstacle_launcher/obstacle_string').split(',end,')
    obstacle_list = []
    for temp in list1:
        if len(temp) > 0:
            obstacle_list.append(temp.split(','))

    for temp in obstacle_list:
        node_name = temp[5]
        if temp[4] == "tree":
            x, y, z, r, node_type, node_name_arg = temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]
            run_tree_subprocess(node_name, str(x), str(y), str(z), str(r))
        elif temp[4] == "apartment":
            x, y, z, r, node_type, node_name_arg = temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]
            run_apartment_subprocess(node_name, str(x), str(y), str(z), str(r))
        elif temp[4] == "box":
            x, y, z, r, node_type, node_name_arg, l, w, h = temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7], temp[8]
            run_box_subprocess(node_name, str(l), str(w), str(h), str(x), str(y), str(z), str(r))

    rospy.spin()


if __name__ == '__main__':  # Parse the box size from the command-line argument
    try:
        # obstacle_list = [[13, 5, 0, 'tree', 'tree_1'], [23, 15, 0, 'tree', 'tree_2']]
        launch_node()
    except rospy.ROSInterruptException:
        pass
