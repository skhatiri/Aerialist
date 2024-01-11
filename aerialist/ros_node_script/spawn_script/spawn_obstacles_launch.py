#!/usr/bin/env python

import rospy
import roslaunch


def launch_node(node_name, x, y, z, node_name_arg):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Create a roslaunch parent and launch the node
    parent = roslaunch.parent.ROSLaunchParent(uuid, [
        (roslaunch.rlutil.resolve_launch_arguments('spawn_obstacles', 'your_launch_file.launch'))])
    parent.start()

    # Pass arguments to the node
    param_dict = {'x': str(x), 'y': str(y), 'z': str(z), 'name': node_name_arg}
    rospy.set_param(node_name, param_dict)

    parent.shutdown()


def main():
    # Get the array_of_arrays from the ROS parameter server
    obstacle_list = rospy.get_param('obstacle_list', [])

    # Iterate over the array and launch nodes
    for idx, temp in enumerate(obstacle_list):
        node_name = f"spawn_{temp[3]}_{idx}"
        x, y, z, node_type, node_name_arg = temp[0], temp[1], temp[2], temp[3], temp[4]
        launch_node(node_name, x, y, z, node_name_arg)


if __name__ == '__main__':
    rospy.init_node('dynamic_node_launcher')
    main()
