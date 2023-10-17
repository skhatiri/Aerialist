#!/usr/bin/env python

import rospy
import os
import sys
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def spawn_tree():
    # Initialize the ROS node

    print(len(sys.argv))
    print(sys.argv)
    tree_name = sys.argv[1]
    x = int(sys.argv[2])
    y = int(sys.argv[3])
    z = int(sys.argv[4])
    # Initialize the ROS node
    rospy.init_node(tree_name)

    # Wait for the Gazebo spawn service
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Define the SDF model for a box with the specified size
    tree_sdf = """
    <?xml version="1.0" ?>
    <sdf version="1.4">
     <include>
      <name>{3}</name>
      <uri>model://pine_tree</uri>
      <pose>{0} {1} {2}  0 0 0</pose>
    </include>
    </sdf>
    """.format(x, y, z, tree_name)

    # Set the model name and pose
    model_name = tree_name
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z  # Adjust the Z position based on box size

    # Call the Gazebo spawn service to add the box model
    try:
        spawn_sdf_model(model_name, tree_sdf, "/", initial_pose, "world")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == '__main__':  # Parse the box size from the command-line argument
    try:
        spawn_tree()
    except rospy.ROSInterruptException:
        pass
