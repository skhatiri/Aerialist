#!/usr/bin/env python

import rospy
import os
import sys
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion


def spawn_apartment():
    # Initialize the ROS node

    apartment_name = sys.argv[1]
    x = float(sys.argv[2])
    y = float(sys.argv[3])
    z = float(sys.argv[4])
    r = float(sys.argv[5])
    # Initialize the ROS node
    rospy.init_node(apartment_name)

    # Wait for the Gazebo spawn service
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Define the SDF model for a box with the specified size
    apartment_sdf = """
    <?xml version="1.0" ?>
    <sdf version="1.4">
    <include>
      <name>{3}</name>
      <uri>model://apartment</uri>
      <pose>{0} {1} {2} 0 0 0</pose>
    </include>
    </sdf>
    """.format(x, y, z, apartment_name)

    # Set the model name and pose
    model_name = apartment_name
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z  # Adjust the Z position based on box size
    initial_pose.orientation = Quaternion(0, 0, r, 1)

    # Call the Gazebo spawn service to add the box model
    try:
        spawn_sdf_model(model_name, apartment_sdf, "/", initial_pose, "world")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == '__main__':  # Parse the box size from the command-line argument
    try:
        spawn_apartment()
    except rospy.ROSInterruptException:
        pass
