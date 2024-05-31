#!/usr/bin/env python

import rospy
import os
import sys
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Quaternion, Point


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
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_model_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

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
    """.format(
        x, y, z, apartment_name
    )

    # Set the model name and pose
    spawn_request = SpawnModelRequest()
    spawn_request.model_name = apartment_name
    spawn_request.model_xml = apartment_sdf
    spawn_request.initial_pose = Pose(
        position=Point(x, y, z), orientation=Quaternion(x=0, y=0, z=r, w=1)
    )
    # Call the Gazebo spawn service to add the box model
    try:
        spawn_model_client(spawn_request)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == "__main__":  # Parse the box size from the command-line argument
    try:
        spawn_apartment()
    except rospy.ROSInterruptException:
        pass
