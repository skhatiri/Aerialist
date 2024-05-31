#!/usr/bin/env python

import rospy
import os
import sys
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Quaternion, Point


def spawn_tree():
    tree_name = sys.argv[1]
    x = float(sys.argv[2])
    y = float(sys.argv[3])
    z = float(sys.argv[4])
    r = float(sys.argv[5])

    # Initialize the ROS node
    rospy.init_node(tree_name)

    # Wait for the Gazebo spawn service
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_model_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    # Define the SDF model for a box with the specified size
    tree_sdf = """
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <model name="pine_tree">
        <static>true</static>
        <link name="link">
          <collision name="collision">
            <geometry>
              <mesh>
                <uri>model://pine_tree/meshes/pine_tree.dae</uri>
              </mesh>
            </geometry>
          </collision>
          <visual name="branch">
            <geometry>
              <mesh>
                <uri>model://pine_tree/meshes/pine_tree.dae</uri>
                <submesh>
                  <name>Branch</name>
                </submesh>
              </mesh>
            </geometry>
            <material>
              <script>
                <uri>model://pine_tree/materials/scripts/</uri>
                <uri>model://pine_tree/materials/textures/</uri>
                <name>PineTree/Branch</name>
              </script>
            </material>
          </visual>
          <visual name="bark">
            <geometry>
              <mesh>
                <uri>model://pine_tree/meshes/pine_tree.dae</uri>
                <submesh>
                  <name>Bark</name>
                </submesh>
              </mesh>
            </geometry>
            <material>
              <script>
                <uri>model://pine_tree/materials/scripts/</uri>
                <uri>model://pine_tree/materials/textures/</uri>
                <name>PineTree/Bark</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>

    """

    # Set the model name and pose
    spawn_request = SpawnModelRequest()
    spawn_request.model_name = tree_name
    spawn_request.model_xml = tree_sdf
    spawn_request.initial_pose = Pose(
        position=Point(x, y, z)
        # orientation=
        # Quaternion(x=0, y=0, z=r, w=1)
    )

    # Call the Gazebo spawn service to add the box model
    try:
        spawn_model_client(spawn_request)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == "__main__":  # Parse the box size from the command-line argument
    try:
        spawn_tree()
    except rospy.ROSInterruptException:
        pass
