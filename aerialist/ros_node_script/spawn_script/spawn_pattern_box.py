#!/usr/bin/env python

import rospy
import os
import sys
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def spawn_box():
    print(len(sys.argv))
    print(sys.argv)
    box_name = sys.argv[1]
    l = int(sys.argv[2])
    w = int(sys.argv[3])
    h = int(sys.argv[4])
    x = int(sys.argv[5])
    y = int(sys.argv[6])
    z = int(sys.argv[7])
    # Initialize the ROS node
    rospy.init_node(box_name)
    # rospy.spin()

    # Wait for the Gazebo spawn service
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    # Define the SDF model for a box with the specified size
    box_sdf = """
    <?xml version="1.0" ?>
    <sdf version='1.4'>
      <model name='box'>
        <pose>{3} {4} {5} 0 -0 0</pose>
          <inertial>
            <pose frame=''>{3} {4} {5} 0 -0 0</pose>
            <mass>100</mass>
            <inertia>
              <ixx>1088</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>550</iyy>
              <iyz>0</iyz>
              <izz>1088</izz>
            </inertia>
          </inertial>
        <link name='box'>
          <collision name='collision'>
            <geometry>
              <box>
                <size>{0} {1} {2}</size>
              </box>
            </geometry>
          </collision>
          <visual name='visual'>
            <geometry>
              <box>
                <size>{0} {1} {2}</size>
              </box>
              <mesh>
                <scale>6 6 10</scale>
                <uri>/home/prasun/Downloads/Test-Aerialist/Aerialist/aerialist/resources/simulation/models/big_box/meshes/big_box_.dae</uri>
              </mesh>
            </geometry>
          </visual>
        </link>
      </model>
    </sdf>
    """.format(
        l, w, h, x, y, z
    )

    # Set the model name and pose
    model_name = box_name
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z  # Adjust the Z position based on box size

    # Call the Gazebo spawn service to add the box model
    try:
        spawn_sdf_model(model_name, box_sdf, "/", initial_pose, "world")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == "__main__":  # Parse the box size from the command-line argument
    try:
        spawn_box()
    except rospy.ROSInterruptException:
        pass
