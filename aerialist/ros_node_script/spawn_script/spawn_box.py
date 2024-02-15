#!/usr/bin/env python

import rospy
import os
import sys
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion


def spawn_box():
    box_name = sys.argv[1]
    l = float(sys.argv[2])
    w = float(sys.argv[3])
    h = float(sys.argv[4])
    x = float(sys.argv[5])
    y = float(sys.argv[6])
    z = float(sys.argv[7])
    r = float(sys.argv[8])
    # Initialize the ROS node
    rospy.init_node(box_name)
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Define the SDF model for a box with the specified size
    box_sdf = """
    <?xml version="1.0" ?>
    <sdf version="1.4">
      <model name="box">
        <static>true</static>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>{0} {1} {2}</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>{0} {1} {2}</size>
              </box>
            </geometry>
          </visual>
        </link>
      </model>
    </sdf>
    """.format(l, w, h)

    # Set the model name and pose
    spawn_request = SpawnModelRequest()
    spawn_request.model_name = box_name
    spawn_request.model_xml = box_sdf
    z_bottom = float(z) + float(h) / 2.0
    spawn_request.initial_pose = Pose(
        position=Point(x, y, z_bottom),
        orientation=
        Quaternion(x=0, y=0, z=r, w=1)
    )

    # Call the Gazebo spawn service to add the box model
    try:
        # spawn_sdf_model(model_name, box_sdf, "/", initial_pose, "world")
        spawn_model_client(spawn_request)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == '__main__':  # Parse the box size from the command-line argument
    try:
        spawn_box()
    except rospy.ROSInterruptException:
        pass
