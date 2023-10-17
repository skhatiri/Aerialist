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
    wind_name = sys.argv[1]
    # Initialize the ROS node
    rospy.init_node(wind_name)

    # Wait for the Gazebo spawn service
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Define the SDF model for a box with the specified size
    wind_sdf = """
    <?xml version="1.0" ?>
    <sdf version="1.4">
        <plugin name='wind_plugin' filename='libgazebo_wind_plugin.so'>
            <frameId>base_link</frameId>
            <robotNamespace/>
            <windVelocityMean>50.0</windVelocityMean>
            <windVelocityMax>50.0</windVelocityMax>
            <windVelocityVariance>0</windVelocityVariance>
            <windDirectionMean>0 1 0</windDirectionMean>
            <windDirectionVariance>0</windDirectionVariance>
            <windGustStart>0</windGustStart>
            <windGustDuration>0</windGustDuration>
            <windGustVelocityMean>0</windGustVelocityMean>
            <windGustVelocityMax>50.0</windGustVelocityMax>
            <windGustVelocityVariance>0</windGustVelocityVariance>
            <windGustDirectionMean>1 0 0</windGustDirectionMean>
            <windGustDirectionVariance>0</windGustDirectionVariance>
            <windPubTopic>world_wind</windPubTopic>
        </plugin>
    </sdf>
    """

    # Set the model name and pose
    model_name = wind_name
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0
    initial_pose.position.z = 0  # Adjust the Z position based on box size

    # Call the Gazebo spawn service to add the box model
    try:
        spawn_sdf_model(model_name, wind_sdf, "/", initial_pose, "world")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == '__main__':  # Parse the box size from the command-line argument
    try:
        spawn_tree()
    except rospy.ROSInterruptException:
        pass
