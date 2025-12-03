#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ur5_4dof')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ur5_4dof.urdf.xacro')

    # Build the URDF from Xacro on the fly:
    robot_desc_cmd = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        # pass any gripper args if needed, e.g. grip_pos_min...
    ])

    # Publish robot_description to robot_state_publisher:
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_desc_cmd, value_type=str)
        }]
    )

    # Also run a JointStatePublisher GUI so you can move joints if you like:
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([rsp_node, jsp_node])

