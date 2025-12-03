#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('ur5_4dof')

    # 1) Start Gazebo (default empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        )
    )

    # 2) Prepare the xacro → URDF command
    xacro_file = os.path.join(pkg, 'urdf', 'ur5_4dof.urdf.xacro')
    robot_desc_cmd = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' grip_pos_min:=0.0 grip_pos_max:=0.8',
        ' tty:=/dev/ttyUSB0 baudrate:=115200 parity:=none',
        ' data_bits:=8 stop_bit:=1 slave_id:=1'
    ])

    # 3) Publish robot_description to robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_desc_cmd, value_type=str)
        }]
    )

    # 4) Spawn the robot into Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ur5_4dof'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
    ])

