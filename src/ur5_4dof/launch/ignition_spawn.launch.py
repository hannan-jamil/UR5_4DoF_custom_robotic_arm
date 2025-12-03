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

    # 1) Launch Ignition Gazebo (Fortress/Garden) with an empty world
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            )
        ),
        launch_arguments={'ign_args': '-r empty.sdf'}.items()  # -r = run GUI
    )

    # 2) robot_state_publisher (xacro → URDF, as a ROS param)
    xacro_file = os.path.join(pkg, 'urdf', 'ur5_4dof.urdf.xacro')
    robot_desc_cmd = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' grip_pos_min:=0.0', ' grip_pos_max:=0.8',
        ' tty:=/dev/ttyUSB0', ' baudrate:=115200',
        ' parity:=none', ' data_bits:=8', ' stop_bit:=1',
        ' slave_id:=1'
    ])
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_desc_cmd, value_type=str)
        }]
    )

    # 3) Spawn the URDF into Ignition via ros_gz_sim's "create" command
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ur5_4dof',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        rsp,
        spawn,
    ])

