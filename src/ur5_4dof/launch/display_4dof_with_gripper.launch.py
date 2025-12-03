from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = FindPackageShare('ur5_4dof')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'ur5_4dof.urdf.xacro'])
    rviz_cfg   = PathJoinSubstitution([pkg, 'rviz', 'view_robot.rviz'])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', xacro_file,
                ' grip_pos_min:=0.0',
                ' grip_pos_max:=0.8',
                ' tty:=/dev/ttyUSB0',
                ' baudrate:=115200',
                ' parity:=none',
                ' data_bits:=8',
                ' stop_bit:=1',
                ' slave_id:=1'
            ]),
            value_type=str
        )
    }

    return LaunchDescription([
        # Publishing the combined URDF to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # GUI sliders for all joints (including the gripper fingers)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz, loading .rviz config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg]
        ),
    ])

