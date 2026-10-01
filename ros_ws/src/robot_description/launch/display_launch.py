import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'robot_description'

    pkg_share = get_package_share_directory(package_name)

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'robot.urdf.xacro'
    )

    robot_description = Command([
        'xacro ',
        xacro_file
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(robot_description, value_type=str)
            }
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])
