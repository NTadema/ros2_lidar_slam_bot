import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'robot_description'

    pkg_share = get_package_share_directory(package_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'robot.urdf.xacro'
    )

    gazebo_launch_file = os.path.join(
        ros_gz_sim_share,
        'launch',
        'gz_sim.launch.py'
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

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/lidar_slam_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            (
                '/world/empty/model/lidar_slam_bot/joint_state',
                '/joint_states'
            ),
        ],
        output='screen'
    )

    wheel_odometry_node = Node(
        package='wheel_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        ros_gz_bridge,
        joint_state_bridge,
        wheel_odometry_node
    ])
