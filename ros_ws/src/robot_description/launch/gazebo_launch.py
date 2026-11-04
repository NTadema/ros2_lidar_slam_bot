import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
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

    models_path = os.path.join(pkg_share, 'models')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            models_path,
            ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    default_world = os.path.join(
        pkg_share,
        'worlds',
        'obstacles.world'
    )

    gazebo_launch_file = os.path.join(
        ros_gz_sim_share,
        'launch',
        'gz_sim.launch.py'
    )

    world_file = LaunchConfiguration('world')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to the Gazebo world file'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    # Spawn the robot into Gazebo using its URDF description.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'lidar_slam_bot'
        ],
        output='screen'
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
            'gz_args': ['-r ', world_file]
        }.items()
    )

    return LaunchDescription([
        declare_world,
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        joint_state_bridge,
        wheel_odometry_node
    ])
