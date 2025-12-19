import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('robot_dino').find('robot_dino')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    urdf_path  =  os.path.join(pkg_share, 'urdf', 'robot_dino.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': "-r empty.sdf"}.items(),
    )

    create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot',
                   '-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.5',
                ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'frame_prefix': "robot/"}
        ]
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_share, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'robot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    rqt_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen',
        arguments=[
            '--ros-args', '--remap', '/cmd_vel:=/robot/cmd_vel'
        ]
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        robot_state_publisher,
        rviz,
        rqt_steering,
        TimerAction(
            period=5.0,
            actions=[create])
    ])