from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'radius',
            default_value='2.0',
            description='Radius of rotation for carrot around turtle1'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation',
            default_value='1',
            description='Direction of rotation: 1 for clockwise, -1 for counter-clockwise'
        ),
        
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('turtle_multi_target'), 'launch', 'turtle_tf2_demo_launch.py'
            ]),
            launch_arguments={'target_frame': 'carrot1'}.items(),
        ),
        
        Node(
            package='turtle_multi_target',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster',
            parameters=[{
                'radius': LaunchConfiguration('radius'),
                'direction_of_rotation': LaunchConfiguration('direction_of_rotation')
            }]
        ),
    ])
