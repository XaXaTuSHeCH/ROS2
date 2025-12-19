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
            description='Radius of rotation for carrot around turtles'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation2',
            default_value='1',
            description='Direction of rotation: 1 for clockwise, -1 for counter-clockwise'
        ),
        DeclareLaunchArgument(
            'switch_threshold',
            default_value='1.0',
            description='Distance threshold for auto-switching targets'
        ),
        
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('turtle_multi_target'), 'launch', 'turtle_tf2_dynamic_frame_demo_launch.py'
            ]),
            launch_arguments={'target_frame': 'carrot1'}.items(),
        ),
        
        Node(
            package='turtle_multi_target',
            executable='spawn_turtle_node',
            name='spawn_turtle3',
            parameters=[{
                'turtle_name': 'turtle3',
                'x': 3.0,
                'y': 3.0,
                'theta': 0.0
            }]
        ),
        
        Node(
            package='turtle_multi_target',
            executable='turtle_tf2_broadcaster',
            name='broadcaster3',
            parameters=[{'turtlename': 'turtle3'}]
        ),
        
        Node(
            package='turtle_multi_target',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster2',
            parameters=[{
                'radius': LaunchConfiguration('radius'),
                'direction_of_rotation': LaunchConfiguration('direction_of_rotation2'),
                'parent_frame': 'turtle3',
                'child_frame': 'carrot2'
            }]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_target_broadcaster',
            arguments=['8.0', '2.0', '0.0', '0.0', '0.0', '0.0', 'world', 'static_target']
        ),
    ])