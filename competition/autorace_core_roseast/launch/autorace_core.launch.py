from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autorace_core_roseast_mission_solver',
            executable='cone_priority_detector',
            name='cone_priority_detector',
            output='screen'
        ),
        Node(
            package='autorace_core_roseast_mission_solver',
            executable='pid_controller',
            name='pid_controller',
            output='screen',
            parameters=[{'kp': 0.6, 'ki': 0.0, 'kd': 0.05, 'speed': 0.1}]
        ),
        Node(
            package='autorace_core_roseast_mission_solver',
            executable='aruco',
            name='aruco',
            output='screen',
            parameters=[{
                'debug': False,
                'detect_start': 154.0,
                'detect_end': 164.0
            }]
        ),
        Node(
            package='autorace_core_roseast_mission_solver',
            executable='traffic_light',
            name='traffic_light',
            output='screen',
            parameters=[{'target_bgr': [0, 189, 0],
                         'bgr_tolerance': [40, 60, 40],
                         'min_area': 10,
                         'debounce_frames': 3}]
        ),
        # Node(
        #     package='autorace_core_roseast_mission_solver',
        #     executable='overtaking',
        #     name='overtaking',
        #     output='screen'
        # ),


    ])