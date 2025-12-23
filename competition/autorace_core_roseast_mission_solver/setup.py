from setuptools import find_packages, setup

package_name = 'autorace_core_roseast_mission_solver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_detector = autorace_core_roseast_mission_solver.line_detector:main',
            'pid_controller = autorace_core_roseast_mission_solver.pid_controller:main',
            'aruco = autorace_core_roseast_mission_solver.aruco:main',
            'traffic_light = autorace_core_roseast_mission_solver.traffic_light:main',
            'cone_priority_detector = autorace_core_roseast_mission_solver.cone_priority_detector:main',
            # 'overtaking = autorace_core_roseast_mission_solver.overtaking:main',
        ],
    },
)
