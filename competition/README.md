# AutoRace 2025
A ROS2 metapackage that has necessary packages for AutoRace 2025 challenge.

## Usage for AutoRace 2025

1. Install dependencies

    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
    ```

Put folders:
/autorace_core_roseast
/autorace_core_roseast_mission_solver
into src folder

2. Build the project

    ```bash
    colcon build
    ```

3. Source the workspace

    ```bash
    . ~/ros2_ws/install/setup.bash
    ```

4. Launch the simulation

    ```bash
    ros2 launch robot_bringup autorace_2025.launch.py
    ```

5. Run your own launch file that controls the robot

    ```bash
    ros2 launch autorace_core_roseast autorace_core.launch.py
    ```

6. Run the referee

    ```bash
    ros2 run referee_console mission_autorace_2025_referee
    ```

**Good luck!**