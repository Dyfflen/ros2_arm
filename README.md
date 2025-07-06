# ros2_arm
![Robot Arm](https://github.com/Dyfflen/ros2_arm/blob/main/media/robot%20arm%20-%20gazebo%20&%20rviz.png?raw=true)

## Dependencies:
### [Ubuntu 22.04.5 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
### [ROS 2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html)

## Terminal 1:
```
cd <your_ws>
colcon build
source install/setup.bash
ros2 launch robotarm_gazebo robotarm_arduino_bringup_ros2_control_gazebo.launch.py
```
## Terminal 2:
```
cd <your_ws>
colcon build
source install/setup.bash
ros2 run robotarm_gazebo joint_trajectory_publisher.py
```
