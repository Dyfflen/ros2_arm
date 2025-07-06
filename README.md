# ros2_arm
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
