# Approach motion of RB1 v.Parameters


https://github.com/Andy-Leo10/checkpoint9/assets/60716487/5d5d8d0d-32ed-4309-8ce6-6cc45c4b8769


## Mandatory
+ Start the simulation in ROS1
```
source ~/simulation_ws/devel/setup.bash
roslaunch rb1_base_gazebo warehouse_rb1.launch
```
+ Start the ROS1 bridge
```
source ~/catkin_ws/devel/setup.bash
roslaunch load_params load_params_base.launch
source /opt/ros/galactic/setup.bash
ros2 run ros1_bridge parameter_bridge
```

## Launch files
- [x] Pre approach
```
ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.3 degrees:=-90
```
- [x] Pre approach and final approach
```
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.3 degrees:=-90 final_approach:=true
```
- [x] Move the robot
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
```

