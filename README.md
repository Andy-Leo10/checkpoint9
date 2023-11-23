# Approach motion of RB1 v.Container


https://github.com/Andy-Leo10/checkpoint9/assets/60716487/5cacddb6-f342-41cd-8af5-d74cb7420e5e


## Mandatory
+ Start the simulation in ROS2
```
source ~/sim_ws/install/setup.bash
ros2 launch the_construct_office_gazebo warehouse_rb1.launch.xml
```
+ Start the container
```
ros2 run rclcpp_components component_container
```

## Launch files
- [x] Pre approach
```
ros2 component load /ComponentManager my_components my_components::PreApproach
```
- [x] Pre approach and final approach
```
ros2 launch my_components attach_to_shelf.launch.py
ros2 service list | grep approach_shelf
ros2 component load /my_container my_components my_components::AttachClient
```
- [ ] Move the robot
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```

