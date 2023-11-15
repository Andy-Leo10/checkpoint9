# Approach motion of RB1

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

