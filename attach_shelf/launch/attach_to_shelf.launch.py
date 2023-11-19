'''
this is a launch file for attaching the shelf to the robot:
-executes the pre_approach_v2.cpp LAUNCH file which has parameters
    obstacle (float)
    degrees (int)
    final_approach (bool)
-executes RVIZ existing LAUNCH file
-executes the approach_service_server.cpp script
'''

#essential libraries
from launch import LaunchDescription
from launch_ros.actions import Node
#essential libraries for including other launch files
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
#libraries for parameters
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
#substitutions
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution
#if want to use namespace
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    package_description="attach_shelf"
    #args for final_approach
    obstacle_arg = DeclareLaunchArgument("obstacle", default_value="0.3")
    degrees_arg = DeclareLaunchArgument("degrees", default_value="-90")
    final_approach_arg = DeclareLaunchArgument("final_approach", default_value="false")
    obstacle_param = LaunchConfiguration("obstacle")
    degrees_param = LaunchConfiguration("degrees")
    final_approach_param = LaunchConfiguration("final_approach")
    #args for rviz
    package_description_arg = DeclareLaunchArgument("attach_shelf", default_value=package_description)
    rviz_config_file_name_arg= DeclareLaunchArgument("rviz_config_file_name", default_value="config1.rviz")
    package_description_param = LaunchConfiguration("attach_shelf")
    rviz_config_file_name_param = LaunchConfiguration("rviz_config_file_name")
    
    #rviz
    start_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_description),
                'launch',
                'start_rviz_with_arguments.launch.py'
            ])
        ]),
        launch_arguments={
            'package_description': package_description_param,
            'rviz_config_file_name': rviz_config_file_name_param
            }.items()
    )
    
    #final_approach
    start_final_approach_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_description),
                'launch',
                'part2.launch.py'
            ])
        ]),
        launch_arguments={
            'obstacle': obstacle_param,
            'degrees': degrees_param,
            'final_approach': final_approach_param
            }.items()
    )
    
    #service_server
    start_service_server = Node(
        package="attach_shelf",
        executable="approach_service_server",
        output="screen",
        name="approach_service_server_node"
    )
    
    # create and return the launch description object
    return LaunchDescription([
            obstacle_arg,
            degrees_arg,
            final_approach_arg,
            package_description_arg,
            rviz_config_file_name_arg,
            start_rviz_launch,
            start_final_approach_launch,
            start_service_server
    ])