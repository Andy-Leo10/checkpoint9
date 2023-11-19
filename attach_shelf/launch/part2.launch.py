'''
this program is to launch the robot pre_approach_v2.cpp script
    obstacle (float)
    degrees (int)
    final_approach (bool)
'''

#essential libraries
from launch import LaunchDescription
from launch_ros.actions import Node
#libraries for parameters
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    obstacle_arg = DeclareLaunchArgument("obstacle", default_value="0.5")
    degrees_arg = DeclareLaunchArgument("degrees", default_value="90")
    final_approach_arg = DeclareLaunchArgument("final_approach", default_value="false")
    obstacle_param = LaunchConfiguration("obstacle")
    degrees_param = LaunchConfiguration("degrees")
    final_approach_param = LaunchConfiguration("final_approach")
    
    final_approach_node = Node(
        package="attach_shelf",
        executable="final_approach",
        output="screen",
        name="pre_approach_v2",
        parameters=[{"obstacle": obstacle_param,
                    "degrees": degrees_param, 
                    "final_approach": final_approach_param}]
    )
    
    #create and return the launch description object
    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        final_approach_node
    ])