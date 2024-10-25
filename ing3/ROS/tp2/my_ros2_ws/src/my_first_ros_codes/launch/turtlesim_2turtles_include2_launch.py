from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    my_launchfile_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("my_first_ros_codes"),
        "/launch",
        "/turtlesim_2turtles_include1_launch.py"])
    )

    my_turtlesim_node2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="t2"
    )
    my_turtle_teleop_key_node2 = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        prefix=["xterm -e"],
        namespace="t2"
    )
    ld.add_action(my_launchfile_1)
    ld.add_action(my_turtlesim_node2)
    ld.add_action(my_turtle_teleop_key_node2)
    return ld
