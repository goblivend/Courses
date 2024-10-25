from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    my_turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    my_turtle_random_node = Node(
        package="my_first_ros_codes",
        executable="turtle_random_node"
    )
    ld.add_action(my_turtlesim_node)
    ld.add_action(my_turtle_random_node)
    return ld
