from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    my_turtlesim_node1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="t1"
    )
    my_turtle_teleop_key_node1 = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        prefix=["xterm -e"],
        namespace="t1"
    )

    ld.add_action(my_turtlesim_node1)
    ld.add_action(my_turtle_teleop_key_node1)
    return ld
