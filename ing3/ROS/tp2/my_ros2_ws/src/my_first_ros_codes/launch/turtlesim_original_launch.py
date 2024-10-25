from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    my_turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    my_turtle_teleop_key_node = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        prefix=["xterm -e"],
        respawn=True,
        respawn_delay=3,
        name="tete"
    )
    ld.add_action(my_turtlesim_node)
    ld.add_action(my_turtle_teleop_key_node)
    return ld
