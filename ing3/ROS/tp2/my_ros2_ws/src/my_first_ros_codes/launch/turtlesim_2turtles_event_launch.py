from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnExecutionComplete

def generate_launch_description():
    ld = LaunchDescription()
    my_turtlesim_node1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="t1",

    )
    my_turtle_teleop_key_node1 = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        prefix=["xterm -e"],
        namespace="t1"
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

    waiting_node_1 = RegisterEventHandler(
        event_handler= OnExecutionComplete(
            target_action=my_turtlesim_node1,
            on_completion=[
                my_turtlesim_node2,
                my_turtle_teleop_key_node2,]
        )
    )


    ld.add_action(waiting_node_1)
    ld.add_action(my_turtlesim_node1)
    ld.add_action(my_turtle_teleop_key_node1)

    return ld
