from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    ld = LaunchDescription()

    g1 = GroupAction(
        actions=[
            PushRosNamespace("t1"),
            Node(
                package="turtlesim",
                executable="turtlesim_node",
            ),
            Node(
                package="turtlesim",
                executable="turtle_teleop_key",
                prefix=["xterm -e"],
            )
        ]
    )


    g2 = GroupAction(
        actions=[
            PushRosNamespace("t2"),
            Node(
                package="turtlesim",
                executable="turtlesim_node",
            ),
            Node(
                package="turtlesim",
                executable="turtle_teleop_key",
                prefix=["xterm -e"],
            )
        ]
    )

    ld.add_action(g1)
    ld.add_action(g2)
    return ld
