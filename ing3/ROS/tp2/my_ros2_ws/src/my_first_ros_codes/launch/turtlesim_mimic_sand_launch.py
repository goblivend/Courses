from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()


    my_turtlesim_node1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="t1",
        remappings=[
            ("/t1/turtle1/pose", "/input/pose")
        ]
    )

    my_mimic_node = Node(
        package="turtlesim",
        executable="mimic",
        remappings= [
            ("/output/cmd_vel", "/t2/turtle1/cmd_vel")
        ]
    )

    my_turtlesim_node2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="t2",
        parameters=[{
            "background_r": 242,
            "background_g": 231,
            "background_b": 191,
        }]
    )

    my_turtle_teleop_key_node1 = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        prefix=["xterm -e"],
        namespace="t1"
    )

    ld.add_action(my_turtlesim_node1)
    ld.add_action(my_turtlesim_node2)
    ld.add_action(my_turtle_teleop_key_node1)
    ld.add_action(my_mimic_node)
    return ld
