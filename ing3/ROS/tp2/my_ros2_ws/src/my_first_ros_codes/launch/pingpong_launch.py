from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    my_pub_node = Node(
        package="my_first_ros_codes",
        executable="publish_node"
    )
    my_sub_node = Node(
        package="my_first_ros_codes",
        executable="subscribe_node",
        prefix=["xterm -e"]
    )
    ld.add_action(my_pub_node)
    ld.add_action(my_sub_node)
    return ld
