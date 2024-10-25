import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package="my_robot_description").find("my_robot_description")
    default_model_path = os.path.join(pkg_share, "src/description/my_robot_description.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    # gui_arg = DeclareLaunchArgument(
    #     name="gui",
    #     default_value="True",
    #     description="Flag to enable joint_state_publisher_gui"
    # )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to robot urdf file"
    )

    rvizconfig_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=default_rviz_config_path,
        description="Absolute path to rviz config file"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", LaunchConfiguration("model")])}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        # condition=UnlessCondition(LaunchConfiguration("gui"))
    )

    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    #     condition=IfCondition(LaunchConfiguration("gui"))
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", LaunchConfiguration("rvizconfig")],
    # )

    world= os.path.join(pkg_share, "world/my_world.sdf")

    gazebo_sim = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
    )

    avatar_spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "my_robot", "-topic", "robot_description"],
    )

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        prefix=["xterm -e"],
        respawn=True,
        respawn_delay=3,
        remappings=[
            ("/cmd_vel", "/demo/cmd_vel")
        ]
    )

    # ld.add_action(gui_arg)
    ld.add_action(model_arg)
    ld.add_action(rvizconfig_arg)
    ld.add_action(joint_state_publisher_node)
    # ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(rviz_node)
    ld.add_action(avatar_spawn)
    ld.add_action(gazebo_sim)
    ld.add_action(teleop_node)
    return ld
