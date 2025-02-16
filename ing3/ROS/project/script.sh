. /opt/ros/"${ROS_DISTRO}"/setup.sh
. /home/ros2_ws/install/setup.sh
ros2 launch ros_robot launch_sim.launch.py world:=/home/ros2_ws/myworlds/generated_world.sdf
