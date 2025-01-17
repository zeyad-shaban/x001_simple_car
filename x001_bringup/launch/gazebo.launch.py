from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ros_gz_bridge.actions import RosGzBridge
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # paths
    pkg_bringup = get_package_share_directory("x001_bringup")
    pkg_desc = get_package_share_directory("x001_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    config_file_path = os.path.join(pkg_bringup, "config", "bridge.yaml")
    x001_desc = xacro.process_file(
        os.path.join(pkg_desc, "urdf", "x001.urdf.xacro")
    ).toxml()

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("world_sdf", default_value="empty.sdf"))
    ld.add_action(DeclareLaunchArgument("bridge_name", default_value="ros_gz_bridge"))
    ld.add_action(DeclareLaunchArgument("config_file", default_value=config_file_path))

    launch_nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"use_sim_time": True}, 
                {"robot_description": x001_desc}
            ],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": ["-r ", LaunchConfiguration("world_sdf")]
            }.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "/robot_description", "-name", "x001", "-allow_renaming", "true"],
            output="screen",
        ),
        RosGzBridge(
            bridge_name=LaunchConfiguration("bridge_name"),
            config_file=LaunchConfiguration("config_file"),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ]

    for node in launch_nodes:
        ld.add_action(node)

    return ld
