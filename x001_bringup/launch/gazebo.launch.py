from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("use_composition", default_value="True"))
    ld.add_action(DeclareLaunchArgument("world_sdf_file", default_value="empty.sdf"))

    launch_nodes = [
        GzServer(
            use_composition=LaunchConfiguration("use_composition"),
            world_sdf_file=LaunchConfiguration("world_sdf_file"),
        )
    ]

    ld.add_action(*launch_nodes)

    return ld
