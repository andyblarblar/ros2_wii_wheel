import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_wheel = get_package_share_directory('wii_wheel')

    wiimote = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wheel, 'launch', 'wiimote_lifecycle_modified.launch.py'))
    )

    dvelocity = LaunchConfiguration('dvelocity', default=1.0)
    max_turn = LaunchConfiguration("max_turn", default=3)
    steering_ratio = LaunchConfiguration("steering_ratio", default=4)

    wheel_node = Node(
        package='wii_wheel',
        executable='wii_wheel_teleop',
        name='wii_wheel_teleop',
        parameters=[{
            "dvelocity": dvelocity,
            "max_turn": max_turn,
            "steering_ratio": steering_ratio
        }]
    )

    return LaunchDescription([
        wiimote,
        wheel_node
    ])
