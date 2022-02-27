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

    wheel_node = Node(
        package='wii_wheel',
        executable='wii_wheel_teleop',
        name='wii_wheel_teleop'
    )

    # TODO add launch params for velocity ect.
    return LaunchDescription([
        wiimote,
        wheel_node
    ])