import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_name = 'vf_back_params.yaml'

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('olei_driver'),
        'config',
        config_name
        )

    node = Node(
        package='olei_driver',
        name='ros_main_back',
        executable='ros_main',
        output='screen',
        parameters=[config]
    )

    # node1 = Node(
    #     package='olei_driver',
    #     name='scan_filter',
    #     executable='scan_filter',
    #     output='screen',
    #     parameters=[config]
    # )

    node2 = Node(
        package='olei_driver',
        name='scan_b2pointcloud',
        executable='scan_b2pointcloud',
        output='screen',
        parameters=[config]
    )


    ld.add_action(node)
    # ld.add_action(node1)
    ld.add_action(node2)
    return ld
