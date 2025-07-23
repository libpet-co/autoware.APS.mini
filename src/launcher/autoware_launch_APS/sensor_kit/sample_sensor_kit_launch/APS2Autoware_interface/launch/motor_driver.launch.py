import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()
    params_file = os.path.join(
        get_package_share_directory('motor_pkg'),
        'config', 
        'params.yaml')
    
    motor_node = Node(
        package='motor_pkg',
        executable='motor2auto',
        name='motor2auto',
        output='screen',
        parameters=[params_file],
    )
    
    auto2motor = Node(
        package='motor_pkg',
        executable='auto2motor',
        name='auto2motor',
        output='screen',
    )

    teleop_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    ld.add_action(motor_node)
    ld.add_action(teleop_node)
    ld.add_action(auto2motor)
    return ld
