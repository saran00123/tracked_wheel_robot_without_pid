import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the full path to the parameter file
    package_share_dir = get_package_share_directory('tank_drive_controller')
    params_file = os.path.join(package_share_dir, 'config', 'advanced_param.yaml')

    # Define the node
    tank_drive_node = Node(
        package='tank_drive_controller',
        executable='advanced_tank_drive_node',
        name='tank_drive_controller',
        parameters=[params_file]
    )

    return LaunchDescription([tank_drive_node])
