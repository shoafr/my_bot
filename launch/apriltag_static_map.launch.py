import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_bot')
    
    # Path to the config file with AprilTag coordinates
    tag_positions_file = os.path.join(pkg_share, 'config', 'tag_positions.yaml')

    return LaunchDescription([
        Node(
            package='my_bot',
            executable='apriltag_static_broadcaster.py',
            name='apriltag_static_broadcaster',
            parameters=[{'tag_positions_file': tag_positions_file}],
            output='screen'
        ),
    ])
