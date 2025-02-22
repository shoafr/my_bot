import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'my_bot'  # Change this if your package name is different

    # Include launch_robot.launch.py (motors, robot_state_publisher, twist_mux, etc.)
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'launch_robot.launch.py')
        ])
    )

    # Include camera.launch.py (camera node)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'camera.launch.py')
        ])
    )
    

    # AprilTag detection node (from apriltag_ros package)
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('apriltag_ros'), 'cfg', 'tags_36h11.yaml')],
        remappings=[
            ('image_rect', '/camera/image_raw'),  # Adjust if needed
            ('camera_info', '/camera/camera_info')
        ]
    )

    # # Movement node that listens to AprilTag detections
    # move_to_tag = Node(
    #     package=package_name,
    #     executable='move_to_tag.py',
    #     name='move_to_tag',
    #     output='screen'
    # )

    return LaunchDescription([
        launch_robot,
        camera_launch,
        apriltag_node
        # move_to_tag
    ])
