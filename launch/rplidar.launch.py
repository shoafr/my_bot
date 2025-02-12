import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }]
        ),
        

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='lidar_filter',
            parameters=[[
                os.path.join(get_package_share_directory('my_bot'), 'config', 'lidar_filter.yaml')
            ]],
            remappings=[('scan', 'filtered_scan')]
        )

    ])
