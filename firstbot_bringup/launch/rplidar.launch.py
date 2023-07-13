import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rplidar_ld = Node(package='rplidar_ros',
                      executable='rplidar_composition',
                      output='screen',
                      parameters=[{
                        'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
                        #'frame_id': 'laser_frame',
                        'frame_id': 'laser',
                        'inverted': False,
                        'angle_compensate': True,
                        'scan_mode': 'Standard'
                      }]
                      )

    rplidar_tf_ld = Node(package='tf2_ros',
                         executable='static_transform_publisher',
                         output='screen',
                         arguments = ['0','0','0.2','0','0','90','1','base_link','laser']
                         )

    return LaunchDescription([
        rplidar_ld,
        rplidar_tf_ld
     ])
