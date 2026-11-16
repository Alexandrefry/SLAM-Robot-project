from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static TF: base_link -> laser_frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_laser_tf",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "laser_frame"],
        ),
        # LD06 LiDAR Driver Node
        Node(
            package="ld06_slam",
            executable="ld06_node",
            name="ld06_node",
            output="screen",
            parameters=[{
                "port": "/dev/ttyUSB0",
                "baudrate": 230400,
                "frame_id": "laser_frame",
                "range_min": 0.05,
                "range_max": 12.0,
                "min_confidence": 100,
            }],
        ),
    ])
