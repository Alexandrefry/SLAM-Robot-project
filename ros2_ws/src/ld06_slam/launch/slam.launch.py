import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("ld06_slam")
    default_slam_params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")

    return LaunchDescription([
        # Static TF: base_link -> laser_frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_laser_tf",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "laser_frame"],
        ),
        # Static TF: odom -> base_link (stand-in until wheel odometry / EKF is added)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_base_link_tf",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        ),
        # LD06 LiDAR Driver
        Node(
            package="ld06_slam",
            executable="ld06_node",
            name="ld06_node",
            output="screen",
            parameters=[{
                "port": "/dev/ttyUSB0",
                "baudrate": 230400,
                "frame_id": "laser_frame",
            }],
        ),
        # SLAM Toolbox Node
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[default_slam_params],
        ),
        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])
