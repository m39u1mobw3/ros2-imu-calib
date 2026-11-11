"""Bring up the calibration node against a raw IMU driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    output_path = LaunchConfiguration("output_path")
    return LaunchDescription(
        [
            DeclareLaunchArgument("output_path", default_value="imu_calibration.yaml"),
            DeclareLaunchArgument("sample_target", default_value="2000"),
            Node(
                package="imu_calib",
                executable="calibrate",
                name="imu_calibrate",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("imu_calib"), "config", "imu_calib.yaml"]),
                    {"output_path": output_path},
                ],
                remappings=[("input", "/imu/data_raw"), ("output", "/imu/data")],
                output="screen",
            ),
        ]
    )
