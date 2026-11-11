"""ROS 2 node that captures a still IMU window and republishes corrected data.

Topics:
  ~/input  (sensor_msgs/Imu)  raw sensor stream, remap to your driver
  ~/output (sensor_msgs/Imu)  bias- and misalignment-corrected stream

The node captures ``sample_target`` messages with the platform at rest, writes
the calibration YAML, and starts forwarding corrected samples.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from imu_calib.estimator import Calibration, SampleSet, estimate


class ImuCalibrateNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_calibrate")
        self.declare_parameter("sample_target", 2000)
        self.declare_parameter("output_path", "imu_calibration.yaml")

        self.sample_target = int(self.get_parameter("sample_target").value)
        self.output_path = Path(str(self.get_parameter("output_path").value))

        self._samples = SampleSet()
        self._calibration: Calibration | None = None
        self._started = time.monotonic()

        self.create_subscription(Imu, "input", self._on_imu, qos_profile_sensor_data)
        self._publisher = self.create_publisher(Imu, "output", qos_profile_sensor_data)
        self.get_logger().info(f"capturing {self.sample_target} still samples")

    def _on_imu(self, msg: Imu) -> None:
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        if self._calibration is None:
            self._samples.add(accel, gyro)
            if len(self._samples) < self.sample_target:
                return
            self._finish(msg.header.frame_id)
        self._publisher.publish(self._correct(msg, accel, gyro))

    def _finish(self, frame_id: str) -> None:
        elapsed = max(time.monotonic() - self._started, 1e-3)
        try:
            self._calibration = estimate(self._samples, frame_id, len(self._samples) / elapsed)
        except ValueError as exc:
            self.get_logger().error(f"calibration failed: {exc}")
            return
        self.output_path.write_text(self._calibration.to_yaml(), encoding="utf-8")
        self.get_logger().info(
            f"residual {self._calibration.residual_mps2:.4f} m/s^2, written to {self.output_path}"
        )

    def _correct(self, msg: Imu, accel: np.ndarray, gyro: np.ndarray) -> Imu:
        assert self._calibration is not None
        accel_fixed = self._calibration.accel_misalignment @ (accel - self._calibration.accel_bias)
        gyro_fixed = gyro - self._calibration.gyro_bias

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.linear_acceleration.x = float(accel_fixed[0])
        out.linear_acceleration.y = float(accel_fixed[1])
        out.linear_acceleration.z = float(accel_fixed[2])
        out.angular_velocity.x = float(gyro_fixed[0])
        out.angular_velocity.y = float(gyro_fixed[1])
        out.angular_velocity.z = float(gyro_fixed[2])
        return out


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuCalibrateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
