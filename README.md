# imu_calib

A small ROS 2 node for characterising a cheap IMU before it goes into a state
estimator. It captures a still window, estimates gyro bias, accelerometer bias,
per-axis noise density, and the accelerometer-to-gravity misalignment, then
republishes corrected samples.

Why: uncalibrated MEMS gyros drift on the order of degrees per second, and a
tilted accelerometer axis quietly leaks gravity into forward acceleration. Both
wrecks a complementary filter long before it reaches the planner. This tool
produces the numbers once, offline, so the node on the robot stays trivial.

## Install

Requires ROS 2 (tested on Humble) and NumPy.

```bash
cd ~/ros2_ws/src
git clone https://github.com/m39u1mobw3/ros2-imu-calib
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select imu_calib
source install/setup.bash
```

## Usage

Place the platform still on a level surface, bring up the driver, then:

```bash
ros2 launch imu_calib calibrate.launch.py output_path:=$HOME/imu_calibration.yaml
```

The node captures `sample_target` samples (2000 by default, roughly two seconds
at 1 kHz), writes the YAML, and from then on publishes corrected messages.
Check the logged residual before trusting a run: above ~0.2 m/s^2 usually means
the platform was moving or the axis misalignment is large enough to warrant
re-mounting the sensor.

Launch arguments:

| argument        | default                | meaning                          |
|-----------------|------------------------|----------------------------------|
| `output_path`   | `imu_calibration.yaml` | where the calibration is written |
| `sample_target` | `2000`                 | still samples required           |

## Topics and frames

| topic (private) | type            | direction |
|-----------------|-----------------|-----------|
| `~/input`       | `sensor_msgs/Imu` | raw, remapped to `/imu/data_raw` |
| `~/output`      | `sensor_msgs/Imu` | corrected, remapped to `/imu/data` |

`~/output` keeps the input `header.frame_id` and timestamp. The accelerometer
is rotated by the estimated body-to-world rotation before bias removal, so the
published axes are gravity-aligned in the reported frame, not the raw case
axes. Consumers that fuse against a fixed extrinsics transform should apply the
`accel_misalignment` matrix themselves instead of using `~/output`.

Only linear acceleration and angular velocity are republished; the covariance
fields are left zero and the orientation is copied unchanged.

## Layout

```
imu_calib/estimator.py       ROS-free sampling and estimation math
imu_calib/calibrate_node.py  capture, YAML output, corrected republish
launch/calibrate.launch.py   bring-up against a raw driver
config/imu_calib.yaml        default parameters
```

## Limits

- Static capture only; it does not estimate scale factors or temperature drift.
- The misalignment model is a pure rotation, not a full 3x3 correction matrix.
- A still window on a tilted surface biases the tilt estimate; level the rig.

## License

MIT, 2026 Lars Berg.
