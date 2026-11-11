# Calibration procedure

1. Power the platform down, remove props or wheels, and place it on a level
   plate. The plate must not rock: a tilted plate is indistinguishable from
   sensor misalignment.
2. Record ambient temperature. MEMS bias moves several percent per 10 K; a
   calibration taken cold and used hot is worse than none.
3. Bring up the driver, check that `/imu/data_raw` publishes at the real rate
   with `ros2 topic hz`, then start `launch/calibrate.launch.py`.
4. Leave the rig untouched for the whole capture. Watch for logged residual
   before the node starts republishing.
5. Run `tools/report.py` on the produced YAML. Re-run if any check fails.

## Thresholds

| field              | limit            | why                                        |
|--------------------|------------------|--------------------------------------------|
| `gyro_bias_rad_s`  | 0.02             | above 1.1 deg/s the drift dominates a 100 Hz filter |
| `accel_bias_m_s2`  | 0.2              | gravity leak beyond ~2% of g tilts the attitude solve |
| `residual_mps2`    | 0.2              | vibration or motion during the capture     |

## On failure

- Residual above the limit means the platform moved or the mount rings. Retry
  on a damping pad before assuming the sensor is bad.
- A large `accel_misalignment` off-diagonal (over ~0.02) usually means the
  sensor is mounted crooked; re-mount and calibrate again rather than software
  correcting a mechanical fault.
- Persistent gyro bias beyond 0.02 rad/s points at a worn part or a warm-up
  period that is too short.
