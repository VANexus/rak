# IMU (LSM6DSRTR, I2C)

> 状态：现行 · 最后更新：2026-11-16

`rak_imu_driver` owns the on-board IMU. It publishes raw inertial data only; any
orientation estimate belongs to the fusion stage (EKF), not to this node.

## Hardware and bus

| Item | Value |
|---|---|
| Chip | ST LSM6DSRTR (accel + gyro, 6-DoF) |
| Bus | I2C, `/dev/i2c-7`, address `0x6B` |
| WHO_AM_I | `0x6b` (must match, otherwise the node refuses to start) |
| SPI | `/dev/spidev0.0` reads all `0x00` on this board — **not usable**, do not switch back |
| Frame | `imu_link` (child of `base_link`, see below) |

```bash
ros2 run rak_imu_driver i2c_imu        # streaming node
ros2 run rak_imu_driver i2c_imu --probe # one-shot self-check, then exit
```

`--probe` performs a burst-read consistency check. A mismatch there is a
one-shot diagnostic item only: it does not affect streaming, which is delivered
frame by frame over the same bus.

## Topic contract

`/imu/data_raw` (`sensor_msgs/msg/Imu`), `frame_id: imu_link`.

- `linear_acceleration` — m/s², includes gravity.
- `angular_velocity` — rad/s.
- `orientation` is published as `(0, 0, 0, 1)` with `orientation_covariance[0] = -1`.
  Per REP-145 that means "this node does not estimate orientation" — consumers
  must not treat it as a valid attitude.
- Rate is nominal 1 kHz; the observed delivery rate on the Jetson varies with
  CPU/DDS load (measured 90–800 Hz depending on what else is running). Do not
  derive timing from the message rate.

## Coordinate convention

REP-103: **x forward, y left, z up**, right-handed, origin at `imu_link`.

The module is mounted **vertically** (chip +Y pointing up), so the driver remaps
the chip axes with `axis_map='-z,-x,y'` before publishing. Consequently:

- `imu_link` has `rpy="0 0 0"` relative to `base_link` (see
  `rak_chassis.urdf.xacro`, comment dated 2026-10-29 — the old SPI-era
  `pitch=-90°` was removed on purpose; keeping it would rotate roll into yaw).
- Therefore the ~1° tilt seen on x/y **at rest is not a frame-definition
  problem**. It is the physical mounting/leveling error of the module (plus the
  accelerometer's own bias). Do not "fix" it in TF; either accept it (the EKF
  estimates roll/pitch from gravity anyway) or remove it as an accelerometer
  bias, per the calibration below.

## Static bias calibration (method — run when the robot is standing still)

Gyro bias dominates dead-reckoning drift; accel bias is what produces the
apparent 1° tilt. Calibrate on level ground, motors disabled, nothing touching
the robot.

1. Let the board warm up ~5 min (LSM6DSR bias drifts with temperature; a cold
   calibration is invalid).
2. Record 60 s of stationary data:
   ```bash
   ros2 bag record -o /tmp/imu_static /imu/data_raw   # stop after 60 s
   ```
3. Compute per-axis means (any tool; example with a small Python script reading
   the bag, or `ros2 topic echo` piped to a file):
   - `gyro_bias = mean(angular_velocity)` — expect |mean| < 0.01 rad/s.
   - `accel_mean = mean(linear_acceleration)`.
   - `accel_bias = accel_mean - R * (0, 0, +9.80665)`, where `R` is identity if
     you accept the current mounting. The residual direction of `accel_mean`
     gives the true tilt: `roll ≈ atan2(-a_y, a_z)`,
     `pitch ≈ atan2(a_x, sqrt(a_y² + a_z²))`.
4. Apply the bias either as driver parameters or by subtracting it downstream,
   then re-record and verify:
   - `|mean(angular_velocity)| < 0.005 rad/s` per axis,
   - `mean(linear_acceleration) ≈ (0, 0, 9.80665)` within ±0.05 m/s²,
   - noise (std) unchanged — calibration must not add filtering.
5. If the residual tilt is larger than the mechanical tolerance you care about,
   fix the bracket. Only if the module is *electrically* re-oriented should the
   `axis_map` change; only if the mount is *intentionally* rotated should
   `imu_link` get a non-zero `rpy`.

Re-run after any remount, and after a firmware/driver change that touches
`axis_map`.

## Quick health check

```bash
ros2 topic hz /imu/data_raw                     # non-zero, roughly 100–1000 Hz
ros2 topic echo /imu/data_raw --once             # frame_id: imu_link
# at rest: |accel| ≈ 9.8 m/s², accel.z ≈ +9.8, gyro ≈ 0
```
