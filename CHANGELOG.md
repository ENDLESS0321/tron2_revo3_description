# Changelog

## [1.1.0] - 2026-09-24

### Features

- Replaced the head-mounted RealSense D435i geometry and nominal frames with a
  RealSense D455, including depth, color, infrared, accelerometer, gyroscope,
  and IMU optical frames.
- Updated the simulated D455 color/depth field of view and ideal depth range.
- Restyled the assembly with a graphite TRON2 body, red and cyan accents,
  silver-gray Revo3 hands, and purple hand-adapter flanges.

### Design Rationale

- The D455 transform values and body mesh follow the official RealSense ROS
  description so the physical envelope and nominal sensor origins stay
  traceable to an upstream source.
- Appearance-only accent geometry is collision-free and does not alter robot
  kinematics, limits, or the existing wrist-camera selection.

### Notes & Caveats

- Camera parameters remain nominal simulation values, not per-device
  calibration.
- The D455's larger 124 mm body needs physical cable and motion-clearance
  validation before hardware installation.
