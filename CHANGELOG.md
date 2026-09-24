# Changelog

## [1.2.1] - 2026-09-24

### Features

- Moved both wrist-camera assemblies to the opposite radial side of their arm
  axes, completing the requested 180-degree orbit around each wrist roll.

### Design Rationale

- A rigid orbit requires rotating both the camera orientation and its radial
  position vector; changing orientation alone only spins the assembly in place.

### Notes & Caveats

- The wrist-camera root offsets now use `z=+0.0753 m` instead of
  `z=-0.0753 m`; camera calibration and hardware clearance must be revalidated.

## [1.2.0] - 2026-09-24

### Features

- Rotated both Revo3 hands by 180 degrees around their mounting axes.
- Rotated both wrist-camera assemblies by 180 degrees around the wrist-roll
  axes while preserving left/right mirror symmetry.

### Design Rationale

- The rotations are applied only at each assembly's fixed root joint, keeping
  hand articulation, camera optical frames, and all actuated limits unchanged.
- The centered head-mounted D455 remains in its forward-facing orientation
  because it is not part of the left/right wrist pair.

### Notes & Caveats

- Any external calibration that depends on the old hand or wrist-camera poses
  must be regenerated.
- Physical cable routing and self-collision clearance should be revalidated on
  hardware after the 180-degree wrist-camera flip.

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
