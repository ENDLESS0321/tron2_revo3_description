# Changelog

## [1.5.0] - 2026-09-28

### Features

- Add `assets/assembly_rl_convex.urdf`, derived from the exact approved assembly,
  with one closed convex collision mesh per flange instead of 48 shapes.
- Include left/right hull meshes, a reproducible builder, collision provenance
  and structural/import checks with paired documentation.

### Design Rationale

- Reduce flange collision-shape count for RL and simulation while preserving
  visual geometry, inertials, all joints and mounting transforms.
- Retain the approved model as the canonical detailed assembly.

### Notes & Caveats

- Convex envelopes fill holes/concavities and may reduce collision clearance.
- MuJoCo import is verified; other simulators need their own shape-count check.
- The default MJCF uses the detailed geometry. No training-speed or task-success
  benchmark is claimed.

## [1.4.1] - 2026-09-28

### Features

- Replace `assets/assembly.urdf` byte-for-byte with the approved bilateral
  axis-180 artifact and remove the redundant variant URDF.
- Expand paired documentation with before/after joint transforms, rotation
  convention, joint-limit headroom motivation and model migration details.
- Document the RL training requirement of one rigid link and one collision
  shape per flange; the detailed assembly remains unchanged.

### Design Rationale

- Make the approved model the single default for all existing viewers and imports.
- Preserve its world base height at 1.20035 m; synchronize the MJCF base and move
  the table top to 0.75035 m to retain the requested 0.45 m separation.

### Notes & Caveats

- Users of the removed variant should load `assets/assembly.urdf`.
- Geometry is the same as the approved artifact; this patch adds no new workspace
  or quantitative joint-limit-margin result.

## [1.4.0] - 2026-09-28

### Features

- Rotate both hand/flange and wrist-camera root orientations 180 degrees around
  each forearm longitudinal centerline; synchronize the default URDF and MJCF.
- Include the exact approved URDF, rotation provenance and paired English/Chinese
  documentation with zero-pose and forward-table previews.

### Design Rationale

- Seek greater joint-limit headroom for palm-down, fingers-forward tabletop
  operation while preserving hand-to-flange connections and physical limits.
- Keep the default base placement and its 45 cm table-top separation consistent
  with 1.3.1; retain the exact generated URDF separately for provenance.

### Notes & Caveats

- The exact variant inherits a 0.35 mm higher world placement than the default.
- Larger joint-limit margin and feasible workspace have not been quantitatively
  evaluated for this installation. Recompute IK and dependent calibration.
- Previews show kinematics, without physics stepping.

## [1.3.1] - 2026-09-26

### Features

- Set the work-table top to `base_Link` origin vertical separation to exactly
  0.45 m in the URDF and MuJoCo scene.

### Design Rationale

- Align the two robot placements at world `z=1.20 m` while keeping the table
  top at `z=0.75 m` and preserving all internal robot geometry.

### Notes & Caveats

- The URDF contains no table geometry; the table is defined in `scene.xml`.
- Any trajectories using the previous `z=1.20035 m` placement need this
  0.35 mm world-frame offset accounted for.

## [1.3.0] - 2026-09-24

### Features

- Rotated both wrist-camera brackets by 180 degrees around the finger-pointing
  axis while retaining their original mounting-hole center positions.

### Design Rationale

- The finger-pointing axis maps to the wrist-roll frame's Z axis. Rotating only
  the bracket attitude around its fixed root keeps the bracket and arm screw
  centers coincident while flipping the bracket to the opposite orientation.

### Notes & Caveats

- Wrist-camera extrinsics change even though the bracket root positions do
  not; camera calibration and physical fastener clearance require validation.

## [1.2.3] - 2026-09-24

### Features

- Restored both wrist-camera assemblies to their complete original root
  positions and orientations for baseline evaluation.

### Design Rationale

- Returning the camera transforms to a known baseline lets subsequent mounting
  decisions be evaluated independently from the retained hand rotation.

### Notes & Caveats

- The left and right Revo3 hands remain rotated by 180 degrees; only the wrist
  cameras return to their pre-1.2.0 transforms.

## [1.2.2] - 2026-09-24

### Features

- Restored both wrist-camera orientations to their original wrist-roll
  attitudes while keeping their mounting positions on the arm-axis opposite
  side.

### Design Rationale

- Camera attitude and orbital position are independent requirements: each
  camera keeps its original attitude at the position reached by rotating its
  radial offset 180 degrees around the mechanical-arm axis.

### Notes & Caveats

- Wrist-camera extrinsics combine the original orientation with the new
  `z=+0.0753 m` root position and require fresh calibration.

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
