# Changelog

Asset release history.

## [1.8.0] - 2026-10-03

### Features

- Align full-scene table size `[0.7, 0.7, 0.4]` m, center `[0.2764393782702718, 0.39253473731213084, 0.307]` m and top `0.507` m with dex-retarget/dex-rl.
- Align scene physical base XYZ/quaternion, retaining the 0.45 m vertical gap; reposition test cubes on the tabletop.

### Design Rationale

- Express shared world placement in scene XML and runtime metadata while preserving the exact full and training robot URDF bytes and installation transforms.

### Notes & Caveats

- Table is external to the robot URDFs. Importers must apply the documented root transform once. The full 58-DOF collision topology remains different from the reduced28 training model. This release verifies kinematic assets and previews, not policy or physical success.
