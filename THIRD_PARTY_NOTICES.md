# Third-party model assets

This integration does not grant additional rights to third-party geometry.

- LimX: `limxdynamics/tron2-robot-description`, commit
  `9939c22e69d27653ec0ba8a505859a2903dd1a71`. The original Apache-2.0 license,
  notice, asset qualifications and third-party notices are retained in
  [licenses/limx](licenses/limx/).
- BrainCo: `BrainCoTech/brainco-description`, commit
  `f332a6f0dc944e26b82976b637074b03f7ee8a2c`. No LICENSE file was present in
  the inspected tree. Its licensing statement is preserved in
  [the upstream README](licenses/brainco/UPSTREAM_README.md). License status
  remains unresolved; obtain appropriate permission before redistribution.
- RealSense: `realsenseai/realsense-ros`, commit
  `9a11121700cb4780e273e34141f6402fe184321d`. The original license, notice
  and copyright text are retained in [licenses/realsense](licenses/realsense/).
- Hand adapters and camera brackets derive from user-provided CAD. Final
  custom geometry includes revised adapter interfaces and aligned V3 camera
  mounts. Ownership/permission for those inputs is not changed here.

Meshes were converted/rebased for URDF and MuJoCo use; upstream authors do not
certify this assembled model. Asset digests and source versions are recorded in
`assets/manifest.json`. Original development assets and local vendor metadata
are preserved on `dev`. Both branches and their shared history are published at
[clearlab-sustech/tron2_revo3_description](https://github.com/clearlab-sustech/tron2_revo3_description).
Publication does not resolve the license qualifications stated above.
