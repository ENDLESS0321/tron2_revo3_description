# Third-party model assets

This repository integration grants no additional rights to third-party geometry.

- LimX: `limxdynamics/tron2-robot-description`, commit `9939c22e69d27653ec0ba8a505859a2903dd1a71`. The original Apache-2.0 license, notices and qualifications are retained in [`licenses/limx`](licenses/limx/).
- BrainCo: `BrainCoTech/brainco-description`, official hand source commit `6db7e2ed1d864ee0a3ab6ff62d43a303787137f6`, prior integration commit `f332a6f0dc944e26b82976b637074b03f7ee8a2c`. No LICENSE file was present in the inspected official source directory. Its README excerpt is retained in [`licenses/brainco/UPSTREAM_README.md`](licenses/brainco/UPSTREAM_README.md). License status is unresolved; obtain appropriate permission before redistribution.
- RealSense: `realsenseai/realsense-ros`, commit `9a11121700cb4780e273e34141f6402fe184321d`. The original license, notices and copyright text are retained in [`licenses/realsense`](licenses/realsense/). The head camera uses `realsense2_description/meshes/d455.stl` from that revision.
- Hand adapters and camera brackets derive from user-provided CAD. The custom geometry includes revised adapter interfaces and aligned V3 camera mounts. This integration does not change ownership or licensing of those inputs.

Meshes were converted or rebased for URDF/MuJoCo use; upstream authors do not certify the assembled robot. Source revisions and asset hashes are recorded in `assets/manifest.json` and `assets/official_revo3_migration.json`. The model does not constitute mechanical or hardware validation. Migrating official hand assets does not grant additional redistribution rights.
