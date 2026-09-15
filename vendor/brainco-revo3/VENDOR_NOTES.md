# Revo3 asset provenance and integration notes

Downloaded from the official BrainCo asset repository, discovered through the
[official Revo3 download page](https://www.brainco-hz.com/docs/revolimb-hand/revo3/download.html).
This is Revo3, with 21 revolute joints per hand. The official parameter page describes
21 independent active degrees of freedom (thumb 5, other four fingers 4 each).

- Upstream: https://github.com/BrainCoTech/brainco-description
- Pinned commit: `f332a6f0dc944e26b82976b637074b03f7ee8a2c`
- Retrieved: 2026-09-10
- Original file SHA-256 hashes: `PROVENANCE.json`
- Local modifications: only these provenance/notes files; upstream assets unchanged.

## Entries and mounting frames

- Left: `revo3_system/urdf/revo3_left.urdf`
- Right: `revo3_system/urdf/revo3_right.urdf`
- Both standalone descriptions have a dummy `world` root and a fixed
  `{side}_hand_base_joint` to `{side}_hand_base_link` at identity transform.
- For integration, remove the standalone `world` and its base joint from the
  generated assembly and attach `{side}_hand_base_link` to the chosen adapter frame.
- At joint zero, the long fingers extend along +Z. The left thumb is on -Y;
  the right thumb is on +Y. Positive index MCP flexion moves the fingertip toward +X,
  so +X is the palmar/closing side inferred from the kinematics.
- The root palm mesh reaches z=0 at its lowest surface and extends to z=0.12297 m.
  Treat z=0 as the candidate mounting plane based on geometry, not as a verified
  manufacturer interface calibration. Mesh inspection does not verify bolt-hole
  fit, tolerances, or the user's printed adapter orientation.
- The fixed palm marker is at `[0.0155, 0, 0.055]` m in the base frame.

## Static asset checks

Each hand contains 29 links and 28 joints: 21 revolute and 7 fixed. Its 22 visual
meshes are present. The left has 46 collision elements, the right 49; all referenced
meshes resolve relative to the original URDF directory. Root-palm collisions use
multiple OBJ parts; finger collisions use STL meshes. This check verifies paths
and XML structure, not contact performance or simulation convergence.

The summed upstream masses are 1.0976 kg (left) and 1.105 kg (right), including tiny
fixed fingertip marker masses. The main palm mass is 0.462 kg on each side.
Physical link inertias are positive definite and pass the principal-moment triangle
inequality, except the following helper-frame issue:

- `{side}_palm` has zero mass but nonzero inertia.
- Five fixed `{side}_{finger}_tip_Link` frames per hand have principal moments
  `[1e-10, 1e-10, 1e-9]` kg m^2, violating the triangle inequality.

A downstream generated simulation URDF should remove inertial blocks from purely
fixed marker frames or merge them appropriately. Keep the frames for fingertip
reference poses. Do not modify this vendor snapshot to apply integration fixes.

All revolute limits in this snapshot declare effort 0.35 and velocity 6.28.
Some angle limits differ from the current official parameter page: for example,
thumb CMR upper 2.0071 rad (~115 deg), thumb MCP upper 0.8727 rad (~50 deg), and
four-finger MPR +/-0.2618 rad (~15 deg). The page lists 105 deg, 75 deg, and
finger-specific asymmetric abduction ranges respectively. Preserve the pinned
simulation model values unless the application explicitly chooses and verifies
a different hardware/model revision. These are simulation parameters, not a
validated hardware operating envelope.

## Optional assembly reference

`revotron_system/urdf/` is included only to inspect the official RevoTron connector
transform convention. Its meshes are intentionally not downloaded, so these
assembly URDFs are not standalone runnable artifacts in this sparse checkout.
Do not assume their connector matches the user's printed adapter or DACH_TRON2A.

## License status

No LICENSE/COPYING file was present anywhere in the pinned upstream recursive tree.
The preserved root `README.md` says license information will be provided with the
public release and asks readers to review the accompanying license before
redistribution or modification. This snapshot records that unresolved status;
it does not assign Apache-2.0 or another license based on a different BrainCo
repository. No public redistribution was performed by this download.
