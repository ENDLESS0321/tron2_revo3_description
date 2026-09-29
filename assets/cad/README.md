# CAD handoff: TRON2 + Revo3 adapters and wrist-camera brackets

English | [简体中文](README.zh-CN.md)

This directory contains the left/right hand adapters and wrist-camera brackets included with the model release. The inventory and hashes are recorded in [`../manifest.json`](../manifest.json); the viewer's selected files, unit scale and display-only rotations are in [`../runtime.json`](../runtime.json). Start with those files when checking that a CAD export matches this release.

![Assembly preview with both hands and wrist cameras](../../docs/images/new_urdf_front_table.png)

## Select a file

| Part | Left | Right | Use |
| --- | --- | --- | --- |
| Hand adapter, V2 | [`adapters/left.3mf`](adapters/left.3mf) or [`adapters/left_print_mm.stl`](adapters/left_print_mm.stl) | [`adapters/right.3mf`](adapters/right.3mf) or [`adapters/right_print_mm.stl`](adapters/right_print_mm.stl) | 3MF is the packaged model file; `_print_mm.stl` is the millimeter STL selected by the parts viewer. |
| Wrist-camera bracket, V3 | [`camera_brackets/left.step`](camera_brackets/left.step) or [`camera_brackets/left_mm.stl`](camera_brackets/left_mm.stl) | [`camera_brackets/right.step`](camera_brackets/right.step) or [`camera_brackets/right_mm.stl`](camera_brackets/right_mm.stl) | STEP is the CAD exchange file; `_mm.stl` is the millimeter STL selected by the parts viewer. Brackets are shown without camera bodies. |

The release runtime identifies the configuration as “purple adapter V2 + camera bracket V3.” The assembly URDF uses the corresponding `adapter_v2` meshes and `supplied_aligned_v3` bracket meshes. These version labels identify the included configurations; they do not certify interchangeability with other revisions or hardware.

## Handedness and units

“Left” and “right” mean the robot's own left and right sides, as viewed from the robot facing forward. They do not mean the viewer's left and right when looking at the robot from the front. Select the matching side file; do not mirror one side to derive the other.

The CAD exports in this directory are millimeter-scale. Import STL as millimeters (or apply scale `0.001` when a meter-based viewer/importer requires it). The simulation meshes are in meters. `runtime.json` applies scale `0.001` to these CAD STLs for display. Check import units and a known dimension before editing or manufacturing; an STL file does not carry a reliable unit declaration.

The parts viewer also sets a display rotation of 180° about Y for `camera_brackets/right_mm.stl`; the left bracket and both adapter display rotations are zero. This is a viewer alignment setting only. It is not a change to the right bracket's CAD coordinates and is not a manufacturing instruction. Keep the supplied CAD geometry intact unless a separately reviewed mechanical change requires an edit. Use the full assembly preview to see the installed arrangement; the preview is kinematic and does not establish physical fit or clearance.

## Installation information confirmed by the simulation model

The full assembly [`../assembly.urdf`](../assembly.urdf) defines fixed robot-relative mounting frames in meters. For each side, the adapter mount is attached to that side's `wrist_roll_*_Link` and the hand base is attached to the adapter. The documented transform from adapter to hand base is translation `(0, 0.023855, 0)` m; the URDF RPY is `(-π/2, -π/2, 0)` on the left and `(-π/2, +π/2, 0)` on the right. The adapter mount origin is `(-0.0317, 0, -0.0812)` m on both sides; its RPY is `(-π/2, 0, -π/2)` left and `(-π/2, 0, +π/2)` right. These are simulation frame transforms, not a complete assembly drawing or a substitute for checking the CAD datum scheme.

The URDF places V3 wrist-bracket and D405 camera frames on each wrist. The CAD directory provides left/right bracket geometry but contains no camera bodies. Refer to the exact URDF and the model README for the complete kinematic chain and preview context:

- [Full assembly and coordinate notes](../../README.md)
- [Chinese full assembly notes](../../README.zh-CN.md)
- [Full assembly kinematic preview](../../docs/images/new_urdf_front_table.png)

## Mechanical handoff status

**Confirmed in this release:** left/right CAD files are present; the manifest records their byte counts and SHA-256 values; the runtime selects the millimeter STL files and records viewer display rotations; the assembly URDF provides named robot-relative mounting frames and simulation transforms. The full model preview shows the intended simulated arrangement.

**Pending before manufacture or hardware installation:** confirm the physical robot/hand/camera revision and interface dimensions; datum definitions and tolerances; fastener size, thread, length, grade, count and torque; material and manufacturing process; print orientation, supports, infill/wall settings and post-processing (if printed); fit, cable routing, tool clearance, load capacity, fatigue and safety review. These specifications are not established by the bundled files. Do not infer or fill in values from the mesh, URDF, filename or viewer settings.

This package is a CAD/model handoff, not a manufacturing drawing, fit guarantee, structural assessment or hardware/manufacturing certification. Verify dimensions and interfaces with the actual components and obtain the responsible mechanical review before fabrication or installation.
