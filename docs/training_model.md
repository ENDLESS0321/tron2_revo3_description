# Model5200 training robot

[English](training_model.md) | [简体中文](training_model.zh-CN.md)

The repository packages source and Isaac runtime robot URDFs for a fresh run using the original model_5200 training framework. The new 6e/8e URDF asset set is incompatible with the historical model_5200.pt checkpoint, which was bound to the earlier 2776/f898 assets. Both new files and the complete relative mesh/texture closure are hash-verified in [assets/training/manifest.json](../assets/training/manifest.json).

## Contents

- [Select the URDF](#select-the-urdf)
- [Mount and simulation values](#mount-and-simulation-values)
- [Hand dynamics from PACE identification](#hand-dynamics-from-pace-identification)
- [Verify](#verify)

## Select the URDF

| File | Purpose | SHA-256 |
| --- | --- | --- |
| assets/training/model5200_official_aligned_20261023_icp.source.urdf | Retargeting/source binding | 68301474686571650d6614be7d30109e6d086f1a6497978605d3f105379667a8 |
| assets/training/model5200_official_aligned_20261023_importer_runtime.urdf | Isaac runtime import and preview | 0e379bce7fdbe731f3447bdbc1c4a119d9a6ea71d79a27c46532404ef7976a7c |

The source has 37 links and 28 moving joints. The runtime has 32 bodies including world and 28 moving joints; it removes five empty fixed tip links while preserving their virtual sensor frames in [runtime_derivation.json](../assets/training/runtime_derivation.json). Both contain 58 collisions: 28 on the palm, 21 on fingers, and 9 on the arm, base, and adapter. Mesh bytes, scales, and per-link collision counts follow the model5200 training asset. The model retains the existing palm/thumb structural overlap; 58 collision elements are a geometry count and do not claim robot-wide geometric or trajectory collision freedom.

## Mount and simulation values

The tabletop top is 0.507 m, the physical base origin is 0.957 m, and the URDF root is -0.24335 m in the authored world frame. The physical-base to tabletop gap is exactly 0.45 m. The complete model and its mesh closure are unchanged.

The hand importer keeps the 6.28 rad/s velocity override. Per-joint hand gains, torque limits, armature and friction now come from the PACE identification below; the earlier uniform 0.35 Nm effort and PD gains 3/0.1 are kept in the manifest only as superseded values. The 1 kg world importer mass is separate from the 1.105000027 kg total mass of the 22 Revo3 hand bodies before startup DR. These values do not establish hardware limits.

## Hand dynamics from PACE identification

[assets/training/revo3_pace_sysid.json](../assets/training/revo3_pace_sysid.json) holds simulation-equivalent dynamics for the 21 right-hand joints, transcribed from the PACE identification report dated 2026-11-01:

| Field | Unit | Meaning |
| --- | --- | --- |
| kp, kd | N·m/rad, N·m·s/rad | Torque-domain PD gains of the simulated controller. These are not SDK gains: the capture ran with SDK Kp 3.0 / Kd 0.1. |
| armature | kg·m² | Added joint-space inertia |
| friction | N·m | Dry friction torque |
| viscous_damping | N·m·s/rad | Passive viscous loss, separate from kd |
| effort_limit | N·m | Fitted torque clip, not a hardware rating |

The fit assumed a 15 ms shared command delay, a 5 ms PD update, a 2.5 ms physics step, asset link masses, a fixed palm and self-collision on. Keep the 5 ms PD period when reusing the values; the report shows the predictions are sensitive to it.

The values are one parameter set: the fit_03 baseline, with the 7 fields that report section 17 refit (candidate balanced_20261101_01) replacing their fit_03 values. The report tables round for display; the full-precision best_parameters.json files are not in this repository and are identified by SHA-256 in the sidecar. Velocity limits, the current-to-torque constant, other SDK gain settings and contact parameters were not identified.

### Load the parameters

```python
from revo3_pace_profile import load_profile
joints = load_profile()   # fit_03 with the 7 section-17 fields replaced
```

`load_profile()` returns `{joint_name: {kp, kd, armature, friction, viscous_damping, effort_limit}}` for all 21 hand joints. There is no profile argument.

### What changed relative to fit_03

Only these 7 values differ from fit_03; the other 18 joints and every friction, damping and torque-limit value are fit_03 values. The sidecar keeps the replaced values under `parameters.replaced_from_fit_03`.

| Joint | Field | fit_03 (replaced) | Current value |
| --- | --- | ---: | ---: |
| thumb CMP | armature (kg·m²) | 0.00532 | 0.004364 |
| thumb CMP | kp (N·m/rad) | 7.151 | 23.543235 |
| thumb CMP | kd (N·m·s/rad) | 0.339 | 0.698884 |
| thumb CMR | kp (N·m/rad) | 0.785 | 1.10393 |
| thumb CMR | kd (N·m·s/rad) | 0.033 | 0.042304 |
| ring PIP | kp (N·m/rad) | 2.29 | 4.652494 |
| ring PIP | kd (N·m·s/rad) | 0.134 | 0.120927 |

fit_03 made the step response of these three joints worse than the initial model. The section 17 refit fixes that at the cost of thumb periodic tracking:

| Replay of the recorded commands (RMSE, °) | fit_03 | Current |
| --- | ---: | ---: |
| All 21 joints, step stages | 1.228 | 1.034 |
| Five fingers, 4 Hz stages | 1.164 | 1.352 |
| Thumb, 4 Hz | 1.605 | 2.303 |
| Ring finger, 4 Hz | 1.241 | 1.086 |

The section 17 refit reused the old step data and has not been validated on new captures.

### Values written into the URDFs

Both training URDFs carry the URDF-expressible part of these values on the 21 hand joints: `<limit effort>` is effort_limit, `<dynamics damping>` is viscous_damping and `<dynamics friction>` is dry friction. Joint ranges, `velocity="0"`, origins, inertials and meshes are unchanged; the manifest keeps the original model5200 SHA-256 of each file. URDF has no field for kp, kd, armature or the command delay, so those come only from the sidecar. If an Isaac Lab actuator config also sets friction or viscous_friction, its values replace what the importer read from the URDF.

From the command line:

```bash
python3 scripts/revo3_pace_profile.py              # table of the 21 hand joints
python3 scripts/revo3_pace_profile.py --isaaclab   # DelayedPDActuatorCfg field dicts
```

In Isaac Lab, map kp/kd/effort_limit/armature/friction/viscous_damping to the `stiffness`/`damping`/`effort_limit`/`armature`/`friction`/`viscous_friction` fields of `DelayedPDActuatorCfg`. With `sim.dt = 0.005` the 15 ms delay is `min_delay = max_delay = 3` steps. The fit's 2.5 ms physics substep has no direct PhysX equivalent.

## Verify

From the repository root, run:

```bash
python3 -m unittest discover -s tests -v
./viewers/view_assembly.sh --model training --check
```

The training viewer sets joint positions directly and calls MuJoCo mj_forward for forward kinematics; it does not step dynamics. These checks verify packaged bytes, relative URI closure, topology, and kinematic preview. They do not verify training replay, policy performance, dynamics, or hardware behavior.
