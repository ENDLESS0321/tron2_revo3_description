# Changelog

## [Unreleased] - 2026-11-12

### Features
- Added `assets/training/revo3_pace_sysid.json`: per-joint simulation-equivalent dynamics for the 21 Revo3 right-hand joints (kp, kd, armature, dry friction, viscous damping, torque limit), plus the fit's timing assumptions (15 ms command delay, 5 ms PD update, 2.5 ms physics step), all from the PACE identification report dated 2026-11-01.
- Two profiles: `fit_03` (default, frozen baseline) and `balanced_20261101_01` (exploratory 7-field overlay on thumb CMP/CMR and ring PIP).
- Added `scripts/revo3_pace_profile.py` to load a profile with the overlay merged, print it, or emit Isaac Lab `DelayedPDActuatorCfg` field dicts.
- `assets/training/manifest.json` `runtime_only_overrides` now points to the sidecar. The uniform hand values (0.35 Nm effort, PD 3.0/0.1) moved to `superseded_uniform_values`. The 6.28 rad/s velocity override is unchanged.
- Added `tests/test_pace_sysid.py`, which checks joint coverage, values, overlay merge and manifest hashes.

### Design Rationale
- The parameters live in a sidecar rather than the URDFs. The training URDFs are SHA-256-pinned as the exact model5200 Isaac importer input, and URDF cannot express PD gains, command delay or armature anyway.
- The old uniform Kp 3.0 / Kd 0.1 were the SDK capture gains. The report states those are not simulation torque-domain gains, so they were not a valid simulation setting.
- `fit_03` is the default because it is the frozen, held-out-validated baseline. The balanced candidate improves step response but worsens thumb 4 Hz tracking and has not been validated on new captures.

### Notes & Caveats
- Values are transcribed from the report tables, which round for display. The full-precision `best_parameters.json` files are not in the repository; their SHA-256 hashes are recorded in the sidecar for a later swap.
- **Breaking for consumers:** code that read `runtime_only_overrides.position_gain`, `velocity_gain` or `actuator_effort_nm` must switch to the sidecar. These keys now live only under `superseded_uniform_values`.
- Right hand only. The left hand in `assembly.urdf` (kinematic preview) has no identified parameters.
- Not identified: velocity limits, current-to-torque constant, behaviour at other SDK gains, static friction/Stribeck, backlash and contact parameters. None of these values are hardware limits.
- PhysX cannot reproduce the fit's 2.5 ms physics substep directly. Keep the PD period at 5 ms (`sim.dt = 0.005`, delay 3 steps); the report shows predictions are sensitive to it.
