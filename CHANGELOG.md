# Changelog

## [Unreleased] - 2026-11-12

### Features

- Added `assets/training/revo3_pace_sysid.json`: per-joint simulation-equivalent dynamics for the 21 Revo3 right-hand joints (kp, kd, armature, dry friction, viscous damping, torque limit), plus the fit's timing assumptions (15 ms command delay, 5 ms PD update, 2.5 ms physics step), from the PACE identification report dated 2026-11-01.
- The file holds a single parameter set: the fit_03 baseline with the 7 fields refit in report section 17 (thumb CMP armature/kp/kd, thumb CMR kp/kd, ring PIP kp/kd) replacing their fit_03 values. The replaced values are kept under `parameters.replaced_from_fit_03`.
- Both training URDFs now carry the URDF-expressible part on the 21 hand joints: `<limit effort>` = torque limit, plus a new `<dynamics damping friction>` = viscous damping and dry friction. Nothing else in the files changed.
- Added `scripts/revo3_pace_profile.py`: `load_profile()` returns the 21-joint parameters; `--isaaclab` emits Isaac Lab `DelayedPDActuatorCfg` field dicts.
- `assets/training/manifest.json` `runtime_only_overrides` now points to the sidecar. The uniform hand values (0.35 Nm effort, PD 3.0/0.1) moved to `superseded_uniform_values`. The 6.28 rad/s velocity override is unchanged.
- Added `tests/test_pace_sysid.py`, which checks joint coverage, the section 17 replacement, URDF/sidecar agreement and manifest hashes.

### Design Rationale

- The old uniform Kp 3.0 / Kd 0.1 were the SDK capture gains. The report states those are not simulation torque-domain gains.
- kp, kd, armature and command delay have no standard URDF field, so they live in the sidecar. The local Isaac Sim URDF importer (2.4.30) reads `<dynamics damping friction>` and `<limit effort>`, so those values are also written into the URDFs.
- The section 17 values were adopted over plain fit_03 because they fix the step-response regression on thumb CMP/CMR and ring PIP (all-21 step RMSE 1.228 -> 1.034 deg).

### Notes & Caveats

- **Training URDF hashes changed.** The new hashes are in `training/manifest.json` and `official_revo3_migration.json`, and the original model5200 hashes are kept as `model5200_original_sha256`. Anything that pinned the old hashes (6e3592a9..., 8e330b11...) must be updated.
- **Breaking for consumers:** code that read `runtime_only_overrides.position_gain`, `velocity_gain` or `actuator_effort_nm` must switch to the sidecar.
- **Trade-off:** thumb 4 Hz tracking is worse than fit_03 (1.605 -> 2.303 deg RMSE).
- **Not validated:** the section 17 refit reused the old step data and has not been validated on new captures.
- **Precision:** values are the report's rounded display values. The full-precision `best_parameters.json` files are not in the repository; their SHA-256 hashes are recorded in the sidecar.
- **Right hand only:** `assembly.urdf` and `scene.xml` (full visual model, both hands) are unchanged.
- **Velocity:** the URDF `velocity="0"` attributes are unchanged because velocity limits were not identified.
- **Isaac Lab:** values set in an actuator config replace what the importer read from the URDF. Keep the PD period at 5 ms (`sim.dt = 0.005`, delay 3 steps).
