# Reduced right-arm training assembly

The training model has 28 moving joints and 58 collision elements. It preserves the new official Revo3 kinematics, inertia, visual geometry and mounting transforms, while merging selected same-link convex collision components. This is a geometry approximation, not a return to the old robot model.

## Contents

- [Model identity](#model-identity)
- [Collision allocation](#collision-allocation)
- [Construction and verification](#construction-and-verification)
- [Compatibility and limits](#compatibility-and-limits)

## Model identity

| Asset | Links | Moving joints | Collision elements |
| --- | ---: | ---: | ---: |
| Full `assets/assembly.urdf` | 114 | 58 | 280 |
| Training `assets/assembly_rl_convex.urdf` | 37 | 28 | 58 |

The official source commit is `6db7e2ed1d864ee0a3ab6ff62d43a303787137f6`. Training URDF SHA-256 is `49120fdadec38e67dd81a39f3197858a9a643f950f18b350c6239dee15afb8ea`. The unsimplified migration input had 91 collision elements and SHA-256 `623be410f99dce037117fc7b08c888aaa690ae4796e1a5f187c27b120f9ad41f`. Only the current training asset is packaged; the full model keeps its official collision geometry.

All 28 joint definitions, inertias, visuals, mount transforms and empty fixed tip aliases retain their meaning. The physical base must remain exactly 0.45 m above the tabletop. With tabletop Z=0.507 m, physical base Z=0.957 m and URDF root Z=-0.24335 m; see the [frame contract](../TABLETOP_FRAME_CONTRACT.md).

## Collision allocation

| Group | Official migration input | Current training model |
| --- | ---: | ---: |
| Non-hand robot geometry | 9 | 9 |
| Palm | 5 | 3 |
| Thumb | 41 | 24 |
| Index | 9 | 6 |
| Middle | 9 | 6 |
| Ring | 9 | 5 |
| Little | 9 | 5 |
| Total | 91 | 58 |

Thumb CMR keeps 19 components, reduced from 32. Every finger's DIP collision geometry, including rubber surfaces, is unchanged. Merges use only components attached to the same rigid link. They conservatively enclose the merged source hulls; this can enlarge local geometry. The largest outward displacement at the checked vertices, edge midpoints and triangle centroids is 0.341512 mm. This sampled result is not a continuous Hausdorff bound.

The [geometry report](../assets/training_collision_geometry.json) records each merge, output mesh, source hash and sampled error. Original official collision meshes remain necessary for the full model.

## Construction and verification

1. Materialize the 91-element official migration using [`build_official_revo3_assembly.py`](../scripts/build_official_revo3_assembly.py), preserving the reduced model's own mount.
2. Run the second-stage reducer in an environment with NumPy, SciPy and trimesh. Outputs must be a new directory outside the repository; the input must be the 91-element migration output, not the already simplified default.

```bash
python scripts/build_collision58.py \
  --urdf /absolute/path/to/migration91/assembly.urdf \
  --output /absolute/path/to/new_collision58 \
  --target 58
```

The producer writes a URDF, generated convex meshes, a sampled report and a shared source-mesh symlink. The distributed asset materializes references under `assets/`; it contains no external symlinks. Producer versions are recorded with training evidence because hull triangulation may vary between library versions.

3. Verify the packaged model and mesh hashes from the repository root:

```bash
python -m unittest discover -s tests -v
./viewers/view_assembly.sh --model training --check --output /tmp/revo3-training-check
```

Tests check counts, mesh closure, retained joint structure, unchanged DIP geometry and FK against the full model at nonzero poses. The viewer is a kinematic preview.

## Compatibility and limits

The 1081-sample contact-v5 screwdriver reference H5 remains SHA-256 `08ccf7990ace5dca291895ffc700eee7c8df4baa20e1b150ff3b37ab5f3442c7`. Fresh 270-frame arm IK has zero fallbacks, maximum position error 2.090e-8 m, maximum orientation error 8.149e-7 rad and maximum velocity ratio 0.168706. On that nominal sampled trajectory, cross-finger separation is at least 2.129 mm, robot-table separation at least 3.121 mm and robot-object separation at least 0.143 mm, with zero sampled penetrations in those checks. DIP contact gaps are unchanged. These are exact-asset sampled results; CCD, randomized trajectories, forces and hardware success remain unverified.

Do not reuse a checkpoint solely because tensor dimensions match. Bind the policy, normalizers, URDF, mesh closure, H5, fresh IK and simulator settings. Isaac importers that assign implicit mass to empty fixed tip links require a separately verified runtime derivative which removes only those empty leaves and implements their sensor frames as offsets; collision count stays 58. The packaged logical URDF retains those aliases.
