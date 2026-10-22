# Reduced right-arm training assembly

This note describes the current reduced URDF and its scope after the official right-hand migration.

## Artifact and scope

[`assets/assembly_rl_convex.urdf`](../assets/assembly_rl_convex.urdf) has SHA-256 `623be410f99dce037117fc7b08c888aaa690ae4796e1a5f187c27b120f9ad41f`. It contains 28 moving joints, 37 links, 36 joints total, 113 visual elements, 91 collision elements, and 196 unique mesh URIs. The seven right arm joints and 21 official right-hand joints remain actuated. The rest of the robot is represented in its existing reduced/frozen topology. This migration preserves its incoming right-hand mount and does not alter root placement.

## Joint identity and ordering

The official right-hand joint source order is thumb CMP, CMR, MCP, PIP, DIP; then index, middle, ring, and little, each MPR, MCP, PIP, DIP. The source-to-official mapping is identity by exact case-sensitive joint name and index. The 21-row mapping and explicit order are in [`assets/official_revo3_migration.json`](../assets/official_revo3_migration.json). The seven arm joint names remain unchanged. Runtime consumers must bind controls and trajectories by names and this exact URDF hash, not by assumed importer indexing or old vectors.

The migrated subtree retains official joint limits, inertias, visuals, collisions, and fixed geometry transforms. It adds geometry-free fixed links for the five fingertip frame aliases used by retargeting. It does not invent masses or change physical hardware limits.

## Preview and compatibility limits

The viewer compiles a kinematic MuJoCo preview. The reduced model's legacy inertia import accommodation is applied only in memory; it does not modify the URDF. A successful compile and FK comparison do not establish dynamics equivalence, policy compatibility, training completion, collision-free motion, grasp success, or hardware readiness. Existing trajectories and checkpoints require retargeting/revalidation against the new URDF. RL training is deferred.

## Placement

The original source model root-to-physical-base transform is untouched. The tabletop-to-physical-base origin difference remains exactly `0.45 m`; see [the frame contract](../TABLETOP_FRAME_CONTRACT.md). The reduced training model has its own right-hand mount context; do not copy full-assembly mounts into it.
