# Verification

[English](README.md) | [简体中文](README.zh-CN.md)

From the repository root, run:

```bash
python3 -m unittest discover -s tests -v
./viewers/view_assembly.sh --model full --check
./viewers/view_assembly.sh --model training --check
```

The suite verifies the unchanged full visual asset lineage, the training source/runtime URDF hashes, training mesh and texture closure, 28-joint/58-collision runtime topology, tabletop mounting metadata, and kinematic loading. It does not launch training or certify dynamics, collision avoidance, policy performance, or hardware operation.
