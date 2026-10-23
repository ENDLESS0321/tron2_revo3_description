# 机器人资产

[English](README.md) | [简体中文](README.zh-CN.md)

未修改的完整可视化总成使用 assembly.urdf 与 scene.xml。model5200 训练溯源使用 training/ 中的两个 URDF：

- model5200_official_aligned_20261023_icp.source.urdf 用作重定向与资产绑定的源模型。
- model5200_official_aligned_20261023_importer_runtime.urdf 是 Isaac 导入器的精确输入。

两个训练 URDF 均保留原始 SHA-256 身份。相对 mesh 引用在 training/meshes/ 下解析；training/manifest.json 校验全部文件。运行时模型包含 28 个可动关节和 58 个碰撞元素。完整可视化模型保持原样。

在仓库根目录运行以下命令，可验证哈希与闭包：

```bash
python3 -m unittest discover -s tests -v
```导入器覆盖参数记录在训练 manifest 中，它们不是硬件限值。
