# 资产验证

在仓库根目录安装 `requirements.txt` 后运行检查。

## 目录

- [单元测试](#单元测试)
- [查看器检查](#查看器检查)
- [资产维护](#资产维护)

## 单元测试

```bash
.venv/bin/python -m unittest discover -s tests -v
```

测试覆盖 manifest 哈希和 mesh 闭包、完整/降阶模型拓扑与加载、安装变换/相机/世界摆放保留、0.45 m 桌面高度差、官方手部惯性和关节顺序、指尖别名、非零姿态下 URDF 与场景 FK 及完整模型与训练模型 FK 对比，以及篡改 mesh 后 manifest 拒绝通过。测试还检查 58 自由度完整模型的 280 个碰撞元素与 28 自由度训练模型的 58 个碰撞元素，确认官方关节、惯量、视觉与 DIP 碰撞几何保持一致，拒绝非固定来源的碰撞简化器输入，并核验保留的训练非手部元素签名。

## 查看器检查

```bash
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

这些检查仅覆盖运动学预览和渲染，不代表 PhysX 运行时接触、动力学、轨迹净空、策略兼容性、硬件运行或任务成功。这些测试命令不会启动训练。

## 资产维护

重建手部子树时使用[`scripts/build_official_revo3_assembly.py`](../scripts/build_official_revo3_assembly.py)，传入精确源总成及其 mesh 根闭包，并验证生成结果对安装变换和非手部结构的不变性。仅在审查源 commit、原生关节顺序、所有引用 mesh 哈希及测试后更新迁移记录和 manifest。对于 58 碰撞元素训练模型，使用 [`scripts/build_collision58.py`](../scripts/build_collision58.py) 处理固定的 91 碰撞元素迁移输出，并校验所得碰撞报告。源备份和临时构建输出必须放在仓库外。
