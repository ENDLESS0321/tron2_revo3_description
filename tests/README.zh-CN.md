# 资产验证

在仓库根目录安装 `requirements.txt` 后运行检查。

## 单元测试

```bash
.venv/bin/python -m unittest discover -s tests -v
```

测试覆盖 manifest 哈希和 mesh 闭包、完整/降阶模型拓扑与加载、安装变换/相机/世界摆放保留、0.45 m 桌面高度差、官方手部惯性和关节顺序、指尖别名、非零手部姿态下 URDF 与场景 FK 对比，以及篡改 mesh 后 manifest 拒绝通过。

## 查看器检查

```bash
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

这些检查仅覆盖运动学预览和渲染，不代表动力学、轨迹净空、策略兼容性、硬件运行或任务成功。训练必须另行遵循已批准的方案；本资产任务不会启动训练。

## 资产维护

重建手部子树时使用[`scripts/build_official_revo3_assembly.py`](../scripts/build_official_revo3_assembly.py)，传入精确源总成及其 mesh 根闭包，并验证生成结果对安装变换和非手部结构的不变性。仅在审查源 commit、原生关节顺序、所有引用 mesh 哈希及测试后更新迁移记录和 manifest。源备份和临时构建输出必须放在仓库外。
