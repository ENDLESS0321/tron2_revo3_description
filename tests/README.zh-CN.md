# 验证

[English](README.md) | [简体中文](README.zh-CN.md)

在仓库根目录运行：

```bash
python3 -m unittest discover -s tests -v
./viewers/view_assembly.sh --model full --check
./viewers/view_assembly.sh --model training --check
```

测试会验证未修改的完整可视化模型溯源、训练源/运行时 URDF 哈希、训练 mesh 与 texture 闭包、28 关节/58 碰撞运行时拓扑、桌面安装元数据和运动学加载。测试不会启动训练，也不认证动力学、碰撞规避、策略性能或硬件运行。
