# 降阶右臂训练总成

本文说明官方右手迁移后的当前降阶 URDF 及其范围。

## 资产与范围

[`assets/assembly_rl_convex.urdf`](../assets/assembly_rl_convex.urdf) 的 SHA-256 为 `623be410f99dce037117fc7b08c888aaa690ae4796e1a5f187c27b120f9ad41f`。该模型包含 28 个可动关节、37 个 link、共 36 个 joint、113 个 visual、91 个 collision 和 196 个唯一 mesh URI。七个右臂关节和 21 个官方右手关节仍可驱动。机器人其余部分保留在已有降阶/冻结拓扑中。本次迁移保留原有右手安装变换，不改变 root 摆放。

## 关节身份与顺序

官方右手关节顺序为拇指 CMP、CMR、MCP、PIP、DIP；之后为食指、中指、无名指和小指，每指均按 MPR、MCP、PIP、DIP 排列。源模型到官方模型按大小写敏感的关节名和索引一一对应。完整 21 项映射及显式顺序见[`assets/official_revo3_migration.json`](../assets/official_revo3_migration.json)。七个手臂关节名称保持不变。运行时控制和轨迹必须绑定关节名及该 URDF 哈希，不可依赖导入器索引或旧向量。

迁移后的子树保留官方关节限位、惯性、视觉、碰撞以及固定几何变换，并添加五个用于 retarget 的无几何固定指尖别名 link。没有虚构质量，也没有更改硬件物理限位。

## 预览与兼容性限制

查看器编译运动学 MuJoCo 预览。降阶模型的旧惯性导入兼容处理仅在内存中应用，不会修改 URDF。成功编译或 FK 比较不代表动力学等价、策略兼容、训练完成、运动无碰撞、抓取成功或硬件就绪。已有轨迹和 checkpoint 需针对新 URDF 重新 retarget 并验证。RL 训练暂缓。

## 坐标摆放

原源模型 root 到物理 base 的变换保持不动。桌面到物理 base 原点的高度差仍精确为 `0.45 m`，见[坐标系约定](../TABLETOP_FRAME_CONTRACT.md)。降阶训练模型具有自己的右手安装上下文，不能将完整总成的安装参数复制过来。
