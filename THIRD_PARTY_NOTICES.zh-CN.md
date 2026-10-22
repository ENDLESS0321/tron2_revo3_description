# 第三方模型资产声明

本仓库集成不会授予第三方几何资产的额外权利。

- LimX：`limxdynamics/tron2-robot-description`，commit `9939c22e69d27653ec0ba8a505859a2903dd1a71`。Apache-2.0 原始许可、声明和第三方通知保存在 [`licenses/limx`](licenses/limx/)。
- BrainCo：`BrainCoTech/brainco-description`，官方手部源 commit `6db7e2ed1d864ee0a3ab6ff62d43a303787137f6`，此前集成源 commit `f332a6f0dc944e26b82976b637074b03f7ee8a2c`。检查的官方源目录没有 LICENSE 文件；其 README 摘录保存在 [`licenses/brainco/UPSTREAM_README.md`](licenses/brainco/UPSTREAM_README.md)。许可状态尚未解决，再分发前须取得适当授权。
- RealSense：`realsenseai/realsense-ros`，commit `9a11121700cb4780e273e34141f6402fe184321d`。原许可、声明和版权文本保存在 [`licenses/realsense`](licenses/realsense/)。头部相机使用该版本中的 `realsense2_description/meshes/d455.stl`。
- 手部转接件和相机支架来自用户提供的 CAD。最终自定义几何包含改版转接接口和对齐的 V3 相机支架。本次集成不改变这些输入的权属或许可。

网格已为 URDF/MuJoCo 转换或重定位；上游作者未认证本机器人总成。源版本和资产哈希见 `assets/manifest.json` 与 `assets/official_revo3_migration.json`。模型不构成机械或硬件验证。官方手部资产迁移没有授予额外再分发权。
