# xr_teleoperate → 智元 G2 遥操作 · 实施文档索引

本目录（`doc/`）记录：以 `xr_teleoperate` 为基础，最终用 **Meta Quest 3** 遥操作 **智元 AgiBot G2 + O10 灵巧手** 的分阶段实施手顺。

---

## 总路线图

| 阶段 | 目标 | 状态 |
|---|---|---|
| Phase 0 | 前置准备：磁盘扩容（DATA 盘）、机器/环境勘察 | ✅ 完成 |
| **Phase 1** | **Quest3 → Unitree G1(29DoF)+Dex3 → Isaac Sim（跑通完整链路）** | ✅ 完成 |
| **Phase 2** | 迁移到 智元 G2 + O10 灵巧手（IK 类 / 手臂控制器 / 手部重定向 / DDS 桥对接 Isaac Sim） | ✅ 完成（仿真全链路） |
| **Phase 3** | **真机遥操作**：下层 DDS 桥换成 teleop↔真机 G2 的 **GDK 直驱**（v3.3.8），上层最大化复用 | 📝 设计完成（未改代码） |

> **为什么先做 Phase 1**：G1+Dex3 是 `xr_teleoperate` + `unitree_sim_isaaclab` 的官方原生组合，几乎零代码改动即可验证「Quest3 → IK → DDS → Isaac Sim → 灵巧手」这条完整链路。链路一旦跑通，Phase 2 换 G2 就只是「替换末端机器人/手模型 + 对接智元 GDK 协议」，风险和工作量都大幅降低。

---

## 文档清单

- 📗 [Phase1_Quest3_to_IsaacSim_G1Dex3.md](./Phase1_Quest3_to_IsaacSim_G1Dex3.md) — **Phase 1 详细手顺（已跑通 ✅）**
- 📘 [Phase2_Quest3_to_G2_O10_Plan.md](./Phase2_Quest3_to_G2_O10_Plan.md) — **Phase 2 规划与工作量评估（决策已定：无真机/仿真 · Isaac Sim · 先 Block1+3）**
- 📘 [Phase2_Block1_3_G2IK_O10Retarget.md](./Phase2_Block1_3_G2IK_O10Retarget.md) — **Phase 2 · Block1+3 执行手顺（G2 双臂 IK + O10 重定向 ✅）**
- 📘 [Phase2_Block2_4_5_G2_IsaacSim_Integration.md](./Phase2_Block2_4_5_G2_IsaacSim_Integration.md) — **Phase 2 · Block2+4+5 G2+O10 仿真集成（DDS 桥，已跑通 ✅）**
- 📙 [Phase3_Quest3_to_G2_RealRobot_GDK.md](./Phase3_Quest3_to_G2_RealRobot_GDK.md) — **Phase 3 · 真机遥操作设计与操作手顺（GDK v3.3.8 直驱，📝 设计完成、未改代码）**

---

## 本机关键事实（勘察结论，写手顺的依据）

- **GPU / 系统**：RTX 4060 8G、驱动 580、Ubuntu 22.04.3、`DISPLAY=:1`
- **Isaac Sim**：5.1.0-rc.19（源码编译）@ `/opt/workspace/IsaacSim`
- **Isaac Lab**：2.3.2 @ `/opt/workspace/IsaacLab`（`_isaac_sim` → IsaacSim build）
- **conda**：交互 shell 默认用 `~/conda`（26.1.1，`.bashrc` 初始化）；另有 `~/miniforge3`（备用）
- **Isaac 环境**：`env_isaaclab`（Python 3.11.14、torch 2.7.0、numpy 1.26.4、isaaclab 0.54.4、dex_retargeting 0.4.6；`conda activate` 后 `isaacsim`/`isaaclab` 均可 import）
- **DATA 盘**：`/home/amit/DATA`（ext4、435G 可用、开机自动挂载）—— 作为仿真工作区
- **teleop 仓库**：`/opt/workspace/xr_teleoperate`（fork：`youkouFR`，`main` 分支，工作区干净）
- **G2 资料**：`/opt/workspace/G2_Robot`（含 O10 灵巧手 URDF/MJCF/mesh + GDK SDK；`app_v2/G2_SDK文档_v3.3.8.md`）
- **GDK 已安装**：`~/.cache/agibot/app/`（v3.3.8，`agibot_gdk` pybind 绑定 Python 3.10/x86_64，与 `tv` 环境一致）；机器人 `10.42.1.101` ↔ 开发机 `10.42.1.102` 网线直连

---

## 约定（Phase 1 用到的路径 / 命名）

| 用途 | 路径 / 值 |
|---|---|
| 仿真工作根目录 | `/home/amit/DATA` |
| teleop 仓库 | `/opt/workspace/xr_teleoperate` |
| teleop 环境（Python 3.10） | `tv` → `/home/amit/DATA/envs/tv` |
| sim 环境 | 复用 `env_isaaclab`（Python 3.11） |
| 公共源码 | `/home/amit/DATA/{cyclonedds, unitree_sdk2_python, unitree_sim_isaaclab}` |
| SSL 证书 | `~/.config/xr_teleoperate/{cert.pem, key.pem}` |
| CycloneDDS | `CYCLONEDDS_HOME=/home/amit/DATA/cyclonedds/install` |

---

## 进度追踪

- [x] Phase 0：DATA 盘格式化 + 开机自动挂载（`/home/amit/DATA`，ext4，435G）
- [x] Phase 1a：teleop 端就绪（环境 + 子模块 + 依赖 + 证书）
- [x] Phase 1b：sim 端就绪（仓库 + 资产 + 依赖，仿真独立拉起 G1+Dex3）
- [x] Phase 1c：两端 DDS 打通
- [x] Phase 1d：Quest3 接入，全链路遥操作 + 灵巧手 ✅
- [x] Phase 2：迁移 G2 + O10 仿真全链路（Block1+3 IK/重定向 + Block2/4/5 DDS 桥集成）✅
- [ ] Phase 3：真机 GDK 直驱（📝 设计手顺已出，待联调执行）
