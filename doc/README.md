# xr_teleoperate → 智元 G2 遥操作 · 实施文档索引

本目录（`doc/`）记录：以 `xr_teleoperate` 为基础，最终用 **Meta Quest 3** 遥操作 **智元 AgiBot G2 + O10 灵巧手** 的分阶段实施手顺。

---

## 总路线图

| 阶段 | 目标 | 状态 |
|---|---|---|
| Phase 0 | 前置准备：磁盘扩容（DATA 盘）、机器/环境勘察 | ✅ 完成 |
| **Phase 1** | **Quest3 → Unitree G1(29DoF)+Dex3 → Isaac Sim（跑通完整链路）** | 🔵 进行中 |
| Phase 2 | 迁移到 智元 G2 + O10 灵巧手（换 IK 类 / 手臂控制器 / 手部重定向 / 对接 GDK 协议） | ⬜ 未开始 |

> **为什么先做 Phase 1**：G1+Dex3 是 `xr_teleoperate` + `unitree_sim_isaaclab` 的官方原生组合，几乎零代码改动即可验证「Quest3 → IK → DDS → Isaac Sim → 灵巧手」这条完整链路。链路一旦跑通，Phase 2 换 G2 就只是「替换末端机器人/手模型 + 对接智元 GDK 协议」，风险和工作量都大幅降低。

---

## 文档清单

- 📘 [Phase1_Quest3_to_IsaacSim_G1Dex3.md](./Phase1_Quest3_to_IsaacSim_G1Dex3.md) — **Phase 1 详细手顺（当前执行这份）**

---

## 本机关键事实（勘察结论，写手顺的依据）

- **GPU / 系统**：RTX 4060 8G、驱动 580、Ubuntu 22.04.3、`DISPLAY=:1`
- **Isaac Sim**：5.1.0-rc.19（源码编译）@ `/opt/workspace/IsaacSim`
- **Isaac Lab**：2.3.2 @ `/opt/workspace/IsaacLab`（`_isaac_sim` → IsaacSim build）
- **conda**：交互 shell 默认用 `~/conda`（26.1.1，`.bashrc` 初始化）；另有 `~/miniforge3`（备用）
- **Isaac 环境**：`env_isaaclab`（Python 3.11.14、torch 2.7.0、numpy 1.26.4、isaaclab 0.54.4、dex_retargeting 0.4.6；`conda activate` 后 `isaacsim`/`isaaclab` 均可 import）
- **DATA 盘**：`/home/amit/DATA`（ext4、435G 可用、开机自动挂载）—— 作为仿真工作区
- **teleop 仓库**：`/opt/workspace/xr_teleoperate`（fork：`youkouFR`，`main` 分支，工作区干净）
- **G2 资料**：`/opt/workspace/G2_Robot`（含 O10 灵巧手 URDF/MJCF/mesh + GDK v2.6.3 SDK）

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
- [ ] Phase 1a：teleop 端就绪（环境 + 子模块 + 依赖 + 证书）
- [ ] Phase 1b：sim 端就绪（仓库 + 资产 + 依赖，仿真独立拉起 G1+Dex3）
- [ ] Phase 1c：两端 DDS 打通
- [ ] Phase 1d：Quest3 接入，全链路遥操作 + 灵巧手
- [ ] Phase 2：迁移 G2 + O10
