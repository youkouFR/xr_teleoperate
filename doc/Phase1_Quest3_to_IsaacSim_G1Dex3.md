# Phase 1 手顺 · Quest3 → Unitree G1(29DoF)+Dex3 → Isaac Sim

> 目标：在**本机**同时跑起「仿真端」和「遥操作端」，用 **Meta Quest 3** 通过浏览器遥操作 **Isaac Sim 里的 G1 机器人 + Dex3 灵巧手**。
> 本阶段**几乎不改代码**，只按官方方式装配两个开源仓库，验证完整链路，为 Phase 2 迁移 G2 打基础。

---

## 0. 架构与数据流（先看懂再动手）

```
┌─────────────┐   WebXR(HTTPS/WSS:8012)   ┌──────────────────────────────┐
│  Quest 3    │ ────────────────────────► │  teleop 端 (tv 环境, py3.10)  │
│  头显浏览器  │   头/手腕位姿 + 手骨架      │  xr_teleoperate               │
└─────────────┘                           │  teleop_hand_and_arm.py       │
                                          │   ├─ televuer  采集 XR 数据    │
                                          │   ├─ robot_arm_ik  CasADi IK   │
                                          │   └─ hand_retargeting 手指重定向│
                                          └───────────────┬──────────────┘
                                                          │ DDS (domain 1)
                                          ┌───────────────▼──────────────┐
                                          │  sim 端 (env_isaaclab, py3.11) │
                                          │  unitree_sim_isaaclab          │
                                          │  sim_main.py → Isaac Sim       │
                                          │   G1(29DoF) + Dex3 + 相机      │
                                          │   image_server(ZMQ) 回传图像   │
                                          └───────────────────────────────┘
```

- 两端是**两个独立进程 / 两个 conda 环境**，靠 **DDS 话题**通信（和真机完全相同的话题）。
- 图像：sim 的 `image_server`(ZMQ) → teleop 的 `image_client` → 经 Vuer 推给 Quest3。
- 本机同时跑两端，DDS 走本地回环即可。

**里程碑总览**

| 里程碑 | 内容 | 验证标志 |
|---|---|---|
| Step 1 | 公共依赖（apt / cyclonedds / unitree_sdk2py） | cyclonedds 编译产物存在 |
| Step 2 (1a) | teleop 环境 + 子模块 + 证书 | 依赖 import 全绿；能看到 Vuer URL |
| Step 3 (1b) | sim 仓库 + 资产 + 依赖 | Isaac Sim 窗口里出现 G1+Dex3 |
| Step 4 (1c) | 两端 DDS 联调 | teleop 终端收到 lowstate；sim 手臂随指令动 |
| Step 5 (1d) | Quest3 接入 | 头显里看到机器人第一视角，按 r 后手臂/手跟随 |

---

## 执行约定（重要）

1. **conda 用你终端默认的那个**（`~/conda`，`.bashrc` 已初始化）。所有 `conda` 命令直接在普通终端跑即可。
2. **符号说明**：🟢 安全步骤 ｜ 🔴 破坏性/需 sudo ｜ ✅ 验证检查点 ｜ 📝 说明 ｜ ⚠️ 注意
3. **一步一步来**：每完成一个「✅ 检查点」，把终端输出贴给我确认，再进行下一步。**遇到红色报错先停下贴给我**，不要硬往下跑。
4. 命令块可整段复制；`#` 后是说明，不用手敲。

---

## Step 1 · 公共系统依赖 + CycloneDDS + unitree_sdk2py 🟢🔴

> 两个环境（teleop / sim）都要用 DDS，这里一次性把公共部分装好。

### 1.1 系统包（官方脚本要求）🔴

```bash
sudo apt-get update && sudo apt-get install -y cmake build-essential openssl git-lfs unzip
```

### 1.2 克隆公共源码到 DATA 盘 🟢

```bash
mkdir -p /home/amit/DATA/envs
cd /home/amit/DATA
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
git clone https://github.com/unitreerobotics/unitree_sdk2_python
```

### 1.3 编译 CycloneDDS 🟢

```bash
cd /home/amit/DATA/cyclonedds
mkdir -p build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install -j"$(nproc)"
```

### 1.4 设置 CYCLONEDDS_HOME（持久化）🟢

```bash
# 写入 ~/.bashrc，之后每装一个环境的 unitree_sdk2py 都能找到
grep -q CYCLONEDDS_HOME ~/.bashrc || echo 'export CYCLONEDDS_HOME=/home/amit/DATA/cyclonedds/install' >> ~/.bashrc
source ~/.bashrc
echo "CYCLONEDDS_HOME=$CYCLONEDDS_HOME"
```

### 1.5 确认 unitree_sdk2py 版本足够新 🟢

> 📝 `xr_teleoperate` v1.1+ 要求 unitree_sdk2_python ≥ commit `404fe44`。克隆的是最新 main，通常满足。

```bash
cd /home/amit/DATA/unitree_sdk2_python && git log -1 --oneline
```

### ✅ 检查点 1

```bash
ls /home/amit/DATA/cyclonedds/install/lib/*/libddsc* 2>/dev/null && echo "cyclonedds 编译 OK"
echo "CYCLONEDDS_HOME=$CYCLONEDDS_HOME"
```
期望：能看到 `libddsc.so` 之类文件，且 `CYCLONEDDS_HOME` 非空。
**→ 把这两行输出贴给我，确认后进入 Step 2。**

---

## Step 2 (Phase 1a) · teleop 端环境 🟢

### 2.1 让 DATA/envs 支持按名字激活（便利，可选）🟢

```bash
conda config --append envs_dirs /home/amit/DATA/envs
```

### 2.2 创建 teleop 环境 `tv`（Python 3.10）🟢

```bash
conda create -y -p /home/amit/DATA/envs/tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv          # 若上一行 config 生效，可直接用名字；否则用: conda activate /home/amit/DATA/envs/tv
python --version           # 应显示 Python 3.10.x
```

### 2.3 初始化 xr_teleoperate 的 3 个子模块 🟢

```bash
cd /opt/workspace/xr_teleoperate
git submodule update --init --depth 1
git submodule status       # 三个子模块前面应无 '-' 号（'-' 表示未初始化）
```

### 2.4 安装 unitree_sdk2py（公共源码）🟢

```bash
conda activate tv
export CYCLONEDDS_HOME=/home/amit/DATA/cyclonedds/install   # 保险起见当前会话再导一次
pip install -e /home/amit/DATA/unitree_sdk2_python
```

### 2.5 安装 3 个子模块 🟢

```bash
cd /opt/workspace/xr_teleoperate
pip install -e teleop/teleimager --no-deps
pip install -e teleop/televuer
pip install 'params_proto==2.13.0'          # ← 关键修正：vuer0.0.60 与 params_proto 3.x 不兼容，见下方 ⚠️
pip install -e teleop/robot_control/dex-retargeting
pip install nvidia-nvjitlink-cu12==12.1.105  # ← 关键修正：补 torch2.3.0(cu121) 缺失的 libnvJitLink.so.12，见下方 ⚠️
```

> 📝 `hand_retargeting.py` 里是 `from dex_retargeting import RetargetingConfig` 直接导入，所以 dex-retargeting 子模块**必须** `pip install -e`。
>
> ⚠️ **已知坑（vuer 0.0.60 × params_proto 3.x 不兼容）**：`televuer` 依赖 `vuer[all]==0.0.60`，而 vuer 只声明 `params-proto>=2.13.0`（**没封上限**），pip 会装上 3.3.0；但 params_proto 3.x 移除了顶层的 `Flag/PrefixProto/Proto`，于是 `vuer/server.py` 的 `from params_proto import Flag, PrefixProto, Proto` 失败 → `from vuer import Vuer` 失败。而 `vuer/__init__.py` 用 `try/except` 把真错误吞了，只打印"请装 vuer[all]"的**误导提示**（其实 aiohttp 早已装好）。**解决：装完 televuer 后把 params_proto 钉回 2.x**（上面命令已含 `pip install 'params_proto==2.13.0'`）。

> ⚠️ **已知坑（torch 2.3.0 cu121 缺 libnvJitLink.so.12）**：`dex-retargeting` 的 `optimizer.py` 会 `import torch`，其 `pyproject.toml` 钉死 `torch==2.3.0`（pip 默认装成 CUDA/cu121 版）。该 wheel 需要 `nvidia-nvjitlink-cu12` 提供 `libnvJitLink.so.12`，但 pip 有时会漏装这一个库（其它 11 个 nvidia-* 都装了），导致 `import torch` 报 `libnvJitLink.so.12: cannot open shared object file`。**解决：补装这个库**（上面命令已含 `pip install nvidia-nvjitlink-cu12==12.1.105`）。若不想让 teleop 占 GPU，也可改装 CPU 版：`pip install torch==2.3.0 --index-url https://download.pytorch.org/whl/cpu`。

### 2.6 安装 requirements + README 未列全的依赖 🟢

```bash
cd /opt/workspace/xr_teleoperate
pip install -r requirements.txt
# README 未显式列出、但代码实际 import 的：
pip install casadi pyzmq logging_mp opencv-python pyyaml
```

### 2.7 生成 SSL 证书（Quest3 走 HTTPS/WSS 必需，⚠️必须带 SAN）🟢

```bash
cd /opt/workspace/xr_teleoperate/teleop/televuer
# ⚠️ 必须带 subjectAltName(SAN) 且含本机局域网 IP！Quest 浏览器(Chromium)只认 SAN、忽略 CN，
#    对需要开 wss/WebXR 的 8012 页面，名字不匹配的证书会直接判"访问被拒绝"（桌面 Chrome 却能点继续）。
openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
  -keyout key.pem -out cert.pem \
  -subj "/CN=xr-teleoperate" \
  -addext "subjectAltName=IP:192.168.10.43,DNS:localhost,IP:127.0.0.1"
# 验证 SAN 已写入（应看到 IP Address:192.168.10.43, DNS:localhost, IP Address:127.0.0.1）
openssl x509 -in cert.pem -noout -text | grep -A1 "Subject Alternative Name"
mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/
ls -l ~/.config/xr_teleoperate/
cd /opt/workspace/xr_teleoperate
```

> 📝 这份证书 teleop 端(Vuer:8012) 和 sim 端(图像 WebRTC:60001) 共用，放 `~/.config/xr_teleoperate/` 一处即可。两端进程**启动时**加载它，所以**换证书后 sim 与 teleop 都要重启**才生效。
> ⚠️ 若本机局域网 IP 变了（DHCP 重分配），SAN 里的 IP 就对不上，Quest 又会"访问被拒绝"——重生成证书即可，或在路由器给本机做 DHCP 保留（固定 IP）一劳永逸。

### 2.8 防火墙放行（Vuer 8012 + WebRTC 60001~60003）🔴

```bash
# 本机 ufw 实测为 active。Quest3 需到达：8012(Vuer 控制+手部数据) 与 60001~60003(相机 WebRTC 信令/TCP)
sudo ufw allow 8012/tcp
sudo ufw allow 60001:60003/tcp
sudo ufw status verbose
```

> 📝 若 `ufw status` 显示 `Status: inactive`（部分 Ubuntu 桌面默认），说明防火墙没开，以上可跳过。
> ⚠️ WebRTC **媒体流走 UDP 临时端口**；ufw 默认放行 `RELATED,ESTABLISHED`，一般够用。若 Step5 头显黑屏（信令通、视频不通），临时 `sudo ufw disable` 测试（测完 `sudo ufw enable` 恢复）。

### ✅ 检查点 2A —— 依赖自检

```bash
conda activate tv
cd /opt/workspace/xr_teleoperate/teleop
python -c "import casadi, pinocchio, zmq, cv2, yaml, logging_mp, sshkeyboard, matplotlib, meshcat, rerun; import unitree_sdk2py; from televuer import TeleVuerWrapper; import dex_retargeting; from teleimager.image_client import ImageClient; print('✅ teleop deps OK')"
```
期望：打印 `✅ teleop deps OK`，无 ImportError。
- 若某个包报 `ModuleNotFoundError`，把包名贴给我，多半是漏装或需换安装方式。

**→ 把检查点 2A 的输出贴给我。通过后我们做 Step 3（仿真端）。**

---

## Step 3 (Phase 1b) · sim 端环境（复用 env_isaaclab）🟢

> 策略：**复用**你已就绪的 `env_isaaclab`（Isaac Sim/Lab 已装好，不重新下载/编译）。为安全，先做 pip 快照备份，便于对比/回滚。

### 3.1 克隆 unitree_sim_isaaclab 到 DATA 盘 🟢

```bash
cd /home/amit/DATA
git clone https://github.com/unitreerobotics/unitree_sim_isaaclab
cd unitree_sim_isaaclab
git submodule update --init --depth 1
ls -d teleimager && ls requirements.txt   # 官方脚本的目录安全校验：两者都要在
```

### 3.2 激活 env_isaaclab + 快照备份 🟢

```bash
conda activate env_isaaclab
pip freeze > /home/amit/DATA/env_isaaclab_pipfreeze_before.txt
python -c "import isaacsim, isaaclab; print('✅ isaacsim + isaaclab OK')"
```
期望：打印 `✅ isaacsim + isaaclab OK`。若报错，停下贴给我（说明 activate 链路有问题）。

### 3.3 安装 unitree_sdk2py（公共源码）🟢

```bash
conda activate env_isaaclab
export CYCLONEDDS_HOME=/home/amit/DATA/cyclonedds/install
pip install -e /home/amit/DATA/unitree_sdk2_python
```

### 3.4 安装 sim 的 requirements 🟢

```bash
cd /home/amit/DATA/unitree_sim_isaaclab
# ⚠️ evdev 源码编译会失败（conda sysroot 旧内核头 与 /usr/include 新头不一致 → KEY_LINK_PHONE 未定义）。
#    改用 conda-forge 的【预编译】evdev 绕开编译；不带 -y，先核对它将改动哪些包再确认。
conda install -c conda-forge evdev
pip show evdev >/dev/null 2>&1 && echo "✅ pip 已能看到 evdev，requirements 不会再源码编译它"
pip install -r requirements.txt
```

> ⚠️ 这一步可能因 `pyzmq==27.0.0` / `rerun-sdk==0.20.1` 触发个别版本调整。装完先别慌，继续 3.5，最后用 3.2 的 import 自检确认 Isaac 仍正常即可。
>
> ⚠️ **已知坑（evdev 源码编译失败，1.7.1/2.0.0 都一样）**：`pynput==1.8.1` 依赖 `evdev>=1.3`，pip 会从源码编译 evdev。编译分两步：**生成** `ecodes.c` 时读系统头 `/usr/include/linux/input-event-codes.h`（较新，**有** `KEY_LINK_PHONE`），但 conda 编译器 `x86_64-conda-linux-gnu-cc` **编译**时用的是 conda sysroot 里的旧内核头 `env_isaaclab/x86_64-conda-linux-gnu/sysroot/usr/include/linux/input-event-codes.h`（**没有** `KEY_LINK_PHONE`）→ 两套头不一致 → `error: 'KEY_LINK_PHONE' undeclared`。所以钉任何 evdev 版本都没用。**解决（二选一）**：① `conda install -c conda-forge evdev`（预编译 py311 版，免源码编译，推荐；不带 -y 先核对改动）；② pynput 只被可选脚本 `send_commands_keyboard.py` 用到（且有 try/except 保护），我们的 Quest3→sim 流程不需要它，可跳过：`grep -v "^pynput" requirements.txt > /tmp/req.txt && pip install -r /tmp/req.txt`。

### 3.5 安装 sim 仓库内 teleimager（出厂配置已就绪）🟢

```bash
cd /home/amit/DATA/unitree_sim_isaaclab/teleimager
pip install -e .

# 📝 出厂配置已正确：head/left_wrist/right_wrist 三个相机都是 type: isaacsim + image_shape: [480, 640]
#    （且 enable_zmq: true，端口 55555/55556/55557）。下面只确认，无需 sed 修改：
grep -nE "type:|image_shape:|enable_zmq:|zmq_port" cam_config_server.yaml
# 仅当 grep 显示 type 不是 isaacsim 时，才手动把对应行改回 isaacsim
```

### 3.6 下载 USD 资产（走 HuggingFace + git-lfs，>1GB）🟢

```bash
cd /home/amit/DATA/unitree_sim_isaaclab
chmod +x fetch_assets.sh
bash fetch_assets.sh
```
> 📝 脚本会 clone `unitree_sim_isaaclab_usds`、校验 `assets.zip`>1GB、解压并把 `assets/` 放到仓库根，最后删临时目录。耗时取决于网速。

### 3.7 （仅按需）libstdc++ 补丁 🟢

```bash
# 只有当 3.8 启动时报 libstdc++/GLIBCXX 相关错误时才执行：
# conda install -y -c conda-forge libstdcxx-ng
```

### ✅ 检查点 3 —— 独立启动仿真（先不接 Quest3）🟢

```bash
conda activate env_isaaclab
cd /home/amit/DATA/unitree_sim_isaaclab
python sim_main.py --device cpu --enable_cameras --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint --enable_dex3_dds --robot_type g129
```

期望：
- 首次启动会加载资源，**耐心等**（8G 显存会更慢）。
- 弹出 Isaac Sim 窗口；按官方提示：点 `PerspectiveCamera → Cameras → PerspectiveCamera` 看主视角。
- **在窗口里点一下激活**，终端出现：`controller started, start main loop...`
- 场景里能看到 **G1 机器人 + 桌上的圆柱体**。

> ⚠️ 8G 显存注意：若卡顿/爆显存，可加 `--no_render`（无窗口 + WebRTC 流），或后续我帮你降相机分辨率。
> 📝 关闭仿真：终端 `Ctrl+C`。

**→ 仿真能独立拉起 G1+Dex3 后，把终端关键输出（尤其 `controller started`）贴给我，进入 Step 4 联调。**

---

## Step 4 (Phase 1c) · 两端 DDS 联调 🟢

> 需要**两个终端**：终端 A 跑 sim，终端 B 跑 teleop。先不戴头显，验证 DDS 通不通。

### 4.1 查本机局域网 IP（Quest3 之后也要用）🟢

```bash
hostname -I            # 记下和 Quest3 同一 WiFi/网段的那个 IP，例如 192.168.x.x
ip -brief addr show    # 更详细：看哪块网卡连着你的路由器
```

> 📝 **本机实测**：局域网 IP = **192.168.10.43**（WiFi 网卡 `wlx34f7168aa14c`，默认路由 192.168.10.1）。Quest3 要连同一路由器 WiFi，浏览器访问 `https://192.168.10.43:8012`。另有 `10.190.174.204` 与 docker `172.17.0.1`——多网卡若 DDS 不通见 4.3 ⚠️。

### 4.2 终端 A —— 启动仿真 🟢

```bash
conda activate env_isaaclab
cd /home/amit/DATA/unitree_sim_isaaclab
# 两端 DDS 统一钉到回环网卡 lo（本机多网卡 wifi+ZeroTier，默认发现易失配 → "Failed to subscribe dds"；见 4.3 ⚠️）
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
python sim_main.py --device cpu --enable_cameras --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint --enable_dex3_dds --robot_type g129
```
等它出现 `========= start controller success =========` 并开始刷 `~20Hz` 频率统计，**保持运行**。

### 4.3 终端 B —— 启动 teleop（--sim 模式）🟢

```bash
conda activate tv
cd /opt/workspace/xr_teleoperate/teleop
# 两端 DDS 必须钉同一网卡 lo（与终端A完全一致），否则发现不了 sim → "Failed to subscribe dds within 5.0 seconds"
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
# Step4 只验证 DDS/ZMQ 链路（不戴头显），--img-server-ip 用 127.0.0.1 即可。
# ⚠️ 到 Step5 戴 Quest3 时【必须】把 --img-server-ip 改成局域网 IP（192.168.10.43），否则头显黑屏取不到 WebRTC 画面（见 Step5.2）。
python teleop_hand_and_arm.py --input-mode=hand --arm=G1_29 --ee=dex3 --sim --img-server-ip=127.0.0.1
```

期望：
- 终端 B 打印 Vuer 服务地址（形如 `https://<IP>:8012`）并等待 XR 设备连接。
- 终端 A（sim）应能收到来自 teleop 的 DDS 指令；若 sim 端手臂有细微归位/响应，说明 DDS 通了。

> 📝 已核对源码：`teleop_hand_and_arm.py` 的 `--sim` 分支执行 `ChannelFactoryInitialize(1)`，与 sim 日志里的 channel 1 一致，DDS 域已对齐 ✅。
>
> ⚠️ **DDS 不通时**（终端 B 收不到 lowstate / sim 不动）：多半是 CycloneDDS 选错网卡（本机有 wifi 192.168.10.43 / 10.190.174.204 / docker 172.17.0.1 三块）。两端都显式指定同一网卡重试，例如：
> ```bash
> export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
> ```
> （`lo`=回环，适合同机；跨机则填局域网网卡名）。teleop 也可加 `--network-interface=<网卡名>`。遇到再把现象贴给我。

**→ 联调现象（两端终端输出）贴给我，确认后进入 Step 5 戴头显。**

---

## Step 5 (Phase 1d) · Quest3 接入 🟢

> 前提：Step 4 已验证 DDS 链路 OK；sim（终端 A）**保持运行**（实测 55555/60000/60001 都在监听）；Quest3 与本机在**同一 WiFi**。
> 本机实测：局域网 IP = **192.168.10.43**（网卡 `wlx34f7168aa14c`）；**ufw 防火墙 = active**（所以下面 5.1 必须做）。

### 5.1 放行防火墙端口（ufw 实测 active，必做）🔴

Step2.8 只放行了 8012，但**头显画面走 WebRTC 的 60001**，必须补放行：

```bash
sudo ufw allow 8012/tcp
sudo ufw allow 60001:60003/tcp      # 三路相机 WebRTC 信令
sudo ufw status numbered | grep -E "8012|6000"
```
> ⚠️ WebRTC 的**媒体流走 UDP 临时端口**。ufw 默认放行 `RELATED,ESTABLISHED`，多数情况下 PC 先发包即可回程；若 5.4 之后**能进 VR 但黑屏**（信令通、视频不通），临时测试可 `sudo ufw disable`（测完 `sudo ufw enable` 恢复），或告诉我帮你精确放行 UDP 段。

### 5.2 重启 teleop（关键：--img-server-ip 换成局域网 IP）🟢

Step4 用的 `127.0.0.1` 只够本机 ZMQ 测试。**戴头显时画面走 WebRTC**：teleop 把 `webrtc_url = https://<img-server-ip>:60001/offer` 塞进网页交给 Quest3 直连取图（源码 `teleop_hand_and_arm.py:137`）。若仍是 `127.0.0.1`，Quest3 会去连它自己 → **黑屏**。终端 B 先 `Ctrl+C`，用局域网 IP 重启：

```bash
conda activate tv
cd /opt/workspace/xr_teleoperate/teleop
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
python teleop_hand_and_arm.py --input-mode=hand --arm=G1_29 --ee=dex3 --sim --img-server-ip=192.168.10.43
```
> 📝 启动后终端 B 打印 Vuer 地址（形如 `https://...:8012`）；图像客户端这次连 `192.168.10.43:60000`（本机 LAN IP，仍可达）。看到 `Press [r] to start syncing` 即就绪，**先别按 r**。

### 5.3 Quest3 浏览器：先分别信任两个证书源，再进 VR

`8012`(Vuer) 与 `60001`(WebRTC 图像) 是**不同端口 = 不同 origin**，自签证书要**各信任一次**，否则会"手部追踪连上、画面黑屏"：

> ⚠️ **若 8012 提示"访问被拒绝"、且没有「继续前往」按钮**（本机 Chrome 却能开）：这是**证书缺 SAN**。Quest 浏览器只认 SAN、忽略 CN，对要开 wss/WebXR 的页面不给绕过。回 **2.7** 用带 `-addext "subjectAltName=IP:192.168.10.43,..."` 的命令重生成证书 → `cp` 到 `~/.config/xr_teleoperate/` → **重启 sim 和 teleop 两端**（进程启动时才加载证书）→ 再回来做 5.3。若 Quest 仍报拒绝（缓存了旧证书例外），清一下浏览器该站点数据或重开浏览器再试。

1. 戴上 Quest3，连到与本机**相同的 WiFi**（192.168.10.x 段）。
2. 打开 Quest 浏览器，**先**访问 WebRTC 图像源信任其证书：
   ```
   https://192.168.10.43:60001/offer
   ```
   出现「不安全」→ **高级 / Advanced** → **继续前往 / Proceed**。页面可能显示报错或一段 JSON，**不要紧**，目的只是让浏览器记住该 origin 的证书。
3. **再**访问 Vuer 主页：
   ```
   https://192.168.10.43:8012/?ws=wss://192.168.10.43:8012
   ```
   同样点「继续前往」信任证书。
4. 页面加载后点 **Virtual Reality**，允许所有权限（手部追踪等），进入 VR 会话。

### 5.4 开始遥操作

1. 头显里应看到**机器人第一视角画面**；同时终端 B 打印：
   ```
   websocket is connected. id:...
   Uplink task running. id:...
   ```
2. **把手臂摆到接近机器人初始位姿**（避免启动瞬间猛动）。
3. 在终端 B 按 **`r`** 开始 → 你的手臂/手指应驱动 sim 里 G1 的手臂 + Dex3 灵巧手。
4. （可选）**`s`** 录制/停止一段 episode；**`q`** 退出。

> ⚠️ **头显黑屏（能进 VR、手部有反应，但没画面）**：按序查 ①5.2 的 `--img-server-ip` 是否是局域网 IP（不是 127.0.0.1）②5.3 第 2 步的 60001 证书是否点了「继续前往」③5.1 的 60001/tcp 是否放行 ④仍不行则 `sudo ufw disable` 试（WebRTC UDP 媒体）。
> ⚠️ **浏览器连不上 8012**：确认 5.1 放行、`192.168.10.43` 是 Quest3 能到达的 IP（同一 WiFi）、teleop 确实在跑并打印了 Vuer 地址。
> 📝 输入模式：`--input-mode=hand`（裸手追踪，体验灵巧手最直观）或 `controller`（手柄）。

**→ 到这一步，Phase 1 全链路就跑通了！把结果（成功 / 卡在哪 + 终端 B 输出）告诉我。**

---

## 故障排查速查

| 现象 | 可能原因 / 处理 |
|---|---|
| `conda activate tv` 找不到环境 | 用全路径 `conda activate /home/amit/DATA/envs/tv`，或重开终端让 2.1 的 config 生效 |
| `import isaacsim` 失败 | 必须**先** `conda activate env_isaaclab`（它靠 activate.d/isaacsim.sh 注入 PYTHONPATH），不能用裸 python 路径 |
| `cannot import name 'Vuer' from 'vuer'`（伴随"装 vuer[all]"误导提示） | 真因是 params_proto 3.x 与 vuer0.0.60 不兼容：`pip install 'params_proto==2.13.0'`（见 2.5 ⚠️） |
| `libnvJitLink.so.12: cannot open shared object` | torch2.3.0(cu121) 缺 nvjitlink 库：`pip install nvidia-nvjitlink-cu12==12.1.105`（见 2.5 ⚠️） |
| 3.4 构建 evdev 报 `KEY_LINK_PHONE undeclared`（1.7.1/2.0.0 都一样） | 真因：conda sysroot 旧内核头 与 /usr/include 新头不一致。用预编译 `conda install -c conda-forge evdev`，或跳过可选的 pynput（见 3.4 ⚠️） |
| sim 启动报 GLIBCXX/libstdc++ | 执行 3.7 的 `conda install -c conda-forge libstdcxx-ng` |
| DDS 两端收不到 / `Failed to subscribe dds` | 见 4.3 ⚠️：两端 `export CYCLONEDDS_URI` 钉同一网卡（同机用 `lo`）并**都重启**；sim 跑很久后新起 teleop 尤其需要 |
| 显存不足/极卡 | sim 加 `--no_render`；或降低相机分辨率（image_shape）；关掉其它占显存程序 |
| Quest3 打不开 8012 | 防火墙(2.8/5.1)、`--img-server-ip` 与浏览器 URL 用同一局域网 IP、同一 WiFi；证书警告点「继续前往」 |
| Quest3 开 8012 报**"访问被拒绝"**（本机 Chrome 正常） | **证书缺 SAN**：Quest 只认 SAN、对 wss/WebXR 页面不给绕过。按 2.7 用 `-addext "subjectAltName=IP:192.168.10.43,DNS:localhost,IP:127.0.0.1"` 重生成证书 → cp 到 ~/.config → **重启 sim+teleop** → Quest3 重新信任两个 origin |
| Quest3 进了 VR 但**黑屏无画面** | Step5 高频坑：①`--img-server-ip` 还是 127.0.0.1（须换 192.168.10.43）②60001 证书没单独信任（先开 `https://192.168.10.43:60001/offer` 点继续）③ufw 没放行 60001/tcp ④WebRTC UDP 媒体被挡（临时 `sudo ufw disable`） |
| 资产下载失败 | 确认 `git lfs install` 已执行、能访问 huggingface；重跑 `bash fetch_assets.sh` |

---

## 完成后 → Phase 2 预告（迁移 G2 + O10，暂不执行）

Phase 1 跑通后，Phase 2 需要在 `xr_teleoperate` 里补齐 5 块（届时我带你逐个做）：
1. **G2 手臂 IK 类**：仿 `robot_arm_ik.py` 写 `G2_ArmIK`，加载 `/opt/workspace/G2_Robot` 的 G2 URDF。
2. **G2 手臂控制器**：仿 `robot_arm.py`，但对接**智元 GDK 协议**（非 unitree DDS）。
3. **O10 手部重定向**：新增 HandType + 写 `o10.yml`（参考 `unitree_dex3.yml`），用 G2_Robot 里的 O10 URDF。
4. **O10 手控制器**：仿 `robot_hand_unitree.py`，对接 GDK 的末端/手指话题。
5. **G2 的 Isaac Sim 资产**：O10/G2 目前只有 URDF/MJCF，需 URDF→USD 转换后自建仿真场景（unitree_sim_isaaclab 不含 G2）。

> 关键差异：智元用 **GDK（Fast-DDS 3.2.2 + Aorta + Cosine Bus）**，与 Unitree 的 `unitree_sdk2py` 协议栈不同，Phase 2 的通信层要换。
