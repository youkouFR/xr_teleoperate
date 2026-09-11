# Phase 3 · 真机遥操作：Quest3 → 智元 G2 + O10（GDK 直驱）

> **状态**：📝 调查 + 设计完成，**本文只做规划与操作手顺，未改动任何代码**。
> **前置**：Phase 1（Quest3→G1+Dex3→Isaac Sim）✅；Phase 2 Block1+3（`G2_ArmIK` + `o10.yml` 重定向）✅；Block2/4/5（G2+O10 **仿真**集成，DDS 桥）✅。
> **本文目标**：把仿真链路的**下层**（teleop↔Isaac Sim 的 DDS 桥）替换成 **teleop↔真机 G2 的 GDK 直驱**，上层（Quest3 采集 / IK / 手部重定向 / 主循环 / 录制）**最大化复用**。
> **分工不变**：我写手顺 + 只读诊断；你执行（含真机上电、网线、sudo、急停监护）。
> **⚠️ 安全红线**：本阶段驱动**真实硬件**。任何下发前必须完成 §6 的安全检查（场地清空、急停踏板在脚边、机器人悬吊/护架、限位确认、专人监护）。第一次联调**手臂必须处于悬吊或零位附近**，严禁载人/靠近。

---

## 0. 与 Phase 2（仿真）的本质区别

| 维度 | Phase 2 仿真（已完成） | Phase 3 真机（本文） |
|---|---|---|
| 本体 | Isaac Sim 里的 G2 USD | 真实 G2（`10.42.1.101`） |
| 下层通信 | CycloneDDS 桥（`rt/lowcmd`、`rt/o10/*/cmd`，domain 1） | **GDK**（`agibot_gdk`，AORTA/FastDDS，`10.42.1.x`） |
| 臂下发 | `G2_ArmController` 250Hz 发 `rt/lowcmd` | GDK `move_arm_joint_servo` / `joint_servo_control` **100Hz** |
| 手下发 | `O10_Controller` 发 `rt/o10/{l,r}/cmd`（HandCmd_） | GDK `move_ee_pos(target_type="o10_t2")` 或并入 `joint_servo_control` |
| 状态回读 | 订阅 `rt/lowstate`、`rt/o10/*/state` | GDK `get_joint_states()` / `get_end_state()` |
| 视觉 | sim 侧 `image_server` 读 USD 相机→WebRTC 60001-3 | GDK `Camera.get_latest_image()`→（适配）WebRTC 60001-3 |
| 运行位置 | 两台逻辑进程（teleop PC + sim PC/进程） | **GDK 直接跑在 teleop PC**（py3.10 与 `tv` 环境一致） |
| 增益来源 | sim 仓 `robots/agibot.py` 的 `ImplicitActuatorCfg` | 真机底层 PD（GDK 位控模式），teleop 不再发 kp/kd |

> **一句话**：上层不动，**把 `G2_ArmController` / `O10_Controller` 里的 DDS 收发换成 GDK 调用**，再加一个「GDK 相机 → teleimager 兼容图像服务」的适配，即可从仿真切到真机。

---

## 1. 摸底结论（本机 + SDK v3.3.8 实测）

### 1.1 SDK 与安装状态
- **SDK 版本**：`G2_SDK文档_v3.3.8.md`（20856 行）+ `app_v2/`（`gdk/`、`lib/`（733+ 个 `.so`）、`bin/`、`conf/`、`tools/`）。
- **已安装**：`~/.cache/agibot/app/`（今日 14:58 装好），即官方 `curl -sSL http://10.42.1.101:8849/install.sh | bash` 的落地目录。
- **Python 绑定**：`agibot_gdk`（pybind，**绑定 Python 3.10 / x86_64**）——与 teleop 的 `tv` 环境（py3.10）**完全一致**，这是真机能直接在 teleop PC 上跑 GDK 的前提。
- **示例代码**：`~/.cache/agibot/app/gdk/examples/python/`（本地副本 `app_v2/gdk/examples/python/`）：
  - `servo_control.py`（臂 14 关节 200Hz 正弦，**最贴近遥操作**）
  - `mc_example.py`（键盘单关节/动作序列回放）
  - `robot_demo.py`（菜单式：`get_joint_states`/`get_end_state`/`move_ee_pos`/`get_whole_body_status`/急停踏板状态，**Stage 0 冒烟首选参照**）
  - `camera_web_viewer.py`（Flask:5000 多相机 base64 预览，相机链路参照）
  - `camera_demo.py`、`imu_demo.py`、`lidar_demo.py`、`slam_demo.py`、`move_chassis.py`、`pnc_example.py`

### 1.2 网络实测（关键，含一个已踩的坑）
- **拓扑**：开发机 ↔（网线直连）↔ G2。机器人默认 IP `10.42.1.101`；开发机需配静态 IP `10.42.1.102`（`10.42.1.x` 网段，除 101 外均可）。
- **本机现状（今日实测）**：
  - `13:27` `mode_switch` 日志显示**成功连上** `http://10.42.1.101:2379`（真机在线、AORTA discovery 通）。
  - `15:46` 某 GDK 进程**失败**：`http client connect to 127.0.0.1:2379 fail / Connection refused` → **根因是该 shell 没 `source env.sh`**，`AORTA_DISCOVERY_URI` 未导出，AORTA 回退到 `127.0.0.1:2379`。
  - **当前网卡**（写本文时）：只有 WiFi `wlx34f7168aa14c=192.168.10.47/24`、`ztcdcjhjsy=10.190.174.204/24`、`docker0`；**`10.42.1.102` 直连网卡当前不在**，`ip route get 10.42.1.101` 竟走 WiFi 网关。→ **说明机器人直连网线当前未插/未配静态 IP，联调前必须先恢复 §4.1。**
- **两个网段并存**：`10.42.1.x`（机器人直连，GDK/AORTA）与 `192.168.10.x`（WiFi，Quest3 WebXR + 图像 WebRTC）。真机联调时**两块网卡同时在线**，注意 DDS/AORTA 只绑 `10.42.1.x`，WebXR/WebRTC 走 `192.168.10.x`。

### 1.3 `env.sh` 做了什么（`app_v2/env.sh`）
```bash
export LD_LIBRARY_PATH+=":<app>/lib 下所有含 .so 的目录"   # 733+ 个 .so
export PATH+=":<app>/bin"                                  # mode_switch 等
export PYTHONPATH="<app>/gdk/lib:$PYTHONPATH"              # protobuf 消息包（rh_msgs_v4_pb 等）
export APP_CONF_PATH="<app>/gdk/config/app_conf.yml"
# 自动探测 10.42.1.* 本机 IP：
export LOCATOR_IP=<10.42.1.x>
export AORTA_DISCOVERY_URI=http://10.42.1.101:2379         # ★缺它就连不上机器人★
export AORTA_DISPATCHER_THREAD_NUM=6
```
> ⚠️ 若探测不到 `10.42.1.*` 网卡，`env.sh` 只打印 `WARN no ip in 10.42.1.* found` 并 `return 0`，**不会导出 `AORTA_DISCOVERY_URI`** → 后续 GDK 回退 `127.0.0.1:2379` 失败（正是 15:46 的坑）。

### 1.4 GDK 模式（`bin/mode_switch`）
| 模式 | 用途 | 遥操作选择 |
|---|---|---|
| `base` | 基础运控 + 传感器 + 建图移动 | **控制走这个**（默认相机：头双目/头彩/头深/左右手彩） |
| `develop` | 可自定义开关相机等 | 需要额外相机时切这个 |
| `base-fastdds` | FastDDS 中间件，支持 ROS2 | 想用 `ros2 topic echo /gdk/joint_state` 调试时 |

切换：`source ~/.cache/agibot/app/env.sh && ~/.cache/agibot/app/bin/mode_switch --mode base`

---

## 2. 真机整体架构与数据流

```
┌─────────────┐  WebXR/HTTPS(8012)   ┌──────────────────────── teleop PC (conda tv, py3.10) ───────────────────────┐
│  Quest 3    │◄────────────────────►│  teleop_hand_and_arm.py  --arm G2 --ee o10   (★不带 --sim★)                    │
│ (WiFi 192.  │  位姿/手骨架 ↑        │   ├─ tv_wrapper      : 头/腕位姿(4x4) + 手骨架(25x3)                            │
│  168.10.x)  │  相机图像 ↓(WebRTC)   │   ├─ G2_ArmIK        : solve_ik → 双臂 14 关节角                               │
└─────────────┘                       │   ├─ HandRetargeting : o10.yml → 双手 20 关节角（含 _HUMAN_TO_O10_ROT 帧修正）  │
                                      │   ├─ ImageClient     : 从「GDK 相机适配服务」取 config + 帧(WebRTC 60001-3)     │
                                      │   └─ ★GDK 后端(新)★ : agibot_gdk.Robot() / Camera()                            │
                                      │        ├─ 100Hz 伺服线程: joint_servo_control(臂14 + 手20)                     │
                                      │        ├─ 状态线程    : get_joint_states()/get_end_state() → IK seed           │
                                      │        └─ 相机适配    : Camera.get_latest_image() → teleimager WebRTC          │
                                      └───────────────────────────────┬──────────────────────────────────────────────┘
                                                                       │ 网线直连 AORTA/FastDDS  10.42.1.102 ↔ 101
                                                                  ┌────▼─────┐
                                                                  │ 真机 G2  │  双臂14 + O10双手20 + 头/腰/底盘
                                                                  └──────────┘
```

### 2.1 复用 vs 替换（改动量）
| 层 | 文件 | 仿真现状 | 真机要做 | 改动量 |
|---|---|---|---|---|
| Quest3 采集 | `televuer/*` | 位姿+手骨架 | **不变** | 0 |
| 手臂 IK | `robot_arm_ik.py::G2_ArmIK` | 已跑通 | **不变**（IK seed 来源换成 GDK） | 0 |
| 手重定向 | `hand_retargeting.py` + `o10.yml` | 已跑通 | **不变** | 0 |
| 主循环 | `teleop_hand_and_arm.py` | `--sim` 走 DDS domain 1 | 加「GDK 后端」分支（不带 `--sim`） | 小 |
| 臂控制器 | `robot_arm.py::G2_ArmController` | 250Hz 发 `rt/lowcmd` | **真机分支**：GDK 100Hz 伺服 | 中 |
| 手控制器 | `robot_hand_o10.py::O10_Controller` | 发 `rt/o10/*/cmd` | **真机分支**：GDK `move_ee_pos`/并入伺服 | 中 |
| 视觉 | sim `image_server` | USD 相机→WebRTC | **GDK 相机适配服务**→WebRTC | 中 |
| 状态订阅 | `sim_state_topic.py` | 订阅 sim 状态 | 换成 GDK 状态（录制用） | 小 |

### 2.2 关节索引契约（沿用 Phase 2 Block2 §1.1，两侧必须一致）
- **双臂 14**（顺序固定，GDK `move_arm_joint_servo` 要求同序）：
  `idx21..27_arm_l_joint1..7`（左）+ `idx61..67_arm_r_joint1..7`（右）。
- **全身 22 关节定位序**（`joint_control_request` / `servo_control.py`）：
  `idx01..05_body_joint1..5`（躯干5）+ 头3 + 臂14。
  > ⚠️ **头关节顺序有坑**：`servo_control.py` 用 `head_joint1, head_joint3, head_joint2`（1,3,2），`mc_example.py` 用 `1,2,3`。以 `get_joint_states()` 返回的 `name` 为准，**按名取值**，不要按死索引。遥操作只控臂+手时，头/躯干可不进伺服列表。
- **O10 每手 10**：见 §3.5，与 `robot_hand_o10.py::O10_Left/Right_JointIndex` **1:1 对齐**（重大利好）。

---

## 3. GDK 控制 API 详解（真机契约，全部来自 v3.3.8 文档 + 示例实测）

### 3.1 生命周期
```python
import agibot_gdk, time
assert agibot_gdk.gdk_init() == agibot_gdk.GDKRes.kSuccess   # 失败勿继续
robot = agibot_gdk.Robot()
time.sleep(2)            # ★必须等 ~2s，等 DDS/AORTA 连接建立★
...
agibot_gdk.gdk_release() # 程序结束前释放
```
- 所有接口失败抛 `RuntimeError`，需 try/except。
- 位置单位**弧度**，速度**弧度/秒**，时间戳**纳秒**。

### 3.2 状态回读（IK seed + 安全监控）
| 接口 | 返回关键字段 | 用途 |
|---|---|---|
| `get_joint_states()` | `{timestamp, nums, states:[{name, motor_position, motor_velocity, effort, error_code, ...}]}` | **臂 14 当前 q/dq**（IK seed）。⚠️用 `motor_position`/`motor_velocity`；`position`/`velocity` 是低速预留字段，勿用 |
| `get_end_state()` | `{left_end_state/right_end_state:{controlled, type, names:[...], end_states:[{id, enable, position, velocity, effort, err_code, ...}]}}` | **O10 手当前 q** + `names`（确认手关节实际顺序） |
| `get_whole_body_status()` | `{left/right_arm_error, left/right_arm_control, left/right_arm_estop, left/right_end_error, left/right_end_model, waist/lift/neck/chassis_error}` | **每周期安全监控**：错误码/急停/是否在被控 |
| `get_motion_control_status()` | `{mode(0停/1伺服/2规划), control_mode(1/2/3), error_code, error_msg, frame_names, frame_poses, twists, wrenches, arm_disturbance_force, collision_pairs_1/2}` | 控制模式确认 + 碰撞/外力监控 |
| `get_chassis_power_state()` | `emergency_stop_pedal_state`、`emergency_stop_button_req`、`battery_states`、… | 急停踏板/电量 |

### 3.3 手臂控制
- **慢速定位（起始/回零）**：`joint_control_request(JointControlReq)`
  ```python
  req = agibot_gdk.JointControlReq()
  req.life_time = 0.1                 # 命令生命周期(s)，避免过期命令
  req.joint_names = [...按名...]       # 任意子集（如仅双臂14）
  req.joint_positions = [...]         # 与 names 等长，必须在限位内
  req.joint_velocities = [0.3]*len    # 或标量
  robot.joint_control_request(req)    # 支持模式 1(位控)/3(关节阻抗)
  ```
- **高频流式伺服（遥操作主力）**：`move_arm_joint_servo(positions, control_period, control_group, enable_low_latency=False)`
  - `positions`：长度 **7（单臂）或 14（双臂）**，顺序 = §2.2 双臂序。
  - `control_period`：控制周期(s)，建议比实际发送周期稍大（如 `0.01`）。
  - `control_group`：**0=左臂 / 1=右臂 / 2=双臂**。
  - **必须以 ~100Hz 持续发送**；超限/长度错→抛异常。
  - `enable_low_latency=True` 走低延时通道，但**无碰撞保护**，遥操作默认用 `False`。

### 3.4 手 / 末端控制（O10）
- **`move_ee_pos(JointStates)`**（夹爪/灵巧手开合，非阻塞）
  ```python
  js = agibot_gdk.JointStates()
  js.group = "left_tool"              # left_tool / right_tool / dual_tool
  js.target_type = "o10_t2"           # ★O10 = "o10_t2"，需 10 个关节★
  js.states = [agibot_gdk.JointState(position=p) for p in positions10]
  js.nums = len(js.states)            # 必须 == len(states)
  robot.move_ee_pos(js)               # dual_tool 时 states 长度 = 单手×2（先左后右）
  ```
  > ⚠️ **互斥警告（文档原文）**：`move_ee_pos` **不可与伺服接口同时使用**；若要与臂伺服同时控手，**必须通过伺服接口（`joint_servo_control`）下发末端**。
- **`move_end_effector_joint(positions, velocities, group)`**：末端关节伺服（positions/velocities 等长）。
- **`end_effector_pose_control(...)`**：末端**笛卡尔位姿**控制（模式 1/2/3），遥操作若走「腕位姿直控」可用，但本项目走 IK→关节，暂不用。

### 3.5 ★O10 关节顺序与限位（`target_type="o10_t2"`，每手 10）★
> 与 `robot_hand_o10.py::O10_Left/Right_JointIndex`（thumb_roll→pinky_pip）**逐项一致**，重定向输出可 1:1 直送（符号/量纲仍需 §7 Stage 3 实测确认）。

**左手（`left_tool`）**
| idx | 关节名 | min | max |
|---|---|---|---|
|0|`idx31_hand_l_thumb_roll_joint`|-1.1214|0.0297|
|1|`idx32_hand_l_thumb_abad_joint`|-0.0454|1.6424|
|2|`idx33_hand_l_thumb_mcp_joint`|-0.8416|0.0|
|3|`idx36_hand_l_index_abad_joint`|0.0|0.1641|
|4|`idx37_hand_l_index_pip_joint`|0.0|1.4835|
|5|`idx39_hand_l_middle_pip_joint`|0.0|1.4835|
|6|`idx41_hand_l_ring_abad_joint`|-0.1693|0.0|
|7|`idx42_hand_l_ring_pip_joint`|0.0|1.4835|
|8|`idx44_hand_l_pinky_abad_joint`|-0.1850|0.0|
|9|`idx45_hand_l_pinky_pip_joint`|0.0|1.4835|

**右手（`right_tool`）**
| idx | 关节名 | min | max |
|---|---|---|---|
|0|`idx71_hand_r_thumb_roll_joint`|-0.0297|1.1214|
|1|`idx72_hand_r_thumb_abad_joint`|-1.6424|0.0454|
|2|`idx73_hand_r_thumb_mcp_joint`|0.0|0.8416|
|3|`idx76_hand_r_index_abad_joint`|-0.1641|0.0|
|4|`idx77_hand_r_index_pip_joint`|0.0|1.4835|
|5|`idx79_hand_r_middle_pip_joint`|0.0|1.4835|
|6|`idx81_hand_r_ring_abad_joint`|0.0|0.1693|
|7|`idx82_hand_r_ring_pip_joint`|0.0|1.4835|
|8|`idx84_hand_r_pinky_abad_joint`|0.0|0.1850|
|9|`idx85_hand_r_pinky_pip_joint`|0.0|1.4835|

**典型值**：左开 `[0]*10`；左握 `[-0.2,1.45,-0.75,0,1,1,0,1,0,1]`；右开 `[0]*10`；右握 `[0.2,-1.45,0.75,0,1,1,0,1,0,1]`。

### 3.6 ★推荐：臂+手统一伺服 `joint_servo_control`（避开 §3.4 互斥）★
```python
req = agibot_gdk.JointServoControlReq()
req.control_period = 0.01
req.joint_names    = ARM14_NAMES + left_ee_names + right_ee_names   # ee_names 从 get_end_state() 取
req.joint_positions = arm14_q    + left10_q      + right10_q
robot.joint_servo_control(req)   # 100Hz；支持模式 1(位控)/3(关节阻抗)
```
- `left_ee_names/right_ee_names` = `get_end_state()['left_end_state']['names']`（O10 即那 10 个手关节名）。
- **一路 100Hz 同时驱动双臂+双手**，无需 `move_ee_pos`，规避互斥；这是本文推荐的真机主控路径。
- 需 §7 Stage 5 实测确认 `joint_servo_control` 接受 O10 手指关节（文档结构支持，示例仅演示了 omnipicker 单关节）。

### 3.7 控制模式与力控（可选，遥操作先用位控）
- `set_control_mode(m)`：**1=关节位控(PD/轨迹，默认) / 2=笛卡尔阻抗 / 3=关节阻抗**。同步阻塞。
- 力控工作流：`set_payload(质量/质心/惯性)` →（持久负载）`torque_sensor_calibration()` → `set_control_mode(2或3)` → `set_cartesian_impedance()/set_joint_impedance()`（**当前仅 `kp` 生效**）。
  - 笛卡尔阻抗双臂 12 维（前6左/后6右，每臂 x,y,z,roll,pitch,yaw）；x/y/z 推荐 600~6000 N/m，rpy 30~300 N·m/rad。
- 遥操作**首阶段用模式 1（位控）**最稳；待跟踪生硬/需柔顺再评估模式 3（关节阻抗）。

### 3.8 相机（视觉回传）
```python
camera = agibot_gdk.Camera()          # 启动全部；或 Camera([CameraType.kHeadColor, kHandLeftColor, kHandRightColor]) 省资源
img = camera.get_latest_image(agibot_gdk.CameraType.kHeadColor, 1000.0)   # timeout ms
# img: {timestamp_ns, width, height, encoding(UNCOMPRESSED/JPEG/PNG), color_format(RGB/BGR/GRAY8/...), bit_depth, data(numpy)}
```
- **CameraType**：`kHeadColor`（头彩）、`kHandLeftColor`/`kHandRightColor`（左右手彩）、`kHeadDepth`、`kHeadStereoLeft/Right`、鱼眼等。
- 常规（base）模式默认开：头双目左右、头彩、头深、左右手彩——**遥操作需要的头彩+双手彩都在默认开启集**。
- 解码：JPEG/PNG 用 `cv2.imdecode`；UNCOMPRESSED 按 `color_format` reshape `(h,w,3)`（RGB→BGR）。
- 辅助：`get_image_shape(type)`、`get_image_fps(type)`、`get_image_latency(type)`、`get_camera_intrinsic(type)`、`set_dev_camera_config(...)`、`close_camera()`。

---

## 4. 环境与网络准备（手顺 · 你执行）

> 全部在 teleop PC 上。**建议新开一个干净 shell**，避免残留 `PYTHONPATH/LD_LIBRARY_PATH` 污染（跨环境诊断先 `unset PYTHONPATH LD_LIBRARY_PATH`）。

### Step 4.1 网线直连 + 静态 IP（★当前缺失，先恢复★）
1. 网线连 G2 的 Debug 网口 ↔ 开发机网口。
2. 配静态 IP（示例网卡名按实际改，如 `enp*s*`/`eth0`）：
   ```bash
   ip addr show                                   # 找到直连网卡名
   sudo ip addr add 10.42.1.102/24 dev <eth>      # 临时；或用 nmcli/netplan 持久化
   sudo ip link set <eth> up
   ip route get 10.42.1.101                        # ★应显示 dev <eth> 直连，而非走 WiFi 网关★
   ping -c3 10.42.1.101                            # 通
   ```
   > 现状（写本文时）`ip route get 10.42.1.101` 走了 WiFi（`via 192.168.10.1`），**必须纠正为直连网卡**，否则 AORTA/GDK 连不上真机。

### Step 4.2 验证 GDK 安装
```bash
ls ~/.cache/agibot/app/{env.sh,bin/mode_switch,gdk/examples/python}
source ~/.cache/agibot/app/env.sh                 # ★每个新 shell 都要先 source★
echo "$AORTA_DISCOVERY_URI"                        # 应=http://10.42.1.101:2379（为空则回 Step 4.1）
```

### Step 4.3 切模式
```bash
~/.cache/agibot/app/bin/mode_switch --mode base    # 遥操作用 base（要额外相机才 develop）
```

### Step 4.4 只读冒烟（不下发，验证链路）
```bash
cd ~/.cache/agibot/app/gdk/examples/python
python3 robot_demo.py       # 菜单选：get_joint_states / get_end_state / get_whole_body_status / 急停踏板
python3 camera_web_viewer.py  # 浏览器开 http://<dev-ip>:5000 看头/手相机
```
**验收**：`robot_demo` 能打印 22+ 关节 `motor_position`、末端 `names`（应见 `idx31/idx71...` O10 手关节）、无 error_code；相机页面出头彩+双手彩。

### Step 4.5 `tv` 环境 × GDK 共存验证（关键风险点）
GDK 的 `env.sh` 会注入 733+ `.so` 到 `LD_LIBRARY_PATH`，可能与 `tv`（torch/CycloneDDS/opencv）冲突。逐条排查：
```bash
source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || source /opt/miniconda3/etc/profile.d/conda.sh 2>/dev/null
conda activate tv
python -c "import sys; print(sys.version)"          # 必须 3.10.x
source ~/.cache/agibot/app/env.sh                    # 在 tv 环境里再 source GDK
python -c "import agibot_gdk; print('agibot_gdk OK')"# ★能 import 即绑定/依赖 OK★
python -c "import torch, cv2, numpy; print('tv libs OK')"  # 确认 GDK 的 .so 没顶掉 tv 的库
```
**若 `import agibot_gdk` 失败**：多半是 pybind `.so` 未装到 tv 的 site-packages → 按文档 `cd ~/.cache/agibot/app/gdk/build_dep/python/pybind && pip install . --no-build-isolation`（在 tv 环境里）。
**若 torch/cv2 崩**：`LD_LIBRARY_PATH` 冲突 → 考虑 §5.7 的「独立 GDK 桥进程」隔离方案。

### Step 4.6 PTP 时间同步（可选，仅录制/多传感器对齐需要）
```bash
cd ~/.cache/agibot/app/gdk/scripts
sudo chmod +x ptp_hard.sh && sudo ./ptp_hard.sh     # Thor 版；非 Thor 用端侧 ptp_server.sh + 客户端 ptp_client.sh
```

---

## 5. 代码改造设计（★只设计，不写实现★）

> 原则：**新增「GDK 后端」分支，不破坏现有仿真路径**。用 `--sim` 的反向（或新增 `--robot-backend {dds,gdk}`）切换；仿真仍走 DDS，真机走 GDK。

### Step 5.1 后端开关（`teleop_hand_and_arm.py`）
- **目标**：`--arm G2 --ee o10` 且**不带 `--sim`** 时走 GDK 真机；带 `--sim` 时维持现状（DDS domain 1）。
- **改动点**：
  - `ChannelFactoryInitialize` 仅在非 GDK 真机时初始化（真机不需要 unitree DDS）。
  - `G2_ArmController(...)` / `O10_Controller(...)` 增加 `robot_backend` 入参（`"gdk"`/`"dds"`）。
  - 真机分支不建 `ImageClient(host=sim_ip)`，改连「GDK 相机适配服务」（§5.5）或直接把相机塞进 tv_wrapper。
- **难点**：`--sim` 与「真机」互斥语义要清晰；`G2_WRIST_TARGET_OFFSET=(0.102,0,1.049)` 是否仍需要（见 §9）。
- **验证**：`--help` 出现新选项；仿真路径回归不受影响。

### Step 5.2 `G2_ArmController` 真机分支（`robot_arm.py`）
- **目标**：把 250Hz DDS 发布换成 **100Hz GDK 伺服**；状态源从 `rt/lowstate` 换成 `get_joint_states()`。
- **现状（仿真，`robot_arm.py:104-220`）**：
  - `_ctrl_motor_state()` 线程 250Hz：填 `msg.motor_cmd[id].q/.dq/.tau`→CRC→`lowcmd_publisher.Write`（`rt/lowcmd`）。
  - `_subscribe_motor_state()` 线程：读 `rt/lowstate`→`lowstate_buffer`。
  - `ctrl_dual_arm(q,tauff)` 只更新 `self.q_target`；`get_current_dual_arm_q/dq()` 读 buffer；`ctrl_dual_arm_go_home()` 缓动到 0。
- **真机设计**：
  - `__init__(robot_backend="gdk")`：`gdk_init()`+`Robot()`+`sleep(2)`；**首帧读 `get_joint_states()` 作为初始目标**（避免上电突跳）。
  - 伺服线程 **100Hz**：把 `self.q_target`（14）经**限速裁剪 + 与上一目标的插值**后 `move_arm_joint_servo(q14, 0.01, 2)`（或与手合并走 §5.4）。
  - 状态线程：轮询 `get_joint_states()`，按 `name` 取 14 臂关节 `motor_position/motor_velocity` 填 buffer（供 IK seed）。
  - 安全线程/每周期：`get_whole_body_status()` 查 `arm_error/arm_estop`；异常→停发→（可选）阻尼。
  - `ctrl_dual_arm_go_home()`：改用 `joint_control_request` 慢速回零或伺服缓动到当前 home。
- **难点**：30Hz(主循环 IK)→100Hz(伺服) 的**插值/平滑**（否则抖）；`life_time`/限速；限位 clamp（用 URDF 限位）。
- **验证**：Stage 2（臂正弦小幅）→ Stage 4（IK 跟随）。

### Step 5.3 `O10_Controller` 真机分支（`robot_hand_o10.py`）
- **目标**：把 DDS `HandCmd_`（`rt/o10/*/cmd`）换成 GDK；状态从 `rt/o10/*/state` 换成 `get_end_state()`。
- **现状（仿真）**：`control_process` @100Hz：手骨架→`_HUMAN_TO_O10_ROT`→`retarget`→`left/right_q_target(10)`→`ctrl_dual_hand`（写 `motor_cmd[id].q`→DDS）；`_subscribe_hand_state` 读 DDS。
- **真机设计（二选一）**：
  - **方案 A（独立）**：`ctrl_dual_hand` → 组 `JointStates(group="dual_tool", target_type="o10_t2", states=20, nums=20)` → `move_ee_pos`。**但 §3.4 互斥**：臂若走 `move_arm_joint_servo`，手不能同时 `move_ee_pos`。→ 仅当臂**不**走伺服时可用。
  - **方案 B（推荐，统一伺服）**：手不再自己下发，把 `left/right_q_target(10+10)` 交给 §5.4 的统一伺服线程，与臂 14 合并成 `joint_servo_control`。O10_Controller 只负责「重定向→产出 20 维目标」，下发集中在臂控制器/统一伺服线程。
- **关键复用**：`_HUMAN_TO_O10_ROT` 帧修正、`o10.yml`、retarget 输出顺序（=§3.5 的 o10_t2 序）**全部不变**。
- **难点**：确认 retarget 输出**符号/量纲**落在 §3.5 限位内（左右手拇指 roll/abad 符号相反）；`nums`/`states` 长度严格匹配。
- **验证**：Stage 3（手开合）。

### Step 5.4 统一 100Hz 伺服线程（推荐主控路径）
- **目标**：一个线程 100Hz 调 `joint_servo_control`，`joint_names = 臂14 + 左手10 + 右手10`，`joint_positions` 同序拼接。
- **数据流**：主循环 30Hz 产出 `arm_q_target(14)` + `hand_q_target(20)` → 写入共享缓冲 → 伺服线程 100Hz 读取、**插值/限速**、下发。
- **ee_names 获取**：启动时 `get_end_state()` 读左右 `names`，缓存（避免每周期查）。
- **难点**：30→100Hz 插值；臂与手的**目标同步**（同一周期拼一帧）；启动首帧 = 当前实测位姿（fade-in）。
- **验证**：Stage 5（臂+手统一伺服）。

### Step 5.5 相机：GDK → teleimager 兼容图像服务
- **目标**：让**现有 `ImageClient` + `tv_wrapper` WebRTC 路径不改**，把 GDK 相机帧喂进 teleimager 的发布协议（WebRTC 60001-3 / ZMQ）。
- **设计**：写一个「GDK 相机适配服务」（teleop PC 本地进程或线程）：
  - `Camera([kHeadColor, kHandLeftColor, kHandRightColor])`；
  - 循环 `get_latest_image()`→解码 BGR→按 teleimager `image_server` 的相机 config（`head/left_wrist/right_wrist`）与 WebRTC 端口发布；
  - 复用 Phase 2 已跑通的 WebRTC/证书（`~/.config/xr_teleoperate/{cert,key}.pem`）。
- **难点**：teleimager `image_server` 原为「读共享内存/IsaacSim 相机」设计，需要一个 GDK 数据源适配（仿 `IsaacSimCamera`，改成 `GdkCamera`）；相机 config 的分辨率/binocular/webrtc_port 要与 tv_wrapper 对齐。
- **省事替代**：若只要「头显第一视角」，可先只接 `kHeadColor` 一路，双手相机后接。
- **验证**：Quest3 浏览器里头显出图（与 Phase 2 体验一致）。

### Step 5.6 主循环频率与安全接线
- teleop 主循环默认 `--frequency 30`；GDK 伺服要 100Hz。→ **主循环维持 30Hz（IK/retarget/录制），伺服在独立 100Hz 线程**（§5.4），两者解耦。
- `finally` 里 `ctrl_dual_arm_go_home()` 改为 GDK 缓动回零 + `gdk_release()`。
- 键盘 `q`（STOP）→ 停伺服线程 → 回零 → 释放。

### Step 5.7 隔离备选：独立 GDK 桥进程（仅当 §4.5 lib 冲突时）
- 若 `tv` 环境与 GDK `LD_LIBRARY_PATH` 无法共存：起一个**独立 py3.10 进程**（纯净 GDK 环境）做「GDK 桥」，teleop 主进程通过本地 IPC（ZMQ/共享内存/DDS domain 0）把 `arm14+hand20` 目标发给桥、从桥收状态/相机。
- 代价：多一跳延迟 + 一套 IPC；**非首选**，仅兜底。

---

## 6. 安全机制设计（真机重中之重）

### 6.1 上电/联调前（人工检查单）
- [ ] 机器人**悬吊/护架**或处于**零位附近**，周围 **≥2m 清空**，无人/无障碍。
- [ ] **急停踏板**（`emergency_stop_pedal`）在操作员脚边，功能已测（`get_chassis_power_state()` 读 `emergency_stop_pedal_state`）。
- [ ] 电量充足（`battery_states`）；线缆不妨碍运动。
- [ ] 有**第二人监护**，手不离急停。
- [ ] 已知本次只控**双臂+双手**（头/腰/底盘不进伺服列表）。

### 6.2 软件安全（下发路径内建）
1. **首帧对齐**：启动先 `get_joint_states()`/`get_end_state()` 读当前位姿，**初始目标 = 当前实测**，再 fade-in 到遥操作目标（`servo_control.py` 的 `fade` 思路），**杜绝上电突跳**。
2. **限位 clamp**：每关节目标 clamp 到 URDF/§3.5 限位；超限丢弃并告警。
3. **限速裁剪**：单周期 `|Δq| ≤ v_max·dt`，防跳变（真机版必须做，仿真版当初省略了）。
4. **life_time**：`joint_control_request` 设合理 `life_time`，避免过期命令被误执行。
5. **状态哨兵**：每周期查 `get_whole_body_status()`（`arm_error/arm_estop`）、`get_motion_control_status()`（`error_code/collision_pairs`）；异常→**立即停发伺服**→保持/阻尼→告警。
6. **超时保护**：若主循环 N 周期没更新目标（XR 掉线），伺服**保持最后安全位姿或缓动回零**，不得放任。
7. **异常兜底**：所有 GDK 调用 try/except `RuntimeError`；`finally` 必 `gdk_release()`。
8. **碰撞检测**：`get/set_collision_detection_config()` 保持开启（低延时模式无碰撞保护，故遥操作**不用** `enable_low_latency`）。
9. **错误恢复**：触发错误后按需 `clear_motion_control_error()`/`clear_hal_error()`，确认 `error_code==0` 再重新使能。

### 6.3 急停层级
- **硬件**：急停踏板 / 急停按钮（最高优先，直接断使能）。
- **软件**：键盘 `q` → 停伺服；XR 手柄（若 `--input-mode controller`）→ 停。
- **被动**：GDK 底层限位/碰撞/错误码触发的自停。

---

## 7. 分阶段联调手顺（bring-up，逐步放权）

> 每一 Stage **先在手臂悬吊/零位附近**做，通过后再进下一 Stage。任何异常先急停。

### Stage 0 — 只读冒烟（不下发）
- 前置：§4.1-4.4 通过。
- 操作：`robot_demo.py` 读 `get_joint_states`/`get_end_state`/`get_whole_body_status`；`camera_web_viewer.py` 看相机。
- 验收：22+ 关节有 `motor_position`；末端 `names` 含 O10 手关节；`arm_error==0`、`estop==False`；相机出图。
- 回退：连不上→查 `AORTA_DISCOVERY_URI`/网卡（§1.2 坑）。

### Stage 1 — 单关节/小幅定位（`joint_control_request`）
- 操作：仿 `mc_example.py`，选**一个**臂关节，±0.1 rad 慢速动。
- 验收：关节按名慢速到位、无报警、`get_joint_states` 回读一致。
- 回退：抖动/超限→降 `joint_velocities`、查限位。

### Stage 2 — 双臂伺服正弦（`move_arm_joint_servo`）
- 操作：复刻 `servo_control.py`：读当前 14 臂角为 base，fade-in 后叠加小幅正弦（amp≈0.087，freq 0.5Hz），100Hz `move_arm_joint_servo(q14, 0.01, 2)`。
- 验收：双臂平滑小幅摆动、无突跳、无碰撞报警。
- 回退：晃/抖→降 amp、查 100Hz 是否稳定、确认首帧对齐。

### Stage 3 — 双手开合（`move_ee_pos` o10_t2）
- 操作：`move_ee_pos(group="dual_tool", target_type="o10_t2", states=20)`，在「开 `[0]*10`」与「握（§3.5 典型值）」间缓动。
- 验收：双手 10 指按序开合、方向正确、无超限。
- 回退：某指反向/顶限位→核 §3.5 限位与 retarget 符号（为 Stage 5 的统一伺服积累映射表）。
- ⚠️ 本 Stage **臂不动**（避开 move_ee_pos×servo 互斥）。

### Stage 4 — IK 跟随（臂，无手）
- 操作：跑 `teleop_hand_and_arm.py --arm G2 --ee o10`（GDK 后端），**手先不下发**，Quest3 动腕→IK→`move_arm_joint_servo` 跟随。
- 验收：双臂跟随腕位姿、可达胸高（验证 `G2_WRIST_TARGET_OFFSET` 真机是否仍需要）、无突跳。
- 回退：整体偏低/偏高→复核 offset（§9）；抖→限速/插值。

### Stage 5 — 臂+手统一伺服（`joint_servo_control`）
- 操作：§5.4 统一线程，`joint_names=臂14+手20`，Quest3 全手势驱动。
- 验收：臂随腕、手随握张，**同步无撕裂**，100Hz 稳定。
- 回退：`joint_servo_control` 若不接受 O10 手关节→退回「臂 servo + 手 move_ee_pos 错时」或查 SDK 更新（§9）。

### Stage 6 — 全链路 + 相机回传
- 操作：接 §5.5 GDK 相机适配服务，Quest3 头显第一视角。
- 验收：头显出头彩（+双手彩），延迟可接受，遥操作闭环。

### Stage 7 — 录制（可选）
- 操作：`--record`，状态源换 GDK（`get_joint_states`/`get_end_state`），相机走 §5.5；必要时 §4.6 PTP 对齐时间戳。
- 验收：episode 落盘，臂/手 qpos 与图像时间对齐。

---

## 8. 整体验收清单（端到端）
- [ ] §4 网络/环境：`10.42.1.102` 直连、`AORTA_DISCOVERY_URI` 正确、`import agibot_gdk` OK、`tv` 库不崩。
- [ ] Stage 0-3 单点能力全过（只读/单关节/臂伺服/手开合）。
- [ ] Stage 4-5：Quest3 → 双臂跟随 + 双手跟随，平滑无突跳、无碰撞报警。
- [ ] Stage 6：头显第一视角出图，闭环延迟可接受。
- [ ] 安全：首帧对齐、限位/限速 clamp、状态哨兵、急停（硬件+软件）全部生效。
- [ ] 退出：`q` → 缓动回零 → `gdk_release()`，机器人安全停。
- [ ] （可选）Stage 7 录制数据时间对齐。

---

## 9. 风险与待确认（open questions，联调中实测）
1. **`tv` × GDK lib 共存**（§4.5）：733 个 `.so` 是否顶掉 torch/cv2/CycloneDDS？失败则走 §5.7 独立桥进程。
2. **`joint_servo_control` 是否接受 O10 手指关节**（§3.6/Stage 5）：文档结构支持，示例仅演示 omnipicker 单关节。若不支持→退回 `move_ee_pos` 与臂伺服**错时/分频**方案。
3. **O10 retarget 输出符号/量纲**（§3.5/Stage 3）：左右手拇指 roll/abad 符号相反，需实测张手/握拳各关节角落在限位内、方向对。
4. **`G2_WRIST_TARGET_OFFSET=(0.102,0,1.049)` 真机是否仍需要**（Stage 4）：该 offset 是为「tv_wrapper 按 G1 腰帧标定、G2 IK root 在地面帧」补的。真机 IK root 与仿真应一致→大概率仍需要，但**实测确认**（若真机臂整体偏低/偏高再调）。
5. **头关节顺序 1,3,2 vs 1,2,3**（§2.2）：若将来纳入头部遥操作，务必**按 `name` 取值**，勿按死索引。
6. **30Hz→100Hz 插值策略**（§5.4）：线性插值 vs 目标保持，影响平滑度/延迟，需调。
7. **相机坐标系/标定**（§5.5）：GDK 相机与 IK/URDF 的外参（`tools/camera_extrinsic_intrinsic.py`、`get_camera_intrinsic`），录制/AR 叠加时需标定。
8. **控制模式**（§3.7）：位控跟踪若生硬，是否切关节阻抗（模式3）+ 调 `kp`。
9. **延迟与抖动**：WiFi(Quest3) + AORTA(机器人) 双网段，端到端延迟需实测；必要时 PTP + 提高伺服稳定性。
10. **底盘/腰/头**：本期只控双臂+双手；若后续要移动底盘（`move_chassis.py`）或腰/头，另开手顺。

---

## 10. 改动文件汇总（预估，真机落地时）
| 文件 | 改动 | 说明 |
|---|---|---|
| `teleop/teleop_hand_and_arm.py` | 加 GDK 后端分支 | `--sim` 反向 / `--robot-backend`；相机源切换；`finally` 加 `gdk_release` |
| `teleop/robot_control/robot_arm.py` | `G2_ArmController` 真机分支 | DDS 发布→GDK 100Hz 伺服；`rt/lowstate`→`get_joint_states`；首帧对齐+限速+哨兵 |
| `teleop/robot_control/robot_hand_o10.py` | `O10_Controller` 真机分支 | DDS HandCmd→GDK；推荐并入统一伺服（方案B） |
| （新）GDK 相机适配服务 | 新建 | `Camera.get_latest_image`→teleimager WebRTC，复用 `ImageClient`/证书 |
| （新，可选）GDK 桥进程 | 新建 | 仅当 §4.5 lib 冲突（§5.7） |
| `teleop/utils/sim_state_topic.py` | 真机状态源 | 录制时状态改从 GDK 取 |

> 上层 `G2_ArmIK`、`hand_retargeting.py`、`o10.yml`、`televuer`、`episode_writer` **不改**。

---

## 附：关键命令 / API 速查
```bash
# 环境（每个新 shell）
sudo ip addr add 10.42.1.102/24 dev <eth>          # 直连静态 IP（当前缺）
source ~/.cache/agibot/app/env.sh                    # 导出 AORTA_DISCOVERY_URI 等
~/.cache/agibot/app/bin/mode_switch --mode base      # 模式
echo $AORTA_DISCOVERY_URI                            # =http://10.42.1.101:2379
cd ~/.cache/agibot/app/gdk/examples/python && python3 robot_demo.py   # 只读冒烟
```
```python
# 控制契约（速查）
agibot_gdk.gdk_init(); robot = agibot_gdk.Robot(); time.sleep(2)
q14 = [s['motor_position'] for s in robot.get_joint_states()['states'] if s['name'] in ARM14]
robot.move_arm_joint_servo(q14, 0.01, 2)                       # 双臂 100Hz
ee_names = robot.get_end_state()['left_end_state']['names']     # O10 手关节名
req = agibot_gdk.JointServoControlReq(); req.control_period=0.01
req.joint_names = ARM14+eeL+eeR; req.joint_positions = arm14+handL10+handR10
robot.joint_servo_control(req)                                  # 臂+手统一伺服
js = agibot_gdk.JointStates(); js.group="dual_tool"; js.target_type="o10_t2"
js.states=[agibot_gdk.JointState(position=p) for p in hand20]; js.nums=20
robot.move_ee_pos(js)                                           # 手开合（勿与伺服同用）
robot.set_control_mode(1)                                       # 1位控/2笛卡尔阻抗/3关节阻抗
cam = agibot_gdk.Camera(); img = cam.get_latest_image(agibot_gdk.CameraType.kHeadColor, 1000.0)
agibot_gdk.gdk_release()
```

> 备注：本文所有 GDK API 签名、O10 关节序/限位、网络与模式、相机类型均来自对 `app_v2`（SDK v3.3.8 文档 + `gdk/examples/python` 示例 + `env.sh`）与本机日志/网卡的**只读**摸底，未改动任何文件。真机行为（伺服平顺度、`joint_servo_control` 对 O10 的支持、lib 共存）以 §7 分阶段实测为准。
