# 2026 赛季规则变更分析报告

> 生成日期: 2026-05-14
> 分析依据: `rules_2026_claude_readable.md` + 现有工程源码
> 目标: 确定 2026 赛季需要增删的内容及可继承的已有模块

---

## 目录

- [1. 规则核心变化总结](#1-规则核心变化总结)
- [2. 可继承的已有模块](#2-可继承的已有模块)
- [3. 需要修改的模块](#3-需要修改的模块)
- [4. 需要新增的模块](#4-需要新增的模块)
- [5. 可废弃/归档的模块](#5-可废弃归档的模块)
- [6. 关键数据流 (2026)](#6-关键数据流-2026)
- [7. 建议优先读写的文件清单](#7-建议优先读写的文件清单)
- [8. 后续任务列表](#8-后续任务列表)

---

## 1. 规则核心变化总结

### 1.1 比赛流程

```
2025（现有 shoot_2025.py）          2026（新规则）
────────────────────────             ────────────────────────
语音识别 → 获取旋转/移动靶ID         语音发布任务信息 → 获取旋转/移动靶编号
导航至 6 个点（含冗余路径）           导航至 3 个任务点 → 每个点射击对应目标
射击: 环形靶(R1) → 旋转靶(R2)       射击: 任务点1→环形计分靶(20分)
       → 移动靶(R3)                        任务点2→旋转靶(15分)
后退结束                                      任务点3→移动靶(15分)
                                      最后到达终点区域(10分)
```

### 1.2 关键参数差异

| 参数 | 2025 (现有) | 2026 (规则要求) |
|---|---|---|
| 场地大小 | 约 3m × 3m (自定义) | 3.6m × 3.6m (固定) |
| 规则任务点数 | 6 个途经点 | 3 个任务点 + 1 个终点区域 |
| 工程导航点数 | 6 个 | ≥ 6 个 (含中继点，见 1.3 节) |
| 任务点尺寸 | 无明确要求 | 38cm × 32cm |
| 到达判定 | `close_enough()` (距离 < 0.03m) | 地面投影完全进入任务点 |
| 驱动方式 | Ackermann / 差速 (`max_vel_y=0`) | **麦克纳姆轮全向移动** |
| 限时 | 无严格限制 | 2 分钟总限时 |
| 启动超时 | 无 | 30s 未运动则结束 |
| 状态卡死 | 无 | 20s 状态不变可被终止 |
| 禁行区 | 无 | 有禁行区域 |
| 围挡检测 | 无 | 触碰围挡则比赛结束 |
| 语音 | 可选的启动触发 | 必须语音发布任务信息 (10分) |
| 环形靶 | 非计分 (仅"检测到有效目标") | **环形计分靶** (需计分) |
| 总分 | 自定义 | 100 分 (语音10+到达30+射击50+技术文档10) |

### 1.3 规则任务点 vs 工程导航点

**关键区分**: 规则要求的"任务点"和工程实现的"导航点"是两个不同层面的概念，不应混为一谈。

**规则层面**:
- 2026 比赛需要依次完成 **1、2、3 号任务点**，最后进入**终点区域**
- 任务点是得分触发点：到达任务点 → 射击对应目标 → 得分
- 终点区域：到达即获得 10 分
- 规则不关心机器人如何行驶到这些点，只关心是否到达并完成射击

**工程实现层面**:
- 导航目标点不应只有 3 个任务点 + 1 个终点
- 在 3.6m × 3.6m 的有限场地内，直线连接 4 个点可能遇到以下问题：
  - 路径穿过禁行区
  - 路径贴近围挡，触碰风险大（触碰围挡即比赛结束）
  - 任务点边缘行驶，转向角度过大（麦克纳姆轮虽可全向，但大角度转向仍耗时）
  - 出发/到达朝向不理想，增加瞄准调整时间
- 因此需要在任务点之间插入**中继点 (relay points)**，起到安全路径过渡作用
- 2026 的 route_points 预计为 **6 个或更多**（3 task + 1 end + N relay），具体数量取决于场地布局和禁行区位置

**三类 route point 的定义**:

| 类型 | 标识 | 目的 | 到达判定 | 触发动作 |
|---|---|---|---|---|
| **task** | 任务点 | 到达后触发射击得分 | footprint 完全进入 38cm×32cm 区域 | 射击对应目标 (环形靶/旋转靶/移动靶) |
| **relay** | 中继点 | 安全路径过渡、避障、调整航向 | 简单距离阈值 (`close_enough`) | 无，直接前往下一个点 |
| **end** | 终点 | 比赛终点区域 | 简单距离阈值 | 无，标记比赛完成 |

**三类点的处理逻辑差异**:
- **relay**: `close_enough(distance < threshold)` → 不需要射击 → 直接发布下一个导航目标。到达条件宽松，只需大致经过即可。
- **task**: `is_robot_at_task_point(footprint, zone)` → 等待导航完成 → 瞄准 → 射击 → 确认射击完成 → 前往下一个点。到达条件严格，需要 footprint 投影完全在任务点区域内。
- **end**: `close_enough(distance < threshold)` → 到达后触发 `FINISH` 状态，比赛结束，记录成绩。

---

## 2. 可继承的已有模块

以下模块可以直接复用，无需重大改动：

### 2.1 导航基础栈

| 模块 | 文件 | 说明 |
|---|---|---|
| **move_base** | `abot_slam/launch/include/move_base.launch.xml` | 标准 move_base 启动，可直接复用 |
| **amcl 定位** | `robot_slam/launch/include/amcl.launch.xml` | 自适应蒙特卡洛定位 |
| **map_server** | `navigation_shoot.launch` 等 | 加载预建地图 |
| **costmap 参数** | `abot_slam/params/costmap_common_params_abot.yaml` | 障碍物层、膨胀层需修改 (见下方须改部分) |
| **全局/局部规划器** | `global_planner_params.yaml`, `move_base_params.yaml` | 参数基本可用 |
| **URDF 模型** | `abot_model/urdf/abot_model.urdf` | 可继承，需根据麦克纳姆轮改造 |

### 2.2 射击机构

| 模块 | 文件 | 说明 |
|---|---|---|
| **串口射击控制** | `shoot_cmd/src/shoot_control.cpp` | **完全可用**。订阅 `/shoot` (String)，协议 0x55... |
| **SerialPort 库** | `shoot_cmd/src/SerialPort.cpp` | 串口通信封装，完全可用 |

### 2.3 AR 标签感知

| 模块 | 文件 | 说明 |
|---|---|---|
| **ar_track_alvar 启动** | `track_tag/launch/ar_track_camera.launch` | AR 标签检测管道，完全可用 |
| **旋转靶跟踪逻辑** | `shoot_2025.py::rotating_target()` | 逻辑可借鉴，需重构到新节点 |
| **移动靶跟踪逻辑** | `shoot_2025.py::moving_target()` | 逻辑可借鉴，需重构到新节点 |

### 2.4 语音模块

| 模块 | 文件 | 说明 |
|---|---|---|
| **FunASR 语音识别** | `2026_shoot_demo.py` | Paraformer 中文识别，**完全可用** |
| **靶位 ID 解析** | `2026_shoot_target.py` | 中文→旋转靶/移动靶编号，**完全可用** |
| **TTS 语音播报** | `robot_voice/src/tts_subscribe.cpp` | iFlytek TTS，**完全可用** |
| **TTS_audio 服务** | `TTS_audio/scripts/TTS.py` | WebSocket TTS 服务，**完全可用** |

### 2.5 VLM 视觉 (可选)

| 模块 | 文件 | 说明 |
|---|---|---|
| **Yi-Vision API 节点** | `abot_vlm/scripts/vlm_node.py` | 可作为环形靶计分的辅助手段 |

### 2.6 环形靶视觉 (HSV)

| 模块 | 文件 | 说明 |
|---|---|---|
| **HSV 颜色检测** | `abot_object_detect/node/abot_hsv_detect.py` | 可用于环形靶检测，需调参 |
| **颜色配置** | `abot_object_detect/launch/abot_ball_object.launch` | 可修改颜色阈值用于环形靶 |

---

## 3. 需要修改的模块

### 3.1 导航参数适配

**文件**: `abot_slam/params/dwa_local_planner_params_abot.yaml`

```yaml
# 当前 (Ackermann/差速)
max_vel_y: 0      # ← 无横向运动
min_vel_y: 0
acc_lim_y: 0

# 需要改为 (麦克纳姆轮全向)
max_vel_y: 0.25   # 全向移动
min_vel_y: -0.25
acc_lim_y: 1.0
holonomic_robot: true   # 新增
```

### 3.2 速度限制

**规则**: 全向运动最高速度 1 m/s

**当前**: `max_vel_x: 0.25`, `max_trans_vel: 0.35` — 远低于规则上限，可以维持或适当提高。

### 3.3 成本地图参数

**文件**: `abot_slam/params/costmap_common_params_abot.yaml`

- 机器人 footprint: `[[-0.20, -0.16], [0.16, -0.16], [0.16, 0.16], [-0.20, 0.16]]` — 与 350mm×300mm 规格基本吻合，可保留
- `inflation_radius: 1.2` — 场地仅 3.6m，此值过大，建议降低到 `0.3~0.5`

### 3.4 射击主控节点重构

**当前**: `shoot_2025.py` (flat goto 模式) 和 `shoot_2025_new.py` (初版状态机)

**需要改为**: 基于 `route_points` 配置文件的循环驱动状态机，不再写死 3 个任务点：

```
WAIT_START → VOICE_RECV → GOT_IDS → NAV_LOOP (遍历 route_points 列表)
  │
  ├─ point.type == "relay": 简单距离判定 → 下一个点
  ├─ point.type == "task":  footprint 到达判定 → 射击 → 下一个点
  └─ point.type == "end":   简单距离判定 → FINISH
```

每条边上需要加：
- **进入任务点检测**（非简单距离判断，仅 task 类型使用）
- **射击超时/重试机制**（仅 task 类型使用）
- **禁行区监控**（全程）
- **relay/end 点只做位置判近**，不触发射击和得分判定

### 3.5 到达判定

**当前**: `close_enough()` 检查欧氏距离 < 0.03m

**规则要求**: 地面投影完全进入 38cm×32cm 任务点

**需要改为**: 检查机器人 footprint 的 4 个角点是否均在任务点矩形区域内，或者检查 `base_link` 在 `map` 坐标系下的坐标是否满足 `x_min ≤ x ≤ x_max` 且 `y_min ≤ y ≤ y_max`。

### 3.6 串口设备路径

**当前**: 多处硬编码 `/dev/ttyUSB0` (shoot_2025_new.py), `/dev/shoot` (shoot_2025.py)

**建议**: 统一通过 rosparam 配置，或创建 udev 规则统一为 `/dev/shoot`

### 3.7 Python 2/3 统一

| 文件 | 当前 | 目标 |
|---|---|---|
| `shoot_2025.py` | Python 2 | Python 3 |
| `shoot_2025_new.py` | Python 2 | Python 3 |
| `2026_shoot_demo.py` | Python 3.9 (conda) | Python 3 |
| `2026_nav_end.py` | Python 3 | Python 3 ✅ |

---

## 4. 需要新增的模块

### 4.1 比赛状态机节点 (高优先级)

**建议文件**: `robot_slam/scripts/competition_control.py` 或 `robot_slam/src/competition_control.cpp`

**功能**:
- 从 `competition_2026_route.yaml` 加载 `route_points` 列表
- 遍历 route_points，根据 `type` 字段执行不同逻辑:
  - **relay**: 发布导航目标 → 简单判近 → 下一个点
  - **task**: 发布导航目标 → footprint 到达判定 → 根据 `target_type` 触发射击 → 下一个点
  - **end**: 发布导航目标 → 判近 → FINISH
- 通过 actionlib 与 move_base 交互
- 错误恢复 (重试、超时跳过)
- 不写死导航点数量，route_points 可任意增减

### 4.2 比赛监控节点 (高优先级)

**建议文件**: `robot_slam/scripts/competition_monitor.py`

**功能**:
- **2 分钟倒计时** — 全局计时，超时触发比赛结束
- **30s 启动检测** — 从裁判宣布开始计时，30s 内必须有速度指令
- **20s 状态卡死检测** — 如果状态机超过 20s 未切换状态，触发超时处理
- **围挡触碰检测** — 监控机器人坐标，超出场地边界 (1.8m × 1.8m) 时停止
- **禁行区监控** — 预定义禁行区域，机器人接近时告警

### 4.3 禁行区域配置 (中优先级)

**建议文件**: `robot_slam/config/forbidden_zones.yaml`

**格式示例**:
```yaml
forbidden_zones:
  - name: "zone_A"
    polygon: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]
  - name: "zone_B"
    polygon: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]
```

集成到 costmap 的 `static_layer` 或通过 `costmap_2d::VirtualWall` 插件实现。

### 4.4 任务点到达判定节点 (中优先级)

**建议文件**: `robot_slam/scripts/arrival_detector.py` 或在状态机中内联实现

**判定逻辑**:
```python
def is_robot_at_task_point(robot_x, robot_y, robot_yaw,
                           task_point_center_x, task_point_center_y,
                           task_point_size=(0.38, 0.32)):
    # 机器人 footprint 4 角点坐标变换
    # 检查所有角点是否在任务点矩形内
    # 或：检查机器人中心坐标是否在缩小的任务点区域内 (保守判定)
    half_w, half_h = task_point_size[0]/2, task_point_size[1]/2
    # 用 base_link footprint 4 个角做旋转变换后判断
    ...
```

### 4.5 环形计分靶识别 (中优先级)

**建议**: 在现有 `abot_object_detect` 的 HSV 检测基础上，增加**环数识别**能力。
- 方案 A: OpenCV 轮廓检测 → 计算命中环数
- 方案 B: VLM (Yi-Vision) 拍照识别环数

### 4.6 比赛 Launch 文件 (中优先级)

**建议文件**: `robot_slam/launch/competition_2026.launch`

```xml
<launch>
  <!-- 底盘 -->
  <include file="$(find abot_bringup)/launch/robot_with_imu.launch"/>
  <!-- 导航 -->
  <include file="$(find robot_slam)/launch/navigation_shoot.launch"/>
  <!-- 摄像头 -->
  <include file="$(find robot_slam)/launch/camera_my.launch"/>
  <!-- AR 标签跟踪 -->
  <include file="$(find track_tag)/launch/ar_track_camera.launch"/>
  <!-- 射击控制 -->
  <node name="shoot_control" pkg="shoot_cmd" type="shoot_control"/>
  <!-- 语音识别 -->
  <node name="audio_subscriber" pkg="robot_slam" type="2026_shoot_demo.py"/>
  <!-- 靶位 ID 解析 -->
  <node name="target_id_parser" pkg="robot_slam" type="2026_shoot_target.py"/>
  <!-- 比赛主控 (新) -->
  <node name="competition_control" pkg="robot_slam" type="competition_control.py"/>
  <!-- 比赛监控 (新) -->
  <node name="competition_monitor" pkg="robot_slam" type="competition_monitor.py"/>
</launch>
```

### 4.7 技术文档 (低优先级)

**规则要求**: 10 分技术文档/答辩

**需要准备**:
- `docs/hardware_spec.md` — 硬件参数 (主控制器、传感器、电机、电池等)
- `docs/software_architecture.md` — 软件架构和方案
- `docs/system_diagram.md` — 系统框图

资料可参考 `CLAUDE.md` 和 `project_map.md` 已有内容。

### 4.8 比赛路线配置文件 (高优先级)

**建议文件**: `robot_slam/config/competition_2026_route.yaml`

路由配置文件定义比赛用到的所有导航点，包括任务点、中继点、终点。状态机按顺序遍历该列表。

**格式示例**:

```yaml
# competition_2026_route.yaml — 2026 赛季导航路线配置
# route_points 按比赛顺序排列，状态机逐一遍历

route_points:
  # ── 起点 (隐含) ──
  - id: 0
    name: "起点"
    type: "relay"         # 起点视为中继点，不做射击
    x: 0.0
    y: 0.0
    yaw: 0.0              # 期望朝向 (rad)

  # ── 中继点 A: 从起点向左前方过渡，避开右侧禁行区 ──
  - id: 1
    name: "中继点A"
    type: "relay"
    x: 0.5
    y: -0.3
    yaw: 1.57

  # ── 任务点 1: 环形计分靶 (20分) ──
  - id: 2
    name: "任务点1-环形靶"
    type: "task"
    target_type: "circular"       # 射击目标类型: circular / rotating / moving
    x: 0.8
    y: 0.8
    yaw: 0.0
    task_zone:                    # 38cm × 32cm 任务点区域 (map 坐标系)
      x_min: 0.61
      x_max: 0.99
      y_min: 0.64
      y_max: 0.96

  # ── 中继点 B: 任务点1 → 任务点2 的过渡 ──
  - id: 3
    name: "中继点B"
    type: "relay"
    x: 1.3
    y: 0.3
    yaw: 0.0

  # ── 任务点 2: 旋转靶 (15分) ──
  - id: 4
    name: "任务点2-旋转靶"
    type: "task"
    target_type: "rotating"
    x: 1.5
    y: -0.8
    yaw: 3.14
    task_zone:
      x_min: 1.31
      x_max: 1.69
      y_min: -0.96
      y_max: -0.64

  # ── 中继点 C: 任务点2 → 任务点3 的过渡 ──
  - id: 5
    name: "中继点C"
    type: "relay"
    x: 0.8
    y: -1.2
    yaw: -1.57

  # ── 任务点 3: 移动靶 (15分) ──
  - id: 6
    name: "任务点3-移动靶"
    type: "task"
    target_type: "moving"
    x: -0.6
    y: -0.8
    yaw: 3.14
    task_zone:
      x_min: -0.79
      x_max: -0.41
      y_min: -0.96
      y_max: -0.64

  # ── 中继点 D: 任务点3 → 终点的过渡 ──
  - id: 7
    name: "中继点D"
    type: "relay"
    x: -0.3
    y: 0.0
    yaw: 1.57

  # ── 终点区域 ──
  - id: 8
    name: "终点"
    type: "end"
    x: 0.0
    y: 1.5
    yaw: 0.0

# 全局参数
global:
  relay_close_threshold: 0.10    # 中继点判近距离 (m)
  end_close_threshold: 0.15      # 终点判近距离 (m)
  task_timeout: 60.0             # 单个任务点超时 (s)
  shoot_timeout: 15.0            # 单次射击超时 (s)
```

**字段说明**:

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | int | 序号，按遍历顺序递增 |
| `name` | string | 点名称 (用于日志和调试) |
| `type` | string | `relay` / `task` / `end` |
| `target_type` | string | 仅 task 类型有效: `circular` / `rotating` / `moving` |
| `x, y` | float | map 坐标系下的坐标 (m) |
| `yaw` | float | 期望到达朝向 (rad) |
| `task_zone` | dict | 仅 task 类型有效: 38cm×32cm 任务点边界 |

**设计原则**:
- 中继点的坐标和数量可以根据实际场地地图灵活调整
- task_zone 边界需根据实际场地标定确定，不能硬编码
- 增加或减少导航点只需修改 YAML 文件，不需要改动状态机代码
- relay 点可以密集分布以避免大角度转向和贴边行驶

---

## 5. 可废弃/归档的模块

以下模块与 2026 赛季无关，建议归档而非删除（可能对后续开发有参考价值）：

| 模块 | 理由 |
|---|---|
| `opencv_demo/` 全部 | 仅 OpenCV 示例，非比赛逻辑 |
| `hector_slam/` | 使用 gmapping/cartographer，hector 不再需要 |
| `voice_demo/` | 测试用 |
| `color_pkg/fire_detector.py` | 火焰检测与比赛无关 |
| `color_pkg/line_follower.py` | 巡线与比赛无关 |
| `robot_slam/scripts/demo.py` | 旧测试脚本 |
| `robot_slam/scripts/fa.py, shan.py, shou.py` | 旧测试脚本 |
| `robot_slam/scripts/4x5_demo.py` | 4x5 场地测试脚本 |

**可以保留但不需维护**:
- `abot_find/` (find_object_2d) — 第三方库，可保留但不作为主要识别方案
- `user_demo/` — 示例代码
- 历年 `multi_goal_20XX.launch` — 可归档但保留作参考

---

## 6. 关键数据流 (2026)

```
 ┌──────────────────────────────────────────────────────────────────┐
 │                   比赛主控 (competition_control.py)               │
 │  ┌──────────┐ ┌───────────┐ ┌──────────────────────────────────┐ │
 │  │ WAIT_    │→│ VOICE_    │→│  NAV_LOOP (遍历 route_points)    │ │
 │  │ START    │ │ RECV      │ │  relay → 导航 → close_enough → 下一个│
 │  └──────────┘ └───────────┘ │  task  → 导航 → footprint判定 → 射击│
 │                             │  end   → 导航 → close_enough → FINISH│
 │                             └──────────────────────────────────┘ │
 ───────────────────────────────────────────────────────────────────┘
                                     │
                    ┌────────────────┼────────────────┐
                    ▼                ▼                ▼
              ┌──────────┐   ┌──────────┐   ┌──────────────┐
              │abot_obj_ │   │ar_track_ │   │ ar_track_    │
              │_detect   │   │alvar     │   │ alvar        │
              │(HSV)     │   │(旋转靶)   │   │ (移动靶)     │
              └────┬─────┘   └────┬─────┘   └──────┬───────┘
                   │              │                 │
                   ▼              ▼                 ▼
              ┌──────────┐ ┌──────────┐   ┌──────────────┐
              │/object_  │ │/ar_pose_ │   │ /ar_pose_    │
              │position  │ │marker    │   │ marker       │
              └──────────┘ └──────────┘   └──────────────┘
                   │              │                 │
                   └──────────────┼─────────────────┘
                                  │
                    ┌─────────────┴──────────────┐
                    │  competition_control        │
                    │  (根据 target_type 选择     │
                    │   对应感知管道)              │
                    └─────────────┬──────────────┘
                                  ▼
                   ┌─────────────────────────────────┐
                   │         /shoot (String)          │
                   └──────────────┬───────────────────┘
                                  ▼
                   ┌─────────────────────────────────┐
                   │      shoot_control (串口)        │
                   │     0x55 0x01 0x12 ...          │
                   └─────────────────────────────────┘

  ┌────────────────────────────────────────────────────────┐
  │ 比赛监控 (competition_monitor.py) — 并行运行           │
  │ • 2分钟倒计时                                          │
  │ • 30s 启动检测                                         │
  │ • 20s 状态卡死检测                                     │
  │ • 围挡坐标检测 (|x|>1.8 || |y|>1.8)                   │
  │ • 禁行区域检测                                         │
  └────────────────────────────────────────────────────────┘
```

---

## 7. 建议优先读写的文件清单

### 7.1 优先阅读 (了解现有逻辑)

| 优先级 | 文件 | 原因 |
|---|---|---|
| P0 | `robot_slam/scripts/shoot_2025.py` | 2025 年射击比赛主逻辑 |
| P0 | `robot_slam/scripts/shoot_2025_new.py` | 已初步重构的状态机版本 |
| P0 | `robot_slam/scripts/2026_shoot_target.py` | 语音靶位 ID 解析 |
| P0 | `robot_slam/scripts/2026_shoot_demo.py` | FunASR 语音识别 |
| P0 | `abot_slam/scripts/mission.py` | navigation_demo 类 (actionlib client) |
| P1 | `track_tag/src/ar_track.cpp` | AR 标签跟踪 P 控制器 |
| P1 | `shoot_cmd/src/shoot_control.cpp` | 串口射击协议 |
| P1 | `abot_slam/params/*.yaml` | 导航参数 |
| P1 | `abot_object_detect/node/abot_hsv_detect.py` | HSV 环形靶检测 |
| P2 | `abot_vlm/scripts/vlm_node.py` | VLM 辅助识别 |

### 7.2 优先创建/修改 (按顺序)

| 优先级 | 文件 | 操作 | 原因 |
|---|---|---|---|
| **P0** | `competition_control.py` | **新建** | 比赛状态机主控 |
| **P0** | `competition_monitor.py` | **新建** | 计时/边界/卡死监控 |
| **P1** | `dwa_local_planner_params_abot.yaml` | **修改** | 全向移动参数 |
| **P1** | `costmap_common_params_abot.yaml` | **修改** | 调小膨胀半径，适配 3.6m 场地 |
| **P1** | `competition_2026.launch` | **新建** | 一键启动 |
| **P2** | `arrival_detector.py` | **新建** | 精确到达判定 |
| **P2** | `forbidden_zones.yaml` | **新建** | 禁行区域配置 |
| **P2** | `docs/hardware_spec.md` | **新建** | 技术文档 |

---

## 8. 后续任务列表

### Phase 1: 基础设施修改 (1-2 天)

- [ ] 修改 DWA planner 参数启用全向移动 (`holonomic_robot: true`, `max_vel_y` 等)
- [ ] 调整 costmap 膨胀半径适配 3.6m 场地
- [ ] 创建 3.6m × 3.6m 场地地图 (或修改现有 `shoot.yaml` 地图)
- [ ] **设计中继点和安全路径**: 基于场地地图规划中继点位置，避开禁行区、围挡，优化转向角度和大角度转向
- [ ] 创建 `competition_2026_route.yaml`，标定所有 route_points (含 task_zone 边界)
- [ ] 标定并配置 1-3 号任务点及终点坐标

### Phase 2: 比赛主控 (2-3 天)

- [ ] 新建 `competition_control.py` 状态机节点
- [ ] 实现环形靶瞄准射击逻辑 (从 `shoot_2025.py::circular_target()` 迁移)
- [ ] 实现旋转靶瞄准射击逻辑 (从 `shoot_2025.py::rotating_target()` 迁移)
- [ ] 实现移动靶瞄准射击逻辑 (从 `shoot_2025.py::moving_target()` 迁移)
- [ ] 集成语音结果 → 靶位 ID → 状态机流程

### Phase 3: 比赛监控 (1 天)

- [ ] 新建 `competition_monitor.py` 节点
- [ ] 实现 2 分钟倒计时
- [ ] 实现 30s 启动检测
- [ ] 实现 20s 状态卡死检测
- [ ] 实现围挡/边界检测
- [ ] 实现禁行区域检测

### Phase 4: 感知调优 (2-3 天)

- [ ] 调优 HSV 参数用于环形靶检测
- [ ] 测试 AR 标签在 1.5m 距离的识别稳定性
- [ ] 测试射击精度和重复性
- [ ] 可选项: VLM 辅助环形靶计分

### Phase 5: 集成测试 (2 天)

- [ ] 新建 `competition_2026.launch` 整合所有节点
- [ ] 模拟比赛流程测试
- [ ] 边界条件测试 (超时、卡死、围挡触碰等)
- [ ] 两次比赛取最高分机制 (日志)

### Phase 6: 文档 (1 天)

- [ ] 硬件参数文档 (`docs/hardware_spec.md`)
- [ ] 软件架构文档
- [ ] 系统框图
- [ ] 更新 `CLAUDE.md` 添加 2026 规则摘要
