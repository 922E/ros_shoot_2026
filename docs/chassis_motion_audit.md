# Chassis Motion Audit — abot_driver 底盘运动能力审查

> 审查日期: 2026-05-14
> 范围: `abot_base/abot_bringup/` 中的底盘驱动代码（只读，不修改）
> 目的: 确认当前底盘是否支持 2026 规则要求的全向移动

---

## 1. abot_driver 是否订阅 /cmd_vel

**是。**

`base_driver.cpp:60`:

```cpp
cmd_vel_sub = nh.subscribe(bdg.cmd_vel_topic, 1000, &BaseDriver::cmd_vel_callback, this);
```

`cmd_vel_topic` 参数从 `base_params.yaml:11` 加载：

```yaml
cmd_vel_topic: cmd_vel
```

注意没有前导 `/`，因此在 ROS1 中解析为节点命名空间下的 `cmd_vel`（默认命名空间 `/`，即 `/cmd_vel`）。

---

## 2. 是否读取 Twist.linear.x、Twist.linear.y、Twist.angular.z

**是，三者全部读取。**

`base_driver.cpp:151-160`:

```cpp
void BaseDriver::cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
    ROS_INFO_STREAM("cmd_vel:[" << vel_cmd.linear.x << " "
                                << vel_cmd.linear.y << " "
                                << vel_cmd.angular.z << "]");

    Data_holder::get()->velocity.v_liner_x = vel_cmd.linear.x*100;
    Data_holder::get()->velocity.v_liner_y = vel_cmd.linear.y*100;
    Data_holder::get()->velocity.v_angular_z = vel_cmd.angular.z*100;

    need_update_speed = true;
}
```

单位转换: m/s → cm/s (×100)。三个分量都存入 `Robot_velocity` 结构体。

---

## 3. linear.y 是否参与底盘速度解算

### 3.1 ROS 侧: 透传，不解算

ROS 侧（`abot_driver`）**不做**麦克纳姆运动学逆解。它只是把三自由度速度向量 (`vx, vy, ω`) 通过串口透传给 STM32。

`simple_dataframe_master.cpp:148-149`:

```cpp
case ID_SET_VELOCITY:
    send_message(id, (unsigned char*)&dh->velocity, sizeof(dh->velocity));
```

`sizeof(dh->velocity)` = 6 字节（3 × int16），包含 `v_liner_y`，不会在传输中被截断。

### 3.2 串口帧格式

`data_holder.h:39-43`:

```cpp
struct Robot_velocity{
    short v_liner_x;    // 线速度 前>0 cm/s
    short v_liner_y;    // 差分轮 为0  cm/s       ← 字段存在且被发送
    short v_angular_z;  // 角速度 左>0 0.01rad/s
};
```

### 3.3 linear.y 是否真正生效取决于 STM32 固件

`v_liner_y` 已送达 STM32，但 STM32 是否将其用于四轮速度分配，取决于固件版本。结构体注释 "差分轮 为0" 暗示早期固件可能忽略该字段。但 ROS 侧参数配置为非零：

`base_driver.cpp:138-140`:

```cpp
param->max_v_liner_x = 400;   // 非零
param->max_v_liner_y = 400;   // 非零 ← 说明固件端已配置 y 轴限幅
param->max_v_angular_z = 600;
```

**结论**: ROS → 串口 → STM32 的 **y 轴速度数据通路是完整的**。`linear.y` 从 `/cmd_vel` topic 到 STM32 固件的整个链路无障碍。

---

## 4. 当前底盘构型判断

### 4.1 URDF 模型证据

`abot_model.urdf` 定义了 **4 个独立驱动轮**:

| Joint | 位置 (xyz) | 类型 |
|---|---|---|
| `joint_left_w` | (0.120, 0.108, 0.028) | continuous (前左轮) |
| `joint_left_s` | (-0.121, 0.111, 0.028) | continuous (后左轮) |
| `joint_right_w` | (0.120, -0.109, 0.028) | continuous (前右轮) |
| `joint_right_s` | (-0.120, -0.107, 0.028) | continuous (后右轮) |

所有 4 个 joint 的旋转轴均为 `xyz="0 1 0"`（绕 Y 轴 = 车轮滚动方向），无转向节 (steering knuckle)。这与 Ackermann 构型**不符**（Ackermann 需要前轮有垂直轴转向 joint）。

### 4.2 电机数量

`base_driver.cpp:104-109` 使用 `MAX_MOTOR_COUNT=4`，PID debug 订阅 4 路 `motor1_input` ~ `motor4_input` 和 4 路 `motor1_output` ~ `motor4_output`。确认为 **4 电机独立驱动**。

### 4.3 参数结构体注释

`data_holder.h:23`:

```cpp
unsigned short wheel_track = 225; // 差分：轮距， 三全向轮：直径，四全向：前后轮距+左右轮距 mm
```

该注释显式涵盖了"四全向"（四轮麦克纳姆）的参数映射方式。

### 4.4 与 CLAUDE.md 的差异

CLAUDE.md 描述为 "Ackermann-style robot"，**与代码实际不符**。从 URDF 和驱动代码分析，真实构型更接近：

```
当前实际配置: 四轮独立驱动 (4WD independent)
             ↓
可能运动模式:
  - 差速 (skid-steer):  忽略 v_liner_y, 仅用 vx + ω
  - 麦克纳姆 (Mecanum): 使用 vx + vy + ω 全向
             ↓
ROS 侧已支持全向数据传输，取决于 STM32 固件
```

### 4.5 DWA 参数反映的运行现状

`abot_slam/params/dwa_local_planner_params_abot.yaml`:

```yaml
max_vel_y: 0        # 禁止 y 轴移动
min_vel_y: 0
acc_lim_y: 0        # 禁止 y 轴加速度
vy_samples: 1       # 仅采样 1 个 y 速度 (= 0)
# holonomic_robot: false   # 被注释，默认 false
```

当前 DWA 配置将底盘当作**差速驱动**来规划路径（`vy=0`，`holonomic=false`）。

---

## 5. 支持 2026 全向移动需要修改的文件

### 5.1 必须修改

| 文件 | 当前值 | 目标值 | 原因 |
|---|---|---|---|
| `abot_slam/params/dwa_local_planner_params_abot.yaml` | `max_vel_y: 0` | `max_vel_y: 0.25` | 允许 y 轴运动 |
| | `min_vel_y: 0` | `min_vel_y: -0.25` | 允许负向 y 轴运动 |
| | `acc_lim_y: 0` | `acc_lim_y: 1.0` | 允许 y 轴加速度 |
| | `vy_samples: 1` | `vy_samples: 10` | 采样多个 y 速度候选 |
| | `# holonomic_robot: false` | `holonomic_robot: true` | 启用全向规划 |
| `abot_slam/params/costmap_common_params_abot.yaml` | `inflation_radius: 1.2` | `0.3 ~ 0.5` | 适配 3.6m × 3.6m 场地 |

### 5.2 可能需要修改

| 文件 | 说明 |
|---|---|
| `abot_base/abot_model/urdf/abot_model.urdf` | 如更换真实麦克纳姆轮，需更新轮子 mesh 和物理参数；当前 4 轮独立布局已可用于全向 |
| Gazebo 插件 (`urdf` L496-516) | 已注释的 `skid_steer_drive` 插件，如需仿真测试全向运动，需替换为 `planar_move` 或 `mecanum_drive` 插件 |
| `robot_slam/params/carto/dwa_local_planner_params.yaml` | 已设 `holonomic_robot: true`，但需补充 `max_vel_y`、`acc_lim_y`、`vy_samples` |
| STM32 固件 | 需确认固件是否包含 Mecanum 逆运动学解算（见第 6 节） |

### 5.3 不需要修改

| 文件 | 原因 |
|---|---|
| `abot_base/abot_bringup/src/base_driver.cpp` | `cmd_vel_callback` 已正确读取 `linear.y` 并透传 |
| `abot_base/abot_bringup/src/simple_dataframe_master.cpp` | `ID_SET_VELOCITY` 已发送完整 `Robot_velocity` 结构体（含 `v_liner_y`） |
| `abot_base/abot_bringup/include/abot_bringup/data_holder.h` | `Robot_velocity` 已包含 `v_liner_y` 字段 |
| `abot_base/abot_bringup/params/base_params.yaml` | `cmd_vel_topic` 无需更改 |
| `abot_slam/params/move_base_params.yaml` | move_base 本身不依赖底盘构型 |

---

## 6. 仅修改 DWA 参数是否足够

**不够。** DWA planner 只是三层控制中的最上层：

```
  ┌──────────────────────────────────────────┐
  │ ① DWA Planner (move_base 本地规划器)      │  ← 修改: holonomic, vy
  │    生成含 vy 分量的 cmd_vel               │
  └──────────────┬───────────────────────────┘
                 │ /cmd_vel (Twist.linear.y ≠ 0)
  ┌──────────────▼───────────────────────────┐
  │ ② abot_driver (ROS 驱动节点)              │  ← 无需修改: 已透传 vy
  │    串口发送 Robot_velocity (含 v_liner_y) │
  └──────────────┬───────────────────────────┘
                 │ 串口 (6 bytes: vx, vy, ω)
  ┌──────────────▼───────────────────────────┐
  │ ③ STM32 固件 (电机控制)                   │  ← ⚠ 需要验证
  │    Mecanum IK: vx,vy,ω → 4 轮转速        │
  │    4 × PID → 4 × 电机                      │
  └──────────────────────────────────────────┘
```

必要条件:

| 条件 | 状态 |
|---|---|
| DWA 生成 vy ≠ 0 | **需修改** `dwa_local_planner_params_abot.yaml` |
| costmap 适配小场地 | **需修改** `costmap_common_params_abot.yaml` |
| abot_driver 透传 vy | **已完成** — 代码无需改动 |
| STM32 执行 Mecanum IK | **需验证** — 从 ROS 代码无法判断 |

### 验证 STM32 固件是否支持全向的方法

1. 检查 STM32 固件源码中 `case ID_SET_VELOCITY:` 的处理，看是否对 `v_liner_y` 做了四轮分配
2. 实测: 发布 `/cmd_vel` 仅含 `linear.y` (不含 `linear.x` 和 `angular.z`)，观察底盘是否纯横向移动
3. 检查 `max_v_liner_y=400` 被 `ID_SET_ROBOT_PARAMTER` 写入后固件是否存储并生效

### 风险点

- 如果 STM32 固件**未**实现 Mecanum IK，修改 DWA 参数后机器人只会在 x 轴进退 + z 轴旋转（差速模式），`linear.y` 指令会被固件忽略，不会产生横向运动
- 如果 STM32 固件**已**实现 Mecanum IK，则 ROS 侧仅需修改 DWA + costmap 参数即可启用全向移动，硬件驱动层无需任何代码改动

---

## 7. 审查总结

| 问题 | 结论 |
|---|---|
| 1. 订阅 /cmd_vel? | **是** — `base_driver.cpp:60` |
| 2. 读取 linear.x/y + angular.z? | **是** — 三者全读，`base_driver.cpp:155-157` |
| 3. linear.y 参与解算? | **ROS 侧透传**，串口帧包含 `v_liner_y`；逆运动学解算在 STM32 侧完成，状态待验证 |
| 4. 当前构型? | **四轮独立驱动**，URDF 为 Mecanum 布局，DWA 当前按差速配置运行 |
| 5. 需修改文件? | 必改 2 个: DWA params + costmap params；ROS 驱动代码无需改 |
| 6. 仅改 DWA 足够? | **不够** — 还需改 costmap + 验证 STM32 固件 Mecanum IK |
