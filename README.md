# ROS2 Nav2 TTC Dynamic Obstacle Avoidance

基于 **ROS2 Humble + Nav2** 的移动机器人自主导航与动态障碍物避障系统。  
项目面向移动机器人在**窄通道、交叉口、动态障碍物横穿、局部碰撞风险**等复杂场景下的安全通行问题，在 Nav2 原有导航链路基础上，扩展了动态障碍物提取、TTC 风险判断、速度门控控制与最终安全保护机制。

---

## 项目简介

本项目基于 **ROS2 Humble** 与 **Nav2** 构建移动机器人自主导航优化系统，完成了 Gazebo 仿真环境、FishBot 差速底盘、AMCL 定位、SmacPlanner2D 全局规划、DWB 局部控制、global/local costmap 配置与 RViz 可视化验证。

针对 Nav2 在实际导航中容易出现的：

- 窄通道无法稳定进入
- 动态障碍物横穿时避让不及时
- 交叉口场景下缺少提前风险判断
- 多个速度源同时控制底盘导致控制链路混乱
- 静态障碍物与动态障碍物混在同一激光数据中，导致避障策略不够精确

等问题，本项目在 Nav2 原有导航链路基础上，自研设计了：

- `dynamic_scan_filter.py`
- `intersection_supervisor.py`
- `intersection_speed_gate.py`

实现了动态障碍物提取、TTC / 相对 TTC 风险判断、交叉口让行决策、速度门控控制，并结合 `collision_monitor` 构建最终安全保护层。

最终系统实现了机器人在动态障碍物靠近时减速、预测存在碰撞风险时停车等待、障碍物离开后恢复通行，以及底盘只接收最终安全速度 `/cmd_vel_safe` 的完整闭环。

---

## 项目亮点

- 基于 ROS2 Humble + Nav2 搭建完整移动机器人自主导航系统
- 使用 AMCL 完成机器人定位
- 使用 SmacPlanner2D 完成全局路径规划
- 使用 DWB 完成局部轨迹跟踪与控制
- 通过 costmap 膨胀层与 DWB critic 权重优化窄通道通行能力
- 基于静态地图过滤激光雷达数据，提取动态障碍物
- 引入 TTC 思想判断动态障碍物是否会进入机器人通行区域
- 引入相对 TTC 判断机器人与动态障碍物之间是否正在靠近
- 设计交叉口监管节点，实现动态障碍物风险决策
- 设计速度门控节点，实现正常行驶、减速、停车等待和恢复通行
- 重新设计速度控制链路，使底盘只接收最终安全速度 `/cmd_vel_safe`
- 使用 `collision_monitor` 作为最后一层安全保护
- 区分“机器人尚未进入冲突区”和“机器人已经进入冲突区”两种情况，避免中途急停带来的二次风险

---

## 系统架构

```text
Gazebo 仿真环境
        ↓
FishBot 差速移动机器人
        ↓
/scan, /odom, /tf
        ↓
Nav2 导航系统
        ↓
dynamic_scan_filter
        ↓
intersection_supervisor
        ↓
intersection_speed_gate
        ↓
collision_monitor
        ↓
fishbot_diff_drive_controller
```

系统主要分为以下几层：

```text
感知层：
    激光雷达 / TF / 静态地图

动态障碍物提取层：
    dynamic_scan_filter

风险决策层：
    intersection_supervisor

速度门控层：
    intersection_speed_gate

安全保护层：
    collision_monitor

执行层：
    fishbot_diff_drive_controller
```

---

## 速度控制链路设计

在普通 Nav2 系统中，`velocity_smoother` 通常直接发布 `/cmd_vel` 给底盘控制器。

本项目没有让 `/cmd_vel` 直接控制底盘，而是重新设计了多级速度链路：

```text
Nav2 controller_server / behavior_server
        ↓
/cmd_vel_raw

velocity_smoother
        ↓
/cmd_vel

intersection_speed_gate
        ↓
/cmd_vel_risk

collision_monitor
        ↓
/cmd_vel_safe

fishbot_diff_drive_controller
```

各速度话题含义如下：

| 话题 | 含义 |
|---|---|
| `/cmd_vel_raw` | Nav2 controller_server / behavior_server 输出的原始速度 |
| `/cmd_vel` | velocity_smoother 输出的平滑速度 |
| `/cmd_vel_risk` | intersection_speed_gate 根据 TTC 风险门控后的速度 |
| `/cmd_vel_safe` | collision_monitor 输出的最终安全速度 |

底盘控制器最终只订阅：

```text
/cmd_vel_safe
```

这样可以避免多个节点同时发布 `/cmd_vel` 导致速度控制混乱，也可以保证动态避障、TTC 风险判断和安全保护模块真正作用到底盘执行层。

---

## 自研节点说明

### 1. dynamic_scan_filter.py

`dynamic_scan_filter.py` 用于从普通激光雷达数据中提取动态障碍物点。

处理流程：

```text
/scan + /map + TF
        ↓
将激光点转换到 map 坐标系
        ↓
利用静态地图过滤墙体和固定障碍物
        ↓
保留可能属于动态障碍物的点
        ↓
发布 /dynamic_scan
```

该节点的核心作用是将静态环境和动态障碍物进行区分，使后续的 `intersection_supervisor` 与 `collision_monitor` 主要关注动态障碍物，而不是墙体、地图边界等静态结构。

---

### 2. intersection_supervisor.py

`intersection_supervisor.py` 是动态障碍物风险判断节点。

主要功能包括：

- 订阅 `/dynamic_scan`
- 获取机器人当前位姿
- 判断机器人是否接近交叉口或冲突区域
- 判断动态障碍物是否正在靠近机器人通行路径
- 判断动态障碍物是否已经通过机器人必经路径
- 计算 TTC / 相对 TTC 风险
- 根据风险状态输出通行、减速、停车或恢复决策
- 发布 `/intersection_decision`

输出决策状态如下：

| 决策状态 | 含义 |
|---|---|
| `NORMAL_NAVIGATION` | 正常导航 |
| `SLOW_DOWN` | 动态障碍物存在潜在风险，机器人减速观察 |
| `YIELD_WAIT` | 预测存在碰撞风险，机器人停车等待 |
| `PASS_INTERSECTION` | 判断可以通过，机器人继续行驶 |
| `RECOVERY` | 长时间等待后的恢复状态 |

---

### 3. intersection_speed_gate.py

`intersection_speed_gate.py` 是速度门控节点。

主要功能包括：

- 订阅 `/cmd_vel`
- 订阅 `/intersection_decision`
- 根据 supervisor 的决策对速度进行门控
- 发布 `/cmd_vel_risk`

控制逻辑如下：

```text
NORMAL_NAVIGATION / PASS_INTERSECTION:
    原样输出速度

SLOW_DOWN:
    按比例降低线速度和角速度

YIELD_WAIT / RECOVERY:
    输出 0 速度
```

该节点的作用是将上层风险决策真正作用到底盘速度链路中，使机器人能够根据动态障碍物风险主动减速、停车或恢复通行。

---

### 4. collision_monitor

`collision_monitor` 作为最后一层安全保护，订阅 `/cmd_vel_risk` 和 `/dynamic_scan`，输出最终安全速度 `/cmd_vel_safe`。

在本项目中，各模块分工如下：

```text
intersection_supervisor:
    负责提前判断动态障碍物风险

intersection_speed_gate:
    负责根据风险决策修改机器人速度

collision_monitor:
    负责近距离风险下的最终安全裁决
```

`collision_monitor` 不负责复杂决策，而是作为兜底安全层，在极近距离风险下进行减速或停车。

---

## TTC 风险判断思想

TTC，即 **Time To Collision**，表示预计发生碰撞所需的时间。

本项目中主要使用两类 TTC 思想。

---

### 1. 动态障碍物到机器人通行路径的 TTC

```text
TTC_path = distance_to_robot_path / obstacle_velocity_to_path
```

当动态障碍物预计在较短时间内进入机器人通行区域时，机器人减速或停车等待。

该判断主要用于解决动态障碍物横穿机器人前方路径的问题。

---

### 2. 机器人与动态障碍物之间的相对 TTC

```text
relative_position = obstacle_position - robot_position

relative_velocity = obstacle_velocity - robot_velocity
```

系统进一步判断机器人和动态障碍物之间的距离是否正在缩小。

如果动态障碍物虽然正在远离原路径，但机器人自身正在斜向靠近障碍物，系统不会直接放行，而是继续减速或停车等待。

核心思想如下：

```text
动态障碍物远离路径线 ≠ 一定安全

只有同时满足：
    动态障碍物已经通过机器人必经路径
    且动态障碍物与机器人之间的距离正在增大

才认为机器人可以放心通过
```

---

## 通过承诺区设计

在动态障碍物横穿场景中，如果机器人已经进入动态障碍物运动区域，不应该突然停车。  
因为机器人停在冲突区域中，可能反而增加被动态障碍物撞击的风险。

因此本项目设计了通过承诺区逻辑：

```text
机器人尚未进入动态障碍物运动区域：
    可以减速或停车等待

机器人运动过程中已经进入动态障碍物运动区域：
    进入 PASS_INTERSECTION，继续通过

机器人启动时已经位于动态障碍物运动区域：
    先进行相对 TTC 判断，确认安全后再通过
```

该设计用于减少机器人在冲突区域中途急停导致的二次风险。

---

## 窄通道优化

本项目针对 Nav2 在窄通道中容易出现的无法进入、路径贴边、局部控制过于保守等问题，对 costmap 和 DWB 参数进行了优化。

主要涉及模块如下：

```text
global_costmap
local_costmap
inflation_layer
robot_radius / footprint
DWB critic 权重
路径平滑参数
```

重点参数包括：

```text
inflation_radius
cost_scaling_factor
PathDist.scale
PathAlign.scale
BaseObstacle.scale
GoalAlign.scale
```

优化思路：

- 减小过大的膨胀层，避免窄通道被虚拟障碍完全堵死
- 根据机器人实际尺寸合理设置 `robot_radius` 或 `footprint`
- 调整 DWB 中路径跟踪和障碍物代价权重
- 在安全距离和通行能力之间取得平衡
- 使用路径平滑降低局部控制抖动
- 提高机器人在狭窄区域中的稳定通行能力

---

## 项目结构

当前项目结构示例：

```text
ros2-nav2-ttc-dynamic-avoidance/
├── src/
│   ├── fishbot_description/
│   │   ├── urdf/
│   │   ├── world/
│   │   ├── launch/
│   │   └── config/
│   │
│   ├── fishbot_navigation2/
│   │   ├── launch/
│   │   │   └── navigation2.launch.py
│   │   ├── config/
│   │   │   └── nav2_params.yaml
│   │   ├── maps/
│   │   ├── scripts/
│   │   │   ├── dynamic_scan_filter.py
│   │   │   ├── intersection_supervisor.py
│   │   │   └── intersection_speed_gate.py
│   │   └── rviz/
│   │
│   └── moving_obstacle_plugin/
│       ├── src/
│       └── CMakeLists.txt
│
├── README.md
└── .gitignore
```

---

## 环境依赖

- Ubuntu 22.04
- ROS2 Humble
- Nav2
- Gazebo Classic
- RViz2
- Python 3
- CMake / colcon

---

## 编译方法

进入工作空间：

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

如果只编译导航包：

```bash
colcon build --packages-select fishbot_navigation2 --symlink-install
source install/setup.bash
```

---

## 启动方法

### 1. 启动 Gazebo 仿真环境

```bash
ros2 launch fishbot_description gazebo_sim.launch.py
```

### 2. 启动 Nav2 与动态避障系统

```bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

---

## 关键话题检查

### 1. 检查自研节点是否启动

```bash
ros2 node list | grep dynamic
ros2 node list | grep intersection
ros2 node list | grep collision
```

期望看到：

```text
/dynamic_scan_filter
/intersection_supervisor
/intersection_speed_gate
/collision_monitor
```

---

### 2. 检查速度链路

```bash
ros2 topic info /cmd_vel_raw -v
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_risk -v
ros2 topic info /cmd_vel_safe -v
```

期望链路如下：

```text
/cmd_vel_raw:
    publisher: controller_server / behavior_server
    subscriber: velocity_smoother

/cmd_vel:
    publisher: velocity_smoother
    subscriber: intersection_speed_gate

/cmd_vel_risk:
    publisher: intersection_speed_gate
    subscriber: collision_monitor

/cmd_vel_safe:
    publisher: collision_monitor
    subscriber: fishbot_diff_drive_controller
```

---

### 3. 检查动态避障决策

```bash
ros2 topic echo /intersection_decision
```

可能输出：

```text
data: "NORMAL_NAVIGATION"
data: "SLOW_DOWN"
data: "YIELD_WAIT"
data: "PASS_INTERSECTION"
data: "RECOVERY"
```

---

## 实验场景

### 1. 基础导航实验

验证机器人能够在 Gazebo 环境中完成定位、全局路径规划、局部控制和目标点到达。

---

### 2. 窄通道通行实验

通过调整 costmap inflation layer 和 DWB critic 权重，使机器人可以稳定进入窄通道。

---

### 3. 动态障碍物减速实验

当动态障碍物靠近机器人通行区域时，`intersection_supervisor` 输出：

```text
SLOW_DOWN
```

`intersection_speed_gate` 降低机器人速度。

---

### 4. TTC 停车等待实验

当系统预测机器人与动态障碍物存在碰撞风险时，`intersection_supervisor` 输出：

```text
YIELD_WAIT
```

机器人停车等待。

---

### 5. 障碍物离开后恢复通行实验

当动态障碍物远离机器人并通过必经路径后，系统输出：

```text
PASS_INTERSECTION
```

机器人恢复运动。

---

## 项目效果

本项目实现了以下效果：

- 机器人能够完成基础 Nav2 自主导航
- 机器人能够通过窄通道
- 动态障碍物靠近时机器人能够减速
- TTC 判断存在风险时机器人能够停车等待
- 动态障碍物离开后机器人能够恢复前进
- 底盘只接收最终安全速度 `/cmd_vel_safe`
- 动态避障、速度门控和安全保护形成完整闭环

---

## 实验记录建议

后续可以进一步补充定量实验记录，例如：

| 实验组 | 动态障碍物判断 | 速度门控 | collision_monitor | 是否发生碰撞 | 是否成功通过 |
|---|---|---|---|---|---|
| 原始 Nav2 | 否 | 否 | 否 | 待补充 | 待补充 |
| Nav2 + 参数优化 | 否 | 否 | 否 | 待补充 | 待补充 |
| Nav2 + TTC 速度门控 | 是 | 是 | 否 | 待补充 | 待补充 |
| Nav2 + TTC + collision_monitor | 是 | 是 | 是 | 待补充 | 待补充 |

建议后续增加：

- 通行成功率
- 碰撞次数
- 最小安全距离
- 平均通行时间
- 停车等待次数
- 动态障碍物触发次数

---

## 后续优化方向

当前系统主要面向结构化动态障碍物横穿场景，适用于单个或少量规则运动障碍物的风险判断。

后续可以进一步优化：

- 引入多目标动态障碍物跟踪
- 增加更完整的动态障碍物轨迹预测模块
- 将动态障碍物预测结果接入局部规划器
- 对比 DWB、RPP、MPPI 在动态障碍物场景下的表现
- 在真实机器人平台上验证参数鲁棒性
- 增加 rosbag 实验记录和定量评估指标
- 将动态障碍物预测结果与局部轨迹采样代价进一步融合
- 增加 RViz Marker 可视化动态障碍物风险区域
- 增加 C++ 版本核心算法模块，提高工程实时性

---

## 适用方向

本项目适合作为以下方向的学习和求职展示项目：

- 移动机器人路径规划
- ROS2 / Nav2 工程开发
- 动态障碍物避障
- 局部规划与速度控制
- 移动机器人系统集成
- 机器人导航系统调参和问题排查

---

## 简历项目描述参考

基于 ROS2 Humble 与 Nav2 搭建移动机器人自主导航系统，完成 AMCL 定位、SmacPlanner2D 全局规划、DWB 局部控制、global/local costmap 配置与 Gazebo/RViz 仿真验证。针对窄通道无法稳定通行、动态障碍物横穿避让不及时、交叉口风险判断不足等问题，设计 `dynamic_scan_filter`、`intersection_supervisor` 与 `intersection_speed_gate` 节点，实现动态障碍物提取、TTC/相对 TTC 风险预测和速度门控控制。重新设计 `/cmd_vel_raw → /cmd_vel → /cmd_vel_risk → /cmd_vel_safe` 多级速度链路，使底盘只接收最终安全速度，提高动态避障场景下的通行稳定性与安全性。

---

## 项目说明

本项目主要用于 ROS2 Nav2 移动机器人导航、动态障碍物避障和路径规划方向的学习、实验验证与求职展示。

项目重点不只是调用 Nav2，而是在 Nav2 原有导航链路基础上，围绕动态障碍物、交叉口让行、窄通道通行和速度安全控制进行了系统级扩展。
