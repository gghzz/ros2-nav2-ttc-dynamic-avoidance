#ROS2 Nav2 TTC 动态障碍物
## 项目简介

本项目基于 **ROS2 Humble** 与 **Nav2** 构建移动机器人自主导航优化系统，完成 Gazebo 仿真环境、FishBot 差速底盘、AMCL 定位、SmacPlanner2D 全局规划、DWB 局部控制、global/local costmap 配置与 RViz 可视化验证。

针对**等问题，本项目在Nav2原有导航链路基础上，设计了dynamic_scan_filter`、intersection_supervisor和intersection_speed_gate节点，实现动态障碍物提取、TTC/相对TTC风险判断、速度门控控制，并结合collision_monitor构建最终安全保护层。

项目最终实现了机器人在动态障碍物靠近时减速、预测存在碰撞风险时停车等待、障碍物离开后恢复通行，以及底盘只接收最终安全速度 `/cmd_vel_safe` 的完整闭环。

---

## 项目特点

- 基于 ROS2 Humble + Nav2 搭建完整自主导航系统
- 使用 AMCL 完成机器人定位
- 使用 SmacPlanner2D 完成全局路径规划
- 使用 DWB 完成局部轨迹跟踪
-通过costmap膨胀层与DWB评判器参数优化窄通道通行能力
- 基于静态地图过滤激光雷达点云，提取动态障碍物
- 引入 TTC 思想判断动态障碍物是否会进入机器人通行区域
- 引入相对 TTC 判断机器人与动态障碍物之间是否正在靠近
- 设计速度门控节点，实现正常行驶、减速、停车等待和恢复通行
- 重新设计速度链路，使底盘只接收最终安全速度 `/cmd_vel_safe`
- 使用 `collision_monitor` 作为最后一层安全保护

---

## 系统架构

```text
凉亭仿真环境
        ↓
FishBot 差速移动机器人
        ↓
/scan, /odom, /tf
        ↓
Nav2 导航系统
        ↓
动态扫描滤波器
        ↓
交叉路口监管
        ↓
交叉路口限速门
        ↓
碰撞监测
        ↓
鱼形机器人差速控制器
```

系统主要分为以下几层：

```text
感知层：
激光扫描 / TF / 地图

动态障碍物提取层：
动态扫描滤波器

决策层：
交叉路口监管

速度门控层：
交叉路口限速门

安全保护层：
碰撞监测

执行层：
鱼形机器人差速控制器
```

---

## 速度控制链路

在普通Nav2系统中，`velocity_smoother`通常直接发布`/cmd_vel`给底盘控制器。

本项目没有让 `/cmd_vel` 直接控制底盘，而是设计了多级速度链路：

```文本
Nav2 控制器_服务器 / 行为_服务器
        ↓
/cmd_vel_raw

速度平滑器
        ↓
/cmd_vel

交叉路口限速门
        ↓
/cmd_vel_risk

碰撞监测
        ↓
/cmd_vel_safe

鱼形机器人差速控制器
```

各速度话题含义如下：

| 话题 | 含义 |
|---|---|
| `/cmd_vel_raw` | Nav2 controller_server / behavior_server 输出的原始速度 |
| `/cmd_vel` | velocity_smoother 输出的平滑速度 |
| `/cmd_vel_risk` | intersection_speed_gate 根据 TTC 风险门控后的速度 |
| `/cmd_vel_safe` |碰撞监测器输出的最终安全速度|

底盘控制器最终只订阅：

```text
/cmd_vel_safe
```

这样可以避免多个节点同时发布 `/cmd_vel` 导致速度控制混乱，也可以保证动态避障和安全保护模块真正作用到底盘速度。

---

## 自研节点说明

### 1. dynamic_scan_filter.py

`dynamic_scan_filter.py` 用于从普通激光雷达数据中提取动态障碍物点。

处理流程：

```text
/scan + /map + TF
        ↓
将激光点转换到地图坐标系
        ↓
利用静态地图过滤墙体和固定障碍物
        ↓
保留可能属于动态障碍物的点
        ↓
发布 /dynamic_scan
```

该节点的作用是将静态环境和动态障碍物进行区分，使后续的 supervisor 与 collision_monitor 主要关注动态障碍物，而不是墙体、地图边界等静态结构。

---

### 2. intersection_supervisor.py

`intersection_supervisor.py` 是动态障碍物风险判断节点。

主要功能：

- 订阅 `/dynamic_scan`
- 读取动态障碍物运动区域
- 判断机器人与动态障碍物的相对位置
- 判断动态障碍物是否正在靠近机器人
- 判断动态障碍物是否已经通过机器人必经路径
- 计算 TTC / 相对 TTC 风险
- 判断机器人是否需要减速、停车或继续通过
- 发布 `/intersection_decision`

输出决策状态：

```文本
正常导航
减速
让行等待
通过路口
恢复
```

各状态含义如下：

| 决策状态 | 含义 |
|---|---|
| `NORMAL_NAVIGATION` | 正常导航 |
| `SLOW_DOWN` | 动态障碍物存在潜在风险，减速观察 |
| `YIELD_WAIT` | 预测存在碰撞风险，停车等待 |
| `PASS_INTERSECTION` | 判断可以通过，继续行驶 |
| `RECOVERY` | 长时间等待后的恢复状态 |

---

### 3. intersection_speed_gate.py

`intersection_speed_gate.py` 是速度门控节点。

主要功能：

- 订阅 `/cmd_vel`
- 订阅 `/intersection_decision`
- 根据 supervisor 的决策对速度进行门控
- 发布 `/cmd_vel_risk`

控制逻辑：

```text
NORMAL_NAVIGATION / PASS_INTERSECTION:
    原样输出速度

SLOW_DOWN:
    按比例降低线速度和角速度

YIELD_WAIT / RECOVERY:
    输出 0 速度
```

该节点让 supervisor 的高层决策真正作用到底盘速度链路中。

---

### 4. collision_monitor

`collision_monitor` 作为最后一层安全保护，订阅 `/cmd_vel_risk` 和 `/dynamic_scan`，输出最终安全速度 `/cmd_vel_safe`。

它不负责复杂决策，只负责在极近距离风险下进行兜底减速或停车。

在本项目中：

```text
intersection_supervisor:
    负责提前判断风险

intersection_speed_gate:
    负责根据决策修改速度

collision_monitor:
    负责最终安全裁决
```

---

## TTC 风险判断思想

TTC 即 **Time To Collision**，表示预计发生碰撞所需的时间。

本项目中使用两类 TTC。

---

### 1. 动态障碍物到机器人通行路径的 TTC

```text
TTC_path = distance_to_robot_path / obstacle_velocity_to_path
```

当动态障碍物预计在较短时间内进入机器人通行区域时，机器人减速或停车等待。

---

### 2. 机器人与动态障碍物的相对 TTC

```text
relative_position = obstacle_position - robot_position
relative_velocity = obstacle_velocity - robot_velocity
```

系统进一步判断机器人和动态障碍物之间的距离是否正在缩小。

如果动态障碍物虽然远离原路径，但机器人自身正在斜向靠近障碍物，系统不会直接放行，而是继续减速或停车等待。

核心判断思想：

```text
动态障碍物远离路径线 ≠ 一定安全

只有同时满足：
    动态障碍物已经通过机器人必经路径
    且动态障碍物与机器人之间的距离正在增大
才认为可以放心通过
```

---

## 通过承诺区设计

在动态障碍物横穿场景中，如果机器人已经进入动态障碍物运动区域，不应该突然停车，否则可能停在障碍物运动路径上，反而增加碰撞风险。

因此本项目设计了通过承诺区：

```text
机器人尚未进入动态障碍物运动区域：
    可以减速或停车等待

机器人运动中进入动态障碍物运动区域：
    进入 PASS_INTERSECTION，继续通过

机器人启动时已经位于动态障碍物运动区域：
    先进行相对 TTC 判断，确认安全后再通过
```

该设计用于解决机器人在冲突区域中途急停导致被动态障碍物撞击的问题。

---

## 窄通道优化

本项目针对 Nav2 在窄通道中容易出现的无法进入、路径贴边、局部控制过于保守等问题，对 costmap 和 DWB 参数进行了优化。

主要涉及：

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
- 合理调整 DWB 中路径跟踪和障碍物代价权重
- 让机器人既能保持安全距离，又能稳定进入狭窄区域
- 使用路径平滑降低局部控制抖动

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

### 1. 启动 Gazebo 仿真

```bash
ros2 launch fishbot_description gazebo_sim.launch.py
```

### 2. 启动 Nav2 与动态避障系统

```bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

---

## 关键话题检查

### 检查自研节点是否启动

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

### 检查速度链路

```bash
ros2 topic info /cmd_vel_raw -v
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_risk -v
ros2 topic info /cmd_vel_safe -v
```

期望链路：

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

### 检查动态避障决策

```bash
ros2 topic echo /intersection_decision
```

可能输出：

```text
data: "NORMAL_NAVIGATION"
data: "SLOW_DOWN"
data: "YIELD_WAIT"
data: "PASS_INTERSECTION"
```

---

## 实验场景

### 1. 基础导航实验

验证机器人能在 Gazebo 环境中完成定位、全局规划、局部控制和目标点到达。

---

### 2. 窄通道通行实验

通过调整 costmap inflation layer 和 DWB critic 权重，使机器人可以稳定进入窄通道。

---

### 3. 动态障碍物减速实验

动态障碍物靠近机器人通行区域时，supervisor 输出：

```text
SLOW_DOWN
```

speed gate 降低机器人速度。

---

### 4. TTC 停车等待实验

当系统预测机器人与动态障碍物存在碰撞风险时，supervisor 输出：

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
- 底盘只接收最终安全速度，避免多速度源抢占控制

---

## 项目亮点

- 不只是调用 Nav2，而是在 Nav2 基础上进行了动态避障链路扩展
- 重新设计了机器人速度控制链路
- 自定义动态障碍物提取节点
-自定义TTC风险判断节点
- 自定义速度门控节点
- 结合 collision_monitor 实现最终安全保护
- 针对窄通道和动态障碍物场景进行了系统调参和验证
- 区分“运动中进入冲突区”和“启动时已在冲突区”两种情况
- 在 TTC 判断中加入机器人自身运动状态，减少误放行和误停车

---

## 局限性与后续优化

当前系统主要面向结构化动态障碍物横穿场景，适用于单个或少量规则运动障碍物的风险判断。

后续可以进一步优化：

- 引入多目标动态障碍物跟踪
- 增加更完整的轨迹预测模块
- 将动态障碍物预测结果接入局部规划器
-对比DWB、RPP、MPPI在动态障碍物场景下的表现
- 在真实机器人平台上验证参数鲁棒性
- 增加 rosbag 实验记录和定量评估指标
- 将动态障碍物预测结果与局部轨迹采样代价进一步融合

---

## 适用方向

本项目适合作为以下方向的学习和求职展示项目：

- 移动机器人路径规划
- ROS2 / Nav2 工程开发
- 动态障碍物避障
- 局部规划与速度控制
- 移动机器人系统集成

---

## 简历项目描述参考

dynamic_scan_filter`、`intersection_supervisor`与`intersection_speed_gate`节点，实现动态障碍物提取、TTC/相对TTC风险预测和速度门控控制。重新设计`/cmd_vel_raw → /cmd_vel → /cmd_vel_risk → /cmd_vel_safe`多级速度链路，使底盘只接收最终安全速度，提高动态避障场景下的通行稳定性与安全性。

---

## 说明

本项目主要用于 ROS2 Nav2 移动机器人导航、动态障碍物避障和路径规划方向的学习与求职展示。
