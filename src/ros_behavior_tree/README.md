# ROS 行为树

这个包为 ROS1 **Melodic**（**Ubuntu 18.04**）提供轻量级行为树（BT）**基础设施**。当前主要用于从 `ros_helm` 的话题规范采集导航数据，行为逻辑可在后续迭代中扩展。

## 目录结构

```
ros_behavior_tree/
├── AGENTS.md
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── alpha.xml
│   ├── behavior_tree_alpha.yaml
│   ├── behavior_tree_test.yaml
│   └── test.xml
├── include/
│   └── ros_behavior_tree/
│       ├── bt/
│       │   └── register_nodes.hpp
│       ├── bt_nodes.hpp
│       ├── nav_subscriber.hpp
│       └── nodes/
│           ├── actions/
│           │   ├── constant_speed_action.hpp
│           │   ├── dummy_action.hpp
│           │   ├── mission_complete_action.hpp
│           │   └── waypoint_action.hpp
│           └── conditions/
│               └── common/
│                   ├── deploy_triggered.hpp
│                   ├── return_triggered.hpp
│                   └── speed_triggered.hpp
├── launch/
│   └── behavior_tree.launch
└── src/
    ├── behavior_tree_node.cpp
    ├── nav_publisher_node.cpp
    ├── bt/
    │   └── register_nodes.cpp
    ├── bt_nodes.cpp
    ├── helm_interface.cpp
    ├── nav_subscriber.cpp
    └── nodes/
        ├── actions/
        │   ├── constant_speed_action.cpp
        │   ├── dummy_action.cpp
        │   ├── mission_complete_action.cpp
        │   └── waypoint_action.cpp
        └── conditions/
            └── common/
                ├── deploy_triggered.cpp
                ├── return_triggered.cpp
                └── speed_triggered.cpp
```

## 数据来源

该节点并行订阅 **两套导航数据**：

- **ROS 风格话题**：`/<vehicle>/current_heading`, `/current_speed`, `/current_depth`,
  `/current_yaw`, `/current_pitch`, `/current_roll`, `/current_x`, `/current_y`
- **NAV/MOOS 风格话题**：`/<vehicle>/NAV_HEADING`, `/NAV_SPEED`, `/NAV_DEPTH`,
  `/NAV_YAW`, `/NAV_PITCH`, `/NAV_ROLL`, `/NAV_X`, `/NAV_Y`

部署/返航与恒速触发话题在两套数据中共用：`/<vehicle>/DEPLOY`、`/<vehicle>/RETURN`、
`/<vehicle>/SPD`。

当只接入 `current_vx/current_vy/current_z/current_yaw` 等基础量时，需要额外生成
`current_speed/current_heading/current_depth`。默认 launch 会启动一个轻量级
`nav_publisher_node` 来完成这一转换，并同步发布派生的
`NAV_SPEED/NAV_HEADING/NAV_DEPTH/NAV_YAW/NAV_PITCH/NAV_ROLL/NAV_X/NAV_Y`，
便于下游组件直接订阅 NAV 风格话题。`current_heading` 直接由
`current_yaw` 计算，`NAV_HEADING/NAV_YAW/NAV_PITCH/NAV_ROLL` 则依赖
`current_yaw/current_pitch/current_roll` 的同步更新，保持与 `ros_helm`
的转换逻辑一致。

默认值与 `ros_helm` 的 ROS bridge 配置中话题命名保持一致。若重映射话题，请同步调整参数。

## 行为树流程

节点会加载 BehaviorTree.CPP 的 XML 文件，当前提供两套示例：

- `config/alpha.xml` 通过 `DeployTriggered`、`ReturnTriggered`、
  `SpeedTriggered` 组合 `WaypointAction` 与 `ConstantSpeedAction`，复刻
  `alpha.bhv` 的行为树逻辑。航点巡航/返航行为保持 `perpetual=true`，
  并在 `WaypointAction` 内部模拟 `endflag`：当航点巡航完成一次遍历时会
  发布 `RETURN=true`，返航完成时发布 `RETURN=false`、`DEPLOY=false`
  与 `MISSION=complete`。`endflag` 也支持发布任意自定义变量，默认映射为
  `/<vehicle>/<VAR>` 话题。返航分支具有最高优先级；在非返航状态下，
  航点巡航与恒速控制会并行运行，以便在发布 `SPD` 触发时保持恒速行为
  不中断巡航。
- `config/test.xml` 仍保留用于验证 BT 运行时与 Groot 连接的最小树。

`alpha.xml` 会通过 HelmEngine 求解并发布 `DESIRED_*`（映射为
`desired_heading`/`desired_speed`/`desired_depth`）话题，并保持与
`ros_helm` 的坐标系转换一致。

可以用 `test.xml` 验证 ROS 节点、BT 运行时与 Groot ZMQ 连接是否正确。

## 配置

默认参数见 `config/behavior_tree_alpha.yaml`：

- `vehicle_name`
- `loop_frequency`
- `bt_xml`
- `groot_enable`
- `groot_publisher_port`
- `groot_server_port`
- `nav_timeout`
- `trigger_hold_time`
- `nav_log_period`
- `latch_deploy`
- `domain_course`, `domain_speed`, `domain_depth`
- `desired_heading_topic`, `desired_speed_topic`, `desired_depth_topic`
- `mission_topic`
- `ros_*_topic`, `nav_*_topic`, `deploy_topic`, `return_topic`
- `speed_trigger_topic`

> 注意：`groot_publisher_port` 与 `groot_server_port` 必须不同；若相同将禁用
> Groot ZMQ 发布器并输出错误日志。

`trigger_hold_time` 用于触发类布尔话题（`DEPLOY/RETURN/SPD`）的保持时间，
当上游持续发布 `false` 时，单次 `true` 仍可在保持窗口内被行为树捕获。
`latch_deploy` 用于将首次 `DEPLOY=true` 锁存为持续真值，适合仅发布一次
部署触发的场景。
`nav_log_period` 控制导航数据日志输出频率（秒），设置为 `<=0` 可关闭日志。

导航数据会写入 `ros_behavior_tree/log/<timestamp>_log/` 中的 `NAV_*` 文件，
`DESIRED_*` 目标输出也会同步写入对应日志文件，并在控制台输出调试信息。

## 运行

```bash
roslaunch ros_behavior_tree behavior_tree.launch
```

该 launch 同时启动 `nav_publisher_node`，用于从
`/<vehicle>/current_vx`、`/<vehicle>/current_vy`、`/<vehicle>/current_yaw`、
`/<vehicle>/current_pitch`、`/<vehicle>/current_roll`、`/<vehicle>/current_z`
等话题生成 `current_speed/current_heading/current_depth`，并同步发布
`NAV_*` 话题。`current_speed` 使用最新的 `vx/vy` 组合，`NAV_SPEED` 则要求
`vx/vy` 时间戳对齐。若系统已提供这些话题，可在自定义 launch 中移除该节点。
