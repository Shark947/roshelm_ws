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
│   ├── behavior_tree.yaml
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

默认值与 `ros_helm` 的 ROS bridge 配置中话题命名保持一致。若重映射话题，请同步调整参数。

## 行为树流程

节点会加载 BehaviorTree.CPP 的 XML 文件，当前提供两套示例：

- `config/alpha.xml` 通过 `DeployTriggered`、`ReturnTriggered`、
  `SpeedTriggered` 组合 `WaypointAction` 与 `ConstantSpeedAction`，复刻
  `alpha.bhv` 的行为树逻辑，并在返航结束时发布 `MISSION=complete`。
- `config/test.xml` 仍保留用于验证 BT 运行时与 Groot 连接的最小树。

`alpha.xml` 会通过 HelmEngine 求解并发布 `DESIRED_*`（映射为
`desired_heading`/`desired_speed`/`desired_depth`）话题，并保持与
`ros_helm` 的坐标系转换一致。

可以用 `test.xml` 验证 ROS 节点、BT 运行时与 Groot ZMQ 连接是否正确。

## 配置

默认参数见 `config/behavior_tree.yaml`：

- `vehicle_name`
- `loop_frequency`
- `bt_xml`
- `groot_enable`
- `groot_publisher_port`
- `groot_server_port`
- `nav_timeout`
- `domain_course`, `domain_speed`, `domain_depth`
- `desired_heading_topic`, `desired_speed_topic`, `desired_depth_topic`
- `mission_topic`
- `ros_*_topic`, `nav_*_topic`, `deploy_topic`, `return_topic`
- `speed_trigger_topic`

> 注意：`groot_publisher_port` 与 `groot_server_port` 必须不同；若相同将禁用
> Groot ZMQ 发布器并输出错误日志。

## 运行

```bash
roslaunch ros_behavior_tree behavior_tree.launch
```
