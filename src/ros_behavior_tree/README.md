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
│   └── behavior_tree.yaml
├── include/
│   └── ros_behavior_tree/
│       ├── bt/
│       │   └── register_nodes.hpp
│       ├── bt_nodes.hpp
│       ├── nav_subscriber.hpp
│       └── nodes/
│           ├── actions/
│           │   └── dummy_action.hpp
│           └── conditions/
│               └── common/
│                   └── deploy_triggered.hpp
├── launch/
│   └── behavior_tree.launch
└── src/
    ├── behavior_tree_node.cpp
    ├── bt/
    │   └── register_nodes.cpp
    ├── bt_nodes.cpp
    ├── nav_subscriber.cpp
    └── nodes/
        ├── actions/
        │   └── dummy_action.cpp
        └── conditions/
            └── common/
                └── deploy_triggered.cpp
```

## 数据来源

该节点并行订阅 **两套导航数据**：

- **ROS 风格话题**：`/<vehicle>/current_heading`, `/current_speed`, `/current_depth`,
  `/current_yaw`, `/current_pitch`, `/current_roll`, `/current_x`, `/current_y`
- **NAV/MOOS 风格话题**：`/<vehicle>/NAV_HEADING`, `/NAV_SPEED`, `/NAV_DEPTH`,
  `/NAV_YAW`, `/NAV_PITCH`, `/NAV_ROLL`, `/NAV_X`, `/NAV_Y`

部署/返航话题在两套数据中共用：`/<vehicle>/DEPLOY` 与 `/<vehicle>/RETURN`。

默认值与 `ros_helm` 的 ROS bridge 配置中话题命名保持一致。若重映射话题，请同步调整参数。

## 行为树流程

节点会加载 BehaviorTree.CPP 的 XML 文件，并提供一个简易树用于在 Groot 中验证：

- 默认的 `config/test.xml` 包含一个 `DeployTriggered` 条件节点和一个
  `DummyAction` 行为节点。
- 当 `/auh/DEPLOY`（或配置的 `deploy_topic`）被发布 `data: true` 时，
  `DeployTriggered` 返回 **SUCCESS**，`DummyAction` 会先 **RUNNING** 一段
  时间后再返回 **SUCCESS**。
- 若 `DeployTriggered` 失败，序列节点会立刻返回失败，整个树直接失败。

可以用该树验证 ROS 节点、BT 运行时与 Groot ZMQ 连接是否正确。

## 配置

默认参数见 `config/behavior_tree.yaml`：

- `vehicle_name`
- `loop_frequency`
- `bt_xml`
- `groot_enable`
- `groot_publisher_port`
- `groot_server_port`
- `ros_*_topic`, `nav_*_topic`, `deploy_topic`, `return_topic`

> 注意：`groot_publisher_port` 与 `groot_server_port` 必须不同；若相同将禁用
> Groot ZMQ 发布器并输出错误日志。

## 运行

```bash
roslaunch ros_behavior_tree behavior_tree.launch
```
