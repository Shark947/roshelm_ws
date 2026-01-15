# ROS Helm 使用说明

本说明介绍如何启动 `ros_helm_node` 并运行行为文件（如 `alpha.bhv`），以及如何在需要与
Helm 交互的任务（如 docking）中配置标准的交互模板与 API。

## 1. 编译与环境准备

```bash
catkin_make
source devel/setup.bash
```

## 2. 直接运行行为文件（无 MODE 切换）

如果只需要运行类似 `alpha.bhv` 的行为文件，不需要与 Helm 做频繁交互，可以直接使用
`roshelm.launch`：

```bash
roslaunch auh_launch roshelm.launch
```

启动后会加载 `config/ros/params.yaml` 并运行 `config/helm/startHelm.yaml`。Helm 成功启动时
会输出可生成的行为列表。若行为列表为空，请确认行为文件路径正确，并检查
`rosparam list | grep ros_helm_node` 是否已经加载参数。

### 发布 DEPLOY 命令

默认 `DEPLOY=false`，需要主动发布命令使行为生效：

```bash
rostopic pub --once /auh/DEPLOY std_msgs/Bool "data: true"
```

需要停止时发布：

```bash
rostopic pub --once /auh/DEPLOY std_msgs/Bool "data: false"
```

### 可选 RETURN 行为

```bash
rostopic pub --once /auh/RETURN std_msgs/Bool "data: true"
```

## 3. 需要与 Helm 交互的任务（docking 示例）

当任务需要动态向 Helm 推送变量（如 `MODE`、`STATIONING`、`DOCKDEPTH_UPDATE` 等）时，
建议将 **算法逻辑与 Helm 启动分离**：

1. **启动 Helm：** 使用 `ros_helm_node` 加载 docking 行为配置。
2. **算法节点：** 由算法节点（如 `docking_nav_main_node`）发布交互话题，将命令送入 Helm。

### 3.1 启动 docking Helm

示例（auh_docking.launch 已经集成）：

```bash
roslaunch auh_launch auh_docking.launch
```

该启动文件会：
- 启动 `ros_helm_node`。
- 加载 `docking_nav/config/helm/docking_startHelm.yaml`（其中引用 `docking.bhv`）。
- 通过 `docking_nav/config/ros/helm_docking_params.yaml` 配置 docking 命令话题。

### 3.2 标准交互模板（API）

以下为 docking 交互的标准话题与变量映射（默认）：

**字符串命令：**

| Helm 变量 | ROS 话题 | 类型 |
| --- | --- | --- |
| `MODE` | `/docking/mode` | `std_msgs/String` |
| `DOCKDEPTH_UPDATE` | `/docking/dockdepth_update` | `std_msgs/String` |
| `DOCKHDG_UPDATES` | `/docking/dockhdg_updates` | `std_msgs/String` |

**模式状态（Helm -> ROS）：**

| Helm 模式变量 | ROS 话题 | 类型 |
| --- | --- | --- |
| `MODE` | `/docking/mode_state` | `std_msgs/String` |

**布尔命令：**

| Helm 变量 | ROS 话题 | 类型 |
| --- | --- | --- |
| `STATIONING` | `/docking/stationing` | `std_msgs/Bool` |
| `CONSTHEIGHT` | `/docking/constheight` | `std_msgs/Bool` |
| `DOCKING_FALLING` | `/docking/docking_falling` | `std_msgs/Bool` |
| `MOOS_MANUAL_OVERIDE` | `/docking/manual_override` | `std_msgs/Bool` |
| `DOCKINGFAILED` | `/docking/docking_failed` | `std_msgs/Bool` |

### 3.3 交互示例

```bash
# 切换模式
rostopic pub --once /docking/mode std_msgs/String "data: 'DOCKING'"

# 更新 docking 深度
rostopic pub --once /docking/dockdepth_update std_msgs/String "data: 'depth=80.0'"

# 发布 stationing 状态
rostopic pub --once /docking/stationing std_msgs/Bool "data: true"
```

如需定制话题名称，请修改 `docking_nav/config/ros/helm_docking_params.yaml` 并确保算法节点使用相同的话题。
