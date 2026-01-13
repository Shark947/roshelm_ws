# docking_nav ROS 化说明

## 节点与话题

**节点：** `docking_nav_node`（算法），`docking_nav_roshelm_node`（入坞 ros-helm 桥接）

**输入：**

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/docking/optical_measurement` | `docking_optical_msgs/OpticalMeasurement` | 视觉测量输入（替代原 TCP 输入） |
| `/docking/mode` | `std_msgs/String` | 入坞模式（如 `CLOSETODOCKING`、`DOCKING`） |
| `/auh/current_depth` | `common_msgs/Float64Stamped` | 当前深度（米） |
| `/auh/current_heading` | `common_msgs/Float64Stamped` | 当前航向（deg） |
| `/auh/current_pitch` | `common_msgs/Float64Stamped` | 当前俯仰（rad） |
| `/auh/current_roll` | `common_msgs/Float64Stamped` | 当前横滚（rad） |
| `/auh/desired_speed` | `std_msgs/Float64` | Helm 输出的期望航速 |

**输出（用于 Helm 与调试）：**

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/docking/mode` | `std_msgs/String` | DockingNav 触发的模式切换 |
| `/docking/dockdepth_update` | `std_msgs/String` | Helm `DOCKDEPTH_UPDATE` |
| `/docking/dockhdg_updates` | `std_msgs/String` | Helm `DOCKHDG_UPDATES` |
| `/docking/stationing` | `std_msgs/Bool` | `STATIONING` |
| `/docking/constheight` | `std_msgs/Bool` | `CONSTHEIGHT` |
| `/docking/docking_falling` | `std_msgs/Bool` | `DOCKING_FALLING` |
| `/docking/manual_override` | `std_msgs/Bool` | `MOOS_MANUAL_OVERIDE` |
| `/docking/docking_failed` | `std_msgs/Bool` | `DOCKINGFAILED` |
| `/docking/phase` | `std_msgs/Int32` | 当前入坞阶段 `nPhaseCount` |
| `/docking/optical_xy` | `geometry_msgs/PointStamped` | 视觉换算的平面偏移 (`dfNextX/dfNextY`) |
| `/docking/optical_feedback` | `docking_optical_msgs/OpticalFeedback` | 视觉端反馈（替代原 TCP 回传） |

## 启动流程

1. **启动入坞任务（含 ros-helm）：**

```bash
roslaunch auh_launch auh_docking.launch
```

2. **将 Helm 行为切换为 docking.bhv（通过 docking_startHelm.yaml 配置）：**

```bash
rosparam get /docking_nav_roshelm_node/config_path
```

3. **发布模式切换（示例：进入 DOCKING）：**

```bash
rostopic pub --once /docking/mode std_msgs/String "data: 'DOCKING'"
```

4. **发布视觉输入示例：**

```bash
rostopic pub --once /docking/optical_measurement docking_optical_msgs/OpticalMeasurement "header:
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
d_heading_deg: 0.0
theta_x_deg: 0.0
theta_y_deg: 0.0
valid: true
fallback_x: 0.0
fallback_y: 0.0"
```

## 配置参数

参数位于 `docking_nav/config/docking_node.yaml`（算法参数）与
`docking_nav/config/docking_topics.yaml`（I/O 话题映射）。ros-helm 入坞配置在
`docking_nav/config/helm/docking_startHelm.yaml`。
如需直接使用 NAV_* 数据源，可在 `docking_topics.yaml` 中将 `nav_*_topic` 指向对应的 NAV 发布话题。
