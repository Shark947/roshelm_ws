# docking_nav ROS 化说明

## 节点与话题

**节点：** `docking_nav_main_node`（入坞算法 + ROS 话题驱动 Helm 交互）

**输入：**

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/docking/optical_measurement` | `docking_optical_msgs/OpticalMeasurement` | 视觉测量输入（替代原 TCP 输入） |
| `/auh/NAV_DEPTH` | `common_msgs/Float64Stamped` | 当前深度（米） |
| `/auh/NAV_HEADING` | `common_msgs/Float64Stamped` | 当前航向（deg） |
| `/auh/NAV_PITCH` | `common_msgs/Float64Stamped` | 当前俯仰（rad） |
| `/auh/NAV_ROLL` | `common_msgs/Float64Stamped` | 当前横滚（rad） |
| `/auh/desired_speed` | `std_msgs/Float64` | Helm 输出的期望航速 |

**输出（用于调试）：**

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/docking/phase` | `std_msgs/Int32` | 当前入坞阶段 `nPhaseCount` |
| `/docking/optical_xy` | `geometry_msgs/PointStamped` | 视觉换算的平面偏移 (`dfNextX/dfNextY`) |
| `/docking/optical_feedback` | `docking_optical_msgs/OpticalFeedback` | 视觉端反馈（替代原 TCP 回传） |

## 启动流程

1. **启动入坞任务（含 ros-helm）：**

```bash
roslaunch auh_launch auh_docking.launch
```

2. **发布视觉输入示例：**

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

参数位于 `docking_nav/config/docking_node.yaml`（算法参数）、`ros_helm/config/ros/params.yaml`
（Helm/ROS 桥接参数）以及 `docking_nav/config/ros/helm_docking_params.yaml`
（入坞行为的 Helm 命令话题）。入坞 Helm 配置在
`docking_nav/config/helm/docking_startHelm.yaml`。
