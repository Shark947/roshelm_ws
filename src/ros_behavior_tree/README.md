# ROS Behavior Tree

This package adds a lightweight behavior tree (BT) **foundation** for ROS1 **Melodic** on **Ubuntu 18.04**. It currently focuses on collecting navigation data from `ros_helm` topic conventions, with behavior logic to be added later.

## Directory structure

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
│       ├── bt_nodes.hpp
│       └── nav_subscriber.hpp
├── launch/
│   └── behavior_tree.launch
└── src/
    ├── behavior_tree_node.cpp
    ├── bt_nodes.cpp
    └── nav_subscriber.cpp
```

## Data sources

The node subscribes to **two navigation data sets** (both are collected in parallel):

- **ROS-style topics**: `/<vehicle>/current_heading`, `/current_speed`, `/current_depth`,
  `/current_yaw`, `/current_pitch`, `/current_roll`, `/current_x`, `/current_y`,
  `/DEPLOY`, `/RETURN`
- **NAV/MOOS-style topics**: `/<vehicle>/NAV_HEADING`, `/NAV_SPEED`, `/NAV_DEPTH`,
  `/NAV_YAW`, `/NAV_PITCH`, `/NAV_ROLL`, `/NAV_X`, `/NAV_Y`, `/DEPLOY`, `/RETURN`

The defaults mirror the topic naming used in `ros_helm`'s ROS bridge configuration. Update the parameters if you remap the topics.

## Behavior tree flow

Behavior logic is intentionally **not implemented yet**. The current node only provides a data collection base and an empty tick loop to be extended in future iterations.

## Configuration

See `config/behavior_tree.yaml` for default parameters:

- `vehicle_name`
- `loop_frequency`
- `ros_*_topic`, `nav_*_topic`

## Run

```bash
roslaunch ros_behavior_tree behavior_tree.launch
```
