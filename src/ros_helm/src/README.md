# ROS Helm Wrapper

This repository packages the IvP Helm into a ROS-friendly node that mirrors the existing CLI behavior while exposing ROS topics and parameters.

## Build

```bash
catkin_make  # or colcon build
```

## Run

Launch with the provided example parameters and remappings:

```bash
roslaunch ros_helm helm_node.launch
```

Key parameters live in `config/ros/params.yaml` and can be overridden with ROS launch or `rosparam`.
