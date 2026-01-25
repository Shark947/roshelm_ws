# ros_behavior_tree agent instructions

## Scope
These instructions apply to files in `ros_behavior_tree/`.

## Development guidelines
- Keep compatibility with Ubuntu 18.04 + ROS1 Melodic (C++11, catkin).
- Prefer `common_msgs/Float64Stamped` for navigation inputs and `std_msgs/Float64` for desired outputs to match existing topics.
- When adding or changing parameters, update `config/behavior_tree.yaml` and `README.md` together.
- Keep the behavior tree structure documented in `README.md` when logic changes.
