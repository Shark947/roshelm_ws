# ROS Helm Launch and Deployment Guide

This quick guide explains how to bring up `roshelm.launch`, verify that the Helm is
running with behaviors loaded, and then explicitly deploy it.

## 1. Build and source the workspace

```bash
catkin_make
source devel/setup.bash
```

## 2. Launch the Helm stack

Start the full stack (simulator, thrusters, controller manager, and ROS Helm):

```bash
roslaunch auh_launch roshelm.launch
```

The launch file loads `config/ros/params.yaml` into the `ros_helm_node` namespace
and then starts `ros_helm_node` with the Helm YAML configuration at
`config/helm/startHelm.yaml`. When the node starts successfully you should see
Helm console output similar to:

```
Helm Iteration:  0
IvP Functions:   0
Behaviors Spawnable: --------- (3)
```

If you still see zero spawnable behaviors, confirm that the `config/helm` files
exist and that the `ros_helm_node` parameters were loaded (e.g., with
`rosparam list | grep ros_helm_node`).

## 3. Deploy the behaviors explicitly

The default configuration now keeps `DEPLOY` **false** until you command it.
After the launch finishes, send a deploy command to activate the behaviors:

```bash
rostopic pub --once /auh/DEPLOY std_msgs/Bool "data: true"
```

Once the message is received, the Helm will report active behaviors and begin
posting desired navigation setpoints on the `/auh/desired_*` topics. To stop the
mission, publish `false` on the same topic:

```bash
rostopic pub --once /auh/DEPLOY std_msgs/Bool "data: false"
```

## 4. Optional: Return behavior

You can trigger the return waypoint behavior at any time by publishing to the
`/auh/RETURN` topic:

```bash
rostopic pub --once /auh/RETURN std_msgs/Bool "data: true"
```

This will switch the Helm into the return leg defined in `alpha.bhv` and post
`MISSION=complete` when finished.
