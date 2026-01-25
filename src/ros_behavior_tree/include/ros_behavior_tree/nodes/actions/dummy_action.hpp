#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>

namespace ros_behavior_tree
{

class DummyAction : public BT::StatefulActionNode
{
public:
  DummyAction(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  ros::Time start_time_;
  double run_duration_{1.0};
};

}  // namespace ros_behavior_tree
