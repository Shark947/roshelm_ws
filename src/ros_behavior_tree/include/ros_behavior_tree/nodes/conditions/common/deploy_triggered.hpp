#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

#include "ros_behavior_tree/bt_nodes.hpp"

namespace ros_behavior_tree
{

class DeployTriggered : public BT::ConditionNode
{
public:
  DeployTriggered(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  const NavDataStore *store_{nullptr};
};

}  // namespace ros_behavior_tree
