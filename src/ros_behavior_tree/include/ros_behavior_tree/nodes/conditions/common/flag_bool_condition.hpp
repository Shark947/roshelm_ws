#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

#include <string>

#include "ros_behavior_tree/helm_interface.hpp"

namespace ros_behavior_tree
{

class FlagBoolCondition : public BT::ConditionNode
{
public:
  FlagBoolCondition(const std::string &name,
                    const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  HelmInterface *helm_interface_{nullptr};
};

}  // namespace ros_behavior_tree
