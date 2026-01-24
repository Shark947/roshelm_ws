#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

namespace bt_executor
{

class AlwaysSuccessCondition : public BT::ConditionNode
{
public:
  AlwaysSuccessCondition(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace bt_executor
