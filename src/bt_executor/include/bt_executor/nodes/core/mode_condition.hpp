#pragma once

#include <memory>

#include <behaviortree_cpp_v3/condition_node.h>

namespace bt_executor
{

struct BTContext;

class ModeCondition : public BT::ConditionNode
{
public:
  ModeCondition(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BTContext> ctx_;
};

}  // namespace bt_executor
