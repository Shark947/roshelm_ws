#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>

namespace bt_executor
{

struct BTContext;

class SetFlagNode : public BT::SyncActionNode
{
public:
  SetFlagNode(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BTContext> ctx_;
};

}  // namespace bt_executor
