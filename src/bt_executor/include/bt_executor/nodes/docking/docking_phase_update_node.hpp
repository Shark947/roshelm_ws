#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>

namespace bt_executor
{

struct BTContext;

class DockingPhaseUpdateNode : public BT::SyncActionNode
{
public:
  DockingPhaseUpdateNode(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BTContext> ctx_;
};

}  // namespace bt_executor
