#include "bt_executor/nodes/core/set_mode_node.hpp"

#include "bt_executor/adapters/helm_adapter.hpp"
#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"

namespace bt_executor
{

SetModeNode::SetModeNode(const std::string &name, const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config), ctx_(getContext(*this))
{
}

BT::PortsList SetModeNode::providedPorts()
{
  return {BT::InputPort<std::string>("mode", "", "Mode to set")};
}

BT::NodeStatus SetModeNode::tick()
{
  if (!ctx_ || !ctx_->mission || !ctx_->helm)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto mode = getInput<std::string>("mode");
  if (!mode || mode->empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  const double stamp = currentTime(*this);
  ctx_->mission->setMode(*mode);
  ctx_->helm->setMode(*mode, stamp);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_executor
