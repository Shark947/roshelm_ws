#include "bt_executor/nodes/helm/activate_behavior_node.hpp"

#include "bt_executor/adapters/helm_adapter.hpp"
#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"

namespace bt_executor
{

ActivateBehaviorNode::ActivateBehaviorNode(const std::string &name, const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config), ctx_(getContext(*this))
{
}

BT::PortsList ActivateBehaviorNode::providedPorts()
{
  return {BT::InputPort<std::string>("behavior", "", "Behavior name to activate"),
          BT::InputPort<std::string>("update_var", "", "Optional update variable"),
          BT::InputPort<std::string>("update_value", "", "Optional update value")};
}

BT::NodeStatus ActivateBehaviorNode::tick()
{
  if (!ctx_ || !ctx_->helm)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto behavior = getInput<std::string>("behavior");
  if (!behavior || behavior->empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  const double stamp = currentTime(*this);
  ctx_->helm->activateBehavior(*behavior, stamp);

  const auto update_var = getInput<std::string>("update_var");
  const auto update_value = getInput<std::string>("update_value");
  if (update_var && !update_var->empty() && update_value && !update_value->empty())
  {
    ctx_->helm->setUpdateVar(*update_var, *update_value, stamp);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_executor
