#include "bt_executor/nodes/core/deploy_ready_condition.hpp"

#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"

namespace bt_executor
{

DeployReadyCondition::DeployReadyCondition(const std::string &name, const BT::NodeConfiguration &config)
  : BT::ConditionNode(name, config), ctx_(getContext(*this))
{
}

BT::PortsList DeployReadyCondition::providedPorts()
{
  return {BT::InputPort<bool>("require_no_override", true, "Fail if manual override is active"),
          BT::InputPort<bool>("require_no_failure", true,
                              "Fail if docking failure has been latched")};
}

BT::NodeStatus DeployReadyCondition::tick()
{
  if (!ctx_ || !ctx_->mission)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto snapshot = ctx_->mission->snapshot();
  const bool require_no_override = getInput<bool>("require_no_override").value_or(true);
  const bool require_no_failure = getInput<bool>("require_no_failure").value_or(true);

  if (!snapshot.deploy || snapshot.should_return)
  {
    return BT::NodeStatus::FAILURE;
  }
  if (require_no_override && snapshot.manual_override)
  {
    return BT::NodeStatus::FAILURE;
  }
  if (require_no_failure && snapshot.docking_failed)
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_executor
