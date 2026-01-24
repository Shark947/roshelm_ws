#include "bt_executor/nodes/core/mode_condition.hpp"

#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"

namespace bt_executor
{

ModeCondition::ModeCondition(const std::string &name, const BT::NodeConfiguration &config)
  : BT::ConditionNode(name, config), ctx_(getContext(*this))
{
}

BT::PortsList ModeCondition::providedPorts()
{
  return {BT::InputPort<std::string>("mode", "", "Required mode string")};
}

BT::NodeStatus ModeCondition::tick()
{
  if (!ctx_ || !ctx_->mission)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto snapshot = ctx_->mission->snapshot();
  const auto mode = getInput<std::string>("mode");
  if (!mode || mode->empty())
  {
    return BT::NodeStatus::FAILURE;
  }
  return snapshot.mode == *mode ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace bt_executor
