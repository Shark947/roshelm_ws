#include "bt_executor/nodes/core/flag_condition.hpp"

#include <string>

#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"

namespace bt_executor
{

FlagCondition::FlagCondition(const std::string &name, const BT::NodeConfiguration &config)
  : BT::ConditionNode(name, config), ctx_(getContext(*this))
{
}

BT::PortsList FlagCondition::providedPorts()
{
  return {BT::InputPort<std::string>("flag", "", "Flag name"),
          BT::InputPort<bool>("value", true, "Expected flag value")};
}

BT::NodeStatus FlagCondition::tick()
{
  if (!ctx_ || !ctx_->mission)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto snapshot = ctx_->mission->snapshot();
  const auto flag = getInput<std::string>("flag");
  const bool expected = getInput<bool>("value").value_or(true);
  if (!flag || flag->empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  bool actual = false;
  if (*flag == "stationing")
  {
    actual = snapshot.stationing;
  }
  else if (*flag == "docking_falling")
  {
    actual = snapshot.docking_falling;
  }
  else if (*flag == "constheight")
  {
    actual = snapshot.constheight;
  }
  else if (*flag == "manual_override")
  {
    actual = snapshot.manual_override;
  }
  else if (*flag == "docking_failed")
  {
    actual = snapshot.docking_failed;
  }
  else if (*flag == "deploy")
  {
    actual = snapshot.deploy;
  }
  else if (*flag == "return")
  {
    actual = snapshot.should_return;
  }
  else if (*flag == "docking_phase_active")
  {
    actual = snapshot.docking_phase_active;
  }
  else if (*flag == "optical_valid")
  {
    actual = snapshot.optical_valid;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }

  return actual == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace bt_executor
