#include "ros_behavior_tree/nodes/conditions/common/flag_bool_condition.hpp"

namespace ros_behavior_tree
{

FlagBoolCondition::FlagBoolCondition(const std::string &name,
                                     const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config)
{
  if (config.blackboard)
  {
    config.blackboard->get("helm_interface", helm_interface_);
  }
}

BT::PortsList FlagBoolCondition::providedPorts()
{
  return {BT::InputPort<std::string>("var"),
          BT::InputPort<bool>("value", true)};
}

BT::NodeStatus FlagBoolCondition::tick()
{
  if (!helm_interface_)
    return BT::NodeStatus::FAILURE;

  std::string var;
  bool expected = true;
  if (!getInput("var", var))
    return BT::NodeStatus::FAILURE;
  getInput("value", expected);

  bool actual = false;
  if (!helm_interface_->queryBool(var, actual))
    return BT::NodeStatus::FAILURE;

  return (actual == expected) ? BT::NodeStatus::SUCCESS
                              : BT::NodeStatus::FAILURE;
}

}  // namespace ros_behavior_tree
