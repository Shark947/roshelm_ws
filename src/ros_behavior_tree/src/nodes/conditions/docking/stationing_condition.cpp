#include "ros_behavior_tree/nodes/conditions/docking/stationing_condition.hpp"

namespace ros_behavior_tree
{

StationingCondition::StationingCondition(const std::string &name,
                                         const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config)
{
  if (config.blackboard)
  {
    config.blackboard->get("helm_interface", helm_interface_);
  }
}

BT::PortsList StationingCondition::providedPorts()
{
  return {BT::InputPort<bool>("value", true, "expected bool value")};
}

BT::NodeStatus StationingCondition::tick()
{
  if (!helm_interface_)
    return BT::NodeStatus::FAILURE;

  bool expected = true;
  getInput("value", expected);

  bool actual = false;
  if (!helm_interface_->queryBool("STATIONING", actual))
    return BT::NodeStatus::FAILURE;

  return (actual == expected) ? BT::NodeStatus::SUCCESS
                              : BT::NodeStatus::FAILURE;
}

}  // namespace ros_behavior_tree
