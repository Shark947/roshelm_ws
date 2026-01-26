#include "ros_behavior_tree/nodes/conditions/common/mode_condition.hpp"

#include <sstream>

namespace ros_behavior_tree
{

ModeCondition::ModeCondition(const std::string &name,
                             const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config)
{
  if (config.blackboard)
  {
    config.blackboard->get("helm_interface", helm_interface_);
  }
}

BT::PortsList ModeCondition::providedPorts()
{
  return {BT::InputPort<std::string>("value")};
}

std::vector<std::string> ModeCondition::splitValues(
    const std::string &value) const
{
  std::vector<std::string> results;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, '|'))
  {
    if (!token.empty())
      results.push_back(token);
  }
  if (!results.empty())
    return results;

  std::stringstream comma_stream(value);
  while (std::getline(comma_stream, token, ','))
  {
    if (!token.empty())
      results.push_back(token);
  }
  if (!results.empty())
    return results;

  if (!value.empty())
    results.push_back(value);
  return results;
}

BT::NodeStatus ModeCondition::tick()
{
  if (!helm_interface_)
    return BT::NodeStatus::FAILURE;

  std::string expected;
  if (!getInput("value", expected))
    return BT::NodeStatus::FAILURE;

  std::string actual;
  if (!helm_interface_->queryString("MODE", actual))
    return BT::NodeStatus::FAILURE;

  const auto options = splitValues(expected);
  for (const auto &option : options)
  {
    if (actual == option)
      return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace ros_behavior_tree
