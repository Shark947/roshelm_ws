#include "ros_behavior_tree/nodes/conditions/common/flag_string_condition.hpp"

#include <sstream>

namespace ros_behavior_tree
{

FlagStringCondition::FlagStringCondition(const std::string &name,
                                         const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config)
{
  if (config.blackboard)
  {
    config.blackboard->get("helm_interface", helm_interface_);
  }
}

BT::PortsList FlagStringCondition::providedPorts()
{
  return {BT::InputPort<std::string>("var"),
          BT::InputPort<std::string>("value")};
}

std::vector<std::string> FlagStringCondition::splitValues(
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

BT::NodeStatus FlagStringCondition::tick()
{
  if (!helm_interface_)
    return BT::NodeStatus::FAILURE;

  std::string var;
  std::string expected;
  if (!getInput("var", var))
    return BT::NodeStatus::FAILURE;
  if (!getInput("value", expected))
    return BT::NodeStatus::FAILURE;

  std::string actual;
  if (!helm_interface_->queryString(var, actual))
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
