#include "bt_executor/nodes/core/any_mode_condition.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>

#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"

namespace bt_executor
{

namespace
{

std::string trimCopy(const std::string &value)
{
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }).base();
  if (begin >= end)
  {
    return std::string{};
  }
  return std::string(begin, end);
}

}  // namespace

AnyModeCondition::AnyModeCondition(const std::string &name, const BT::NodeConfiguration &config)
  : BT::ConditionNode(name, config), ctx_(getContext(*this))
{
}

BT::PortsList AnyModeCondition::providedPorts()
{
  return {BT::InputPort<std::string>("modes", "",
                                     "Semicolon separated mode list (e.g. RETURNING;CLOSETODOCKING)")};
}

BT::NodeStatus AnyModeCondition::tick()
{
  if (!ctx_ || !ctx_->mission)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto snapshot = ctx_->mission->snapshot();
  const auto modes_input = getInput<std::string>("modes");
  if (!modes_input || modes_input->empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  std::stringstream stream(*modes_input);
  std::string mode;
  while (std::getline(stream, mode, ';'))
  {
    if (trimCopy(mode) == snapshot.mode)
    {
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_executor
