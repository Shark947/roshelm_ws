#include "bt_executor/nodes/core/phase_condition.hpp"

#include <limits>

#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"

namespace bt_executor
{

PhaseCondition::PhaseCondition(const std::string &name, const BT::NodeConfiguration &config)
  : BT::ConditionNode(name, config), ctx_(getContext(*this))
{
}

BT::PortsList PhaseCondition::providedPorts()
{
  return {BT::InputPort<int>("phase", std::numeric_limits<int>::min(), "Exact phase"),
          BT::InputPort<int>("min_phase", std::numeric_limits<int>::min(), "Minimum phase"),
          BT::InputPort<int>("max_phase", std::numeric_limits<int>::max(), "Maximum phase")};
}

BT::NodeStatus PhaseCondition::tick()
{
  if (!ctx_ || !ctx_->mission)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto snapshot = ctx_->mission->snapshot();
  const int phase = snapshot.phase;

  const int exact_phase = getInput<int>("phase").value_or(std::numeric_limits<int>::min());
  if (exact_phase != std::numeric_limits<int>::min())
  {
    return phase == exact_phase ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  const int min_phase = getInput<int>("min_phase").value_or(std::numeric_limits<int>::min());
  const int max_phase = getInput<int>("max_phase").value_or(std::numeric_limits<int>::max());
  if (phase < min_phase || phase > max_phase)
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_executor
