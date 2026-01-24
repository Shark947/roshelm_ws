#include "bt_executor/nodes/core/set_flag_node.hpp"

#include "bt_executor/adapters/helm_adapter.hpp"
#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"

namespace bt_executor
{

SetFlagNode::SetFlagNode(const std::string &name, const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config), ctx_(getContext(*this))
{
}

BT::PortsList SetFlagNode::providedPorts()
{
  return {BT::InputPort<std::string>("flag", "", "Flag name"),
          BT::InputPort<bool>("value", true, "Flag value")};
}

BT::NodeStatus SetFlagNode::tick()
{
  if (!ctx_ || !ctx_->mission || !ctx_->helm)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto flag = getInput<std::string>("flag");
  if (!flag || flag->empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  const bool value = getInput<bool>("value").value_or(true);
  const double stamp = currentTime(*this);

  if (*flag == "stationing")
  {
    ctx_->mission->setStationing(value);
    ctx_->helm->setBool("STATIONING", value, stamp);
  }
  else if (*flag == "constheight")
  {
    ctx_->mission->setConstHeight(value);
    ctx_->helm->setBool("CONSTHEIGHT", value, stamp);
  }
  else if (*flag == "docking_falling")
  {
    ctx_->mission->setDockingFalling(value);
    ctx_->helm->setBool("DOCKING_FALLING", value, stamp);
  }
  else if (*flag == "manual_override")
  {
    ctx_->mission->setManualOverride(value);
    ctx_->helm->setBool("MOOS_MANUAL_OVERIDE", value, stamp);
  }
  else if (*flag == "docking_failed")
  {
    ctx_->mission->setDockingFailed(value);
    ctx_->helm->setBool("DOCKINGFAILED", value, stamp);
  }
  else if (*flag == "deploy")
  {
    ctx_->mission->setDeploy(value);
    ctx_->helm->setBool("DEPLOY", value, stamp);
  }
  else if (*flag == "return")
  {
    ctx_->mission->setReturn(value);
    ctx_->helm->setBool("RETURN", value, stamp);
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_executor
