#include "bt_executor/nodes/actions.hpp"

#include <ros/ros.h>

namespace bt_executor
{

namespace
{

double currentTime(const BT::TreeNode &node)
{
  if (node.config().blackboard)
  {
    try
    {
      return node.config().blackboard->get<double>("current_time");
    }
    catch (const std::exception &)
    {
    }
  }
  return ros::Time::now().toSec();
}

}  // namespace

BT::PortsList ActivateBehaviorAction::providedPorts()
{
  return {BT::InputPort<std::string>("behavior", "", "Behavior name to activate"),
          BT::InputPort<std::string>("update_var", "", "Optional update variable"),
          BT::InputPort<std::string>("update_value", "", "Optional update value")};
}

BT::NodeStatus ActivateBehaviorAction::tick()
{
  const auto behavior = getInput<std::string>("behavior");
  if (!behavior || behavior->empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  const double stamp = currentTime(*this);
  helm_adapter_->activateBehavior(*behavior, stamp);

  const auto update_var = getInput<std::string>("update_var");
  const auto update_value = getInput<std::string>("update_value");
  if (update_var && !update_var->empty() && update_value && !update_value->empty())
  {
    helm_adapter_->setUpdateVar(*update_var, *update_value, stamp);
  }

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetModeAction::providedPorts()
{
  return {BT::InputPort<std::string>("mode", "", "Mode to set")};
}

BT::NodeStatus SetModeAction::tick()
{
  const auto mode = getInput<std::string>("mode");
  if (!mode || mode->empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  const double stamp = currentTime(*this);
  mission_state_->setMode(*mode);
  helm_adapter_->setMode(*mode, stamp);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetFlagAction::providedPorts()
{
  return {BT::InputPort<std::string>("flag", "", "Flag name"),
          BT::InputPort<bool>("value", true, "Flag value")};
}

BT::NodeStatus SetFlagAction::tick()
{
  const auto flag = getInput<std::string>("flag");
  if (!flag || flag->empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  const bool value = getInput<bool>("value").value_or(true);
  const double stamp = currentTime(*this);

  if (*flag == "stationing")
  {
    mission_state_->setStationing(value);
    helm_adapter_->setBool("STATIONING", value, stamp);
  }
  else if (*flag == "constheight")
  {
    mission_state_->setConstHeight(value);
    helm_adapter_->setBool("CONSTHEIGHT", value, stamp);
  }
  else if (*flag == "docking_falling")
  {
    mission_state_->setDockingFalling(value);
    helm_adapter_->setBool("DOCKING_FALLING", value, stamp);
  }
  else if (*flag == "manual_override")
  {
    mission_state_->setManualOverride(value);
    helm_adapter_->setBool("MOOS_MANUAL_OVERIDE", value, stamp);
  }
  else if (*flag == "docking_failed")
  {
    mission_state_->setDockingFailed(value);
    helm_adapter_->setBool("DOCKINGFAILED", value, stamp);
  }
  else if (*flag == "deploy")
  {
    mission_state_->setDeploy(value);
    helm_adapter_->setBool("DEPLOY", value, stamp);
  }
  else if (*flag == "return")
  {
    mission_state_->setReturn(value);
    helm_adapter_->setBool("RETURN", value, stamp);
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_executor
