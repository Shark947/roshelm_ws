#include "ros_behavior_tree/nodes/actions/constant_speed_action.hpp"

#include "ros_behavior_tree/nodes/actions/behavior_action_utils.hpp"

#include <ros/ros.h>

namespace ros_behavior_tree
{

ConstantSpeedAction::ConstantSpeedAction(const std::string &name,
                                         const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
  if (config.blackboard)
  {
    config.blackboard->get("helm_interface", helm_interface_);
  }
}

BT::PortsList ConstantSpeedAction::providedPorts()
{
  return {BT::InputPort<std::string>("params")};
}

BT::NodeStatus ConstantSpeedAction::onStart()
{
  return runBehavior();
}

BT::NodeStatus ConstantSpeedAction::onRunning()
{
  return runBehavior();
}

void ConstantSpeedAction::onHalted()
{
  cached_params_.clear();
  if (helm_interface_ && behavior_)
    helm_interface_->deactivateBehavior(*behavior_);
}

bool ConstantSpeedAction::ensureBehavior()
{
  if (!helm_interface_)
    return false;

  if (!behavior_)
  {
    behavior_.reset(new ConstantSpeedBehavior(helm_interface_->domain()));
  }
  return true;
}

bool ConstantSpeedAction::configureBehavior(const std::string &params)
{
  if (!behavior_)
    return false;

  const auto parsed = parseParams(params);
  for (const auto &entry : parsed)
  {
    if (entry.first == "name")
    {
      if (!behavior_->setBehaviorNamePublic(entry.second))
      {
        ROS_WARN_STREAM("[ros_behavior_tree] Failed to set ConstantSpeed name: "
                        << entry.second);
      }
      continue;
    }
    if (behavior_->IvPBehavior::setParam(entry.first, entry.second))
      continue;

    if (!behavior_->setParam(entry.first, entry.second))
    {
      ROS_WARN_STREAM(
          "[ros_behavior_tree] Failed to set ConstantSpeed param: "
          << entry.first << "=" << entry.second);
    }
  }
  behavior_->onSetParamComplete();
  return true;
}

BT::NodeStatus ConstantSpeedAction::runBehavior()
{
  if (!helm_interface_)
    return BT::NodeStatus::FAILURE;

  if (!ensureBehavior())
    return BT::NodeStatus::FAILURE;

  std::string params;
  getInput("params", params);
  if (params != cached_params_)
  {
    cached_params_ = params;
    configureBehavior(params);
  }

  if (!helm_interface_->solveForBehavior(*behavior_))
    return BT::NodeStatus::FAILURE;

  const std::string runnable_state = behavior_->isRunnable();
  if (runnable_state == "completed")
  {
    helm_interface_->deactivateBehavior(*behavior_);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace ros_behavior_tree
