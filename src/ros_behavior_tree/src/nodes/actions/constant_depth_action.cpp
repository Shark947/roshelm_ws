#include "ros_behavior_tree/nodes/actions/constant_depth_action.hpp"

#include "ros_behavior_tree/nodes/actions/behavior_action_utils.hpp"

#include <ros/ros.h>

namespace ros_behavior_tree
{

ConstantDepthAction::ConstantDepthAction(const std::string &name,
                                         const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
  if (config.blackboard)
  {
    config.blackboard->get("helm_interface", helm_interface_);
  }
}

BT::PortsList ConstantDepthAction::providedPorts()
{
  return {BT::InputPort<std::string>("params")};
}

BT::NodeStatus ConstantDepthAction::onStart()
{
  return runBehavior();
}

BT::NodeStatus ConstantDepthAction::onRunning()
{
  return runBehavior();
}

void ConstantDepthAction::onHalted()
{
  if (behavior_active_ && helm_interface_ && behavior_)
    helm_interface_->deactivateBehavior(*behavior_);
  behavior_active_ = false;
}

bool ConstantDepthAction::ensureBehavior()
{
  if (!helm_interface_)
    return false;

  if (!behavior_)
  {
    behavior_.reset(new ConstantDepthBehavior(helm_interface_->domain()));
  }
  return true;
}

bool ConstantDepthAction::configureBehavior(const std::string &params)
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
        ROS_WARN_STREAM("[ros_behavior_tree] Failed to set ConstantDepth name: "
                        << entry.second);
      }
      continue;
    }
    if (behavior_->IvPBehavior::setParam(entry.first, entry.second))
      continue;

    if (!behavior_->setParam(entry.first, entry.second))
    {
      ROS_WARN_STREAM(
          "[ros_behavior_tree] Failed to set ConstantDepth param: "
          << entry.first << "=" << entry.second);
    }
  }
  behavior_->onSetParamComplete();
  return true;
}

BT::NodeStatus ConstantDepthAction::runBehavior()
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
    end_flags_published_ = false;
  }

  if (!helm_interface_->solveForBehavior(*behavior_))
    return BT::NodeStatus::FAILURE;

  behavior_active_ = true;
  const std::string runnable_state = behavior_->isRunnable();
  if (runnable_state == "completed")
  {
    if (!end_flags_published_)
    {
      const auto &end_flags = behavior_->endFlags();
      for (const auto &flag : end_flags)
      {
        helm_interface_->publishFlag(flag);
      }
      end_flags_published_ = true;
    }
    helm_interface_->deactivateBehavior(*behavior_);
    behavior_active_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace ros_behavior_tree
