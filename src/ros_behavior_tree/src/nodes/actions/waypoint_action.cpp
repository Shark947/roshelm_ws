#include "ros_behavior_tree/nodes/actions/waypoint_action.hpp"

#include "ros_behavior_tree/nodes/actions/behavior_action_utils.hpp"

#include <ros/ros.h>

#include "BHV_Waypoint.h"

namespace ros_behavior_tree
{

WaypointAction::WaypointAction(const std::string &name,
                               const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
  if (config.blackboard)
  {
    config.blackboard->get("helm_interface", helm_interface_);
  }
}

BT::PortsList WaypointAction::providedPorts()
{
  return {BT::InputPort<std::string>("params")};
}

BT::NodeStatus WaypointAction::onStart()
{
  return runBehavior();
}

BT::NodeStatus WaypointAction::onRunning()
{
  return runBehavior();
}

void WaypointAction::onHalted()
{
  cached_params_.clear();
}

bool WaypointAction::ensureBehavior()
{
  if (!helm_interface_)
    return false;

  if (!behavior_)
  {
    behavior_.reset(new BHV_Waypoint(helm_interface_->domain()));
  }
  return true;
}

bool WaypointAction::configureBehavior(const std::string &params)
{
  if (!behavior_)
    return false;

  const auto parsed = parseParams(params);
  for (const auto &entry : parsed)
  {
    if (entry.first == "name")
    {
      if (!behavior_->setBehaviorName(entry.second))
      {
        ROS_WARN_STREAM(
            "[ros_behavior_tree] Failed to set Waypoint name param: "
            << entry.first << "=" << entry.second);
      }
      continue;
    }

    if (behavior_->IvPBehavior::setParam(entry.first, entry.second))
      continue;

    if (!behavior_->setParam(entry.first, entry.second))
    {
      ROS_WARN_STREAM("[ros_behavior_tree] Failed to set Waypoint param: "
                      << entry.first << "=" << entry.second);
    }
  }
  behavior_->onSetParamComplete();
  return true;
}

BT::NodeStatus WaypointAction::runBehavior()
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
    return BT::NodeStatus::SUCCESS;

  return BT::NodeStatus::RUNNING;
}

}  // namespace ros_behavior_tree
