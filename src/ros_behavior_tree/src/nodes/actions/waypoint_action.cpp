#include "ros_behavior_tree/nodes/actions/waypoint_action.hpp"

#include "ros_behavior_tree/nodes/actions/behavior_action_utils.hpp"

#include <ros/ros.h>

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
  // Preserve cached params/state across halts so the behavior does not
  // reinitialize each tick in reactive trees. This avoids resetting waypoint
  // progress and prevents repeated "state changed" log spam.
}

bool WaypointAction::ensureBehavior()
{
  if (!helm_interface_)
    return false;

  if (!behavior_)
  {
    behavior_.reset(new WaypointBehavior(helm_interface_->domain()));
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
      if (!behavior_->setBehaviorNamePublic(entry.second))
      {
        ROS_WARN_STREAM("[ros_behavior_tree] Failed to set Waypoint name: "
                        << entry.second);
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
  last_total_hits_ = behavior_->totalHits();
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

  publishEndFlagsIfNeeded();

  const std::string runnable_state = behavior_->isRunnable();
  if (runnable_state != last_runnable_state_)
  {
    ROS_INFO_STREAM("[ros_behavior_tree] WaypointAction state changed: "
                    << runnable_state);
    last_runnable_state_ = runnable_state;
  }
  if (runnable_state == "completed")
  {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaypointAction::publishEndFlagsIfNeeded()
{
  if (!behavior_ || !helm_interface_)
    return;

  const unsigned int total_hits = behavior_->totalHits();
  const unsigned int waypoint_count = behavior_->waypointCount();

  if (total_hits < last_total_hits_)
  {
    if (waypoint_count > 0 && last_total_hits_ >= waypoint_count)
    {
      const auto &end_flags = behavior_->endFlags();
      for (const auto &flag : end_flags)
      {
        helm_interface_->publishFlag(flag);
      }
    }
    last_total_hits_ = total_hits;
    return;
  }
  if (total_hits == last_total_hits_)
    return;

  if (waypoint_count == 0)
  {
    last_total_hits_ = total_hits;
    return;
  }

  const unsigned int last_cycles = last_total_hits_ / waypoint_count;
  const unsigned int current_cycles = total_hits / waypoint_count;
  if (current_cycles <= last_cycles)
  {
    last_total_hits_ = total_hits;
    return;
  }

  const auto &end_flags = behavior_->endFlags();
  for (const auto &flag : end_flags)
  {
    helm_interface_->publishFlag(flag);
  }
  last_total_hits_ = total_hits;
}

}  // namespace ros_behavior_tree
