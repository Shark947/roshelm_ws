#include "ros_behavior_tree/nodes/actions/dummy_action.hpp"

namespace ros_behavior_tree
{

DummyAction::DummyAction(const std::string &name,
                         const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
}

BT::PortsList DummyAction::providedPorts()
{
  return {BT::InputPort<double>("duration")};
}

BT::NodeStatus DummyAction::onStart()
{
  run_duration_ = 1.0;
  getInput("duration", run_duration_);
  start_time_ = ros::Time::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DummyAction::onRunning()
{
  if ((ros::Time::now() - start_time_).toSec() >= run_duration_)
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::RUNNING;
}

void DummyAction::onHalted()
{
  start_time_ = ros::Time(0.0);
}

}  // namespace ros_behavior_tree
