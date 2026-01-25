#include "bt_executor/nodes/helm/helm_pre_tick_node.hpp"

#include "bt_executor/adapters/helm_adapter.hpp"
#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"
#include "bt_executor/ros/ros_io.hpp"

namespace bt_executor
{

HelmPreTickNode::HelmPreTickNode(const std::string &name, const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config), ctx_(getContext(*this))
{
}

BT::NodeStatus HelmPreTickNode::tick()
{
  if (!ctx_ || !ctx_->helm || !ctx_->mission || !ctx_->nav || !ctx_->ros_io)
  {
    return BT::NodeStatus::FAILURE;
  }

  const double stamp = ctx_->ros_io->now();
  if (config().blackboard)
  {
    config().blackboard->set("current_time", stamp);
  }

  const NavSnapshot nav = ctx_->nav->snapshot();
  const MissionSnapshot mission = ctx_->mission->snapshot();
  ctx_->helm->syncFromState(nav, mission, stamp);
  ctx_->helm->resetActivations(stamp);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_executor
