#include "bt_executor/nodes/helm/helm_post_tick_node.hpp"

#include "bt_executor/adapters/docking_phase_manager.hpp"
#include "bt_executor/adapters/helm_adapter.hpp"
#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"
#include "bt_executor/ros/ros_io.hpp"

namespace bt_executor
{

HelmPostTickNode::HelmPostTickNode(const std::string &name, const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config), ctx_(getContext(*this))
{
}

BT::NodeStatus HelmPostTickNode::tick()
{
  if (!ctx_ || !ctx_->helm || !ctx_->mission || !ctx_->nav || !ctx_->ros_io || !ctx_->docking)
  {
    return BT::NodeStatus::FAILURE;
  }

  double stamp = currentTime(*this);
  if (stamp <= 0.0)
  {
    stamp = ctx_->ros_io->now();
    if (config().blackboard)
    {
      config().blackboard->set("current_time", stamp);
    }
  }

  const NavSnapshot nav = ctx_->nav->snapshot();
  const MissionSnapshot mission = ctx_->mission->snapshot();

  ctx_->helm->syncFromState(nav, mission, stamp);

  const HelmDecision decision = ctx_->helm->solve(stamp);
  const auto messages = ctx_->helm->consumeBehaviorMessages(stamp);
  ctx_->ros_io->applyBehaviorMessages(messages, stamp);

  const MissionSnapshot mission_after_msgs = ctx_->mission->snapshot();
  ctx_->helm->syncFromState(nav, mission_after_msgs, stamp);

  ctx_->ros_io->publishCommands(decision, stamp);
  ctx_->ros_io->publishModeState(mission_after_msgs.mode);
  ctx_->docking->setDesiredSpeed(ctx_->ros_io->lastCommandSpeed(nav.speed));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_executor
