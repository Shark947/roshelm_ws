#include "bt_executor/nodes/docking/docking_phase_update_node.hpp"

#include "bt_executor/adapters/docking_phase_manager.hpp"
#include "bt_executor/bt_context.hpp"
#include "bt_executor/nodes/node_context.hpp"
#include "bt_executor/ros_io.hpp"

namespace bt_executor
{

DockingPhaseUpdateNode::DockingPhaseUpdateNode(const std::string &name, const BT::NodeConfiguration &config)
  : BT::SyncActionNode(name, config), ctx_(getContext(*this))
{
}

BT::NodeStatus DockingPhaseUpdateNode::tick()
{
  if (!ctx_ || !ctx_->docking || !ctx_->mission || !ctx_->nav || !ctx_->ros_io)
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

  const DockingPhaseResult result = ctx_->docking->tick(nav, mission);
  ctx_->ros_io->applyDockingCommands(result.commands, stamp);

  const MissionSnapshot updated = ctx_->mission->snapshot();
  const bool docking_active = (updated.mode == "DOCKING" || updated.mode == "CLOSETODOCKING");
  ctx_->mission->updateDockingTelemetry(result.outputs.phase, ctx_->docking->phaseTotal(),
                                        result.outputs.current_depth, result.outputs.inner_radius,
                                        result.outputs.outer_radius, docking_active,
                                        result.outputs.optical_valid, result.outputs.distance);

  ctx_->ros_io->publishDockingOutputs(result.outputs, stamp);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_executor
