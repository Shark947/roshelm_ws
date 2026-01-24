#include "bt_executor/nodes/conditions.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <sstream>

#include <ros/ros.h>

namespace bt_executor
{

namespace
{

std::string trimCopy(const std::string &value)
{
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }).base();
  if (begin >= end)
  {
    return std::string{};
  }
  return std::string(begin, end);
}

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

BT::PortsList DeployReadyCondition::providedPorts()
{
  return {BT::InputPort<bool>("require_no_override", true,
                              "Fail if manual override is active"),
          BT::InputPort<bool>("require_no_failure", true,
                              "Fail if docking failure has been latched")};
}

BT::NodeStatus DeployReadyCondition::tick()
{
  const auto snapshot = mission_state_->snapshot();
  const bool require_no_override = getInput<bool>("require_no_override").value_or(true);
  const bool require_no_failure = getInput<bool>("require_no_failure").value_or(true);

  if (!snapshot.deploy || snapshot.should_return)
  {
    return BT::NodeStatus::FAILURE;
  }
  if (require_no_override && snapshot.manual_override)
  {
    return BT::NodeStatus::FAILURE;
  }
  if (require_no_failure && snapshot.docking_failed)
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList ModeCondition::providedPorts()
{
  return {BT::InputPort<std::string>("mode", "", "Required mode string")};
}

BT::NodeStatus ModeCondition::tick()
{
  const auto snapshot = mission_state_->snapshot();
  const auto mode = getInput<std::string>("mode");
  if (!mode || mode->empty())
  {
    return BT::NodeStatus::FAILURE;
  }
  return snapshot.mode == *mode ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList AnyModeCondition::providedPorts()
{
  return {BT::InputPort<std::string>(
      "modes", "",
      "Semicolon separated mode list (e.g. RETURNING;CLOSETODOCKING)")};
}

BT::NodeStatus AnyModeCondition::tick()
{
  const auto snapshot = mission_state_->snapshot();
  const auto modes_input = getInput<std::string>("modes");
  if (!modes_input || modes_input->empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  std::stringstream stream(*modes_input);
  std::string mode;
  while (std::getline(stream, mode, ';'))
  {
    if (trimCopy(mode) == snapshot.mode)
    {
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}

BT::PortsList FlagCondition::providedPorts()
{
  return {BT::InputPort<std::string>("flag", "", "Flag name"),
          BT::InputPort<bool>("value", true, "Expected flag value")};
}

BT::NodeStatus FlagCondition::tick()
{
  const auto snapshot = mission_state_->snapshot();
  const auto flag = getInput<std::string>("flag");
  const bool expected = getInput<bool>("value").value_or(true);
  if (!flag || flag->empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  bool actual = false;
  if (*flag == "stationing")
  {
    actual = snapshot.stationing;
  }
  else if (*flag == "docking_falling")
  {
    actual = snapshot.docking_falling;
  }
  else if (*flag == "constheight")
  {
    actual = snapshot.constheight;
  }
  else if (*flag == "manual_override")
  {
    actual = snapshot.manual_override;
  }
  else if (*flag == "docking_failed")
  {
    actual = snapshot.docking_failed;
  }
  else if (*flag == "deploy")
  {
    actual = snapshot.deploy;
  }
  else if (*flag == "return")
  {
    actual = snapshot.should_return;
  }
  else if (*flag == "docking_phase_active")
  {
    actual = snapshot.docking_phase_active;
  }
  else if (*flag == "optical_valid")
  {
    actual = snapshot.optical_valid;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }

  return actual == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList PhaseCondition::providedPorts()
{
  return {BT::InputPort<int>("phase", std::numeric_limits<int>::min(), "Exact phase"),
          BT::InputPort<int>("min_phase", std::numeric_limits<int>::min(), "Minimum phase"),
          BT::InputPort<int>("max_phase", std::numeric_limits<int>::max(), "Maximum phase")};
}

BT::NodeStatus PhaseCondition::tick()
{
  const auto snapshot = mission_state_->snapshot();
  const int phase = snapshot.phase;

  const int exact_phase = getInput<int>("phase").value_or(std::numeric_limits<int>::min());
  if (exact_phase != std::numeric_limits<int>::min())
  {
    return phase == exact_phase ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  const int min_phase = getInput<int>("min_phase").value_or(std::numeric_limits<int>::min());
  const int max_phase = getInput<int>("max_phase").value_or(std::numeric_limits<int>::max());
  if (phase < min_phase || phase > max_phase)
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList DockingPhaseUpdateCondition::providedPorts()
{
  return {};
}

BT::NodeStatus DockingPhaseUpdateCondition::tick()
{
  const double stamp = currentTime();
  const NavSnapshot nav = nav_state_->snapshot();
  const MissionSnapshot mission = mission_state_->snapshot();

  const DockingPhaseResult result = docking_manager_->tick(ros::Time(stamp), nav, mission);
  applyCommands(result.commands, stamp);

  const MissionSnapshot updated = mission_state_->snapshot();
  const bool docking_active = (updated.mode == "DOCKING" || updated.mode == "CLOSETODOCKING");
  mission_state_->updateDockingTelemetry(result.outputs.phase, docking_manager_->phaseTotal(),
                                         result.outputs.current_depth, result.outputs.inner_radius,
                                         result.outputs.outer_radius, docking_active,
                                         result.outputs.optical_valid, result.outputs.distance);

  outputs_publisher_->publish(result.outputs);
  return BT::NodeStatus::SUCCESS;
}

double DockingPhaseUpdateCondition::currentTime() const
{
  if (config().blackboard)
  {
    try
    {
      return config().blackboard->get<double>("current_time");
    }
    catch (const std::exception &)
    {
    }
  }
  return ros::Time::now().toSec();
}

void DockingPhaseUpdateCondition::applyCommands(const DockingPhaseCommands &commands, double stamp)
{
  auto applyBool = [&](const std::optional<bool> &value, const std::string &key,
                       auto setter) {
    if (!value.has_value())
    {
      return;
    }
    setter(*value);
    helm_adapter_->setBool(key, *value, stamp);
    command_publisher_->publishBool(key, *value);
  };

  if (commands.mode)
  {
    mission_state_->setMode(*commands.mode);
    helm_adapter_->setMode(*commands.mode, stamp);
    command_publisher_->publishString("MODE", *commands.mode);
  }

  applyBool(commands.stationing, "STATIONING", [&](bool value) { mission_state_->setStationing(value); });
  applyBool(commands.constheight, "CONSTHEIGHT", [&](bool value) { mission_state_->setConstHeight(value); });
  applyBool(commands.docking_falling, "DOCKING_FALLING",
            [&](bool value) { mission_state_->setDockingFalling(value); });
  applyBool(commands.manual_override, "MOOS_MANUAL_OVERIDE",
            [&](bool value) { mission_state_->setManualOverride(value); });
  applyBool(commands.docking_failed, "DOCKINGFAILED",
            [&](bool value) { mission_state_->setDockingFailed(value); });

  if (commands.depth_update)
  {
    helm_adapter_->setUpdateVar("DOCKDEPTH_UPDATE", *commands.depth_update, stamp);
    command_publisher_->publishString("DOCKDEPTH_UPDATE", *commands.depth_update);
  }
  if (commands.heading_update)
  {
    helm_adapter_->setUpdateVar("DOCKHDG_UPDATES", *commands.heading_update, stamp);
    command_publisher_->publishString("DOCKHDG_UPDATES", *commands.heading_update);
  }
}

}  // namespace bt_executor
