#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp_v3/condition_node.h>

#include "bt_executor/command_publisher.hpp"
#include "bt_executor/docking_outputs_publisher.hpp"
#include "bt_executor/docking_phase_manager.hpp"
#include "bt_executor/helm_adapter.hpp"
#include "bt_executor/mission_state.hpp"
#include "bt_executor/nav_state.hpp"

namespace bt_executor
{

class DeployReadyCondition : public BT::ConditionNode
{
public:
  DeployReadyCondition(const std::string &name, const BT::NodeConfig &config,
                       std::shared_ptr<MissionState> mission_state)
    : BT::ConditionNode(name, config), mission_state_(std::move(mission_state))
  {
  }

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  std::shared_ptr<MissionState> mission_state_;
};

class ModeCondition : public BT::ConditionNode
{
public:
  ModeCondition(const std::string &name, const BT::NodeConfig &config,
                std::shared_ptr<MissionState> mission_state)
    : BT::ConditionNode(name, config), mission_state_(std::move(mission_state))
  {
  }

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  std::shared_ptr<MissionState> mission_state_;
};

class AnyModeCondition : public BT::ConditionNode
{
public:
  AnyModeCondition(const std::string &name, const BT::NodeConfig &config,
                   std::shared_ptr<MissionState> mission_state)
    : BT::ConditionNode(name, config), mission_state_(std::move(mission_state))
  {
  }

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  std::shared_ptr<MissionState> mission_state_;
};

class FlagCondition : public BT::ConditionNode
{
public:
  FlagCondition(const std::string &name, const BT::NodeConfig &config,
                std::shared_ptr<MissionState> mission_state)
    : BT::ConditionNode(name, config), mission_state_(std::move(mission_state))
  {
  }

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  std::shared_ptr<MissionState> mission_state_;
};

class PhaseCondition : public BT::ConditionNode
{
public:
  PhaseCondition(const std::string &name, const BT::NodeConfig &config,
                 std::shared_ptr<MissionState> mission_state)
    : BT::ConditionNode(name, config), mission_state_(std::move(mission_state))
  {
  }

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  std::shared_ptr<MissionState> mission_state_;
};

class DockingPhaseUpdateCondition : public BT::ConditionNode
{
public:
  DockingPhaseUpdateCondition(const std::string &name, const BT::NodeConfig &config,
                              std::shared_ptr<MissionState> mission_state,
                              std::shared_ptr<NavState> nav_state,
                              std::shared_ptr<HelmAdapter> helm_adapter,
                              std::shared_ptr<DockingPhaseManager> docking_manager,
                              std::shared_ptr<DockingOutputsPublisher> outputs_publisher,
                              std::shared_ptr<CommandPublisher> command_publisher)
    : BT::ConditionNode(name, config), mission_state_(std::move(mission_state)),
      nav_state_(std::move(nav_state)), helm_adapter_(std::move(helm_adapter)),
      docking_manager_(std::move(docking_manager)),
      outputs_publisher_(std::move(outputs_publisher)),
      command_publisher_(std::move(command_publisher))
  {
  }

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  double currentTime() const;
  void applyCommands(const DockingPhaseCommands &commands, double stamp);

  std::shared_ptr<MissionState> mission_state_;
  std::shared_ptr<NavState> nav_state_;
  std::shared_ptr<HelmAdapter> helm_adapter_;
  std::shared_ptr<DockingPhaseManager> docking_manager_;
  std::shared_ptr<DockingOutputsPublisher> outputs_publisher_;
  std::shared_ptr<CommandPublisher> command_publisher_;
};

class AlwaysSuccessCondition : public BT::ConditionNode
{
public:
  AlwaysSuccessCondition(const std::string &name, const BT::NodeConfig &config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace bt_executor
