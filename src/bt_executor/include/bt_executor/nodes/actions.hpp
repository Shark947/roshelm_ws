#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>

#include "bt_executor/helm_adapter.hpp"
#include "bt_executor/mission_state.hpp"

namespace bt_executor
{

class ActivateBehaviorAction : public BT::SyncActionNode
{
public:
  ActivateBehaviorAction(const std::string &name, const BT::NodeConfiguration &config,
                         std::shared_ptr<HelmAdapter> helm_adapter)
    : BT::SyncActionNode(name, config), helm_adapter_(std::move(helm_adapter))
  {
  }

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<HelmAdapter> helm_adapter_;
};

class SetModeAction : public BT::SyncActionNode
{
public:
  SetModeAction(const std::string &name, const BT::NodeConfiguration &config,
                std::shared_ptr<MissionState> mission_state,
                std::shared_ptr<HelmAdapter> helm_adapter)
    : BT::SyncActionNode(name, config), mission_state_(std::move(mission_state)),
      helm_adapter_(std::move(helm_adapter))
  {
  }

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<MissionState> mission_state_;
  std::shared_ptr<HelmAdapter> helm_adapter_;
};

class SetFlagAction : public BT::SyncActionNode
{
public:
  SetFlagAction(const std::string &name, const BT::NodeConfiguration &config,
                std::shared_ptr<MissionState> mission_state,
                std::shared_ptr<HelmAdapter> helm_adapter)
    : BT::SyncActionNode(name, config), mission_state_(std::move(mission_state)),
      helm_adapter_(std::move(helm_adapter))
  {
  }

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<MissionState> mission_state_;
  std::shared_ptr<HelmAdapter> helm_adapter_;
};

}  // namespace bt_executor
