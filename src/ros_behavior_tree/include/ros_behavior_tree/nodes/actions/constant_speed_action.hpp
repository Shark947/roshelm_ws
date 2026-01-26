#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <memory>
#include <string>

#include "ros_behavior_tree/helm_interface.hpp"

#include "BHV_ConstantSpeed.h"

namespace ros_behavior_tree
{

class ConstantSpeedAction : public BT::StatefulActionNode
{
public:
  ConstantSpeedAction(const std::string &name,
                      const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool ensureBehavior();
  bool configureBehavior(const std::string &params);
  BT::NodeStatus runBehavior();

  HelmInterface *helm_interface_{nullptr};
  class ConstantSpeedBehavior : public BHV_ConstantSpeed
  {
  public:
    explicit ConstantSpeedBehavior(IvPDomain domain) : BHV_ConstantSpeed(domain)
    {
    }

    bool setBehaviorNamePublic(const std::string &name)
    {
      return setBehaviorName(name);
    }
  };

  std::unique_ptr<ConstantSpeedBehavior> behavior_;
  std::string cached_params_;
  bool behavior_active_{false};
};

}  // namespace ros_behavior_tree
