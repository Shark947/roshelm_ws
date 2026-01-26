#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <memory>
#include <string>

#include "ros_behavior_tree/helm_interface.hpp"

class BHV_ConstantSpeed;

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
  std::unique_ptr<BHV_ConstantSpeed> behavior_;
  std::string cached_params_;
};

}  // namespace ros_behavior_tree
