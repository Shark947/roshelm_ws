#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <memory>
#include <string>

#include "ros_behavior_tree/helm_interface.hpp"

#include "BHV_Waypoint.h"

namespace ros_behavior_tree
{

class WaypointAction : public BT::StatefulActionNode
{
public:
  WaypointAction(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool ensureBehavior();
  bool configureBehavior(const std::string &params);
  BT::NodeStatus runBehavior();

  HelmInterface *helm_interface_{nullptr};
  class WaypointBehavior : public BHV_Waypoint
  {
  public:
    explicit WaypointBehavior(IvPDomain domain) : BHV_Waypoint(domain) {}

    bool setBehaviorNamePublic(const std::string &name)
    {
      return setBehaviorName(name);
    }
  };

  std::unique_ptr<WaypointBehavior> behavior_;
  std::string cached_params_;
  std::string last_runnable_state_;
};

}  // namespace ros_behavior_tree
