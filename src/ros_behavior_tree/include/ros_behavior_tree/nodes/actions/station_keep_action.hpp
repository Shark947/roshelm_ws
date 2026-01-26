#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <memory>
#include <string>

#include "ros_behavior_tree/helm_interface.hpp"

#include "BHV_StationKeep.h"
#include "VarDataPair.h"

namespace ros_behavior_tree
{

class StationKeepAction : public BT::StatefulActionNode
{
public:
  StationKeepAction(const std::string &name,
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
  class StationKeepBehavior : public BHV_StationKeep
  {
  public:
    explicit StationKeepBehavior(IvPDomain domain) : BHV_StationKeep(domain) {}

    bool setBehaviorNamePublic(const std::string &name)
    {
      return setBehaviorName(name);
    }

    const std::vector<VarDataPair> &endFlags() const
    {
      return m_end_flags;
    }
  };

  std::unique_ptr<StationKeepBehavior> behavior_;
  std::string cached_params_;
  bool behavior_active_{false};
  bool end_flags_published_{false};
};

}  // namespace ros_behavior_tree
