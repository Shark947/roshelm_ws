#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <memory>
#include <string>

#include "ros_behavior_tree/helm_interface.hpp"

#include "BHV_GoToDepth.h"
#include "VarDataPair.h"

namespace ros_behavior_tree
{

class GoToDepthAction : public BT::StatefulActionNode
{
public:
  GoToDepthAction(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool ensureBehavior();
  bool configureBehavior(const std::string &params);
  BT::NodeStatus runBehavior();

  HelmInterface *helm_interface_{nullptr};
  class GoToDepthBehavior : public BHV_GoToDepth
  {
  public:
    explicit GoToDepthBehavior(IvPDomain domain) : BHV_GoToDepth(domain) {}

    void setComplete(std::string post_mortem = "") override
    {
      completion_requested_ = true;
      BHV_GoToDepth::setComplete(post_mortem);
    }

    bool setBehaviorNamePublic(const std::string &name)
    {
      return setBehaviorName(name);
    }

    const std::vector<VarDataPair> &endFlags() const
    {
      return m_end_flags;
    }

    bool consumeCompletionRequest()
    {
      if (!completion_requested_)
        return false;
      completion_requested_ = false;
      return true;
    }

  private:
    bool completion_requested_{false};
  };

  std::unique_ptr<GoToDepthBehavior> behavior_;
  std::string cached_params_;
  bool behavior_active_{false};
  bool end_flags_published_{false};
};

}  // namespace ros_behavior_tree
