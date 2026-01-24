#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "bt_executor/adapters/docking_phase_manager.hpp"
#include "bt_executor/bt_context.hpp"
#include "bt_executor/bt_executor_core.hpp"

namespace bt_executor
{

class RosIO;

class BTExecutorNode
{
public:
  BTExecutorNode(ros::NodeHandle nh, ros::NodeHandle private_nh);

  bool initialize();

private:
  bool loadParameters();
  bool resolveTreePath();
  bool loadPlugins();
  bool loadDockingConfig();

  void onTimerTick(const ros::TimerEvent &);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  double loop_frequency_{10.0};
  std::string tree_path_{"behavior_trees/docking.xml"};
  std::string resolved_tree_path_;
  std::string initial_mode_{"LIFT"};
  bool deploy_default_{false};
  bool return_default_{false};
  std::vector<std::string> plugins_;

  DockingPhaseManagerConfig docking_config_{};

  BTExecutorCore core_;
  std::shared_ptr<BTContext> ctx_;

  ros::Timer timer_;
};

}  // namespace bt_executor
