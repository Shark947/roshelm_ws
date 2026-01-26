#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <memory>
#include <string>

#include "ros_behavior_tree/bt_nodes.hpp"

#include "BehaviorSet.h"
#include "HelmEngine.h"
#include "IvPDomain.h"
#include "InfoBuffer.h"
#include "LedgerSnap.h"

namespace ros_behavior_tree
{

class HelmInterface
{
public:
  HelmInterface(const ros::NodeHandle &nh, const NavDataStore *store);

  bool initialize();
  bool solveForBehavior(IvPBehavior &behavior);
  void publishMissionComplete(const std::string &value);

  const IvPDomain &domain() const { return domain_; }

private:
  bool updateInfoBuffer();
  bool parseDomain(const std::string &name, const std::string &value);
  void publishDesired(const HelmReport &report);

  ros::NodeHandle nh_;
  const NavDataStore *store_{nullptr};

  IvPDomain domain_;
  InfoBuffer info_buffer_;
  LedgerSnap ledger_snap_;
  std::unique_ptr<HelmEngine> helm_engine_;
  BehaviorSet behavior_set_;
  IvPBehavior *active_behavior_{nullptr};

  ros::Publisher desired_heading_pub_;
  ros::Publisher desired_speed_pub_;
  ros::Publisher desired_depth_pub_;
  ros::Publisher deploy_pub_;
  ros::Publisher return_pub_;
  ros::Publisher mission_pub_;

  ros::Time start_time_;
  double nav_timeout_{0.0};

  std::string desired_heading_topic_;
  std::string desired_speed_topic_;
  std::string desired_depth_topic_;
  std::string deploy_topic_;
  std::string return_topic_;
  std::string mission_topic_;
  std::string vehicle_name_;
};

}  // namespace ros_behavior_tree
