#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <map>
#include <memory>
#include <string>

#include "ros_behavior_tree/bt_nodes.hpp"

#include "BehaviorSet.h"
#include "HelmEngine.h"
#include "IvPDomain.h"
#include "InfoBuffer.h"
#include "LedgerSnap.h"
#include "VarDataPair.h"

namespace ros_behavior_tree
{

class HelmInterface
{
public:
  HelmInterface(const ros::NodeHandle &nh, const NavDataStore *store);

  bool initialize();
  bool solveForBehavior(IvPBehavior &behavior);
  void deactivateBehavior(IvPBehavior &behavior);
  void publishMissionComplete(const std::string &value);
  void publishFlag(const VarDataPair &flag);
  void publishReturn(bool value);

  const IvPDomain &domain() const { return domain_; }
  std::map<std::string, double> desiredValues() const;

private:
  bool updateInfoBuffer();
  bool parseDomain(const std::string &name, const std::string &value);
  void publishDesired(const HelmReport &report);
  std::string topicForFlag(const std::string &var) const;
  ros::Publisher &publisherForFlag(const std::string &var,
                                   const std::string &type,
                                   std::map<std::string, ros::Publisher> &cache);
  void rebuildBehaviorSet();

  ros::NodeHandle nh_;
  const NavDataStore *store_{nullptr};

  IvPDomain domain_;
  InfoBuffer info_buffer_;
  LedgerSnap ledger_snap_;
  std::unique_ptr<HelmEngine> helm_engine_;
  BehaviorSet behavior_set_;
  std::map<IvPBehavior *, bool> behavior_active_;
  bool behavior_set_dirty_{false};

  ros::Publisher desired_heading_pub_;
  ros::Publisher desired_speed_pub_;
  ros::Publisher desired_depth_pub_;
  ros::Publisher deploy_pub_;
  ros::Publisher return_pub_;
  ros::Publisher mission_pub_;
  std::map<std::string, ros::Publisher> flag_string_pubs_;
  std::map<std::string, ros::Publisher> flag_double_pubs_;
  std::map<std::string, ros::Publisher> flag_bool_pubs_;

  ros::Time start_time_;
  double nav_timeout_{0.0};

  std::string desired_heading_topic_;
  std::string desired_speed_topic_;
  std::string desired_depth_topic_;
  std::string deploy_topic_;
  std::string return_topic_;
  std::string mission_topic_;
  std::string vehicle_name_;
  std::map<std::string, double> desired_values_;
};

}  // namespace ros_behavior_tree
