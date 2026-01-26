#pragma once

#include <ros/ros.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

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
  bool queryString(const std::string &var, std::string &out) const;
  bool queryDouble(const std::string &var, double &out) const;
  bool queryBool(const std::string &var, bool &out) const;

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
  void updateExternalInfoBuffer();
  void setFlagValue(const std::string &var, const std::string &value);
  void setFlagValue(const std::string &var, double value);
  void setFlagValue(const std::string &var, bool value);
  void onBoolCommand(const std::string &key, const std_msgs::Bool::ConstPtr &msg);
  void onStringCommand(const std::string &key, const std_msgs::String::ConstPtr &msg);
  void onDoubleCommand(const std::string &key, const std_msgs::Float64::ConstPtr &msg);
  void publishModeState(const std::string &key, const std::string &value);
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
  std::map<std::string, ros::Publisher> mode_state_pubs_;

  std::map<std::string, std::string> bool_command_topics_;
  std::map<std::string, std::string> string_command_topics_;
  std::map<std::string, std::string> double_command_topics_;
  std::map<std::string, std::string> mode_state_topics_;
  std::vector<ros::Subscriber> command_subs_;

  struct ExternalBool
  {
    bool value{false};
    ros::Time stamp;
    bool has_value{false};
  };
  struct ExternalDouble
  {
    double value{0.0};
    ros::Time stamp;
    bool has_value{false};
  };
  struct ExternalString
  {
    std::string value;
    ros::Time stamp;
    bool has_value{false};
  };

  std::map<std::string, ExternalBool> external_bools_;
  std::map<std::string, ExternalDouble> external_doubles_;
  std::map<std::string, ExternalString> external_strings_;

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
