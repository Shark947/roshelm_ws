#pragma once

#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <ros/ros.h>
#include <common_msgs/Float64Stamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "HelmIvP.h"
#include "HelmMsg.h"
#include "Notify.h"
#include "RosConfigLoader.h"

class RosBridge
{
public:
  RosBridge(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
            const RosNodeConfig &config);

  bool initialize();

  void deliverPending(HelmIvP &helm);
  void publishDesired(const HelmIvP &helm);

private:
  std::map<std::string, double> collectDesiredDoubles(const HelmIvP &helm) const;

  void enqueueNavValue(const std::string &key, double value,
                       const ros::Time &stamp);
  void enqueueBoolValue(const std::string &key, bool value,
                        const ros::Time &stamp);
  ros::Subscriber subscribeCurrent(const std::string &topic,
                                   const std::string &nav_key);
  ros::Subscriber subscribeBoolean(const std::string &topic,
                                   const std::string &key);

  void currentValueCallback(const common_msgs::Float64Stamped::ConstPtr &msg,
                            const std::string &nav_key);
  void booleanCallback(const std_msgs::Bool::ConstPtr &msg,
                       const std::string &key);
  bool setupLogDirectory();
  void logValue(const std::string &name, double value, const ros::Time &stamp);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  RosNodeConfig config_;

  ros::Subscriber heading_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber deploy_sub_;
  ros::Subscriber return_sub_;

  ros::Publisher deploy_pub_;
  ros::Publisher return_pub_;

  std::map<std::string, ros::Publisher> desired_scalar_pubs_;

  std::deque<HelmMsg> pending_mail_;
  std::mutex mail_mutex_;

  std::string log_directory_;
  std::map<std::string, std::ofstream> log_streams_;
  std::mutex log_mutex_;
};
