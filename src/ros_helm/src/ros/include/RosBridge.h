#pragma once

#include <fstream>
#include <list>
#include <map>
#include <mutex>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <common_msgs/Float64Stamped.h>
#include <std_msgs/Float64.h>

#include "HelmIvP.h"
#include "HelmMsg.h"
#include "Notify.h"
#include "RosCommandPublisher.h"
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

  void currentValueCallback(const common_msgs::Float64Stamped::ConstPtr &msg,
                            const std::string &nav_key);
  ros::Time getNavStamp() const;
  bool setupLogDirectory();
  void logValue(const std::string &name, double value, const ros::Time &stamp);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  RosNodeConfig config_;

  ros::Subscriber heading_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber x_sub_;
  ros::Subscriber y_sub_;
  ros::Subscriber z_sub_;
  ros::Subscriber vx_sub_;
  ros::Subscriber vy_sub_;
  ros::Subscriber yaw_sub_;
  ros::Subscriber pitch_sub_;
  ros::Subscriber roll_sub_;
  std::unique_ptr<RosCommandPublisher> command_publisher_;

  std::map<std::string, ros::Publisher> desired_scalar_pubs_;

  std::list<HelmMsg> pending_mail_;
  std::mutex mail_mutex_;

  std::string log_directory_;
  std::map<std::string, std::ofstream> log_streams_;
  std::mutex log_mutex_;
  std::size_t log_lines_since_flush_{0};
  ros::Time last_log_flush_time_;
  ros::Time last_nav_stamp_;
  mutable std::mutex nav_stamp_mutex_;
};
