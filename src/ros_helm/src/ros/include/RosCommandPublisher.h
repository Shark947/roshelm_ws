#pragma once

#include <functional>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "RosConfigLoader.h"

class RosCommandPublisher
{
public:
  using EnqueueCallback =
      std::function<void(const std::string &, bool, const ros::Time &)>;
  using StampProvider = std::function<ros::Time()>;

  RosCommandPublisher(ros::NodeHandle &nh, const RosNodeConfig &config,
                      EnqueueCallback enqueue_callback,
                      StampProvider stamp_provider);

  bool initialize();

private:
  ros::Subscriber subscribeBoolean(const std::string &topic,
                                   const std::string &key);
  void handleBoolean(const std_msgs::Bool::ConstPtr &msg,
                     const std::string &key);

  ros::NodeHandle nh_;
  const RosNodeConfig &config_;
  EnqueueCallback enqueue_callback_;
  StampProvider stamp_provider_;

  ros::Subscriber deploy_sub_;
  ros::Subscriber return_sub_;
  ros::Publisher deploy_pub_;
  ros::Publisher return_pub_;
};
