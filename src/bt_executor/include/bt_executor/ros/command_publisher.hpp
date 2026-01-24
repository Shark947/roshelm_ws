#pragma once

#include <string>
#include <unordered_map>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace bt_executor
{

class CommandPublisher
{
public:
  CommandPublisher() = default;

  void initialize(ros::NodeHandle &nh,
                  const std::unordered_map<std::string, std::string> &bool_topics,
                  const std::unordered_map<std::string, std::string> &string_topics)
  {
    bool_publishers_.clear();
    string_publishers_.clear();

    for (const auto &entry : bool_topics)
    {
      bool_publishers_.emplace(entry.first, nh.advertise<std_msgs::Bool>(entry.second, 10));
    }
    for (const auto &entry : string_topics)
    {
      string_publishers_.emplace(entry.first, nh.advertise<std_msgs::String>(entry.second, 10));
    }
  }

  void publishBool(const std::string &key, bool value) const
  {
    const auto it = bool_publishers_.find(key);
    if (it == bool_publishers_.end())
    {
      return;
    }
    std_msgs::Bool msg;
    msg.data = value;
    it->second.publish(msg);
  }

  void publishString(const std::string &key, const std::string &value) const
  {
    const auto it = string_publishers_.find(key);
    if (it == string_publishers_.end())
    {
      return;
    }
    std_msgs::String msg;
    msg.data = value;
    it->second.publish(msg);
  }

private:
  std::unordered_map<std::string, ros::Publisher> bool_publishers_;
  std::unordered_map<std::string, ros::Publisher> string_publishers_;
};

}  // namespace bt_executor
