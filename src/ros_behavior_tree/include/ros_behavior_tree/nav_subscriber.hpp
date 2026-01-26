#pragma once

#include <common_msgs/Float64Stamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <string>

#include "ros_behavior_tree/bt_nodes.hpp"

namespace ros_behavior_tree
{

struct NavTopics
{
  std::string ros_heading;
  std::string ros_speed;
  std::string ros_depth;
  std::string ros_yaw;
  std::string ros_pitch;
  std::string ros_roll;
  std::string ros_x;
  std::string ros_y;
  std::string deploy;
  std::string return_home;
  std::string speed_trigger;

  std::string nav_heading;
  std::string nav_speed;
  std::string nav_depth;
  std::string nav_yaw;
  std::string nav_pitch;
  std::string nav_roll;
  std::string nav_x;
  std::string nav_y;
};

class NavDataSubscriber
{
public:
  NavDataSubscriber(ros::NodeHandle &nh, const NavTopics &topics,
                    NavDataStore &store);

private:
  ros::Subscriber ros_heading_sub_;
  ros::Subscriber ros_speed_sub_;
  ros::Subscriber ros_depth_sub_;
  ros::Subscriber ros_yaw_sub_;
  ros::Subscriber ros_pitch_sub_;
  ros::Subscriber ros_roll_sub_;
  ros::Subscriber ros_x_sub_;
  ros::Subscriber ros_y_sub_;

  ros::Subscriber nav_heading_sub_;
  ros::Subscriber nav_speed_sub_;
  ros::Subscriber nav_depth_sub_;
  ros::Subscriber nav_yaw_sub_;
  ros::Subscriber nav_pitch_sub_;
  ros::Subscriber nav_roll_sub_;
  ros::Subscriber nav_x_sub_;
  ros::Subscriber nav_y_sub_;
  ros::Subscriber deploy_sub_;
  ros::Subscriber return_sub_;
  ros::Subscriber speed_trigger_sub_;
};

}  // namespace ros_behavior_tree
