#pragma once

#include <ros/ros.h>

#include <std_msgs/Int32.h>

#include "bt_executor/docking_phase_manager.hpp"

namespace bt_executor
{

class DockingOutputsPublisher
{
public:
  DockingOutputsPublisher() = default;

  void initialize(ros::NodeHandle &nh, const std::string &phase_topic,
                  const std::string &optical_xy_topic,
                  const std::string &optical_feedback_topic)
  {
    phase_pub_ = nh.advertise<std_msgs::Int32>(phase_topic, 10);
    optical_xy_pub_ = nh.advertise<geometry_msgs::PointStamped>(optical_xy_topic, 10);
    optical_feedback_pub_ = nh.advertise<docking_optical_msgs::OpticalFeedback>(optical_feedback_topic, 10);
  }

  void publish(const DockingPhaseOutputs &outputs) const
  {
    if (!phase_pub_)
    {
      return;
    }
    std_msgs::Int32 phase_msg;
    phase_msg.data = outputs.phase;
    phase_pub_.publish(phase_msg);
    optical_xy_pub_.publish(outputs.optical_xy);
    optical_feedback_pub_.publish(outputs.optical_feedback);
  }

private:
  ros::Publisher phase_pub_;
  ros::Publisher optical_xy_pub_;
  ros::Publisher optical_feedback_pub_;
};

}  // namespace bt_executor
