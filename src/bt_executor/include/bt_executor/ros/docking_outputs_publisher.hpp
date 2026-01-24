#pragma once

#include <string>

#include <ros/ros.h>

#include <docking_optical_msgs/OpticalFeedback.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>

#include "bt_executor/adapters/docking_phase_manager.hpp"

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
    optical_feedback_pub_ =
        nh.advertise<docking_optical_msgs::OpticalFeedback>(optical_feedback_topic, 10);
  }

  void publish(const DockingPhaseOutputs &outputs, const ros::Time &stamp) const
  {
    if (!phase_pub_)
    {
      return;
    }

    std_msgs::Int32 phase_msg;
    phase_msg.data = outputs.phase;
    phase_pub_.publish(phase_msg);

    geometry_msgs::PointStamped optical_xy;
    optical_xy.header.stamp = stamp;
    optical_xy.header.frame_id = "docking";
    optical_xy.point.x = outputs.next_x;
    optical_xy.point.y = outputs.next_y;
    optical_xy.point.z = 0.0;
    optical_xy_pub_.publish(optical_xy);

    docking_optical_msgs::OpticalFeedback feedback;
    feedback.header.stamp = stamp;
    feedback.depth_error = outputs.depth_error;
    feedback.nav_heading_deg = outputs.nav_heading_deg;
    feedback.next_x = outputs.next_x;
    feedback.next_y = outputs.next_y;
    feedback.delta_imu_x = outputs.delta_imu_x;
    feedback.delta_imu_y = outputs.delta_imu_y;
    optical_feedback_pub_.publish(feedback);
  }

private:
  ros::Publisher phase_pub_;
  ros::Publisher optical_xy_pub_;
  ros::Publisher optical_feedback_pub_;
};

}  // namespace bt_executor
