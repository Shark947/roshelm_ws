#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
#include <common_msgs/Float64Stamped.h>

namespace auh_thrust_manager {

class ThrustManager {
public:
    ThrustManager(ros::NodeHandle& nh, const std::string& ns, int thruster_count);

private:
    std::string vehicle_name_;
    int thruster_count_;
    std::vector<std::vector<double>> thrust_matrix_;

    // 控制量订阅与推进器发布
    std::vector<ros::Subscriber> subs_;
    std::vector<ros::Publisher> pubs_;
    std::vector<double> input_signals_;
    std::vector<bool> input_received_;
    ros::Subscriber current_speed_sub_;
    double current_speed_;
    bool current_speed_received_;

    // 浮力补偿
    double buoyancy_compensation_;
    std::vector<int> vertical_thruster_indices_;

    // 速度线性俯仰补偿
    double pitch_speed_gain_;
    std::vector<int> pitch_speed_up_thruster_indices_;
    std::vector<int> pitch_speed_down_thruster_indices_;

    ros::Timer timer_;

    void inputCallback(const std_msgs::Float64::ConstPtr& msg, int index);
    void currentSpeedCallback(const common_msgs::Float64Stamped::ConstPtr& msg);
    void computeThrust(const ros::TimerEvent&);
};

}  // namespace auh_thrust_manager
