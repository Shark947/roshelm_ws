#include "auh_thrust_manager/ThrustManager.hpp"
#include <boost/bind.hpp>
#include <algorithm>

namespace auh_thrust_manager {

ThrustManager::ThrustManager(ros::NodeHandle& nh, const std::string& ns, int thruster_count)
    : auv_ns_(ns), thruster_count_(thruster_count)
{
    input_signals_.resize(5, 0.0);
    input_received_.resize(5, false);

    // 加载控制矩阵
    std::vector<double> flat_matrix;
    if (!nh.getParam("thrust_matrix", flat_matrix) || flat_matrix.size() != 5 * thruster_count_) {
        ROS_ERROR("Failed to load thrust_matrix (%zu elements)", flat_matrix.size());
        ros::shutdown();
        return;
    }

    thrust_matrix_.resize(5, std::vector<double>(thruster_count_));
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < thruster_count_; ++j)
            thrust_matrix_[i][j] = flat_matrix[i * thruster_count_ + j];

    // 加载浮力补偿参数
    nh.param<double>("buoyancy_compensation", buoyancy_compensation_, 0.0);
    if (!nh.getParam("vertical_thrusters", vertical_thruster_indices_)) {
        ROS_WARN("Parameter 'vertical_thrusters' not set, buoyancy compensation will be ignored.");
    }

    // 加载速度线性俯仰补偿参数
    nh.param<double>("pitch_speed_gain", pitch_speed_gain_, 0.0);
    if (!nh.getParam("pitch_speed_up_thrusters", pitch_speed_up_thruster_indices_)) {
        pitch_speed_up_thruster_indices_ = {2, 4};
        ROS_WARN("Parameter 'pitch_speed_up_thrusters' not set, using default [2, 4].");
    }
    if (!nh.getParam("pitch_speed_down_thrusters", pitch_speed_down_thruster_indices_)) {
        pitch_speed_down_thruster_indices_ = {3, 5};
        ROS_WARN("Parameter 'pitch_speed_down_thrusters' not set, using default [3, 5].");
    }

    // 订阅输入控制器话题
    const std::vector<std::string> keys = {"speed", "heading", "depth", "pitch", "roll"};
    for (size_t i = 0; i < keys.size(); ++i) {
        std::string topic = "/" + auv_ns_ + "/desired_" + keys[i];
        subs_.emplace_back(nh.subscribe<std_msgs::Float64>(
            topic, 1, boost::bind(&ThrustManager::inputCallback, this, _1, i)));
    }

    // 设置输出话题
    for (int i = 0; i < thruster_count_; ++i) {
        std::string topic = "/" + auv_ns_ + "/t" + std::to_string(i);
        pubs_.emplace_back(nh.advertise<std_msgs::Float64>(topic, 1));
    }

    // 定时器：20Hz 推力输出
    timer_ = nh.createTimer(ros::Duration(0.05), &ThrustManager::computeThrust, this);
}

void ThrustManager::inputCallback(const std_msgs::Float64::ConstPtr& msg, int index) {
    input_signals_[index] = msg->data;
    input_received_[index] = true;
}

void ThrustManager::computeThrust(const ros::TimerEvent&) {
    std::vector<double> thrust_values(thruster_count_, 0.0);

    bool any_input_received = std::any_of(input_received_.begin(), input_received_.end(), [](bool r) { return r; });
    static bool warned_once = false;
    if (!any_input_received && !warned_once) {
        ROS_WARN("[ThrustManager] No control inputs received yet. Publishing only buoyancy compensation.");
        warned_once = true;
    }

    // 控制项输入
    for (int j = 0; j < thruster_count_; ++j) {
        for (int i = 0; i < 5; ++i) {
            if (input_received_[i])
                thrust_values[j] += input_signals_[i] * thrust_matrix_[i][j];
        }
    }

    // 根据速度的线性俯仰补偿
    if (pitch_speed_gain_ != 0.0 && input_received_[0]) {
        const double pitch_speed_thrust = input_signals_[0] * pitch_speed_gain_;
        for (int idx : pitch_speed_up_thruster_indices_) {
            if (idx >= 0 && idx < thruster_count_)
                thrust_values[idx] += pitch_speed_thrust;
        }
        for (int idx : pitch_speed_down_thruster_indices_) {
            if (idx >= 0 && idx < thruster_count_)
                thrust_values[idx] -= pitch_speed_thrust;
        }
    }

    // 浮力补偿（始终添加）
    if (!vertical_thruster_indices_.empty() && buoyancy_compensation_ != 0.0) {
        double per_thruster = buoyancy_compensation_ / vertical_thruster_indices_.size();
        for (int j : vertical_thruster_indices_)
            thrust_values[j] += per_thruster;
    }

    // 推进器输出
    for (int j = 0; j < thruster_count_; ++j) {
        std_msgs::Float64 msg;
        msg.data = thrust_values[j];
        pubs_[j].publish(msg);
    }
}

}  // namespace auh_thrust_manager

// ----------------------------
// main 函数入口
// ----------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "thrust_manager");
    ros::NodeHandle nh("~");

    std::string auv_ns;
    int thruster_count;

    nh.param<std::string>("auv_ns", auv_ns, "auh");
    nh.param<int>("thruster_count", thruster_count, 6);


    auh_thrust_manager::ThrustManager manager(nh, auv_ns, thruster_count);
    ros::spin();
    return 0;
}
