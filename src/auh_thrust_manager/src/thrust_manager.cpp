#include "auh_thrust_manager/ThrustManager.hpp"
#include <boost/bind.hpp>
#include <algorithm>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>

namespace auh_thrust_manager {

ThrustManager::ThrustManager(ros::NodeHandle& nh, const std::string& ns, int thruster_count)
    : vehicle_name_(ns),
      thruster_count_(thruster_count),
      current_speed_(0.0),
      current_speed_received_(false)
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

    // 加载线性俯仰控制参数
    if (!nh.getParam("pitch_speed_points", pitch_speed_points_)) {
        pitch_speed_points_ = {0.25, 0.5, 0.75, 1.0};
    }
    if (!nh.getParam("pitch_thrust_points", pitch_thrust_points_)) {
        pitch_thrust_points_ = {0.25, 0.5, 0.75, 1.0};
    }
    if (!nh.getParam("pitch_up_thrusters", pitch_up_thrusters_)) {
        pitch_up_thrusters_ = {2, 3};
    }
    if (!nh.getParam("pitch_down_thrusters", pitch_down_thrusters_)) {
        pitch_down_thrusters_ = {4, 5};
    }
    if (pitch_speed_points_.size() != pitch_thrust_points_.size() || pitch_speed_points_.empty()) {
        ROS_WARN("Pitch speed/thrust points mismatch, linear pitch control disabled.");
        pitch_speed_points_.clear();
        pitch_thrust_points_.clear();
    }

    // 订阅输入控制器话题
    const std::vector<std::string> keys = {"speed", "heading", "depth", "pitch", "roll"};
    for (size_t i = 0; i < keys.size(); ++i) {
        std::string topic = "/" + vehicle_name_ + "/output_" + keys[i];
        subs_.emplace_back(nh.subscribe<std_msgs::Float64>(
            topic, 1, boost::bind(&ThrustManager::inputCallback, this, _1, i)));
    }
    std::string speed_topic = "/" + vehicle_name_ + "/current_speed";
    current_speed_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
        speed_topic, 1, &ThrustManager::currentSpeedCallback, this);

    // 设置输出话题
    for (int i = 0; i < thruster_count_; ++i) {
        std::string topic = "/" + vehicle_name_ + "/thrusters/" + std::to_string(i) + "/input";
        pubs_.emplace_back(nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(topic, 1));
    }

    // 定时器：20Hz 推力输出
    timer_ = nh.createTimer(ros::Duration(0.05), &ThrustManager::computeThrust, this);
}

void ThrustManager::inputCallback(const std_msgs::Float64::ConstPtr& msg, int index) {
    input_signals_[index] = msg->data;
    input_received_[index] = true;
}

void ThrustManager::currentSpeedCallback(const common_msgs::Float64Stamped::ConstPtr& msg) {
    current_speed_ = msg->data;
    current_speed_received_ = true;
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

    // 固定线性俯仰控制：根据 current_speed 线性增加推力，抬头姿态
    if (current_speed_received_ && !pitch_speed_points_.empty()) {
        double pitch_thrust = pitch_thrust_points_.front();
        if (current_speed_ <= pitch_speed_points_.front()) {
            pitch_thrust = pitch_thrust_points_.front();
        } else if (current_speed_ >= pitch_speed_points_.back()) {
            pitch_thrust = pitch_thrust_points_.back();
        } else {
            for (size_t i = 1; i < pitch_speed_points_.size(); ++i) {
                if (current_speed_ <= pitch_speed_points_[i]) {
                    double s0 = pitch_speed_points_[i - 1];
                    double s1 = pitch_speed_points_[i];
                    double t0 = pitch_thrust_points_[i - 1];
                    double t1 = pitch_thrust_points_[i];
                    double ratio = (current_speed_ - s0) / (s1 - s0);
                    pitch_thrust = t0 + ratio * (t1 - t0);
                    break;
                }
            }
        }
        for (int index : pitch_up_thrusters_) {
            if (index >= 0 && index < thruster_count_)
                thrust_values[index] += pitch_thrust;
        }
        for (int index : pitch_down_thrusters_) {
            if (index >= 0 && index < thruster_count_)
                thrust_values[index] -= pitch_thrust;
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
        uuv_gazebo_ros_plugins_msgs::FloatStamped msg;
        msg.header.stamp = ros::Time::now();
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

    std::string vehicle_name;
    int thruster_count;

    nh.param<std::string>("vehicle_name", vehicle_name, "auh");
    nh.param<int>("thruster_count", thruster_count, 6);


    auh_thrust_manager::ThrustManager manager(nh, vehicle_name, thruster_count);
    ros::spin();
    return 0;
}
