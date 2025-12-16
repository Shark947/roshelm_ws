#include "controller_manager/PIDController.hpp"
#include <common_tools/YamlUtils.hpp>
#include <cmath>
#include <limits>
#include <algorithm>
#include "controller_manager/RosCompat.hpp"

namespace controller_manager {

PIDController::PIDController() = default;
PIDController::~PIDController() = default;

bool PIDController::init(const YAML::Node& config, ControllerNodeHandle /*nh*/) {

    if (!loadCommonConfig(config)) return false;

    // 读取PID参数(读取多种拼写)
    kp_ = config["Kp"] ? config["Kp"].as<double>() :
        config["kp"] ? config["kp"].as<double>() :
        config["p"]  ? config["p"].as<double>() : 0.0;

    ki_ = config["Ki"] ? config["Ki"].as<double>() :
        config["ki"] ? config["ki"].as<double>() :
        config["i"]  ? config["i"].as<double>() : 0.0;

    kd_ = config["Kd"] ? config["Kd"].as<double>() :
        config["kd"] ? config["kd"].as<double>() :
        config["d"]  ? config["d"].as<double>() : 0.0;

    // 读取 integral_limit 和 output_limit，设置默认值
    integral_limit_ = config["integral_limit"] ? config["integral_limit"].as<double>() : 100.0;
    output_limit_ = config["output_limit"] ? config["output_limit"].as<double>() : 50.0;


    integral_     = 0.0;
    last_error_   = 0.0;
    initialized_  = false;
    diff_index_   = 0;
    diff_history_.assign(10, 0.0);  // 可选：作为参数暴露配置

    ROS_INFO_STREAM("[PIDController-" << getName() << "] Kp=" << kp_ << " Ki=" << ki_ << " Kd=" << kd_
    << " integral_limit=" << integral_limit_ << " output_limit=" << output_limit_ );

    return true;
}

double PIDController::update(double current_value, double desired_value, double dt) {
    if (dt <= 0.0) {
        debugOutput(current_value, desired_value, 0.0, 0.0, dt, "PIDController");
        return 0.0;
    }
    double error = computeError(current_value, desired_value);

    if (!initialized_) {
        last_error_  = error;
        initialized_ = true;
        return 0.0;
    }

    // --- 积分项 ---
    integral_ += ki_ * error * dt;
    if (std::abs(integral_) > integral_limit_) {
        integral_ = std::copysign(integral_limit_, integral_);
    }

    // --- 微分项 ---
    double diff_now = (error - last_error_) / dt;
    diff_history_[diff_index_ % diff_history_.size()] = diff_now;
    diff_index_++;
    size_t win = std::min(diff_index_, diff_history_.size());
    double diff_sum = 0.0;
    for (size_t i = 0; i < win; ++i) diff_sum += diff_history_[i];
    double diff_avg = diff_sum / win;

    // --- PID 输出 ---
    double output = kp_ * error + integral_ + kd_ * diff_avg;
    if (std::abs(output) > output_limit_) output = std::copysign(output_limit_, output);

    output = applyOutputReverse(output);

    debugOutput(current_value, desired_value, error, output, dt, "PIDController");
    last_error_ = error;
    return output;
}

} // namespace controller_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(controller_manager::PIDController, controller_manager::ControllerBase)