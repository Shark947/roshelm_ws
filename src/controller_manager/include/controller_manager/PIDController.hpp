#pragma once
#include "controller_manager/ControllerBase.hpp"
#include <vector>
#include <yaml-cpp/yaml.h>

/**
 * @file PIDController.hpp
 * @brief PID控制器插件，实现ControllerBase基类，适配controller_manager插件体系。
 */

namespace controller_manager {

/**
 * @class PIDController
 * @brief 标准PID控制器插件。所有参数初始化均依赖ControllerBase::config_结构体。
 */
class PIDController : public ControllerBase {
public:
    PIDController();
    virtual ~PIDController();

    // 禁止拷贝、赋值
    PIDController(const PIDController&) = delete;
    PIDController& operator=(const PIDController&) = delete;

    /**
     * @brief 初始化，加载所有PID相关参数（包括通用config_字段和专属PID参数）。
     * @param config YAML控制器配置节点
     * @param nh ROS1为ros::NodeHandle&，ROS2为rclcpp::Node::SharedPtr
     * @return 初始化是否成功
     */
    bool init(const YAML::Node& config, ControllerNodeHandle nh) override;

    /**
     * @brief 计算PID控制输出。
     * @param current_value 当前值
     * @param desired_value 期望值
     * @param dt 控制周期
     * @return 控制输出
     */
    double update(double current_value, double desired_value, double dt) override;

private:
    // ---------- PID参数 ----------
    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};
    double integral_limit_{0.0};
    double output_limit_{0.0};

    // ---------- PID内部状态 ----------
    double integral_{0.0};
    double last_error_{0.0};
    bool initialized_{false};

    // 差分滤波历史
    std::vector<double> diff_history_;
    size_t diff_index_{0};
};

} // namespace controller_manager
