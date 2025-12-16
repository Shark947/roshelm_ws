#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cstring>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <limits>  
#include <pluginlib/class_loader.h>

// 引入自定义消息 Float64Stamped
#include <common_msgs/Float64Stamped.h>

// 引入 VariableExtractor 接口头文件（pluginlib 接口）
#include "variable_extractor/VariableExtractorInterface.hpp"


namespace auh_controller_pid {

class PIDController {
public:
    explicit PIDController(ros::NodeHandle& nh);
    ~PIDController() = default;

private:
    // —— 枚举：支持的被控量
    enum ControlledVariable { HEADING, PITCH, ROLL, DEPTH, SPEED, UNKNOWN };
    ControlledVariable var_;

    // —— 参数成员
    std::string vehicle_name_;   // 从私有命名空间读取
    std::string var_name_;       // 被控量字符串 ("heading"/"depth"/...)
    bool        is_angle_;       // 是否视作角度

    double Kp_, Ki_, Kd_;
    double integral_limit_, output_limit_;

    // —— PID 内部状态
    double integral_;
    double last_error_;
    ros::Time last_time_;
    bool initialized_;

    // —— 差分滤波历史
    std::vector<double> diff_history_;
    size_t diff_index_;

    // —— 期望值 Subscriber
    ros::Subscriber desired_sub_;
    double desired_value_;
    bool   desired_received_;

    // —— 当前值 Subscriber（接收 BehaviorManager 发布的 Float64Stamped）
    ros::Subscriber status_sub_;
    double current_value_;
    bool   current_received_;
    ros::Time current_stamp_;

    // —— PID 输出 Publisher
    ros::Publisher output_pub_;

    // —— NodeHandle 成员：将会在构造时赋值
    ros::NodeHandle nh_;

    // —— Pluginlib Loader 与插件接口指针
    std::shared_ptr<pluginlib::ClassLoader<variable_extractor::VariableExtractorInterface>> extractor_loader_;
    boost::shared_ptr<variable_extractor::VariableExtractorInterface> extractor_;

    // —— 存储所有订阅到的变量最新值
    //     key 都用大写： "HEADING","DEPTH","SPEED","PITCH","ROLL"
    std::map<std::string, double> variable_values_;

    bool debug_{false};

    // —— 回调：收到期望值时
    void desiredCallback(const std_msgs::Float64::ConstPtr& msg);

    // —— 回调：收到 "/<vehicle_name>/current_<var_name>" 时触发控制
    void statusCallback(const common_msgs::Float64Stamped::ConstPtr& msg);

    // —— 真正的控制逻辑
    void controlLoop(const ros::Time& current_time);

    // —— 工具：把角度归一到 [-180,180]
    double normalizeAngle180(double angle_deg);

    // —— 根据 var_name_（字符串），返回对应的枚举 var_
    ControlledVariable parseControlledVariable(const std::string& name) const;

    // —— 通过 pluginlib 加载 VariableExtractor 插件并订阅相应变量
    void setupVariableExtractor();

    // —— 订阅单个变量，例如 "HEADING"、"DEPTH" 等
    void subscribeVariable(const std::string& var_upper, ros::NodeHandle& nh);

    // —— 工具：将字符串转成全大写
    static std::string toUpper(const std::string& s) {
        std::string r = s;
        std::transform(r.begin(), r.end(), r.begin(),
                       [](unsigned char c){ return std::toupper(c); });
        return r;
    }
};

} // namespace auh_controller_pid
