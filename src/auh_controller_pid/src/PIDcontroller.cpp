#include <auh_controller_pid/PIDController.h>

namespace auh_controller_pid {

PIDController::PIDController(ros::NodeHandle& nh)
  : nh_(nh),  // 将传入的 NodeHandle 赋给成员 nh_
    integral_(0.0),
    last_error_(0.0),
    diff_index_(0),
    initialized_(false),
    desired_value_(0.0),
    desired_received_(false),
    current_value_(0.0),
    current_received_(false),
    debug_(false)
{
    // 1) 私有命名空间（nh_ 已经是私有命名空间）
    ros::NodeHandle& pnh = nh_;

    // 2) 读取 vehicle_name 与 debug 标志
    pnh.param<std::string>("vehicle_name", vehicle_name_, "auv");
    pnh.param<bool>("enable_debug", debug_, false);

    // 3) 读取 PID 参数
    pnh.param("Kp",             Kp_,              1.0);
    pnh.param("Ki",             Ki_,              0.0);
    pnh.param("Kd",             Kd_,              0.0);
    pnh.param("integral_limit", integral_limit_, 10.0);
    pnh.param("output_limit",   output_limit_,   5.0);
    pnh.param<bool>("is_angle", is_angle_,       false);
    pnh.param<std::string>("controlled_variable", var_name_, "heading");

    // 4) 把 var_name_ 转枚举
    var_ = parseControlledVariable(var_name_);
    if (var_ == UNKNOWN) {
        ROS_WARN_STREAM("[PID] unsupported controlled_variable `"
                        << var_name_ << "`; defaulting to HEADING");
        var_ = HEADING;
    }

    // 5) 差分滤波历史初始化
    diff_history_.assign(10, 0.0);

    // 6) 订阅期望值 "/<vehicle_name>/desired_<var_name>"
    {
        std::string desired_topic = "/" + vehicle_name_ + "/desired_" + var_name_;
        desired_sub_ = nh_.subscribe<std_msgs::Float64>(
            desired_topic, 1, &PIDController::desiredCallback, this);
    }

    ROS_INFO_STREAM("[PIDController] Private namespace: " << pnh.getNamespace());
    ROS_INFO_STREAM("[PIDController] vehicle_name=" << vehicle_name_
                    << ", controlled_variable=" << var_name_);

    // 7) 设置并加载 VariableExtractor 插件、以及订阅对应的传感器变量
    setupVariableExtractor();

    // 8) 订阅 "/<vehicle_name>/current_<VAR>"（来自 VariableExtractor 发布的 Float64Stamped）
    {
        // 把 var_name_ 转为全小写，用于 topic 后缀
        std::string var_lower = var_name_;
        std::transform(var_lower.begin(), var_lower.end(), var_lower.begin(),
                    [](unsigned char c){ return std::tolower(c); });
        std::string status_topic = "/" + vehicle_name_ + "/current_" + var_lower;
        status_sub_ = nh_.subscribe<common_msgs::Float64Stamped>(
            status_topic, 1, &PIDController::statusCallback, this);
    }

    // 9) 创建输出话题 "/<vehicle_name>/pid_<var_name>"
    {
        std::string output_topic = "/" + vehicle_name_ + "/pid_" + var_name_;
        output_pub_ = pnh.advertise<std_msgs::Float64>(output_topic, 1);
    }
}

PIDController::ControlledVariable
PIDController::parseControlledVariable(const std::string& name) const {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    if      (lower == "heading") return HEADING;
    else if (lower == "pitch")   return PITCH;
    else if (lower == "roll")    return ROLL;
    else if (lower == "depth")   return DEPTH;
    else if (lower == "speed")   return SPEED;
    else                         return UNKNOWN;
}

void PIDController::setupVariableExtractor() {
    // 1) 创建 pluginlib loader，用来加载 VariableExtractorInterface 实现
    extractor_loader_.reset(
      new pluginlib::ClassLoader<variable_extractor::VariableExtractorInterface>(
          "variable_extractor",                                    // 插件所在的 package
          "variable_extractor::VariableExtractorInterface"       // 插件基类的 fully qualified name
      )
    );

    // 2) 从参数读取要用哪个插件（可在 launch / YAML 里修改）
    std::string extractor_plugin_name;
    ros::NodeHandle pnh("~");
    pnh.param<std::string>(
      "variable_extractor_plugin",
      extractor_plugin_name,
      std::string("variable_extractor/DefaultVariableExtractor")
    );
    
    // 3) **推荐用 createInstance()**，它返回一个 boost::shared_ptr<VariableExtractorInterface>
    try {
        extractor_ = extractor_loader_->createInstance(extractor_plugin_name);
    }
    catch (pluginlib::PluginlibException& ex) {
        ROS_ERROR_STREAM("[PID] Failed to load VariableExtractor plugin ["
                        << extractor_plugin_name << "]: "
                        << ex.what());
        throw;
    }

    // 4) 让插件内部拼接 topic 的 vehicle_name 前缀
    extractor_->setVehicleName(vehicle_name_);

    // 5) 初始化 variable_values_ map：先插入所有可能用到的 key，并赋 NaN
    //    key 使用全大写，与插件内部一致
    variable_values_["HEADING"] = std::numeric_limits<double>::quiet_NaN();
    variable_values_["PITCH"]   = std::numeric_limits<double>::quiet_NaN();
    variable_values_["ROLL"]    = std::numeric_limits<double>::quiet_NaN();
    variable_values_["DEPTH"]   = std::numeric_limits<double>::quiet_NaN();
    variable_values_["SPEED"]   = std::numeric_limits<double>::quiet_NaN();

    // 6) 订阅本控制器关注的那个变量
    switch (var_) {
        case HEADING: subscribeVariable("HEADING", nh_); break;
        case PITCH:   subscribeVariable("PITCH",   nh_); break;
        case ROLL:    subscribeVariable("ROLL",    nh_); break;
        case DEPTH:   subscribeVariable("DEPTH",   nh_); break;
        case SPEED:   subscribeVariable("SPEED",   nh_); break;
        default:      break;
    }
}

void PIDController::subscribeVariable(const std::string& var_upper, ros::NodeHandle& nh) {
    // extractor_->subscribe(var_name, nodehandle, variable_values_map, debug_flag)
    extractor_->subscribe(var_upper, nh, variable_values_, debug_);
    if (debug_) {
        ROS_DEBUG_STREAM("[PID] Called VariableExtractor.subscribe(" << var_upper << ")");
    }
}

void PIDController::desiredCallback(const std_msgs::Float64::ConstPtr& msg) {
    if (is_angle_) {
        desired_value_ = normalizeAngle180(msg->data);
    } else {
        desired_value_ = msg->data;
    }
    desired_received_ = true;
}

void PIDController::statusCallback(const common_msgs::Float64Stamped::ConstPtr& msg) {
    // 插件已经内部把 msg->data 写入 variable_values_[var_upper]，
    // 这里我们仍把它保存在 current_value_ 以便做控制计算
    current_value_     = msg->data;
    current_received_  = true;
    current_stamp_     = msg->header.stamp;
    controlLoop(current_stamp_);
}

void PIDController::controlLoop(const ros::Time& current_time) {
    // 1) 检查期望值是否已收到
    if (!desired_received_) {
        ROS_WARN_STREAM_THROTTLE(5.0,
            "[PID " << var_name_ << "] Waiting for desired value...");
        return;
    }
    // 2) 检查当前值是否已收到
    if (!current_received_) {
        ROS_WARN_STREAM_THROTTLE(5.0,
            "[PID " << var_name_ << "] Waiting for sensor value...");
        return;
    }

    // 3) 计算误差
    double error = 0.0;
    if (is_angle_) {
        double c = normalizeAngle180(current_value_);
        double d = normalizeAngle180(desired_value_);
        error = normalizeAngle180(c - d);
    } else {
        // 非角度型量使用常规的期望-当前值，确保正期望输出对应正推力
        error = desired_value_ - current_value_;
    }

    // 4) 计算 dt
    double dt = (current_time - last_time_).toSec();
    if (!initialized_ || dt <= 0.0) {
        last_time_   = current_time;
        initialized_ = true;
        last_error_  = error;
        return;
    }
    last_time_ = current_time;

    // 5) 积分限幅
    integral_ += Ki_ * error * dt;
    if (std::abs(integral_) > integral_limit_) {
        integral_ = std::copysign(integral_limit_, integral_);
    }

    // 6) 微分滑窗平均
    double diff_now = (error - last_error_) / dt;
    diff_history_[diff_index_ % diff_history_.size()] = diff_now;
    diff_index_++;
    size_t win = std::min(diff_index_, diff_history_.size());
    double diff_sum = 0.0;
    for (size_t i = 0; i < win; ++i) {
        diff_sum += diff_history_[i];
    }
    double diff_avg = diff_sum / win;

    // 7) PID 输出与限幅
    double output = Kp_ * error + integral_ + Kd_ * diff_avg;
    if (std::abs(output) > output_limit_) {
        output = std::copysign(output_limit_, output);
    }

    // 8) 发布（角度类取反）
    std_msgs::Float64 out_msg;
    if (var_ == HEADING || var_ == ROLL || var_ == PITCH) {
        out_msg.data = -output;
    } else {
        out_msg.data = output;
    }
    output_pub_.publish(out_msg);

    // 9) 日志（1秒打印一次）
    ROS_INFO_STREAM_THROTTLE(1.0,
        "[PID " << var_name_
      << "] desired=" << desired_value_
      << ", current=" << current_value_
      << ", error="   << error
      << ", output="  << out_msg.data);

    last_error_       = error;
    current_received_ = false;
}

double PIDController::normalizeAngle180(double angle_deg) {
    angle_deg = std::fmod(angle_deg, 360.0);
    if (angle_deg > 180.0)  angle_deg -= 360.0;
    if (angle_deg <= -180.0) angle_deg += 360.0;
    return angle_deg;
}

} // namespace auh_controller_pid

// 主函数保持不变
int main(int argc, char** argv) {
    ros::init(argc, argv, "PIDcontroller");
    ros::NodeHandle pnh("~");
    auh_controller_pid::PIDController controller(pnh);
    ros::spin();
    return 0;
}
