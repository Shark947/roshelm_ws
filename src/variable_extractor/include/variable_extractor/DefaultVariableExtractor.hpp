#ifndef VARIABLE_EXTRACTOR_DEFAULT_VARIABLE_EXTRACTOR_HPP
#define VARIABLE_EXTRACTOR_DEFAULT_VARIABLE_EXTRACTOR_HPP

#include <ros/ros.h>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <tf/transform_datatypes.h>

#include <common_msgs/Float64Stamped.h>                     // 发布用的消息
#include "variable_extractor/VariableExtractorInterface.hpp"  // 插件接口

namespace variable_extractor {

/**
 * @brief DefaultVariableExtractor：VariableExtractorInterface 的“默认”实现，
 *        自动订阅以下变量：
 *          - X  （从 /<vehicle>/pose_gt 中提取 x 轴原值）
 *          - Y  （从 /<vehicle>/pose_gt 中提取 y 轴原值）
 *          - Z  （从 /<vehicle>/pose_gt 中提取 z 轴原值）
 *          - VX （从 /<vehicle>/dvl 中提取 x 方向速度）
 *          - VY （从 /<vehicle>/dvl 中提取 y 方向速度）
 *          - SPEED （由 VX/VY 计算得到的速度大小）
 *          - YAW   （从 /<vehicle>/imu 中提取航向角(rad)）
 *          - PITCH （从 /<vehicle>/imu 中提取俯仰角(rad)）
 *          - ROLL  （从 /<vehicle>/imu 中提取滚转角(rad)）
 *        提取后，将数值写入外部传入的 var_map[var_name]，同时发布到
 *        /<vehicle>/current_<var_name> 话题（消息类型：common_msgs/Float64Stamped）。
 */
class DefaultVariableExtractor : public VariableExtractorInterface {
public:
  /// 必须提供 public 无参构造，供 pluginlib 使用
  DefaultVariableExtractor() = default;
  ~DefaultVariableExtractor() override = default;

  // 禁止拷贝
  DefaultVariableExtractor(const DefaultVariableExtractor&) = delete;
  DefaultVariableExtractor& operator=(const DefaultVariableExtractor&) = delete;

  // 实现接口：
  void setVehicleName(const std::string& vn) override {
    ROS_INFO_STREAM("[DefaultVarExt] setVehicleName called with: [" << vn << "]");
    if (vehicle_name_ != vn) {
      vehicle_name_ = vn;
      pubs_.clear();
      subscribed_vars_.clear();
      subs_.clear();
      speed_pub_ = ros::Publisher();
      speed_pub_ready_ = false;
      have_vx_ = false;
      have_vy_ = false;
    }
}

  std::vector<std::string> supportedVariables() const override {
    // 插件实际支持的变量列表
    return {"X", "Y", "Z", "VX", "VY", "SPEED", "YAW", "PITCH", "ROLL"};
}

  void subscribe(const std::string& var_name_in,
                 ros::NodeHandle& nh_parent,
                 std::map<std::string, double>& var_map,
                 bool debug) override
  {
    // 1) 变量名称转大写，作为内部key
    std::string var_upper = var_name_in;
    std::transform(var_upper.begin(), var_upper.end(), var_upper.begin(),
                   [](unsigned char c){ return std::toupper(c); });
    // 变量小写，用于拼 topic 名： current_<var_lower>
    std::string var_lower = var_name_in;
    std::transform(var_lower.begin(), var_lower.end(), var_lower.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    // 已经订阅过就直接返回
    if (subscribed_vars_.count(var_upper)) {
      return;
    }
    subscribed_vars_.insert(var_upper);

    // 2) 创建发布器：/<vehicle_name>/current_<var_lower>
    std::string pub_topic = "/" + vehicle_name_ + "/current_" + var_lower;
    ROS_INFO_STREAM("[DefaultVarExt] Advertising topic: " << pub_topic);
    ros::Publisher pub =
      nh_parent.advertise<common_msgs::Float64Stamped>(pub_topic, 1);
    pubs_[var_upper] = pub;

    // 3) 在 var_map 中初始化一个 NaN
    var_map[var_upper] = std::numeric_limits<double>::quiet_NaN();

    // 4) 根据 var_upper 选择订阅逻辑
    if (var_upper == "X") {
      std::string topic = "/" + vehicle_name_ + "/pose_gt";
      auto cb = [this, var_upper, &var_map, debug](const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        var_map[var_upper] = x;

        // 发布 Float64Stamped
        common_msgs::Float64Stamped out;
        out.header  = msg->header;
        out.data    = x;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][X] published "
                            << x << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<nav_msgs::Odometry>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else if (var_upper == "Y") {
      std::string topic = "/" + vehicle_name_ + "/pose_gt";
      auto cb = [this, var_upper, &var_map, debug](const nav_msgs::Odometry::ConstPtr& msg) {
        double y = msg->pose.pose.position.y;
        var_map[var_upper] = y;

        common_msgs::Float64Stamped out;
        out.header  = msg->header;
        out.data    = y;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][Y] published "
                           << y << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<nav_msgs::Odometry>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else if (var_upper == "Z") {
      std::string topic = "/" + vehicle_name_ + "/pose_gt";
      auto cb = [this, var_upper, &var_map, debug](const nav_msgs::Odometry::ConstPtr& msg) {
        double z = msg->pose.pose.position.z;
        var_map[var_upper] = z;

        common_msgs::Float64Stamped out;
        out.header  = msg->header;
        out.data    = z;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][Z] published "
                           << z << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<nav_msgs::Odometry>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else if (var_upper == "VX") {
      std::string topic = "/" + vehicle_name_ + "/dvl";
      auto cb = [this, var_upper, &var_map, debug](const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) {
        double vx = msg->velocity.x;
        var_map[var_upper] = vx;
        latest_vx_ = vx;
        latest_vy_ = msg->velocity.y;
        have_vx_ = true;
        have_vy_ = true;

        common_msgs::Float64Stamped out;
        out.header.stamp = msg->header.stamp;
        out.data         = vx;
        pubs_[var_upper].publish(out);

        updateSpeed(var_map, debug);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][VX] published "
                           << vx << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<uuv_sensor_ros_plugins_msgs::DVL>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else if (var_upper == "VY") {
      std::string topic = "/" + vehicle_name_ + "/dvl";
      auto cb = [this, var_upper, &var_map, debug](const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) {
        double vy = msg->velocity.y;
        var_map[var_upper] = vy;
        latest_vx_ = msg->velocity.x;
        latest_vy_ = vy;
        have_vx_ = true;
        have_vy_ = true;

        common_msgs::Float64Stamped out;
        out.header.stamp = msg->header.stamp;
        out.data         = vy;
        pubs_[var_upper].publish(out);

        updateSpeed(var_map, debug);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][VY] published "
                           << vy << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<uuv_sensor_ros_plugins_msgs::DVL>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else if (var_upper == "SPEED") {
      // 速度由 VX/VY 计算得到，这里仅保证变量存在
      if (!have_vx_ || !have_vy_) {
        ROS_WARN_STREAM("[DefaultVarExt] SPEED unavailable until VX/VY are received.");
      }
    }
    else if (var_upper == "YAW") {
      std::string topic = "/" + vehicle_name_ + "/imu";
      auto cb = [this, var_upper, &var_map, debug](const sensor_msgs::Imu::ConstPtr& msg) {
        const auto& q = msg->orientation;
        tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        var_map[var_upper] = yaw;

        common_msgs::Float64Stamped out;
        out.header.stamp = msg->header.stamp;
        out.data         = yaw;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][YAW] published "
                           << yaw << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<sensor_msgs::Imu>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else if (var_upper == "PITCH") {
      std::string topic = "/" + vehicle_name_ + "/imu";
      auto cb = [this, var_upper, &var_map, debug](const sensor_msgs::Imu::ConstPtr& msg) {
        const auto& q = msg->orientation;
        tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        var_map[var_upper] = pitch;

        common_msgs::Float64Stamped out;
        out.header.stamp = msg->header.stamp;
        out.data         = pitch;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][PITCH] published "
                           << pitch << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<sensor_msgs::Imu>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else if (var_upper == "ROLL") {
      std::string topic = "/" + vehicle_name_ + "/imu";
      auto cb = [this, var_upper, &var_map, debug](const sensor_msgs::Imu::ConstPtr& msg) {
        const auto& q = msg->orientation;
        tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        var_map[var_upper] = roll;

        common_msgs::Float64Stamped out;
        out.header.stamp = msg->header.stamp;
        out.data         = roll;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][ROLL] published "
                           << roll << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<sensor_msgs::Imu>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else {
      ROS_WARN_STREAM("[DefaultVarExt] 无法识别变量 ["
                      << var_name_in << "]，保持 NaN");
      var_map[var_upper] = std::numeric_limits<double>::quiet_NaN();
      // 发布器已创建但不发布
    }
  }

private:
  void updateSpeed(std::map<std::string, double>& var_map, bool debug) {
    if (!(have_vx_ && have_vy_)) {
      return;
    }
    double speed = std::hypot(latest_vx_, latest_vy_);
    auto it = var_map.find("SPEED");
    if (it != var_map.end()) {
      it->second = speed;
    }
    if (debug) {
      ROS_DEBUG_STREAM("[DefaultVarExt][SPEED] updated " << speed);
    }
  }

  std::string vehicle_name_;                     ///< 车辆名称
  std::set<std::string> subscribed_vars_;        ///< 已经订阅的变量（大写形式）
  std::vector<ros::Subscriber> subs_;            ///< 保存 Subscriber 以免析构
  std::map<std::string, ros::Publisher> pubs_;   ///< var_upper -> Publisher
  double latest_vx_{0.0};
  double latest_vy_{0.0};
  bool have_vx_{false};
  bool have_vy_{false};
};

}  // namespace variable_extractor

#endif  // VARIABLE_EXTRACTOR_DEFAULT_VARIABLE_EXTRACTOR_HPP
