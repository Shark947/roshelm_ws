#ifndef VARIABLE_EXTRACTOR_DEFAULT_VARIABLE_EXTRACTOR_HPP
#define VARIABLE_EXTRACTOR_DEFAULT_VARIABLE_EXTRACTOR_HPP

#include <ros/ros.h>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

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
 *          - DEPTH   （从 /<vehicle>/pose_gt 中提取 z 轴负值）
 *          - SPEED   （从 /<vehicle>/dvl 中提取水平速度 sqrt(vx^2+vy^2)）
 *          - HEADING （从 /<vehicle>/imu 中提取航向角(取 yaw, 转为 [0,360) )）
 *          - PITCH   （从 /<vehicle>/imu 中提取俯仰角(deg)）
 *          - ROLL    （从 /<vehicle>/imu 中提取滚转角(deg)）
 *          - ALTITUDE（从 /<vehicle>/dvl 中直接读取 altitude）
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
    }
}

  std::vector<std::string> supportedVariables() const override {
    // 插件实际支持的变量列表
    return {"DEPTH", "SPEED", "HEADING", "PITCH", "ROLL", "ALTITUDE"};
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
    if (var_upper == "DEPTH") {
      std::string topic = "/" + vehicle_name_ + "/pose_gt";
      auto cb = [this, var_upper, &var_map, debug](const nav_msgs::Odometry::ConstPtr& msg) {
        double depth = -msg->pose.pose.position.z;  // 取负值，使海平面下 depth>0
        var_map[var_upper] = depth;

        // 发布 Float64Stamped
        common_msgs::Float64Stamped out;
        out.header  = msg->header;
        out.data    = depth;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][DEPTH] published "
                            << depth << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<nav_msgs::Odometry>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else if (var_upper == "SPEED") {
      std::string topic = "/" + vehicle_name_ + "/dvl";
      auto cb = [this, var_upper, &var_map, debug](const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) {
        double vx    = msg->velocity.x;
        double vy    = msg->velocity.y;
        double speed = std::hypot(vx, vy);
        var_map[var_upper] = speed;

        common_msgs::Float64Stamped out;
        out.header.stamp = msg->header.stamp;
        out.data         = speed;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][SPEED] published "
                           << speed << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<uuv_sensor_ros_plugins_msgs::DVL>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else if (var_upper == "HEADING") {
      std::string topic = "/" + vehicle_name_ + "/imu";
      auto cb = [this, var_upper, &var_map, debug](const sensor_msgs::Imu::ConstPtr& msg) {
        const auto& q = msg->orientation;
        tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        double heading = std::fmod((yaw * 180.0/M_PI + 360.0), 360.0);
        var_map[var_upper] = heading;

        common_msgs::Float64Stamped out;
        out.header.stamp = msg->header.stamp;
        out.data         = heading;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][HEADING] published "
                           << heading << " @ " << msg->header.stamp);
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
        double pitch_deg = pitch * 180.0/M_PI;
        var_map[var_upper] = pitch_deg;

        common_msgs::Float64Stamped out;
        out.header.stamp = msg->header.stamp;
        out.data         = pitch_deg;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][PITCH] published "
                           << pitch_deg << " @ " << msg->header.stamp);
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
        double roll_deg = roll * 180.0/M_PI;
        var_map[var_upper] = roll_deg;

        common_msgs::Float64Stamped out;
        out.header.stamp = msg->header.stamp;
        out.data         = roll_deg;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][ROLL] published "
                           << roll_deg << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<sensor_msgs::Imu>(topic, 2, cb);
      subs_.push_back(sub);

    }
    else if (var_upper == "ALTITUDE") {
      std::string topic = "/" + vehicle_name_ + "/dvl";
      auto cb = [this, var_upper, &var_map, debug](const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) {
        double alt = msg->altitude;
        var_map[var_upper] = alt;

        common_msgs::Float64Stamped out;
        out.header.stamp = msg->header.stamp;
        out.data         = alt;
        pubs_[var_upper].publish(out);

        if (debug) {
          ROS_DEBUG_STREAM("[DefaultVarExt][ALTITUDE] published "
                           << alt << " @ " << msg->header.stamp);
        }
      };
      ros::Subscriber sub =
        nh_parent.subscribe<uuv_sensor_ros_plugins_msgs::DVL>(topic, 2, cb);
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
  std::string vehicle_name_;                     ///< 车辆名称
  std::set<std::string> subscribed_vars_;        ///< 已经订阅的变量（大写形式）
  std::vector<ros::Subscriber> subs_;            ///< 保存 Subscriber 以免析构
  std::map<std::string, ros::Publisher> pubs_;   ///< var_upper -> Publisher
};

}  // namespace variable_extractor

#endif  // VARIABLE_EXTRACTOR_DEFAULT_VARIABLE_EXTRACTOR_HPP
