#pragma once

#include <fstream>
#include <list>
#include <map>
#include <mutex>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <common_msgs/Float64Stamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "HelmIvP.h"
#include "HelmMsg.h"
#include "Notify.h"
#include "RosCommandPublisher.h"
#include "RosConfigLoader.h"

class RosBridge
{
public:
  RosBridge(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
            const RosNodeConfig &config);

  bool initialize();

  void deliverPending(HelmIvP &helm);
  void publishDesired(const HelmIvP &helm);
  void logStatusIfNeeded(const HelmIvP &helm);

private:
  std::map<std::string, double> collectDesiredDoubles(const HelmIvP &helm) const;

protected:
  void enqueueNavValue(const std::string &key, double value,
                       const ros::Time &stamp);
  void enqueueBoolValue(const std::string &key, bool value,
                        const ros::Time &stamp);
  void enqueueStringValue(const std::string &key, const std::string &value,
                          const ros::Time &stamp);
  ros::Subscriber subscribeCurrent(const std::string &topic,
                                   const std::string &nav_key);
  ros::Subscriber subscribeBoolean(const std::string &topic,
                                   const std::string &key);
  ros::Subscriber subscribeString(const std::string &topic,
                                  const std::string &key);
  ros::Time getNavStamp() const;

private:

  void currentValueCallback(const common_msgs::Float64Stamped::ConstPtr &msg,
                            const std::string &nav_key);
  bool setupLogDirectory();
  void logValue(const std::string &name, double value, const ros::Time &stamp);

  struct AttitudeCache
  {
    double yaw{0.0};
    double pitch{0.0};
    double roll{0.0};
    ros::Time stamp_yaw;
    ros::Time stamp_pitch;
    ros::Time stamp_roll;
    bool has_yaw{false};
    bool has_pitch{false};
    bool has_roll{false};

    void reset()
    {
      yaw = 0.0;
      pitch = 0.0;
      roll = 0.0;
      stamp_yaw = ros::Time();
      stamp_pitch = ros::Time();
      stamp_roll = ros::Time();
      has_yaw = false;
      has_pitch = false;
      has_roll = false;
    }
  };

  struct SpeedCache
  {
    double vx{0.0};
    double vy{0.0};
    ros::Time stamp_vx;
    ros::Time stamp_vy;
    bool has_vx{false};
    bool has_vy{false};

    void reset()
    {
      vx = 0.0;
      vy = 0.0;
      stamp_vx = ros::Time();
      stamp_vy = ros::Time();
      has_vx = false;
      has_vy = false;
    }
  };

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  RosNodeConfig config_;

  ros::Subscriber x_sub_;
  ros::Subscriber y_sub_;
  ros::Subscriber z_sub_;
  ros::Subscriber vx_sub_;
  ros::Subscriber vy_sub_;
  ros::Subscriber yaw_sub_;
  ros::Subscriber pitch_sub_;
  ros::Subscriber roll_sub_;
  std::unique_ptr<RosCommandPublisher> command_publisher_;

  std::map<std::string, ros::Publisher> desired_scalar_pubs_;
  std::map<std::string, double> nav_values_;
  std::map<std::string, double> desired_values_;
  std::map<std::string, bool> bool_values_;
  std::mutex status_mutex_;
  ros::Time start_time_;
  ros::Time last_status_log_;
  double status_log_period_{0.0};

  std::list<HelmMsg> pending_mail_;
  std::mutex mail_mutex_;

  std::string log_directory_;
  std::map<std::string, std::ofstream> log_streams_;
  std::mutex log_mutex_;
  std::size_t log_lines_since_flush_{0};
  ros::Time last_log_flush_time_;
  ros::Time last_nav_stamp_;
  mutable std::mutex nav_stamp_mutex_;

  AttitudeCache attitude_cache_;
  std::mutex attitude_mutex_;
  SpeedCache speed_cache_;
  std::mutex speed_mutex_;
};
