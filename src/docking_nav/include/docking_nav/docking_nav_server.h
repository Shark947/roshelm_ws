#pragma once

#include <functional>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <docking_optical_msgs/OpticalFeedback.h>
#include <docking_optical_msgs/OpticalMeasurement.h>
#include <geometry_msgs/PointStamped.h>

class DockingNavServer
{
public:
  using BoolCommandCallback =
      std::function<void(const std::string &, bool value)>;
  using StringCommandCallback =
      std::function<void(const std::string &, const std::string &)>;
  using DebugLogCallback = std::function<void(const std::string &)>;

  struct Outputs
  {
    int phase{0};
    geometry_msgs::PointStamped optical_xy;
    docking_optical_msgs::OpticalFeedback optical_feedback;
  };

  DockingNavServer();

  bool initialize(ros::NodeHandle &private_nh);
  void setCommandCallbacks(BoolCommandCallback bool_callback,
                           StringCommandCallback string_callback);
  void setDebugLogCallback(DebugLogCallback debug_callback);

  void setOpticalMeasurement(
      const docking_optical_msgs::OpticalMeasurement &msg);
  void setNavHeading(double heading_deg);
  void setNavDepth(double depth_m);
  void setDockDepth(double depth_m);
  void setNavPitch(double pitch_rad);
  void setNavRoll(double roll_rad);
  void setDesiredSpeed(double speed_mps);
  void setModeFromExternal(const std::string &mode);

  Outputs update(const ros::Time &stamp);

  double navServerPeriod() const { return nav_server_period_; }
  double opticalTimeoutSec() const { return optical_timeout_sec_; }

private:
  struct AlignDepth
  {
    double depth{0.0};
    double inner_radius{0.0};
    double outer_radius{0.0};
  };

  bool loadParams(ros::NodeHandle &private_nh);
  void handleDocking(const ros::Time &stamp, Outputs &outputs);
  void setMode(const std::string &mode, bool force = false);
  void setModeInternal(const std::string &mode, bool publish, bool force);
  bool shouldAutoEnterCloseToDocking(double &distance) const;

  void injectDepthUpdate(double depth);
  void injectHeadingUpdate(const std::string &update);
  void injectBool(const std::string &key, bool value);
  void injectString(const std::string &key, const std::string &value);

  void fillPhase(Outputs &outputs) const;
  void fillOpticalXY(Outputs &outputs, double x, double y,
                     const ros::Time &stamp) const;
  void fillFeedback(Outputs &outputs, double depth_error, double heading,
                    double next_x, double next_y, const ros::Time &stamp);

  BoolCommandCallback bool_callback_;
  StringCommandCallback string_callback_;
  DebugLogCallback debug_log_callback_;

  double nav_server_period_{0.1};
  double optical_timeout_sec_{0.5};

  double dfDockHeading_{0.0};
  double dfDockPitch_{0.0};
  double dfDockRoll_{0.0};
  double dfIMURollAmountBias_{0.0};
  double dfIMUPitchAmountBias_{0.0};
  double m_dfDepthBias_{0.0};
  double m_dfDistanceBias_{0.3};
  double dfAngleBias_{0.5};
  double dfFallDepth_{0.0};
  double dfCameraViewAngle_{25.0};
  int nMaxtryMinuteNum_{1};
  int nConstantDepthMinuteNum_{5};
  double m_dfDepthGroundBias_{0.0};
  double m_dfDepthCameraBias_{0.0};
  double m_dfDockPanel_{0.0};
  double m_nLastPhaseFloatTime_{500.0};
  int m_nTransimitDuration_{30};
  int m_nDockingMaxTry_{3};

  std::vector<AlignDepth> dfvAlignDepth_;
  int nPhaseNum_{1};

  std::string mode_{"LIFT"};
  double dfOpticalNavLoc_[3]{0.0, 0.0, 0.0};
  double dfNavDepth_{0.0};
  double dfNavHeading_{0.0};
  double dfNavPitch_{0.0};
  double dfNavRoll_{0.0};
  double desired_speed_{0.0};
  double dfDockDepth_{0.0};
  double dfDeltaL_{0.0};
  double dfDX_{0.0};
  double dfDY_{0.0};
  double dfInnerRadius_{0.0};
  double dfOuterRadius_{0.0};
  double dfCurrentDepth_{0.0};
  double dradius_{0.0};
  double dinheading_{0.0};
  double distance_{0.0};
  double previousdistance_{0.0};
  double radius1th_{0.0};
  bool bDataFlag_{false};
  double dfLastIMUX_{0.0};
  double dfLastIMUY_{0.0};
  double dfCurrentIMUX_{0.0};
  double dfCurrentIMUY_{0.0};
  int nPhaseCount_{0};
  bool bRetryLastPhase_{false};
  double dfCameraDepth_{0.0};
  double dfLightDepth_{0.0};
  bool bDockingPhase_{false};
  int nCntuDockingCount_{0};
  int nDockingTryCount_{0};
  int nRetryLastPhase_{0};
  int nFloatIteration_{0};
  int nCtnuInvalidDataCount_{0};
  int nConstantDepthCount_{0};
  bool constheight_{true};
  bool stationing_{true};
  bool docking_falling_{false};
  bool manual_override_{false};
  bool docking_failed_{false};
  double m_dfFreq_{10.0};
  unsigned int m_iterations_{0};

  bool optical_received_{false};
  ros::Time last_optical_stamp_;
  double fallback_x_{0.0};
  double fallback_y_{0.0};

  bool auto_enter_closetodocking_{false};
  double auto_enter_duration_sec_{2.0};
  int auto_enter_count_{0};

  bool last_data_flag_{false};
  int last_phase_count_{0};
  bool last_stationing_{true};
  bool last_constheight_{true};
  bool last_docking_falling_{false};
  bool last_manual_override_{false};
  bool last_docking_failed_{false};
};
