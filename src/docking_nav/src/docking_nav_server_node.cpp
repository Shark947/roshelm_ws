#include <cmath>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <common_msgs/Float64Stamped.h>
#include <docking_optical_msgs/OpticalMeasurement.h>
#include <docking_optical_msgs/OpticalFeedback.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include "HelmIvP.h"
#include "HelmVariableInjector.h"
#include "RosBridge.h"
#include "RosConfigLoader.h"
#include "RosNavPublisher.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double degToRad(double deg)
{
  return deg * kPi / 180.0;
}

double angle360(double deg)
{
  double normalized = std::fmod(deg, 360.0);
  if (normalized < 0.0)
  {
    normalized += 360.0;
  }
  return normalized;
}

double distToPoint(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

struct AlignDepth
{
  double depth{0.0};
  double inner_radius{0.0};
  double outer_radius{0.0};
};
}  // namespace

class DockingNavServer
{
public:
  DockingNavServer(ros::NodeHandle nh, ros::NodeHandle private_nh,
                   HelmVariableInjector *helm_injector)
      : nh_(nh),
        private_nh_(private_nh),
        helm_injector_(helm_injector)
  {
  }

  bool initialize()
  {
    if (!loadParams())
    {
      return false;
    }

    optical_sub_ = nh_.subscribe(
        optical_measurement_topic_, 10,
        &DockingNavServer::opticalCallback, this);

    heading_sub_ = nh_.subscribe(nav_heading_topic_, 10,
                                 &DockingNavServer::headingCallback, this);
    depth_sub_ = nh_.subscribe(nav_depth_topic_, 10,
                               &DockingNavServer::depthCallback, this);
    pitch_sub_ = nh_.subscribe(nav_pitch_topic_, 10,
                               &DockingNavServer::pitchCallback, this);
    roll_sub_ = nh_.subscribe(nav_roll_topic_, 10,
                              &DockingNavServer::rollCallback, this);
    desired_speed_sub_ = nh_.subscribe(
        desired_speed_topic_, 10, &DockingNavServer::desiredSpeedCallback, this);
    phase_pub_ = nh_.advertise<std_msgs::Int32>(phase_topic_, 10);
    optical_xy_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
        optical_xy_topic_, 10);
    optical_feedback_pub_ =
        nh_.advertise<docking_optical_msgs::OpticalFeedback>(
            optical_feedback_topic_, 10);

    const ros::Duration period(nav_server_period_);
    timer_ = nh_.createTimer(period, &DockingNavServer::timerCallback, this);

    setMode(mode_, true);
    injectBool("CONSTHEIGHT", constheight_);
    injectBool("STATIONING", stationing_);
    injectBool("DOCKING_FALLING", docking_falling_);
    injectBool("MOOS_MANUAL_OVERIDE", manual_override_);
    injectBool("DOCKINGFAILED", docking_failed_);
    return true;
  }

private:
  bool loadParams()
  {
    if (!private_nh_.getParam("nav_server_period", nav_server_period_))
    {
      ROS_ERROR("[docking_nav] nav_server_period not set");
      return false;
    }
    private_nh_.param("optical_camera_delta_l", dfDeltaL_, 0.0);
    private_nh_.param("camera_view_angle_deg", dfCameraViewAngle_, 25.0);
    private_nh_.param("dock_depth", dfDockDepth_, 0.0);
    private_nh_.param("dock_panel", m_dfDockPanel_, 0.0);
    private_nh_.param("dock_heading_deg", dfDockHeading_, 0.0);
    private_nh_.param("dock_pitch_deg", dfDockPitch_, 0.0);
    private_nh_.param("dock_roll_deg", dfDockRoll_, 0.0);
    private_nh_.param("depth_bias", m_dfDepthBias_, 0.0);
    private_nh_.param("depth_ground_bias", m_dfDepthGroundBias_, 0.0);
    private_nh_.param("depth_camera_bias", m_dfDepthCameraBias_, 0.0);
    private_nh_.param("fall_depth", dfFallDepth_, 0.0);
    private_nh_.param("dradius", dradius_, 0.0);
    private_nh_.param("dinheading_deg", dinheading_, 0.0);
    private_nh_.param("distance_bias", m_dfDistanceBias_, 0.3);
    private_nh_.param("angle_bias_deg", dfAngleBias_, 0.5);
    private_nh_.param("radius1th", radius1th_, 0.0);
    private_nh_.param("maxtry_minute_num", nMaxtryMinuteNum_, 1);
    private_nh_.param("constant_depth_minute_num", nConstantDepthMinuteNum_, 5);
    private_nh_.param("last_phase_float_time_ms", m_nLastPhaseFloatTime_, 500.0);
    private_nh_.param("transimit_duration_sec", m_nTransimitDuration_, 30);
    private_nh_.param("docking_max_try", m_nDockingMaxTry_, 3);
    private_nh_.param("optical_timeout_sec", optical_timeout_sec_, 0.5);
    private_nh_.param("initial_mode", mode_, mode_);

    dfDockHeading_ = angle360(dfDockHeading_ + 90.0);
    m_dfFreq_ = 1.0 / nav_server_period_;

    std::vector<double> align_offsets;
    private_nh_.param("align_depth_offsets", align_offsets,
                      std::vector<double>());
    dfLightDepth_ = dfDockDepth_ - m_dfDockPanel_;

    std::set<double> align_depths;
    for (double offset : align_offsets)
    {
      align_depths.insert(dfLightDepth_ - offset);
    }

    dfvAlignDepth_.clear();
    for (auto it = align_depths.begin(); it != align_depths.end(); ++it)
    {
      const double depth = *it;
      const double outer_radius = (dfLightDepth_ - m_dfDepthBias_ -
                                   m_dfDepthGroundBias_ - depth) *
                                  std::tan(degToRad(dfCameraViewAngle_));
      double inner_radius = dradius_;
      auto next_it = std::next(it);
      if (next_it != align_depths.end())
      {
        inner_radius = (dfLightDepth_ - m_dfDepthBias_ - m_dfDepthGroundBias_ -
                        *next_it) *
                       std::tan(degToRad(dfCameraViewAngle_));
      }
      dfvAlignDepth_.push_back({depth, inner_radius, outer_radius});
    }

    nPhaseNum_ = static_cast<int>(dfvAlignDepth_.size()) + 1;
    return true;
  }

  void opticalCallback(
      const docking_optical_msgs::OpticalMeasurement::ConstPtr &msg)
  {
    last_optical_stamp_ = msg->header.stamp;
    optical_received_ = true;
    dfOpticalNavLoc_[0] = msg->d_heading_deg;
    dfOpticalNavLoc_[1] = msg->theta_x_deg;
    dfOpticalNavLoc_[2] = msg->theta_y_deg;
    fallback_x_ = msg->fallback_x;
    fallback_y_ = msg->fallback_y;
    bDataFlag_ = msg->valid;
  }

  void headingCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    dfNavHeading_ = msg->data;
  }

  void depthCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    dfNavDepth_ = msg->data;
  }

  void pitchCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    dfNavPitch_ = msg->data * 180.0 / kPi;
    dfCurrentIMUY_ = dfNavPitch_;
  }

  void rollCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
  {
    dfNavRoll_ = msg->data * 180.0 / kPi;
    dfCurrentIMUX_ = dfNavRoll_;
  }

  void desiredSpeedCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    desired_speed_ = msg->data;
  }

  void timerCallback(const ros::TimerEvent &)
  {
    ++m_iterations_;
    const ros::Time now = ros::Time::now();

    if (!optical_received_ ||
        (now - last_optical_stamp_).toSec() > optical_timeout_sec_)
    {
      bDataFlag_ = false;
    }

    if (!bDataFlag_)
    {
      dfOpticalNavLoc_[1] = fallback_x_;
      dfOpticalNavLoc_[2] = fallback_y_;
    }

    dfCameraDepth_ = dfNavDepth_ + m_dfDepthBias_ + m_dfDepthCameraBias_;

    if (mode_ == "DOCKING" || mode_ == "CLOSETODOCKING")
    {
      handleDocking(now);
    }
    else
    {
      publishFeedback(1.0, 0.0, 0.0, 0.0);
    }
  }

  void handleDocking(const ros::Time &stamp)
  {
    if (bDataFlag_)
    {
      ++nCntuDockingCount_;
      if (nCntuDockingCount_ > m_dfFreq_ * m_nTransimitDuration_ &&
          !bDockingPhase_ && mode_ == "CLOSETODOCKING")
      {
        bDockingPhase_ = true;
        setMode("DOCKING");
        constheight_ = false;
        injectBool("CONSTHEIGHT", constheight_);
      }
    }
    else
    {
      nCntuDockingCount_ = 0;
    }

    double dfNextX = 0.0;
    double dfNextY = 0.0;
    if (bDataFlag_)
    {
      dfDX_ = (dfLightDepth_ - dfCameraDepth_) *
                  std::tan(degToRad(dfOpticalNavLoc_[1])) *
                  std::sin(degToRad(dfNavHeading_)) -
              (dfLightDepth_ - dfCameraDepth_) *
                  std::tan(degToRad(dfOpticalNavLoc_[2])) *
                  std::cos(degToRad(dfNavHeading_)) -
              dfDeltaL_ * std::sin(degToRad(dfNavHeading_));
      dfDY_ = -(dfLightDepth_ - dfCameraDepth_) *
                  std::tan(degToRad(dfOpticalNavLoc_[2])) *
                  std::sin(degToRad(dfNavHeading_)) -
              (dfLightDepth_ - dfCameraDepth_) *
                  std::tan(degToRad(dfOpticalNavLoc_[1])) *
                  std::cos(degToRad(dfNavHeading_)) +
              dfDeltaL_ * std::cos(degToRad(dfNavHeading_));

      dfNextX = dfDY_;
      dfNextY = dfDX_;
    }
    else
    {
      dfNextX = dfOpticalNavLoc_[1];
      dfNextY = dfOpticalNavLoc_[2];
    }

    distance_ = distToPoint(0, 0, dfNextX, dfNextY);

    if (nPhaseCount_ > 0 && nPhaseCount_ < nPhaseNum_)
    {
      dfCurrentDepth_ = dfvAlignDepth_[nPhaseCount_ - 1].depth;
      injectDepthUpdate(dfCurrentDepth_);

      if (bDataFlag_ && distance_ <= dfOuterRadius_)
      {
        nCtnuInvalidDataCount_ = 0;
        if (distance_ < dfInnerRadius_ &&
            nPhaseCount_ < nPhaseNum_ - 1)
        {
          ++nPhaseCount_;
          nCtnuInvalidDataCount_ = 0;
          dfCurrentDepth_ = dfvAlignDepth_[nPhaseCount_ - 1].depth;
          injectDepthUpdate(dfCurrentDepth_);
          injectHeadingUpdate("pwt=1");
          stationing_ = true;
          injectBool("STATIONING", stationing_);
          dfInnerRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].inner_radius;
          dfOuterRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].outer_radius;
          if (nPhaseCount_ == nPhaseNum_ - 1)
          {
            injectHeadingUpdate("pwt=50");
          }
        }
        else if (bDataFlag_ && nPhaseCount_ == nPhaseNum_ - 1)
        {
          if (distance_ < dfInnerRadius_ * 0.8 && distance_ > 0.3)
          {
            stationing_ = false;
            injectBool("STATIONING", stationing_);
            injectHeadingUpdate(
                "pwt=50,heading=" + std::to_string(dfDockHeading_));
          }
          else if (distance_ > dfInnerRadius_ * 1.2)
          {
            stationing_ = true;
            injectBool("STATIONING", stationing_);
            injectHeadingUpdate("pwt=1");
          }
          if ((desired_speed_ * 7 + distance_) < dradius_)
          {
            if (distance_ < previousdistance_)
            {
              if (std::abs(dfNavHeading_ - dfDockHeading_) < dinheading_)
              {
                nPhaseCount_ = nPhaseNum_;
                dfCurrentDepth_ = dfDockDepth_ + dfFallDepth_;
                injectDepthUpdate(dfCurrentDepth_);
                injectHeadingUpdate(
                    "pwt=200,heading=" + std::to_string(dfDockHeading_));
                stationing_ = false;
                injectBool("STATIONING", stationing_);
                docking_falling_ = true;
                injectBool("DOCKING_FALLING", docking_falling_);
              }
            }
          }
          previousdistance_ = distance_;
        }
      }
      else if (!bDataFlag_ || distance_ > dfOuterRadius_)
      {
        ++nCtnuInvalidDataCount_;
        if (nCtnuInvalidDataCount_ > m_dfFreq_ * 60 * nMaxtryMinuteNum_)
        {
          if (nPhaseCount_ > 1)
          {
            --nPhaseCount_;
            nCtnuInvalidDataCount_ = 0;
            dfCurrentDepth_ = dfvAlignDepth_[nPhaseCount_ - 1].depth;
            injectDepthUpdate(dfCurrentDepth_);
            injectHeadingUpdate("pwt=1");
            stationing_ = true;
            injectBool("STATIONING", stationing_);
            docking_falling_ = false;
            injectBool("DOCKING_FALLING", docking_falling_);
            dfInnerRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].inner_radius;
            dfOuterRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].outer_radius;
          }
          else if (nPhaseCount_ == 1)
          {
            if (nDockingTryCount_ < m_nDockingMaxTry_)
            {
              setMode("CLOSETODOCKING");
              constheight_ = true;
              injectBool("CONSTHEIGHT", constheight_);
              bDockingPhase_ = false;
              ++nDockingTryCount_;
            }
            else
            {
              docking_failed_ = true;
              injectBool("DOCKINGFAILED", docking_failed_);
            }
          }
        }
      }
    }

    if (bDataFlag_)
    {
      previousdistance_ = distance_;
    }

    if (nPhaseCount_ == nPhaseNum_)
    {
      const double depth_error = std::abs(dfLightDepth_ - dfCameraDepth_);
      if (depth_error > m_dfDistanceBias_ ||
          std::abs(dfNavPitch_ - dfIMUPitchAmountBias_ - dfDockPitch_) >
              dfAngleBias_ ||
          std::abs(dfNavRoll_ - dfIMURollAmountBias_ - dfDockRoll_) >
              dfAngleBias_)
      {
        ++nConstantDepthCount_;
        if (nConstantDepthCount_ >
                nConstantDepthMinuteNum_ * 60 * m_dfFreq_ ||
            nRetryLastPhase_ >= 5)
        {
          if (nRetryLastPhase_ >= 5 || depth_error > 1)
          {
            --nPhaseCount_;
            nRetryLastPhase_ = 0;
            bRetryLastPhase_ = false;
            nCtnuInvalidDataCount_ = 0;
            dfCurrentDepth_ = dfvAlignDepth_[nPhaseCount_ - 1].depth;
            injectDepthUpdate(dfCurrentDepth_);
            docking_falling_ = false;
            injectBool("DOCKING_FALLING", docking_falling_);
            dfInnerRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].inner_radius;
            dfOuterRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].outer_radius;
            nConstantDepthCount_ = 0;
          }
          else
          {
            if (nFloatIteration_ <
                m_nLastPhaseFloatTime_ * m_dfFreq_ / 1000)
            {
              ++nFloatIteration_;
              manual_override_ = true;
              injectBool("MOOS_MANUAL_OVERIDE", manual_override_);
            }
            else
            {
              ++nRetryLastPhase_;
              manual_override_ = false;
              injectBool("MOOS_MANUAL_OVERIDE", manual_override_);
              docking_falling_ = true;
              injectBool("DOCKING_FALLING", docking_falling_);
              nConstantDepthCount_ = 0;
              nCtnuInvalidDataCount_ = 0;
              nFloatIteration_ = 0;
            }
          }
        }
      }
      else
      {
        docking_falling_ = false;
        injectBool("DOCKING_FALLING", docking_falling_);
      }
    }

    publishPhase();
    publishOpticalXY(dfNextX, dfNextY, stamp);
    publishFeedback(dfLightDepth_ - dfCameraDepth_, dfNavHeading_, dfNextX,
                    dfNextY);
  }

  void setMode(const std::string &mode, bool force = false)
  {
    if (!force && mode == mode_)
      return;
    mode_ = mode;
    injectString("MODE", mode_);
    if (mode_ == "DOCKING")
    {
      nPhaseCount_ = 1;
      if (!dfvAlignDepth_.empty())
      {
        dfInnerRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].inner_radius;
        dfOuterRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].outer_radius;
        dfCurrentDepth_ = dfvAlignDepth_[nPhaseCount_ - 1].depth;
        injectDepthUpdate(dfCurrentDepth_);
      }
    }
  }

  void injectDepthUpdate(double depth)
  {
    injectString("DOCKDEPTH_UPDATE", "depth=" + std::to_string(depth));
  }

  void injectHeadingUpdate(const std::string &update)
  {
    injectString("DOCKHDG_UPDATES", update);
  }

  void injectBool(const std::string &key, bool value)
  {
    if (!helm_injector_)
      return;
    helm_injector_->queueBool(key, value, ros::Time::now().toSec());
  }

  void injectString(const std::string &key, const std::string &value)
  {
    if (!helm_injector_)
      return;
    helm_injector_->queueString(key, value, ros::Time::now().toSec());
  }

  void publishPhase()
  {
    std_msgs::Int32 msg;
    msg.data = nPhaseCount_;
    phase_pub_.publish(msg);
  }

  void publishOpticalXY(double x, double y, const ros::Time &stamp)
  {
    geometry_msgs::PointStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "docking";
    msg.point.x = x;
    msg.point.y = y;
    msg.point.z = 0.0;
    optical_xy_pub_.publish(msg);
  }

  void publishFeedback(double depth_error, double heading, double next_x,
                       double next_y)
  {
    docking_optical_msgs::OpticalFeedback msg;
    msg.header.stamp = ros::Time::now();
    msg.depth_error =
        depth_error > m_dfDistanceBias_ ? depth_error : m_dfDistanceBias_;
    msg.nav_heading_deg = heading;
    msg.next_x = next_x;
    msg.next_y = next_y;
    msg.delta_imu_x = dfCurrentIMUX_ - dfLastIMUX_;
    msg.delta_imu_y = dfCurrentIMUY_ - dfLastIMUY_;
    optical_feedback_pub_.publish(msg);
    dfLastIMUX_ = dfCurrentIMUX_;
    dfLastIMUY_ = dfCurrentIMUY_;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  HelmVariableInjector *helm_injector_{nullptr};

  ros::Subscriber optical_sub_;
  ros::Subscriber heading_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber pitch_sub_;
  ros::Subscriber roll_sub_;
  ros::Subscriber desired_speed_sub_;

  ros::Publisher phase_pub_;
  ros::Publisher optical_xy_pub_;
  ros::Publisher optical_feedback_pub_;

  ros::Timer timer_;

  double nav_server_period_{0.1};
  double optical_timeout_sec_{0.5};
  std::string optical_measurement_topic_{"/docking/optical_measurement"};
  std::string phase_topic_{"/docking/phase"};
  std::string optical_xy_topic_{"/docking/optical_xy"};
  std::string optical_feedback_topic_{"/docking/optical_feedback"};
  std::string nav_heading_topic_{"/auh/NAV_HEADING"};
  std::string nav_depth_topic_{"/auh/NAV_DEPTH"};
  std::string nav_pitch_topic_{"/auh/NAV_PITCH"};
  std::string nav_roll_topic_{"/auh/NAV_ROLL"};
  std::string desired_speed_topic_{"/auh/desired_speed"};

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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "docking_nav_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  RosNodeConfig config;
  RosConfigLoader loader(private_nh);
  if (!loader.load(config, DOCKING_NAV_DEFAULT_CONFIG_PATH))
  {
    ROS_ERROR("Failed to load docking ROS Helm configuration");
    return 1;
  }

  HelmIvP helm;
  helm.setAppName(config.node_name);
  helm.setConfigPath(config.config_path);

  if (!helm.OnStartUp())
  {
    ROS_ERROR("HelmIvP startup failed");
    return 1;
  }

  RosBridge bridge(nh, private_nh, config);
  if (!bridge.initialize())
  {
    ROS_ERROR("RosBridge initialization failed");
    return 1;
  }

  RosNavPublisher nav_publisher(nh);
  if (!nav_publisher.initialize(config.vehicle_name))
  {
    ROS_ERROR("RosNavPublisher initialization failed");
    return 1;
  }

  HelmVariableInjector injector("docking_nav_server");
  DockingNavServer node(nh, private_nh, &injector);
  if (!node.initialize())
  {
    return 1;
  }

  ros::Rate rate(config.loop_frequency);
  while (ros::ok())
  {
    bridge.deliverPending(helm);
    injector.flush(helm);
    helm.Iterate();
    bridge.publishDesired(helm);
    bridge.logStatusIfNeeded(helm);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
