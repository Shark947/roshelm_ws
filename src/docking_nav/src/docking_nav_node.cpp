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
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

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

class DockingNavNode
{
public:
  DockingNavNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : nh_(nh), private_nh_(private_nh)
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
        &DockingNavNode::opticalCallback, this);
    mode_sub_ = nh_.subscribe(mode_topic_, 10,
                              &DockingNavNode::modeCallback, this);

    heading_sub_ = nh_.subscribe(nav_heading_topic_, 10,
                                 &DockingNavNode::headingCallback, this);
    depth_sub_ = nh_.subscribe(nav_depth_topic_, 10,
                               &DockingNavNode::depthCallback, this);
    pitch_sub_ = nh_.subscribe(nav_pitch_topic_, 10,
                               &DockingNavNode::pitchCallback, this);
    roll_sub_ = nh_.subscribe(nav_roll_topic_, 10,
                              &DockingNavNode::rollCallback, this);
    desired_speed_sub_ = nh_.subscribe(
        desired_speed_topic_, 10, &DockingNavNode::desiredSpeedCallback, this);

    mode_pub_ = nh_.advertise<std_msgs::String>(mode_topic_, 10, true);
    stationing_pub_ =
        nh_.advertise<std_msgs::Bool>(stationing_topic_, 10, true);
    constheight_pub_ =
        nh_.advertise<std_msgs::Bool>(constheight_topic_, 10, true);
    dockdepth_update_pub_ =
        nh_.advertise<std_msgs::String>(dockdepth_update_topic_, 10);
    dockhdg_updates_pub_ =
        nh_.advertise<std_msgs::String>(dockhdg_updates_topic_, 10);
    docking_falling_pub_ =
        nh_.advertise<std_msgs::Bool>(docking_falling_topic_, 10, true);
    manual_override_pub_ =
        nh_.advertise<std_msgs::Bool>(manual_override_topic_, 10, true);
    docking_failed_pub_ =
        nh_.advertise<std_msgs::Bool>(docking_failed_topic_, 10, true);
    phase_pub_ = nh_.advertise<std_msgs::Int32>(phase_topic_, 10);
    optical_xy_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
        optical_xy_topic_, 10);
    optical_feedback_pub_ =
        nh_.advertise<docking_optical_msgs::OpticalFeedback>(
            optical_feedback_topic_, 10);

    const ros::Duration period(nav_server_period_);
    timer_ = nh_.createTimer(period, &DockingNavNode::timerCallback, this);

    publishMode(mode_);
    publishBool(constheight_pub_, constheight_);
    publishBool(stationing_pub_, stationing_);
    publishBool(docking_falling_pub_, docking_falling_);
    publishBool(manual_override_pub_, manual_override_);
    publishBool(docking_failed_pub_, docking_failed_);
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
    private_nh_.param("optical_measurement_topic", optical_measurement_topic_,
                      std::string("/docking/optical_measurement"));
    private_nh_.param("mode_topic", mode_topic_,
                      std::string("/docking/mode"));
    private_nh_.param("stationing_topic", stationing_topic_,
                      std::string("/docking/stationing"));
    private_nh_.param("constheight_topic", constheight_topic_,
                      std::string("/docking/constheight"));
    private_nh_.param("dockdepth_update_topic", dockdepth_update_topic_,
                      std::string("/docking/dockdepth_update"));
    private_nh_.param("dockhdg_updates_topic", dockhdg_updates_topic_,
                      std::string("/docking/dockhdg_updates"));
    private_nh_.param("docking_falling_topic", docking_falling_topic_,
                      std::string("/docking/docking_falling"));
    private_nh_.param("manual_override_topic", manual_override_topic_,
                      std::string("/docking/manual_override"));
    private_nh_.param("docking_failed_topic", docking_failed_topic_,
                      std::string("/docking/docking_failed"));
    private_nh_.param("phase_topic", phase_topic_,
                      std::string("/docking/phase"));
    private_nh_.param("optical_xy_topic", optical_xy_topic_,
                      std::string("/docking/optical_xy"));
    private_nh_.param("optical_feedback_topic", optical_feedback_topic_,
                      std::string("/docking/optical_feedback"));
    private_nh_.param("nav_heading_topic", nav_heading_topic_,
                      std::string("/auh/NAV_HEADING"));
    private_nh_.param("nav_depth_topic", nav_depth_topic_,
                      std::string("/auh/NAV_DEPTH"));
    private_nh_.param("nav_pitch_topic", nav_pitch_topic_,
                      std::string("/auh/NAV_PITCH"));
    private_nh_.param("nav_roll_topic", nav_roll_topic_,
                      std::string("/auh/NAV_ROLL"));
    private_nh_.param("desired_speed_topic", desired_speed_topic_,
                      std::string("/auh/desired_speed"));

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

  void modeCallback(const std_msgs::String::ConstPtr &msg)
  {
    const std::string new_mode = msg->data;
    if (new_mode == mode_)
    {
      return;
    }
    mode_ = new_mode;
    if (mode_ == "DOCKING")
    {
      nPhaseCount_ = 1;
      if (!dfvAlignDepth_.empty())
      {
        dfInnerRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].inner_radius;
        dfOuterRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].outer_radius;
        dfCurrentDepth_ = dfvAlignDepth_[nPhaseCount_ - 1].depth;
        publishDepthUpdate(dfCurrentDepth_);
      }
    }
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
        publishMode("DOCKING");
        mode_ = "DOCKING";
        constheight_ = false;
        publishBool(constheight_pub_, constheight_);
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
      publishDepthUpdate(dfCurrentDepth_);

      if (bDataFlag_ && distance_ <= dfOuterRadius_)
      {
        nCtnuInvalidDataCount_ = 0;
        if (distance_ < dfInnerRadius_ &&
            nPhaseCount_ < nPhaseNum_ - 1)
        {
          ++nPhaseCount_;
          nCtnuInvalidDataCount_ = 0;
          dfCurrentDepth_ = dfvAlignDepth_[nPhaseCount_ - 1].depth;
          publishDepthUpdate(dfCurrentDepth_);
          publishHeadingUpdate("pwt=1");
          stationing_ = true;
          publishBool(stationing_pub_, stationing_);
          dfInnerRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].inner_radius;
          dfOuterRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].outer_radius;
          if (nPhaseCount_ == nPhaseNum_ - 1)
          {
            publishHeadingUpdate("pwt=50");
          }
        }
        else if (bDataFlag_ && nPhaseCount_ == nPhaseNum_ - 1)
        {
          if (distance_ < dfInnerRadius_ * 0.8 && distance_ > 0.3)
          {
            stationing_ = false;
            publishBool(stationing_pub_, stationing_);
            publishHeadingUpdate(
                "pwt=50,heading=" + std::to_string(dfDockHeading_));
          }
          else if (distance_ > dfInnerRadius_ * 1.2)
          {
            stationing_ = true;
            publishBool(stationing_pub_, stationing_);
            publishHeadingUpdate("pwt=1");
          }
          if ((desired_speed_ * 7 + distance_) < dradius_)
          {
            if (distance_ < previousdistance_)
            {
              if (std::abs(dfNavHeading_ - dfDockHeading_) < dinheading_)
              {
                nPhaseCount_ = nPhaseNum_;
                dfCurrentDepth_ = dfDockDepth_ + dfFallDepth_;
                publishDepthUpdate(dfCurrentDepth_);
                publishHeadingUpdate(
                    "pwt=200,heading=" + std::to_string(dfDockHeading_));
                stationing_ = false;
                publishBool(stationing_pub_, stationing_);
                docking_falling_ = true;
                publishBool(docking_falling_pub_, docking_falling_);
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
            publishDepthUpdate(dfCurrentDepth_);
            publishHeadingUpdate("pwt=1");
            stationing_ = true;
            publishBool(stationing_pub_, stationing_);
            docking_falling_ = false;
            publishBool(docking_falling_pub_, docking_falling_);
            dfInnerRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].inner_radius;
            dfOuterRadius_ = dfvAlignDepth_[nPhaseCount_ - 1].outer_radius;
          }
          else if (nPhaseCount_ == 1)
          {
            if (nDockingTryCount_ < m_nDockingMaxTry_)
            {
              publishMode("CLOSETODOCKING");
              mode_ = "CLOSETODOCKING";
              constheight_ = true;
              publishBool(constheight_pub_, constheight_);
              bDockingPhase_ = false;
              ++nDockingTryCount_;
            }
            else
            {
              docking_failed_ = true;
              publishBool(docking_failed_pub_, docking_failed_);
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
            publishDepthUpdate(dfCurrentDepth_);
            docking_falling_ = false;
            publishBool(docking_falling_pub_, docking_falling_);
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
              publishBool(manual_override_pub_, manual_override_);
            }
            else
            {
              ++nRetryLastPhase_;
              manual_override_ = false;
              publishBool(manual_override_pub_, manual_override_);
              docking_falling_ = true;
              publishBool(docking_falling_pub_, docking_falling_);
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
        publishBool(docking_falling_pub_, docking_falling_);
      }
    }

    publishPhase();
    publishOpticalXY(dfNextX, dfNextY, stamp);
    publishFeedback(dfLightDepth_ - dfCameraDepth_, dfNavHeading_, dfNextX,
                    dfNextY);
  }

  void publishMode(const std::string &mode)
  {
    std_msgs::String msg;
    msg.data = mode;
    mode_pub_.publish(msg);
  }

  void publishDepthUpdate(double depth)
  {
    std_msgs::String msg;
    msg.data = "depth=" + std::to_string(depth);
    dockdepth_update_pub_.publish(msg);
  }

  void publishHeadingUpdate(const std::string &update)
  {
    std_msgs::String msg;
    msg.data = update;
    dockhdg_updates_pub_.publish(msg);
  }

  void publishBool(const ros::Publisher &pub, bool value)
  {
    std_msgs::Bool msg;
    msg.data = value;
    pub.publish(msg);
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

  ros::Subscriber optical_sub_;
  ros::Subscriber mode_sub_;
  ros::Subscriber heading_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber pitch_sub_;
  ros::Subscriber roll_sub_;
  ros::Subscriber desired_speed_sub_;

  ros::Publisher mode_pub_;
  ros::Publisher stationing_pub_;
  ros::Publisher constheight_pub_;
  ros::Publisher dockdepth_update_pub_;
  ros::Publisher dockhdg_updates_pub_;
  ros::Publisher docking_falling_pub_;
  ros::Publisher manual_override_pub_;
  ros::Publisher docking_failed_pub_;
  ros::Publisher phase_pub_;
  ros::Publisher optical_xy_pub_;
  ros::Publisher optical_feedback_pub_;

  ros::Timer timer_;

  double nav_server_period_{0.1};
  double optical_timeout_sec_{0.5};
  std::string optical_measurement_topic_{"/docking/optical_measurement"};
  std::string mode_topic_{"/docking/mode"};
  std::string stationing_topic_{"/docking/stationing"};
  std::string constheight_topic_{"/docking/constheight"};
  std::string dockdepth_update_topic_{"/docking/dockdepth_update"};
  std::string dockhdg_updates_topic_{"/docking/dockhdg_updates"};
  std::string docking_falling_topic_{"/docking/docking_falling"};
  std::string manual_override_topic_{"/docking/manual_override"};
  std::string docking_failed_topic_{"/docking/docking_failed"};
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
  ros::init(argc, argv, "docking_nav_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  DockingNavNode node(nh, private_nh);
  if (!node.initialize())
  {
    return 1;
  }

  ros::spin();
  return 0;
}
