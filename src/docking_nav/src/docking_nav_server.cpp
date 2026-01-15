#include "docking_nav/docking_nav_server.h"

#include <cmath>
#include <iterator>
#include <set>

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
}  // namespace

DockingNavServer::DockingNavServer() = default;

bool DockingNavServer::initialize(ros::NodeHandle &private_nh)
{
  if (!loadParams(private_nh))
  {
    return false;
  }

  setMode(mode_, true);
  injectBool("CONSTHEIGHT", constheight_);
  injectBool("STATIONING", stationing_);
  injectBool("DOCKING_FALLING", docking_falling_);
  injectBool("MOOS_MANUAL_OVERIDE", manual_override_);
  injectBool("DOCKINGFAILED", docking_failed_);
  return true;
}

void DockingNavServer::setCommandCallbacks(BoolCommandCallback bool_callback,
                                           StringCommandCallback string_callback)
{
  bool_callback_ = std::move(bool_callback);
  string_callback_ = std::move(string_callback);
}

void DockingNavServer::setOpticalMeasurement(
    const docking_optical_msgs::OpticalMeasurement &msg)
{
  last_optical_stamp_ = msg.header.stamp;
  optical_received_ = true;
  dfOpticalNavLoc_[0] = msg.d_heading_deg;
  dfOpticalNavLoc_[1] = msg.theta_x_deg;
  dfOpticalNavLoc_[2] = msg.theta_y_deg;
  fallback_x_ = msg.fallback_x;
  fallback_y_ = msg.fallback_y;
  bDataFlag_ = msg.valid;
}

void DockingNavServer::setNavHeading(double heading_deg)
{
  dfNavHeading_ = heading_deg;
}

void DockingNavServer::setNavDepth(double depth_m)
{
  dfNavDepth_ = depth_m;
}

void DockingNavServer::setNavPitch(double pitch_rad)
{
  dfNavPitch_ = pitch_rad * 180.0 / kPi;
  dfCurrentIMUY_ = dfNavPitch_;
}

void DockingNavServer::setNavRoll(double roll_rad)
{
  dfNavRoll_ = roll_rad * 180.0 / kPi;
  dfCurrentIMUX_ = dfNavRoll_;
}

void DockingNavServer::setDesiredSpeed(double speed_mps)
{
  desired_speed_ = speed_mps;
}

DockingNavServer::Outputs DockingNavServer::update(const ros::Time &stamp)
{
  Outputs outputs;
  ++m_iterations_;

  if (!optical_received_ ||
      (stamp - last_optical_stamp_).toSec() > optical_timeout_sec_)
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
    handleDocking(stamp, outputs);
  }
  else
  {
    fillFeedback(outputs, 1.0, 0.0, 0.0, 0.0, stamp);
    fillPhase(outputs);
    fillOpticalXY(outputs, 0.0, 0.0, stamp);
  }

  return outputs;
}

bool DockingNavServer::loadParams(ros::NodeHandle &private_nh)
{
  if (!private_nh.getParam("nav_server_period", nav_server_period_))
  {
    ROS_ERROR("[docking_nav] nav_server_period not set");
    return false;
  }
  private_nh.param("optical_camera_delta_l", dfDeltaL_, 0.0);
  private_nh.param("camera_view_angle_deg", dfCameraViewAngle_, 25.0);
  private_nh.param("dock_depth", dfDockDepth_, 0.0);
  private_nh.param("dock_panel", m_dfDockPanel_, 0.0);
  private_nh.param("dock_heading_deg", dfDockHeading_, 0.0);
  private_nh.param("dock_pitch_deg", dfDockPitch_, 0.0);
  private_nh.param("dock_roll_deg", dfDockRoll_, 0.0);
  private_nh.param("depth_bias", m_dfDepthBias_, 0.0);
  private_nh.param("depth_ground_bias", m_dfDepthGroundBias_, 0.0);
  private_nh.param("depth_camera_bias", m_dfDepthCameraBias_, 0.0);
  private_nh.param("fall_depth", dfFallDepth_, 0.0);
  private_nh.param("dradius", dradius_, 0.0);
  private_nh.param("dinheading_deg", dinheading_, 0.0);
  private_nh.param("distance_bias", m_dfDistanceBias_, 0.3);
  private_nh.param("angle_bias_deg", dfAngleBias_, 0.5);
  private_nh.param("radius1th", radius1th_, 0.0);
  private_nh.param("maxtry_minute_num", nMaxtryMinuteNum_, 1);
  private_nh.param("constant_depth_minute_num", nConstantDepthMinuteNum_, 5);
  private_nh.param("last_phase_float_time_ms", m_nLastPhaseFloatTime_, 500.0);
  private_nh.param("transimit_duration_sec", m_nTransimitDuration_, 30);
  private_nh.param("docking_max_try", m_nDockingMaxTry_, 3);
  private_nh.param("optical_timeout_sec", optical_timeout_sec_, 0.5);
  private_nh.param("initial_mode", mode_, mode_);

  dfDockHeading_ = angle360(dfDockHeading_ + 90.0);
  m_dfFreq_ = 1.0 / nav_server_period_;

  std::vector<double> align_offsets;
  private_nh.param("align_depth_offsets", align_offsets,
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
    const double outer_radius =
        (dfLightDepth_ - m_dfDepthBias_ - m_dfDepthGroundBias_ - depth) *
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

void DockingNavServer::handleDocking(const ros::Time &stamp, Outputs &outputs)
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
      if (distance_ < dfInnerRadius_ && nPhaseCount_ < nPhaseNum_ - 1)
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
      if (nConstantDepthCount_ > nConstantDepthMinuteNum_ * 60 * m_dfFreq_ ||
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
          if (nFloatIteration_ < m_nLastPhaseFloatTime_ * m_dfFreq_ / 1000)
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

  fillPhase(outputs);
  fillOpticalXY(outputs, dfNextX, dfNextY, stamp);
  fillFeedback(outputs, dfLightDepth_ - dfCameraDepth_, dfNavHeading_, dfNextX,
               dfNextY, stamp);
}

void DockingNavServer::setMode(const std::string &mode, bool force)
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

void DockingNavServer::injectDepthUpdate(double depth)
{
  injectString("DOCKDEPTH_UPDATE", "depth=" + std::to_string(depth));
}

void DockingNavServer::injectHeadingUpdate(const std::string &update)
{
  injectString("DOCKHDG_UPDATES", update);
}

void DockingNavServer::injectBool(const std::string &key, bool value)
{
  if (bool_callback_)
  {
    bool_callback_(key, value);
  }
}

void DockingNavServer::injectString(const std::string &key,
                                    const std::string &value)
{
  if (string_callback_)
  {
    string_callback_(key, value);
  }
}

void DockingNavServer::fillPhase(Outputs &outputs) const
{
  outputs.phase = nPhaseCount_;
}

void DockingNavServer::fillOpticalXY(Outputs &outputs, double x, double y,
                                     const ros::Time &stamp) const
{
  outputs.optical_xy.header.stamp = stamp;
  outputs.optical_xy.header.frame_id = "docking";
  outputs.optical_xy.point.x = x;
  outputs.optical_xy.point.y = y;
  outputs.optical_xy.point.z = 0.0;
}

void DockingNavServer::fillFeedback(Outputs &outputs, double depth_error,
                                    double heading, double next_x,
                                    double next_y, const ros::Time &stamp)
{
  outputs.optical_feedback.header.stamp = stamp;
  outputs.optical_feedback.depth_error =
      depth_error > m_dfDistanceBias_ ? depth_error : m_dfDistanceBias_;
  outputs.optical_feedback.nav_heading_deg = heading;
  outputs.optical_feedback.next_x = next_x;
  outputs.optical_feedback.next_y = next_y;
  outputs.optical_feedback.delta_imu_x = dfCurrentIMUX_ - dfLastIMUX_;
  outputs.optical_feedback.delta_imu_y = dfCurrentIMUY_ - dfLastIMUY_;
  dfLastIMUX_ = dfCurrentIMUX_;
  dfLastIMUY_ = dfCurrentIMUY_;
}
