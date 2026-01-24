#include "bt_executor/bt_executor.hpp"

#include <algorithm>
#include <utility>

#include <XmlRpcValue.h>

#include <ros/package.h>

#include "bt_executor/adapters/docking_phase_manager.hpp"
#include "bt_executor/adapters/helm_adapter.hpp"
#include "bt_executor/mission_state.hpp"
#include "bt_executor/nav_state.hpp"
#include "bt_executor/ros_io.hpp"

namespace bt_executor
{

namespace
{

bool loadPluginList(ros::NodeHandle &nh, std::vector<std::string> &plugins)
{
  XmlRpc::XmlRpcValue param;
  if (!nh.getParam("plugins", param))
  {
    return true;
  }

  if (param.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("[bt_executor] plugins must be a list of strings");
    return false;
  }

  plugins.clear();
  for (int i = 0; i < param.size(); ++i)
  {
    const auto &entry = param[i];
    if (entry.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      plugins.emplace_back(static_cast<std::string>(entry));
    }
  }
  return true;
}

}  // namespace

BTExecutorNode::BTExecutorNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(std::move(nh)), private_nh_(std::move(private_nh))
{
}

bool BTExecutorNode::initialize()
{
  if (!loadParameters() || !resolveTreePath())
  {
    return false;
  }

  ctx_ = std::make_shared<BTContext>();
  ctx_->nav = std::make_shared<NavState>();
  ctx_->mission = std::make_shared<MissionState>(initial_mode_);
  ctx_->mission->setDeploy(deploy_default_);
  ctx_->mission->setReturn(return_default_);

  ctx_->helm = std::make_shared<HelmAdapter>();
  ctx_->docking = std::make_shared<DockingPhaseManager>();

  if (!ctx_->docking->initialize(docking_config_))
  {
    ROS_ERROR("[bt_executor] Failed to initialize docking phase manager");
    return false;
  }

  ctx_->mission->setDockingTargets(ctx_->docking->dockHeadingDeg(), ctx_->docking->dockDepth(),
                                   ctx_->docking->dockPitchDeg(), ctx_->docking->dockRollDeg());
  ctx_->mission->updateDockingTelemetry(0, ctx_->docking->phaseTotal(), 0.0, 0.0, 0.0, false, false, 0.0);

  if (!ctx_->helm->initialize(private_nh_, initial_mode_, deploy_default_, return_default_))
  {
    ROS_ERROR("[bt_executor] Failed to initialize helm adapter");
    return false;
  }

  ctx_->ros_io = std::make_shared<RosIO>();
  if (!ctx_->ros_io->init(nh_, private_nh_, ctx_))
  {
    ROS_ERROR("[bt_executor] Failed to initialize ROS I/O");
    return false;
  }

  if (!core_.init(resolved_tree_path_, plugins_, ctx_))
  {
    ROS_ERROR_STREAM("[bt_executor] Failed to initialize BT core with tree=" << resolved_tree_path_);
    return false;
  }

  const ros::Duration period(1.0 / std::max(loop_frequency_, 1e-3));
  timer_ = nh_.createTimer(period, &BTExecutorNode::onTimerTick, this);

  ROS_INFO_STREAM("[bt_executor] Initialized with tree=" << resolved_tree_path_
                  << " loop_frequency=" << loop_frequency_ << "Hz");
  return true;
}

bool BTExecutorNode::loadParameters()
{
  private_nh_.param("loop_frequency", loop_frequency_, loop_frequency_);
  private_nh_.param("tree_path", tree_path_, tree_path_);
  private_nh_.param("initial_mode", initial_mode_, initial_mode_);
  private_nh_.param("deploy_default", deploy_default_, deploy_default_);
  private_nh_.param("return_default", return_default_, return_default_);

  return loadPlugins() && loadDockingConfig();
}

bool BTExecutorNode::resolveTreePath()
{
  resolved_tree_path_ = tree_path_;
  if (!tree_path_.empty() && tree_path_.front() != '/')
  {
    const std::string package_path = ros::package::getPath("bt_executor");
    if (package_path.empty())
    {
      ROS_ERROR("[bt_executor] Unable to resolve bt_executor package path");
      return false;
    }
    resolved_tree_path_ = package_path + "/" + tree_path_;
  }
  return true;
}

bool BTExecutorNode::loadPlugins()
{
  return loadPluginList(private_nh_, plugins_);
}

bool BTExecutorNode::loadDockingConfig()
{
  if (!private_nh_.getParam("docking/nav_server_period", docking_config_.nav_server_period))
  {
    ROS_ERROR("[bt_executor] docking/nav_server_period not set");
    return false;
  }

  private_nh_.param("docking/optical_camera_delta_l", docking_config_.optical_camera_delta_l, 0.0);
  private_nh_.param("docking/camera_view_angle_deg", docking_config_.camera_view_angle_deg, 25.0);
  private_nh_.param("docking/dock_depth", docking_config_.dock_depth, 0.0);
  private_nh_.param("docking/dock_panel", docking_config_.dock_panel, 0.0);
  private_nh_.param("docking/dock_heading_deg", docking_config_.dock_heading_deg, 0.0);
  private_nh_.param("docking/dock_pitch_deg", docking_config_.dock_pitch_deg, 0.0);
  private_nh_.param("docking/dock_roll_deg", docking_config_.dock_roll_deg, 0.0);
  private_nh_.param("docking/depth_bias", docking_config_.depth_bias, 0.0);
  private_nh_.param("docking/depth_ground_bias", docking_config_.depth_ground_bias, 0.0);
  private_nh_.param("docking/depth_camera_bias", docking_config_.depth_camera_bias, 0.0);
  private_nh_.param("docking/fall_depth", docking_config_.fall_depth, 0.0);
  private_nh_.param("docking/dradius", docking_config_.dradius, 0.0);
  private_nh_.param("docking/dinheading_deg", docking_config_.dinheading_deg, 0.0);
  private_nh_.param("docking/distance_bias", docking_config_.distance_bias, 0.3);
  private_nh_.param("docking/angle_bias_deg", docking_config_.angle_bias_deg, 0.5);
  private_nh_.param("docking/radius1th", docking_config_.radius1th, 0.0);
  private_nh_.param("docking/maxtry_minute_num", docking_config_.max_try_minute_num, 1);
  private_nh_.param("docking/constant_depth_minute_num", docking_config_.constant_depth_minute_num, 5);
  private_nh_.param("docking/last_phase_float_time_ms", docking_config_.last_phase_float_time_ms, 500.0);
  private_nh_.param("docking/transimit_duration_sec", docking_config_.transmit_duration_sec, 30);
  private_nh_.param("docking/docking_max_try", docking_config_.docking_max_try, 3);
  private_nh_.param("docking/auto_enter_closetodocking", docking_config_.auto_enter_closetodocking, false);
  private_nh_.param("docking/auto_enter_duration_sec", docking_config_.auto_enter_duration_sec, 2.0);
  private_nh_.param("docking/align_depth_offsets", docking_config_.align_depth_offsets, std::vector<double>{});

  docking_config_.imu_roll_bias_deg = private_nh_.param("docking/imu_roll_bias_deg", docking_config_.imu_roll_bias_deg);
  docking_config_.imu_pitch_bias_deg = private_nh_.param("docking/imu_pitch_bias_deg", docking_config_.imu_pitch_bias_deg);

  return true;
}

void BTExecutorNode::onTimerTick(const ros::TimerEvent &)
{
  core_.tickOnce();
}

}  // namespace bt_executor
