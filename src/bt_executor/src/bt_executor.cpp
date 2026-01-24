#include "bt_executor/bt_executor.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <utility>

#include <XmlRpcValue.h>

#include <behaviortree_cpp_v3/xml_parsing.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

#include "bt_executor/nodes/actions.hpp"
#include "bt_executor/nodes/conditions.hpp"
#include "bt_executor/utils.hpp"

namespace bt_executor
{

namespace
{

void loadTopicMap(ros::NodeHandle &nh, const std::string &param_name,
                  std::unordered_map<std::string, std::string> &topic_map)
{
  XmlRpc::XmlRpcValue param;
  if (!nh.getParam(param_name, param))
  {
    return;
  }
  if (param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_WARN_STREAM("[bt_executor] " << param_name << " must be a map");
    return;
  }

  for (auto it = param.begin(); it != param.end(); ++it)
  {
    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      topic_map[it->first] = static_cast<std::string>(it->second);
    }
  }
}

bool parseBool(const VarDataPair &pair, bool &value)
{
  if (!pair.is_string())
  {
    value = std::abs(pair.get_ddata()) > 1e-6;
    return true;
  }
  std::string text = pair.get_sdata();
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  if (text == "true" || text == "1")
  {
    value = true;
    return true;
  }
  if (text == "false" || text == "0")
  {
    value = false;
    return true;
  }
  return false;
}

}  // namespace

BTExecutor::BTExecutor(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(std::move(nh)), private_nh_(std::move(private_nh))
{
}

bool BTExecutor::initialize()
{
  if (!loadParameters())
  {
    return false;
  }

  nav_state_ = std::make_shared<NavState>();
  mission_state_ = std::make_shared<MissionState>(initial_mode_);
  mission_state_->setDeploy(deploy_default_);
  mission_state_->setReturn(return_default_);

  helm_adapter_ = std::make_shared<HelmAdapter>();
  docking_manager_ = std::make_shared<DockingPhaseManager>();
  outputs_publisher_ = std::make_shared<DockingOutputsPublisher>();
  command_publisher_ = std::make_shared<CommandPublisher>();

  if (!docking_manager_->initialize(private_nh_))
  {
    return false;
  }
  mission_state_->setDockingTargets(docking_manager_->dockHeadingDeg(), docking_manager_->dockDepth(),
                                    docking_manager_->dockPitchDeg(), docking_manager_->dockRollDeg());

  if (!helm_adapter_->initialize(private_nh_, initial_mode_, deploy_default_, return_default_))
  {
    return false;
  }

  outputs_publisher_->initialize(nh_, topics_.phase_topic, topics_.optical_xy_topic,
                                 topics_.optical_feedback_topic);
  command_publisher_->initialize(nh_, command_topics_.bool_topics, command_topics_.string_topics);

  registerNodes();
  if (!loadTree())
  {
    return false;
  }

  setupSubscribers();
  setupPublishers();

  const ros::Duration period(1.0 / std::max(loop_frequency_, 1e-3));
  timer_ = nh_.createTimer(period, &BTExecutor::tick, this);

  ROS_INFO_STREAM("[bt_executor] Initialized with tree=" << tree_path_
                  << " loop_frequency=" << loop_frequency_ << "Hz");
  return true;
}

void BTExecutor::registerNodes()
{
  factory_.registerBuilder<DeployReadyCondition>(
      "DeployReady", [this](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<DeployReadyCondition>(name, config, mission_state_);
      });
  factory_.registerBuilder<ModeCondition>(
      "ModeIs", [this](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<ModeCondition>(name, config, mission_state_);
      });
  factory_.registerBuilder<AnyModeCondition>(
      "ModeIn", [this](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<AnyModeCondition>(name, config, mission_state_);
      });
  factory_.registerBuilder<FlagCondition>(
      "FlagIs", [this](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<FlagCondition>(name, config, mission_state_);
      });
  factory_.registerBuilder<PhaseCondition>(
      "PhaseIs", [this](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<PhaseCondition>(name, config, mission_state_);
      });
  factory_.registerBuilder<DockingPhaseUpdateCondition>(
      "DockingPhaseUpdate", [this](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<DockingPhaseUpdateCondition>(name, config, mission_state_, nav_state_,
                                                              helm_adapter_, docking_manager_,
                                                              outputs_publisher_, command_publisher_);
      });
  factory_.registerNodeType<AlwaysSuccessCondition>("AlwaysSuccess");

  factory_.registerBuilder<ActivateBehaviorAction>(
      "ActivateBehavior", [this](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<ActivateBehaviorAction>(name, config, helm_adapter_);
      });
  factory_.registerBuilder<SetModeAction>(
      "SetMode", [this](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<SetModeAction>(name, config, mission_state_, helm_adapter_);
      });
  factory_.registerBuilder<SetFlagAction>(
      "SetFlag", [this](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<SetFlagAction>(name, config, mission_state_, helm_adapter_);
      });
}

bool BTExecutor::loadParameters()
{
  private_nh_.param("loop_frequency", loop_frequency_, loop_frequency_);
  private_nh_.param("tree_path", tree_path_, tree_path_);
  private_nh_.param("initial_mode", initial_mode_, initial_mode_);
  private_nh_.param("deploy_default", deploy_default_, deploy_default_);
  private_nh_.param("return_default", return_default_, return_default_);

  private_nh_.param("topics/nav_x_topic", topics_.nav_x_topic, topics_.nav_x_topic);
  private_nh_.param("topics/nav_y_topic", topics_.nav_y_topic, topics_.nav_y_topic);
  private_nh_.param("topics/nav_depth_topic", topics_.nav_depth_topic, topics_.nav_depth_topic);
  private_nh_.param("topics/nav_heading_topic", topics_.nav_heading_topic, topics_.nav_heading_topic);
  private_nh_.param("topics/nav_speed_topic", topics_.nav_speed_topic, topics_.nav_speed_topic);
  private_nh_.param("topics/nav_pitch_topic", topics_.nav_pitch_topic, topics_.nav_pitch_topic);
  private_nh_.param("topics/nav_roll_topic", topics_.nav_roll_topic, topics_.nav_roll_topic);

  private_nh_.param("topics/deploy_topic", topics_.deploy_topic, topics_.deploy_topic);
  private_nh_.param("topics/return_topic", topics_.return_topic, topics_.return_topic);

  private_nh_.param("topics/mode_topic", topics_.mode_topic, topics_.mode_topic);
  private_nh_.param("topics/mode_state_topic", topics_.mode_state_topic, topics_.mode_state_topic);

  private_nh_.param("topics/stationing_topic", topics_.stationing_topic, topics_.stationing_topic);
  private_nh_.param("topics/constheight_topic", topics_.constheight_topic, topics_.constheight_topic);
  private_nh_.param("topics/docking_falling_topic", topics_.docking_falling_topic, topics_.docking_falling_topic);
  private_nh_.param("topics/manual_override_topic", topics_.manual_override_topic, topics_.manual_override_topic);
  private_nh_.param("topics/docking_failed_topic", topics_.docking_failed_topic, topics_.docking_failed_topic);
  private_nh_.param("topics/dockdepth_update_topic", topics_.dockdepth_update_topic, topics_.dockdepth_update_topic);
  private_nh_.param("topics/dockhdg_updates_topic", topics_.dockhdg_updates_topic, topics_.dockhdg_updates_topic);

  private_nh_.param("topics/optical_measurement_topic", topics_.optical_measurement_topic,
                    topics_.optical_measurement_topic);
  private_nh_.param("topics/phase_topic", topics_.phase_topic, topics_.phase_topic);
  private_nh_.param("topics/optical_xy_topic", topics_.optical_xy_topic, topics_.optical_xy_topic);
  private_nh_.param("topics/optical_feedback_topic", topics_.optical_feedback_topic,
                    topics_.optical_feedback_topic);
  private_nh_.param("topics/dock_depth_topic", topics_.dock_depth_topic, topics_.dock_depth_topic);

  private_nh_.param("topics/desired_heading_topic", topics_.desired_heading_topic, topics_.desired_heading_topic);
  private_nh_.param("topics/desired_speed_topic", topics_.desired_speed_topic, topics_.desired_speed_topic);
  private_nh_.param("topics/desired_depth_topic", topics_.desired_depth_topic, topics_.desired_depth_topic);

  command_topics_.bool_topics["STATIONING"] = topics_.stationing_topic;
  command_topics_.bool_topics["CONSTHEIGHT"] = topics_.constheight_topic;
  command_topics_.bool_topics["DOCKING_FALLING"] = topics_.docking_falling_topic;
  command_topics_.bool_topics["MOOS_MANUAL_OVERIDE"] = topics_.manual_override_topic;
  command_topics_.bool_topics["DOCKINGFAILED"] = topics_.docking_failed_topic;

  command_topics_.string_topics["MODE"] = topics_.mode_topic;
  command_topics_.string_topics["DOCKDEPTH_UPDATE"] = topics_.dockdepth_update_topic;
  command_topics_.string_topics["DOCKHDG_UPDATES"] = topics_.dockhdg_updates_topic;

  loadTopicMap(private_nh_, "bool_command_topics", command_topics_.bool_topics);
  loadTopicMap(private_nh_, "string_command_topics", command_topics_.string_topics);

  ROS_INFO_STREAM("[bt_executor] Topics: nav_heading=" << topics_.nav_heading_topic
                  << " nav_depth=" << topics_.nav_depth_topic
                  << " desired_heading=" << topics_.desired_heading_topic
                  << " mode_cmd=" << topics_.mode_topic);

  return true;
}

bool BTExecutor::loadTree()
{
  std::string resolved_path = tree_path_;
  if (!tree_path_.empty() && tree_path_.front() != '/')
  {
    const std::string package_path = ros::package::getPath("bt_executor");
    resolved_path = package_path + "/" + tree_path_;
  }

  try
  {
    tree_ = factory_.createTreeFromFile(resolved_path);
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM("[bt_executor] Failed to load tree: " << ex.what());
    return false;
  }

  tree_path_ = resolved_path;
  return true;
}

void BTExecutor::setupSubscribers()
{
  nav_x_sub_ = nh_.subscribe(topics_.nav_x_topic, 10, &BTExecutor::navXCallback, this);
  nav_y_sub_ = nh_.subscribe(topics_.nav_y_topic, 10, &BTExecutor::navYCallback, this);
  nav_depth_sub_ = nh_.subscribe(topics_.nav_depth_topic, 10, &BTExecutor::navDepthCallback, this);
  nav_heading_sub_ = nh_.subscribe(topics_.nav_heading_topic, 10, &BTExecutor::navHeadingCallback, this);
  nav_speed_sub_ = nh_.subscribe(topics_.nav_speed_topic, 10, &BTExecutor::navSpeedCallback, this);
  nav_pitch_sub_ = nh_.subscribe(topics_.nav_pitch_topic, 10, &BTExecutor::navPitchCallback, this);
  nav_roll_sub_ = nh_.subscribe(topics_.nav_roll_topic, 10, &BTExecutor::navRollCallback, this);

  deploy_sub_ = nh_.subscribe(topics_.deploy_topic, 10, &BTExecutor::deployCallback, this);
  return_sub_ = nh_.subscribe(topics_.return_topic, 10, &BTExecutor::returnCallback, this);

  mode_sub_ = nh_.subscribe(topics_.mode_topic, 10, &BTExecutor::modeCallback, this);
  stationing_sub_ = nh_.subscribe(topics_.stationing_topic, 10, &BTExecutor::stationingCallback, this);
  constheight_sub_ = nh_.subscribe(topics_.constheight_topic, 10, &BTExecutor::constHeightCallback, this);
  docking_falling_sub_ = nh_.subscribe(topics_.docking_falling_topic, 10, &BTExecutor::dockingFallingCallback, this);
  manual_override_sub_ = nh_.subscribe(topics_.manual_override_topic, 10, &BTExecutor::manualOverrideCallback, this);
  docking_failed_sub_ = nh_.subscribe(topics_.docking_failed_topic, 10, &BTExecutor::dockingFailedCallback, this);
  dockdepth_update_sub_ = nh_.subscribe(topics_.dockdepth_update_topic, 10, &BTExecutor::dockDepthUpdateCallback, this);
  dockhdg_updates_sub_ = nh_.subscribe(topics_.dockhdg_updates_topic, 10, &BTExecutor::dockHeadingUpdateCallback, this);
  optical_sub_ = nh_.subscribe(topics_.optical_measurement_topic, 10, &BTExecutor::opticalCallback, this);
  dock_depth_sub_ = nh_.subscribe(topics_.dock_depth_topic, 10, &BTExecutor::dockDepthCallback, this);
}

void BTExecutor::setupPublishers()
{
  desired_heading_pub_ = nh_.advertise<std_msgs::Float64>(topics_.desired_heading_topic, 10);
  desired_speed_pub_ = nh_.advertise<std_msgs::Float64>(topics_.desired_speed_topic, 10);
  desired_depth_pub_ = nh_.advertise<std_msgs::Float64>(topics_.desired_depth_topic, 10);
  mode_state_pub_ = nh_.advertise<std_msgs::String>(topics_.mode_state_topic, 10, true);
}

void BTExecutor::tick(const ros::TimerEvent &)
{
  const ros::Time now = ros::Time::now();
  const double stamp = now.toSec();

  if (tree_.rootBlackboard())
  {
    tree_.rootBlackboard()->set("current_time", stamp);
  }

  const NavSnapshot nav_before = nav_state_->snapshot();
  const MissionSnapshot mission_before = mission_state_->snapshot();

  helm_adapter_->syncFromState(nav_before, mission_before, stamp);
  helm_adapter_->resetActivations(stamp);

  tree_.tickRoot();

  const MissionSnapshot mission_after_tree = mission_state_->snapshot();
  helm_adapter_->syncFromState(nav_before, mission_after_tree, stamp);

  const HelmDecision decision = helm_adapter_->solve(stamp);
  const auto messages = helm_adapter_->consumeBehaviorMessages(stamp);
  applyBehaviorMessages(messages, stamp);

  const MissionSnapshot mission_after_msgs = mission_state_->snapshot();
  helm_adapter_->syncFromState(nav_before, mission_after_msgs, stamp);

  publishCommands(decision, stamp);
  publishModeState(mission_after_msgs.mode);

  docking_manager_->setDesiredSpeed(last_command_.speed);
}

void BTExecutor::publishCommands(const HelmDecision &decision, double)
{
  const NavSnapshot nav = nav_state_->snapshot();

  const double heading = decision.has_heading
                             ? normalizeAngle360(decision.heading_deg)
                             : (last_command_.initialized ? last_command_.heading_deg : nav.heading);
  const double speed = decision.has_speed ? decision.speed : 0.0;
  const double depth = decision.has_depth ? decision.depth : nav.depth;

  std_msgs::Float64 heading_msg;
  heading_msg.data = heading;
  desired_heading_pub_.publish(heading_msg);

  std_msgs::Float64 speed_msg;
  speed_msg.data = speed;
  desired_speed_pub_.publish(speed_msg);

  std_msgs::Float64 depth_msg;
  depth_msg.data = depth;
  desired_depth_pub_.publish(depth_msg);

  last_command_.heading_deg = heading;
  last_command_.speed = speed;
  last_command_.depth = depth;
  last_command_.initialized = true;
}

void BTExecutor::publishModeState(const std::string &mode)
{
  if (mode != last_mode_published_)
  {
    ROS_INFO_STREAM("[bt_executor] Mode state=" << mode);
    last_mode_published_ = mode;
  }
  std_msgs::String msg;
  msg.data = mode;
  mode_state_pub_.publish(msg);
}

void BTExecutor::applyBehaviorMessages(const std::vector<VarDataPair> &messages, double stamp)
{
  for (const auto &msg : messages)
  {
    const std::string key = msg.get_var();
    bool bool_value = false;

    if (key == "MODE" && msg.is_string())
    {
      mission_state_->setMode(msg.get_sdata());
      helm_adapter_->setMode(msg.get_sdata(), stamp);
      command_publisher_->publishString("MODE", msg.get_sdata());
      continue;
    }

    if (key == "DOCKDEPTH_UPDATE" && msg.is_string())
    {
      helm_adapter_->setUpdateVar(key, msg.get_sdata(), stamp);
      command_publisher_->publishString(key, msg.get_sdata());
      continue;
    }

    if (key == "DOCKHDG_UPDATES" && msg.is_string())
    {
      helm_adapter_->setUpdateVar(key, msg.get_sdata(), stamp);
      command_publisher_->publishString(key, msg.get_sdata());
      continue;
    }

    if (parseBool(msg, bool_value))
    {
      if (key == "STATIONING")
      {
        mission_state_->setStationing(bool_value);
        helm_adapter_->setBool(key, bool_value, stamp);
        command_publisher_->publishBool(key, bool_value);
      }
      else if (key == "CONSTHEIGHT")
      {
        mission_state_->setConstHeight(bool_value);
        helm_adapter_->setBool(key, bool_value, stamp);
        command_publisher_->publishBool(key, bool_value);
      }
      else if (key == "DOCKING_FALLING")
      {
        mission_state_->setDockingFalling(bool_value);
        helm_adapter_->setBool(key, bool_value, stamp);
        command_publisher_->publishBool(key, bool_value);
      }
      else if (key == "MOOS_MANUAL_OVERIDE")
      {
        mission_state_->setManualOverride(bool_value);
        helm_adapter_->setBool(key, bool_value, stamp);
        command_publisher_->publishBool(key, bool_value);
      }
      else if (key == "DOCKINGFAILED")
      {
        mission_state_->setDockingFailed(bool_value);
        helm_adapter_->setBool(key, bool_value, stamp);
        command_publisher_->publishBool(key, bool_value);
      }
      else if (key == "DEPLOY")
      {
        mission_state_->setDeploy(bool_value);
        helm_adapter_->setBool(key, bool_value, stamp);
      }
      else if (key == "RETURN")
      {
        mission_state_->setReturn(bool_value);
        helm_adapter_->setBool(key, bool_value, stamp);
      }
    }
  }
}

void BTExecutor::navXCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  nav_state_->setX(msg->data);
}

void BTExecutor::navYCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  nav_state_->setY(msg->data);
}

void BTExecutor::navDepthCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  nav_state_->setDepth(msg->data);
}

void BTExecutor::navHeadingCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  nav_state_->setHeading(msg->data);
}

void BTExecutor::navSpeedCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  nav_state_->setSpeed(msg->data);
}

void BTExecutor::navPitchCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  nav_state_->setPitch(msg->data);
}

void BTExecutor::navRollCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  nav_state_->setRoll(msg->data);
}

void BTExecutor::deployCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = ros::Time::now().toSec();
  mission_state_->setDeploy(msg->data);
  helm_adapter_->setBool("DEPLOY", msg->data, stamp);
}

void BTExecutor::returnCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = ros::Time::now().toSec();
  mission_state_->setReturn(msg->data);
  helm_adapter_->setBool("RETURN", msg->data, stamp);
}

void BTExecutor::modeCallback(const std_msgs::String::ConstPtr &msg)
{
  const double stamp = ros::Time::now().toSec();
  mission_state_->setMode(msg->data);
  helm_adapter_->setMode(msg->data, stamp);
}

void BTExecutor::stationingCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = ros::Time::now().toSec();
  mission_state_->setStationing(msg->data);
  helm_adapter_->setBool("STATIONING", msg->data, stamp);
}

void BTExecutor::constHeightCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = ros::Time::now().toSec();
  mission_state_->setConstHeight(msg->data);
  helm_adapter_->setBool("CONSTHEIGHT", msg->data, stamp);
}

void BTExecutor::dockingFallingCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = ros::Time::now().toSec();
  mission_state_->setDockingFalling(msg->data);
  helm_adapter_->setBool("DOCKING_FALLING", msg->data, stamp);
}

void BTExecutor::manualOverrideCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = ros::Time::now().toSec();
  mission_state_->setManualOverride(msg->data);
  helm_adapter_->setBool("MOOS_MANUAL_OVERIDE", msg->data, stamp);
}

void BTExecutor::dockingFailedCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = ros::Time::now().toSec();
  mission_state_->setDockingFailed(msg->data);
  helm_adapter_->setBool("DOCKINGFAILED", msg->data, stamp);
}

void BTExecutor::dockDepthUpdateCallback(const std_msgs::String::ConstPtr &msg)
{
  const double stamp = ros::Time::now().toSec();
  helm_adapter_->setUpdateVar("DOCKDEPTH_UPDATE", msg->data, stamp);
}

void BTExecutor::dockHeadingUpdateCallback(const std_msgs::String::ConstPtr &msg)
{
  const double stamp = ros::Time::now().toSec();
  helm_adapter_->setUpdateVar("DOCKHDG_UPDATES", msg->data, stamp);
}

void BTExecutor::opticalCallback(const docking_optical_msgs::OpticalMeasurement::ConstPtr &msg)
{
  docking_manager_->setOpticalMeasurement(*msg);
}

void BTExecutor::dockDepthCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  mission_state_->setDockDepth(msg->data);
  docking_manager_->setDockDepth(msg->data);
}

}  // namespace bt_executor
