#include "bt_executor/ros/ros_io.hpp"

#include <XmlRpcValue.h>

#include <std_msgs/Float64.h>

#include "bt_executor/adapters/docking_phase_manager.hpp"
#include "bt_executor/adapters/helm_adapter.hpp"
#include "bt_executor/utils/utils.hpp"

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

double resolveStamp(double stamp)
{
  return stamp > 0.0 ? stamp : ros::Time::now().toSec();
}

}  // namespace

bool RosIO::init(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
                 const std::shared_ptr<BTContext> &ctx)
{
  nh_ = nh;
  private_nh_ = private_nh;
  ctx_ = ctx;

  if (!ctx_ || !ctx_->mission || !ctx_->nav || !ctx_->helm || !ctx_->docking)
  {
    ROS_ERROR("[bt_executor] RosIO requires a fully constructed context");
    return false;
  }

  if (!loadParameters(private_nh_))
  {
    return false;
  }

  outputs_publisher_ = std::make_shared<DockingOutputsPublisher>();
  command_publisher_ = std::make_shared<CommandPublisher>();

  setupPublishers();
  setupSubscribers();

  return true;
}

double RosIO::now() const
{
  return ros::Time::now().toSec();
}

bool RosIO::loadParameters(ros::NodeHandle &private_nh)
{
  private_nh.param("topics/nav_x_topic", topics_.nav_x_topic, topics_.nav_x_topic);
  private_nh.param("topics/nav_y_topic", topics_.nav_y_topic, topics_.nav_y_topic);
  private_nh.param("topics/nav_depth_topic", topics_.nav_depth_topic, topics_.nav_depth_topic);
  private_nh.param("topics/nav_heading_topic", topics_.nav_heading_topic, topics_.nav_heading_topic);
  private_nh.param("topics/nav_speed_topic", topics_.nav_speed_topic, topics_.nav_speed_topic);
  private_nh.param("topics/nav_pitch_topic", topics_.nav_pitch_topic, topics_.nav_pitch_topic);
  private_nh.param("topics/nav_roll_topic", topics_.nav_roll_topic, topics_.nav_roll_topic);

  private_nh.param("topics/deploy_topic", topics_.deploy_topic, topics_.deploy_topic);
  private_nh.param("topics/return_topic", topics_.return_topic, topics_.return_topic);

  private_nh.param("topics/mode_topic", topics_.mode_topic, topics_.mode_topic);
  private_nh.param("topics/mode_state_topic", topics_.mode_state_topic, topics_.mode_state_topic);

  private_nh.param("topics/stationing_topic", topics_.stationing_topic, topics_.stationing_topic);
  private_nh.param("topics/constheight_topic", topics_.constheight_topic, topics_.constheight_topic);
  private_nh.param("topics/docking_falling_topic", topics_.docking_falling_topic, topics_.docking_falling_topic);
  private_nh.param("topics/manual_override_topic", topics_.manual_override_topic, topics_.manual_override_topic);
  private_nh.param("topics/docking_failed_topic", topics_.docking_failed_topic, topics_.docking_failed_topic);
  private_nh.param("topics/dockdepth_update_topic", topics_.dockdepth_update_topic, topics_.dockdepth_update_topic);
  private_nh.param("topics/dockhdg_updates_topic", topics_.dockhdg_updates_topic, topics_.dockhdg_updates_topic);

  private_nh.param("topics/optical_measurement_topic", topics_.optical_measurement_topic,
                   topics_.optical_measurement_topic);
  private_nh.param("topics/dock_depth_topic", topics_.dock_depth_topic, topics_.dock_depth_topic);

  private_nh.param("topics/phase_topic", topics_.phase_topic, topics_.phase_topic);
  private_nh.param("topics/optical_xy_topic", topics_.optical_xy_topic, topics_.optical_xy_topic);
  private_nh.param("topics/optical_feedback_topic", topics_.optical_feedback_topic,
                   topics_.optical_feedback_topic);

  private_nh.param("topics/desired_heading_topic", topics_.desired_heading_topic, topics_.desired_heading_topic);
  private_nh.param("topics/desired_speed_topic", topics_.desired_speed_topic, topics_.desired_speed_topic);
  private_nh.param("topics/desired_depth_topic", topics_.desired_depth_topic, topics_.desired_depth_topic);

  command_topics_.bool_topics["STATIONING"] = topics_.stationing_topic;
  command_topics_.bool_topics["CONSTHEIGHT"] = topics_.constheight_topic;
  command_topics_.bool_topics["DOCKING_FALLING"] = topics_.docking_falling_topic;
  command_topics_.bool_topics["MOOS_MANUAL_OVERIDE"] = topics_.manual_override_topic;
  command_topics_.bool_topics["DOCKINGFAILED"] = topics_.docking_failed_topic;

  command_topics_.string_topics["MODE"] = topics_.mode_topic;
  command_topics_.string_topics["DOCKDEPTH_UPDATE"] = topics_.dockdepth_update_topic;
  command_topics_.string_topics["DOCKHDG_UPDATES"] = topics_.dockhdg_updates_topic;

  loadTopicMap(private_nh, "bool_command_topics", command_topics_.bool_topics);
  loadTopicMap(private_nh, "string_command_topics", command_topics_.string_topics);

  ROS_INFO_STREAM("[bt_executor] Topics: nav_heading=" << topics_.nav_heading_topic
                  << " nav_depth=" << topics_.nav_depth_topic
                  << " desired_heading=" << topics_.desired_heading_topic
                  << " mode_cmd=" << topics_.mode_topic);

  return true;
}

void RosIO::setupPublishers()
{
  desired_heading_pub_ = nh_.advertise<std_msgs::Float64>(topics_.desired_heading_topic, 10);
  desired_speed_pub_ = nh_.advertise<std_msgs::Float64>(topics_.desired_speed_topic, 10);
  desired_depth_pub_ = nh_.advertise<std_msgs::Float64>(topics_.desired_depth_topic, 10);
  mode_state_pub_ = nh_.advertise<std_msgs::String>(topics_.mode_state_topic, 10, true);

  outputs_publisher_->initialize(nh_, topics_.phase_topic, topics_.optical_xy_topic,
                                 topics_.optical_feedback_topic);
  command_publisher_->initialize(nh_, command_topics_.bool_topics, command_topics_.string_topics);
}

void RosIO::setupSubscribers()
{
  nav_x_sub_ = nh_.subscribe(topics_.nav_x_topic, 10, &RosIO::navXCallback, this);
  nav_y_sub_ = nh_.subscribe(topics_.nav_y_topic, 10, &RosIO::navYCallback, this);
  nav_depth_sub_ = nh_.subscribe(topics_.nav_depth_topic, 10, &RosIO::navDepthCallback, this);
  nav_heading_sub_ = nh_.subscribe(topics_.nav_heading_topic, 10, &RosIO::navHeadingCallback, this);
  nav_speed_sub_ = nh_.subscribe(topics_.nav_speed_topic, 10, &RosIO::navSpeedCallback, this);
  nav_pitch_sub_ = nh_.subscribe(topics_.nav_pitch_topic, 10, &RosIO::navPitchCallback, this);
  nav_roll_sub_ = nh_.subscribe(topics_.nav_roll_topic, 10, &RosIO::navRollCallback, this);

  deploy_sub_ = nh_.subscribe(topics_.deploy_topic, 10, &RosIO::deployCallback, this);
  return_sub_ = nh_.subscribe(topics_.return_topic, 10, &RosIO::returnCallback, this);

  mode_sub_ = nh_.subscribe(topics_.mode_topic, 10, &RosIO::modeCallback, this);
  stationing_sub_ = nh_.subscribe(topics_.stationing_topic, 10, &RosIO::stationingCallback, this);
  constheight_sub_ = nh_.subscribe(topics_.constheight_topic, 10, &RosIO::constHeightCallback, this);
  docking_falling_sub_ =
      nh_.subscribe(topics_.docking_falling_topic, 10, &RosIO::dockingFallingCallback, this);
  manual_override_sub_ =
      nh_.subscribe(topics_.manual_override_topic, 10, &RosIO::manualOverrideCallback, this);
  docking_failed_sub_ =
      nh_.subscribe(topics_.docking_failed_topic, 10, &RosIO::dockingFailedCallback, this);
  dockdepth_update_sub_ =
      nh_.subscribe(topics_.dockdepth_update_topic, 10, &RosIO::dockDepthUpdateCallback, this);
  dockhdg_updates_sub_ =
      nh_.subscribe(topics_.dockhdg_updates_topic, 10, &RosIO::dockHeadingUpdateCallback, this);
  optical_sub_ =
      nh_.subscribe(topics_.optical_measurement_topic, 10, &RosIO::opticalCallback, this);
  dock_depth_sub_ = nh_.subscribe(topics_.dock_depth_topic, 10, &RosIO::dockDepthCallback, this);
}

void RosIO::publishCommands(const HelmDecision &decision, double stamp)
{
  const NavSnapshot nav = ctx_->nav->snapshot();

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

  (void)stamp;
}

void RosIO::publishModeState(const std::string &mode)
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

void RosIO::applyBehaviorMessages(const std::vector<VarDataPair> &messages, double stamp)
{
  const double resolved_stamp = resolveStamp(stamp);

  for (const auto &msg : messages)
  {
    const std::string key = msg.get_var();
    bool bool_value = false;

    if (key == "MODE" && msg.is_string())
    {
      ctx_->mission->setMode(msg.get_sdata());
      ctx_->helm->setMode(msg.get_sdata(), resolved_stamp);
      command_publisher_->publishString("MODE", msg.get_sdata());
      continue;
    }

    if (key == "DOCKDEPTH_UPDATE" && msg.is_string())
    {
      ctx_->helm->setUpdateVar(key, msg.get_sdata(), resolved_stamp);
      command_publisher_->publishString(key, msg.get_sdata());
      continue;
    }

    if (key == "DOCKHDG_UPDATES" && msg.is_string())
    {
      ctx_->helm->setUpdateVar(key, msg.get_sdata(), resolved_stamp);
      command_publisher_->publishString(key, msg.get_sdata());
      continue;
    }

    if (parseVarDataPairBool(msg, bool_value))
    {
      if (key == "STATIONING")
      {
        ctx_->mission->setStationing(bool_value);
        ctx_->helm->setBool(key, bool_value, resolved_stamp);
        command_publisher_->publishBool(key, bool_value);
      }
      else if (key == "CONSTHEIGHT")
      {
        ctx_->mission->setConstHeight(bool_value);
        ctx_->helm->setBool(key, bool_value, resolved_stamp);
        command_publisher_->publishBool(key, bool_value);
      }
      else if (key == "DOCKING_FALLING")
      {
        ctx_->mission->setDockingFalling(bool_value);
        ctx_->helm->setBool(key, bool_value, resolved_stamp);
        command_publisher_->publishBool(key, bool_value);
      }
      else if (key == "MOOS_MANUAL_OVERIDE")
      {
        ctx_->mission->setManualOverride(bool_value);
        ctx_->helm->setBool(key, bool_value, resolved_stamp);
        command_publisher_->publishBool(key, bool_value);
      }
      else if (key == "DOCKINGFAILED")
      {
        ctx_->mission->setDockingFailed(bool_value);
        ctx_->helm->setBool(key, bool_value, resolved_stamp);
        command_publisher_->publishBool(key, bool_value);
      }
      else if (key == "DEPLOY")
      {
        ctx_->mission->setDeploy(bool_value);
        ctx_->helm->setBool(key, bool_value, resolved_stamp);
      }
      else if (key == "RETURN")
      {
        ctx_->mission->setReturn(bool_value);
        ctx_->helm->setBool(key, bool_value, resolved_stamp);
      }
    }
  }
}

void RosIO::applyDockingCommands(const DockingPhaseCommands &commands, double stamp)
{
  const double resolved_stamp = resolveStamp(stamp);

  auto applyBool = [&](const std::optional<bool> &value, const std::string &key, auto setter) {
    if (!value.has_value())
    {
      return;
    }
    setter(*value);
    ctx_->helm->setBool(key, *value, resolved_stamp);
    command_publisher_->publishBool(key, *value);
  };

  if (commands.mode)
  {
    ctx_->mission->setMode(*commands.mode);
    ctx_->helm->setMode(*commands.mode, resolved_stamp);
    command_publisher_->publishString("MODE", *commands.mode);
  }

  applyBool(commands.stationing, "STATIONING", [&](bool value) { ctx_->mission->setStationing(value); });
  applyBool(commands.constheight, "CONSTHEIGHT", [&](bool value) { ctx_->mission->setConstHeight(value); });
  applyBool(commands.docking_falling, "DOCKING_FALLING",
            [&](bool value) { ctx_->mission->setDockingFalling(value); });
  applyBool(commands.manual_override, "MOOS_MANUAL_OVERIDE",
            [&](bool value) { ctx_->mission->setManualOverride(value); });
  applyBool(commands.docking_failed, "DOCKINGFAILED",
            [&](bool value) { ctx_->mission->setDockingFailed(value); });

  if (commands.depth_update)
  {
    ctx_->helm->setUpdateVar("DOCKDEPTH_UPDATE", *commands.depth_update, resolved_stamp);
    command_publisher_->publishString("DOCKDEPTH_UPDATE", *commands.depth_update);
  }
  if (commands.heading_update)
  {
    ctx_->helm->setUpdateVar("DOCKHDG_UPDATES", *commands.heading_update, resolved_stamp);
    command_publisher_->publishString("DOCKHDG_UPDATES", *commands.heading_update);
  }
}

void RosIO::publishDockingOutputs(const DockingPhaseOutputs &outputs, double stamp)
{
  outputs_publisher_->publish(outputs, ros::Time(resolveStamp(stamp)));
}

double RosIO::lastCommandSpeed(double fallback_speed) const
{
  return last_command_.initialized ? last_command_.speed : fallback_speed;
}

void RosIO::navXCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  ctx_->nav->setX(msg->data);
}

void RosIO::navYCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  ctx_->nav->setY(msg->data);
}

void RosIO::navDepthCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  ctx_->nav->setDepth(msg->data);
}

void RosIO::navHeadingCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  ctx_->nav->setHeading(msg->data);
}

void RosIO::navSpeedCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  ctx_->nav->setSpeed(msg->data);
}

void RosIO::navPitchCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  ctx_->nav->setPitch(msg->data);
}

void RosIO::navRollCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  ctx_->nav->setRoll(msg->data);
}

void RosIO::deployCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = now();
  ctx_->mission->setDeploy(msg->data);
  ctx_->helm->setBool("DEPLOY", msg->data, stamp);
}

void RosIO::returnCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = now();
  ctx_->mission->setReturn(msg->data);
  ctx_->helm->setBool("RETURN", msg->data, stamp);
}

void RosIO::modeCallback(const std_msgs::String::ConstPtr &msg)
{
  const double stamp = now();
  ctx_->mission->setMode(msg->data);
  ctx_->helm->setMode(msg->data, stamp);
}

void RosIO::stationingCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = now();
  ctx_->mission->setStationing(msg->data);
  ctx_->helm->setBool("STATIONING", msg->data, stamp);
}

void RosIO::constHeightCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = now();
  ctx_->mission->setConstHeight(msg->data);
  ctx_->helm->setBool("CONSTHEIGHT", msg->data, stamp);
}

void RosIO::dockingFallingCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = now();
  ctx_->mission->setDockingFalling(msg->data);
  ctx_->helm->setBool("DOCKING_FALLING", msg->data, stamp);
}

void RosIO::manualOverrideCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = now();
  ctx_->mission->setManualOverride(msg->data);
  ctx_->helm->setBool("MOOS_MANUAL_OVERIDE", msg->data, stamp);
}

void RosIO::dockingFailedCallback(const std_msgs::Bool::ConstPtr &msg)
{
  const double stamp = now();
  ctx_->mission->setDockingFailed(msg->data);
  ctx_->helm->setBool("DOCKINGFAILED", msg->data, stamp);
}

void RosIO::dockDepthUpdateCallback(const std_msgs::String::ConstPtr &msg)
{
  ctx_->helm->setUpdateVar("DOCKDEPTH_UPDATE", msg->data, now());
}

void RosIO::dockHeadingUpdateCallback(const std_msgs::String::ConstPtr &msg)
{
  ctx_->helm->setUpdateVar("DOCKHDG_UPDATES", msg->data, now());
}

void RosIO::opticalCallback(const docking_optical_msgs::OpticalMeasurement::ConstPtr &msg)
{
  OpticalMeasurementData measurement;
  measurement.valid = msg->valid;
  measurement.theta_x_deg = msg->theta_x_deg;
  measurement.theta_y_deg = msg->theta_y_deg;
  measurement.fallback_x = msg->fallback_x;
  measurement.fallback_y = msg->fallback_y;
  ctx_->docking->setOpticalMeasurement(measurement);
}

void RosIO::dockDepthCallback(const common_msgs::Float64Stamped::ConstPtr &msg)
{
  ctx_->mission->setDockDepth(msg->data);
  ctx_->docking->setDockDepth(msg->data);
}

}  // namespace bt_executor
