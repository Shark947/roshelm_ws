#include "bt_executor/helm_adapter.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <XmlRpcValue.h>

#include "BehaviorSet.h"
#include "HelmEngine.h"
#include "HelmReport.h"
#include "InfoBuffer.h"
#include "IvPBehavior.h"
#include "IvPDomain.h"
#include "LedgerSnap.h"
#include "VarDataPair.h"

#include "BHV_ConstantDepth.h"
#include "BHV_ConstantHeading.h"
#include "BHV_ConstantSpeed.h"
#include "BHV_GoToDepth.h"
#include "BHV_StationKeep.h"
#include "BHV_Waypoint.h"

namespace bt_executor
{

namespace
{

std::string trimCopy(const std::string &value)
{
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }).base();
  if (begin >= end)
  {
    return std::string{};
  }
  return std::string(begin, end);
}

bool parseDomainSpec(const std::string &spec, double &low, double &high,
                     unsigned int &points)
{
  std::stringstream stream(spec);
  std::string low_s;
  std::string high_s;
  std::string points_s;
  if (!std::getline(stream, low_s, ':') || !std::getline(stream, high_s, ':') ||
      !std::getline(stream, points_s))
  {
    return false;
  }

  try
  {
    low = std::stod(trimCopy(low_s));
    high = std::stod(trimCopy(high_s));
    points = static_cast<unsigned int>(std::stoul(trimCopy(points_s)));
  }
  catch (const std::exception &)
  {
    return false;
  }
  return points >= 2 && high > low;
}

std::string xmlRpcToString(const XmlRpc::XmlRpcValue &value)
{
  switch (value.getType())
  {
    case XmlRpc::XmlRpcValue::TypeString:
      return static_cast<std::string>(value);
    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      std::ostringstream stream;
      stream << static_cast<double>(value);
      return stream.str();
    }
    case XmlRpc::XmlRpcValue::TypeInt:
    {
      std::ostringstream stream;
      stream << static_cast<int>(value);
      return stream.str();
    }
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return static_cast<bool>(value) ? "true" : "false";
    default:
      return std::string{};
  }
}

std::vector<std::pair<std::string, std::string>> loadParamMap(const XmlRpc::XmlRpcValue &map_value)
{
  std::vector<std::pair<std::string, std::string>> params;
  if (map_value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    return params;
  }
  for (auto it = map_value.begin(); it != map_value.end(); ++it)
  {
    const std::string key = it->first;
    const std::string value = xmlRpcToString(it->second);
    if (!key.empty() && !value.empty())
    {
      params.emplace_back(key, value);
    }
  }
  return params;
}

std::vector<std::string> loadStringArray(const XmlRpc::XmlRpcValue &array_value)
{
  std::vector<std::string> values;
  if (array_value.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    return values;
  }
  for (int i = 0; i < array_value.size(); ++i)
  {
    const auto &entry = array_value[i];
    if (entry.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      values.emplace_back(static_cast<std::string>(entry));
    }
  }
  return values;
}

}  // namespace

HelmAdapter::HelmAdapter() = default;

HelmAdapter::~HelmAdapter() = default;

bool HelmAdapter::initialize(ros::NodeHandle &private_nh, const std::string &initial_mode,
                             bool deploy_default, bool return_default)
{
  heading_var_ = private_nh.param<std::string>("helm/decision_vars/heading", heading_var_);
  speed_var_ = private_nh.param<std::string>("helm/decision_vars/speed", speed_var_);
  depth_var_ = private_nh.param<std::string>("helm/decision_vars/depth", depth_var_);
  ownship_ = private_nh.param<std::string>("helm/ownship", ownship_);

  if (!loadDomain(private_nh) || !buildIvPDomain())
  {
    return false;
  }

  info_buffer_ = std::make_unique<InfoBuffer>();
  ledger_snap_ = std::make_unique<LedgerSnap>();
  behavior_set_ = std::make_unique<BehaviorSet>();

  behavior_set_->setDomain(*domain_);
  behavior_set_->setOwnship(ownship_);
  behavior_set_->connectInfoBuffer(info_buffer_.get());
  behavior_set_->connectLedgerSnap(ledger_snap_.get());

  if (!loadInitialValues(private_nh) || !loadBehaviors(private_nh) || !createBehaviors())
  {
    return false;
  }

  helm_engine_ = std::make_unique<HelmEngine>(*domain_, info_buffer_.get(), ledger_snap_.get());
  helm_engine_->setBehaviorSet(behavior_set_.get());

  const double start_time = ros::Time::now().toSec();
  info_buffer_->setStartTime(start_time);
  info_buffer_->setCurrTime(start_time);
  behavior_set_->setCurrTime(start_time);

  setInfoBufferString("MODE", initial_mode, start_time);
  setInfoBufferString("DEPLOY", deploy_default ? "true" : "false", start_time);
  setInfoBufferString("RETURN", return_default ? "true" : "false", start_time);

  for (const auto &entry : initial_doubles_)
  {
    setInfoBufferDouble(entry.first, entry.second, start_time);
  }

  resetActivations(start_time);
  return true;
}

void HelmAdapter::syncFromState(const NavSnapshot &nav, const MissionSnapshot &mission,
                                double stamp)
{
  info_buffer_->setCurrTime(stamp);
  behavior_set_->setCurrTime(stamp);

  setInfoBufferDouble("NAV_X", nav.x, stamp);
  setInfoBufferDouble("NAV_Y", nav.y, stamp);
  setInfoBufferDouble("NAV_DEPTH", nav.depth, stamp);
  setInfoBufferDouble("NAV_HEADING", nav.heading, stamp);
  setInfoBufferDouble("NAV_SPEED", nav.speed, stamp);
  setInfoBufferDouble("NAV_PITCH", nav.pitch, stamp);
  setInfoBufferDouble("NAV_ROLL", nav.roll, stamp);

  setInfoBufferString("MODE", mission.mode, stamp);
  setInfoBufferString("DEPLOY", mission.deploy ? "true" : "false", stamp);
  setInfoBufferString("RETURN", mission.should_return ? "true" : "false", stamp);
  setInfoBufferString("STATIONING", mission.stationing ? "true" : "false", stamp);
  setInfoBufferString("CONSTHEIGHT", mission.constheight ? "true" : "false", stamp);
  setInfoBufferString("DOCKING_FALLING", mission.docking_falling ? "true" : "false", stamp);
  setInfoBufferString("MOOS_MANUAL_OVERIDE", mission.manual_override ? "true" : "false", stamp);
  setInfoBufferString("DOCKINGFAILED", mission.docking_failed ? "true" : "false", stamp);
  setInfoBufferDouble("DOCK_HEADING", mission.dock_heading_deg, stamp);
  setInfoBufferDouble("DOCK_DEPTH", mission.dock_depth, stamp);
}

void HelmAdapter::resetActivations(double stamp)
{
  for (const auto &entry : activation_vars_)
  {
    setInfoBufferString(entry.second, "false", stamp);
  }
}

bool HelmAdapter::activateBehavior(const std::string &name, double stamp)
{
  const auto it = activation_vars_.find(name);
  if (it == activation_vars_.end())
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "[bt_executor] Unknown behavior activation request: " << name);
    return false;
  }
  setInfoBufferString(it->second, "true", stamp);
  return true;
}

void HelmAdapter::setUpdateVar(const std::string &var, const std::string &value,
                               double stamp)
{
  if (var.empty())
  {
    return;
  }
  setInfoBufferString(var, value, stamp);
}

void HelmAdapter::setMode(const std::string &mode, double stamp)
{
  setInfoBufferString("MODE", mode, stamp);
}

void HelmAdapter::setBool(const std::string &key, bool value, double stamp)
{
  setInfoBufferString(key, value ? "true" : "false", stamp);
}

void HelmAdapter::setString(const std::string &key, const std::string &value,
                            double stamp)
{
  setInfoBufferString(key, value, stamp);
}

HelmDecision HelmAdapter::solve(double stamp)
{
  HelmDecision decision;
  if (!helm_engine_ || !behavior_set_)
  {
    return decision;
  }

  behavior_set_->resetStateOK();
  const HelmReport report = helm_engine_->determineNextDecision(behavior_set_.get(), stamp);
  last_report_ = std::make_unique<HelmReport>(report);

  decision.has_heading = report.hasDecision(heading_var_);
  decision.has_speed = report.hasDecision(speed_var_);
  decision.has_depth = report.hasDecision(depth_var_);

  if (decision.has_heading)
  {
    decision.heading_deg = report.getDecision(heading_var_);
  }
  if (decision.has_speed)
  {
    decision.speed = report.getDecision(speed_var_);
  }
  if (decision.has_depth)
  {
    decision.depth = report.getDecision(depth_var_);
  }

  behavior_set_->refreshMapUpdateVars();
  info_buffer_->clearDeltaVectors();
  return decision;
}

std::vector<VarDataPair> HelmAdapter::consumeBehaviorMessages(double stamp)
{
  std::vector<VarDataPair> messages;
  if (!behavior_set_)
  {
    return messages;
  }

  const unsigned int count = behavior_set_->size();
  for (unsigned int i = 0; i < count; ++i)
  {
    auto bhv_msgs = behavior_set_->getMessages(i, true);
    messages.insert(messages.end(), bhv_msgs.begin(), bhv_msgs.end());
  }

  for (const auto &msg : messages)
  {
    if (msg.is_string())
    {
      setInfoBufferString(msg.get_var(), msg.get_sdata(), stamp);
    }
    else
    {
      setInfoBufferDouble(msg.get_var(), msg.get_ddata(), stamp);
    }
  }

  return messages;
}

bool HelmAdapter::loadDomain(ros::NodeHandle &private_nh)
{
  domain_entries_.clear();
  XmlRpc::XmlRpcValue domain_param;
  if (!private_nh.getParam("helm/domain", domain_param))
  {
    ROS_ERROR("[bt_executor] Missing helm/domain parameter");
    return false;
  }

  if (domain_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("[bt_executor] helm/domain must be a list");
    return false;
  }

  for (int i = 0; i < domain_param.size(); ++i)
  {
    const auto &entry = domain_param[i];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct || entry.size() != 1)
    {
      continue;
    }

    const std::string name = entry.begin()->first;
    const std::string spec = xmlRpcToString(entry.begin()->second);
    double low = 0.0;
    double high = 0.0;
    unsigned int points = 0;
    if (!parseDomainSpec(spec, low, high, points))
    {
      ROS_ERROR_STREAM("[bt_executor] Invalid domain spec for " << name << ": " << spec);
      return false;
    }
    domain_entries_.push_back({name, low, high, points});
  }

  return !domain_entries_.empty();
}

bool HelmAdapter::loadInitialValues(ros::NodeHandle &private_nh)
{
  initial_doubles_.clear();
  XmlRpc::XmlRpcValue initial_param;
  if (!private_nh.getParam("helm/initial_doubles", initial_param))
  {
    return true;
  }
  if (initial_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_WARN("[bt_executor] helm/initial_doubles must be a map");
    return true;
  }
  for (auto it = initial_param.begin(); it != initial_param.end(); ++it)
  {
    const std::string key = it->first;
    const std::string value_s = xmlRpcToString(it->second);
    try
    {
      initial_doubles_[key] = std::stod(value_s);
    }
    catch (const std::exception &)
    {
      ROS_WARN_STREAM("[bt_executor] Skipping invalid initial double: " << key << "=" << value_s);
    }
  }
  return true;
}

bool HelmAdapter::loadBehaviors(ros::NodeHandle &private_nh)
{
  behavior_definitions_.clear();
  XmlRpc::XmlRpcValue behaviors_param;
  if (!private_nh.getParam("helm/behaviors", behaviors_param))
  {
    ROS_ERROR("[bt_executor] Missing helm/behaviors parameter");
    return false;
  }
  if (behaviors_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("[bt_executor] helm/behaviors must be a list");
    return false;
  }

  for (int i = 0; i < behaviors_param.size(); ++i)
  {
    const auto &entry = behaviors_param[i];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      continue;
    }

    BehaviorDefinition definition;
    if (entry.hasMember("name"))
    {
      definition.name = xmlRpcToString(entry["name"]);
    }
    if (entry.hasMember("type"))
    {
      definition.type = xmlRpcToString(entry["type"]);
    }
    if (entry.hasMember("update_var"))
    {
      definition.update_var = xmlRpcToString(entry["update_var"]);
    }
    if (entry.hasMember("params"))
    {
      definition.parameters = loadParamMap(entry["params"]);
    }
    if (entry.hasMember("conditions"))
    {
      definition.extra_conditions = loadStringArray(entry["conditions"]);
    }
    if (entry.hasMember("runflags"))
    {
      definition.runflags = loadStringArray(entry["runflags"]);
    }
    if (entry.hasMember("endflags"))
    {
      definition.endflags = loadStringArray(entry["endflags"]);
    }

    if (definition.name.empty() || definition.type.empty())
    {
      ROS_WARN_STREAM("[bt_executor] Skipping behavior with missing name/type at index " << i);
      continue;
    }

    behavior_definitions_.push_back(definition);
  }

  return !behavior_definitions_.empty();
}

bool HelmAdapter::buildIvPDomain()
{
  domain_ = std::make_unique<IvPDomain>();
  for (const auto &entry : domain_entries_)
  {
    if (!domain_->addDomain(entry.name, entry.low, entry.high, entry.points))
    {
      ROS_ERROR_STREAM("[bt_executor] Failed to add domain entry: " << entry.name);
      return false;
    }
  }
  return true;
}

bool HelmAdapter::createBehaviors()
{
  activation_vars_.clear();
  for (const auto &definition : behavior_definitions_)
  {
    std::unique_ptr<IvPBehavior> behavior(makeBehavior(definition));
    if (!behavior)
    {
      ROS_ERROR_STREAM("[bt_executor] Failed to create behavior: " << definition.type);
      return false;
    }

    const std::string activation_var = "BT_ACTIVE_" + definition.name;
    activation_vars_[definition.name] = activation_var;

    applyStandardConditions(*behavior, definition, activation_var);
    if (!applyParameters(*behavior, definition))
    {
      ROS_ERROR_STREAM("[bt_executor] Failed to configure behavior: " << definition.name);
      return false;
    }

    behavior_set_->addBehavior(behavior.release());
  }
  return true;
}

IvPBehavior *HelmAdapter::makeBehavior(const BehaviorDefinition &definition) const
{
  if (!domain_)
  {
    return nullptr;
  }

  if (definition.type == "ConstantSpeed")
  {
    return new BHV_ConstantSpeed(*domain_);
  }
  if (definition.type == "GoToDepth")
  {
    return new BHV_GoToDepth(*domain_);
  }
  if (definition.type == "ConstantDepth")
  {
    return new BHV_ConstantDepth(*domain_);
  }
  if (definition.type == "Waypoint")
  {
    return new BHV_Waypoint(*domain_);
  }
  if (definition.type == "StationKeep")
  {
    return new BHV_StationKeep(*domain_);
  }
  if (definition.type == "ConstantHeading")
  {
    return new BHV_ConstantHeading(*domain_);
  }

  ROS_ERROR_STREAM("[bt_executor] Unsupported behavior type: " << definition.type);
  return nullptr;
}

bool HelmAdapter::applyParameters(IvPBehavior &behavior,
                                  const BehaviorDefinition &definition) const
{
  if (!definition.name.empty())
  {
    behavior.setParam("name", definition.name);
  }
  if (!definition.update_var.empty())
  {
    behavior.setParam("updates", definition.update_var);
  }

  for (const auto &param : definition.parameters)
  {
    if (!behavior.setParam(param.first, param.second))
    {
      ROS_WARN_STREAM("[bt_executor] Behavior " << definition.name
                       << " rejected param " << param.first << "=" << param.second);
    }
  }

  for (const auto &flag : definition.runflags)
  {
    if (!flag.empty() && !behavior.setParam("runflag", flag))
    {
      ROS_WARN_STREAM("[bt_executor] Behavior " << definition.name << " rejected runflag " << flag);
    }
  }
  for (const auto &flag : definition.endflags)
  {
    if (!flag.empty() && !behavior.setParam("endflag", flag))
    {
      ROS_WARN_STREAM("[bt_executor] Behavior " << definition.name << " rejected endflag " << flag);
    }
  }
  return true;
}

void HelmAdapter::applyStandardConditions(IvPBehavior &behavior,
                                          const BehaviorDefinition &definition,
                                          const std::string &activation_var) const
{
  behavior.setParam("condition", "DEPLOY=true");
  behavior.setParam("condition", "RETURN=false");
  behavior.setParam("condition", "MOOS_MANUAL_OVERIDE=false");
  behavior.setParam("condition", "DOCKINGFAILED=false");
  behavior.setParam("condition", activation_var + "=true");

  for (const auto &condition : definition.extra_conditions)
  {
    if (!condition.empty())
    {
      behavior.setParam("condition", condition);
    }
  }
}

void HelmAdapter::setInfoBufferDouble(const std::string &key, double value, double stamp)
{
  if (!info_buffer_)
  {
    return;
  }
  info_buffer_->setValue(key, value, stamp);
}

void HelmAdapter::setInfoBufferString(const std::string &key, const std::string &value,
                                      double stamp)
{
  if (!info_buffer_)
  {
    return;
  }
  info_buffer_->setValue(key, value, stamp);
}

}  // namespace bt_executor
