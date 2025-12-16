#include "RosBridge.h"

#include <list>
#include "MBUtils.h"

RosBridge::RosBridge(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
                     const RosNodeConfig &config)
    : nh_(nh), private_nh_(private_nh), config_(config)
{
}

bool RosBridge::initialize()
{
  heading_sub_ = subscribeCurrent(config_.current_heading_topic, "NAV_HEADING");
  speed_sub_ = subscribeCurrent(config_.current_speed_topic, "NAV_SPEED");
  depth_sub_ = subscribeCurrent(config_.current_depth_topic, "NAV_DEPTH");

  deploy_sub_ = subscribeBoolean(config_.deploy_topic, "DEPLOY");
  return_sub_ = subscribeBoolean(config_.return_topic, "RETURN");

  deploy_pub_ = nh_.advertise<std_msgs::Bool>(config_.deploy_topic, 1, true);
  return_pub_ = nh_.advertise<std_msgs::Bool>(config_.return_topic, 1, true);

  std_msgs::Bool default_msg;
  default_msg.data = config_.deploy_default;
  deploy_pub_.publish(default_msg);
  enqueueBoolValue("DEPLOY", default_msg.data, ros::Time::now());

  default_msg.data = config_.return_default;
  return_pub_.publish(default_msg);
  enqueueBoolValue("RETURN", default_msg.data, ros::Time::now());

  desired_scalar_pubs_["DESIRED_HEADING"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_heading_topic, 10);
  desired_scalar_pubs_["DESIRED_SPEED"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_speed_topic, 10);
  desired_scalar_pubs_["DESIRED_DEPTH"] = nh_.advertise<std_msgs::Float64>(
      config_.desired_depth_topic, 10);

  const ros::Time stamp = ros::Time::now();
  for (const auto &entry : config_.nav_defaults)
    enqueueNavValue(entry.first, entry.second, stamp);

  return true;
}

void RosBridge::enqueueNavValue(const std::string &key, double value,
                                const ros::Time &stamp)
{
  std::lock_guard<std::mutex> guard(mail_mutex_);
  pending_mail_.emplace_back(static_cast<char>(MsgType::Notify), key, value,
                             stamp.toSec(), "ros_bridge");
}

void RosBridge::enqueueBoolValue(const std::string &key, bool value,
                                 const ros::Time &stamp)
{
  const std::string text = value ? "true" : "false";

  std::lock_guard<std::mutex> guard(mail_mutex_);
  pending_mail_.emplace_back(static_cast<char>(MsgType::Notify), key, text,
                             stamp.toSec(), "ros_bridge");
}

ros::Subscriber RosBridge::subscribeCurrent(const std::string &topic,
                                            const std::string &nav_key)
{
  if (topic.empty())
    return {};

  return nh_.subscribe<common_msgs::Float64Stamped>(
      topic, 10,
      [this, nav_key](const common_msgs::Float64Stamped::ConstPtr &msg) {
        this->currentValueCallback(msg, nav_key);
      });
}

ros::Subscriber RosBridge::subscribeBoolean(const std::string &topic,
                                            const std::string &key)
{
  if (topic.empty())
    return {};

  return nh_.subscribe<std_msgs::Bool>(
      topic, 10,
      [this, key](const std_msgs::Bool::ConstPtr &msg) {
        this->booleanCallback(msg, key);
      });
}

void RosBridge::currentValueCallback(
    const common_msgs::Float64Stamped::ConstPtr &msg,
    const std::string &nav_key)
{
  enqueueNavValue(nav_key, msg->data, msg->header.stamp);
}

void RosBridge::booleanCallback(const std_msgs::Bool::ConstPtr &msg,
                                const std::string &key)
{
  enqueueBoolValue(key, msg->data, ros::Time::now());
}

void RosBridge::deliverPending(HelmIvP &helm)
{
  std::deque<HelmMsg> mail;
  {
    std::lock_guard<std::mutex> guard(mail_mutex_);
    mail.swap(pending_mail_);
  }

  if (mail.empty())
    return;

  std::list<HelmMsg> pending(mail.begin(), mail.end());
  helm.OnNewMail(pending);
}

void RosBridge::publishDesired(const HelmIvP &helm)
{
  const auto desired = collectDesiredDoubles(helm);
  if (desired.empty())
    return;

  auto publish_scalar = [&](const std::string &key, ros::Publisher &pub) {
    auto it = desired.find(key);
    if (it != desired.end() && pub)
    {
      std_msgs::Float64 msg;
      msg.data = it->second;
      pub.publish(msg);
    }
  };

  for (auto &[key, pub] : desired_scalar_pubs_)
    publish_scalar(key, pub);
}

std::map<std::string, double> RosBridge::collectDesiredDoubles(
    const HelmIvP &helm) const
{
  std::map<std::string, double> desired;

  for (const auto &entry : helm.getOutgoingDoubles())
  {
    if (entry.first.rfind("DESIRED_", 0) == 0)
      desired[entry.first] = entry.second;
  }

  const auto &domain = helm.getIvPDomain();
  const auto &report = helm.getHelmReport();
  for (int i = 0; i < domain.size(); ++i)
  {
    std::string domain_var = domain.getVarName(i);
    std::string desired_key = "DESIRED_" + toupper(domain_var);
    if (desired_key == "DESIRED_COURSE")
      desired_key = "DESIRED_HEADING";

    if (report.hasDecision(domain_var))
      desired[desired_key] = report.getDecision(domain_var);
  }

  return desired;
}
