#include "RosCommandPublisher.h"

#include <utility>

RosCommandPublisher::RosCommandPublisher(
    ros::NodeHandle &nh, const RosNodeConfig &config,
    EnqueueCallback enqueue_callback, StampProvider stamp_provider)
    : nh_(nh),
      config_(config),
      enqueue_callback_(std::move(enqueue_callback)),
      stamp_provider_(std::move(stamp_provider))
{
}

bool RosCommandPublisher::initialize()
{
  deploy_sub_ = subscribeBoolean(config_.deploy_topic, "DEPLOY");
  return_sub_ = subscribeBoolean(config_.return_topic, "RETURN");

  deploy_pub_ = nh_.advertise<std_msgs::Bool>(config_.deploy_topic, 1, true);
  return_pub_ = nh_.advertise<std_msgs::Bool>(config_.return_topic, 1, true);

  std_msgs::Bool default_msg;
  default_msg.data = config_.deploy_default;
  deploy_pub_.publish(default_msg);
  if (enqueue_callback_)
    enqueue_callback_("DEPLOY", default_msg.data, ros::Time::now());

  default_msg.data = config_.return_default;
  return_pub_.publish(default_msg);
  if (enqueue_callback_)
    enqueue_callback_("RETURN", default_msg.data, ros::Time::now());

  return true;
}

ros::Subscriber RosCommandPublisher::subscribeBoolean(const std::string &topic,
                                                      const std::string &key)
{
  if (topic.empty())
    return {};

  return nh_.subscribe<std_msgs::Bool>(
      topic, 10,
      [this, key](const std_msgs::Bool::ConstPtr &msg) {
        this->handleBoolean(msg, key);
      });
}

void RosCommandPublisher::handleBoolean(const std_msgs::Bool::ConstPtr &msg,
                                        const std::string &key)
{
  if (!enqueue_callback_)
    return;

  ros::Time stamp = stamp_provider_ ? stamp_provider_() : ros::Time();
  if (stamp.isZero())
    stamp = ros::Time::now();

  enqueue_callback_(key, msg->data, stamp);
}
