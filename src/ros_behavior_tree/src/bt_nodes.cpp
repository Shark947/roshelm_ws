#include "ros_behavior_tree/bt_nodes.hpp"

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/package.h>

namespace ros_behavior_tree
{

namespace
{
class DeployTriggered : public BT::ConditionNode
{
public:
  DeployTriggered(const std::string &name,
                  const BT::NodeConfiguration &config,
                  const NavDataStore *store)
      : BT::ConditionNode(name, config), store_(store)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (!store_)
      return BT::NodeStatus::FAILURE;

    bool value = false;
    if (!store_->preferredDeploy(value, ros::Duration(0.0)))
      return BT::NodeStatus::FAILURE;
    return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  const NavDataStore *store_{nullptr};
};

class DummyAction : public BT::StatefulActionNode
{
public:
  DummyAction(const std::string &name, const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("duration")};
  }

  BT::NodeStatus onStart() override
  {
    run_duration_ = 1.0;
    getInput("duration", run_duration_);
    start_time_ = ros::Time::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if ((ros::Time::now() - start_time_).toSec() >= run_duration_)
      return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    start_time_ = ros::Time(0.0);
  }

private:
  ros::Time start_time_;
  double run_duration_{1.0};
};
}  // namespace

void NavDataStore::updateHeading(double value, const ros::Time &stamp,
                                 bool nav_style)
{
  NavData &data = nav_style ? nav_data_ : ros_data_;
  data.heading.value = value;
  data.heading.stamp = stamp;
  data.heading.has_value = true;
}

void NavDataStore::updateSpeed(double value, const ros::Time &stamp,
                               bool nav_style)
{
  NavData &data = nav_style ? nav_data_ : ros_data_;
  data.speed.value = value;
  data.speed.stamp = stamp;
  data.speed.has_value = true;
}

void NavDataStore::updateDepth(double value, const ros::Time &stamp,
                               bool nav_style)
{
  NavData &data = nav_style ? nav_data_ : ros_data_;
  data.depth.value = value;
  data.depth.stamp = stamp;
  data.depth.has_value = true;
}

void NavDataStore::updateYaw(double value, const ros::Time &stamp,
                             bool nav_style)
{
  NavData &data = nav_style ? nav_data_ : ros_data_;
  data.yaw.value = value;
  data.yaw.stamp = stamp;
  data.yaw.has_value = true;
}

void NavDataStore::updatePitch(double value, const ros::Time &stamp,
                               bool nav_style)
{
  NavData &data = nav_style ? nav_data_ : ros_data_;
  data.pitch.value = value;
  data.pitch.stamp = stamp;
  data.pitch.has_value = true;
}

void NavDataStore::updateRoll(double value, const ros::Time &stamp,
                              bool nav_style)
{
  NavData &data = nav_style ? nav_data_ : ros_data_;
  data.roll.value = value;
  data.roll.stamp = stamp;
  data.roll.has_value = true;
}

void NavDataStore::updateX(double value, const ros::Time &stamp,
                           bool nav_style)
{
  NavData &data = nav_style ? nav_data_ : ros_data_;
  data.x.value = value;
  data.x.stamp = stamp;
  data.x.has_value = true;
}

void NavDataStore::updateY(double value, const ros::Time &stamp,
                           bool nav_style)
{
  NavData &data = nav_style ? nav_data_ : ros_data_;
  data.y.value = value;
  data.y.stamp = stamp;
  data.y.has_value = true;
}

void NavDataStore::updateDeploy(bool value, const ros::Time &stamp,
                                bool nav_style)
{
  NavData &data = nav_style ? nav_data_ : ros_data_;
  data.deploy.value = value;
  data.deploy.stamp = stamp;
  data.deploy.has_value = true;
}

void NavDataStore::updateReturn(bool value, const ros::Time &stamp,
                                bool nav_style)
{
  NavData &data = nav_style ? nav_data_ : ros_data_;
  data.return_home.value = value;
  data.return_home.stamp = stamp;
  data.return_home.has_value = true;
}

bool NavDataStore::sampleFresh(const NavSample &sample,
                               const ros::Duration &timeout)
{
  if (!sample.has_value)
    return false;
  if (timeout.isZero())
    return true;
  return (ros::Time::now() - sample.stamp) <= timeout;
}

bool NavDataStore::sampleFresh(const BoolSample &sample,
                               const ros::Duration &timeout)
{
  if (!sample.has_value)
    return false;
  if (timeout.isZero())
    return true;
  return (ros::Time::now() - sample.stamp) <= timeout;
}

bool NavDataStore::getPreferredSample(const NavSample &nav_sample,
                                      const NavSample &ros_sample,
                                      const ros::Duration &timeout,
                                      double &out)
{
  if (sampleFresh(nav_sample, timeout))
  {
    out = nav_sample.value;
    return true;
  }
  if (sampleFresh(ros_sample, timeout))
  {
    out = ros_sample.value;
    return true;
  }
  return false;
}

bool NavDataStore::getPreferredSample(const BoolSample &nav_sample,
                                      const BoolSample &ros_sample,
                                      const ros::Duration &timeout,
                                      bool &out)
{
  if (sampleFresh(nav_sample, timeout))
  {
    out = nav_sample.value;
    return true;
  }
  if (sampleFresh(ros_sample, timeout))
  {
    out = ros_sample.value;
    return true;
  }
  return false;
}

bool NavDataStore::hasFreshData(const ros::Duration &timeout) const
{
  double value = 0.0;
  return getPreferredSample(nav_data_.heading, ros_data_.heading, timeout, value) &&
         getPreferredSample(nav_data_.speed, ros_data_.speed, timeout, value) &&
         getPreferredSample(nav_data_.depth, ros_data_.depth, timeout, value);
}

bool NavDataStore::preferredHeading(double &out,
                                    const ros::Duration &timeout) const
{
  return getPreferredSample(nav_data_.heading, ros_data_.heading, timeout, out);
}

bool NavDataStore::preferredSpeed(double &out,
                                  const ros::Duration &timeout) const
{
  return getPreferredSample(nav_data_.speed, ros_data_.speed, timeout, out);
}

bool NavDataStore::preferredDepth(double &out,
                                  const ros::Duration &timeout) const
{
  return getPreferredSample(nav_data_.depth, ros_data_.depth, timeout, out);
}

bool NavDataStore::preferredYaw(double &out,
                                const ros::Duration &timeout) const
{
  return getPreferredSample(nav_data_.yaw, ros_data_.yaw, timeout, out);
}

bool NavDataStore::preferredPitch(double &out,
                                  const ros::Duration &timeout) const
{
  return getPreferredSample(nav_data_.pitch, ros_data_.pitch, timeout, out);
}

bool NavDataStore::preferredRoll(double &out,
                                 const ros::Duration &timeout) const
{
  return getPreferredSample(nav_data_.roll, ros_data_.roll, timeout, out);
}

bool NavDataStore::preferredX(double &out,
                              const ros::Duration &timeout) const
{
  return getPreferredSample(nav_data_.x, ros_data_.x, timeout, out);
}

bool NavDataStore::preferredY(double &out,
                              const ros::Duration &timeout) const
{
  return getPreferredSample(nav_data_.y, ros_data_.y, timeout, out);
}

bool NavDataStore::preferredDeploy(bool &out,
                                   const ros::Duration &timeout) const
{
  return getPreferredSample(nav_data_.deploy, ros_data_.deploy, timeout, out);
}

bool NavDataStore::preferredReturn(bool &out,
                                   const ros::Duration &timeout) const
{
  return getPreferredSample(nav_data_.return_home, ros_data_.return_home,
                            timeout, out);
}

void BehaviorTreeManager::tick()
{
  if (!tree_loaded_)
    return;
  tree_.tickRoot();
}

BehaviorTreeManager::BehaviorTreeManager(const ros::NodeHandle &nh,
                                         NavDataStore *store)
    : nh_(nh), store_(store)
{
  std::string bt_xml;
  std::string default_xml =
      ros::package::getPath("ros_behavior_tree") + "/config/test.xml";
  nh_.param("bt_xml", bt_xml, default_xml);

  bool groot_enable = true;
  int groot_publisher_port = 1666;
  int groot_server_port = 1667;
  nh_.param("groot_enable", groot_enable, true);
  nh_.param("groot_publisher_port", groot_publisher_port, 1666);
  nh_.param("groot_server_port", groot_server_port, 1667);

  auto deploy_builder = [this](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<DeployTriggered>(name, config, store_);
  };
  factory_.registerBuilder<DeployTriggered>("DeployTriggered", deploy_builder);
  factory_.registerNodeType<DummyAction>("DummyAction");

  try
  {
    tree_ = factory_.createTreeFromFile(bt_xml);
    tree_loaded_ = true;
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM("[ros_behavior_tree] Failed to load BT XML: " << ex.what());
    tree_loaded_ = false;
  }

  if (tree_loaded_ && groot_enable)
  {
    groot_publisher_ = std::make_unique<BT::PublisherZMQ>(
        tree_, groot_publisher_port, groot_server_port);
    ROS_INFO_STREAM("[ros_behavior_tree] Groot ZMQ publisher enabled (pub="
                    << groot_publisher_port << ", server=" << groot_server_port
                    << ")");
  }
  ROS_INFO_STREAM("[ros_behavior_tree] Behavior tree loaded from " << bt_xml);
}

}  // namespace ros_behavior_tree
