#include "ros_behavior_tree/bt_nodes.hpp"
#include "ros_behavior_tree/nav_subscriber.hpp"

#include <ros/ros.h>

#include <memory>
#include <string>
namespace
{
std::string defaultTopic(const std::string &vehicle, const std::string &suffix)
{
  if (vehicle.empty())
    return suffix;
  return "/" + vehicle + "/" + suffix;
}
}

namespace ros_behavior_tree
{

class BehaviorTreeNode
{
public:
  BehaviorTreeNode()
      : private_nh_("~")
  {
    private_nh_.param("vehicle_name", vehicle_name_, std::string("auh"));
    private_nh_.param("loop_frequency", loop_frequency_, 5.0);

    NavTopics topics;
    private_nh_.param("ros_heading_topic", topics.ros_heading,
                      defaultTopic(vehicle_name_, "current_heading"));
    private_nh_.param("ros_speed_topic", topics.ros_speed,
                      defaultTopic(vehicle_name_, "current_speed"));
    private_nh_.param("ros_depth_topic", topics.ros_depth,
                      defaultTopic(vehicle_name_, "current_depth"));
    private_nh_.param("ros_yaw_topic", topics.ros_yaw,
                      defaultTopic(vehicle_name_, "current_yaw"));
    private_nh_.param("ros_pitch_topic", topics.ros_pitch,
                      defaultTopic(vehicle_name_, "current_pitch"));
    private_nh_.param("ros_roll_topic", topics.ros_roll,
                      defaultTopic(vehicle_name_, "current_roll"));
    private_nh_.param("ros_x_topic", topics.ros_x,
                      defaultTopic(vehicle_name_, "current_x"));
    private_nh_.param("ros_y_topic", topics.ros_y,
                      defaultTopic(vehicle_name_, "current_y"));
    private_nh_.param("ros_deploy_topic", topics.ros_deploy,
                      defaultTopic(vehicle_name_, "DEPLOY"));
    private_nh_.param("ros_return_topic", topics.ros_return,
                      defaultTopic(vehicle_name_, "RETURN"));

    private_nh_.param("nav_heading_topic", topics.nav_heading,
                      defaultTopic(vehicle_name_, "NAV_HEADING"));
    private_nh_.param("nav_speed_topic", topics.nav_speed,
                      defaultTopic(vehicle_name_, "NAV_SPEED"));
    private_nh_.param("nav_depth_topic", topics.nav_depth,
                      defaultTopic(vehicle_name_, "NAV_DEPTH"));
    private_nh_.param("nav_yaw_topic", topics.nav_yaw,
                      defaultTopic(vehicle_name_, "NAV_YAW"));
    private_nh_.param("nav_pitch_topic", topics.nav_pitch,
                      defaultTopic(vehicle_name_, "NAV_PITCH"));
    private_nh_.param("nav_roll_topic", topics.nav_roll,
                      defaultTopic(vehicle_name_, "NAV_ROLL"));
    private_nh_.param("nav_x_topic", topics.nav_x,
                      defaultTopic(vehicle_name_, "NAV_X"));
    private_nh_.param("nav_y_topic", topics.nav_y,
                      defaultTopic(vehicle_name_, "NAV_Y"));
    private_nh_.param("nav_deploy_topic", topics.nav_deploy,
                      defaultTopic(vehicle_name_, "DEPLOY"));
    private_nh_.param("nav_return_topic", topics.nav_return,
                      defaultTopic(vehicle_name_, "RETURN"));

    ROS_INFO_STREAM("[ros_behavior_tree] vehicle_name=" << vehicle_name_);
    ROS_INFO_STREAM("[ros_behavior_tree] ROS topics: heading=" << topics.ros_heading
                    << " speed=" << topics.ros_speed
                    << " depth=" << topics.ros_depth
                    << " yaw=" << topics.ros_yaw
                    << " pitch=" << topics.ros_pitch
                    << " roll=" << topics.ros_roll
                    << " x=" << topics.ros_x
                    << " y=" << topics.ros_y
                    << " deploy=" << topics.ros_deploy
                    << " return=" << topics.ros_return);
    ROS_INFO_STREAM("[ros_behavior_tree] NAV topics: heading=" << topics.nav_heading
                    << " speed=" << topics.nav_speed
                    << " depth=" << topics.nav_depth
                    << " yaw=" << topics.nav_yaw
                    << " pitch=" << topics.nav_pitch
                    << " roll=" << topics.nav_roll
                    << " x=" << topics.nav_x
                    << " y=" << topics.nav_y
                    << " deploy=" << topics.nav_deploy
                    << " return=" << topics.nav_return);

    nav_subscriber_ = std::make_shared<NavDataSubscriber>(nh_, topics, store_);
    manager_ = std::make_shared<BehaviorTreeManager>();
  }

  void spin()
  {
    ros::Rate rate(loop_frequency_);
    while (ros::ok())
    {
      ros::spinOnce();
      if (manager_)
        manager_->tick();
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::string vehicle_name_;
  double loop_frequency_{5.0};

  NavDataStore store_;
  std::shared_ptr<NavDataSubscriber> nav_subscriber_;
  std::shared_ptr<BehaviorTreeManager> manager_;
};

}  // namespace ros_behavior_tree

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_behavior_tree");
  ros_behavior_tree::BehaviorTreeNode node;
  node.spin();
  return 0;
}
