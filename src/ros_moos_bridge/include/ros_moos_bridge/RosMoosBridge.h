#ifndef ROS_MOOS_BRIDGE_H
#define ROS_MOOS_BRIDGE_H

#include <MOOS/libMOOS/App/MOOSApp.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <thread>

class RosMoosBridge : public CMOOSApp {
public:
    RosMoosBridge() = default;
    virtual ~RosMoosBridge();  //  自定义析构函数，确保 ros_spin_thread_ 正确 join

    // --- MOOS核心函数 ---
    bool OnNewMail(MOOSMSG_LIST &NewMail) override;
    bool Iterate() override;
    bool OnConnectToServer() override;
    bool OnStartUp() override;

    // --- 桥接主入口 ---
    void RunBridge(const ros::NodeHandle& nh, const std::string& vehicle_name);

private:
    ros::NodeHandle nh_;
    std::string vehicle_name_;

    // --- ROS 订阅器 ---
    ros::Subscriber sub_odom;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_dvl;

    // --- ROS 发布器 ---
    ros::Publisher pub_heading;
    ros::Publisher pub_pitch;
    ros::Publisher pub_roll;
    ros::Publisher pub_speed;
    ros::Publisher pub_depth;

    // --- ROS → MOOS 回调函数 ---
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void dvlCallback(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg);

    // --- ROS 回调线程 ---
    std::thread ros_spin_thread_;
};

#endif // ROS_MOOS_BRIDGE_H
