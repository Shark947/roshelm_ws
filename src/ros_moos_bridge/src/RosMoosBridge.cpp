#include "ros_moos_bridge/RosMoosBridge.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>

RosMoosBridge::~RosMoosBridge() {
    if (ros_spin_thread_.joinable()) {
        ros_spin_thread_.join();
    }
}

void RosMoosBridge::RunBridge(const ros::NodeHandle& nh, const std::string& vehicle_name) {
    nh_ = nh;
    vehicle_name_ = vehicle_name;

    ROS_DEBUG("[Bridge] Starting bridge for vehicle: %s", vehicle_name_.c_str());

    // 订阅 ROS → MOOS
    sub_odom = nh_.subscribe("/auh/pose_gt", 100, &RosMoosBridge::odomCallback, this);
    sub_imu  = nh_.subscribe("/auh/imu", 100, &RosMoosBridge::imuCallback, this);
    sub_dvl  = nh_.subscribe("/auh/dvl", 100, &RosMoosBridge::dvlCallback, this);

    // 发布 MOOS → ROS
    pub_heading = nh_.advertise<std_msgs::Float64>("/auh/desired_heading", 10);
    pub_pitch   = nh_.advertise<std_msgs::Float64>("/auh/desired_pitch", 10);
    pub_roll    = nh_.advertise<std_msgs::Float64>("/auh/desired_roll", 10);
    pub_speed   = nh_.advertise<std_msgs::Float64>("/auh/desired_speed", 10);
    pub_depth   = nh_.advertise<std_msgs::Float64>("/auh/desired_depth", 10);

    //  启动 ROS 回调线程
    ros_spin_thread_ = std::thread([]() {
        ROS_DEBUG("[Bridge] ROS spin thread started.");
        ros::spin();
    });
}

void RosMoosBridge::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double depth = -msg->pose.pose.position.z;

    m_Comms.Notify("NAV_X", x);
    m_Comms.Notify("NAV_Y", y);
    m_Comms.Notify("NAV_DEPTH", depth);

    ROS_DEBUG("[Bridge] odomCallback -> NAV_X=%.2f NAV_Y=%.2f NAV_DEPTH=%.2f", x, y, depth);
}

void RosMoosBridge::dvlCallback(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) {
    double vx = msg->velocity.x;
    double vy = msg->velocity.y;
    double speed = std::hypot(vx, vy);

    m_Comms.Notify("NAV_SPEED", speed);

    ROS_DEBUG("[Bridge] dvlCallback -> NAV_SPEED=%.2f", speed);
}

void RosMoosBridge::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    const auto& q = msg->orientation;
    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    double roll, pitch, yaw;
    tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    double heading = std::fmod((yaw * 180.0 / M_PI + 360.0), 360.0);
    pitch = std::fmod((pitch * 180.0 / M_PI + 360.0), 360.0);
    roll = std::fmod((roll * 180.0 / M_PI + 360.0), 360.0);

    m_Comms.Notify("NAV_HEADING", heading);
    m_Comms.Notify("NAV_PITCH", pitch);
    m_Comms.Notify("NAV_ROLL", roll);

    ROS_DEBUG("[Bridge] imuCallback -> NAV_HEADING=%.2f NAV_PITCH=%.2f NAV_ROLL=%.2f", heading, pitch, roll);
}

bool RosMoosBridge::OnNewMail(MOOSMSG_LIST &NewMail) {
    for (auto& msg : NewMail) {
        std::string var = msg.GetKey();
        double value = msg.GetDouble();

        std_msgs::Float64 ros_msg;
        ros_msg.data = value;

        if (var == "DESIRED_HEADING") {
            pub_heading.publish(ros_msg);
        } else if (var == "DESIRED_PITCH") {
            pub_pitch.publish(ros_msg);
        } else if (var == "DESIRED_ROLL") {
            pub_roll.publish(ros_msg);
        } else if (var == "DESIRED_SPEED") {
            pub_speed.publish(ros_msg);
        } else if (var == "DESIRED_DEPTH") {
            pub_depth.publish(ros_msg);
        }
    }
    return true;
}

bool RosMoosBridge::Iterate() {
    double now = ros::Time::now().toSec();
    m_Comms.Notify("ROS_BRIDGE_HEARTBEAT", now);
    ROS_DEBUG("[Bridge] Iterate - Heartbeat sent");
    return true;
}

bool RosMoosBridge::OnConnectToServer() {
    ROS_DEBUG("[Bridge] Connected to MOOSDB");

    // 显式注册变量
    m_Comms.Register("DESIRED_HEADING", 0);
    m_Comms.Register("DESIRED_PITCH", 0);
    m_Comms.Register("DESIRED_ROLL", 0);
    m_Comms.Register("DESIRED_SPEED", 0);
    m_Comms.Register("DESIRED_DEPTH", 0);

    return true;
}

bool RosMoosBridge::OnStartUp() {
    ROS_DEBUG("[Bridge] OnStartUp called");

    std::string appName = m_MissionReader.GetAppName();
    if (!appName.empty()) {
        ROS_DEBUG("[Bridge] AppName from MOOS: %s", appName.c_str());
    } else {
        ROS_WARN("[Bridge] AppName not found");
    }

    SetAppFreq(50.0);
    SetCommsFreq(50.0);
    return true;
}
