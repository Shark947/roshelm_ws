#include <ros/ros.h>
#include <stdexcept>
#include "ros_moos_bridge/RosMoosBridge.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "bridge_node");
    ros::NodeHandle nh("~");

    std::string moos_file;
    std::string vehicle_name;

    // 读取必要参数
    if (!nh.getParam("moos_mission_file", moos_file)) {
        throw std::runtime_error("[bridge_node] Missing required param: moos_mission_file");
    }

    if (!nh.getParam("vehicle_name", vehicle_name)) {
        throw std::runtime_error("[bridge_node] Missing required param: vehicle_name");
    }

    // 初始化桥接
    RosMoosBridge bridge;
    bridge.RunBridge(nh, vehicle_name);
    bridge.Run("ros_moos_bridge_node", moos_file.c_str());

    ros::Rate loop_rate(50);  // 与 AppTick 保持一致

    while (ros::ok()) {
        ros::spinOnce();
        bridge.Iterate();
        loop_rate.sleep();
    }

    return 0;
}