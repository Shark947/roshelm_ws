#pragma once
// ===================
//  ROS1 兼容适配层
// ===================

// ------- 节点句柄 --------
#include <ros/node_handle.h>
using ControllerNodeHandle = ros::NodeHandle&;

// ------- 时间类型 --------
#include <ros/time.h>
using ControllerTime = ros::Time;

// ------- 智能指针 --------
#include <boost/shared_ptr.hpp>
template <typename T>
using ControllerSharedPtr = boost::shared_ptr<T>;

// ------- 订阅/发布 --------
#include <ros/ros.h>
using ControllerSubscriber = ros::Subscriber;
using ControllerPublisher  = ros::Publisher;

// ------- 定时器类型 --------
#include <ros/timer.h>
using ControllerTimer = ros::Timer;
using ControllerTimerEvent = ros::TimerEvent;

// ------- 日志接口 --------
#include <ros/console.h>
#define CONTROLLER_INFO_STREAM(msg)         ROS_INFO_STREAM(msg)
#define CONTROLLER_ERROR_STREAM(msg)        ROS_ERROR_STREAM(msg)
#define CONTROLLER_WARN_STREAM(msg)         ROS_WARN_STREAM(msg)
#define CONTROLLER_DEBUG_STREAM(msg)        ROS_DEBUG_STREAM(msg)
#define CONTROLLER_INFO_STREAM_THROTTLE(period, msg)  ROS_INFO_STREAM_THROTTLE(period, msg)
#define CONTROLLER_WARN_STREAM_THROTTLE(period, msg)  ROS_WARN_STREAM_THROTTLE(period, msg)
#define CONTROLLER_ERROR_STREAM_THROTTLE(period, msg) ROS_ERROR_STREAM_THROTTLE(period, msg)

// ------- 其它常用类型/宏(可按需补充) --------
// #include <std_msgs/Float64.h>
// using ControllerFloat64Msg = std_msgs::Float64;

// 日后如有自定义消息、定时器等通用工具，可以继续往下补充
