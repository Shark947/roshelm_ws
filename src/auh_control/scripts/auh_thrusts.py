#! /usr/bin/env python
# Haisu Xing

import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Float64

def pubThrust0(thrust):
    output = FloatStamped()
    output.header.stamp = rospy.Time.now()
    output.data = thrust.data
    command_pub = rospy.Publisher("/auh/thrusters/0/input", FloatStamped,queue_size=10)
    command_pub.publish(output)

def pubThrust1(thrust):
    output = FloatStamped()
    output.header.stamp = rospy.Time.now()
    output.data = thrust.data
    command_pub = rospy.Publisher("/auh/thrusters/1/input", FloatStamped,queue_size=10)
    command_pub.publish(output)

def pubThrust2(thrust):
    output = FloatStamped()
    output.header.stamp = rospy.Time.now()
    output.data = thrust.data
    command_pub = rospy.Publisher("/auh/thrusters/2/input", FloatStamped,queue_size=10)
    command_pub.publish(output)

def pubThrust3(thrust):
    output = FloatStamped()
    output.header.stamp = rospy.Time.now()
    output.data = thrust.data
    command_pub = rospy.Publisher("/auh/thrusters/3/input", FloatStamped,queue_size=10)
    command_pub.publish(output)

def pubThrust4(thrust):
    output = FloatStamped()
    output.header.stamp = rospy.Time.now()
    output.data = thrust.data
    command_pub = rospy.Publisher("/auh/thrusters/4/input", FloatStamped,queue_size=10)
    command_pub.publish(output)

def pubThrust5(thrust):
    output = FloatStamped()
    output.header.stamp = rospy.Time.now()
    output.data = thrust.data
    command_pub = rospy.Publisher("/auh/thrusters/5/input", FloatStamped,queue_size=10)
    command_pub.publish(output)
    

if __name__ == "__main__":

    rospy.init_node("auh_thrusts")  

    sub0 = rospy.Subscriber("/auh/t0", Float64, pubThrust0, queue_size=10)
    sub1 = rospy.Subscriber("/auh/t1", Float64, pubThrust1, queue_size=10)
    sub2 = rospy.Subscriber("/auh/t2", Float64, pubThrust2, queue_size=10)
    sub3 = rospy.Subscriber("/auh/t3", Float64, pubThrust3, queue_size=10)
    sub4 = rospy.Subscriber("/auh/t4", Float64, pubThrust4, queue_size=10)
    sub5 = rospy.Subscriber("/auh/t5", Float64, pubThrust5, queue_size=10)

    # spin()
    rospy.spin()