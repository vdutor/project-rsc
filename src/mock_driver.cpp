#include <unistd.h>
#include <iostream>
#include <string>

#include <project_rsc/stop.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#include "global.h"

ros::Publisher serialPub;
ros::Subscriber serialSub;
ros::Subscriber serialRWheelEncoder;
ros::Subscriber serialLWheelEncoder;

void enableRobot()
{
    std_msgs::String msg;

    msg.data = "en";
    serialPub.publish(msg);
}

void moveRobot()
{
    std_msgs::String msg;

    msg.data = "1 POS";
    serialPub.publish(msg);
    msg.data = "2 POS";
    serialPub.publish(msg);

    // msg.data = "1v-200";
    // serialPub.publish(msg);
    // msg.data = "2v200";
    // serialPub.publish(msg);
}

void stopRobot()
{
    std_msgs::String msg;

    msg.data = "1v0";
    serialPub.publish(msg);
    msg.data = "2v0";
    serialPub.publish(msg);
}

void serialRWheelEncoderCB(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO("received right wheel encoder value: %d", msg->data);
}

void serialLWheelEncoderCB(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO("received left wheel encoder value: %d", msg->data);
}

void serialSubCB(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("response from robot %s", msg->data.c_str());
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mockdriver_node");
    ROS_INFO("mock driver starting");

    ros::NodeHandle nh;
    serialPub = nh.advertise<std_msgs::String>(SERIAL_CMD,100);
    serialSub = nh.subscribe(SERIAL_RSP, 100, serialSubCB);
    serialRWheelEncoder = nh.subscribe(SERIAL_R_WHEEL_ENCODER_VALUE, 100, serialRWheelEncoderCB);
    serialLWheelEncoder = nh.subscribe(SERIAL_L_WHEEL_ENCODER_VALUE, 100, serialLWheelEncoderCB);

    enableRobot();
    std::cin.get();
    moveRobot();
    usleep(4000000);
    std::cin.get();
    stopRobot();

    usleep(1000000);
    return 0;
}
