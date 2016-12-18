#include <iostream>
#include <string>

#include <project_rsc/stop.h>

#include "pilot.h"

using namespace std;

double defaultSpeed = 412; // 10 cm/s

Pilot* pilot;

ros::Publisher rWheelEncoder;
ros::Publisher lWheelEncoder;
ros::Subscriber serialRWheelEncoder;
ros::Subscriber serialLWheelEncoder;
ros::Subscriber rWheelTarget;
ros::Subscriber lWheelTarget;

ros::Publisher serialPub;
ros::Subscriber serialSub;

ros::Subscriber stopCommandSub;

Pilot::Pilot(double speed,
             double translateTime,
             double rotationTime)
{
    this->speed = speed;
    this->translateTime = translateTime;
    this->rotationTime = rotationTime;
    ROS_INFO("rotational velocity of wheels set to %f", speed);
}

void Pilot::enableRobot()
{
    std_msgs::String msg;

    msg.data = "en";
    serialPub.publish(msg);
}

void Pilot::setLSpeed(int rSpeed)
{
    std_msgs::String msg;

    msg.data = "2v" + to_string(rSpeed);
    ROS_INFO("left wheel set to %s", msg.data.c_str());
    serialPub.publish(msg);
}

void Pilot::setRSpeed(int rSpeed)
{
    std_msgs::String msg;

    msg.data = "1v" + to_string(rSpeed);
    ROS_INFO("rigth wheel set to %s", msg.data.c_str());
    serialPub.publish(msg);
}

void Pilot::stopRobot()
{
    std_msgs::String msg;

    msg.data = "1v0";
    serialPub.publish(msg);
    msg.data = "2v0";
    serialPub.publish(msg);
}

void rWheelTargetCB(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("received target value for right wheel");

    //TODO: speed to voltage
    int voltage = msg->data;
    pilot->setRSpeed(voltage);
}

void lWheelTargetCB(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("received target value for left wheel");

    //TODO: speed to voltage
    int voltage = msg->data;
    pilot->setLSpeed(voltage);
}

void serialRWheelEncoderCB(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO("received right wheel encoder value");

    std_msgs::Int16 newMsg = *msg;
    rWheelEncoder.publish(newMsg);
}

void serialLWheelEncoderCB(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO("received left wheel encoder value");

    std_msgs::Int16 newMsg = *msg;
    lWheelEncoder.publish(newMsg);
}

void serialSubCB(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("response from robot %s", msg->data.c_str());
}

void stopCommandCB(const project_rsc::stop::ConstPtr& msg)
{
    ROS_INFO("received stop command");

    pilot->stopRobot();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pilot_node");
    ROS_INFO("pilot starting");

    ros::NodeHandle nh;

    rWheelEncoder = nh.advertise<std_msgs::Int16>(R_WHEEL_ENCODER, 100);
    lWheelEncoder = nh.advertise<std_msgs::Int16>(L_WHEEL_ENCODER, 100);
    serialRWheelEncoder = nh.subscribe(SERIAL_R_WHEEL_ENCODER_VALUE, 100, serialRWheelEncoderCB);
    serialLWheelEncoder = nh.subscribe(SERIAL_L_WHEEL_ENCODER_VALUE, 100, serialLWheelEncoderCB);
    rWheelTarget = nh.subscribe(R_WHEEL_VEL, 100, rWheelTargetCB);
    lWheelTarget = nh.subscribe(L_WHEEL_VEL, 100, lWheelTargetCB);
    serialPub = nh.advertise<std_msgs::String>(SERIAL_CMD,100);
    serialSub = nh.subscribe(SERIAL_RSP, 100, serialSubCB);
    stopCommandSub = nh.subscribe(STOP_CMD, 100, stopCommandCB);

    pilot = new Pilot();

    ros::spin();

    ROS_INFO("pilot stopping");

    delete pilot;
    return 0;
}
