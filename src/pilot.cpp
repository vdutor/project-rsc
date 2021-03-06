#include <iostream>
#include <string>

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

void Pilot::setRSpeed(int rSpeed)
{
    std_msgs::String msg;

    msg.data = "2v" + to_string(rSpeed);
    ROS_INFO("right wheel set to %s", msg.data.c_str());
    serialPub.publish(msg);
}

void Pilot::setLSpeed(int lSpeed)
{
    std_msgs::String msg;

    // multiply speed by factor to compensate for bad motor
    int adaptedSpeed = 1.018 * lSpeed;
    msg.data = "1v" + to_string(adaptedSpeed);
    ROS_INFO("left wheel set to %s", msg.data.c_str());
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
    // LOL WTF BOOM
    pilot->setLSpeed(-voltage);
}

void lWheelTargetCB(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("received target value for left wheel");

    //TODO: speed to voltage
    int voltage = msg->data;
    // LOL WTF BOOM
    pilot->setRSpeed(voltage);
}

void serialRWheelEncoderCB(const std_msgs::Int16::ConstPtr& msg)
{
    // ROS_INFO("received right wheel encoder value");

    rWheelEncoder.publish(*msg);
}

void serialLWheelEncoderCB(const std_msgs::Int16::ConstPtr& msg)
{
    // ROS_INFO("received left wheel encoder value");

    lWheelEncoder.publish(*msg);
}

void serialSubCB(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("response from robot %s", msg->data.c_str());
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

    pilot = new Pilot();

    ros::spin();

    ROS_INFO("pilot stopping");

    delete pilot;
    return 0;
}
