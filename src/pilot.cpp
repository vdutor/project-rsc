#include <iostream>
#include <string>

#include <std_msgs/String.h>
#include <project_rsc/move.h>
#include <project_rsc/rotate.h>

#include "pilot.h"

using namespace std;

double defaultWheelRadius = 0.09; // m
double defaultSpeed = 3; // m/s

ros::Publisher serialCommandMsg;
ros::Subscriber serialResponseMsg;
ros::Publisher pilotResponseMsg;
ros::Subscriber moveCommandMsg;
ros::Subscriber rotateCommandMsg;

Pilot* pilot;

Pilot::Pilot()
{
}

Pilot::Pilot(ros::Publisher serialCommandMsg, double speed, double wheelRadius)
{
    this->serialPub = serialCommandMsg;
    this->speed = speed;
    this->wheelRadius = wheelRadius;
    this->rotSpeed = speed * 60 / (wheelRadius * 2 * M_PI);
    ROS_DEBUG("rotational velocity of wheels set to %f", wheelRadius);
}

void serialResponseCB(const std_msgs::String::ConstPtr& msg)
{
    cout << endl << "Response from robot: " << msg->data << endl;
}

void moveCommandCB(const project_rsc::move::ConstPtr& msg)
{
    ROS_INFO("received move command");

    int d = msg->direction;
    double l = msg->length;

    if (d != 1 && d != -1)
    {
        ROS_DEBUG("ERR: invalid direction argument");
        return;
    }
    pilot->move(d, l);
}

void rotateCommandCB(const project_rsc::rotate::ConstPtr& msg)
{
    ROS_INFO("received rotate command");

    int d = msg->direction;
    double a = msg->degrees;

    if (d != 1 && d != -1)
    {
        ROS_DEBUG("ERR: invalid direction argument");
        return;
    }
    pilot->rotate(d, a);

    std_msgs::String arrival_msg;
    msg.data = ROBOT_DONE;
    pilotResponseMsg.publish(arrival_msg);
}

int main(int argc, char* argv[])
{
    // pthread_t cmdThrID;

    ros::init(argc, argv, "pilot_node");
    ROS_INFO("pilot starting");

    ros::NodeHandle nh;

    serialResponseMsg = nh.subscribe(SERIAL_RSP, 100, serialResponseCB);
    serialCommandMsg = nh.advertise<std_msgs::String>(SERIAL_CMD,100);

    moveCommandMsg = nh.subscribe(MOVE_CMD, 100, moveCommandCB);
    rotateCommandMsg = nh.subscribe(ROTATE_CMD, 100, rotateCommandCB);
    pilotResponseMsg = nh.advertise<std_msgs::String>(PILOT_RSP,100);

    pilot = new Pilot(serialCommandMsg, defaultSpeed, defaultWheelRadius);

    ros::spin();

    ROS_INFO("pilot stopping");

    delete pilot;
    return 0;
}
