#include <math.h>
#include <unistd.h>
#include <string>

#include <std_msgs/String.h>

#include "pilot.h"

using namespace std;

void Pilot::enableRobot()
{
    std_msgs::String msg;

    msg.data = "en";
    serialPub.publish(msg);
}

void Pilot::setLSpeed(int rotSpeed)
{
    std_msgs::String msg;

    msg.data = "2v" + to_string(rotSpeed);
    serialPub.publish(msg);
}

void Pilot::setRSpeed(int rotSpeed)
{
    std_msgs::String msg;

    msg.data = "1v" + to_string(rotSpeed);
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

void Pilot::move(int direction, double length)
{
    unsigned int sleepTime = length / speed * 1000000; // microseconds
    setLSpeed(direction * rotSpeed);
    setRSpeed(-direction * rotSpeed);
    ROS_INFO("sleeping for %u ms", sleepTime);
    usleep(sleepTime);
    stopRobot();
}

void Pilot::rotate(int direction, double angle)
{
    double length = angle * wheelRadius;
    unsigned int sleepTime = length / speed * 1000000; // microseconds
    setLSpeed(direction * rotSpeed);
    setRSpeed(direction * rotSpeed);
    usleep(sleepTime);
    stopRobot();
}
