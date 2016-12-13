#include <math.h>
#include <unistd.h>
#include <string>
#include <ctime>

#include <std_msgs/String.h>

#include "pilot.h"

using namespace std;

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

void Pilot::move(int direction, double length)
{
    unsigned int driveTime = length * translateTime;

    double dX = direction * length * cos(odometry.currentPose.theta);
    double dY = direction * length * sin(odometry.currentPose.theta);
    odometry.addNextPose(dX, dY, 0, time(0) + driveTime / 1000000);

    setLSpeed(direction * speed);
    setRSpeed(-direction * speed);

    ROS_INFO("sleeping for %u microsec", driveTime);
    usleep(driveTime); // sleep in microseconds
    stopRobot();

    odometry.arrivedAtPose();
    // odometry.broadcastPose(tfBroadcaster);
}

void Pilot::rotate(int direction, double angle)
{
    unsigned int rotateTime = angle / (2*M_PI) * rotationTime;

    odometry.addNextPose(0, 0, direction * angle, time(0) + rotateTime / 1000000);

    setLSpeed(direction * speed);
    setRSpeed(direction * speed);

    ROS_INFO("sleeping for %u microsec", rotateTime);
    usleep(rotateTime); // sleep in microseconds
    stopRobot();

    odometry.arrivedAtPose();
    // odometry.broadcastPose(tfBroadcaster);
}
