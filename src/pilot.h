#ifndef RSC_PILOT
#define RSC_PILOT

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#include "global.h"

class Pilot
{
public:
    Pilot(double speed = 412,
          double translateTime = 9570000,
          double rotationTime = 10370000);

    // Sends the "en" message to the robot
    void enableRobot();

    void stopRobot();
    void setLSpeed(int rotSpeed);
    void setRSpeed(int rotSpeed);

private:
    double speed; // speed of the wheels of the robot
    double rotationTime; // time to complete 1 rotation in microseconds
    double translateTime; // time to drive 1 m in microseconds
    double wheelRadius;
};

#endif
