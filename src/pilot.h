#ifndef RSC_PILOT
#define RSC_PILOT

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "global.h"
#include "odometry.h"

class Pilot
{
public:
    Pilot();
    Pilot(ros::Publisher serialCommandMsg, double speed, double wheelRadius);

    /*
     * Sends the "en" message to the robot
     */
    void enableRobot();

    /*
     * Moves the robot in a straight line
     * length: the distance to traverse, in meters
     * direction: 1 => forward
     *           -1 => backward
     */
    void move(int direction, double length);

    /*
     * Rotates the robot
     * angle: the angle the robot should rotate, in radians
     * direction: 1 => positive rotation (to the right)
     *           -1 => negative rotattion (to the left)
     */
    void rotate(int direction, double angle);

    /*
     * Sets the speed of both wheels to 0
     */
    void stopRobot();

private:
    void setLSpeed(int rotSpeed);
    void setRSpeed(int rotSpeed);

    tf::TransformBroadcaster tfBroadcaster;
    ros::Publisher serialPub;

    Odometry odometry;
    int speed; // speed of the robot in m/s
    int rotSpeed; // speed of the wheels in rotations/min
    int wheelRadius;
};

#endif
