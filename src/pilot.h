#ifndef RSC_PILOT
#define RSC_PILOT

#include <ros/ros.h>

#include "global.h"
#include "odometry.h"

class Pilot
{
public:
    Pilot();
    Pilot(ros::Publisher serialCommandMsg,
          double speed = 412,
          double translateTime = 9570000,
          double rotationTime = 10370000);

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
     * Sets linear velocity
     * direction: 1 => forward
     *           -1 => backward
     */
    void setLinearVelocity(int direction, float vel)
    {

    }

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

    /*
     * Broadcasts a tf message
     */
    void publishOdometry();

private:
    void setLSpeed(int rotSpeed);
    void setRSpeed(int rotSpeed);

    tf::TransformBroadcaster tfBroadcaster;
    ros::Publisher serialPub;

    Odometry odometry;
    double speed; // speed of the wheels of the robot
    double rotationTime; // time to complete 1 rotation in microseconds
    double translateTime; // time to drive 1 m in microseconds
    double wheelRadius;
};

#endif
