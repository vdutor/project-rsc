#include "odometry.h"

using namespace std;

Odometry::Odometry()
{
    currentPose = {0, 0, 0, -1};
    nextPose = {0, 0, 0, -1};
}

void Odometry::addNextPose(double dX, double dY, double dTheta, time_t time)
{
    nextPose.x = currentPose.x + dX;
    nextPose.y = currentPose.y + dY;
    nextPose.theta = currentPose.theta + dTheta;
    nextPose.arrival_time = time;
}

void Odometry::arrivedAtPose()
{
    previousPoses.push_back(currentPose);
    currentPose = nextPose;
}

void Odometry::broadcastPose(tf::TransformBroadcaster& tfBroadcaster)
{
    ROS_INFO("broadcasting pose");
    if (nextPose.arrival_time == -1 || currentPose.arrival_time == -1)
    {
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "odom"));
        return;
    }

    double dt = (nextPose.arrival_time - currentPose.arrival_time);
    double t = dt == 0 ? 0 : (time(0) - currentPose.arrival_time) / dt;
    double x = currentPose.x + (nextPose.x - currentPose.x) * t;
    double y = currentPose.y + (nextPose.y - currentPose.y) * t;
    double theta = currentPose.theta + (nextPose.theta - currentPose.theta) * t;

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    transform.setRotation(q);
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "odom"));
}
