#include "odometry.h"

using namespace std;

Odometry::Odometry()
{
    currentPose = {0, 0, 0};
}

void Odometry::addPose(double dX, double dY, double dT)
{
    previousPoses.push_back(currentPose);
    currentPose.x += dX;
    currentPose.y += dY;
    currentPose.theta += dT;
}

void Odometry::broadcastPose(tf::TransformBroadcaster& tfBroadcaster)
{
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(currentPose.x,
                                     currentPose.y,
                                     0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, currentPose.theta);
    transform.setRotation(q);
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "odom"));
}
