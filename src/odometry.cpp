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
