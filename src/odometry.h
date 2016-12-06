#ifndef RSC_ODOMETRY
#define RSC_ODOMETRY

#include <list>
#include <ctime>
#include <tf/transform_broadcaster.h>

struct Pose
{
    double x;
    double y;
    double theta;
};

struct TimedPose
{
    Pose pose;
    std::time_t arrival_time;
};

class Odometry
{
public:
    Pose currentPose;
    Pose nextPose;
    std::list<Pose> previousPoses;

    Odometry();
    void addPose(double dX, double dY, double dTheta);
    void broadcastPose(tf::TransformBroadcaster& tfBroadcaster);
};

#endif
