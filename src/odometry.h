#ifndef RSC_ODOMETRY
#define RSC_ODOMETRY

#include <list>
#include <ctime>
#include <tf/transform_broadcaster.h>

struct TimedPose
{
    double x;
    double y;
    double theta;
    std::time_t arrival_time;
};

class Odometry
{
public:
    TimedPose currentPose;
    TimedPose nextPose;
    std::list<TimedPose> previousPoses;

    Odometry();
    void addNextPose(double dX, double dY, double dTheta, std::time_t time);
    void arrivedAtPose();
    void broadcastPose(tf::TransformBroadcaster& tfBroadcaster);
};

#endif
