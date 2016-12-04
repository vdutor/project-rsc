#ifndef RSC_ODOMETRY
#define RSC_ODOMETRY

#include <list>

struct Pose
{
    double x;
    double y;
    double theta;
};

class Odometry
{
public:
    Pose currentPose;
    std::list<Pose> previousPoses;

    Odometry();
    void addPose(double dX, double dY, double dT);
};

#endif
