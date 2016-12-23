#include "wallfollowing.hpp"
#include <cmath>
#include <math.h>
#include <iostream>

#define PI 3.141592
#define SUB_BUFFER_SIZE 1       // Size of buffer for subscriber.
#define PUB_BUFFER_SIZE 1000    // Size of buffer for publisher.
#define WALL_DISTANCE .5
#define MAX_SPEED 5
#define ROTATION_FACTOR 3
#define P_DEFAULT 10            // Proportional constant for controller
#define D_DEFAULT 5             // Derivative constant for controller
#define ANGLE_COEF 1            // Proportional constant for angle controller
#define DIRECTION -1            // 1 for wall on the left side of the robot (-1 for the right side).
#define PUB_TOPIC "/twist"
#define SUB_TOPIC "/scan"
#define TO_DEGREE(r) (((r)/PI) * 180.0)
#define DEBUG

using namespace std;

WallFollowing::WallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an)
{
    wallDistance = wallDist;
    maxSpeed = maxSp;
    direction = dir;
    P = pr;
    D = di;
    angleCoef = an;
    e = 0;
    angleMin = 0;  //angle, at which was measured the shortest distance
    pubMessage = pub;
}

WallFollowing::~WallFollowing()
{
}

//Publisher
void WallFollowing::publishMessage()
{
    //preparing message
    geometry_msgs::Twist msg;

    double fac = 1;
    if (distFront < WALL_DISTANCE * 1.5)
        fac = distFront - WALL_DISTANCE;


    msg.angular.z = -ROTATION_FACTOR * (direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2)) / max(0.01, fac);
    if (distFront < wallDistance)
    {
        msg.linear.x = 0;
    }
    else if (distFront < wallDistance * 2)
    {
        msg.linear.x = 0.5*maxSpeed;
    }
    else if (fabs(angleMin)>1.70)
    {
        msg.linear.x = 0.4*maxSpeed;
    }
    else
    {
        msg.linear.x = maxSpeed;
    }

#ifdef DEBUG
    cout << "angle vel: " << msg.angular.z;
    cout << " speed: " << msg.linear.x << endl;
#endif

    //publishing message
    pubMessage.publish(msg);
}

//Subscriber
void WallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    e = 0; // TODO
    int size = msg->ranges.size();

    //Variables whith index of highest and lowest value in array.
    int minIndex = size * (direction+1)/4.0 + 80;
    int maxIndex = size * (direction+3)/4.0 - 80;

    //This cycle goes through array and finds minimum
    for(int i = minIndex; i < maxIndex; i++)
    {
        if (isnan(msg->ranges.at(i))) continue;

        if (msg->ranges.at(minIndex) < 0.10 || isnan(msg->ranges.at(minIndex)) ||
            (msg->ranges.at(i) < msg->ranges.at(minIndex) && msg->ranges.at(i) > 0.10))
        {
            minIndex = i;
        }
    }

    //Calculation of angles from indexes and storing data to class variables.
    angleMin = (minIndex-size/2)*msg->angle_increment;
    double distMin;
    distMin = msg->ranges.at(minIndex);

    // find minimum distance in front of the robot
    int delta = 20;
    distFront = msg->ranges.at(size/2);
    for (int i = -delta; i <= delta; i++)
    {
        int j = size/2 + i;
        if (isnan(msg->ranges.at(j))) continue;
        distFront =min(distFront,(double) msg->ranges.at(j));
    }

    diffE = (distMin - wallDistance) - e;
    e = distMin - wallDistance;

#ifdef DEBUG
    cout << "angleMin: " << TO_DEGREE(angleMin);
    cout << " distMin: " << distMin;
    cout << " distFront: " << distFront;
    cout << " error: " << e << endl;
#endif

    //Invoking method for publishing message
    publishMessage();
}


int main(int argc, char **argv)
{
    //Initialization of node
    ros::init(argc, argv, "wallFollowing");
    ros::NodeHandle n;

    //Creating publisher
    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUB_TOPIC, PUB_BUFFER_SIZE);

    //Creating object, which stores data from sensors and has methods for
    //publishing and subscribing
    WallFollowing *follower = new WallFollowing(pubMessage, WALL_DISTANCE, MAX_SPEED, DIRECTION, P_DEFAULT, D_DEFAULT, 1);

    //Creating subscriber and publisher
    ros::Subscriber sub = n.subscribe(SUB_TOPIC, SUB_BUFFER_SIZE, &WallFollowing::messageCallback, follower);
    ros::spin();

    return 0;
}
