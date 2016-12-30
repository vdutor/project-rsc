#include "wallfollowing.hpp"
#include <unistd.h>
#include <cmath>
#include <math.h>
#include <iostream>

#define PI 3.141592
#define SUB_BUFFER_SIZE 1       // Size of buffer for subscriber.
#define PUB_BUFFER_SIZE 1000    // Size of buffer for publisher.
#define WALL_DISTANCE 0.5
#define MAX_SPEED 5
#define ROTATION_FACTOR 20
#define P_DEFAULT 5            // Proportional constant for controller
#define D_DEFAULT 0             // Derivative constant for controller
#define ANGLE_COEF 1            // Proportional constant for angle controller
#define DIRECTION -1            // 1 for wall on the left side of the robot (-1 for the right side).
#define PUB_TOPIC "/twist"
#define SUB_TOPIC "/scan"
#define TO_DEGREE(r) (((r)/PI) * 180.0)
#define DEBUG

using namespace std;

WallFollowing::WallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an)
{
    state = 0;
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
    {
        fac = distFront - WALL_DISTANCE;
        msg.angular.z = direction * 10;
    }
    else
    {
        double current_z = -ROTATION_FACTOR * (direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2));
        msg.angular.z = current_z;
    }

    //prev_z = msg.angular.z;

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
    if (state != 3)
        pubMessage.publish(msg);
}

void WallFollowing::stopCallback(const std_msgs::String::ConstPtr& msg)
{
    cout << "Received stop command" << endl;
    state = 3;
    // stop driving
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0;
    twist_msg.angular.z = 0;
    pubMessage.publish(twist_msg);
}

//Subscriber
void WallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (state == 3) return;
    //e = 0; // TODO
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
    distFront = 5.;
    for (int i = -delta; i <= delta; i++)
    {
        int j = size/2 + i;
        if (isnan(msg->ranges.at(j)) || msg->ranges.at(j) < 0.1) continue;
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

    if (state == 0)
    {
        if (distFront > 0.5)
        {
            // drive forward
            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = 5;
            twist_msg.angular.z = 0;
            pubMessage.publish(twist_msg);
            return;
        }
        state = 1;
    }

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

    // Creating subscriber
    ros::Subscriber subStop = n.subscribe("stop", SUB_BUFFER_SIZE, &WallFollowing::stopCallback, follower);

    //Creating subscriber and publisher
    ros::Subscriber sub = n.subscribe(SUB_TOPIC, SUB_BUFFER_SIZE, &WallFollowing::messageCallback, follower);
    ros::spin();

    return 0;
}
