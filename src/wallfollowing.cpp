#include "wallfollowing.hpp"
#include <math.h>
#define PI 3.141592

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

    msg.angular.z = direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2);    //PD controller

    if (distFront < wallDistance){
        msg.linear.x = 0;
    }
    else if (distFront < wallDistance * 2){
        msg.linear.x = 0.5*maxSpeed;
    }
    else if (fabs(angleMin)>1.75){
        msg.linear.x = 0.4*maxSpeed;
    }
    else {
        msg.linear.x = maxSpeed;
    }

    //publishing message
    pubMessage.publish(msg);
}

//Subscriber
void WallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //vector<float> ranges = msg->ranges;
    int size = msg->ranges.size();

    //Variables whith index of highest and lowest value in array.
    int minIndex = size*(direction+1)/4;
    int maxIndex = size*(direction+3)/4;

    //This cycle goes through array and finds minimum
    for(int i = minIndex; i < maxIndex; i++)
    {
        if (msg->ranges.at(i) < msg->ranges.at(minIndex) && msg->ranges.at(i) > 0.0){
            minIndex = i;
        }
    }

    //Calculation of angles from indexes and storing data to class variables.
    angleMin = (minIndex-size/2)*msg->angle_increment;
    double distMin;
    distMin = msg->ranges.at(minIndex);
    distFront = msg->ranges.at(size/2);
    diffE = (distMin - wallDistance) - e;
    e = distMin - wallDistance;

    //Invoking method for publishing message
    publishMessage();
}

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
#define WALL_DISTANCE 0.13
#define MAX_SPEED 0.1
#define P 10    // Proportional constant for controller
#define D 5     // Derivative constant for controller
#define ANGLE_COEF 1    // Proportional constant for angle controller
#define DIRECTION 1 // 1 for wall on the left side of the robot (-1 for the right side).
// #define PUBLISHER_TOPIC "/syros/base_cmd_vel"
#define PUBLISHER_TOPIC "/cmd_vel"
// #define SUBSCRIBER_TOPIC "/syros/laser_laser"
#define SUBSCRIBER_TOPIC "/scan"

int main(int argc, char **argv)
{
    //Initialization of node
    ros::init(argc, argv, "wallFollowing");
    ros::NodeHandle n;

    //Creating publisher
    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);

    //Creating object, which stores data from sensors and has methods for
    //publishing and subscribing
    WallFollowing *follower = new WallFollowing(pubMessage, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, 1);

    //Creating subscriber and publisher
    ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &WallFollowing::messageCallback, follower);
    ros::spin();

    return 0;
}