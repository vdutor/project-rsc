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
#define ROTATION_FACTOR 60
#define P_DEFAULT 6            // Proportional constant for controller
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
        msg.angular.z = direction * 15;
    }
    else
    {
        double current_z = -ROTATION_FACTOR * (direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2));
        current_z = 0.7 * prev_z + 0.3 * current_z;
        prev_z = current_z;
        msg.angular.z = current_z;
    }

    //prev_z = msg.angular.z;

    if (distFront < wallDistance)
    {
        msg.linear.x = 0;
    }
    else if (distFront < wallDistance * 2)
    {
        msg.linear.x = 0.7*maxSpeed;
    }
    else if (fabs(angleMin)>1.70)
    {
        msg.linear.x = 0.6*maxSpeed;
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
    if (state == 1)
        pubMessage.publish(msg);
}

void WallFollowing::stopCallback(const std_msgs::String::ConstPtr& msg)
{
    if (state != 1)
        return;

    cout << "Received stop command" << endl;
    state = 2;

    // turn 90 degrees
    geometry_msgs::Twist twist_msg;
    ros::Rate r(10); // 10 hz
    int counter = 29;
    while (ros::ok() && counter > 0)
    {
        twist_msg.angular.z = -30;
        twist_msg.linear.x = 0;
        pubMessage.publish(twist_msg);
        r.sleep();
        counter--;
    }
    twist_msg.angular.z = 0;
    twist_msg.linear.x = 0;
    pubMessage.publish(twist_msg);

    state = 3;
}

//Subscriber
void WallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
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
    angleMin = (minIndex-384)*msg->angle_increment;
    double distMin;
    distMin = msg->ranges.at(minIndex);

    // find minimum distance in front of the robot
    int delta = 20;
    double distFrontTemp = 5.;
    for (int i = -delta; i <= delta; i++)
    {
        int j = 384 + i;
        if (isnan(msg->ranges.at(j)) || msg->ranges.at(j) < 0.1) continue;
        distFrontTemp = min(distFrontTemp,(double) msg->ranges.at(j));
    }
    distFront = distFrontTemp;

    // find minimum distance in front of the robot
    // double distRightTemp = 0;
    // for (int i = -delta; i <= delta; i++)
    // {
    //     int j = 128 + i;
    //     if (isnan(msg->ranges.at(j)) || msg->ranges.at(j) < 0.1) continue;
    //     distRightTemp += msg->ranges.at(j);
    // }
    // distRightTemp = distRightTemp / 2 * delta;
    // distRight = distRightTemp;

    diffE = (distMin - wallDistance) - e;
    e = distMin - wallDistance;

#ifdef DEBUG
    cout << "angleMin: " << TO_DEGREE(angleMin);
    cout << " distMin: " << distMin;
    cout << " distFront: " << distFront;
    cout << " error: " << e;
    cout << " diffE: " << diffE << endl;
#endif

    geometry_msgs::Twist twist_msg;
    if (state == 0)
    {
        if (distFront > 0.5)
        {
            // drive forward
            twist_msg.linear.x = 5;
            twist_msg.angular.z = 0;
            pubMessage.publish(twist_msg);
            return;
        }
        cout << "state becoming one" << endl;
        state = 1;
    }
    else if (state == 2) return;
    else if (state == 3)
    {
        // drive forward until distFront == CONSTANT
        if (distFront > 2.65)
        {
            // drive forward
            twist_msg.linear.x = 5;
            twist_msg.angular.z = 0;
            pubMessage.publish(twist_msg);
            cout << "After rotation, distFront is " << distFront << endl;
        }
        else
        {
            cout << "Arrived at destination" << endl;
            // stop driving
            twist_msg.linear.x = 0;
            twist_msg.angular.z = 0;
            pubMessage.publish(twist_msg);
            state = 4;
        }
        return;
    }
    else if (state == 4)
    {
        // stop driving
        twist_msg.linear.x = 0;
        twist_msg.angular.z = 0;
        pubMessage.publish(twist_msg);
        return;
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
