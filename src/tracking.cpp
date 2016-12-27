#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <people_msgs/PositionMeasurement.h>

#include "global.h"

#define DEBUG

#define TAN15 0.268
#define TAN25 0.466
#define TWIST_PUB_TOPIC "twist"
#define LASER_SCAN_SUB_TOPIC "scan"
#define LEG_TRACK_SUB_TOPIC "people_tracker_measurements"
#define TRANS_MULTIPLIER 10
#define TRANS_MULTIPLIER_SLOW 4
#define ROT_MULTIPLIER 150
#define ANGLE_DELTA .001
#define X_DELTA .001

using namespace std;

ros::Publisher twistPub;
ros::Subscriber laserScanSub;
ros::Subscriber legTrackSub;

geometry_msgs::Point predictedPos;

bool stopRobot = false;

void laserScanCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int size = msg->ranges.size();
    double distFront = msg->ranges.at(size/2);

    // find minimum distance in front of the robot
    int delta = 20;
    for (int i = -delta; i <= delta; i++)
    {
        int j = size/2 + i;
        if (isnan(msg->ranges.at(j))) continue;
        distFront = min(distFront,(double) msg->ranges.at(j));
    }

#ifdef DEBUG
    cout << "distFront: " << distFront << endl;
#endif

    if (distFront < 0.5 && distFront > 0.05)
    {
        cout << "too close to nearest obstacle" << endl;
        stopRobot = true;
        // stop the robot
        geometry_msgs::Twist twist_msg;
        twist_msg.angular.z = 0;
        twist_msg.linear.x = 0;
        twistPub.publish(twist_msg);
        predictedPos.x = 0;
        predictedPos.y = 0;
    }
    else
    {
        stopRobot = false;
    }
}

void legTrackCB(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
    geometry_msgs::Point closestPair;
    geometry_msgs::Twist twist_msg;
    closestPair.x = 100;
    closestPair.y = 100;

    cout << "I have detected " << msg->people.size() << " people" << endl;
    for( auto it = msg->people.begin(); it != msg->people.end(); ++it )
    {
        geometry_msgs::Point pos = it->pos;
        if (abs(predictedPos.y - pos.y) < TAN15 * pos.x &&
            pos.x > 0 &&
            abs(predictedPos.x - pos.x) < abs(predictedPos.x - closestPair.x))
        {
            closestPair.x = pos.x;
            closestPair.y = pos.y;
        }
    }

    if (closestPair.x > 10)
    {
        cout << "too far from nearest person: " << closestPair.x << endl;
        // stop the robot
        twist_msg.angular.z = 0;
        twist_msg.linear.x = 0;
        twistPub.publish(twist_msg);
        predictedPos.x = 0;
        predictedPos.y = 0;
        return;
    }

    double angle = atan2(closestPair.y, closestPair.x);
    cout << "person detected at x: " << closestPair.x << " y: " << closestPair.y << " angle: " << angle << endl;
    // HOHO HAHA
    if (closestPair.x < 0.75)
    {
        twist_msg.linear.x = closestPair.x * TRANS_MULTIPLIER_SLOW;
        twist_msg.angular.z = - closestPair.y * ROT_MULTIPLIER;
    }
    else
    {
        twist_msg.linear.x = closestPair.x * TRANS_MULTIPLIER;
        twist_msg.angular.z = - closestPair.y * ROT_MULTIPLIER * 1.5;
    }
    if (stopRobot)
    {
        twist_msg.linear.x = 0;
    }
    twistPub.publish(twist_msg);

    predictedPos.x = closestPair.x - closestPair.x * X_DELTA;
    predictedPos.y = tan(angle - angle * ANGLE_DELTA) * predictedPos.x;
    cout << "I predict you will be at x: " << predictedPos.x << " y: " << predictedPos.y << endl;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tracking");
    ROS_INFO("leg tracker starting");

    ros::NodeHandle nh;

    predictedPos.x = 0;
    predictedPos.y = 0;

    twistPub = nh.advertise<geometry_msgs::Twist>(TWIST_PUB_TOPIC, 100);
    legTrackSub = nh.subscribe(LEG_TRACK_SUB_TOPIC, 100, legTrackCB);
    laserScanSub = nh.subscribe(LASER_SCAN_SUB_TOPIC, 100, laserScanCB);

    ros::spin();

    ROS_INFO("leg tracker stopping");

    return 0;
}
