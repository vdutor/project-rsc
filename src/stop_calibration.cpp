#include <unistd.h>
#include <cmath>
#include <ctime>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <people_msgs/PositionMeasurement.h>

#include "global.h"

#define TWIST_PUB_TOPIC "twist"
#define LASER_SCAN_SUB_TOPIC "scan"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tracking");
    ROS_INFO("leg tracker starting");

    ros::NodeHandle nh;

    ros::Publisher twistPub = nh.advertise<geometry_msgs::Twist>(TWIST_PUB_TOPIC, 100);
    ros::Rate poll_rate(100);
    while(twistPub.getNumSubscribers() == 0)
        poll_rate.sleep();

    geometry_msgs::Twist twist_msg;
    ros::Rate r(10); // 10 hz
    int counter = 28;
    while (ros::ok() && counter > 0)
    {
        twist_msg.angular.z = -30;
        twist_msg.linear.x = 0;
        twistPub.publish(twist_msg);
        ros::spinOnce();
        r.sleep();
        counter--;
    }
    twist_msg.angular.z = 0;
    twist_msg.linear.x = 0;
    twistPub.publish(twist_msg);
    ros::spinOnce();
    ros::Duration(2).sleep();

    ROS_INFO("leg tracker stopping");

    return 0;
}
