#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <project_rsc/stop.h>

#include "global.h"

int running;

void pilotCB(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("received response from pilot");
    running = 0;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mockdriver_node");
    ROS_INFO("mock driver starting");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(PILOT_RSP, 100, pilotCB);
    ros::Publisher stopPub = nh.advertise<project_rsc::stop>(STOP_CMD,100);

    project_rsc::stop stop_msg;
    stop_msg.foo = 1;
    stopPub.publish(stop_msg);
    ros::spinOnce();
    sleep (5);
    stopPub.publish(stop_msg);
    ros::spinOnce();
    sleep (5);

    return 0;
}
