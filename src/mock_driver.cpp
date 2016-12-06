#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <project_rsc/move.h>
#include <project_rsc/rotate.h>

#include "global.h"

int running;

void pilotCB(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("received response from pilot");

    if (msg.data == ROBOT_DONE)
        running = 0;
    else
        running = 1;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mockdriver_node");
    ROS_INFO("mock driver starting");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(PILOT_RSP, 100, pilotCB);
    ros::Publisher mvPub = nh.advertise<project_rsc::move>(MOVE_CMD,100);
    ros::Publisher rotPub = nh.advertise<project_rsc::rotate>(ROTATE_CMD,100);

    for (int i = 0; i < 10; i++)
    {
        project_rsc::move mv_msg;
        mv_msg.direction = 1;
        mv_msg.length = 10 + i;

        project_rsc::rotate rot_msg;
        rot_msg.direction = 1;
        rot_msg.degrees = 3.14 + i;

        if (i%2 == 0)
            mvPub.publish(mv_msg);
        else
            rotPub.publish(rot_msg);

        ros::spinOnce();
        sleep(2);
    }

    return 0;
}
