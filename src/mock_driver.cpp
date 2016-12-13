#include <math.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <project_rsc/move.h>
#include <project_rsc/rotate.h>

#include "global.h"

int running = 0;

void pilotCB(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("received response from pilot");

    if (msg->data == ROBOT_DONE)
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

    tf::TransformListener tfListener;

    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            tfListener.waitForTransform("odom", "base_link",
                                        ros::Time(0), ros::Duration(10.0));
            tfListener.lookupTransform("odom", "base_link",
                                        ros::Time(0), transform);
            tf::Vector3 org = transform.getOrigin();
            tf::Quaternion rot = transform.getRotation();
            ROS_INFO("x %f, y %f, theta %f", org.getX(), org.getY(), rot.getAngle());
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        project_rsc::move mv_msg;
        mv_msg.direction = 1;
        mv_msg.length = 10;
        mvPub.publish(mv_msg);
        ros::spinOnce();
        running = 1;

        while(running == 1)
            sleep (2);
    }

    // for (int i = 0; i < 1; i++)
    // {
    //     project_rsc::rotate rot_msg;
    //     rot_msg.direction = 1;
    //     rot_msg.degrees = 2*M_PI;
    //     rotPub.publish(rot_msg);
    //     ros::spinOnce();
    //     sleep (8);

    //     project_rsc::move mv_msg;
    //     mv_msg.direction = 1;
    //     mv_msg.length = 18;
    //     mvPub.publish(mv_msg);
    //     ros::spinOnce();
    //     sleep (5);
    // }

    return 0;
}
