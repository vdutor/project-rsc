#include <pthread.h>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

ros::Publisher pilotCommandMsg;
ros::Subscriber pilotResponseMsg;

void response_cb(const std_msgs::String::ConstPtr& msg)
{
    cout << endl << "Response from robot: " << msg->data << endl;
}

void *cmdThread(void *arg)
{
    ROS_INFO("command thread running");

    while(true)
    {
        std_msgs::String msg;
        msg.data = "en";
        pilotCommandMsg.publish(msg);
        msg.data = "1v-300";
        pilotCommandMsg.publish(msg);
        msg.data = "2v300";
        pilotCommandMsg.publish(msg);

        sleep(10);

        msg.data = "1v0";
        pilotCommandMsg.publish(msg);
        msg.data = "2v0";
        pilotCommandMsg.publish(msg);
    }
    return NULL;
}

int main(int argc, char* argv[])
{
    pthread_t cmdThrID;

    ros::init(argc, argv, "pilot_node");
    ROS_INFO("pilot starting");

    ros::NodeHandle nh;
    pilotResponseMsg = nh.subscribe("uc0Response", 100, response_cb);
    pilotCommandMsg = nh.advertise<std_msgs::String>("uc0Command",100);

    int err = pthread_create(&cmdThrID, NULL, cmdThread, NULL);
    if (err != 0) {
        ROS_ERROR("unable to create command thread");
        return 1;
    }

    ros::spin();

    ROS_INFO("pilot stopping");

    return 0;
}
