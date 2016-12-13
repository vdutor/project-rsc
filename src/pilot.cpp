#include <pthread.h>
#include <iostream>
#include <string>

#include <std_msgs/String.h>
#include <project_rsc/move.h>
#include <project_rsc/rotate.h>
#include <project_rsc/stop.h>
#include "geometry_msgs/Twist.h"

#include "pilot.h"

using namespace std;

double defaultSpeed = 412; // 10 cm/s

ros::Subscriber twistCommandMsg;
ros::Publisher serialCommandMsg;
ros::Subscriber serialResponseMsg;
ros::Publisher pilotResponseMsg;
ros::Subscriber moveCommandMsg;
ros::Subscriber rotateCommandMsg;
ros::Subscriber stopCommandMsg;

Pilot* pilot;

Pilot::Pilot()
{
}

Pilot::Pilot(ros::Publisher serialCommandMsg,
             double speed,
             double translateTime,
             double rotationTime)
{
    this->serialPub = serialCommandMsg;
    this->speed = speed;
    this->translateTime = translateTime;
    this->rotationTime = rotationTime;
    ROS_INFO("rotational velocity of wheels set to %f", speed);
}

void Pilot::publishOdometry()
{
    odometry.broadcastPose(tfBroadcaster);
}

void serialResponseCB(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("response from robot %s", msg->data.c_str());
}

void twistCommandCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    cout << msg->angular.z << endl;
    cout << msg->linear.x << endl;

    int dx = msg->linear.x;
    int direction = 1; // forward
    if (dx < 0)
    {
        direction = -1;
        dx = -dx;
    }
}

void moveCommandCB(const project_rsc::move::ConstPtr& msg)
{
    ROS_INFO("received move command");

    int d = msg->direction;
    double l = msg->length;

    if (d != 1 && d != -1)
    {
        ROS_WARN("ERR: invalid direction argument");
        return;
    }
    pilot->move(d, l);

    std_msgs::String arrival_msg;
    arrival_msg.data = ROBOT_DONE;
    pilotResponseMsg.publish(arrival_msg);
}

void rotateCommandCB(const project_rsc::rotate::ConstPtr& msg)
{
    ROS_INFO("received rotate command");

    int d = msg->direction;
    double a = msg->degrees;

    if (d != 1 && d != -1)
    {
        ROS_WARN("ERR: invalid direction argument");
        return;
    }
    pilot->rotate(d, a);

    std_msgs::String arrival_msg;
    arrival_msg.data = ROBOT_DONE;
    pilotResponseMsg.publish(arrival_msg);
}

void stopCommandCB(const project_rsc::stop::ConstPtr& msg)
{
    ROS_INFO("received stop command");

    pilot->stopRobot();

    std_msgs::String arrival_msg;
    arrival_msg.data = ROBOT_DONE;
    pilotResponseMsg.publish(arrival_msg);
}

void *odomThread(void *arg)
{
    ROS_INFO("Odometry thread running");

    while(true)
    {
        pilot->publishOdometry();
        sleep(1);
    }
    return NULL;
}

int main(int argc, char* argv[])
{
    pthread_t odomThrID;

    ros::init(argc, argv, "pilot_node");
    ROS_INFO("pilot starting");

    ros::NodeHandle nh;

    serialResponseMsg = nh.subscribe(SERIAL_RSP, 100, serialResponseCB);
    serialCommandMsg = nh.advertise<std_msgs::String>(SERIAL_CMD,100);

    twistCommandMsg = nh.subscribe(TWIST_CMD, 100, twistCommandCB);
    //moveCommandMsg = nh.subscribe(MOVE_CMD, 100, moveCommandCB);
    //rotateCommandMsg = nh.subscribe(ROTATE_CMD, 100, rotateCommandCB);
    stopCommandMsg = nh.subscribe(STOP_CMD, 100, stopCommandCB);
    pilotResponseMsg = nh.advertise<std_msgs::String>(PILOT_RSP,100);

    pilot = new Pilot(serialCommandMsg);

    // int err = pthread_create(&odomThrID, NULL, odomThread, NULL);
    // if (err != 0) {
    //     ROS_ERROR("unable to create command thread");
    //     return 1;
    // }

    ros::spin();

    ROS_INFO("pilot stopping");

    delete pilot;
    return 0;
}
