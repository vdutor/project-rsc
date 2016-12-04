#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    cout << endl << "Laser Message :" << endl;
    cout << "angle_min: " << msg->angle_min << endl;
    cout << "angle_max:  " << msg->angle_max << endl;
    cout << "angle_inc: " << msg->angle_increment << endl;
    cout << "scan_time: " << msg->scan_time << endl;
    cout << "time_inc: " << msg->time_increment << endl;
    cout << "range_min: " << msg->range_min << endl;
    cout << "range_max: " << msg->range_max << endl;

    vector<float> ranges = msg->ranges;

    cout << "ranges size: " << ranges.size() << endl;
    int center_idx = ranges.size() / 2;
    cout << "range[0]: " << ranges.at(center_idx) << endl;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "laser_node");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("scan", MY_ROS_QUEUE_SIZE, laser_cb);

    ros::spin();

    return 0;
}
