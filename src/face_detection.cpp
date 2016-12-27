#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "face_detection_manager.hpp"

using namespace std;

// main

FaceDetectionManager* detectionManager;

void image_rgb_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);
        detectionManager->detect_faces(cv_ptr->image);
    } catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}

int main(int argc, char* argv[])
{
    if (argc <= 1) {
        cout << "usage: " << argv[0] << " </path/to/resource_folder>" << endl;
        exit(1);
    }
    string res_path = string(argv[1]);
    string pictures = "dataset2/";
    string recordings = "recordings/";

    detectionManager = new FaceDetectionManager(res_path, pictures, recordings);

    ros::init(argc, argv, "faceRecognition");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/rgb/image_color", 100, image_rgb_cb);

    ros::spin();

    return 0;
}


