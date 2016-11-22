#include <iostream>
#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// both ways of including work on vincent's machine
//#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include "opencv2/core/core.hpp"
//#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

using namespace cv;
using namespace std;

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

int ctr = 0;
CascadeClassifier haar_cascade;

void detect_faces(Mat frame)
{
    int im_width = frame.cols;
    int im_height = frame.rows;
    // Clone the current frame:
    Mat original = frame.clone();
    // Convert the current frame to grayscale:
    Mat gray;
    cvtColor(original, gray, CV_BGR2GRAY);
    // Find the faces in the frame:
    vector< Rect_<int> > faces;
    haar_cascade.detectMultiScale(gray, faces);
    // At this point you have the position of the faces in
    // faces. Now we'll get the faces, make a prediction and
    // annotate it in the video. Cool or what?
    for(int i = 0; i < faces.size(); i++) {
        // Process face by face:
        Rect face_i = faces[i];
        // Crop the face from the image. So simple with OpenCV C++:
        //Mat face = gray(face_i);
        //Mat face_resized;
        //cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
        // Now perform the prediction, see how easy that is:
        //int prediction = model->predict(face_resized);
        // And finally write all we've found out to the original image!
        // First of all draw a green rectangle around the detected face:
        rectangle(original, face_i, CV_RGB(0, 255,0), 1);
        // Create the text we will annotate the box with:
        //string box_text = format("Prediction = %d", prediction);
        // Calculate the position for annotated text (make sure we don't
        // put illegal values in there):
        //int pos_x = std::max(face_i.tl().x - 10, 0);
        //int pos_y = std::max(face_i.tl().y - 10, 0);
        // And now put it into the image:
        //putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
    }
    // Show the result:
    imshow("face_recognizer", original);
    // And display it:
    char key = (char) waitKey(20);
}


void imgage_rgb_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);
        if(ctr++ % 100 == 1) 
            detect_faces(cv_ptr->image);
        //cv::imshow("foo", cv_ptr->image);
        //cv::waitKey(1);  // Update screen
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "foo");

    haar_cascade.load("/usr/local/src/opencv-3.1.0/data/haarcascades/haarcascade_frontalface_default.xml");

    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("camera/rgb/image_raw", MY_ROS_QUEUE_SIZE, imgcb);
    ros::Subscriber sub = nh.subscribe("camera/rgb/image_color", MY_ROS_QUEUE_SIZE, imgage_rgb_cb);

    cv::namedWindow("foo");
    ros::spin();
    Mat frame;
    return 0;
}
