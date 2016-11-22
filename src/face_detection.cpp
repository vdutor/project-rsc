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
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

using namespace cv;
using namespace std;

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

int ctr = 0;
int im_width; 
int im_height;
CascadeClassifier haar_cascade;
//Ptr<cv::face::FaceRecognizer> model = cv::face::createFisherFaceRecognizer();
Ptr<FaceRecognizer> model = createFisherFaceRecognizer();


void read_csv(vector<Mat>& images, vector<int>& labels, char separator = ';')
{
    string filename = "../res/faces.csv";
    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file)
    {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, path, classlabel;
    while (getline(file, line))
    {
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty())
        {
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
}

void init_model()
{
    vector<Mat> images;
    vector<int> labels;
    read_csv(images, labels);
    model->train(images, labels);
}

void recognize_face(Mat original, Mat gray, Rect face_contour)
{
    //Crop the face from the image. 
    Mat face = gray(face_contour);
    Mat face_resized;
    cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
    // Now perform the prediction
    int prediction = model->predict(face_resized);
    // Create the text we will annotate the box with:
    string box_text = format("Prediction = %d", prediction);
    // Calculate the position for annotated text
    int pos_x = std::max(face_contour.tl().x - 10, 0);
    int pos_y = std::max(face_contour.tl().y - 10, 0);
    putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
}

void detect_faces(Mat frame)
{
    im_width = frame.cols;
    im_height = frame.rows;
    Mat original = frame.clone();
    Mat gray;
    cvtColor(original, gray, CV_BGR2GRAY);
    vector< Rect_<int> > faces;
    haar_cascade.detectMultiScale(gray, faces);
    for(int i = 0; i < faces.size(); i++)
    {
        recognize_face(original, gray, faces[i]);
        rectangle(original, faces[i], CV_RGB(0, 255,0), 1);
    }
    imshow("face_recognizer", original);
    waitKey(1);
}


void imgage_rgb_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    try 
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);
        if(ctr++ % 100 == 1) 
            detect_faces(cv_ptr->image);
        //cv::imshow("foo", cv_ptr->image);
        //cv::waitKey(1);  // Update screen
    } catch (const cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "foo");

    init_model();
    haar_cascade.load("/usr/local/src/opencv-3.1.0/data/haarcascades/haarcascade_frontalface_default.xml");
    VideoCapture cap(0);
    Mat frame;
    while(1)
    {
       cap >> frame;
       detect_faces(frame);
    }

    //ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("camera/rgb/image_raw", MY_ROS_QUEUE_SIZE, imgcb);
    //ros::Subscriber sub = nh.subscribe("camera/rgb/image_color", MY_ROS_QUEUE_SIZE, imgage_rgb_cb);

    //cv::namedWindow("foo");
    //ros::spin();
    //Mat frame;
    return 0;
}
