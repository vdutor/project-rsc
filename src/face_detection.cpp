#include <iostream>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include "opencv2/face.hpp"

using namespace cv::face;
using namespace cv;
using namespace std;

//#define DEBUG

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

int ctr = 0;
int im_width;
int im_height;
const std::string names[3] = {"vincent", "jonas","alexis"};
CascadeClassifier face_cascade;
CascadeClassifier eye_cascade;
Ptr<FaceRecognizer> model = createLBPHFaceRecognizer();
Ptr<FaceRecognizer> model2 = createEigenFaceRecognizer();
Ptr<FaceRecognizer> model3 = createFisherFaceRecognizer();


void read_csv(string res_path, string dataset, vector<Mat>& images, vector<int>& labels, char separator = ',')
{
    string filename = res_path + dataset + "faces_dataset2.csv";
    cout << filename << endl;

    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file)
    {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, im_name, classlabel;
    while (getline(file, line))
    {
        cout << line << endl;
        stringstream liness(line);
        getline(liness, im_name, separator);
        getline(liness, classlabel);
        if(!im_name.empty() && !classlabel.empty())
        {
            Mat image;
            int label = atoi(classlabel.c_str());
            cvtColor(imread(res_path + dataset + im_name,1),image,CV_BGR2GRAY);

#ifdef DEBUG
            cout << "label: " << label << endl;
            imshow("face_recognizer", image);
            waitKey(0);
#endif

            images.push_back(image);
            labels.push_back(label);
        }
    }
}

void init_model(string res_path, string dataset)
{
    vector<Mat> images;
    vector<int> labels;

    read_csv(res_path, dataset, images, labels);

    // TODO ideally the model is only trained once, and then loaded
    // this funcitonallity is available in opencv
    model->train(images, labels);
    //model2->train(images, labels);
    //model3->train(images, labels);
}

void recognize_face(Mat original, Mat gray, Rect face_contour)
{
    //Crop the face from the image.
    Mat face = gray(face_contour);
    Mat face_resized;
    cv::resize(face, face_resized, Size(200, 200), 1.0, 1.0, INTER_CUBIC);


    // Now perform the prediction
    int prediction = -1;
    int prediction2 = -1;
    int prediction3 = -1;
    double confidence = 0.0;
    model->predict(face_resized, prediction, confidence);
    //model2->predict(face_resized, prediction2, confidence);
    //model3->predict(face_resized, prediction3, confidence);

    //if (prediction3 == prediction2)
        //prediction = prediction2;

    // Create the text we will annotate the box with:
    std::string box_text = names[prediction];

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
    face_cascade.detectMultiScale(gray, faces);
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
        if(ctr++ % 3 == 0) detect_faces(cv_ptr->image);
        //else
        //{
            //cv::imshow("foo", cv_ptr->image);
            //cv::waitKey(1);  // Update screen
        //}
    } catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}
int main(int argc, char* argv[])
{
    cout << "argc: " << argc << endl;
    cout << argv[0] << endl;
    cout << argv[1] << endl;
    cout << argv[2] << endl;
    cout << argv[3] << endl;
    if (argc <= 1) {
        cout << "usage: " << argv[0] << " </path/to/resource_folder>" << endl;
        exit(1);
    }

    string res_fldr = string(argv[1]);
    string dataset = "dataset2/";

    ros::init(argc, argv, "faceRecognition");

    init_model(res_fldr, dataset);
    face_cascade.load(res_fldr + "haarcascade_frontalface_default.xml");
    eye_cascade.load(res_fldr + "haarcascade_eye.xml");

    cout << "created haarcascades" << endl;

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/rgb/image_color", MY_ROS_QUEUE_SIZE, imgage_rgb_cb);

    ros::spin();

    return 0;
}
