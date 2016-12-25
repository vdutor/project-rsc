#include <iostream>
#include <fstream>
#include <sstream>

#include "face_detection_manager.hpp"

using namespace cv::face;
using namespace cv;
using namespace std;

//#define SHOW_DATASET

FaceDetectionManager:: FaceDetectionManager(string res_path, string pictures, string recordings)
{
    this->res_path = res_path;
    this->pictures = pictures;
    this->recordings = recordings;

    // FaceRecognizer
    vector<Mat> images;
    vector<int> labels;
    read_csv(images, labels);
    model->train(images, labels);

    // Cascades
    string face_cascade_name = res_path + "haarcascade_frontalface_default.xml";
    string eyes_cascade_name = res_path + "haarcascade_eye.xml";

    if (!face_cascade.load(face_cascade_name))
    {
        cout << "Error loading face cascade" << endl;
    }
    if (!eye_cascade.load(eyes_cascade_name))
    {
        cout << "Error loading eye cascade" << endl;
    }
}

void FaceDetectionManager::read_csv(vector<Mat>& images, vector<int>& labels, char separator)
{
    string filename = res_path + pictures + "faces_dataset2.csv";

    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file)
    {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, im_name, classlabel;
    while (getline(file, line))
    {
        stringstream liness(line);
        getline(liness, im_name, separator);
        getline(liness, classlabel);
        if(!im_name.empty() && !classlabel.empty())
        {
            Mat image;
            int label = atoi(classlabel.c_str());
            cvtColor(imread(res_path + pictures + im_name, 1), image,CV_BGR2GRAY);

#ifdef SHOW_DATASET
            cout << line << endl;
            cout << "label: " << label << endl;
            imshow("face_recognizer", image);
            waitKey(0);
#endif

            images.push_back(image);
            labels.push_back(label);
        }
    }
}


void FaceDetectionManager::recognize_face(Mat original, Mat gray, Rect face_contour)
{
    //Crop the face from the image.
    Mat face = gray(face_contour);
    Mat face_resized;
    cv::resize(face, face_resized, Size(200, 200), 1.0, 1.0, INTER_CUBIC);


    // Now perform the prediction
    int prediction = -1;
    double confidence = 0.0;
    model->predict(face_resized, prediction, confidence);

    detection_history[prediction][history_itr] = true;

    // Create the text we will annotate the box with:
    std::string box_text = names[prediction];

    // Calculate the position for annotated text
    int pos_x = std::max(face_contour.tl().x - 10, 0);
    int pos_y = std::max(face_contour.tl().y - 10, 0);
    putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
}

void FaceDetectionManager::detect_faces(Mat frame)
{
    // only do detections for every third frame
    if (frame_counter++ % 3 != 0)
        return;

    // clear previous history entry
    for (int i = 0; i < NUM_PEOPLE; i++)
    {
        detection_history[i][history_itr] = false;
    }

    im_width = frame.cols;
    im_height = frame.rows;
    Mat original = frame.clone();
    Mat gray;
    cvtColor(original, gray, CV_BGR2GRAY);
    vector<Rect_<int>> faces;

    face_cascade.detectMultiScale(gray, faces);
    for(int i = 0; i < faces.size(); i++)
    {
        recognize_face(original, gray, faces[i]);
        rectangle(original, faces[i], CV_RGB(0, 255,0), 1);
    }

    imshow("face_recognizer", original);
    waitKey(1);

    history_itr++;
    if (history_itr >= HISTORY_LENGTH)
        history_itr = 0;

    analyse_detection_history();
}


void FaceDetectionManager::print_detection_history()
{
    for (int i = 0; i < NUM_PEOPLE; i++)
    {
        cout << endl << names[i] << endl;
        for (int j = 0; j < HISTORY_LENGTH; j++)
            cout << detection_history[i][j] << " ";
    }
}

void FaceDetectionManager::analyse_detection_history()
{
    for (int i = 0; i < NUM_PEOPLE; i++)
    {
        int sum = 0;
        for (int j = 0; j < HISTORY_LENGTH; j++)
        {
            if (detection_history[i][j])
                sum++;
        }

        if (sum >= 0.85 * HISTORY_LENGTH && !in_frame[i])
        {
            cout << "hello, " << names[i] << endl;
            in_frame[i] = true;
        }
        if (sum <= 0.15 * HISTORY_LENGTH && in_frame[i])
        {
            cout << "bye, " << names[i] << endl;
            in_frame[i] = false;
        }

    }
}


void FaceDetectionManager::exec(const char* cmd)
{
    //char buffer[128];
    //std::string result = "";
    //shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    //if (!pipe) throw std::runtime_error("popen() failed!");
    //while (!feof(pipe.get())) {
        //if (fgets(buffer, 128, pipe.get()) != NULL)
            //result += buffer;
    //}
}
