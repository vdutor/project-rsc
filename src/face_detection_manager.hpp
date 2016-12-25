#ifndef FACE_DETECTION_MANAGER_H
#define FACE_DETECTION_MANAGER_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/face.hpp"

#define HISTORY_LENGTH 30
#define NUM_PEOPLE 3

class FaceDetectionManager
{
    public:
        FaceDetectionManager(std::string res_path, std::string pictures, std::string recordings);
        void detect_faces(cv::Mat frame);

    private:
        int frame_counter = 0;
        int history_itr = 0;
        int im_width;
        int im_height;
        const std::string names[NUM_PEOPLE] = {"vincent", "jonas", "alexis"};
        std::string res_path;
        std::string pictures;
        std::string recordings;
        cv::CascadeClassifier face_cascade;
        cv::CascadeClassifier eye_cascade;
        cv::Ptr<cv::face::FaceRecognizer> model = cv::face::createLBPHFaceRecognizer();
        bool detection_history[NUM_PEOPLE][HISTORY_LENGTH] = {{false}};
        bool in_frame[NUM_PEOPLE] = {false};

        void exec(const char* cmd);
        void read_csv(std::vector<cv::Mat>& images, std::vector<int>& labels, char separator = ',');
        void recognize_face(cv::Mat original, cv::Mat gray, cv::Rect face_contour);
        void print_detection_history();
        void analyse_detection_history();

};

#endif
