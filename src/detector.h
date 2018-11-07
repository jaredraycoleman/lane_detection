#ifndef DETECTOR_H
#define DETECTOR_H

using namespace std;

#include "opencv2/opencv.hpp"
#include "lane.h"
#include "helpers.h"

#include <string>
#include <cmath>
#include <vector>
#include <functional>
#include <thread>

class Detector
{
private:
    int threshold;
    int row_step;
    int col_step;
    int l_start;
    int r_start;
    int img_threshold;
    cv::Mat matrix_transform_birdseye;
    cv::Mat matrix_transform_fiperson;
   
    double vehicle_length;
    double vehicle_width;

    double cam_angle;
    int frame_width;
    int frame_height;
    double m_per_px;

    cv::VideoCapture cap;
    Lane lane;
    
    cv::Mat getTransformMatrix(int height, int width, double angle, double perc_low, double perc_high, bool undo=false);

    std::thread *detect_thread;
    std::function<void(const Lane &lane)> callback;

    void detect(std::function<cv::Mat()> get_frame, double freq_hz);

public:
    Detector(string config_path, std::function<cv::Mat()> get_frame, double freq_hz, std::function<void(const Lane &lane)> callback);
    virtual ~Detector();
    void update(const cv::Mat &img);
    void drawLane(cv::Mat &img);

    double getOffset();


    // double getTurningRadius();
    // std::vector<double> getDesiredConfiguration();
    // std::vector<double> getAckermannSteering();
    // std::vector<double> getDifferentialSteering(double speed);
};

#endif
