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
    cv::VideoWriter out;
   
    double vehicle_length;
    double vehicle_width;

    double cam_angle;
    int frame_width;
    int frame_height;
    double m_per_px;

    cv::VideoCapture cap;
    Lane *lane;

    std::function<cv::Mat()> get_frame;
    double freq_hz;
    
    cv::Mat getTransformMatrix(int height, int width, double angle, double perc_low, double perc_high, bool undo=false);

    std::thread *detect_thread = nullptr;

    void detect(double freq_hz, std::function<void(const Lane &lane)> callback);
    void update(const cv::Mat &img);

public:
    Detector(string config_path, std::function<cv::Mat()> get_frame);
    virtual ~Detector();
    const cv::Mat&  drawLane() const;

    void start(double freq_hz, std::function<void(const Lane &lane)> callback);
    void join();

    double getTurningRadius();

};

#endif
