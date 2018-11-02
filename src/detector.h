#ifndef DETECTOR_H
#define DETECTOR_H

using namespace std;

#include "opencv2/opencv.hpp"
#include "lane.h"
#include "helpers.h"

#include <string>
#include <cmath>
#include <vector>

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
    
    cv::Mat getTransformMatrix(int height, int width, double angle, double perc_low, double perc_high, bool undo=false);

public:
    Detector(string config_path, int, int);
    void getLanes(const cv::Mat &img, Lane &lane);
    void drawLane(cv::Mat &img, Lane &lane);

    double getOffset(Lane &lane);


    double getTurningRadius(Lane &lane);
    std::vector<double> getDesiredConfiguration(Lane &lane);
    std::vector<double> getAckermannSteering(Lane &lane);
    std::vector<double> getDifferentialSteering(Lane &lane, double speed);
};

#endif
