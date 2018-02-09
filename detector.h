#ifndef DETECTOR_H
#define DETECTOR_H

using namespace std;

#include "opencv2/opencv.hpp"
#include "lane.h"

#include <string>

using namespace cv;

class Detector
{
private:
    int threshold;
    int row_step;
    int col_step;
    int l_start;
    int r_start;

public:
    Detector(string config_path);
    void getLanes(const Mat &img, Lane &lane);
    void drawLane(Mat &img, Lane &lane, Mat &m);
    Mat getTransformMatrix(Mat img, bool undo=false);
    void thresh(cv::Mat &src, cv::Mat &dst);
};

#endif