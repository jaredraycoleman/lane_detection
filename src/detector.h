#ifndef DETECTOR_H
#define DETECTOR_H

using namespace std;

#include "opencv2/opencv.hpp"
#include "lane.h"

#include <string>
#include <cmath>

using namespace cv;

class Detector
{
private:
    int threshold;
    int row_step;
    int col_step;
    int l_start;
    int r_start;
    Mat matrix_transform_birdseye;
    Mat matrix_transform_fiperson;

    double cam_angle;
    int frame_width;
    int frame_height;
    int frame_floor;
    int frame_ceiling;
    
    Mat getTransformMatrix(double angle, int width, int height, int floor, int ceiling, bool undo=false);

public:
    Detector(string config_path);
    void getLanes(const Mat &img, Lane &lane);
    void drawLane(Mat &img, Lane &lane);
};

#endif