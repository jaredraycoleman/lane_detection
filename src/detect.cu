/**
 * Detect.cu
 * Provides functions for lane detection
 * 
 * @author Oscar Morales Ponce
 * @author Jared Coleman
 * @version 1.0 12/02/17
 */

using namespace std;

#include <fstream>
#include <string>
#include <cmath>
#include <libconfig.h++>

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "thrust/device_vector.h"
#include "thrust/host_vector.h"

#include "polifitgsl.h"

using namespace cv;


/**
 * Gets the perpective transform matrix for warping an image to birdseye perspective
 * @param img Image to generate matrix for
 * @param undo If undo is true, return the matrix for transforming from birdseye to first-person perspective
 * @return perspective transform matrix
 */
Mat getTransformMatrix(Mat img, bool undo=false)
{
    int width = img.cols;
    int height = img.rows;
    vector<Point2f> src = {Point2f(width*0.44,height*0.20), Point2f(width*0.56,height*0.20), Point2f(width*1.00,height*0.85), Point2f(width*0.00,height*0.85)};
    vector<Point2f> dst = {Point2f(width*0.20,height*0.00), Point2f(width*0.80,height*0.00), Point2f(width*0.80,height*1.00), Point2f(width*0.20,height*1.00)};
    
    Mat m;
    if (undo) m = getPerspectiveTransform(&dst[0], &src[0]);
    else m = getPerspectiveTransform(&src[0], &dst[0]);
    return m;
}

/**
 * Thresholds the image. Uses GPU acceleration
 * Process:
 *   1. Convert image to grayscale
 *   2. Blur the image to remove noise (gaussian)
 *   3. threshold image (binary)
 * @param src image to threshold (GpuMat)
 * @param dst destination for thresholded image (GpuMat)
 */
void thresh(gpu::GpuMat &src, gpu::GpuMat &dst)
{
    gpu::cvtColor(src, dst, CV_BGR2GRAY);
    
    gpu::GaussianBlur(dst, dst, Size( 7, 7 ), 1.5, 1.5 );
    gpu::threshold(dst, dst, 185, 255, THRESH_BINARY);
}

/**
 * Evaluates a polynomial expression
 * @param params Array of polynomial coefficients
 * @param degree Degree of polynomial (size of params)
 * @param x Polynomial input
 * @return Evaluated expression.
 */
int polynomial(const double *params, int degree, double x)
{
    double val = 0;
    for (int i = 0; i < degree; i++)
    {
        val += params[i] * pow(x, i);
    }
    return (int)val;
}	

/**
 * Get lanes
 * @param img processed (thresholded and warped to birdseye perpective) frame from video
 * @param lane Detected lane
 */
void getLanes(const Mat &img, Lane *lane)
{
    static int row_step = config.row_step;
    static int col_step = config.col_step;
    static int d = config.lane_start_threshold;
    
    int width = img.cols;
    int height = img.rows;
    
    int left = width * config.left_lane_start / 100;
    int right = width * config.right_lane_start / 100;
    
    vector<double> lx;
    vector<double> rx;
    vector<double> ly;
    vector<double> ry;
    
    //Loop through frame rows at row_step
    for (int i = height-1; i >= 0; i-=row_step)
    {
        //Loop through left side
        lx.push_back(left);
        ly.push_back(i);
        for (int j = left + d; j >= left - d; j-=col_step)
        {
            if (img.at<uchar>(i, j) == 255) 
            {
                lx.back() = j;
                left = j;
                break;
            }
        }
        
        //Loop through right side
        rx.push_back(right);
        ry.push_back(i);
        for (int j = right - d; j < right + d; j+=col_step)
        {
            if (img.at<uchar>(i, j) == 255) 
            {
                rx.back() = j;
                right = j;
                break;
            }
        }
         
    }
    
    vector<double> l_new(lane->getN(), 0.0);
    vector<double> r_new(lane->getN(), 0.0);
    polynomialfit(lx.size(), lane->getN(), &ly[0], &lx[0], &l_new[0]);
    polynomialfit(rx.size(), lane->getN(), &ry[0], &rx[0], &r_new[0]);
    
    lane->update(l_new, r_new);
}

/**
 * Draws lane on an image
 * @param img Image on which to draw lane
 * @param lane Lane to draw
 * @param m perspective transform matrix for lane
 */
void drawLane(Mat &img, const Lane *lane, Mat &m)
{
    Mat blank(img.size(), img.type(), Scalar(0, 0, 0));
    for (int i = 0; i < img.rows; i++)
    {
        circle(blank, Point(polynomial(&(lane->params)[0], lane->getN(), i), i), 3, Scalar(150, 0, 0), 3);
    }
    warpPerspective(blank, blank, m, Size(img.cols, img.rows));
    for (int i = 0; i < img.rows; i+=2)
    {
        for (int j = 0; j < img.cols; j+=2)
        {
            if (blank.at<Vec3b>(i, j)[0] == 150)
                circle(img, Point(j, i), 1, Scalar(150, 0, 0), 1);
        }
    }
}
 
void sendMessage(SerialCommunication *serial, int8_t angle)
{
    UARTCommand command1, command2;
    command1.speed = 0;
    command1.wheelOrientation = angle;
    command1.maxTime = 100;
    command1.orientation= 0;
    serial->sendCommand(&command1);
 
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " <config file>" << endl;
        return 0;
    }
    
    Lane *lane;
    SerialCommunication *serial;
    VideoCapture *cap;
    Detector *detector;
    double *k;
    
    if (getConfig(argv[1], lane, serial, cap, detector, k))
    {
        cout << argv[1] << " is not a valid config file" << endl;
        return 0;
    }
    
    serial->run();
        
    if(!cap->isOpened()) return -1;
    
    //namedWindow("output", 1);
    Mat frame;
    *cap >> frame;
    Mat birdseye = getTransformMatrix(frame);
    Mat first_person = getTransformMatrix(frame, true);
    
    gpu::GpuMat src;
    gpu::GpuMat th;
    gpu::GpuMat dst;
    while(true)
    {
        //get frame from stream
        *cap >> frame;
        
        //copy of original frame for drawing
        Mat original = frame.clone();
        
        //preprocess img
        src.upload(frame);
        thresh(src, th);
        gpu::warpPerspective(th, dst, birdseye, Size(frame.cols, frame.rows));
        dst.download(frame);
    
        //Get lanes
        //getLanes(frame, lane);
        
        //draw lanes
        //drawLane(original, lane, first_person);
        
        //send lane
        sendMessage(serial, (int)(k * lane->curvature()));
        
        //-------------------------------------------------------//
        //imshow("output", original);
        if(waitKey(1) >= 0) break;
    }
}
