using namespace std;

#include "detector.h"
#include "polifitgsl.h"

#include <libconfig.h++>

using namespace cv;

//-----CLASS METHOD DECLARATIONS-----//

int polynomial(const double *params, int degree, double x);
void thresh(cv::Mat &src, cv::Mat &dst);

//-----CLASS METHODS-----//

Detector::Detector(string config_path)
{
    libconfig::Config cfg;
    try
    {
        cfg.readFile(config_path.c_str());
        threshold = cfg.lookup("detector.threshold");
        row_step = cfg.lookup("detector.row_step");
        col_step = cfg.lookup("detector.col_step");
        l_start = cfg.lookup("detector.start.left");
        r_start = cfg.lookup("detector.start.right");

        double cam_angle = cfg.lookup("camera.angle");
        int frame_width = cfg.lookup("camera.frame.width");
        int frame_height = cfg.lookup("camera.frame.height");
        int frame_floor = cfg.lookup("camera.frame.floor");
        int frame_ceiling = cfg.lookup("camera.frame.ceiling");

        string path = cfg.lookup("video.file");
        VideoCapture cap(path);
        Mat frame;
        cap >> frame;
        
        matrix_transform_birdseye = getTransformMatrix(cam_angle, frame_width, frame_height, frame_floor, frame_ceiling);
        matrix_transform_fiperson = getTransformMatrix(cam_angle, frame_width, frame_height, frame_floor, frame_ceiling, true);
        cap.release();
    }
    catch(...)
    {
        cerr << "Invalid config file" << endl;
    }
}

/**
 * Get lanes
 * @param img processed (thresholded and warped to birdseye perpective) frame from video
 * @param lane Detected lane
 */
void Detector::getLanes(const Mat &img, Lane &lane)
{                
    Mat th;
    Mat dst;
    Mat frame = img.clone();
    
    thresh(frame, th);
    cv::warpPerspective(th, dst, matrix_transform_birdseye, Size(img.cols, img.rows));
    
    int width = dst.cols;
    int height = dst.rows;
    
    int left = width * l_start / 100;
    int right = width * r_start / 100;
    
    std::vector<double> lx;
    std::vector<double> rx;
    std::vector<double> ly;
    std::vector<double> ry;
    
    //Loop through frame rows at row_step
    for (int i = height-1; i >= 0; i-=row_step)
    {
        //Loop through left side
        lx.push_back(left);
        ly.push_back(i);
        for (int j = 0; j <= threshold; j+=col_step)
        {
            if (dst.at<uchar>(i, left+j) == 255) 
            {
                lx.back() = left+j;
                left += j;
                break;
            }
            
            if (dst.at<uchar>(i, left-j) == 255) 
            {
                lx.back() = left-j;
                left -= j;
                break;
            }
        }
        
        //Loop through right side
        rx.push_back(right);
        ry.push_back(i);
        for (int j = 0; j <= threshold; j+=col_step)
        {
            if (dst.at<uchar>(i, right-j) == 255) 
            {
                rx.back() = right-j;
                right -= j;
                break;
            }
            
            if (dst.at<uchar>(i, right+j) == 255) 
            {
                rx.back() = right+j;
                right += j;
                break;
            }
        }
         
    }
    
    std::vector<double> l_new(lane.getN(), 0.0);
    std::vector<double> r_new(lane.getN(), 0.0);
    polynomialfit(lx.size(), lane.getN(), &ly[0], &lx[0], &l_new[0]);
    polynomialfit(rx.size(), lane.getN(), &ry[0], &rx[0], &r_new[0]);
    
    lane.update(l_new, r_new);
}

/**
 * Draws lane on an image
 * @param img Image on which to draw lane
 * @param lane Lane to draw
 * @param m perspective transform matrix for lane
 */
void Detector::drawLane(Mat &img, Lane &lane)
{
    Mat blank(img.size(), img.type(), Scalar(0, 0, 0));
    for (int i = 0; i < img.rows; i++)
    {
        circle(blank, Point(polynomial(&(lane.getParams())[0], lane.getN(), i), i), 3, Scalar(150, 0, 0), 3);
    }
    warpPerspective(blank, blank, matrix_transform_fiperson, Size(img.cols, img.rows));
    for (int i = 0; i < img.rows; i+=2)
    {
        for (int j = 0; j < img.cols; j+=2)
        {
            if (blank.at<Vec3b>(i, j)[0] == 150)
                circle(img, Point(j, i), 1, Scalar(150, 0, 0), 1);
        }
    }
}



/**
 * Gets the perpective transform matrix for warping an image to birdseye perspective
 * @param img Image to generate matrix for
 * @param undo If undo is true, return the matrix for transforming from birdseye to first-person perspective
 * @return perspective transform matrix
 */
Mat Detector::getTransformMatrix(double angle, int width, int height, int floor, int ceiling, bool undo)
{
    int left = (int)(ceiling / tan(angle));
    int right = (int)(width / 2.0 + width / 2.0 - left);
    std::vector<Point2f> src = {Point2f(0.0, ceiling), Point2f(width, ceiling), Point2f(width, floor), Point2f(0.0, floor)};
    std::vector<Point2f> dst = {Point2f(0.0, 0.0), Point2f(width, 0.0), Point2f(right, height), Point2f(left, height)};
    
    Mat m;
    if (undo) m = getPerspectiveTransform(&dst[0], &src[0]);
    else m = getPerspectiveTransform(&src[0], &dst[0]);
    return m;
}


//-----NON CLASS METHODS-----//

/**
 * Thresholds the image. Uses GPU acceleration
 * Process:
 *   1. Convert image to grayscale
 *   2. Blur the image to remove noise (gaussian)
 *   3. threshold image (binary)
 * @param src image to threshold (GpuMat)
 * @param dst destination for thresholded image (GpuMat)
 */
void thresh(cv::Mat &src, cv::Mat &dst)
{
    cv::cvtColor(src, dst, CV_BGR2GRAY);
    
    cv::GaussianBlur(dst, dst, Size( 7, 7 ), 1.5, 1.5 );
    cv::threshold(dst, dst, 185, 255, THRESH_BINARY);
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