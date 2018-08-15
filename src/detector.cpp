using namespace std;

#include "detector.h"
#include "polifitgsl.h"
#include "logger.h"
#include "helpers.h"

#include <libconfig.h++>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace cv;

//-----CLASS METHOD DECLARATIONS-----//

int polynomial(std::vector<double> params, double x);
void thresh(cv::Mat &src, cv::Mat &dst, int threshold);

//-----CLASS METHODS-----//

Detector::Detector(string config_path, int cam_height, int cam_width)
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

        img_threshold = cfg.lookup("camera.threshold");

        double cam_angle = cfg.lookup("camera.angle");
        double frame_floor = cfg.lookup("camera.frame.floor");
        double frame_ceiling = cfg.lookup("camera.frame.ceiling");
       
        frame_height = cam_height; 
        frame_width = cam_width;        
        m_per_px = (double)cfg.lookup("camera.range") / frame_height;

        
        matrix_transform_birdseye = getTransformMatrix(cam_height, cam_width, cam_angle, frame_floor, frame_ceiling);
        matrix_transform_fiperson = getTransformMatrix(cam_height, cam_width, cam_angle, frame_floor, frame_ceiling, true);
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
    
    thresh(frame, th, img_threshold);
    cv::warpPerspective(th, dst, matrix_transform_birdseye, Size(img.cols, img.rows));

    imshow("birds", dst);
    
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
        circle(blank, Point(polynomial(lane.getLParams(), i), i), 3, Scalar(150, 0, 0), 3);
        circle(blank, Point(polynomial(lane.getRParams(), i), i), 3, Scalar(150, 0, 0), 3);
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
Mat Detector::getTransformMatrix(int height, int width, double angle, double perc_low, double perc_high, bool undo)
{
    int low = (int)(perc_low * height);
    int high = (int)(perc_high * height);
    int left = (int)(high / tan(angle));
    int right = (int)(width / 2.0 + width / 2.0 - left);

    std::vector<Point2f> src = {Point2f(0, high), Point2f(width, high), Point2f(width, low), Point2f(0, low)};
    std::vector<Point2f> dst = {Point2f(0, 0), Point2f(width, 0), Point2f(right, height), Point2f(left, height)};
   
    // std::vector<Point2f> src = {Point2f(width*0.44,height*0.20), Point2f(width*0.56,height*0.20), Point2f(width*1.00,height*0.85), Point2f(width*0.00,height*0.85)};
    // std::vector<Point2f> dst = {Point2f(width*0.20,height*0.00), Point2f(width*0.80,height*0.00), Point2f(width*0.80,height*1.00), Point2f(width*0.20,height*1.00)};
     
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
void thresh(cv::Mat &src, cv::Mat &dst, int threshold)
{
    cv::cvtColor(src, dst, CV_BGR2GRAY);
    
    cv::GaussianBlur(dst, dst, Size( 7, 7 ), 1.5, 1.5 );
    cv::threshold(dst, dst, threshold, 255, THRESH_BINARY);
}

/**
 * Evaluates a polynomial expression
 * @param params Array of polynomial coefficients
 * @param degree Degree of polynomial (size of params)
 * @param x Polynomial input
 * @return Evaluated expression.
 */
int polynomial(std::vector<double> params, double x)
{
    double val = 0;
    for (int i = 0; i < params.size(); i++)
    {
        val += params[i] * pow(x, i);
    }
    return (int)val;
}	

/**
 * Evaluates the derivative of a second degree polynomial 
 * @param params vector of polynomial's coefficients, from lowest degree to highest
 * @return vector of derivative polynomial coefficients
 */
std::vector<double> derivative(std::vector<double> params)
{
    assert(params.size() == 3);
    std::vector<double> deriv(params.size()-1);
    for (int i = 1; i < params.size(); i++)
    {
        deriv.push_back(params[i] * (double)i);
    }

    return deriv;
}



/**
 * Calculates the turning radius of the vehicle
 * @return Turning radius of vehcile
 */
double Detector::getTurningRadius(Lane &lane)
{  
    auto params = lane.getParams();         // vector
    double y_pos = (double)frame_height;
    double x_pos = polynomial(params, y_pos);
    double m_pos = M_PI/2 + 0.001;          // 0.001 to avoid division by 0
    double b_pos = m_pos*x_pos - y_pos;

    double y_des = (double)frame_height * 0.25;
    double x_des = polynomial(params, y_des);
    double m_des = polynomial(derivative(params), y_des);
    double b_des = m_des * x_des - y_des;

    double radius = 0;
    if (m_pos != m_des)
    {
        double x_center = (m_des * m_pos) * (b_des - b_pos) / (m_pos - m_des);
        double y_center = (-1 / m_pos) * x_center + b_pos;

        radius = sqrt(pow(x_pos-x_center, 2) + pow(y_pos-y_center, 2));

        // Negative turning radius is for left turn
        if (x_center < x_pos) 
            radius *= -1;

        radius *= this->m_per_px;
    }

    std::ostringstream oss;
    oss << "radius: " << radius;
    Logger::getLogger().log("lane", oss.str(), Levels::INFO, {});
    return radius;
}

// /**
//  * Calculate the ackermann steering angle for vehcile
//  * @returns two-element vector (left and right wheel) of steering angles
//  */
// std::vector<double> Detector::AckermannSteering()
// {
//     double radius = this->getTurningRadius();

//     std::vector<double> steering_angle{0, 0};
//     if (radius != 0)
//     {
//         steering_angle[0] = atan2(vehicle_length, radius + (vehicle_width/2));
//         steering_angle[1] = atan2(vehicle_length, radius + (vehicle_width/2));
//     }

//     return steering_angle;
// }

// /**
//  * Calculate the differential steering velocities for vehcile
//  * @returns two-element vector (left and right wheel) of steering velocities 
//  */
// std::vector<double> Detector::DifferentialSteering(double speed)
// {
//     double radius = this->getTurningRadius();

//     std::vector<double> differential{0, 0};
//     if (radius != 0)
//     {
//         double temp = vehicle_width / (2 * radius);
//         differential.at(0) = speed * (1 + temp);
//         differential.at(1) = speed * (1 - temp);
//     }

//     return differential;
// }
