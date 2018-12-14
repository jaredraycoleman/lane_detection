using namespace std;

#include "detector.h"
#include "polifitgsl.h"
#include "helpers.h"

#include <libconfig.h++>

#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdlib>
#include <queue>
#include <list>
#include <chrono>
#include <future>

using namespace cv;

using namespace std::literals::chrono_literals;
//-----CLASS METHOD DECLARATIONS-----//

double polynomial(std::vector<double> params, double x);
void thresh(const cv::Mat &src, cv::Mat &dst, int threshold);

//-----CLASS METHODS-----//

Detector::Detector(string config_path, std::function<cv::Mat()> get_frame)
    : get_frame(get_frame)
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
       
        vehicle_length = cfg.lookup("vehicle.length");
        vehicle_width = cfg.lookup("vehicle.width");
        
        cv::Mat frame = get_frame();
        frame_height = frame.rows; 
        frame_width = frame.cols; 
        out = cv::VideoWriter("output.avi", CV_FOURCC('M', 'P', 'E', 'G'), 2, Size(frame_width, frame_height));


        m_per_px = (double)cfg.lookup("camera.range") / frame_height;

        matrix_transform_birdseye = getTransformMatrix(frame_height, frame_width, cam_angle, frame_floor, frame_ceiling);
        matrix_transform_fiperson = getTransformMatrix(frame_height, frame_width, cam_angle, frame_floor, frame_ceiling, true); 
    
        lane = new Lane(config_path, 
                    {(double)frame_width * l_start / 100, 0, 0},  // initial lparams
                    {(double)frame_width * r_start / 100, 0, 0}); // initial rparams
    }
    catch(...)
    {
        cerr << "Invalid config file" << endl;
    }
}

Detector::~Detector()
{
    delete detect_thread;
    delete lane;
}

void Detector::start(double freq_hz, std::function<void(const Lane &lane)> callback)
{
    if (detect_thread == nullptr)
    {
        detect_thread = new std::thread(&Detector::detect, this, freq_hz, callback);
    }
}

void Detector::join()
{
    if (detect_thread != nullptr)
    {
        detect_thread->join();
    }
}

void Detector::detect(double freq_hz, std::function<void(const Lane &lane)> callback)
{
    auto dt = std::chrono::duration<double>(1.0/freq_hz);
    auto end = std::chrono::high_resolution_clock::now() + dt;

    while (true)
    {           
        update(get_frame());       
        callback(*lane);
        std::this_thread::sleep_until(end);
        end = std::chrono::high_resolution_clock::now() + dt;
    }

}

/**
 * Get lanes
 * @param img processed (thresholded and warped to birdseye perpective) frame from video
 * @param lane Detected lane
 */
void Detector::update(const cv::Mat &frame)
{          
    static cv::Mat th;
    static cv::Mat dst;
    static int width = frame_width;
    static int height = frame_height;
    static std::vector<double> lx;
    static std::vector<double> rx;
    static std::vector<double> ly;
    static std::vector<double> ry;
    static int degree = lane->getDegree();
    
    // Preprocesstry
    thresh(frame, th, img_threshold);
    cv::warpPerspective(th, dst, matrix_transform_birdseye, Size(width, height));

    lx.push_back(polynomial(lane->getLParams(), height));
    ly.push_back(height);
    rx.push_back(polynomial(lane->getRParams(), height));
    ry.push_back(height);

    // Loop through frame rows at row_step
    for (int i = height-1; i >= 0; i-=row_step)
    {
        int left = polynomial(lane->getLParams(), i); 
        int right = polynomial(lane->getRParams(), i);
        bool found_left = false;
        bool found_right = false;
        for (int j = 0; j <= threshold; j+=col_step)
        {
            if (found_left && found_right) 
            {
                break;
            } 

            if (!found_left && dst.at<uchar>(i, left+j) == 255) 
            {
                lx.push_back(left+j);
                ly.push_back(i);
                found_left = true;
            }
            
            if (!found_left && dst.at<uchar>(i, left-j) == 255) 
            {
                lx.push_back(left-j);
                ly.push_back(i);
                found_left = true;
            }

            if (!found_right && dst.at<uchar>(i, right-j) == 255) 
            {
                rx.push_back(right-j);
                ry.push_back(i);
                found_right = true;
            }
            
            if (!found_right && dst.at<uchar>(i, right+j) == 255) 
            {
                rx.push_back(right+j);
                ry.push_back(i);
                found_right = true;
            }
        }
         
    }

    if (lx.size() > 3 && rx.size() > 3)
    {
        std::vector<double> l_new(degree, 0.0);
        std::vector<double> r_new(degree, 0.0);
        polynomialfit(lx.size(), degree, &ly[0], &lx[0], &l_new[0]);
        polynomialfit(rx.size(), degree, &ry[0], &rx[0], &r_new[0]);
        
        lane->update(l_new, r_new);
        lx.clear();
        rx.clear();
        ly.clear();
        ry.clear();
    }    
}

/**
 * Draws lane on an image
 * @param img Image on which to draw lane
 * @param lane Lane to draw
 * @param m perspective transform matrix for lane
 */
const cv::Mat& Detector::drawLane() const
{
    static cv::Mat img;
    img = get_frame();  
    Mat blank(img.size(), img.type(), Scalar(0, 0, 0));
    auto lparams = lane->getLParams();
    auto rparams = lane->getRParams();
    for (int i = 0; i < img.rows; i++)
    {
        circle(blank, Point((int)polynomial(lparams, i), i), 3, Scalar(150, 0, 0), 3);
        circle(blank, Point((int)polynomial(rparams, i), i), 3, Scalar(150, 0, 0), 3);
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

    return img;
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

double Detector::getTurningRadius()
{
    static const double x1 = (double)frame_width / 2;
    static const double y1 = (double)frame_height;
    static const double y2 = (double)frame_height * 0.25;
    static const double y3 = (double)frame_height * 0.5;

    auto params = lane->getParams();
    double x2 = polynomial(params, y1);
    double x3 = polynomial(params, y2);

    double A = sqrt(pow(x2 - x3, 2) + pow(y2 - y3, 2));
    double B = sqrt(pow(x3 - x1, 2) + pow(y3 - y1, 2));
    double C = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));

    double alpha = acos((A * A - B * B - C * C) / (2 * B * C));

    double radius = A / (2 * sin(alpha));
    double d = (x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1);
    return d > 0 ? radius : -radius;
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
void thresh(const cv::Mat &src, cv::Mat &dst, int threshold)
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
double polynomial(std::vector<double> params, double x)
{
    double val = 0;
    for (uint i = 0; i < params.size(); i++)
    {
        val += params[i] * pow(x, i);
    }
    return val;
}	