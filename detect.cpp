/**
 * Detect.cpp
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

#include "opencv2/opencv.hpp"
//#include "opencv2/gpu/gpu.hpp"
//#include "thrust/device_vector.h"
//#include "thrust/host_vector.h"

#include "polifitgsl.h"

using namespace cv;

/**
 * Class implements a lane
 */
class Lane
{
public:
	int degree; //degree of polynomial that defines lanes
	vector<double> l_params; //array of size degree. Defines coefficients for left lane curve.
	vector<double> r_params; //array of size degree. Defines coefficients for right lane curve.
	double filter; //filter for curve to remove jitter. lane = old_lane*filter + new_lane*(1-filter).
	
	/**
	 * Only provided constructor for Lane.
	 * @param degree Degree of polynomial that defines lanes.
	 * @param filter Filter for curve to remove jitter.
	 */ 
	Lane(int degree, double filter = 0.5) 
		: degree(degree)
		, l_params(vector<double>(degree, (double)nan("1")))
		, r_params(vector<double>(degree, (double)nan("1")))
		, filter(filter)
	{ 
		if (this->filter < 0.0 || this->filter > 1.0) this->filter = 0.9;
	}
	
	/**
	 * Updates left and right lane polynomial coefficients.
	 * @param l_new Array of size degree. Defines coefficients for left lane curve.
	 * @param r_new Array of size degree. Defines coefficients for right lane curve.
	 */
	void update(vector<double> l_new, vector<double> r_new)
	{
		if (l_params[0] != l_params[0])
		{
			l_params = l_new;
			r_params = r_new;
		}
		else
		{
			for (int i = 0; i < degree; i++)
			{
				l_params[i] = filter * l_params[i] + (1 - filter) * l_new[i];
				r_params[i] = filter * r_params[i] + (1 - filter) * r_new[i];
			}
		}
	}
};

/**
 * Defines configuration parameter provided by config.txt
 */
struct Config
{
	string video_file; // video file to read
	int lane_degree; // degree of polynomial that defines lanes
	double lane_filter; // filter for curve to remove jitter. lane = old_lane*filter + new_lane*(1-filter).
	int lane_start_threshold; // +/- pixel threshold to look for lane. 
	int left_lane_start; // percentage of width to start looking for left lane
	int right_lane_start; // percentage of width to start looking for right lane
	int row_step; // stride for stepping through image rows
	int col_step; // stride for stepping through image columns
};

//Configuration
Config config;

/**
 * Reads configuration file and sets configuration parameters
 * @return Data structure of configuration parameters
 */
Config getConfig()
{
	Config config;
	
	ifstream ifs("config.txt");
	istringstream is_file(string((std::istreambuf_iterator<char>(ifs)),
                 std::istreambuf_iterator<char>()));

	string line;
	while( getline(is_file, line) )
	{
	  istringstream is_line(line);
	  string key;
	  if( getline(is_line, key, '=') )
	  {
		string value;
		if( getline(is_line, value) ) 
		{
			if (key =="video_file") config.video_file = string(value); 
			else if (key == "lane_degree") config.lane_degree = stoi(value);
			else if (key == "lane_filter") config.lane_filter = stod(value);
			else if (key == "lane_start_threshold") config.lane_start_threshold = stoi(value);
			else if (key == "left_lane_start") config.left_lane_start = stoi(value);
			else if (key == "right_lane_start") config.right_lane_start = stoi(value);
			else if (key == "row_step") config.row_step = stoi(value);
			else if (key == "col_step") config.col_step = stoi(value);
		}
	  }
	}
	return config;
}

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

/**
 * Get lanes
 * @param img processed (thresholded and warped to birdseye perpective) frame from video
 * @param lane Detected lane
 */
void getLanes(const Mat &img, Lane &lane)
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
	
	vector<double> l_new(lane.degree, 0.0);
	vector<double> r_new(lane.degree, 0.0);
	polynomialfit(lx.size(), lane.degree, &ly[0], &lx[0], &l_new[0]);
	polynomialfit(rx.size(), lane.degree, &ry[0], &rx[0], &r_new[0]);
	
	lane.update(l_new, r_new);
}

/**
 * Draws lane on an image
 * @param img Image on which to draw lane
 * @param lane Lane to draw
 * @param m perspective transform matrix for lane
 */
void drawLane(Mat &img, const Lane &lane, Mat &m)
{
	Mat blank(img.size(), img.type(), Scalar(0, 0, 0));
	for (int i = 0; i < img.rows; i++)
	{
		circle(blank, Point(polynomial(&(lane.l_params)[0], lane.degree, i), i), 3, Scalar(150, 0, 0), 3);
		circle(blank, Point(polynomial(&(lane.r_params)[0], lane.degree, i), i), 3, Scalar(150, 0, 0), 3); 
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

int main(int argc, char* argv[])
{
	config = getConfig();
    Lane lane(config.lane_degree, config.lane_filter);
		
	VideoCapture cap(config.video_file); 
    if(!cap.isOpened()) return -1;
    
    //namedWindow("output", 1);
    Mat frame;
    cap >> frame;
	Mat birdseye = getTransformMatrix(frame);
	Mat first_person = getTransformMatrix(frame, true);
    
    cv::Mat src;
	cv::Mat th;
	cv::Mat dst;
    while(true)
    {
		//get frame from stream
		cap >> frame;
		
		//copy of original frame for drawing
		Mat original = frame.clone();
		
        //TODO PROBABLY DON'T NEED THESE EXTRA MATS SINCE NO LONGER USING GPU.UPLOAD
		//preprocess img
        frame.copyTo(src);
		//src.upload(frame);
		thresh(src, th);
		cv::warpPerspective(th, dst, birdseye, Size(frame.cols, frame.rows));
        dst.copyTo(frame);
		//dst.download(frame);
	
		//Get lanes
		//getLanes(frame, lane);
		
		//draw lanes
		//drawLane(original, lane, first_person);
		
		//-------------------------------------------------------//
		//imshow("output", original);
		if(waitKey(1) >= 0) break;
	}
}
