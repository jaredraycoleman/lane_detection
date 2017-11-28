using namespace std;

#include <fstream>
#include <string>

#include "opencv2/opencv.hpp"
//#include "opencv2/core/cuda.hpp"

#include "polifitgsl.h"

using namespace cv;

struct Lane
{
	Mat left;
	Mat right;
};

struct Config
{
	string video_file;
	int lane_start_threshold;
	int left_lane_start;
	int right_lane_start;
	int row_step;
	int col_step;
};

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

void thresh(Mat &img)
{
	cvtColor(img, img, CV_BGR2GRAY);
	GaussianBlur( img, img, Size( 7, 7 ), 1.5, 1.5 );
	threshold(img, img, 185, 255, THRESH_BINARY);
}

void birdseye(Mat &img)
{
	int width = img.cols;
	int height = img.rows;
	Point2f src[] = {Point2f(width*0.44,height*0.20), Point2f(width*0.56,height*0.20), Point2f(width*1.00,height*0.85), Point2f(width*0.00,height*0.85)};
	Point2f dst[] = {Point2f(width*0.20,height*0.00), Point2f(width*0.80,height*0.00), Point2f(width*0.80,height*1.00), Point2f(width*0.20,height*1.00)};
	Mat m = getPerspectiveTransform(src, dst);
	warpPerspective(img, img, m, Size(width, height));
}	

void first_person(Mat &img)
{
	int width = img.cols;
	int height = img.rows;
	Point2f dst[] = {Point2f(width*0.44,height*0.20), Point2f(width*0.56,height*0.20), Point2f(width*1.00,height*0.85), Point2f(width*0.00,height*0.85)};
	Point2f src[] = {Point2f(width*0.20,height*0.00), Point2f(width*0.80,height*0.00), Point2f(width*0.80,height*1.00), Point2f(width*0.20,height*1.00)};
	Mat m = getPerspectiveTransform(src, dst);
	warpPerspective(img, img, m, Size(width, height));
}

int polynomial(double *params, int degree, double x)
{
	double val = 0;
	for (int i = 0; i < degree; i++)
	{
		val += params[i] * pow(x, i);
	}
	return (int)val;
}	

Lane getLanes(Mat &img, Config &config)
{
	birdseye(img);
	Mat img_thresh = img.clone();
	thresh(img_thresh);
	int width = img_thresh.cols;
	int height = img_thresh.rows;
	
	int d = config.lane_start_threshold;
	int left = width * config.left_lane_start / 100;
	int right = width * config.right_lane_start / 100;
	
	Lane lane;
	
	//Loop through frame rows
	int row_step = config.row_step;
	int col_step = config.col_step;
	for (int i = height-1; i >= 0; i-=row_step)
	{
		lane.left.push_back(Point(left, i));
		for (int j = left + d; j >= left - d; j-=col_step)
		{
			if (img_thresh.at<uchar>(i, j) == 255) 
			{
				lane.left.at<Point>(lane.left.rows - 1).x = j;
				left = j;
				break;
			}
		}
		
		lane.right.push_back(Point(right, i));
		for (int j = right - d; j < right + d; j+=col_step)
		{
			if (img_thresh.at<uchar>(i, j) == 255) 
			{
				lane.right.at<Point>(lane.right.rows - 1).x = j;
				right = j;
				break;
			}
		}
		
	}
	
	
	Lane polyLane;
	int degree = 3;
	double x[lane.left.rows];
	double y[lane.left.rows];
	
	//left
	for (int i = 0; i < lane.left.rows; i++)
	{
		x[i] = (double)lane.left.at<Point>(i).y;
		y[i] = (double)lane.left.at<Point>(i).x;
	}

	double params[degree];
	polynomialfit(lane.left.rows, degree, x, y, params);
	
	for (int i = 0; i < height; i++)
	{
		polyLane.left.push_back(Point(polynomial(params,degree,i), i));
	}
	
	//right
	for (int i = 0; i < lane.right.rows; i++)
	{
		x[i] = (double)lane.right.at<Point>(i).y;
		y[i] = (double)lane.right.at<Point>(i).x;
	}

	polynomialfit(lane.right.rows, degree, x, y, params);
	
	for (int i = 0; i < height; i++)
	{
		polyLane.right.push_back(Point(polynomial(params,degree,i), i));
	}
	
		
	//cout << lefts.rows << endl;
	for (int i = 0; i < height; i++)
	{
		circle(img, polyLane.left.at<Point>(i), 3, Scalar(150, 0, 0), 3);
		circle(img, polyLane.right.at<Point>(i), 3, Scalar(150, 0, 0), 3); 
	}
	
	first_person(img);
	return polyLane;
}

int main(int argc, char* argv[])
{
	Config config = getConfig();
		
	VideoCapture cap(config.video_file); 
    if(!cap.isOpened()) return -1;
    
    namedWindow("output", 1);
    Mat frame;
    while(true)
    {
		cap >> frame;
		//-------------------------------------------------------//
		Lane lane = getLanes(frame, config);
		
		
		//-------------------------------------------------------//
		imshow("output", frame);
		if(waitKey(1) >= 0) break;
	}
}
