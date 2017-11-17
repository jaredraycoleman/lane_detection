using namespace std;

#include "opencv2/opencv.hpp"
#include "opencv2/core/cuda.hpp"

using namespace cv;

int main (int argc, char* argv[])
{ 
	VideoCapture cap("video.mp4"); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    namedWindow("output",1);
	while(true)
	{
		try
		{
			cv::Mat src_host = cv::imread("file.png", CV_LOAD_IMAGE_GRAYSCALE);
			cv::cuda::GpuMat dst, src;
			src.upload(src_host);

			cv::cuda::threshold(src, dst, 128.0, 255.0, CV_THRESH_BINARY);

			cv::Mat result_host;
			dst.download(result_host);

			cv::imshow("output", result_host);
			if(waitKey(30) >= 0) break;
		}
		catch(const cv::Exception& ex)
		{
			std::cout << "Error: " << ex.what() << std::endl;
		}
	}
	
	return 0;
}
