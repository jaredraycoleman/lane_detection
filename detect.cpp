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
#include <libconfig.h++>

#include "opencv2/opencv.hpp"
//#include "opencv2/gpu/gpu.hpp"
//#include "thrust/device_vector.h"
//#include "thrust/host_vector.h"

#include "polifitgsl.h"
#include "lane.h"
#include "uartcommander.h"
#include "detector.h"

using namespace cv;
using namespace libconfig;

Lane *lane;
//SerialCommunication *serial;
VideoCapture *cap;
Detector *detector;
double k;


/**
 * Reads configuration file and sets configuration parameters
 * @return Data structure of configuration parameters
 */
bool setup(string path)
{
    Config cfg;
    
    // Read the file. If there is an error, report it and exit.
    try
    {
        cfg.readFile(path.c_str());
        
        lane = new Lane(cfg.lookup("lane.n"), cfg.lookup("lane.filter"));
        //serial = new SerialCommunication(cfg.lookup("serial.port"), cfg.lookup("serial.baud"));
        string path = cfg.lookup("video.file");
        cap = new VideoCapture(path);
        detector = new Detector(cfg.lookup("detector.threshold"), cfg.lookup("detector.row_step"), 
                                cfg.lookup("detector.col_step"), cfg.lookup("detector.start.left"), 
                                cfg.lookup("detector.start.right"));
        k = cfg.lookup("controller.k");
    }
    catch(const SettingNotFoundException &nfex)
    {
        cerr << "Invalid config file" << endl;
        return false;
    }
    catch(const FileIOException &fioex)
    {
        std::cerr << "I/O error while reading file." << std::endl;
        return false;
    }
    catch(const ParseException &pex)
    {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
              << " - " << pex.getError() << std::endl;
        return false;
    }
    
    cout << "Config file parsed successfully" << endl;
    return true;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " <config file>" << endl;
        return 0;
    }
    
    if (!setup(argv[1]))
    {
        cout << "invalid config file!" << endl;
        return 0;
    }
    
    if(!cap->isOpened()) return -1;
    
    //namedWindow("output", 1);
    Mat frame;
    *cap >> frame;
    Mat birdseye = detector->getTransformMatrix(frame);
    Mat first_person = detector->getTransformMatrix(frame, true);
    
    cv::Mat src;
    cv::Mat th;
    cv::Mat dst;
    
    bool cont = true;
    while(cont)
    {
        try
        {
            //get frame from stream
            *cap >> frame;
            
            //copy of original frame for drawing
            Mat original = frame.clone();
            
            //preprocess img
            detector->thresh(frame, th);
            cv::warpPerspective(th, dst, birdseye, Size(frame.cols, frame.rows));
        
            //Get lanes
            detector->getLanes(dst, *lane);
            
            //draw lanes
            detector->drawLane(original, *lane, first_person);
            
            //show image//
            imshow("output", original);
            if(waitKey(1) >= 0) break;
        }catch(cv::Exception e){
            cout << e.what() << endl; 
            cont = false;
        }
    }
}
