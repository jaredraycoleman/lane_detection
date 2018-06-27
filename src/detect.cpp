/**
 * Detect.cpp
 * Provides functions for lane detection
 * 
 * @author Oscar Morales Ponce
 * @author Jared Coleman
 * @version 1.0 12/02/17
 */

using namespace std;

#include <string>
#include <libconfig.h++>

#include "opencv2/opencv.hpp"

#include "lane.h"
#include "uartcommander.h"
#include "detector.h"

using namespace cv;
   
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
    
    string config_path(argv[1]);
    
    string video_path;
    string serial_port;
    int serial_baud;
    double k;
    try
    {
        libconfig::Config cfg;
        cfg.readFile(config_path.c_str());

        video_path = cfg.lookup("video.file").c_str();
        //serial_port = cfg.lookup("serial.port").c_str();
        //serial_baud = cfg.lookup("serial.baud");
        k = cfg.lookup("controller.k");
    }
    catch(...)
    {
        cerr << "Invalid config file" << endl;
        return 0;
    }
    
    VideoCapture cap(video_path);
    //SerialCommunication serial(serial_port, serial_baud);
    Lane lane(config_path);
    Detector detector(config_path);
    
    if(!cap.isOpened()) return -1;
    
    namedWindow("output", 1);
    Mat frame;
    cap >> frame;
    
    while(true)
    {
        try
        {
            //get frame from stream
            cap >> frame;
            detector.getLanes(frame, lane);
            
            //draw lanes
            detector.drawLane(frame, lane);
            
            //sends message
            double speed = 1.0; //Get speed in same units as desired output speed for differential steering
            std::vector<double> ackermann = lane.AckermannSteering();
            std::vector<double> differential = lane.DifferentialSteering(speed);
            //sendMessage(&serial, lane.getSteeringAngle());
            
            //show image
            imshow("output", frame);
            if(waitKey(1) >= 0) break;
        }
        catch(cv::Exception e)
        {
            cout << e.what() << endl; 
            break;
        }
    }
}
