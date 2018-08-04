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
#include <string.h>
#include <libconfig.h++>

#include "opencv2/opencv.hpp"

#include "lane.h"
#include "uartcommander.h"
#include "detector.h"
#include "logger.h"

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

vector<int> position {0, 0, 0};
vector<int> speed {0, 0, 0};

std::string vec_to_string(std::vector<int> vec)
{
    std::ostringstream oss;
    oss << "[";
    for (auto i = vec.begin(); i != vec.end(); ++i)
        oss << *i << ' ';
    oss << "]";
    return oss.str();
}

double vec_magnitude(std::vector<int> vec)
{
    double sum = 0.0;
    for (auto i = vec.begin(); i != vec.end(); ++i)
    {
        sum += *i;
    }
    return sum / vec.size();
}

void receive(LDMap ldmap)
{
  position[0] = ldmap.position_x;
  position[1] = ldmap.position_y;
  position[2] = ldmap.position_z;
  speed[0] = ldmap.speed_x;
  speed[1] = ldmap.speed_y;
  speed[2] = ldmap.speed_z;

  std::cout << "Position: " << vec_to_string(position) << std::endl;
  std::cout << "Speed: " << vec_to_string(speed) << std::endl;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " <config file>" << endl;
        return 0;
    }

    Logger::init(false, "test.log", Levels::DEBUG);
    auto logger = Logger::getLogger();

    string config_path(argv[1]);

    string video_path;
    string serial_port;
    int skip_frames;
    int serial_baud;
    try
    {
        libconfig::Config cfg;
        cfg.readFile(config_path.c_str());

        video_path = cfg.lookup("video.file").c_str();
        skip_frames = cfg.lookup("video.skip_frames");
        serial_port = cfg.lookup("serial.port").c_str();
        serial_baud = cfg.lookup("serial.baud");
    }
    catch(...)
    {
        cerr << "Invalid config file" << endl;
        return 0;
    }

    VideoCapture cap(video_path);
//    SerialCommunication serial(serial_port, serial_baud);

//    serial.run(&receive);
    Lane lane(config_path);
    Detector detector(config_path);

    if(!cap.isOpened()) return -1;

    //namedWindow("original", 1);
    namedWindow("output", 1);
    namedWindow("birds", 1);
    Mat frame;
    cap >> frame;
    int i = 0;

    while(true)
    {
        try
        {
            //get frame from stream
            cap >> frame;
            if (skip_frames != 0 && i++ % skip_frames != 0)
            {
                continue;
            }
            //imshow("original", frame);
            detector.getLanes(frame, lane);
            //draw lanes
            detector.drawLane(frame, lane);

            //sends message
            double radius = detector.getTurningRadius(lane);

  //          sendMessage(&serial, (uint8_t)(radius*100));

            //show image
            imshow("output", frame);
            waitKey(1)
        }
        catch(cv::Exception e)
        {
            cout << e.what() << endl;
            break;
        }
    }
}
