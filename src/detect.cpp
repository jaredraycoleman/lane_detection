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
#include <algorithm>
#include <thread>
#include <mutex>

#include "opencv2/opencv.hpp"

#include "lane.h"
#include "uartcommander.h"
#include "detector.h"
#include "logger.h"
#include "helpers.h"

using namespace cv;

void sendMessage(SerialCommunication *serial, int8_t angle)
{
    UARTCommand command1, command2;
    command1.speed = 10;
    command1.maxTime = 300;
    command1.distance = 0;
    command1.dir = 1;
    command1.orientation = angle;
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

void process_frames(VideoCapture *cap, Mat *current_frame, std::mutex *mtx) 
{
    Mat frame = *current_frame;
    while(true)
    {
        try
        {
            mtx->lock();
            (*cap) >> frame;
            mtx->unlock();
            waitKey(1);
        }
        catch(cv::Exception e)
        {
            cout << e.what() << endl;
            break;
        }
    }
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

    VideoCapture cap;


    string serial_port;
    int skip_frames;
    int serial_baud;
    SerialCommunication *serial = nullptr;
    try
    {
        libconfig::Config cfg;
        cfg.readFile(config_path.c_str());

        if (cfg.exists("video.index")) {
            int index = cfg.lookup("video.index");
            cap = VideoCapture(index);
        } else {
            std::string path(cfg.lookup("video.file").c_str());
            path = abs_path(path, get_dir(config_path));
            cap = VideoCapture(path);
        }

        skip_frames = cfg.lookup("video.skip_frames");

        if (cfg.exists("serial.port") && cfg.exists("serial.baud")) 
        {
            serial_port = cfg.lookup("serial.port").c_str();
            serial_baud = cfg.lookup("serial.baud");
            serial = new SerialCommunication(serial_port, serial_baud);
        }
    }
    catch(const std::exception &exc)
    {
        cerr << "Invalid config file" << endl;
        cerr << exc.what() << endl;
        return 0;
    }


    Mat frame;
    cap >> frame;

    if (serial != nullptr) 
    {
        serial->run();
    }
    Lane lane(config_path);
    Detector detector(config_path, frame.rows, frame.cols);

    if(!cap.isOpened()) return -1;

    //namedWindow("original", 1);
    //namedWindow("output", 1);
    //namedWindow("birds", 1);
    int i = 0;

    Mat current_frame = frame;
    std::mutex mtx;
    std::thread cap_thread(process_frames, &cap, &current_frame, &mtx);

    while(true)
    {
        try
        {
            //get frame from stream
            mtx.lock();
            frame = current_frame.clone();
            mtx.unlock();

            //imshow("original", frame);
            detector.getLanes(frame, lane);
            //draw lanes
            detector.drawLane(frame, lane);

            //sends message
            double radius = detector.getTurningRadius(lane);

            if (serial != nullptr) 
            {
                if (radius > 0) 
                {
                    sendMessage(serial, 15);
                } 
                else 
                {
                    sendMessage(serial, -15);
                }
            }

            //show image
            //imshow("output", frame);
            waitKey(1);
        }
        catch(cv::Exception e)
        {
            cout << e.what() << endl;
            break;
        }
    }
}
