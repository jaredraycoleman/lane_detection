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

#include "opencv2/opencv.hpp"

#include "lane.h"
#include "uartcommander.h"
#include "detector.h"
#include "logger.h"
#include "helpers.h"

using namespace cv;

/**
 * Send a message to the serial 
 * @param serial SerialCommunication object
 * @param angle Angle (in degrees) to rotate
 */
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

/**
 * Converts a vector to a string
 * @param vec vector of integers
 */
std::string vec_to_string(std::vector<int> vec)
{
    std::ostringstream oss;
    oss << "[";
    for (auto i = vec.begin(); i != vec.end(); ++i)
        oss << *i << ' ';
    oss << "]";
    return oss.str();
}

/**
 * Gets the magnitude of a vector of integers
 * @param vec vector of integers
 */
double vec_magnitude(std::vector<int> vec)
{
    double sum = 0.0;
    for (auto i = vec.begin(); i != vec.end(); ++i)
    {
        sum += *i;
    }
    return sum / vec.size();
}

/**
 * Callback method execute on receiving
 * @param ldmap Local Dynamic Map
 */
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

    VideoCapture cap;


    string serial_port;
    int skip_frames;
    int serial_baud;
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

        serial_port = cfg.lookup("serial.port").c_str();
        serial_baud = cfg.lookup("serial.baud");
    }
    catch(...)
    {
        cerr << "Invalid config file" << endl;
        return 0;
    }


    Mat frame;
    cap >> frame;
    
    SerialCommunication serial(serial_port, serial_baud);

    serial.run();
    Lane lane(config_path);
    Detector detector(config_path, frame.rows, frame.cols);

    if(!cap.isOpened()) return -1;

    namedWindow("output", 1);
    while(true)
    {
        try
        {
            //get frame from stream
            cap >> frame;
            detector.getLanes(frame, lane);
            detector.drawLane(frame, lane);

            //sends message
            double radius = detector.getTurningRadius(lane);

            if (radius > 0) {
                sendMessage(&serial, 15);
            } else {
                sendMessage(&serial, -15);
            }

            //show image
            imshow("output", frame);
            waitKey(1);
        }
        catch(cv::Exception e)
        {
            cout << e.what() << endl;
            break;
        }
    }
}
