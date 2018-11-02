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
#include <functional>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <chrono>

#include "opencv2/opencv.hpp"

#include "lane.h"
#include "uartcommander.h"
#include "detector.h"
#include "helpers.h"
#include "pid.h"

#define TIMEOUT 500
using namespace cv;

using time_stamp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>;

/**
 * Send a message to the serial 
 * @param serial SerialCommunication object
 * @param angle Angle (in degrees) to rotate
 */
void sendMessage(SerialCommunication *serial, int8_t angle, int16_t distance)
{
    UARTCommand command1, command2;
    command1.speed = 10;
    command1.maxTime = TIMEOUT;
    command1.distance = distance;
    command1.dir = 1;
    command1.orientation = angle;
    serial->sendCommand(&command1);
}

void sleep() {
    static auto t = std::chrono::high_resolution_clock::now();
    static auto end = t + std::chrono::milliseconds(TIMEOUT);
    
    std::this_thread::sleep_until(end);
    end = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(TIMEOUT);
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

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " <config file>" << endl;
        return 0;
    }
    string config_path(argv[1]);


    std::function<Mat()> get_frame;
    string serial_port;
    int serial_baud;
    SerialCommunication *serial = nullptr;
    bool show_output = false;

    double Kp;
    double Ki;
    double Kd;
    try
    {
        libconfig::Config cfg;
        cfg.readFile(config_path.c_str());

        if (cfg.exists("detector.pid_gains"))
        {
            Kp = cfg.lookup("detector.pid_gains.kP"); 
            Ki = cfg.lookup("detector.pid_gains.kI");
            Kd = cfg.lookup("detector.pid_gains.kD");
        }

        if (cfg.exists("video.index")) 
        {
            int index = cfg.lookup("video.index");
            get_frame = std::function<Mat()>([index](){
                            VideoCapture cap(index);
                            Mat frame;
                            cap >> frame;
                            cap.release();
                            return frame;
                        });
        } 
        else 
        {
            std::string path(cfg.lookup("video.file").c_str());
            path = abs_path(path, get_dir(config_path));
            get_frame = std::function<Mat()>([path](){
                            static VideoCapture cap(path);
                            Mat frame;
                            cap >> frame;
                            return frame;
                      });
        }

        if (cfg.exists("video.show")) 
        {
            show_output = cfg.lookup("video.show");
        }

        if (cfg.exists("serial.port") && cfg.exists("serial.baud")) 
        {
            serial_port = cfg.lookup("serial.port").c_str();
            serial_baud = cfg.lookup("serial.baud");
            serial = new SerialCommunication(serial_port, serial_baud);
            serial->register_callback([](const LDMap& ldmap){
                //std::cout << "orientation: " << ldmap.orientation << std::endl;
            });
        }
    }
    catch(const std::exception &exc)
    {
        cerr << "Invalid config file" << endl;
        cerr << exc.what() << endl;
        return 0;
    }

    Mat frame = get_frame();

    if (serial != nullptr) 
    {
        serial->run();
    }
    Lane lane(config_path);
    Detector detector(config_path, frame.rows, frame.cols);

    if (show_output) 
    {
        namedWindow("output", 1);
    }

    PID pid(TIMEOUT, 10, 0, Kp, Kd, Ki);
    while(true)
    {
        try
        {
            //get frame from stream
            frame = get_frame();
            detector.getLanes(frame, lane);
            
            if (show_output) 
            {
                //draw lanes
                detector.drawLane(frame, lane);
                imshow("output", frame);
                waitKey(1);
            }

            //sends message
            // std::vector<double> configuration = detector.getDesiredConfiguration(lane);
            
            // double angle = (180 * configuration[0] / M_PI) / 2;
            // double distance = configuration[1] * 100;

            // std::cout << "\nangle: " << angle;
            // std::cout << "\ndistance: " << distance << std::endl;

            double angle = pid.calculate(0, detector.getOffset(lane));

            std::cout << "angle: " << angle << std::endl;

            sleep();
            if (serial != nullptr) 
            {
                sendMessage(serial, angle, 60);
            }
        }
        catch(cv::Exception e)
        {
            std::cout << "Exception: " << e.what() << std::endl;
            break;
        }
    }
}
