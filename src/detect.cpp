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
using namespace std::literals::chrono_literals;

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

    if (show_output)
    {
        cv::namedWindow("output");
    }
    
    Detector detector(config_path, get_frame);

    PID pid(TIMEOUT / 1000.0, 10.0, 0.0, Kp, Kd, Ki);
    detector.start(1.0/(TIMEOUT / 1000.0), [&detector, serial, &pid, show_output] (const Lane &lane) {
        if (show_output)
        {
            cv::imshow("output", detector.getOffset());
            cv::waitKey(1);
        }

        double angle = pid.calculate(0.0, detector.getOffset());

        if (serial != nullptr)
        {
            UARTCommand command { 
                .maxTime = TIMEOUT,
                .speed = 10, 
                .orientation = (int16_t)angle, 
                .distance = 60,
                .dir = 1
            };
            serial->sendCommand(command);
        }
    });
    detector.join();
}
