#include "lane.h"

#include <libconfig.h++>
#include <iostream>

#define PI 3.141592653589793238462643383279502884


/**
 * Only provided constructor for Lane.
 * @param degree Degree of polynomial that defines lanes.
 * @param filter Filter for curve to remove jitter.
 */ 
Lane::Lane(std::string config_path)
{ 
    libconfig::Config cfg;
    try
    {
        cfg.readFile(config_path.c_str());
        n = cfg.lookup("lane.n");
        filter = cfg.lookup("lane.filter");
        params = std::vector<double>(n, (double)nan("1"));
        camera_height = cfg.lookup("camera.height");
        vehicle_length = cfg.lookup("vehicle.length");
    }
    catch(...)
    {
        std::cerr << "Invalid config file" << std::endl;
    }
    if (filter < 0.0 || filter > 1.0) filter = 0.9;
}

/**
 * Updates left and right lane polynomial coefficients.
 * @param l_new Array of size degree. Defines coefficients for left lane curve.
 * @param r_new Array of size degree. Defines coefficients for right lane curve.
 */
void Lane::update(std::vector<double> l, std::vector<double> r)
{
    if (params[0] != params[0])
    {
        for (int i = 0; i < n; i++)
        {
            this->params[i] = (l[i] + r[i]) / 2;
        }
    }
    else
    {
        for (int i = 0; i < n; i++)
        {
            double temp = (l[i] + r[i]) / 2;
            this->params[i] = filter * params[i] + (1 - filter) * temp;
        }
    }
}

int Lane::getN() { return this->n; }
std::vector<double> Lane::getParams() { return this->params; }
double Lane::getFilter() { return this->filter; }
double Lane::getCurvature() { return this->params[2]; }
double Lane::getWidth() { return this->width; }


double polynomial(std::vector<double> params, double x)
{
    double val = 0;
    for (int i = 0; i < params.size(); i++)
    {
        val += params[i] * pow(x, i);
    }
    return val;
}

double derivative(std::vector<double> params, double x)
{
    return params[1] + params[2]*x*2;
}

double Lane::getSteeringAngle()
{
    double y_pos = (double)camera_height;
    double x_pos = polynomial(params, y_pos);
    double m_pos = PI/2 + 0.001;
    double b_pos = m_pos*x_pos - y_pos;

    double y_des = camera_height-100;
    double x_des = polynomial(params, y_des);
    double m_des = derivative(params, y_des);
    double b_des = m_des*x_des - y_des;

    double steering_angle = 0;
    if (m_pos != m_des)
    {
        double x_center = (b_pos - b_des) / (m_des - m_pos);
        double y_center = m_pos * x_center + b_pos;

        double radius = sqrt(pow(x_pos-x_center, 2) + pow(y_pos-y_center, 2));
        //std::cout << "radius: " << radius << std::endl;

        steering_angle = atan2(vehicle_length, radius);
    }

    //std::cout << "steering_angle: " << steering_angle * 180 / PI << std::endl;

    return steering_angle;
}
