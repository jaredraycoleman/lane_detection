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
        vehicle_width = cfg.lookup("vehicle.width")
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


// TODO: Put polynomial and derivative in helper file
// TODO: Put steering angle in vehicle class

/**
 * Evaluates a polynomial 
 * @param params vector of polynomial's coefficients, from lowest degree to highest
 * @param x value at which to evaluate polynomial at
 */
double polynomial(std::vector<double> params, double x)
{
    double val = 0;
    for (int i = 0; i < params.size(); i++)
    {
        val += params[i] * pow(x, i);
    }
    return val;
}

/**
 * Evaluates the derivative of a second degree polynomial 
 * @param params vector of polynomial's coefficients, from lowest degree to highest
 * @return vector of derivative polynomial coefficients
 */
double derivative(std::vector<double> params)
{
    assert(params.size() == 3);
    vector<double> deriv(params.size()-1);
    for (int i = 1; i < params.size(); i++)
    {
        deriv.push_back(params[i] * (double)i);
    }

    return deriv;
}

/**
 * Calculates the turning radius of the vehicle
 * @return Turning radius of vehcile
 */
double Lane::getTurningRadius()
{
    
    double y_pos = (double)camera_height;
    double x_pos = polynomial(params, y_pos);
    double m_pos = PI/2 + 0.001;
    double b_pos = m_pos*x_pos - y_pos;

    double y_des = camera_height-100;
    double x_des = polynomial(params, y_des);
    double m_des = polynomial(derivative(params), y_des);
    double b_des = m_des*x_des - y_des;

    double radius = 0;
    if (m_pos != m_des)
    {
        double x_center = (b_pos - b_des) / (m_des - m_pos);
        double y_center = m_pos * x_center + b_pos;

        double radius = sqrt(pow(x_pos-x_center, 2) + pow(y_pos-y_center, 2));
    }
    return radius;
}

/**
 * Calculate the ackermann steering angle for vehcile
 * @returns two-element vector (left and right wheel) of steering angles
 */
vector<double> Lane::AckermannSteering()
{
    double radius = this->getTurningRadius();

    vector<double> steering_angle{0, 0};
    if (radius != 0)
    {
        steering_angle[0] = atan2(vehicle_length, radius + (vehicle_width/2));
        steering_angle[1] = atan2(vehicle_length, radius + (vehicle_width/2));
    }

    return steering_angle;
}

/**
 * Calculate the differential steering velocities for vehcile
 * @returns two-element vector (left and right wheel) of steering velocities 
 */
vector<double> Lane::DifferentialSteering(double speed)
{
    double radius = this->getTurningRadius();

    vector<double> differential{0, 0};
    if (radius != 0)
    {
        double temp = vehicle_width / (2 * radius);
        differential.at(0) = speed * (1 + temp);
        differential.at(1) = speed * (1 - temp);
    }

    return steering_angle;
}
