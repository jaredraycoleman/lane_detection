#include "lane.h"
#include "logger.h"

#include <libconfig.h++>
#include <iostream>
#include <sstream>

#include <assert.h>

/**
 * Only provided constructor for Lane.
 * @param config_path path to config file
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
        lparams = std::vector<double>(n, (double)nan("1"));
        rparams = std::vector<double>(n, (double)nan("1"));
        vehicle_length = cfg.lookup("vehicle.length");
        vehicle_width = cfg.lookup("vehicle.width");
    }
    catch(...)
    {
        std::cerr << "Invalid config file" << std::endl;
    }
    if (filter < 0.0 || filter > 1.0) filter = 0.9;
}

/**
 * Updates left and right lane polynomial coefficients.
 * @param l Array of size degree. Defines coefficients for left lane curve.
 * @param r Array of size degree. Defines coefficients for right lane curve.
 */
void Lane::update(std::vector<double> l, std::vector<double> r)
{
    if (params[0] != params[0])
    {
        for (int i = 0; i < n; i++)
        {
            this->lparams[i] = l[i];
            this->rparams[i] = r[i];
            this->params[i] = (l[i] + r[i]) / 2;
        }
    }
    else
    {
        for (int i = 0; i < n; i++)
        {
            double temp = (l[i] + r[i]) / 2;
            
            this->lparams[i] = filter * lparams[i] + (1 - filter) * l[i];
            this->rparams[i] = filter * rparams[i] + (1 - filter) * r[i];
            this->params[i] = filter * params[i] + (1 - filter) * temp;
        }
    }
}

int Lane::getN() { return this->n; }
std::vector<double> Lane::getParams() const { return this->params; }
std::vector<double> Lane::getLParams() const { return this->lparams; }
std::vector<double> Lane::getRParams() const { return this->rparams; }

double Lane::getFilter() { return this->filter; }
double Lane::getCurvature() { return this->params[2]; }
double Lane::getWidth() { return this->width; }

