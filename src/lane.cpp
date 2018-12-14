#include "lane.h"

#include <libconfig.h++>
#include <iostream>
#include <sstream>

#include <assert.h>

/**
 * Only provided constructor for Lane.
 * @param config_path path to config file
 */ 
Lane::Lane(std::string config_path, std::vector<double> lparams, std::vector<double> rparams)
{ 
    libconfig::Config cfg;
    try
    {
        uint n = lparams.size();
        assert(n == rparams.size());
        cfg.readFile(config_path.c_str());
        filter = cfg.lookup("lane.filter");
        for (uint i = 0; i < lparams.size(); i++)
        {
            this->lparams.push_back(lparams[i]);
            this->rparams.push_back(rparams[i]);
            params.push_back((lparams[i] + rparams[i]) / 2);
        }
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
        for (uint i = 0; i < params.size(); i++)
        {
            this->lparams[i] = l[i];
            this->rparams[i] = r[i];
            this->params[i] = (l[i] + r[i]) / 2;
        }
    }
    else
    {
        for (uint i = 0; i < params.size(); i++)
        {
            double temp = (l[i] + r[i]) / 2;
            
            this->lparams[i] = filter * lparams[i] + (1 - filter) * l[i];
            this->rparams[i] = filter * rparams[i] + (1 - filter) * r[i];
            this->params[i] = filter * params[i] + (1 - filter) * temp;
        }
    }
}

int Lane::getDegree() { return this->params.size(); }
std::vector<double> Lane::getParams() const { return this->params; }
std::vector<double> Lane::getLParams() const { return this->lparams; }
std::vector<double> Lane::getRParams() const { return this->rparams; }

double Lane::getFilter() { return this->filter; }
double Lane::getCurvature() { return this->params[2]; }
double Lane::getWidth() { return this->width; }

