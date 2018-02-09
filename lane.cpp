#include "lane.h"

/**
 * Only provided constructor for Lane.
 * @param degree Degree of polynomial that defines lanes.
 * @param filter Filter for curve to remove jitter.
 */ 
Lane::Lane(int n, double filter = 0.9)
    : n(n)
    , params(std::vector<double>(n, (double)nan("1")))
    , filter(filter)
{ 
    if (this->filter < 0.0 || this->filter > 1.0) this->filter = 0.9;
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
